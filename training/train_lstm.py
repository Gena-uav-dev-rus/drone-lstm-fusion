#!/usr/bin/env python3
"""
Тренировочный скрипт для трёх LSTM noise estimator.

Вход: CSV датасет собранный lstm_data_collector
Выход: три .pt файла с весами моделей:
  - models/lstm_gps.pt
  - models/lstm_vio.pt
  - models/lstm_depth.pt

Запуск (из venv с CUDA):
  source ~/depth_anything_venv/bin/activate
  cd ~/drone-lstm-fusion/training
  python3 train_lstm.py --dataset /tmp/lstm_dataset.csv
"""

import argparse
import os
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from sklearn.preprocessing import StandardScaler
import pickle

DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
SEQ_LEN = 20       # история N шагов на входе LSTM
HIDDEN_SIZE = 64
NUM_LAYERS = 2
BATCH_SIZE = 64
EPOCHS = 50
LR = 1e-3

print(f"Using device: {DEVICE}")


# ─── Модель ────────────────────────────────────────────────────────────────────

class NoiseEstimatorLSTM(nn.Module):
    def __init__(self, input_size, hidden_size=HIDDEN_SIZE, num_layers=NUM_LAYERS):
        super().__init__()
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers,
                            batch_first=True, dropout=0.1)
        self.head = nn.Sequential(
            nn.Linear(hidden_size, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Softplus()   # variance всегда > 0
        )

    def forward(self, x):
        out, _ = self.lstm(x)
        return self.head(out[:, -1, :])   # берём последний шаг последовательности


# ─── Dataset ───────────────────────────────────────────────────────────────────

class SensorNoiseDataset(Dataset):
    def __init__(self, features, targets, seq_len=SEQ_LEN):
        self.features = torch.tensor(features, dtype=torch.float32)
        self.targets = torch.tensor(targets, dtype=torch.float32)
        self.seq_len = seq_len

    def __len__(self):
        return len(self.features) - self.seq_len

    def __getitem__(self, idx):
        x = self.features[idx:idx + self.seq_len]
        y = self.targets[idx + self.seq_len]
        return x, y


# ─── Подготовка данных ─────────────────────────────────────────────────────────

def prepare_data(df):
    """
    Вычисляем реальные ошибки (variance target) для каждого источника.
    GPS позиция в lat/lon, конвертируем в метры относительно первой точки.
    """
    # GPS: конвертация lat/lon -> метры (плоская аппроксимация)
    EARTH_R = 6378137.0
    lat0 = df['gps_lat'].iloc[0]
    lon0 = df['gps_lon'].iloc[0]
    lat_rad = np.deg2rad(lat0)

    df['gps_north'] = np.deg2rad(df['gps_lat'] - lat0) * EARTH_R
    df['gps_east'] = np.deg2rad(df['gps_lon'] - lon0) * EARTH_R * np.cos(lat_rad)
    df['gps_down'] = -(df['gps_alt'] - df['gps_alt'].iloc[0])

    # Ошибки = (измерение - ground truth)^2 усреднённые по осям
    gps_err = ((df['gps_north'] - df['gt_x'])**2 +
               (df['gps_east']  - df['gt_y'])**2 +
               (df['gps_down']  - df['gt_z'])**2) / 3.0

    vio_err = ((df['vio_x'] - df['gt_x'])**2 +
               (df['vio_y'] - df['gt_y'])**2 +
               (df['vio_z'] - df['gt_z'])**2) / 3.0

    depth_err = (df['depth_altitude'] - df['gt_z'].abs())**2

    # Входные фичи для каждой LSTM
    gps_feats = df[['gps_north', 'gps_east', 'gps_down',
                     'gps_vn', 'gps_ve', 'gps_vd',
                     'gps_fix_type', 'gps_satellites']].values

    vio_feats = df[['vio_x', 'vio_y', 'vio_z',
                    'vio_qx', 'vio_qy', 'vio_qz', 'vio_qw']].values

    depth_feats = df[['depth_altitude',
                       'gt_vx', 'gt_vy', 'gt_vz']].values  # скорость как context

    return {
        'gps':   (gps_feats,   gps_err.values),
        'vio':   (vio_feats,   vio_err.values),
        'depth': (depth_feats, depth_err.values),
    }


# ─── Обучение одной модели ─────────────────────────────────────────────────────

def train_one(name, features_raw, targets_raw, output_dir):
    print(f"\n{'='*50}")
    print(f"Training {name.upper()} LSTM...")

    # Нормализация фичей (не нормализуем targets — это уже variance в м^2)
    scaler = StandardScaler()
    features = scaler.fit_transform(features_raw)
    targets = targets_raw.reshape(-1, 1)

    # Сплит train/val (80/20)
    split = int(len(features) * 0.8)
    train_ds = SensorNoiseDataset(features[:split], targets[:split])
    val_ds   = SensorNoiseDataset(features[split:], targets[split:])

    train_dl = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
    val_dl   = DataLoader(val_ds,   batch_size=BATCH_SIZE, shuffle=False)

    model = NoiseEstimatorLSTM(input_size=features.shape[1]).to(DEVICE)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=5, factor=0.5)
    criterion = nn.MSELoss()

    best_val_loss = float('inf')
    best_state = None

    for epoch in range(EPOCHS):
        # Train
        model.train()
        train_loss = 0.0
        for x, y in train_dl:
            x, y = x.to(DEVICE), y.to(DEVICE)
            optimizer.zero_grad()
            pred = model(x)
            loss = criterion(pred, y)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            train_loss += loss.item()
        train_loss /= len(train_dl)

        # Val
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for x, y in val_dl:
                x, y = x.to(DEVICE), y.to(DEVICE)
                pred = model(x)
                val_loss += criterion(pred, y).item()
        val_loss /= len(val_dl)

        scheduler.step(val_loss)

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_state = {k: v.clone() for k, v in model.state_dict().items()}

        if (epoch + 1) % 10 == 0:
            print(f"  Epoch {epoch+1:3d}/{EPOCHS}  train={train_loss:.6f}  val={val_loss:.6f}")

    # Сохраняем лучшую модель + scaler
    os.makedirs(output_dir, exist_ok=True)
    model.load_state_dict(best_state)

    torch.save({
        'model_state': best_state,
        'input_size': features.shape[1],
        'hidden_size': HIDDEN_SIZE,
        'num_layers': NUM_LAYERS,
        'seq_len': SEQ_LEN,
        'val_loss': best_val_loss,
    }, os.path.join(output_dir, f'lstm_{name}.pt'))

    with open(os.path.join(output_dir, f'scaler_{name}.pkl'), 'wb') as f:
        pickle.dump(scaler, f)

    print(f"  Best val loss: {best_val_loss:.6f}")
    print(f"  Saved to {output_dir}/lstm_{name}.pt")
    return model, scaler


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=str, default='/tmp/lstm_dataset.csv')
    parser.add_argument('--output_dir', type=str,
                        default=os.path.expanduser('~/drone-lstm-fusion/training/models'))
    args = parser.parse_args()

    print(f"Loading dataset: {args.dataset}")
    df = pd.read_csv(args.dataset)
    print(f"Dataset shape: {df.shape}")
    print(f"Columns: {list(df.columns)}")

    data = prepare_data(df)

    for name, (feats, targets) in data.items():
        train_one(name, feats, targets, args.output_dir)

    print("\n✅ All three LSTM models trained successfully!")
    print(f"   Models saved to: {args.output_dir}")


if __name__ == '__main__':
    main()
