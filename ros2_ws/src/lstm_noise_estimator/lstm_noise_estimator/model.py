"""
Общее определение архитектуры LSTM noise estimator.
ДОЛЖНО ТОЧНО СОВПАДАТЬ с архитектурой в training/train_lstm.py,
иначе веса не загрузятся (mismatch state_dict).
"""
import torch
import torch.nn as nn


class NoiseEstimatorLSTM(nn.Module):
    def __init__(self, input_size, hidden_size=64, num_layers=2):
        super().__init__()
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers,
                            batch_first=True, dropout=0.1)
        self.head = nn.Sequential(
            nn.Linear(hidden_size, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Softplus()
        )

    def forward(self, x):
        out, _ = self.lstm(x)
        return self.head(out[:, -1, :])


def load_model(checkpoint_path, device='cpu'):
    """Загружает модель из .pt чекпоинта, сохранённого train_lstm.py"""
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model = NoiseEstimatorLSTM(
        input_size=checkpoint['input_size'],
        hidden_size=checkpoint['hidden_size'],
        num_layers=checkpoint['num_layers'],
    )
    model.load_state_dict(checkpoint['model_state'])
    model.to(device)
    model.eval()
    seq_len = checkpoint['seq_len']
    return model, seq_len
