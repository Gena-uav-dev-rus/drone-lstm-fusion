#ifndef GLOBAL_FUSION_EKF_HPP
#define GLOBAL_FUSION_EKF_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace global_fusion {

// Error-state EKF, 15 states:
// [0:3]   position error   (x, y, z)        - world frame
// [3:6]   velocity error    (vx, vy, vz)     - world frame
// [6:9]   orientation error (roll, pitch, yaw) - small-angle, world frame
// [9:12]  accel bias error
// [12:15] gyro bias error
constexpr int STATE_DIM = 15;

class EKF {
public:
    EKF();

    // Сбрасывает фильтр в исходное состояние (нулевая позиция, единичный quaternion)
    void reset();

    // --- Predict step, вызывается на каждый IMU кадр ---
    // accel, gyro - сырые измерения IMU (м/с^2, рад/с), dt - время с прошлого вызова (сек)
    void predict(const Eigen::Vector3d& accel,
                 const Eigen::Vector3d& gyro,
                 double dt);

    // --- Update steps, каждый источник вызывается независимо (sequential update) ---

    // GPS даёт абсолютную позицию (NED, относительно точки взлёта) и скорость
    void updateGPS(const Eigen::Vector3d& position,
                   const Eigen::Vector3d& velocity);

    // VIO (ORB-SLAM3) даёт позицию и ориентацию относительно точки старта VIO
    void updateVIO(const Eigen::Vector3d& position,
                    const Eigen::Quaterniond& orientation);

    // Depth Anything даёт только высоту (z) относительно земли под дроном
    void updateDepthAltitude(double altitude);

    // --- Геттеры состояния (nominal state, после применения error-state коррекции) ---
    Eigen::Vector3d getPosition() const { return nominal_position_; }
    Eigen::Vector3d getVelocity() const { return nominal_velocity_; }
    Eigen::Quaterniond getOrientation() const { return nominal_orientation_; }

    // Устанавливает начальную ориентацию EKF (используется один раз при старте,
    // чтобы избежать "heading estimate not stable" из-за расхождения с PX4
    // internal magnetometer-based yaw estimate, см. install_notes.md)
    void setInitialOrientation(const Eigen::Quaterniond& q) { nominal_orientation_ = q; }

    // Постоянная лёгкая коррекция yaw от внешнего источника (PX4 internal
    // magnetometer-based attitude estimate) — компенсирует накопление gyro
    // bias drift по курсу, когда нет другого yaw-источника (VIO ещё не в
    // рабочем состоянии). weight задаёт долю внешнего yaw на каждый вызов
    // (например 0.05 = 5%), сохраняет roll/pitch от собственного EKF estimate.
    void blendYawCorrection(const Eigen::Quaterniond& external_q, double weight) {
        // Извлекаем yaw компонент из обоих quaternion через спроецированный
        // на плоскость XY вектор "вперёд" — это устойчивее чем напрямую
        // интерполировать quaternion, так как сохраняет roll/pitch нетронутыми.
        double yaw_external = std::atan2(
            2.0 * (external_q.w() * external_q.z() + external_q.x() * external_q.y()),
            1.0 - 2.0 * (external_q.y() * external_q.y() + external_q.z() * external_q.z()));
        double yaw_current = std::atan2(
            2.0 * (nominal_orientation_.w() * nominal_orientation_.z() +
                   nominal_orientation_.x() * nominal_orientation_.y()),
            1.0 - 2.0 * (nominal_orientation_.y() * nominal_orientation_.y() +
                         nominal_orientation_.z() * nominal_orientation_.z()));

        // Кратчайший угловой путь (избегаем скачка через +-180 град границу)
        double yaw_diff = yaw_external - yaw_current;
        while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

        double yaw_new = yaw_current + weight * yaw_diff;

        Eigen::Quaterniond yaw_rotation(Eigen::AngleAxisd(yaw_new - yaw_current, Eigen::Vector3d::UnitZ()));
        nominal_orientation_ = (yaw_rotation * nominal_orientation_).normalized();
    }
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> getCovariance() const { return P_; }

    // --- Настройка R-матриц (шум измерений) ---
    // ВАЖНО: сейчас фиксированные значения, на Этапе 4 будут перезаписываться
    // из LSTM noise estimator через callback в global_fusion_node.cpp
    void setGpsPositionVariance(double var) { gps_pos_var_ = var; }
    void setGpsVelocityVariance(double var) { gps_vel_var_ = var; }
    void setVioPositionVariance(double var) { vio_pos_var_ = var; }
    void setVioOrientationVariance(double var) { vio_orient_var_ = var; }
    void setDepthVariance(double var) { depth_var_ = var; }

private:
    // Nominal state (полные значения, error-state добавляется и сбрасывается в ноль)
    Eigen::Vector3d nominal_position_;
    Eigen::Vector3d nominal_velocity_;
    Eigen::Quaterniond nominal_orientation_;
    Eigen::Vector3d accel_bias_;
    Eigen::Vector3d gyro_bias_;

    // Error-state covariance
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_;

    // Process noise (Q matrix diagonal terms) — насколько доверяем модели движения
    double accel_noise_density_;
    double gyro_noise_density_;
    double accel_bias_random_walk_;
    double gyro_bias_random_walk_;

    // Measurement noise (R matrix terms) — фиксированные пока нет LSTM (Этап 4)
    double gps_pos_var_;
    double gps_vel_var_;
    double vio_pos_var_;
    double vio_orient_var_;
    double depth_var_;

    static constexpr double GRAVITY = 9.80665;

    // Применяет error-state коррекцию к nominal state и сбрасывает error-state в ноль
    void injectErrorState(const Eigen::Matrix<double, STATE_DIM, 1>& delta_x);
};

}  // namespace global_fusion

#endif  // GLOBAL_FUSION_EKF_HPP
