#include "global_fusion/ekf.hpp"
#include <cmath>

namespace global_fusion {

EKF::EKF() {
    accel_noise_density_ = 0.02;
    gyro_noise_density_ = 0.002;
    accel_bias_random_walk_ = 0.0005;
    gyro_bias_random_walk_ = 0.00005;

    gps_pos_var_ = 1.0;
    gps_vel_var_ = 0.25;
    vio_pos_var_ = 0.05;
    vio_orient_var_ = 0.01;
    depth_var_ = 0.5;

    reset();
}

void EKF::reset() {
    nominal_position_.setZero();
    nominal_velocity_.setZero();
    nominal_orientation_ = Eigen::Quaterniond::Identity();
    accel_bias_.setZero();
    gyro_bias_.setZero();

    P_.setIdentity();
    P_ *= 0.1;
}

void EKF::predict(const Eigen::Vector3d& accel_raw,
                   const Eigen::Vector3d& gyro_raw,
                   double dt) {
    if (dt <= 0.0 || dt > 1.0) {
        return;
    }

    Eigen::Vector3d accel = accel_raw - accel_bias_;
    Eigen::Vector3d gyro = gyro_raw - gyro_bias_;

    Eigen::Vector3d delta_theta = gyro * dt;
    double angle = delta_theta.norm();
    Eigen::Quaterniond dq;
    if (angle > 1e-8) {
        Eigen::Vector3d axis = delta_theta / angle;
        dq = Eigen::AngleAxisd(angle, axis);
    } else {
        dq = Eigen::Quaterniond::Identity();
    }
    nominal_orientation_ = (nominal_orientation_ * dq).normalized();

    Eigen::Vector3d gravity_world(0, 0, -GRAVITY);
    Eigen::Vector3d accel_world = nominal_orientation_ * accel + gravity_world;

    nominal_position_ += nominal_velocity_ * dt + 0.5 * accel_world * dt * dt;
    nominal_velocity_ += accel_world * dt;

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F;
    F.setIdentity();

    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    Eigen::Matrix3d accel_skew;
    accel_skew <<        0, -accel.z(),  accel.y(),
                  accel.z(),         0, -accel.x(),
                 -accel.y(),  accel.x(),         0;
    Eigen::Matrix3d R = nominal_orientation_.toRotationMatrix();
    F.block<3, 3>(3, 6) = -R * accel_skew * dt;

    F.block<3, 3>(3, 9) = -R * dt;

    F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;
    Q.setZero();
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * accel_noise_density_ * accel_noise_density_ * dt;
    Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * gyro_noise_density_ * gyro_noise_density_ * dt;
    Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * accel_bias_random_walk_ * accel_bias_random_walk_ * dt;
    Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * gyro_bias_random_walk_ * gyro_bias_random_walk_ * dt;

    P_ = F * P_ * F.transpose() + Q;
}

void EKF::injectErrorState(const Eigen::Matrix<double, STATE_DIM, 1>& delta_x) {
    nominal_position_ += delta_x.segment<3>(0);
    nominal_velocity_ += delta_x.segment<3>(3);

    Eigen::Vector3d delta_theta = delta_x.segment<3>(6);
    double angle = delta_theta.norm();
    Eigen::Quaterniond dq;
    if (angle > 1e-8) {
        dq = Eigen::AngleAxisd(angle, delta_theta / angle);
    } else {
        dq = Eigen::Quaterniond::Identity();
    }
    nominal_orientation_ = (nominal_orientation_ * dq).normalized();

    accel_bias_ += delta_x.segment<3>(9);
    gyro_bias_ += delta_x.segment<3>(12);
}

void EKF::updateGPS(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
    Eigen::Matrix<double, 6, STATE_DIM> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 1> innovation;
    innovation.segment<3>(0) = position - nominal_position_;
    innovation.segment<3>(3) = velocity - nominal_velocity_;

    Eigen::Matrix<double, 6, 6> R;
    R.setZero();
    R.diagonal() << gps_pos_var_, gps_pos_var_, gps_pos_var_,
                     gps_vel_var_, gps_vel_var_, gps_vel_var_;

    Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, 6> K = P_ * H.transpose() * S.inverse();

    Eigen::Matrix<double, STATE_DIM, 1> delta_x = K * innovation;
    injectErrorState(delta_x);

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> I;
    I.setIdentity();
    P_ = (I - K * H) * P_;
}

void EKF::updateVIO(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    Eigen::Matrix<double, 3, STATE_DIM> H_pos;
    H_pos.setZero();
    H_pos.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Vector3d innovation_pos = position - nominal_position_;

    Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * vio_pos_var_;
    Eigen::Matrix3d S_pos = H_pos * P_ * H_pos.transpose() + R_pos;
    Eigen::Matrix<double, STATE_DIM, 3> K_pos = P_ * H_pos.transpose() * S_pos.inverse();

    Eigen::Matrix<double, STATE_DIM, 1> delta_x_pos = K_pos * innovation_pos;
    injectErrorState(delta_x_pos);

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> I;
    I.setIdentity();
    P_ = (I - K_pos * H_pos) * P_;

    Eigen::Quaterniond q_error = orientation * nominal_orientation_.inverse();
    Eigen::AngleAxisd aa(q_error);
    Eigen::Vector3d innovation_orient = aa.axis() * aa.angle();

    Eigen::Matrix<double, 3, STATE_DIM> H_orient;
    H_orient.setZero();
    H_orient.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_orient = Eigen::Matrix3d::Identity() * vio_orient_var_;
    Eigen::Matrix3d S_orient = H_orient * P_ * H_orient.transpose() + R_orient;
    Eigen::Matrix<double, STATE_DIM, 3> K_orient = P_ * H_orient.transpose() * S_orient.inverse();

    Eigen::Matrix<double, STATE_DIM, 1> delta_x_orient = K_orient * innovation_orient;
    injectErrorState(delta_x_orient);

    P_ = (I - K_orient * H_orient) * P_;
}

void EKF::updateDepthAltitude(double altitude) {
    Eigen::Matrix<double, 1, STATE_DIM> H;
    H.setZero();
    H(0, 2) = -1.0;

    double innovation = altitude - (-nominal_position_.z());

    double R = depth_var_;
    double S = (H * P_ * H.transpose())(0, 0) + R;
    Eigen::Matrix<double, STATE_DIM, 1> K = P_ * H.transpose() / S;

    Eigen::Matrix<double, STATE_DIM, 1> delta_x = K * innovation;
    injectErrorState(delta_x);

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> I;
    I.setIdentity();
    P_ = (I - K * H) * P_;
}

}  // namespace global_fusion
