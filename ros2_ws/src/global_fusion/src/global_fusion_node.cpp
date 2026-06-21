/*
 * GlobalFusionNode — ROS 2 версия адаптивного EKF sensor fusion.
 *
 * Входы:
 *   /imu                          (sensor_msgs/Imu)       — IMU, predict step
 *   /fmu/out/vehicle_gps_position (px4_msgs/SensorGps)    — GPS от PX4
 *   /vio/odometry                 (nav_msgs/Odometry)     — ORB-SLAM3 VIO
 *   /depth/altitude               (std_msgs/Float32)      — Depth Anything
 *
 * Выход:
 *   /global_odometry              (nav_msgs/Odometry)     — fused state → PX4 OFFBOARD
 *
 * ОТЛИЧИЕ ОТ ROS 1: ros::NodeHandle -> rclcpp::Node, ros::Subscriber ->
 * rclcpp::Subscription, ros::Publisher -> rclcpp::Publisher. Callback-и теперь
 * методы класса забинженные через std::bind, а не свободные функции с NodeHandle.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"

#include "global_fusion/ekf.hpp"

#include <Eigen/Dense>
#include <cmath>

using std::placeholders::_1;

class GlobalFusionNode : public rclcpp::Node {
public:
    GlobalFusionNode() : Node("global_fusion_node") {
        ekf_ = std::make_shared<global_fusion::EKF>();

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 50, std::bind(&GlobalFusionNode::imuCallback, this, _1));

        rclcpp::QoS px4_qos(10);
        px4_qos.best_effort();

        sub_gps_ = this->create_subscription<px4_msgs::msg::SensorGps>(
            "/fmu/out/vehicle_gps_position", px4_qos,
            std::bind(&GlobalFusionNode::gpsCallback, this, _1));

        sub_vio_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vio/odometry", 10, std::bind(&GlobalFusionNode::vioCallback, this, _1));

        sub_depth_ = this->create_subscription<std_msgs::msg::Float32>(
            "/depth/altitude", 10, std::bind(&GlobalFusionNode::depthCallback, this, _1));

        pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/global_odometry", 10);

        // Публикуем тот же fused estimate в PX4 через vehicle_visual_odometry,
        // чтобы наш LSTM-адаптивный EKF реально участвовал в управлении полётом,
        // не только логировался. PX4 требует BEST_EFFORT QoS на входящих топиках.
        rclcpp::QoS px4_in_qos(10);
        px4_in_qos.best_effort();
        pub_px4_odometry_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", px4_in_qos);

        // Подписка на timesync_status для конвертации ROS time -> PX4 internal time.
        // КРИТИЧНО: PX4 EKF2 ожидает timestamp в своей собственной clock domain
        // (микросекунды с момента старта PX4), а не Unix epoch ROS clock —
        // несовпадение вызывает катастрофические скачки позиции в EKF2.
        rclcpp::QoS timesync_qos(10);
        timesync_qos.best_effort();
        sub_timesync_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
            "/fmu/out/timesync_status", timesync_qos,
            [this](const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
                px4_time_offset_us_ = msg->estimated_offset;
                timesync_received_ = true;
            });

        // Подписки на LSTM-предсказанную variance (Этап 4) — заменяют
        // фиксированные R-константы динамическими значениями в реальном времени
        sub_gps_variance_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lstm_noise/gps_variance", 10,
            std::bind(&GlobalFusionNode::gpsVarianceCallback, this, _1));

        sub_vio_variance_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lstm_noise/vio_variance", 10,
            std::bind(&GlobalFusionNode::vioVarianceCallback, this, _1));

        sub_depth_variance_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lstm_noise/depth_variance", 10,
            std::bind(&GlobalFusionNode::depthVarianceCallback, this, _1));

        // Точка отсчёта GPS (lat/lon точки взлёта), задаётся первым принятым фиксом
        gps_origin_set_ = false;

        RCLCPP_INFO(this->get_logger(), "GlobalFusionNode started. Waiting for sensor data...");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time stamp(msg->header.stamp);
        double dt = 0.0;

        if (last_imu_time_.nanoseconds() > 0) {
            dt = (stamp - last_imu_time_).seconds();
        }
        last_imu_time_ = stamp;

        if (dt <= 0.0) {
            return;  // первый кадр — пропускаем, нет dt
        }

        Eigen::Vector3d accel(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        Eigen::Vector3d gyro(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        ekf_->predict(accel, gyro, dt);
        publishFusedOdometry(stamp);
    }

    void gpsCallback(const px4_msgs::msg::SensorGps::SharedPtr msg) {
        // ПРОСТОЕ приближение: используем GPS только если он уже даёт fix.
        // Точная конвертация lat/lon -> локальные NED координаты (с учётом точки
        // взлёта) будет уточнена позже; пока берём готовые относительные поля
        // если PX4 их предоставляет, иначе пропускаем обновление.
        if (msg->fix_type < 3) {
            return;  // нет валидного 3D fix
        }

        if (!gps_origin_set_) {
            origin_lat_ = msg->latitude_deg;
            origin_lon_ = msg->longitude_deg;
            origin_alt_ = msg->altitude_msl_m;
            gps_origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "GPS origin set: lat=%.6f lon=%.6f alt=%.2f",
                        origin_lat_, origin_lon_, origin_alt_);
            return;
        }

        // Грубая локальная аппроксимация lat/lon -> метры (плоская Земля,
        // достаточно точно на масштабе сотен метров вокруг точки взлёта)
        constexpr double EARTH_RADIUS = 6378137.0;
        double dlat = (msg->latitude_deg - origin_lat_) * M_PI / 180.0;
        double dlon = (msg->longitude_deg - origin_lon_) * M_PI / 180.0;
        double lat_rad = origin_lat_ * M_PI / 180.0;

        double north = dlat * EARTH_RADIUS;
        double east = dlon * EARTH_RADIUS * std::cos(lat_rad);
        double down = -(msg->altitude_msl_m - origin_alt_);

        Eigen::Vector3d position(north, east, down);
        Eigen::Vector3d velocity(msg->vel_n_m_s, msg->vel_e_m_s, msg->vel_d_m_s);

        ekf_->updateGPS(position, velocity);
    }

    void vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        Eigen::Vector3d vio_position(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        Eigen::Quaterniond vio_orientation(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        // VIO (ORB-SLAM3) живёт в собственной произвольной системе координат,
        // привязанной к точке инициализации трекинга, а не к world frame.
        // Выравниваем VIO-frame с world-frame один раз, при первом VIO сообщении
        // после того как EKF уже имеет разумную оценку позиции (например от GPS).
        if (!vio_aligned_) {
            // Ждём пока EKF получит хотя бы один GPS fix для разумной точки отсчёта
            if (!gps_origin_set_) {
                return;  // пропускаем VIO обновления пока нет GPS привязки
            }

            Eigen::Vector3d ekf_position = ekf_->getPosition();
            Eigen::Quaterniond ekf_orientation = ekf_->getOrientation();

            // Transform: world = vio_to_world_rotation * vio + vio_to_world_translation
            vio_to_world_rotation_ = ekf_orientation * vio_orientation.inverse();
            vio_to_world_translation_ = ekf_position - vio_to_world_rotation_ * vio_position;

            vio_aligned_ = true;
            RCLCPP_INFO(this->get_logger(),
                "VIO aligned to world frame at EKF position (%.2f, %.2f, %.2f)",
                ekf_position.x(), ekf_position.y(), ekf_position.z());
        }

        // Применяем выравнивающий transform к каждому VIO измерению
        Eigen::Vector3d position = vio_to_world_rotation_ * vio_position + vio_to_world_translation_;
        Eigen::Quaterniond orientation = vio_to_world_rotation_ * vio_orientation;

        ekf_->updateVIO(position, orientation);
    }

    void depthCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        ekf_->updateDepthAltitude(static_cast<double>(msg->data));
    }

    // Callbacks для LSTM noise estimator — обновляют R-матрицы EKF в реальном
    // времени вместо фиксированных констант (см. ekf.hpp setter методы)
    void gpsVarianceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        double var = static_cast<double>(msg->data);
        if (var > 0.0 && std::isfinite(var)) {
            ekf_->setGpsPositionVariance(var);
        }
    }

    void vioVarianceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        double var = static_cast<double>(msg->data);
        if (var > 0.0 && std::isfinite(var)) {
            ekf_->setVioPositionVariance(var);
        }
    }

    void depthVarianceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        double var = static_cast<double>(msg->data);
        if (var > 0.0 && std::isfinite(var)) {
            ekf_->setDepthVariance(var);
        }
    }

    void publishFusedOdometry(const rclcpp::Time& stamp) {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = stamp;
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        Eigen::Vector3d pos = ekf_->getPosition();
        Eigen::Vector3d vel = ekf_->getVelocity();
        Eigen::Quaterniond q = ekf_->getOrientation();

        msg.pose.pose.position.x = pos.x();
        msg.pose.pose.position.y = pos.y();
        msg.pose.pose.position.z = pos.z();

        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        msg.twist.twist.linear.x = vel.x();
        msg.twist.twist.linear.y = vel.y();
        msg.twist.twist.linear.z = vel.z();

        pub_odometry_->publish(msg);

        // Публикуем также в PX4 как vehicle_visual_odometry, чтобы наш
        // fused estimate реально использовался PX4 EKF2 для управления,
        // не только логировался в /global_odometry для анализа.
        // КРИТИЧНО: не публикуем пока не получен хотя бы один timesync_status —
        // иначе timestamp будет в неправильной clock domain (см. install_notes.md,
        // инцидент с дрейфом 866м из-за рассинхронизации ~20 дней).
        if (!timesync_received_) {
            return;
        }

        px4_msgs::msg::VehicleOdometry px4_msg;
        int64_t ros_time_us = static_cast<int64_t>(stamp.nanoseconds() / 1000);
        px4_msg.timestamp = static_cast<uint64_t>(ros_time_us + px4_time_offset_us_);
        px4_msg.timestamp_sample = px4_msg.timestamp;
        px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        px4_msg.position = {
            static_cast<float>(pos.x()),
            static_cast<float>(pos.y()),
            static_cast<float>(pos.z())
        };
        px4_msg.q = {
            static_cast<float>(q.w()),
            static_cast<float>(q.x()),
            static_cast<float>(q.y()),
            static_cast<float>(q.z())
        };
        px4_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        px4_msg.velocity = {
            static_cast<float>(vel.x()),
            static_cast<float>(vel.y()),
            static_cast<float>(vel.z())
        };
        px4_msg.position_variance = {1.0f, 1.0f, 1.0f};
        px4_msg.orientation_variance = {0.1f, 0.1f, 0.1f};
        px4_msg.velocity_variance = {0.5f, 0.5f, 0.5f};
        px4_msg.quality = 100;

        pub_px4_odometry_->publish(px4_msg);
    }

    std::shared_ptr<global_fusion::EKF> ekf_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sub_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_depth_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_px4_odometry_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr sub_timesync_;
    int64_t px4_time_offset_us_ = 0;
    bool timesync_received_ = false;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_gps_variance_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vio_variance_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_depth_variance_;

    rclcpp::Time last_imu_time_;

    bool gps_origin_set_;
    double origin_lat_, origin_lon_, origin_alt_;

    bool vio_aligned_ = false;
    Eigen::Quaterniond vio_to_world_rotation_;
    Eigen::Vector3d vio_to_world_translation_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalFusionNode>());
    rclcpp::shutdown();
    return 0;
}
