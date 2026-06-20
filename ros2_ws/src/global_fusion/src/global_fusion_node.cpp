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

#include "global_fusion/ekf.hpp"

#include <Eigen/Dense>

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
    }

    std::shared_ptr<global_fusion::EKF> ekf_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sub_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_depth_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

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
