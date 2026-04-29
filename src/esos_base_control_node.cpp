/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file esos_base_control_node.cpp
 * @brief ROS2 node for ESOS base chassis control via RPMsg
 *
 * Subscribes to /cmd_vel and publishes odometry. Uses chassis.h interface
 * to communicate with ESOS small core via RPMsg.
 */

#include <chassis.h>

#include <cstdio>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// Default robot parameters
#ifndef WHEEL_DIAMETER
#define WHEEL_DIAMETER 0.067  // 67mm diameter
#endif
#ifndef WHEEL_BASE
#define WHEEL_BASE 0.28  // 280mm wheel base
#endif

class EsosBaseControlNode : public rclcpp::Node {
public:
        EsosBaseControlNode()
                : rclcpp::Node("esos_base_control_node"), chassis_(nullptr) {
    // Declare parameters
    declare_parameter("send_hz", 20.0);
    declare_parameter("odom_hz", 50.0);
    declare_parameter("cmd_vel_timeout", 0.4);
    declare_parameter("publish_tf", true);
    declare_parameter("odom_topic", "odom");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_footprint");

    // Robot parameters
    declare_parameter("wheel_diameter", WHEEL_DIAMETER);
    declare_parameter("wheel_base", WHEEL_BASE);
    declare_parameter("motor1_factor", 1.0);
    declare_parameter("motor2_factor", 1.0);
    declare_parameter("reduction_ratio", 56.0);
    declare_parameter("ff_factor", 0.3);
    declare_parameter("pid_kp", 0.05);
    declare_parameter("pid_ki", 0.2);
    declare_parameter("pid_kd", 0.01);
    declare_parameter("cfg_send_on_startup", true);
    declare_parameter("feedback_enable", false);

    // RPMSG configuration
    declare_parameter("rpmsg_ctrl_dev", "/dev/rpmsg_ctrl0");
    declare_parameter("rpmsg_data_dev", "/dev/rpmsg0");
    declare_parameter("rpmsg_service_name", "rpmsg:motor_ctrl");
    declare_parameter("rpmsg_local_addr", 1003);
    declare_parameter("rpmsg_remote_addr", 1002);

    // Get parameters
    send_hz_ = get_parameter("send_hz").as_double();
    odom_hz_ = get_parameter("odom_hz").as_double();
    cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    odom_topic_ = get_parameter("odom_topic").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    wheel_diameter_ = get_parameter("wheel_diameter").as_double();
    wheel_base_ = get_parameter("wheel_base").as_double();
    motor1_factor_ = get_parameter("motor1_factor").as_double();
    motor2_factor_ = get_parameter("motor2_factor").as_double();
    reduction_ratio_ = get_parameter("reduction_ratio").as_double();
    ff_factor_ = get_parameter("ff_factor").as_double();
    pid_kp_ = get_parameter("pid_kp").as_double();
    pid_ki_ = get_parameter("pid_ki").as_double();
    pid_kd_ = get_parameter("pid_kd").as_double();
    cfg_send_on_startup_ = get_parameter("cfg_send_on_startup").as_bool();
    feedback_enable_ = get_parameter("feedback_enable").as_bool();

    rpmsg_ctrl_dev_ = get_parameter("rpmsg_ctrl_dev").as_string();
    rpmsg_data_dev_ = get_parameter("rpmsg_data_dev").as_string();
    rpmsg_service_name_ = get_parameter("rpmsg_service_name").as_string();
    rpmsg_local_addr_ = get_parameter("rpmsg_local_addr").as_int();
    rpmsg_remote_addr_ = get_parameter("rpmsg_remote_addr").as_int();

    // Initialize chassis
    init_chassis();

    // Create publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    // Create TF broadcaster
    if (publish_tf_) {
                        tf_broadcaster_ =
                                std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Create subscriber
                        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
                                "cmd_vel", 10,
                                std::bind(&EsosBaseControlNode::cmd_vel_callback, this,
                                                    std::placeholders::_1));

    // Create timers
    auto odom_period = std::chrono::duration<double>(1.0 / odom_hz_);
                        odom_timer_ = create_wall_timer(
                                std::chrono::duration_cast<std::chrono::nanoseconds>(odom_period),
                                std::bind(&EsosBaseControlNode::odom_timer_callback, this));

    last_cmd_time_ = now();

                        RCLCPP_INFO(
                                get_logger(),
                                "esos_base_control_node started: wheel_diameter=%.4f, wheel_base=%.4f",
                                wheel_diameter_, wheel_base_);
        }

        ~EsosBaseControlNode() override {
                if (chassis_) {
                        chassis_brake(chassis_);
                        chassis_free(chassis_);
                        chassis_ = nullptr;
                }
        }

private:
    void init_chassis() {
        struct chassis_rpmsg_config cfg = {};
        cfg.base.type = CHASSIS_TYPE_DIFF_2WD;
        cfg.base.wheel_diameter = static_cast<float>(wheel_diameter_);
        cfg.base.wheel_base = static_cast<float>(wheel_base_);
        cfg.base.wheel_track = 0.0f;
        cfg.base.max_speed = 1.0f;
        cfg.base.max_angular = 3.14f;

        // RPMSG config: use parameter values or NULL for defaults
        cfg.ctrl_dev =
            rpmsg_ctrl_dev_.empty() ? nullptr : rpmsg_ctrl_dev_.c_str();
        cfg.data_dev =
            rpmsg_data_dev_.empty() ? nullptr : rpmsg_data_dev_.c_str();
        cfg.service_name =
            rpmsg_service_name_.empty() ? nullptr : rpmsg_service_name_.c_str();
        cfg.local_addr = static_cast<uint32_t>(rpmsg_local_addr_);
        cfg.remote_addr = static_cast<uint32_t>(rpmsg_remote_addr_);
        cfg.reduction_ratio = static_cast<float>(reduction_ratio_);
        cfg.ff_factor = static_cast<float>(ff_factor_);
        cfg.pid_kp = static_cast<float>(pid_kp_);
        cfg.pid_ki = static_cast<float>(pid_ki_);
        cfg.pid_kd = static_cast<float>(pid_kd_);
        cfg.cfg_send_on_startup = cfg_send_on_startup_;
        cfg.feedback_enable = feedback_enable_;

        chassis_ = chassis_alloc("drv_rpmsg_esos", &cfg);
        if (!chassis_) {
            RCLCPP_ERROR(get_logger(), "Failed to create chassis device");
            throw std::runtime_error("Failed to create chassis device");
        }

        RCLCPP_INFO(get_logger(), "Chassis initialized successfully");
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double vx = msg->linear.x;
        double wz = msg->angular.z;

        // Apply motor factors for wheel speed correction
        // Decompose to wheel speeds, apply factors, then recompose
        double v_left = vx - wz * wheel_base_ / 2.0;
        double v_right = vx + wz * wheel_base_ / 2.0;

        v_left *= motor1_factor_;
        v_right *= motor2_factor_;

        double corrected_vx = (v_left + v_right) / 2.0;
        double corrected_wz = (v_right - v_left) / wheel_base_;

        chassis_velocity_t vel;
        vel.vx = static_cast<float>(corrected_vx);
        vel.vy = static_cast<float>(msg->linear.y);
        vel.wz = static_cast<float>(corrected_wz);

        if (chassis_) {
            chassis_set_velocity(chassis_, &vel);
        }

        last_cmd_time_ = now();
    }

    void odom_timer_callback() {
        // Check cmd_vel timeout
        auto dt = (now() - last_cmd_time_).seconds();
        if (dt > cmd_vel_timeout_) {
            chassis_velocity_t zero = {0.0f, 0.0f, 0.0f};
            if (chassis_) {
                chassis_set_velocity(chassis_, &zero);
            }
        }

        // Get odometry from chassis
        chassis_velocity_t vel;
        chassis_pose_t pose;
        if (!chassis_ || chassis_get_odom(chassis_, &vel, &pose) != CHASSIS_OK) {
            return;
        }

        auto stamp = now();

        // Publish odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        // Position
        odom_msg.pose.pose.position.x = pose.x;
        odom_msg.pose.pose.position.y = pose.y;
        odom_msg.pose.pose.position.z = 0.0;

        // Orientation (quaternion from yaw)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pose.yaw);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Velocity
        odom_msg.twist.twist.linear.x = vel.vx;
        odom_msg.twist.twist.linear.y = vel.vy;
        odom_msg.twist.twist.angular.z = vel.wz;

        odom_pub_->publish(odom_msg);

        // Publish TF
        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id = base_frame_;

            tf.transform.translation.x = pose.x;
            tf.transform.translation.y = pose.y;
            tf.transform.translation.z = 0.0;

            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(tf);
        }
    }

    // Chassis device
    struct chassis_dev *chassis_;

    // Parameters
    double send_hz_;
    double odom_hz_;
    double cmd_vel_timeout_;
    bool publish_tf_;
    std::string odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    double wheel_diameter_;
    double wheel_base_;
    double motor1_factor_;
    double motor2_factor_;
    double reduction_ratio_;
    double ff_factor_;
    double pid_kp_;
    double pid_ki_;
    double pid_kd_;
    bool cfg_send_on_startup_;
    bool feedback_enable_;
    std::string rpmsg_ctrl_dev_;
    std::string rpmsg_data_dev_;
    std::string rpmsg_service_name_;
    int64_t rpmsg_local_addr_;
    int64_t rpmsg_remote_addr_;

    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    // State
    rclcpp::Time last_cmd_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<EsosBaseControlNode>());
    } catch (const std::exception &e) {
        fprintf(stderr, "esos_base_control_node exception: %s\n", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
