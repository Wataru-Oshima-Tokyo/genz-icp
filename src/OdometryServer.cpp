// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

// GenZ-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 2 headers
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace genz_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    // clang-format off
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    publish_debug_clouds_ = declare_parameter<bool>("visualize", publish_debug_clouds_);
    config_.max_range = declare_parameter<double>("max_range", config_.max_range);
    config_.min_range = declare_parameter<double>("min_range", config_.min_range);
    config_.deskew = declare_parameter<bool>("deskew", config_.deskew);
    config_.voxel_size = declare_parameter<double>("voxel_size", config_.max_range);
    config_.map_cleanup_radius = declare_parameter<double>("map_cleanup_radius", config_.map_cleanup_radius);
    config_.planarity_threshold = declare_parameter<double>("planarity_threshold", config_.planarity_threshold);
    config_.max_points_per_voxel = declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel);
    config_.max_points_per_voxelized_scan = declare_parameter<int>("max_points_per_voxelized_scan", config_.max_points_per_voxelized_scan);
    config_.min_points_per_voxelized_scan = declare_parameter<int>("min_points_per_voxelized_scan", config_.min_points_per_voxelized_scan);
    config_.max_num_iterations = declare_parameter<int>("max_num_iterations", config_.max_num_iterations);
    config_.convergence_criterion = declare_parameter<double>("convergence_criterion", config_.convergence_criterion);
    config_.initial_threshold = declare_parameter<double>("initial_threshold", config_.initial_threshold);
    config_.min_motion_th = declare_parameter<double>("min_motion_th", config_.min_motion_th);
    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config_.min_range = 0.0;
    }
    // clang-format on

    // Construct the main GenZ-ICP odometry node
    odometry_ = genz_icp::pipeline::GenZICP(config_);
    callback_group_sub1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Subscription options for subscriptions
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_sub1_;
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_sub2_;
    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1), sub1_opt);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::ProcessImuData, this, std::placeholders::_1), sub2_opt);
    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/genz/odometry", qos);
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("/genz/trajectory", qos);
    path_msg_.header.frame_id = odom_frame_;
    if (publish_debug_clouds_) {
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/local_map", qos);
        planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/planar_points", qos);
        non_planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/non_planar_points", qos);
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    RCLCPP_INFO(this->get_logger(), "GenZ-ICP ROS 2 odometry node initialized");
}



void OdometryServer::ProcessImuData(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    imu_data_.emplace_back(*msg);
}


void OdometryServer::GetSyncedImuData(const rclcpp::Time &scan_time) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    // Filter IMU messages around the scan time
    // for (const auto &imu : imu_data_) {
    //     if (imu.header.stamp >= scan_time - rclcpp::Duration(0.05) &&
    //         imu.header.stamp <= scan_time + rclcpp::Duration(0.05)) {
    //         synced_imu_data.push_back(imu);
    //     }
    // }

}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    const auto scan_time = msg->header.stamp;
    // const auto imu_data = GetSyncedImuData(scan_time);
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(msg);
    }();
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // Register frame, main entry point to GenZ-ICP pipeline
    const auto &[planar_points, non_planar_points] = odometry_.RegisterFrame(points, timestamps);

    // Compute the pose using GenZ, ego-centric to the LiDAR
    const Sophus::SE3d genz_pose = odometry_.poses().back();


    // Spit the current estimated pose to ROS msgs
    PublishOdometry(genz_pose, msg->header.stamp, cloud_frame_id);
    // Publishing this clouds is a bit costly, so do it only if we are debugging
    if (publish_debug_clouds_) {
        PublishClouds(msg->header.stamp, cloud_frame_id, planar_points, non_planar_points);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const rclcpp::Time &stamp,
                                     const std::string &cloud_frame_id) {

    // publish trajectory msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    // publish odometry msg
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_publisher_->publish(std::move(odom_msg));
}

void OdometryServer::PublishClouds(const rclcpp::Time &stamp,
                                   const std::string &cloud_frame_id,
                                   const std::vector<Eigen::Vector3d> &planar_points,
                                   const std::vector<Eigen::Vector3d> &non_planar_points) {
    std_msgs::msg::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    // Publish map
    const auto genz_map = odometry_.LocalMap();

    // debugging happens in an egocentric world
    std_msgs::msg::Header cloud_header;
    cloud_header.stamp = stamp;
    cloud_header.frame_id = cloud_frame_id;

    map_publisher_->publish(std::move(EigenToPointCloud2(genz_map, odom_header)));
    planar_points_publisher_->publish(std::move(EigenToPointCloud2(planar_points, cloud_header)));
    non_planar_points_publisher_->publish(std::move(EigenToPointCloud2(non_planar_points, cloud_header)));

}
}  // namespace genz_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(genz_icp_ros::OdometryServer)
