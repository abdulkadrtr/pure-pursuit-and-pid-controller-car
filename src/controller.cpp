//  Copyright 2023 LeoDrive A.Ş. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "controller_task_node/controller.hpp"

#include <algorithm>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <filesystem>
namespace controller
{


  Controller::Controller(const rclcpp::NodeOptions &options)
      : Node("controller", options)
  {
    pub_cmd_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", 1);
    pub_trajectory_ = create_publisher<Trajectory>("/control/trajectory", 1);;
    pub_lateral_deviation_ = create_publisher<std_msgs::msg::Float64>("/control/lateral_deviation", 1);;
    pub_longitudinal_velocity_error_ = create_publisher<std_msgs::msg::Float64>("/control/longitudinal_velocity_error", 1);;
    pub_gear_cmd_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 1);;
    sub_kinematics_ = create_subscription<Odometry>(
        "/localization/kinematic_state", 1, [this](const Odometry::SharedPtr msg)
        { odometry_ = msg; });

    trajectory_msg_ = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>();
    prepareTrajectory();
    using namespace std::literals::chrono_literals;
    timer_ = rclcpp::create_timer(
        this, get_clock(), 30ms, std::bind(&Controller::onTimer, this));
      
  }

  void Controller::onTimer()
  {
    // To make vehicle able to drive, we need to publish the gear command to drive

    autoware_auto_vehicle_msgs::msg::GearCommand gear_cmd;
    gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
    gear_cmd.stamp = this->now();
    pub_gear_cmd_->publish(gear_cmd);

    // Calculate the control command and error values.
    // araca sabit 0.2 hız verin
    /*
    autoware_auto_control_msgs::msg::AckermannControlCommand control_cmd;
    control_cmd.stamp = this->now();
    control_cmd.lateral.steering_tire_angle = 0.2;
    control_cmd.longitudinal.acceleration = 0.1;
    control_cmd.longitudinal.speed = 0.2;
    pub_cmd_->publish(control_cmd);
    */

    
    //Yol noktaları publish ediliyor.
    trajectory_msg_->header.stamp = this->now();
    pub_trajectory_->publish(*trajectory_msg_);
  }

  double Controller::calcSteerCmd()
  {
    double steer = 0.0;
    // Calculate the steering angle here.
    return steer;
  }

  double Controller::calcAccCmd()
  {
    double acc = 0.0;
    // Calculate the acceleration here.
    return acc;
  }
  double calcLateralDeviation()
  {
    double lateral_deviation = 0.0;
    // Calculate the lateral deviation here.
    return lateral_deviation;
  }

  void Controller::prepareTrajectory()
  {
    std::filesystem::path current_path = std::filesystem::current_path();
    std::string local_path = "/src/control_task/controller_task_node/config/waypoints.txt";
    std::string waypoint_path = current_path.string() + local_path;
    std::vector<std::array<double, 8>> trajectory_points;
    std::ifstream file(waypoint_path.c_str());
    if (file.is_open())
    {
        double values[8];
        while (file >> values[0] >> values[1] >> values[2] >> values[3] >> values[4] >> values[5] >> values[6] >> values[7])
        {
            std::array<double, 8> point;
            for (int i = 0; i < 8; i++)
            {
                point[i] = values[i];
            }
            trajectory_points.push_back(point);
        }
      file.close();
      autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
      int k = trajectory_points.size();
      for (int i = 0; i < k; i++)
      {
          trajectory_point.pose.position.x = trajectory_points[i][0];
          trajectory_point.pose.position.y = trajectory_points[i][1];
          trajectory_point.pose.position.z = trajectory_points[i][2];
          trajectory_point.pose.orientation.x = trajectory_points[i][3];
          trajectory_point.pose.orientation.y = trajectory_points[i][4];
          trajectory_point.pose.orientation.z = trajectory_points[i][5];
          trajectory_point.pose.orientation.w = trajectory_points[i][6];
          trajectory_point.longitudinal_velocity_mps = trajectory_points[i][7];
          trajectory_msg_->points.push_back(trajectory_point);
      }
      trajectory_msg_->header.frame_id = "map";
    }else{
        RCLCPP_ERROR(this->get_logger(), "Dosya açılamadı.");
    }
  }

} // namespace controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(controller::Controller)
