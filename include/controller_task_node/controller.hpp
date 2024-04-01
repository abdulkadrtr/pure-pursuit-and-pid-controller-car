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

#ifndef CONTROLLER_TASK_NODE__CONTROLLER_HPP_
#define CONTROLLER_TASK_NODE__CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>

namespace controller
{
  using autoware_auto_control_msgs::msg::AckermannControlCommand;
  using autoware_auto_planning_msgs::msg::Trajectory;
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using geometry_msgs::msg::Pose;
  using geometry_msgs::msg::Twist;
  using nav_msgs::msg::Odometry;

  class Controller : public rclcpp::Node
  {
  public:
    explicit Controller(const rclcpp::NodeOptions &options);
    ~Controller() = default;

  private:
    rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
    rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_lateral_deviation_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_longitudinal_velocity_error_;

    rclcpp::TimerBase::SharedPtr timer_;
    Odometry::SharedPtr odometry_;

    void onTimer();
    double calcSteerCmd();
    double calcAccCmd();
    double calcLateralDeviation();
  };

} // namespace controller

#endif // CONTROLLER_TASK_NODE__CONTROLLER_HPP_
