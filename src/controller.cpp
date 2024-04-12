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
    sub_kinematics_ = create_subscription<Odometry>("/localization/kinematic_state", 1, [this](const Odometry::SharedPtr msg){ odometry_ = msg; });
    trajectory_msg_ = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>();
 
    prepareTrajectory(); // Trajectory dosyasını oku ve hazırla

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

    control_cmd.stamp = this->now();
    steerCmd = calcSteerCmd(); // Araç için hesaplanan direksiyon açısı
    accCmd = calcAccCmd(); // Araç için hesaplanan ivme

    control_cmd.longitudinal.acceleration = accCmd;
    control_cmd.lateral.steering_tire_angle = steerCmd;
    pub_cmd_->publish(control_cmd); // Hesaplanan kontrol komutlarını yayınla

    
    lateral_deviation = calcLateralDeviation(); // Araç için hesaplanan lateral deviation
    lateral_deviation_msg.data = lateral_deviation;
    pub_lateral_deviation_->publish(lateral_deviation_msg); // Lateral deviation mesajını yayınla

    longitudinal_velocity_error = calcLongitudinalVelocityError(); // Araç için hesaplanan longitudinal velocity error
    longitudinal_velocity_error_msg.data = longitudinal_velocity_error;
    pub_longitudinal_velocity_error_->publish(longitudinal_velocity_error_msg); // Longitudinal velocity error mesajını yayınla
    
    trajectory_msg_->header.stamp = this->now();
    pub_trajectory_->publish(*trajectory_msg_); // Trajectory mesajını yayınla
  }

  double Controller::calcSteerCmd()
  {
    double steer = 0.0;
    
    // Calculate the steering angle here.
    if (odometry_ != nullptr)
    {
      tf2::Quaternion q(
          odometry_->pose.pose.orientation.x,
          odometry_->pose.pose.orientation.y,
          odometry_->pose.pose.orientation.z,
          odometry_->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      steer = yaw;
    }
    double min_distance = std::numeric_limits<double>::max();
    size_t k = trajectory_points.size();
    for (size_t i = closest_point_index; i < k; i++)
    {
      double distance = sqrt(pow(trajectory_points[i][0] - odometry_->pose.pose.position.x, 2) + pow(trajectory_points[i][1] - odometry_->pose.pose.position.y, 2));
      if (distance < min_distance)
      {
        min_distance = distance;
        closest_point_index = i;
      }
    }
    size_t look_head_index = closest_point_index;
    while(look_head_index < k-1 &&
          std::hypot(trajectory_points[look_head_index][0] - odometry_->pose.pose.position.x, trajectory_points[look_head_index][1] - odometry_->pose.pose.position.y) < look_head_distance){
      ++look_head_index;
    }if(look_head_index == k-1){ //Hedef nokta son nokta ise
      target_speed = 0.0;
      steer = 0.0;
    }else{ //Hedef nokta son nokta degilse
      double target_x = trajectory_points[look_head_index][0];
      double target_y = trajectory_points[look_head_index][1];
      double dx = target_x - odometry_->pose.pose.position.x;
      double dy = target_y - odometry_->pose.pose.position.y;
      double target_yaw = atan2(dy, dx);
      double delta_yaw = target_yaw - steer;

      while (delta_yaw > M_PI){delta_yaw -= 2 * M_PI;}
      while (delta_yaw < -M_PI){delta_yaw += 2 * M_PI;}

      steer = delta_yaw;
      target_speed = trajectory_points[look_head_index][7];
      look_head_distance = target_speed * 1.4; //Dinamik look_head_distance mesafesi
    }    
    return steer;
  }

  double Controller::calcAccCmd()
  {
    double acc = 0.0;

    const double Kp = 0.8; 
    const double Ki = 0.00001;
    const double Kd = 0.00002;

    double error = target_speed - odometry_->twist.twist.linear.x;

    static double integral = 0.0;
    integral += error;

    static double previous_error = 0.0;
    double derivative = error - previous_error;
    previous_error = error;

    acc = Kp * error + Ki * integral + Kd * derivative;

    const double max_acceleration = 5.0;
    acc = std::min(acc, max_acceleration);

    return acc;
  }

  double Controller::calcLateralDeviation()
  {
    double lateral_deviation = 0.0;
    // Calculate the lateral deviation here.
    double car_y = odometry_->pose.pose.position.y;
    double car_x = odometry_->pose.pose.position.x;
    double target_y = trajectory_points[closest_point_index][1];
    double target_x = trajectory_points[closest_point_index][0];
    lateral_deviation = sqrt(pow(car_x - target_x, 2) + pow(car_y - target_y, 2)); // Hesaplanan lateral deviation
    return lateral_deviation;
  }

  double Controller::calcLongitudinalVelocityError()
  {
    double longitudinal_velocity_error = 0.0;
    // Calculate the longitudinal velocity error here.
    longitudinal_velocity_error = target_speed - odometry_->twist.twist.linear.x; // Hesaplanan longitudinal velocity error
    longitudinal_velocity_error = std::abs(longitudinal_velocity_error);
    return longitudinal_velocity_error;
  }

  void Controller::prepareTrajectory()
  {
    std::filesystem::path current_path = std::filesystem::current_path();
    std::string local_path = "/src/control_task/controller_task_node/config/waypoints.txt";
    std::string waypoint_path = current_path.string() + local_path;
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
