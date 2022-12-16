// Copyright Sanchit Tanwar
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file Obstacle_avoider.cpp
 * @author Sanchit Tanwar
 * @brief Illustration of obstacle avoiding algorithm
 * @version 0.1
 * @date 2022-12-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ObstacleAvoider : public rclcpp::Node {
 public:
  ObstacleAvoider():
          Node("walker_node") {
            /**
             * @brief Initializing a publisher and subscriber.
             * 
             */

            vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);

            lidar_subscriber_ = this->create_subscription
            <sensor_msgs::msg::LaserScan>(
              "scan", 5, std::bind(&ObstacleAvoider::lidar_callback, this, _1));
          }

 private:
    /**
     * @brief Check for obstacles using lidar
     * 
     * if obstacle distance is less than threshold, rotate the robot
     * 
     * @param msg : Lidar message
     */

    void lidar_callback(const sensor_msgs::msg::LaserScan &msg) {
      if (msg.header.stamp.sec == 0) {
        return;
      }
      auto lidar_data = msg.ranges;
      auto field_of_view = 60;
      auto angle_initial = 330;

      bool flag = false;

      for (int i = angle_initial ; i < angle_initial + field_of_view ; i++) {
        if (lidar_data[i % 360] < 0.5) {
          flag = true;
          break;
        }
      }

      if (flag) {
          move_robot(0.0, 0.3);
      } else {
          move_robot(0.5, 0.0);
      }
    }

    /**
     * @brief function to move robot straight when there is no obstacle 
     * 
     * 
     * @param linear_velocity : linear velocity
     * @param angular_velocity  : angular velocity
     */

    void move_robot(double linear_velocity , double angular_velocity) {
      auto velocity = geometry_msgs::msg::Twist();

      velocity.linear.x = linear_velocity;
      velocity.angular.z = angular_velocity;

      vel_publisher_-> publish(velocity);
    }

    /**
     * @brief private data_members
     * 
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
     lidar_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<ObstacleAvoider>());
    rclcpp::shutdown();
    return 0;
}

