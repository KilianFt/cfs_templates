// Copyright 2021 CFS
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2 specific imports
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "node_templates/subpub_example.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;


namespace subpub_example
{

SubpubExample::SubpubExample()
: rclcpp::Node("subpub_example"),
  count_(0)  // initialize variable count_ as 0
{
  RCLCPP_INFO(get_logger(), "Creating subpub example");

  // declare_parameter("controller_frequency", 20.0);
  // declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  // declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  // declare_parameter("controller_plugins", default_ids_);
}

// deconstructor if needed
// ControllerServer::~ControllerServer()
// {
//   progress_checker_.reset();
//   goal_checkers_.clear();
//   controllers_.clear();
//   costmap_thread_.reset();
// }

SubpubExample::on_configure(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring subpub example node interface");

  // get_parameter("controller_frequency", controller_frequency_);
  // get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  // get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  // RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  // initialize publisher with msg type, topic name and required queue size
  publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);

  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "my_topic", 10, std::bind(&SubpubExample::my_topic_callback, this, _1));

  // time that executes time_callback twice every second
  timer_ = this->create_wall_timer(
    500ms, std::bind(&SubpubExample::timer_callback, this));
}



void SubpubExample::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, CFS22! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

void SubpubExample::my_topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}


}  // namespace subpub_example


