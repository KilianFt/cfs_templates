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

#include <string>

#include "node_templates/subpub_example.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace subpub_example
{

SubpubExample::SubpubExample()
: Node("subpub_example"), count_(0)  // initialize variable count_ as 0
{
  RCLCPP_INFO(get_logger(), "Creating subpub example");

  // declare a ROS parameter, it can be used in any node
  this->declare_parameter<std::string>("my_parameter_name", "Hello CFS");

  publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);

  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "my_topic", 10, std::bind(&SubpubExample::my_topic_callback, this, _1));

  // time that executes time_callback twice every second
  timer_ = this->create_wall_timer(
    500ms, std::bind(&SubpubExample::timer_callback, this));
}

// this function is called by timer
void SubpubExample::timer_callback()
{
  // get ROS parameter
  this->get_parameter("my_parameter_name", parameter_string_);

  // create message
  auto message = std_msgs::msg::String();
  message.data = parameter_string_ + std::to_string(count_++);

  // publish message
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

// subscription callback
void SubpubExample::my_topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

}  // namespace subpub_example
