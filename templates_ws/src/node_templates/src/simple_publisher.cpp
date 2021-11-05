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

#include "node_templates/simple_publisher.hpp"

using namespace std::chrono_literals;

namespace simple_publisher
{

SimplePublisher::SimplePublisher()
: Node("simple_publisher"), count_(0)  // initialize variable count_ as 0
{
  RCLCPP_INFO(get_logger(), "Creating subpub example");

  publisher_ = this->create_publisher<std_msgs::msg::String>("pub_topic", 10);

  // time that executes time_callback twice every second
  timer_ = this->create_wall_timer(
    700ms, std::bind(&SimplePublisher::timer_callback, this));
}

// this function is called by timer
void SimplePublisher::timer_callback()
{
  // create message
  auto message = std_msgs::msg::String();
  message.data = "Hello CFS " + std::to_string(count_++);

  // publish message
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

}  // namespace simple_publisher
