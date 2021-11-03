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

#ifndef NODE_TEMPLATES__SUBPUB_EXAMPLE_HPP_
#define NODE_TEMPLATES__SUBPUB_EXAMPLE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// using namespace std::chrono_literals;

namespace subpub_example
{

class SubpubExample : public rclcpp::Node
{
public:
  SubpubExample();

private:
  // functions
  void timer_callback();

  void my_topic_callback(const std_msgs::msg::String::SharedPtr msg) const;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // variables
  size_t count_;
  std::string parameter_string_;
};

}  // namespace subpub_example

#endif  // NODE_TEMPLATES__SUBPUB_EXAMPLE_HPP_
