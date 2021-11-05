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
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cfs_msgs/srv/combine_strings.hpp"


void combine(
  const std::shared_ptr<cfs_msgs::srv::CombineStrings::Request> request,
  std::shared_ptr<cfs_msgs::srv::CombineStrings::Response> response)
{
  std::string a = request->a;
  std::string b = request->b;
  response->combined = a + b;
  RCLCPP_INFO(
    rclcpp::get_logger("rclcpp"), "Incoming request\na: %s" " b: %s",
    request->a.c_str(), request->b.c_str());
  RCLCPP_INFO(
    rclcpp::get_logger("rclcpp"), "sending back response: [%s]",
    response->combined.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_server_example");

  rclcpp::Service<cfs_msgs::srv::CombineStrings>::SharedPtr service =
    node->create_service<cfs_msgs::srv::CombineStrings>("combine_strings", &combine);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to combine strings.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
