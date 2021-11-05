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

#ifndef NODE_TEMPLATES__TIMESYNC_SUBSCRIPTION_HPP_
#define NODE_TEMPLATES__TIMESYNC_SUBSCRIPTION_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

namespace timesync_subscription
{

class TimesyncSubscription : public rclcpp::Node
{
public:
  TimesyncSubscription();

private:
  // functions
  void my_sync_callback(
    const std_msgs::msg::String::ConstSharedPtr & tmp_1,
    const std_msgs::msg::String::ConstSharedPtr & tmp_2) const;
  void init_parameters();

  // subscriptions
  message_filters::Subscriber<std_msgs::msg::String> subscription_temp_1_;
  message_filters::Subscriber<std_msgs::msg::String> subscription_temp_2_;

  // time synchronizer
  std::shared_ptr<message_filters::TimeSynchronizer
    <std_msgs::msg::String, std_msgs::msg::String>> sync_;

  // variables
  std::string sub1_topic_name_;
  std::string sub2_topic_name_;
};

}  // namespace timesync_subscription

#endif  // NODE_TEMPLATES__TIMESYNC_SUBSCRIPTION_HPP_
