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

#include "node_templates/timesync_subscription.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace timesync_subscription
{

TimesyncSubscription::TimesyncSubscription()
: Node("timesync_subscription")
{
  RCLCPP_INFO(get_logger(), "Creating timesync subscription example");

  TimesyncSubscription::init_parameters();

  subscription_temp_1_.subscribe(this, sub1_topic_name_);
  subscription_temp_2_.subscribe(this, sub2_topic_name_);

  sync_ = std::make_shared<message_filters::TimeSynchronizer
      <std_msgs::msg::String, std_msgs::msg::String>>(
    subscription_temp_1_, subscription_temp_2_,
    3);
  sync_->registerCallback(std::bind(&TimesyncSubscription::my_sync_callback, this, _1, _2));
}

void TimesyncSubscription::init_parameters()
{
  // declare parameter with default values
  this->declare_parameter("subsciption1.topic", "topic1");
  this->declare_parameter("subsciption2.topic", "topic2");

  // get global values, usually set by yaml file. They will overwrite the default values if present
  this->get_parameter("subsciption1.topic", sub1_topic_name_);
  this->get_parameter("subsciption2.topic", sub2_topic_name_);
}

// subscription callback
void TimesyncSubscription::my_sync_callback(
  const std_msgs::msg::String::ConstSharedPtr & tmp_1,
  const std_msgs::msg::String::ConstSharedPtr & tmp_2) const
{
  const char * temp_1 = tmp_1->data.c_str();
  const char * temp_2 = tmp_2->data.c_str();
  RCLCPP_INFO(this->get_logger(), "I heard: '%s' and '%s'", temp_1, temp_2);
}

}  // namespace timesync_subscription
