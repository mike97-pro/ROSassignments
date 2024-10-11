// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <chrono>
#include <cinttypes>

//add the ROS header
#include "rclcpp/rclcpp.hpp"
//add the headers for messages and services
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class MyController : public rclcpp::Node
{
public:
  MyController()
  : Node("my_controller")
  { 
    //initialize the publisher, the subscriber, client1, client2
    subscription_ = this->create_subscription<turtlesim::msg::Pose>("/my_turtle/pose", 10, std::bind(&MyController::sub_callback, this, _1));
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/my_turtle/cmd_vel", 10);
    
    client1_ = this->create_client<turtlesim::srv::Kill>("/kill");
    client2_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
      
    while (!client1_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    while (!client2_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    
  }

  //function to call the kill service
  void call_client1(){
      auto request = std::make_shared<turtlesim::srv::Kill::Request>();
      request->name = "turtle1";
      auto result_future = client1_->async_send_request(request);
  }
  
  //function to call the spawn service
  void call_client2(){
      auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
      request->x = 2.0;
      request->y = 1.0;
      request->theta = 0.0;
      request->name = "my_turtle";
      auto result_future = client2_->async_send_request(request);
  }
private:

  //get the robot's position and set the velocity
  void sub_callback(const turtlesim::msg::Pose::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Robot pos is: %2.2f, %2.2f, %2.2f", msg->x, msg->y, msg->theta);
    
    geometry_msgs::msg::Twist turtle_vel;
     if(msg->x>9.0){
	if(msg->theta-180 <= 0)
		turtle_vel.angular.z=2;
     }
     else if (msg->x<2.0){
	if( 180 - msg->theta >=0)
		turtle_vel.angular.z=-2;
     }
		
     if(msg->x <= 9.0 && msg->x >= 2.0)
         turtle_vel.angular.z=0.0;
		
     turtle_vel.linear.x = 1.0;
     
     publisher_->publish(turtle_vel);
  }
  
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client1_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client2_;
};

int main(int argc, char * argv[])
{
  // init
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyController>();
  
  //call service 1 and 2
  node->call_client1();
  node->call_client2();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
