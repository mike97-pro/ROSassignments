#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "assignment1/VelocityA.h"

float x_pos=0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	ROS_INFO("The robot x position is [%2.2f]", msg->pose.pose.position.x);
	x_pos = msg->pose.pose.position.x;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "robot_controller");
	
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/odom", 10, odomCallback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	ros::ServiceClient client = nh.serviceClient<assignment1::VelocityA>("/armonic_velocity");
	client.waitForExistence();
	assignment1::VelocityA serverVel;
	
	
	ros::Rate loop_rate(100);
	geometry_msgs::Twist robot_vel;
	while(ros::ok()){
		
		if(x_pos >= 2 && x_pos<=9){
			ROS_INFO("The robot is in place");
			serverVel.request.x = x_pos;
			client.call(serverVel);
			robot_vel.linear.x = serverVel.response.vel_x;
			pub.publish(robot_vel);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
} 
