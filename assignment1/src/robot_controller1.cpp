#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlerbot_controller/Vel.h"
#include "turtlerbot_controller/Velocity.h"

float x_pos, y_pos;
float th;
void turtleCallback(const turtlesim::Pose::ConstPtr& msg){
	x_pos=msg->x;
	y_pos=msg->y;
	th = msg->theta*180/3.14;
	ROS_INFO("The robot position is [%2.2f, %2.2f, %2.2f]", msg->x, msg->y, th);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "turtlesim_subscriber");
	
	ros::NodeHandle nh;
	
	ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("/kill");
	turtlesim::Kill kill_srv;
	kill_srv.request.name = "turtle1";
	kill_client.call(kill_srv);
	
	ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
	ros::ServiceClient client2 = nh.serviceClient<turtlerbot_controller::Velocity>("/velocity");
	
	client.waitForExistence();
	client2.waitForExistence();
	
	turtlerbot_controller::Velocity serverVel;
	serverVel.request.min= 0.0;
	serverVel.request.max = 3.0;
	
	
	turtlesim::Spawn srv;
	srv.request.x =2.0;
	srv.request.y=1.0;
	srv.request.theta=0.0;
	srv.request.name="rpr_turtle";
	client.call(srv);
	
	ros::Subscriber sub = nh.subscribe("rpr_turtle/pose", 10, turtleCallback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("rpr_turtle/cmd_vel",10);
	
	ros::Publisher pub2 = nh.advertise<turtlerbot_controller::Vel>("new_topic",10);
	
	ros::Rate loop_rate(100);
	geometry_msgs::Twist turtle_vel;
	turtlerbot_controller::Vel cust_vel;
	
	while(ros::ok() && y_pos<10){
		client2.call(serverVel);
		if(x_pos>9.0){
			if(th-180 <= 0)
				turtle_vel.angular.z=2;
		}
		else if (x_pos<2.0){
			if( 180 - th >=0)
				turtle_vel.angular.z=-2;
			
		}
		
		if(x_pos <= 9.0 && x_pos >= 2.0)
			turtle_vel.angular.z=0.0;
		
		turtle_vel.linear.x = 1.0;
		cust_vel.name = "lin";
		cust_vel.vel=1;
		
		pub.publish(turtle_vel);
		pub2.publish(cust_vel);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
} 
