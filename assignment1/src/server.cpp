#include "ros/ros.h"
#include "assignment1/VelocityA.h"

#define _USE_MATH_DEFINES
#include <cmath>


bool random_callback(assignment1::VelocityA::Request &req,
		assignment1::VelocityA::Response &res) {
	
	if(req.x < 2 || req.x > 9)
		return false;
	
	res.vel_x = 0.1 + 2*sin(M_PI*req.x/7-2*M_PI/7);
	
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "server");
	ros::NodeHandle nh;
	
	ros::ServiceServer server = nh.advertiseService("/armonic_velocity", random_callback);
	ros::spin();
	return 0;
}
