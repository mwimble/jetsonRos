// TODO dynamic params for everything.

#include <ros/ros.h>
#include <ros/console.h>

#include "FindObject.h"

FindObject* findObject; 

int main(int argc, char** argv) {
	ROS_INFO("##### main start");
	ros::init(argc, argv, "kaimi_mid_camera_node");
	ROS_INFO("##### main post ros::init");
	findObject = &FindObject::Singleton();
	ROS_INFO("##### main post FindObject::Singleton()");
	ros::spin();
	return 0;
}

