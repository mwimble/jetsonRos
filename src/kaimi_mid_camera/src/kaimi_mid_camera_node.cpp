// TODO dynamic params for everything.

#include <ros/ros.h>
#include <ros/console.h>

#include "FindObject.h"

FindObject* findObject; 

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinect_mid_camera_node");
	findObject = &FindObject::Singleton();
	ros::spin();
	return 0;
}

