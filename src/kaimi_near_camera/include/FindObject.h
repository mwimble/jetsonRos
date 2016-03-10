
#include <ros/ros.h>
#include <ros/console.h>

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <string>

#include "kaimi_near_camera/kaimi_near_camera_paramsConfig.h"

using namespace std;

class FindObject {
private:

	int iLowH;
	int iHighH;

	int iLowS; 
	int iHighS;

	int iLowV;
	int iHighV;

	int contourSizeThreshold;


	static const string OPENCV_WINDOW;

	ros::NodeHandle nh_;
	string imageTopicName_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher nearSampleFoundPub_;

	dynamic_reconfigure::Server<kaimi_near_camera::kaimi_near_camera_paramsConfig> dynamicConfigurationServer;
	dynamic_reconfigure::Server<kaimi_near_camera::kaimi_near_camera_paramsConfig>::CallbackType f;

	static void configurationCallback(kaimi_near_camera::kaimi_near_camera_paramsConfig &config, uint32_t level);

	void imageCb(const sensor_msgs::ImageConstPtr& msg);


public:
	FindObject();
};