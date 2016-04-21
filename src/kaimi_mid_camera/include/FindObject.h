
#include <ros/ros.h>
#include <ros/console.h>

#include "opencv2/core/core.hpp"
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

//#include "kaimi_mid_camera/kaimi_mid_camera_paramsConfig.h"

using namespace std;
using namespace cv;

class FindObject {
private:

	// Values for the sample thresholding operation.
	int iLowH;
	int iHighH;

	int iLowS;
	int iHighS;

	int iLowV;
	int iHighV;

	int contourSizeThreshold;
	// End values for the sample thresholding operation.

	ros::NodeHandle nh_;
	string imageTopicName_;
	bool showWindows_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher midSampleFoundPub_;

//	dynamic_reconfigure::Server<kaimi_mid_camera::kaimi_mid_camera_paramsConfig> dynamicConfigurationServer;
//	dynamic_reconfigure::Server<kaimi_mid_camera::kaimi_mid_camera_paramsConfig>::CallbackType f;
//
//	static void configurationCallback(kaimi_mid_camera::kaimi_mid_camera_paramsConfig &config, uint32_t level);

	//void imageCb(Mat& image);
	void imageCb(const sensor_msgs::ImageConstPtr& msg);

	FindObject();
	FindObject(FindObject const&) : it_(nh_) {};
	FindObject& operator=(FindObject const&) {};
	static FindObject* singleton;

public:
	static FindObject& Singleton();

};
