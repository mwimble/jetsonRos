#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

using namespace cv;
using namespace std;

class FindObject {
private:
	int iLowH;
	int iHighH;

	int iLowS; 
	int iHighS;

	int iLowV;
	int iHighV;

	static const string OPENCV_WINDOW;

	ros::NodeHandle nh_;
	string imageTopicName_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
    }

    // Draw an example circle on the video stream
    if (1 /*cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60*/) {
    	Mat imgHSV;
		cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		Mat imgThresholded;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (fill small holes in the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow(OPENCV_WINDOW, cv_ptr->image); //show the original image
		cv::waitKey(3);
		//cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    }

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }


public:
	FindObject() : it_(nh_) {
		iLowH = 116;
		iHighH = 134;
		iLowS = 96;
		iHighS = 255;
		iLowV = 0;
		iHighV = 255;

		ros::param::param<std::string>("image_topic_name", imageTopicName_, "/camera/rgb/image_color");
		ROS_INFO("PARAM image_topic_name: %s", imageTopicName_.c_str());
		image_sub_ = it_.subscribe(imageTopicName_.c_str(), 1, &FindObject::imageCb, this);
	    namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
		
		//Create trackbars in "Control" window
		namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
		cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", "Control", &iHighH, 179);

		cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", "Control", &iHighS, 255);

		cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	}
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "kinect_find_object_node");
	FindObject findObject; 
	ros::spin();
	return 0;
}

const string FindObject::OPENCV_WINDOW = "Image window";
