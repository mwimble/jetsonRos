// TODO
// Make singleton.

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

//#include "/usr/local/include/raspicam/raspicam_cv.h"
#include <camera_info_manager/camera_info_manager.h>

#include "FindObject.h"

using namespace cv;
using namespace std;

extern FindObject* findObject;

//void FindObject::configurationCallback(kaimi_mid_camera::kaimi_mid_camera_paramsConfig &config, uint32_t level) {
//	ROS_INFO("Reconfigure Request hue_low: %d, hue_high: %d, saturation_low: %d, saturation high: %d, value_low: %d, value_high: %d, contourSizeThreshold: %d",
//	         config.hue_low, config.hue_low,
//	         config.saturation_low, config.saturation_high,
//	         config.value_low, config.value_high,
//	         config.contourSizeThreshold);
//	if (findObject) {
//		findObject->iLowH = config.hue_low;
//		findObject->iHighH = config.hue_high;
//		findObject->iLowS = config.saturation_low;
//		findObject->iHighS = config.saturation_high;
//		findObject->iLowV = config.value_low;
//		findObject->iHighV = config.value_high;
//		findObject->contourSizeThreshold = config.contourSizeThreshold;
//	}
//}

//void FindObject::imageCb(Mat& image) {
void FindObject::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
    try {
    	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("[FindObject::imageCb] exception: %s", e.what());
      return;
    }

    //ROS_INFO("FindObject::imageCb] cv_ptr->image.rows: %d, cv_ptr->image.cols: %d", cv_ptr->image.rows, cv_ptr->image.cols);
    if (1 /*cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60*/) {
    	Mat imgHSV;
		cvtColor(cv_ptr->image, imgHSV, CV_BGR2HSV); // Convert the captured frame from BGR to HSV

		Mat imgThresholded;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//morphological closing (fill small holes in the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		double contourSize;
		Mat tempImage;

		imgThresholded.copyTo(tempImage);
		findContours(tempImage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		vector<Rect> boundRect( contours.size() );
		vector<vector<Point> > contours_poly( contours.size() );
		//ROS_INFO("Thresholded iLowH: %d, iHighH: %d, iLowS: %d, iHighS: %d, iLowV: %d, iHighV: %d, countours: %d", iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, contours.size());
		Point2f center;
		float radius;

		if (!contours.empty()) {
			// Find largest blob.
			size_t maxBlobIndex = -1;
			int maxBlobSize = 0;
			for (size_t i = 0; i < contours.size(); i++) {
				contourSize = contourArea(contours[i]);
				if ((contourSize > maxBlobSize) && (contourSize >= contourSizeThreshold)) {
					maxBlobIndex = i;
					maxBlobSize = contourSize;
				}
			}

			if (maxBlobIndex != -1) {
				approxPolyDP( Mat(contours[maxBlobIndex]), contours_poly[maxBlobIndex], 3, true );
				boundRect[maxBlobIndex] = boundingRect( Mat(contours_poly[maxBlobIndex]) );
				minEnclosingCircle( (Mat) contours_poly[maxBlobIndex], center, radius );

				double x = center.x;
				double y = center.y;

				int cols = cv_ptr->image.cols;
				int rows = cv_ptr->image.rows;

				if (showWindows_) {
					Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
					circle(cv_ptr->image, center, (int)radius, color, 2, 8, 0 );
				}

				stringstream msg;
				msg << "MidCamera:Found;X:" << x
					<< ";Y:" << y
					<< ";AREA:" << maxBlobSize
					<< ";I:" << maxBlobIndex
					<< ";ROWS:" << cv_ptr->image.rows
					<< ";COLS:" << cv_ptr->image.cols;
				std_msgs::String message;
				message.data = msg.str();
				midSampleFoundPub_.publish(message);
				ROS_INFO("[FindObject::imageCb] FOUND at x: %7.2f, y: %7.2f, area: %d", x, y, maxBlobSize);
			}
		} else {
			stringstream msg;
			msg << "MidCamera:NotFound;X:0;Y:0;AREA:0;I:0;ROWS:"
				<< cv_ptr->image.rows
				<< ";COLS:" << cv_ptr->image.cols;
			std_msgs::String message;
			message.data = msg.str();
			midSampleFoundPub_.publish(message);
			ROS_INFO("[FindObject::imageCb] NOT FOUND");
		}

		if (showWindows_) {
			imshow("[kaimi_mid_camera] Raw Image", cv_ptr->image); //show the original image
			imshow("[kaimi_mid_camera] Thresholded Image", imgThresholded); //show the thresholded image
			ROS_INFO("[FindObject::imageCb] showed images");
			cv::waitKey(25);
		}

    }
}


FindObject::FindObject() : it_(nh_),
	iLowH(111),
	iHighH(150),
	iLowS(10),
	iHighS(210),
	iLowV(20),
	iHighV(170),
	contourSizeThreshold(100),
	showWindows_(false)
	{
//	f = boost::bind(&FindObject::configurationCallback, _1, _2);
//	dynamicConfigurationServer.setCallback(f);

	ros::param::get("~image_topic_name", imageTopicName_);
	ros::param::get("~show_windows", showWindows_);
	ROS_INFO("PARAM image_topic_name: %s", imageTopicName_.c_str());
	ROS_INFO("PARAM show_windows: %d", showWindows_);
	ROS_INFO("[KaimiMidCamera iLowH: %d, iHighH: %d, iLowS: %d, iHighS: %d, iLowV: %d, iHighV: %d", iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);

	showWindows_ = true;

	midSampleFoundPub_ = nh_.advertise<std_msgs::String>("midSampleFound", 2);
	if (showWindows_) {
		static const char* controlWindowName = "[kaimi_mid_camera] Control";

    	namedWindow("[kaimi_mid_camera] Raw Image", CV_WINDOW_AUTOSIZE);
    	namedWindow("[kaimi_mid_camera] Thresholded Image", CV_WINDOW_AUTOSIZE);

		// Create trackbars in "Control" window
		namedWindow(controlWindowName, CV_WINDOW_AUTOSIZE); //create a window called "Control"
		cvCreateTrackbar("LowH", controlWindowName, &iLowH, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", controlWindowName, &iHighH, 179);

		cvCreateTrackbar("LowS", controlWindowName, &iLowS, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", controlWindowName, &iHighS, 255);

		cvCreateTrackbar("LowV", controlWindowName, &iLowV, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", controlWindowName, &iHighV, 255);

		cvCreateTrackbar("contourSizeThreshold", controlWindowName, &contourSizeThreshold, 10000);
	}

	image_sub_ = it_.subscribe(imageTopicName_, 1, &FindObject::imageCb, this);


 //    int fps;
 //    nh_.param("fps", fps, 20);
 //    std::string color_mode = "rgb8";

 //    //image_transport::ImageTransport it(nh);
 //    // std::string camera_name = nh.getNamespace();
 //    // camera_info_manager::CameraInfoManager cinfo_(nh, camera_name);

 //    raspicam::RaspiCam_Cv cap;
 //    cap.open();
	// ros::Rate rate(fps);
 //    while(ros::ok()) {
 //        Mat imgOriginal;

 //        //bool bSuccess = cap.read(imgOriginal); // read a new frame from video
 //        cap.grab();
 //        bool bSuccess = true;
 //        cap.retrieve(imgOriginal); // read a new frame from video

 //        // std_msgs::Header header();
 //        // cv_bridge::CvImage imgmsg;
 //        // sensor_msgs::CameraInfo ci = cinfo_.getCameraInfo();
 //        // imgmsg.header.frame_id = camera_name + "_optical_frame";
 //        // ci.header.frame_id = imgmsg.header.frame_id;
 //        // imgmsg.encoding = sensor_msgs::image_encodings::BGR8;// color_mode;
 //        // imgmsg.image = imgOriginal;
 //        imageCb(imgOriginal);//, ci, ros::Time::now());
 //        rate.sleep();
 //        ros::spinOnce();
	// };
}

FindObject& FindObject::Singleton() {
	static FindObject singleton_;
	return singleton_;
}


