// TODO
// Make singleton.

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "FindObject.h"

using namespace cv;
using namespace std;

extern FindObject* findObject;

void FindObject::configurationCallback(kaimi_near_camera::kaimi_near_camera_paramsConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request hue_low: %d, hue_high: %d, saturation_low: %d, saturation high: %d, value_low: %d, value_high: %d, contourSizeThreshold: %d",
	         config.hue_low, config.hue_low,
	         config.saturation_low, config.saturation_high,
	         config.value_low, config.value_high,
	         config.contourSizeThreshold);
	//setTrackbarPos("LowV", "Control", 13/*config.value_low*/);
	if (findObject) {
		findObject->iLowV = config.value_low;
	}
}

void FindObject::imageCb(const sensor_msgs::ImageConstPtr& msg) {
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


		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		vector<int> smallBlobs;
		double contourSize;
		Mat tempImage;

		imgThresholded.copyTo(tempImage);
		findContours(tempImage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		vector<Rect> boundRect( contours.size() );
		vector<vector<Point> > contours_poly( contours.size() );
		vector<Point2f>center( contours.size() );
		vector<float>radius( contours.size() );

		if (!contours.empty()) {
			for (size_t i = 0; i < contours.size(); i++) {
				contourSize = contourArea(contours[i]);
				if (contourSize > contourSizeThreshold) {
					approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
					boundRect[i] = boundingRect( Mat(contours_poly[i]) );
					minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
       				smallBlobs.push_back(i);//#####
				}
			}

			// for (size_t i = 0; i < smallBlobs.size(); i++) {
			// 	Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
			// 	drawContours(cv_ptr->image, contours, smallBlobs[i], color, CV_FILLED, 8);
			// }
			for (size_t i = 0; i < contours.size(); i++) {
				double area = contourArea(contours[i]);
				if (area < 100) continue;

				double x = center[i].x;
				double y = center[i].y;

				int cols = cv_ptr->image.cols;
				int rows = cv_ptr->image.rows;

				string lr;
				string fb;

				if (x < int(cols * 0.33)) {
					lr = "FAR LEFT";
				} else if (x > int(cols * 0.66)) {
					lr = "FAR RIGHT";
				} else if (x < int(cols * 0.47)) {
					lr = "LEFT";
				} else if (x > int(cols * 0.53)) {
					lr = "RIGHT";
				} else {
					lr = "LR OK";
				}

				if (y < int(rows * 0.33)) {
					fb = "VERY FAR AWAY";
				} else if (y < int(rows * 0.5)) {
					fb = "FAR AWAY";
				} else if (y < int(rows * 0.66)) {
					fb = "NEAR";
				} else {
					fb = "VERY NEAR";
				}

				Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
				circle(cv_ptr->image, center[i], (int)radius[i], color, 2, 8, 0 );
				stringstream msg;
				msg << "NearCamera;Found;LEFT-RIGHT:" << lr 
					<< ";FRONT-BACK:" << fb
					<< ";X:" << x
					<< ";Y:" << y
					<< ";AREA:" << area
					<< ";I:" << i
					<< ";ROWS:" << cv_ptr->image.rows
					<< ";COLS:" << cv_ptr->image.cols;
				std_msgs::String message;
				message.data = msg.str();
				nearSampleFoundPub_.publish(message);
//				ROS_INFO("i: %d, area: %f, center: %f, %f, lr: %s, fb: %s", i, area, x, y, lr.c_str(), fb.c_str());
			}
		}

		imshow(OPENCV_WINDOW, cv_ptr->image); //show the original image
		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		cv::waitKey(1);
		//cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    }

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
}


FindObject::FindObject() : it_(nh_) {
	iLowH = 116;
	iHighH = 134;
	iLowS = 50;
	iHighS = 255;
	iLowV = 67;
	iHighV = 255;
	contourSizeThreshold = 200;


	f = boost::bind(&FindObject::configurationCallback, _1, _2);
	dynamicConfigurationServer.setCallback(f);

	ros::param::param<std::string>("image_topic_name", imageTopicName_, "/camera/rgb/image_color");
	ROS_INFO("PARAM image_topic_name: %s", imageTopicName_.c_str());
	image_sub_ = it_.subscribe(imageTopicName_.c_str(), 1, &FindObject::imageCb, this);
	nearSampleFoundPub_ = nh_.advertise<std_msgs::String>("nearSampleFound", 2);
    namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
	
	//Create trackbars in "Control" window
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	cvCreateTrackbar("contourSizeThreshold", "Control", &contourSizeThreshold, 10000);
}

const string FindObject::OPENCV_WINDOW = "Image window";

