//============================================================================
// Name        : calibrationUE_node.cpp
// Author      : utku
// Version     : 2 - cornerSubPix added
// Copyright   : UtkuE-2022
// Description : Camera calibration for RNM2022
//============================================================================


#include <ros/ros.h>
#include <cmath>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>		// std::stringstream
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp> // for the findChessboardCorners and cameraCalibration
#include <opencv2/imgproc.hpp> // for the cornerSubPix()


using namespace std;
using namespace cv;

// example and tutorial: https://learnopencv.com/camera-calibration-using-opencv/

// Introducing the necessary camera and checkherboard variables

float frames = 40; // number of frames to be used in the calibration

cv::Size RGBFrameSize(2000,1000); // dummy frame size for RGB images ===> another approach can be used cv::Size(gray.rows,gray.cols)
cv::Size IRFrameSize(800,600)    // dummy frame size for IR images

std::vector<std::vector<cv::Point2f>> RGBcorners;  // corners for rgb frames
std::vector<std::vector<cv::Point2f>> IRcorners;  // corners for ir frames

//Naming
int checkerboard[2] = {9, 6};        // Pattern of the given checkerboards
cv::Size patternSize(8, 5);  // Number of corners
float fieldSize = 0.040;


// counters for filenames
int RGB_pose = 1;
int IR_pose = 1;

// other counters for program termination
int rgb_count = 0;
int ir_count = 0;

static const std::string dataPath = "/home/calibration_data"; // the general calibration data path ???

/////////////////////////////////////////////////////

// Two callback functions are needed. IRImageCallBack and RGBImageCallBack are transports the images ...
// and changes them to bgr8 format. Also, functions are saving the images to a defined data file.
// converts the ROS image message to an OpenCV image with BGR pixel encoding

// Example code :
// https://github.com/tzutalin/ros_sample_image_transport/blob/master/src/image_transport_subscriber.cpp
// http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages

stringstream ss; // Creates a stringstream object, closer functionality to StringIO in Python.

void RGBImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    if(rgb_count < frames) {

    	// transporting the RGB images from msg to bgr8 --- bgr8 is a pixel format!
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

      // File name for RGB images
      string extension = "/images/rgb/pose_";
      string type = ".jpg";
      ss << dataPath << extension << (RGB_pose) << type;
      string fileName = ss.str(); //ss.str(""); check!!

      // Saving the images to a specified file. https://docs.opencv.org/3.4/d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce

      cv::imwrite(fileName, img);

      std::cout << "RGB: pose_" << RGB_pose << " written." << endl;

      RGB_pose++;
      rgb_count++;
    }
  }
  catch (cv_bridge::Exception &e) {
    std::cout << "RGBImageCallBack does not work for: pose_" << RGB_pose-1 << " image." << endl;
  }
}


void IRImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    while(ir_count < frames) {

      // transporting the IR images from msg to bgr8 --- bgr8 is a pixel format!
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

      // File name for IR images
      string extension1 = "/images/ir/pose_";
      string type = ".jpg";
      ss << dataPath << extension1 << (IR_pose) << type;
      string fileName1 = ss.str();  // This function changes stringstream to string
      //ss.str(""); // check!!

      // Saving the images to a specified file. https://docs.opencv.org/3.4/d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce
      cv::imwrite(fileName1, img);

      std::cout << "ir: Pose_" << IR_pose << " written." << endl;


      IR_pose++;
      ir_count++;
    }
  }
  catch (cv_bridge::Exception &e) {
    std::cout << "IRImageCallBack does not work for: pose_" << IR_pose-1 << endl;
  }
}


cv::Mat RGBcameraMatrix;  // calibration Matrix for RGB intrinsics
cv::Mat RGBdistCo = (Mat1d(1, 8));
cv::Mat IRcameraMatrix;  // calibration Matrix for IR intrinsics
cv::Mat IRdistCo = (Mat1d(1, 8));


cv::Mat K1, K2, R, F, E; // for stereoCalibration
cv::Vec3d T;
cv::Mat D1, D2;

float RGBCalibration;
float IRCalibration;
double Extrinsic; // for stereoCalibration

// RGBCameraCalibration function will calibrate the intrinsics of the RGB Camera
void RGBCameraCalibration() { // should it be int?

	std::vector<cv::String> RGBFileNames(frames);
	//std::vector<std::vector<cv::Point2f>> RGBcorners;  // I put it outside of the method to be used in the streoCalibration
	std::vector<std::vector<cv::Point3f>> RGBObjP;  // Checkerboard world coordinates
	std::vector<cv::Point3f> objpRGB; //dummy to be used in defining the world coordinates for 3D points

	std::string type = ".jpg" ;

	for(int i=0; i< frames; i++){
		RGBFileNames[i] = dataPath + "/imgs/rgb/pose_" + to_string(i + 1) + type ;
	}


	  for (int i = 1; i < checkerboard[1]; i++) {
	    for (int j = 1; j < checkerboard[0]; j++) {
	      objpRGB.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
	    }
	  }


	cv::Mat rgbimg;

	for(int i=0; i<RGBFileNames.size(); i++ ){
	    string f = RGBFileNames[i];
	    rgbimg = cv::imread(f); // Load the images
	    cv::Mat rgbgray; // grayscale the image
	    cv::cvtColor(rgbimg, rgbgray, cv::COLOR_RGB2GRAY);

	    // documentation for findChessboardCorners, must be bool
	    // https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a

	    std::vector<Point2f> corners;
	    bool rgbPattern = cv::findChessboardCorners(rgbgray, patternSize, corners,
		                                                     cv::CALIB_CB_ADAPTIVE_THRESH
		                                                     + cv::CALIB_CB_NORMALIZE_IMAGE
		                                                     + cv::CALIB_CB_FILTER_QUADS
								       + cv::CALIB_CB_FAST_CHECK);

		    // Use cv::cornerSubPix() to refine the found corner detections with values given by opencv
	    	    // https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e

	    if(rgbPattern == true) {
	    cv::cornerSubPix( rgbgray, corners, cv::Size(16, 16), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001));

	    RGBcorners.push_back(corners);
	    RGBObjP.push_back(objpRGB);
	    }

	}


	std::vector<cv::Mat> rvecs, tvecs; // rotation and translation vectors

	// cv flags for calibrateCamera
	int flags = CALIB_RATIONAL_MODEL;

	RGBCalibration = cv::calibrateCamera(RGBObjP, RGBcorners, RGBFrameSize, RGBcameraMatrix, RGBdistCo, rvecs, tvecs, flags);

	std::cout << "Error = " << RGBCalibration << "\nCamera Matrix= \n" << RGBcameraMatrix << "\nDistance Coeff= \n" << RGBdistCo << std::endl;

}

// IRCameraCalibration function will calibrate the intrinsics of the IR Camera
void IRCameraCalibration() { // should it be int?

	std::vector<cv::String> IRFileNames(frames);
	std::vector<std::vector<cv::Point3f>> IRObjP;  // Checkerboard world coordinates
	std::vector<cv::Point3f> objpIR; //dummy to be used in defining the world coordinates for 3D points
	//std::vector<std::vector<cv::Point2f>> IRcorners; // I put it outside of the method to be used in the streoCalibration


	std::string type = ".jpg" ;

    for (int i = 0; i < frames; i++) {
      IRFileNames[i] = dataPath + "/images/ir/pose_" + to_string(i + 1) + type;
    }


// Define the world coordinates for 3D points of the ir frame

    for (int i = 1; i < checkerboard[1]; i++) {
      for (int j = 1; j < checkerboard[0]; j++) {
        objpIR.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
      }
    }


    cv::Mat irimg;

    for (int i = 0; i < IRFileNames.size(); i++) {
      string g = IRFileNames[i];
      vector<Point2f> corners2;

      irimg = cv::imread(g);  // Load the images
      cv::Mat irgray;         // grayscale the image
      cv::cvtColor(irimg, irgray, cv::COLOR_RGB2GRAY);


      bool IRpattern = cv::findChessboardCorners(irgray, patternSize, corners2, cv::CALIB_CB_ADAPTIVE_THRESH
				   						  + cv::CALIB_CB_NORMALIZE_IMAGE
				   						  + cv::CALIB_CB_FILTER_QUADS
								                 + cv::CALIB_CB_FAST_CHECK);
    
    if(IRpattern == true){
    	cv::cornerSubPix( irgray, corners2, cv::Size(16, 16), cv::Size(-1, -1),
    	            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001)); // try Size(18,18)


    	IRcorners.push_back(corners2);
    	IRObjP.push_back(objpIR);
    }
}
    std::vector<cv::Mat> irrvecs, irtvecs; // rotation and translation vectors
    int irflags = cv::CALIB_FIX_S1_S2_S3_S4 + cv::CALIB_FIX_TAUX_TAUY + cv::CALIB_RATIONAL_MODEL; 

    IRCalibration = cv::calibrateCamera(IRObjP, IRcorners, IRFrameSize, IRcameraMatrix, IRdistCo, irrvecs, irtvecs, irflags);

    std::cout << "Error= " << IRCalibration << "\nCamera Matrix =\n" << IRcameraMatrix << "\nDistance Coeff=\n" << IRdistCo << std::endl;


}

//StereoCameraCalibration function is for calibrating the extrinsic parameters of the camera
void stereoCameraCalibration(){

	//https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga91018d80e2a93ade37539f01e6f07de5

    vector<vector<Point3f>> objectPoints;


    K1 = RGBcameraMatrix;
    K2 = IRcameraMatrix;
    D1 = RGBdistCo;
    D2 = IRdistCo;

    /*
	R	Output rotation matrix. Together with the translation vector T, this matrix brings points given in the first camera's coordinate system to points in the second camera's coordinate system.
		In more technical terms, the tuple of R and T performs a change of basis from the first camera's coordinate system to the second camera's coordinate system.
		Due to its duality, this tuple is equivalent to the position of the first camera with respect to the second camera coordinate system.
	T	Output translation vector, see description above.
	E	Output essential matrix.
	F	Output fundamental matrix.
     */

    int stereoFlags = CALIB_FIX_INTRINSIC + CALIB_FIX_S1_S2_S3_S4 + CALIB_FIX_TAUX_TAUY + CALIB_RATIONAL_MODEL;


    Extrinsic = stereoCalibrate(objectPoints, RGBcorners, IRcorners, K1, D1, K2, D2, RGBFrameSize, R, T, E, F, stereoFlags);


    // Stereo Rectification
    //https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6


    cv::Mat R1, R2, P1, P2, Q;
    
    cv::stereoRectify(K1, D1, K2, D2, RGBFrameSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY);

}



int main(int argc, char** argv) {

	ros::init(argc, argv, "calibrationUE_node");
	ros::NodeHandle nh;

	//creating an ImageTransport instance, initialising it with our NodeHandle.
	image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub = it.subscribe("/calibration_rgb_img", 1, RGBImageCallback);
    image_transport::Subscriber ir_sub = it.subscribe("/calibration_ir_img", 1, IRImageCallback);

    while (rgb_count < frames || ir_count < frames || ros::ok()) { // I don't know if it is a safe approach, it can be stuck in this loop
    
    /*
     * ros::spin() will not return until the node has been shut down. Thus, you have no control over your program while ROS is spinning.
     * The advantage of ros::spinOnce() is that it will check for callbacks/service calls/etc only as often as you call it.
     * Why is this useful? Being able to control the rate at which ROS updates allows you to do certain calculations at a certain frequency.
     */

      ros::spinOnce();
    }

    RGBCameraCalibration();
    IRCameraCalibration();
    stereoCameraCalibration();

	return 0;
}
