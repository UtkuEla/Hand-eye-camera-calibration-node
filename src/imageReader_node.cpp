//============================================================================
// Name        : imageReader_node.cpp
// Author      : Utku Elagoz
// Version     :
// Copyright   : UtkuE-2022
// Description : Reading the images for testing the cameraCalibration in C++
//============================================================================

#include <iostream>
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using namespace std;

// creating publishers
ros::Publisher RGBpub;
ros::Publisher IRpub;

void RGBCallback(const sensor_msgs::ImageConstPtr& rgbmsg) {
  try {
    RGBpub.publish(rgbmsg);
    cout << "IR message published successfully" << endl;
  } catch (const std::exception&) {
    ROS_ERROR("Could not callback RGB");
  }
}

void IRCallback(const sensor_msgs::ImageConstPtr& irmsg) {
  try {
    IRpub.publish(irmsg);
    cout << "RGB message published successfully" << endl;
  } catch (const std::exception&) {
    ROS_ERROR("Could not callback IR");
  }
}


int main(int argc, char **argv) {

// example codes : http://wiki.ros.org/image_transport/Tutorials/PublishingImages
//  		   http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
//		   https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c++)

  ros::init(argc, argv, "imageReader_node");
  ros::NodeHandle nhimg;
  image_transport::ImageTransport it(nhimg);

  RGBpub = nhimg.advertise<sensor_msgs::Image>("calibration_rgb_img", 1);
  IRpub = nhimg.advertise<sensor_msgs::Image>("calibration_ir_img", 1);

  image_transport::Subscriber RGBsub = it.subscribe("k4a/rgb/image_raw", 1, RGBCallback);
  image_transport::Subscriber IRsub = it.subscribe("k4a/ir/image_raw", 1, IRCallback);

  while (ros::ok()) {
       ros::spinOnce();
     }

	return 0;
}
