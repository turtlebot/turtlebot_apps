/*
 * opencv_camera_driver.cpp
 *
 *  Created on: Oct 26, 2010
 *      Author: erublee - originally from mihelich
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/CvBridge.h>

#include "pano_core/Camera.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencv_camera");

  std::string camera_file = "";
  if (argc != 2)
  {
    ROS_ERROR("usage: <camera.yml>");
    return 1;
  }
  camera_file = argv[1];
  pano::Camera camera;
  camera.setCameraIntrinsics(camera_file);

  sensor_msgs::CameraInfo cam_info;
  for (int i = 0; i < 9; i++)
  {
    cam_info.K[i] = camera.K().at<float> (i / 3, i % 3);
  }
  cam_info.width = camera.img_size().width;
  cam_info.height = camera.img_size().height;

  cv::VideoCapture capture(0);
  if (!capture.isOpened())
  {
    ROS_FATAL("Camera could not be opened");
    return 1;
  }
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher streaming_pub;
  streaming_pub = it.advertiseCamera("image_raw", 1);

  sensor_msgs::CameraInfoConstPtr cam_info_ptr(new sensor_msgs::CameraInfo(cam_info));
  cv::Mat frame;
  while (ros::ok())
  {
    capture >> frame;

    IplImage ipl = frame;
    sensor_msgs::ImageConstPtr img = sensor_msgs::CvBridge::cvToImgMsg(&ipl, "bgr8");

    // streaming_pub.publish
    streaming_pub.publish(img, cam_info_ptr);

    ros::spinOnce();
  }
}
