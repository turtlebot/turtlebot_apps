/*
 * Camera.cpp
 *
 *  Created on: Oct 17, 2010
 *      Author: ethan
 */

#include "pano_core/Camera.h"
#include <pano_core/ModelFitter.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace cv;

namespace pano
{
namespace
{

bool readKfromCalib(cv::Mat& K, cv::Mat& distortion, cv::Size & img_size, const std::string& calibfile)
{
  cv::FileStorage fs(calibfile, cv::FileStorage::READ);
  cv::Mat cameramat;
  cv::Mat cameradistortion;
  float width = 0, height = 0;
  if (fs.isOpened())
  {
    cv::read(fs["camera_matrix"], cameramat, cv::Mat());
    cv::read(fs["distortion_coefficients"], cameradistortion, cv::Mat());
    cv::read(fs["image_width"], width, 0);
    cv::read(fs["image_height"], height, 0);

    fs.release();

  }
  else
  {
    return false;
  }

  cv::Size _size(width, height);
  img_size = _size;

  K = cameramat;

  distortion = cameradistortion;
  return true;
}

}


Camera::Camera() :
  K_(Mat::eye(3, 3, CV_32F))
{

}

Camera::Camera(const std::string& camera_file) :
  K_(Mat::eye(3, 3, CV_32F))
{
  setCameraIntrinsics(camera_file);
}
Camera::~Camera()
{

}
void Camera::initUndistort(){
  if(P_.empty() && !D_.empty()){
    P_ = getOptimalNewCameraMatrix(K_, D_, img_size_, 0);
    initUndistortRectifyMap(K_,D_,Mat(),P_,img_size_,CV_16SC2,u_rm1_,u_rm2_);
  }
}

void Camera::undistort(const cv::Mat&image ,cv::Mat& uimage) const{
 if(!P_.empty()){
  cv::remap(image,uimage,u_rm1_,u_rm2_,CV_INTER_LINEAR);
 }else
   uimage = image;
}
void Camera::ptsToRays(const std::vector<cv::Point2f>& pts, std::vector<cv::Point3f>& rays) const
{
  rays.resize(pts.size());
  points2fto3f(pts.begin(), pts.end(), rays.begin(), Pinv_.empty() ? Kinv_ : Pinv_);
 // proje
}
void Camera::raysToPts(const std::vector<cv::Point3f>& rays, std::vector<cv::Point2f>& pts) const
{
  pts.resize(rays.size());
  points3fto2f(rays.begin(), rays.end(), pts.begin(), P_.empty() ? K_ : P_);
  //projectPoints()
}
namespace
{
void force2float(cv::Mat & m)
{
  if (m.empty())
    return;
  Mat t;
  m.convertTo(t, CV_32FC1);
  m = t;
}
}
void Camera::setupK()
{
  force2float(K_);
  force2float(D_);

  Camera::KtoFOV(K_, fov_x_, fov_y_);
  Kinv_ = K_.inv();
}
void Camera::setCameraIntrinsics(const std::string& filename)
{
  if (!readKfromCalib(K_, D_, img_size_, filename))
  {
    std::cerr << "Bad read on the Calibration File! : " << filename << std::endl;
    throw std::runtime_error("bad calibration file : " + filename);
  }
  setupK();
}
void Camera::setCameraIntrinsics(const cv::Mat& K, const cv::Mat& D, const cv::Size& img_size)
{
  K_ = K;
  D_ = D;

  img_size_ = img_size;

  setupK();
}

void Camera::scale(float x,float y){
  K_ = K_.clone();
  K_.at<float>(0,0) *=x;
  K_.at<float>(1,1) *=y;
  K_.at<float>(0,2) *=x;
  K_.at<float>(1,2) *=y;
  img_size_.width *= x;
  img_size_.height *=y;
  setupK();
}



void Camera::write(const std::string& file_name) const{
	FileStorage fs(file_name,FileStorage::WRITE);
	fs << "camera";
	serialize(fs);
}
void Camera::serialize(cv::FileStorage& fs) const
{
  fs << "{";
  cvWriteComment(*fs,"Camera class" , 0);
  fs << "K" << K_;
  fs << "Kinv" << Kinv_;
  if(!D_.empty())
    fs << "D" << D_;
  fs << "width" << img_size_.width;
  fs << "height" << img_size_.height;
  fs << "}";
}
void Camera::deserialize(const cv::FileNode& fn)
{
  fn["K"] >> K_;
  fn["Kinv"] >> Kinv_;
  if(!fn["D"].empty())
    fn["D"] >> D_;
  img_size_.width = (int)fn["width"] ;
  img_size_.height = (int)fn["height"] ;
  setupK();
}

void Camera::KtoFOV(const cv::Mat& K, float & fovx, float & fovy)
  {
    int K_type = K.type();
    CV_Assert( K_type == CV_32FC1 || K_type == CV_64FC1 )
      ;

    switch (K_type)
    {
      case CV_32FC1:
        fovx = 2.0f * std::atan(K.at<float> (0, 2) / K.at<float> (0, 0));
        fovy = 2.0f * std::atan(K.at<float> (1, 2) / K.at<float> (1, 1));
        break;
      case CV_64FC1:
        fovx = 2.0f * std::atan(K.at<double> (0, 2) / K.at<double> (0, 0));
        fovy = 2.0f * std::atan(K.at<double> (1, 2) / K.at<double> (1, 1));
        break;
      default:
        break;
    }
  }

}
