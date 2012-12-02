#include "pano_core/Projector.h"
#include "pano_core/Camera.h"
#include <cmath>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>

using namespace cv;

namespace pano
{

static const float PI = 3.14159265f;
/**** static functions *********/

void Projector::getSphereRMap(const cv::Mat& K, const cv::Mat& R, cv::Mat& remap1, cv::Mat&remap2,
                              const cv::Mat& spherical_points, const cv::Size& output_size)
{
  static std::vector<cv::Mat> working_mats;
  getSphereRMap(K, R, remap1, remap2, spherical_points, output_size, working_mats);

}

void Projector::getSphereGMap(const cv::Mat& K, const cv::Mat& G, cv::Mat& remap1, cv::Mat&remap2,
                              const cv::Mat& spherical_points, const cv::Size& output_size)
{
  static std::vector<cv::Mat> working_mats;
  getSphereGMap(K, G, remap1, remap2, spherical_points, output_size, working_mats);
}
void Projector::getSphereRMapMask(const cv::Mat& _K, const cv::Mat& R, cv::Mat& map, cv::Mat& _mask,
                                  const cv::Mat& spherical_coords, cv::Mat& tm)
{

  if (map.size() != spherical_coords.size())
  {
    map = Mat(spherical_coords.size(), cv::DataType<cv::Point2f>::type);
  }

  Mat& mask = _mask;
  mask = Mat_<unsigned char>::zeros(map.size());

  cv::Mat K;
  _K.convertTo(K, CV_32F);
  float fx, fy, cx, cy;

  fx = K.at<float> (0, 0);
  fy = K.at<float> (1, 1);

  cx = K.at<float> (0, 2);
  cy = K.at<float> (1, 2);

  float fovx, fovy;
  Camera::KtoFOV(K, fovx, fovy);

  Rect valid_rect(0, 0, 2 * cx, 2 * cy);

  // float cosfov = cos(KtoFOVmax(K));

  //rotate our unit sphere of points by TM.
  //so the imput image is "rotated" on the sphere.
  cv::transform(spherical_coords, tm, R);

  //this is the center pixel of image.
  // cv::Point3d z(0, 0, 1);

  //iterate over the entire output image
  //and calculate the mapping
  for (int y = 0; y < spherical_coords.rows; y++)
  {
    for (int x = 0; x < spherical_coords.cols; x++)
    {

      cv::Point3f & point = tm.at<cv::Point3f> (y, x);

      cv::Point2f & mappoint = map.at<cv::Point2f> (y, x);

      Point2f imgpt;
      imgpt.x = fx * point.x / point.z + cx;
      imgpt.y = fy * point.y / point.z + cy;
      mask.at<unsigned char> (y, x) = valid_rect.contains(Point(imgpt.x, imgpt.y)) && point.z > 0;
      //only map points that are valid pixel coords
      if (mask.at<unsigned char> (y, x))
      {
        //project the rotated sphere points into impute image coordinates
        //using the camera intrinsic parameters
        //so the center pixel of the input image is given by
        //a 3d vector of (0,0,1)
        //
        mappoint = imgpt;
      }
      else
      {

        int xs = point.x <= 0.01 ? -1 : 1;
        int ys = point.y <= 0.01 ? -1 : 1;
        mappoint.x = 1.0e4 * xs;
        mappoint.y = 1.0e4 * ys;
      }

    }
  }

}
void Projector::getSphereRMap(const cv::Mat& _K, const cv::Mat& R, cv::Mat& remap1, cv::Mat&remap2,
                              const cv::Mat&spherical_coords, const cv::Size& output_size,
                              std::vector<cv::Mat>& working_mats)
{
  if (working_mats.size() != 4)
  {
    working_mats.clear();
    working_mats.resize(4);
  }
  Mat &tm = working_mats[0];
  Mat &map = working_mats[1];
  Mat &scaled = working_mats[2];
  Mat &mask = working_mats[3];

  getSphereRMapMask(_K, R, map, mask, spherical_coords, tm);
  float _scaling_matrix[] = {output_size.width / (float)map.size().width, 0, 0, 0, output_size.height
      / (float)map.size().height, 0};
  warpAffine(map, scaled, Mat(2, 3, CV_32F, _scaling_matrix), output_size, INTER_LINEAR, BORDER_WRAP);
  convertMaps(scaled, Mat(), remap1, remap2, CV_16SC2);
}

void Projector::getSphereGMap(const cv::Mat& _K, const cv::Mat& G, cv::Mat& remap1, cv::Mat&remap2,
                              const cv::Mat & homog_sphr_coords, const cv::Size& output_size,
                              std::vector<cv::Mat>& working_mats)
{
  if (working_mats.size() != 3)
  {
    working_mats.clear();
    working_mats.resize(3);
  }
  Mat &tm = working_mats[0];
  Mat &map = working_mats[1];
  if (map.size() != homog_sphr_coords.size())
  {
    map = Mat(homog_sphr_coords.size(), cv::DataType<cv::Point2f>::type);
  }
  if (G.cols != 4 || G.rows != 4)
  {
    throw cv::Exception(1, "Invalid G! Must be 4x4!", "Projector.cpp", "", 0);
  }
  map = Scalar(INFINITY, INFINITY, INFINITY, INFINITY);

  Mat &scaled = working_mats[2];
  cv::Mat K;
  _K.convertTo(K, CV_32F);
  float fx, fy, cx, cy;

  fx = K.at<float> (0, 0);
  fy = K.at<float> (1, 1);
  cx = K.at<float> (0, 2);
  cy = K.at<float> (1, 2);

  cv::transform(homog_sphr_coords, tm, G);

  float lambda;
  for (int y = 0; y < homog_sphr_coords.rows; y++)
  {
    for (int x = 0; x < homog_sphr_coords.cols; x++)
    {
      cv::Vec4f & point = tm.at<cv::Vec4f> (y, x);
      cv::Point2f & mappoint = map.at<cv::Point2f> (y, x);
      lambda = 1e-1;
      if (point[2] < lambda)
      {
        int xs = point[0] <= 0.01 ? -1 : 1;
        int ys = point[1] <= 0.01 ? -1 : 1;
        mappoint.x = 1.0e4 * xs;
        mappoint.y = 1.0e4 * ys;
        continue;
      }

      mappoint.x = fx * point[0] / point[2] + cx;
      mappoint.y = fy * point[1] / point[2] + cy;

    }
  }

  float _scaling_matrix[] = {output_size.width / (float)map.size().width, 0, 0, 0, output_size.height
      / (float)map.size().height, 0};
  warpAffine(map, scaled, Mat(2, 3, CV_32F, _scaling_matrix), output_size, INTER_LINEAR, BORDER_WRAP);

  convertMaps(scaled, Mat(), remap1, remap2, CV_16SC2);
}

cv::Mat createHomogSphrCoords(const cv::Size& sphere_size, float theta_range = 2 * CV_PI, float phi_range = CV_PI)
{

  Mat _homog_sphr_coords = Mat(sphere_size, cv::DataType<cv::Vec4f>::type);
  for (int i = 0; i < _homog_sphr_coords.rows; i++)
  {

    float phi = (i - _homog_sphr_coords.rows / 2.0f) * (phi_range) / _homog_sphr_coords.rows;
    float sinphi = std::sin(phi);
    float cosphi = std::cos(phi);
    for (int j = 0; j < _homog_sphr_coords.cols; j++)
    {

      float theta = (j - _homog_sphr_coords.cols / 2.0f) * (theta_range) / _homog_sphr_coords.cols;
      float sintheta = std::sin(theta);
      float costheta = std::cos(theta);

      cv::Point3f point = cv::Point3f(sintheta * cosphi, sinphi, costheta * cosphi);

      cv::Vec4f homog_pt(point.x, point.y, point.z, 1.0);
      _homog_sphr_coords.at<cv::Vec4f> (i, j) = homog_pt;

    }
  }
  return _homog_sphr_coords;

}

cv::Mat Projector::createSphericalCoords(const cv::Size& sphere_size, float theta_range, float phi_range)
{

  Mat sc;
  createSphericalCoords(sphere_size, -theta_range / 2, theta_range / 2, -phi_range / 2, phi_range / 2, sc);
  return sc;
}

void Projector::createSphericalCoords(const cv::Size& sphere_size, float theta_0, float theta_1, float phi_0,
                                      float phi_1, Mat& spherical_coords)
{

  spherical_coords.create(sphere_size, cv::DataType<cv::Point3f>::type);
  //this creates a matrix of 3d points for the entire output image, all of these points lie on
  //a unit sphere.
  // [theta, phi] = meshgrid( linspace(-pi,pi,1000),linspace(-pi/2,pi/2,500) );
  // rho          = 1 + 0*theta;
  // [x,y,z]      = sphr2cart( theta(:),phi(:),rho(:));
  // ** Subtle Side Effect ** : 
  // theta=0 is meant to be north, but corresponds to x-axis
  //      or "East" when drawing a polar diagram. 

  float phi_range   = (phi_1 - phi_0);
  float theta_range = (theta_1 - theta_0);
  float phi_step    = (phi_range) / spherical_coords.rows;


  float theta_step = (theta_range) / spherical_coords.cols;
  for (int y = 0; y < spherical_coords.rows; y++)
  {

    float phi = phi_0 + y * phi_step;

    float sinphi  = std::sin(phi);

    float cosphi = std::cos(phi);
    for (int x = 0; x < spherical_coords.cols; x++)
    {
      float theta = theta_0 + x * theta_step;
      float sintheta = std::sin(theta);
      float costheta = std::cos(theta);

      cv::Point3f point = cv::Point3f(sintheta * cosphi, sinphi, costheta * cosphi);
      spherical_coords.at<cv::Point3f> (y, x) = point;
    }
  }
}

void Projector::getSphereMask(const cv::Mat& onezies, const cv::Mat& remap1, const cv::Mat&remap2, cv::Mat& mask)
{
  if (mask.size() != remap1.size() || mask.type() != onezies.type())
  {

    mask = Mat::zeros(remap1.size(), onezies.type());
  }
  cv::remap(onezies, mask, remap1, remap2, cv::INTER_AREA, cv::BORDER_TRANSPARENT);
}
void Projector::projectImage(const cv::Mat& image, const cv::Mat& remap1, const cv::Mat&remap2, cv::Mat & outputimage,
                             int filltype, const Scalar& value)
{
  if (outputimage.size() != remap1.size() || outputimage.type() != image.type())
  {

    outputimage = Mat::zeros(remap1.size(), image.type());
  }

  int interp_mode = cv::INTER_LINEAR; // cv::INTER_AREA; Better ?!

  cv::remap(image, outputimage, remap1, remap2, interp_mode, filltype, value);

}

Projector::Projector(const cv::Size& output_size, float theta_range, float phi_range) :
  outimage_size(output_size), spherical_coords(createSphericalCoords(Size(output_size.width / 10.0f, output_size.height
      / 10.0f), theta_range, phi_range))
{

}

Projector::~Projector(void)
{
}

namespace
{
bool checksize(const cv::Size& lhs, const cv::Size& rhs)
{
  return lhs.width == rhs.width && lhs.height == rhs.height;
}
}

/** \brief Sets up the projector to remap based on the given R
 * and K.  Call this before calling any of the member project functions
 */
void Projector::setSRandK(const cv::Size& inputsz, const cv::Mat& R, const cv::Mat& _K)
{
  input_image_sz = inputsz;
  K = _K.clone();
  getSphereRMap(K, R, remap1, remap2, spherical_coords, outimage_size, working_mats);

}

void Projector::projectMat(const cv::Mat& image, cv::Mat& outimage, int filltype, const Scalar& value)
{

  projectImage(image, remap1, remap2, outimage, filltype, value);

}
namespace
{
std::vector<cv::Rect> GenRois(cv::Size output_size, cv::Size grids)
{
  std::vector<cv::Rect> rois(grids.area());
  float width = output_size.width / float(grids.width);
  float height = output_size.height / float(grids.height);
  for (int x = 0; x < grids.width; x++)
  {
    for (int y = 0; y < grids.height; y++)
    {
      Point2f imgpt(x * width, y * height);
      Rect& roi = rois[x + y * grids.width];
      roi = cv::Rect(imgpt.x, imgpt.y, width, height);
    }
  }
  return rois;
}
}
SparseProjector::SparseProjector(const cv::Size& output_size, const cv::Size& n_grids) :
  output_size_(output_size),spherical_coords_small_(Projector::createSphericalCoords(Size(100,50),2 * CV_PI,CV_PI)), grids_(n_grids), rois_(GenRois(output_size_, grids_)),
      small_rois_(GenRois(spherical_coords_small_.size(), grids_)), workingid_(0)
{

}

/** \brief Sets up the projector to remap based on the given R
 * and K.  Call this before calling any of the member project functions
 */
void SparseProjector::setSRandK(const cv::Size& inputsz, const cv::Mat& R, const cv::Mat& K, std::vector<int>& roi_ids)
{
  Projector::getSphereRMapMask(K, R, remap_, mask_, spherical_coords_small_, tm_);
  roi_ids.clear();
  for (int i = 0; i < int(rois_.size()); i++)
  {
    const Rect& roi = small_rois_[i];
    Mat m = mask_(roi);
    if (countNonZero(m))
    {
      roi_ids.push_back(i);
    }
  }

  K_ = K;
  R_ = R;
  workingid_ = -1;
}

void SparseProjector::setWorkingRoi(int id)
{
  if (workingid_ == id)
    return;
  workingid_ = id;

  Rect roi = small_rois_[id];
  Rect broi = rois_[id];
  float theta_0, theta_1, phi_0, phi_1;
  float theta_step = (2 * CV_PI) / output_size_.width;
  float phi_step = CV_PI / output_size_.height;

  theta_0 = (broi.x - output_size_.width / float(2)) * theta_step;
  theta_1 = theta_0 + broi.width * theta_step;

  phi_0 = (broi.y - output_size_.height / float(2)) * phi_step;// - broi.height * phi_step/2;
  phi_1 = phi_0 + broi.height * phi_step;

  Projector::createSphericalCoords(broi.size(), theta_0, theta_1, phi_0, phi_1, spherical_coords_);

  Mat map, mask, tm;
  Projector::getSphereRMapMask(K_, R_, map, mask, spherical_coords_, tm);
  convertMaps(map, Mat(), remap1_, remap2_, CV_16SC2);
}

void SparseProjector::projectMat(int roi_id, const cv::Mat& m, cv::Mat& outimage, int filltype, const cv::Scalar& value)
{
  setWorkingRoi(roi_id);
  Projector::projectImage(m, remap1_, remap2_, outimage, filltype, value);
}

}
