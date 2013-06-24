/*
 * Extrinsics.cpp
 *
 *  Created on: Oct 20, 2010
 *      Author: erublee
 */

#include "pano_core/Extrinsics.h"
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
namespace pano
{
Extrinsics::Extrinsics() :
  mats_(N_STD_MATS), vals_(N_STD_VALS), flags_(N_STD_FLAGS)
{

}

Extrinsics::~Extrinsics()
{

}
Extrinsics::Extrinsics(const std::vector<Mat>& mats, const std::vector<double>& vals, const std::vector<int>& flags) :
  mats_(mats), vals_(vals), flags_(flags)
{

}
Extrinsics::Extrinsics(const cv::Mat& R, const cv::Mat& T, double latitude, double longitude, double gps_accuracy, double confidence, double gps_time,
                       bool estimated) :
  mats_(N_STD_MATS), vals_(N_STD_VALS), flags_(N_STD_FLAGS)
{
  mat(Extrinsics::R) = R;
  mat(Extrinsics::T) = T;
  val(LATITUDE) = latitude;
  val(LONGITUDE) = longitude;
  val(GPS_ACCURACY) = gps_accuracy;
  val(CONFIDENCE) = confidence;
  val(GPS_TIME) = gps_time;
  flag(ESTIMATED) = estimated;
}
Extrinsics::Extrinsics(const cv::Mat& R, double confidence) :
  mats_(N_STD_MATS), vals_(N_STD_VALS), flags_(N_STD_FLAGS)
{
  mat(Extrinsics::R) = R;

  mat(Extrinsics::T) = Mat();
  val(LATITUDE) = 0;
  val(LONGITUDE) = 0;
  val(GPS_ACCURACY) = 0;
  val(CONFIDENCE) = confidence;
  flag(ESTIMATED) = false;

}
Extrinsics::Extrinsics(const Extrinsics& rhs) :
  vals_(rhs.vals_), flags_(rhs.flags_)
{
  copyData(rhs);
}
Extrinsics& Extrinsics::operator=(const Extrinsics& rhs)
{

  if (&rhs != this)
  {
    vals_ = rhs.vals_;
    flags_ = rhs.flags_;
    copyData(rhs);
  }
  return *this;
}
void Extrinsics::copyData(const Extrinsics& rhs)
{
  mats_.resize(rhs.mats_.size());
  for (size_t i = 0; i < mats_.size(); ++i)
  {
    if (!rhs.mats_[i].empty())
    {
      rhs.mats_[i].copyTo(mats_[i]);
    }
    else
      mats_[i] = Mat();
  }
}
void Extrinsics::serialize(cv::FileStorage& fs) const
{
  fs << "{";
  cvWriteComment(*fs, "Extrinsics class", 0);
  fs << "mats" << "[";
  for (size_t i = 0; i < mats_.size(); ++i)
  {
    if (mats_[i].empty())
    {
      fs << Mat::zeros(3, 1, CV_32FC1);
    }
    else
      fs << mats_[i];
  }
  fs << "]";

  fs << "vals" << "[";
  for (size_t i = 0; i < vals_.size(); ++i)
  {
    fs << vals_[i];
  }
  fs << "]";

  fs << "flags" << "[";
  for (size_t i = 0; i < flags_.size(); ++i)
  {
    fs << (int)flags_[i];
  }
  fs << "]";

  fs << "}";
}
void Extrinsics::deserialize(const cv::FileNode& fn)
{

  FileNode mats = fn["mats"];
  CV_Assert(mats.type() == FileNode::SEQ);
  mats_.resize(mats.size());
  for (size_t i = 0; i < mats.size(); i++)
  {
    try
    {
      cv::read(mats[i], mats_[i], Mat());
    }
    catch (Exception exc)
    {
      if (exc.code == -201)
      {
        mats_[i] = Mat();
      }
      else
        throw exc;
    }
  }

  FileNode vals = fn["vals"];
  CV_Assert(vals.type() == FileNode::SEQ);
  vals_.resize(vals.size());
  for (size_t i = 0; i < vals.size(); i++)
  {
    vals_[i] = (double)vals[i];
  }

  FileNode flags = fn["flags"];
  CV_Assert(flags.type() == FileNode::SEQ);
  flags_.resize(flags.size());
  for (size_t i = 0; i < flags.size(); i++)
  {
    flags_[i] = (int)flags[i];
  }
}

}
