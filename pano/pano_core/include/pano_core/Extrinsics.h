/*
 * Extrinsics.h
 *
 *  Created on: Oct 20, 2010
 *      Author: erublee
 */

#ifndef EXTRINSICS_H_
#define EXTRINSICS_H_

#include "pano_core/pano_interfaces.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
namespace pano
{
class Extrinsics : public serializable
{
public:
  Extrinsics();
  Extrinsics(const Extrinsics& rhs);
  Extrinsics& operator=(const Extrinsics& rhs);
  Extrinsics(const std::vector<cv::Mat>& mats, const std::vector<double>& vals, const std::vector<int>& flags);
  Extrinsics(const cv::Mat& R, const cv::Mat& T, double latitude, double longitude, double gps_accuracy, double confidence, double gps_time,
             bool estimated);
  Extrinsics(const cv::Mat& R, double confidence);
  ~Extrinsics();

  virtual int version() const
  {
    return 0;
  }
  virtual void serialize(cv::FileStorage& fs) const;
  virtual void deserialize(const cv::FileNode& fn);

  const cv::Mat& mat(int idx) const
  {
    return mats_[idx];
  }

  cv::Mat relativeToOther(const Extrinsics& ext, int idx) const
  {
    return  mats_[idx] * ext.mats_[idx].t();
  }

  double val(int idx) const
  {
    return vals_[idx];
  }
  bool flag(int idx) const
  {
    return flags_[idx];
  }

  cv::Mat& mat(int idx)
  {
    return mats_[idx];
  }

  double& val(int idx)
  {
    return vals_[idx];
  }
  int& flag(int idx)
  {
    return flags_[idx];
  }

  enum StdMats
  {
    R = 0, T, W, N_STD_MATS
  };
  enum StdVals
  {
    LATITUDE = 0, LONGITUDE, GPS_ACCURACY, CONFIDENCE, GPS_TIME, N_STD_VALS
  };

  enum StdFlags
  {
    ESTIMATED = 0, N_STD_FLAGS
  };

private:
  void copyData(const Extrinsics& rhs);
  std::vector<cv::Mat> mats_;
  std::vector<double> vals_;
  std::vector<int> flags_;
};

inline float angularDist(const Extrinsics& ext1, const Extrinsics& ext2)
{
  cv::Mat R12 = ext1.relativeToOther(ext2, Extrinsics::R);
  cv::Mat v;
  cv::Rodrigues(R12, v);
  return norm(v);

}

inline cv::Mat skewsym(const cv::Mat& x)
{
  // W = skewsym( w )
  cv::Mat_<float> m(3,3);
  m(0,1) = -(m(1,0) = x.at<float>(2));
  m(1,2) = -(m(2,1) = x.at<float>(0));
  m(2,0) = -(m(0,2) = x.at<float>(1));
  return m;
}

}
#endif /* EXTRINSICS_H_ */
