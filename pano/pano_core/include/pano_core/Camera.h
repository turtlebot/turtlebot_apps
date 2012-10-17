/*
 * Camera.h
 *
 *  Created on: Oct 17, 2010
 *      Author: ethan
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <opencv2/core/core.hpp>

#include <pano_core/pano_interfaces.h>
namespace pano
{

class Camera : public serializable
{

public:
  Camera();
  ~Camera();

  explicit Camera(const std::string& camera_file);

  void setCameraIntrinsics(const std::string& filename);
  void setCameraIntrinsics(const cv::Mat& K, const cv::Mat& D, const cv::Size& img_size);

  void initUndistort();

  void undistort(const cv::Mat& image, cv::Mat& uimage) const;

  void ptsToRays(const std::vector<cv::Point2f>& pts, std::vector<cv::Point3f>& rays) const;
  void raysToPts(const std::vector<cv::Point3f>& rays, std::vector<cv::Point2f>& pts) const;

  float fovX() const
  {
    return fov_x_;
  }
  float fovY() const
  {
    return fov_y_;
  }
  float fov_max() const
  {
    return fov_x_ < fov_y_ ? fov_y_ : fov_x_;
  }
  const cv::Size& img_size() const
  {
    return img_size_;
  }
  const cv::Mat& K() const
  {
    return K_;
  }
  const cv::Mat& Kinv() const
  {
    return Kinv_;
  }
  const cv::Mat & D() const
  {
    return D_;
  }

  void scale(float x, float y);
  /*
   * serializable functions
   */
  virtual int version() const
  {
    return 0;
  }

  void write(const std::string& file_name) const;
  virtual void serialize(cv::FileStorage& fs) const;
  virtual void deserialize(const cv::FileNode& fn);

  static void KtoFOV(const cv::Mat& K, float & fovx, float & fovy);

private:
  void setupK();
  cv::Mat K_; //!<camera matrix
  cv::Mat Kinv_; //!< pre computed camera inverse, for projecting points

  cv::Mat D_; //!< distortion coeffecients;

  cv::Size img_size_; //!< image size that this is based on

  float fov_x_; //!< camera field of view x not focal length.
  float fov_y_; //!< camera field of view y

  cv::Mat P_;
  cv::Mat Pinv_;
  cv::Mat u_rm1_, u_rm2_;

};

}

#endif /* CAMERA_H_ */
