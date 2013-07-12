/*
 * Features.h
 *
 *  Created on: Oct 16, 2010
 *      Author: ethan
 */

#ifndef PANO_FEATURES_H_
#define PANO_FEATURES_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pano_core/pano_interfaces.h>

#include <typeinfo>

namespace pano
{

class Features : public serializable
{
public:

  Features();
  virtual ~Features()
  {
  }

  Features(const Features& rhs)
  {
    copyData(rhs);
  }

  Features& operator=(const Features& rhs)
  {
    if (this != &rhs)
    {
      copyData(rhs);
    }
    return *this;
  }

  void detect(const cv::FeatureDetector& detect, const cv::Mat& img);

  cv::Ptr<cv::DescriptorMatcher> makeMatcher() const{
    return matcher_->clone(true);//matcher_copier_->clone(*matcher_);
  }
  template<typename T>
    void addMatcher(const T& matcher)
    {
      matcher_copier_ = cv::Ptr<Copier<cv::DescriptorMatcher> >(new SCopier<T, cv::DescriptorMatcher> ());
      matcher_ = cv::Ptr<cv::DescriptorMatcher>(new T(matcher));
    }

  template<typename T>
    void extract(const cv::DescriptorExtractor& extracter, const cv::Mat& img, const T& matcher)
    {
      addMatcher(matcher);
      extract(extracter, img);
    }

  void match(const Features& features, const cv::Mat& mask, std::vector<cv::DMatch>& matches) const;

  cv::Mat& descriptors()
  {
    return descriptors_;
  }
  const cv::Mat& descriptors() const
  {
    return descriptors_;
  }

  std::vector<cv::KeyPoint>& kpts()
  {
    return kpts_;
  }
  const std::vector<cv::KeyPoint>& kpts() const
  {
    return kpts_;
  }
  std::vector<cv::Point2f>& pts()
  {
    return pts_;
  }
  const std::vector<cv::Point2f>& pts() const
  {
    return pts_;
  }
  /*
   * serializable functions
   */
  virtual int version() const
  {
    return 0;
  }

  virtual void serialize(cv::FileStorage& fs) const;
  virtual void deserialize(const cv::FileNode& fn);

private:

  void extract(const cv::DescriptorExtractor& extract, const cv::Mat& img);

  void copyData(const Features& rhs)
  {
    if (!rhs.descriptors_.empty())
      rhs.descriptors_.copyTo(descriptors_);
    else
      descriptors_ = cv::Mat();

    kpts_ = rhs.kpts_;
    pts_ = rhs.pts_;
    if(rhs.matcher_copier_)
      matcher_ = rhs.matcher_copier_->make();
    matcher_copier_ = rhs.matcher_copier_;

}

  cv::Mat descriptors_;
  std::vector<cv::KeyPoint> kpts_;
  std::vector<cv::Point2f> pts_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  cv::Ptr<Copier<cv::DescriptorMatcher> > matcher_copier_;

};

void drawMatchesRelative(const Features& train, const Features& query, const std::vector<cv::DMatch>& matches,
                         cv::Mat& img, const std::vector<unsigned char>& mask = std::vector<unsigned char>());

}//namespace pano
#endif /* FEATURES_H_ */
