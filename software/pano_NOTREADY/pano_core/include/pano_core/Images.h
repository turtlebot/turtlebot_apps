/*
 * Images.h
 *
 *  Created on: Oct 16, 2010
 *      Author: ethan
 */

#ifndef IMAGES_H_
#define IMAGES_H_

#include <opencv2/core/core.hpp>
#include <string>

#include <pano_core/pano_interfaces.h>
#include <map>

namespace pano
{
class Images : public serializable
{

public:
  Images() :
    ondisk_(false), persist_img_(false)
  {
  }

  Images(const Images& rhs)
  {
    copyData(rhs);
  }
  Images& operator=(const Images& rhs)
  {
    if (&rhs != this)
    {
      copyData(rhs);
    }
    return *this;
  }
  explicit Images(const cv::Mat& src);
  Images(const std::string& fname, const std::string& path = ".");

  virtual ~Images()
  {
  }
  void clear();

  void load(const cv::Mat& src,bool dogray = true);
  void load(const cv::Mat& src, const std::string& fname, const std::string& path,bool persist = false);
  void load(const std::string& fname, const std::string& path);

  void restore();

  const cv::Mat& src() const
  {
    return src_;
  }
  const cv::Mat& grey() const
  {
    return grey_;
  }

  std::string& fname()
  {
    return fname_;
  }
  std::string& path()
  {
    return path_;
  }
  const std::string& fname() const
  {
    return fname_;
  }
  const std::string& path() const
  {
    return path_;
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
  void copyData(const Images& rhs)
  {
    fname_ = rhs.fname_;
    path_ = rhs.path_;
    ondisk_ = rhs.ondisk_;
    persist_img_ = rhs.persist_img_;


    if (!rhs.src_.empty())
      rhs.src_.copyTo(src_);
    if (!rhs.grey_.empty())
      rhs.grey_.copyTo(grey_);
  }
  cv::Mat src_;
  cv::Mat grey_;

  std::string fname_;
  std::string path_;
  bool ondisk_;
  bool persist_img_;
};

class HugeImage
{
public:

  void setSize(cv::Size size){
    size_ = size;
  }
 // cv::Rect getEncompassingRect() const;
  cv::Mat loadAll() const;
  void serialize(const std::string& name) const;
  void deserialize(const std::string& name);

  void addRoi(int id, cv::Rect rect)
  {
    rois_[id] = rect;
  }

  std::string addName(int id, std::string&prefix);

  cv::Rect getRoi(int id)
  {
    return rois_[id];
  }
  std::string getName(int id)
  {
    return names_[id];
  }

private:
  std::map<int, cv::Rect> rois_;
  std::map<int, std::string> names_;
  cv::Size size_;
};

}
#endif /* IMAGES_H_ */
