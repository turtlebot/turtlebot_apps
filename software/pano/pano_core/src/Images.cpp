#include "pano_core/Images.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <iomanip>
#include <map>

using namespace cv;
using std::map;

namespace pano
{
Images::Images(const cv::Mat& src) :
  ondisk_(false), persist_img_(false)
{
  load(src);
}
Images::Images(const std::string& fname, const std::string& path) :
  ondisk_(false), persist_img_(false)
{
  load(fname, path);
}

void Images::clear()
{
  src_ = Mat();
  grey_ = Mat();

}
void Images::load(const cv::Mat& src, bool dogray)
{
  src.copyTo(src_);
  if (src_.type() == CV_8UC3 && dogray)
  {
    cvtColor(src_, grey_, CV_RGB2GRAY);
  }
  else if (src_.type() == CV_8UC1)
  {
    grey_ = src_;
  }
  else
  {
    CV_Error(CV_StsUnsupportedFormat,"only supports 3 channel 8 bit or 1 channel 8 bit images");
  }
  // integral(grey_, sum_, CV_32S);

}
void Images::load(const cv::Mat& src, const std::string& fname, const std::string& path,bool persist)
{
  fname_ = fname;
  path_ = path;
  persist_img_ = persist;
  ondisk_ = !fname_.empty();
  load(src);
}
void Images::load(const std::string& fname, const std::string& path)
{

  fname_ = fname;
  path_ = path;
  Mat img;
  if(!path.empty())
    img = imread(path + "/" + fname);
  else
    img = imread(fname);
  CV_Assert(!img.empty())
    ;
  ondisk_ = true;
  persist_img_ = false;
  load(img);

}

void Images::restore()
{
  if (src_.empty() && ondisk_)
  {
    //std::cout << "attempted to load from " << fname_ << " at " << path_ << std::endl;
    load(fname_, path_);
  }
}

void Images::serialize(cv::FileStorage& fs) const
{
  if (!ondisk_ && persist_img_)
  {
    imwrite(path_ + "/" + fname_, src_);
  }
  fs << "{";
  cvWriteComment(*fs, "Images class", 0);
  fs << "fname" << fname_;
  fs << "path" << path_;
  fs << "ondisk" << (int)(ondisk_ || persist_img_);
  fs << "persist" << (int)(persist_img_);
  fs << "}";
}
void Images::deserialize(const cv::FileNode& fn)
{
  fname_ = (string)fn["fname"];
  path_ = (string)fn["path"];
  ondisk_ = (int)fn["ondisk"];
  persist_img_ = (int)fn["persist"];
  if (ondisk_)
  {
    //load(fname_, path_);
  }
}
// cv::Rect getEncompassingRect() const;
cv::Mat HugeImage::loadAll() const
{
  map<int, string>::const_iterator nit = names_.begin();
  map<int, Rect>::const_iterator rit = rois_.begin();
  Mat image = Mat::zeros(size_, CV_8UC3);
  for (; nit != names_.end(), rit != rois_.end(); ++nit, ++rit)
  {
    Mat roi = image(rit->second);
    Mat img = imread(nit->second);
    img.copyTo(roi);
  }
  return image;
}
void HugeImage::serialize(const std::string& name) const
{
  FileStorage fs(name, FileStorage::WRITE);
  fs << "big_image" << "[";
  map<int, string>::const_iterator nit = names_.begin();
  map<int, Rect>::const_iterator rit = rois_.begin();
  for (; nit != names_.end(), rit != rois_.end(); ++nit, ++rit)
  {
    fs << "{" << "id" << rit->first << "roi" << rit->second << "name" << nit->second << "}";
  }
  fs << "]";

}
void HugeImage::deserialize(const std::string& name)
{

}
std::string HugeImage::addName(int id, std::string&prefix)
{
  std::stringstream ss;
  ss << prefix << "_" << std::setw(5) << std::setfill('0') << id << ".png";

  names_[id] = ss.str();
  return names_[id];
}

}
