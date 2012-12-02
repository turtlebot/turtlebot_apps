#include <pano_core/Features.h>
#include <pano_core/feature_utils.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
namespace pano
{

Features::Features()
{
}

void Features::detect(const cv::FeatureDetector& detect, const cv::Mat& img)
{
  detect.detect(img, kpts_);
 // KeyPointsToPoints(kpts_, pts_);
}

void Features::extract(const cv::DescriptorExtractor& extract, const cv::Mat& img)
{
  extract.compute(img, kpts_, descriptors_);
  KeyPointsToPoints(kpts_, pts_);
  //  matcher_->add(vector<Mat>(1,descriptors_));
  //  matcher_->train();
}

void Features::match(const Features& features, const cv::Mat& mask, std::vector<cv::DMatch>& matches) const
{

  if (!descriptors_.empty() && !features.descriptors_.empty())
  {
    matcher_->match(features.descriptors_, descriptors_, matches, mask);
  }
}

void Features::serialize(cv::FileStorage& fs) const
{
  fs << "{";
  cvWriteComment(*fs, "Features class", 0);
  //  cv::write(fs,"keypoints",kpts_);
  fs << "}";
}
void Features::deserialize(const cv::FileNode& fn)
{
  read(fn["keypoints"], kpts_);

}
void drawMatchesRelative(const Features& train, const Features& query, const std::vector<cv::DMatch>& matches,
                         Mat& img, const vector<unsigned char>& mask)
{
  for (int i = 0; i < (int)matches.size(); i++)
  {
    if (mask.empty() || mask[i])
    {
      Point2f pt_new = query.pts()[matches[i].queryIdx];
      Point2f pt_old = train.pts()[matches[i].trainIdx];
      Point2f dist = pt_new - pt_old;

      cv::line(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
      cv::circle(img, pt_new, 2, Scalar(255, 0, 125), 1);

    }
  }
}
}
