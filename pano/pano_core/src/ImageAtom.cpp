#include "pano_core/ImageAtom.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pano_core/feature_utils.h"

#include <iomanip>
#include <sstream>

#include <pano_core/ModelFitter.h>
using namespace cv;
using namespace std;
namespace pano
{
static int uid_gen = 0;
int getUID()
{
  return uid_gen++;
}
ImageAtom::ImageAtom() :
  extrinsics_(cv::Mat::eye(3, 3, CV_32FC1), 0), uid_(-1)
{

}
ImageAtom::~ImageAtom()
{

}

ImageAtom::ImageAtom(const Camera& camera, const Images& images) :
  images_(images), camera_(camera), uid_(-1)
{

}
//cv::Mat ImageAtom::undistortPoints(){
//  vector<Point2f> points(features_.pts().size());
////  Mat mpoints(points);
//  Mat nK;
//  cv::undistortPoints(Mat(features_.pts()),points,camera_.K(),camera_.D());
//  features_.pts() = points;
//  for(size_t i = 0; i < features_.kpts().size();i++){
//    features_.kpts()[i] = features_.pts()[i];
//  }
//  camera_.setCameraIntrinsics(nK,Mat(),camera_.img_size());
//  return nK;
//}
void ImageAtom::detect(const cv::FeatureDetector& detector)
{
  features_.detect(detector, images_.grey());
}

#define MATCH_DEBUG 0

void ImageAtom::descriptorMatchMask(const ImageAtom& query, cv::Mat& mask, const cv::Mat& H, float uncertainty) const
{
  const ImageAtom* prior = this;
  Mat _H = H;

  if (_H.empty() && query.extrinsics().flag(Extrinsics::ESTIMATED) && prior->extrinsics().flag(Extrinsics::ESTIMATED))
  {
    if(angularDist(query.extrinsics(),prior->extrinsics()) > camera().fov_max()){
      mask = Mat::zeros(query.features().kpts().size(),prior->features().kpts().size(),CV_8UC1);
      return;
    }
    Mat R21 = prior->extrinsics().relativeToOther(query.extrinsics(), Extrinsics::R);
    _H = query.camera().K() * R21 * prior->camera().Kinv();
    uncertainty += query.extrinsics().val(Extrinsics::CONFIDENCE) + prior->extrinsics().val(Extrinsics::CONFIDENCE);
  }
  if (!_H.empty())
  {
    vector<Point2f> pts(query.features().pts().size());
    Mat m_pts(pts);
    Mat src(query.features().pts());
    perspectiveTransform(src, m_pts, _H);
    vector<KeyPoint> kpts;
    PointsToKeyPoints(pts, kpts);
#if MATCH_DEBUG
    {
      Mat hypo;
      drawKeypoints(query.images().grey(), kpts, hypo, Scalar(255, 255, 255));
      //drawKeypoints(hypo, query.features().kpts(), hypo, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_OVER_OUTIMG);
      drawKeypoints(hypo, prior->features().kpts(), hypo, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
      imshow("hypo", hypo);
      waitKey(10);
    }
#endif
    mask = windowedMatchingMask(kpts, prior->features().kpts(), uncertainty, uncertainty);
  }
}

void ImageAtom::match(const ImageAtom& query, std::vector<cv::DMatch>& matches, const Mat& H, float uncertainty) const
{
  Mat mask;
  const ImageAtom* prior = this;
  if(uncertainty >0){
    descriptorMatchMask(query, mask, H, uncertainty);
  }
  prior->features().match(query.features(), mask, matches);
  std::sort(matches.begin(), matches.end());
  DMatch match(0, 0, 80);
  size_t i;
  for (i = 0; i < matches.size(); i++)
  {
    if (match < matches[i])
      break;
  }
  //vector<DMatch>::iterator it = find(matches.begin(),matches.end(),match);
  if (i != matches.size())
    matches.resize(i);
#if MATCH_DEBUG
  Mat m_img; //= query.images().grey().clone();
  //drawMatchesRelative(prior->features(),query.features(),matches,m_img,vector<uchar>());
  drawMatches(query.images().src(), query.features().kpts(), images().src(), prior->features().kpts(), matches, m_img);
  imshow("matches", m_img);
  waitKey(30);
#endif

}

/*
 * drawable functions
 */

void ImageAtom::draw(cv::Mat* out, int flags)
{
  Mat tout;
  if (flags & DRAW_FEATURES)
  {
    cv::drawKeypoints(images_.grey(), features_.kpts(), tout);
  }
  *out = tout;

}

void ImageAtom::setUid(int id)
{
  uid_ = id;
  if (images().fname().empty())
  {
    std::stringstream ss;

    ss << "img" << std::setfill('0') << std::setw(5) << uid_ << ".jpg";
    images().fname() = ss.str();
  }

}

void ImageAtom::serialize(cv::FileStorage& fs) const
{
  fs << "{";
  fs << "features";
  features_.serialize(fs);
  fs << "images";
  images_.serialize(fs);
  fs << "extrinsics";
  extrinsics_.serialize(fs);
  fs << "camera";
  camera_.serialize(fs);
  fs << "uid" << uid_;
  fs << "}";
}

void ImageAtom::deserialize(const cv::FileNode& fn)
{
  features_.deserialize(fn["features"]);
  images_.deserialize(fn["images"]);
  extrinsics_.deserialize(fn["extrinsics"]);
  camera_.deserialize(fn["camera"]);
  uid_ = (int)fn["uid"];
}

}
