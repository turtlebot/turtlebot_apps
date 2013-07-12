/*
 * ModelFitter.cpp
 *
 *
 *
 */

#include <stdint.h>
#include <iostream>
#include <algorithm>

#include <pano_core/ModelFitter.h>
#include <pano_core/feature_utils.h>

#include <opencv2/highgui/highgui.hpp>

using namespace cv;

namespace pano
{
const char* FitterResult::GetStdMatName(int which)
{
  switch (which)
  {
    case R:
      return "R";
    case W_HAT:
      return "W_HAT";
    case T:
      return "T";
    default:
      return "unknown";
  }
}

std::vector<cv::Mat> FitterResult::GenerateStdMats()
{
  std::vector<cv::Mat> mats(N_MATS);
  mats[R] = cv::Mat::eye(cv::Size(3, 3), CV_32F);
  mats[W_HAT] = cv::Mat::zeros(cv::Size(3, 1), CV_32F);
  mats[T] = cv::Mat::zeros(cv::Size(3, 1), CV_32F);
  return mats;
}

FitterResult::FitterResult() :
  mats_(N_MATS), success_(false), err_(1.0e8), err_thresh_(0), inliers_(0), empty_(true)
{
}
FitterResult::FitterResult(const std::vector<cv::Mat>& mats, bool success, double err, double err_thresh,
                           const std::vector<uchar>& inlier_mask, size_t inliers) :
  mats_(mats), success_(success), err_(err), err_thresh_(err_thresh), inlier_mask_(inlier_mask), inliers_(inliers),
      empty_(false), names_(mats_.size())
{

}
const char* FitterResult::getMatName(int which) const
{
  if (!names_[which].empty())
    return names_[which].c_str();
  else
    return GetStdMatName(which);
}
void FitterResult::setNonStdMatName(int idx, const std::string& name)
{
  names_[idx] = name;
}
void FitterResult::serialize(cv::FileStorage& fs) const
{
  fs << "{";
  cvWriteComment(*fs, "FitterResult class", 0);
  fs << "mats" << "[";
  for (size_t i = 0; i < mats_.size(); ++i)
  {
    if(!mats_[i].empty()){
      cvWriteComment(*fs, getMatName(i), 0);
      fs << mats_[i];
    }
//    else
//      fs << "[]";
  }
  fs << "]";
  fs << "names" << "[";
  for (size_t i = 0; i < names_.size(); ++i)
  {
    fs << getMatName(i);
  }
  fs << "]";
  fs << "empty" << (int)empty_;
  fs << "success" << (int)success_;
  fs << "inliers" << (int)inliers_;
  fs << "err" << err_;
  fs << "err_thresh" << err_thresh_;
  //fs << "inlier_mask" << result.inlier_mask;
  fs << "}";
}
void FitterResult::deserialize(const cv::FileNode& node)
{
  FileNode mats = node["mats"];
  CV_Assert(mats.type() == FileNode::SEQ)
    ;
  mats_.resize(mats.size());
  for (size_t i = 0; i < mats.size(); i++)
  {
    mats[i] >> mats_[i];
  }

  FileNode names = node["names"];
  CV_Assert(names.type() == FileNode::SEQ)
    ;
  names_.resize(names.size());
  for (size_t i = 0; i < names.size(); i++)
  {
    names_[i] = (string)names[i];
  }
  empty_ = (int)node["empty"];
  success_ = (int)node["success"];
  inliers_ = (int)node["inliers"];
  err_ = (double)node["err"];
  err_thresh_ = (double)node["err_thresh"];
}

AtomPair::AtomPair() :
  /* pair<cv::Ptr<ImageAtom>, cv::Ptr<ImageAtom> > (),*/atom1_(), atom2_(), matches_(new std::vector<DMatch>()), result_(new FitterResult),
      pts1_(new std::vector<cv::Point2f>()), pts2_(new std::vector<cv::Point2f>()),
      rays1_(new std::vector<cv::Point3f>()), rays2_(new std::vector<cv::Point3f>())
{

}

AtomPair::AtomPair(const cv::Ptr<ImageAtom>& atom1, const cv::Ptr<ImageAtom>& atom2,
                   const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2) :
  atom1_(atom1), atom2_(atom2), result_(new FitterResult()), pts1_(new std::vector<cv::Point2f>(points1)),
      pts2_(new std::vector<cv::Point2f>(points2)), rays1_(new std::vector<cv::Point3f>(points1.size())),
      rays2_(new std::vector<cv::Point3f>(points2.size()))
{

  points2fto3f(pts1_->begin(), pts1_->end(), rays1_->begin(), atom1_->camera().Kinv());
  points2fto3f(pts2_->begin(), pts2_->end(), rays2_->begin(), atom2_->camera().Kinv());
}
AtomPair::AtomPair(const Ptr<ImageAtom>& atom1, const Ptr<ImageAtom>& atom2, const std::vector<cv::DMatch>& matches) :
  atom1_(atom1), atom2_(atom2), matches_(new vector<DMatch> (matches)), result_(new FitterResult()),
      pts1_(new std::vector<cv::Point2f>()), pts2_(new std::vector<cv::Point2f>()),
      rays1_(new std::vector<cv::Point3f>()), rays2_(new std::vector<cv::Point3f>())
{
  matches2points(atom1_->features().kpts(), atom2_->features().kpts(), *matches_, *pts1_, *pts2_);

  rays1_->resize(pts1_->size());
  rays2_->resize(pts2_->size());
  points2fto3f(pts1_->begin(), pts1_->end(), rays1_->begin(), atom1_->camera().Kinv());
  points2fto3f(pts2_->begin(), pts2_->end(), rays2_->begin(), atom2_->camera().Kinv());
}

void AtomPair::setMatches(const std::vector<cv::DMatch>& matches){
  *matches_ = matches;
  matches2points(atom1_->features().kpts(), atom2_->features().kpts(), matches, *pts1_, *pts2_);
  rays1_->resize(pts1_->size());
  rays2_->resize(pts2_->size());
  points2fto3f(pts1_->begin(), pts1_->end(), rays1_->begin(), atom1_->camera().Kinv());
  points2fto3f(pts2_->begin(), pts2_->end(), rays2_->begin(), atom2_->camera().Kinv());
}
const std::vector<cv::DMatch>& AtomPair::getMatches() const
{
  return *matches_;
}
FitterResult & AtomPair::result()
{
  return *result_;
}
const FitterResult & AtomPair::result() const
{
  return *result_;
}

void AtomPair::serialize(cv::FileStorage& fs) const
{
  fs << "{";
  fs << "atom1";
  fs << atom1_->uid();
  fs << "atom2";
  fs << atom2_->uid();
  fs << "result";
  result_->serialize(fs);
  fs << "}";
}
void AtomPair::deserialize(const cv::FileNode& fn)
{
  if (atom1_.empty())
    atom1_ = new ImageAtom();
  atom1_->setUid(int(fn["atom1"]));
  if (atom2_.empty())
    atom2_ = new ImageAtom();
  atom2_->setUid(int(fn["atom2"]));
  result_->deserialize(fn["result"]);
}

void AtomPair::drawMatches(cv::Mat& out) const{

  cv::drawMatches(atom2_->images().src(),atom2_->features().kpts(), atom1_->images().src(),atom1_->features().kpts(),*matches_,out);

}
namespace
{

inline Mat reprojectPoint(const Mat& p13d, const Mat& p23d, const Mat& R, const Mat& K)
{

  Mat p23dhat = R * p13d;
  Mat p2 = K * p23d;
  Mat p2hat = K * p23dhat;
  p2 /= p2.at<float> (2);
  p2hat /= p2hat.at<float> (2);
  return (p2 - p2hat); // return difference!
}

inline float calcErrorL2(const Mat& p13d, const Mat& p23d, const Mat& R, const Mat& K)
{
  return norm(reprojectPoint(p13d, p23d, R, K), cv::NORM_L2);
}

float calcErrorL1(const Mat& p13d, const Mat& p23d, const Mat& R, const Mat& K)
{
  return norm(reprojectPoint(p13d, p23d, R, K), cv::NORM_L1);
}

} // end anonymous namespace


float calcError(const Point3f& p1, const Point3f& p2, const Mat& R, const Mat& K)
{
  return calcErrorL1(Mat(p1), Mat(p2), R, K);
}

namespace
{

typedef float (*calcErr_ptr)(const Mat& /*p13d*/, const Mat& /*p23d*/, const Mat& /*R*/, const Mat& /*K*/);

}

float calcError(const std::vector<Point3f>& pts1, const std::vector<Point3f>& pts2, const std::vector<uchar> & mask,
                const cv::Mat& R, const cv::Mat& K, int norm_type)
{
  calcErr_ptr erf_ptr;
  erf_ptr = (norm_type == cv::NORM_L1) ? calcErrorL1 : calcErrorL2;

  float Esum = 0;
  int inliers = 0;
  for (size_t i = 0; i < pts1.size(); i++)
  {
    if (mask.empty() || mask[i])
    {
      Esum += erf_ptr(Mat(pts1[i]), Mat(pts2[i]), R, K);
      inliers++;
    }
  }

  Esum = (norm_type == cv::NORM_L1) ? Esum : sqrt(Esum);
  return Esum / inliers;

}

float calcReprojectonError(const vector<Point2f>& pts1, const vector<Point2f>& pts2, const std::vector<uchar> & mask,
                           const cv::Mat& R, const cv::Mat& _K, int norm_type)
{
  calcErr_ptr erf_ptr;
  erf_ptr = (norm_type == cv::NORM_L1) ? calcErrorL1 : calcErrorL2;
  size_t num_pts = pts1.size();
  Mat Rinv = R.t();
  cv::Mat K;
  _K.convertTo(K, CV_32F);
  Mat Kinv = K.inv();
  int inliers = 0;
  float Esum = 0.0;
  CV_Assert(num_pts == pts2.size());
  //std::cout << "Kinv" << Kinv << std::endl;
  for (size_t k = 0; k < num_pts; k++)
  {
    if (mask.empty() || mask[k])
    {
      Mat p13d = point2fto3dMat(pts1[k], Kinv);
      Mat p23d = point2fto3dMat(pts2[k], Kinv); // raise 2d points uv2 to xyz2
//      std::cout << "p1" << pts1[k] << "p2" << pts2[k] << std::endl;
//      std::cout << "p13d" << p13d << " p23d" << p23d << std::endl;
      Esum += erf_ptr(p13d, p23d, R, K);
      inliers++;
    }
  }
  Esum = (norm_type == cv::NORM_L1) ? Esum : sqrt(Esum);
  if (inliers < 1.0)
  {
    inliers++;
  }
  return Esum / inliers;
}

void convertRTtoG(const cv::Mat& R, const cv::Mat& T, cv::Mat& G)
{
  assert(R.type() == T.type() && R.type() == CV_32F && R.rows == T.rows && T.rows == 3 && T.cols == 1 && R.cols == 3);
  G = cv::Mat::eye(4, 4, CV_32F);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      G.at<float> (i, j) = R.at<float> (i, j);
    }
    G.at<float> (i, 3) = T.at<float> (i);
  }
}

void sparsifyMatches(AtomPair& pair, int iKeep)
{

  //  Ptr<ImageAtom> atom1;
  //  Ptr<ImageAtom> atom2;
  //  atom1 = pair.atom1();
  //  atom2 = pair.atom2();

  //  bool atom1_has_trinsics = atom1->getTrinsics().isSet();
  //  bool atom2_has_trinsics = atom2->getTrinsics().isSet();
  //  if (!atom1_has_trinsics || !atom2_has_trinsics)
  //  {
  //    Lout(Logger::WARNING) << "one or both atoms doesn't have trinsics, not sparsifying..." << std::endl;
  //    return;
  //  }
  //  Mat R1 = Mat::eye(3, 3, atom1->getTrinsics().R.type());
  //  Mat R2 = atom2->getTrinsics().R * atom1->getTrinsics().R.t();
  //
  //  std::vector<Point3f> &rays1 = *pair.getRays1(), &rays2 = *pair.getRays2();
  //
  //  std::vector<std::pair<float, int/*idx*/> > reproj_errs(rays1.size());
  //  for (size_t i = 0; i < rays1.size(); ++i)
  //  {
  //    double dist = calcErrorL1(Mat(rays1[i]), Mat(rays2[i]), R2, atom1->getK());
  //    (reproj_errs[i]).first = dist;
  //    (reproj_errs[i]).second = i;
  //  }
  //  std::sort(reproj_errs.begin(), reproj_errs.end(), compareRealToIndexPair);
  //
  //  int ikeep = (int)reproj_errs.size() < iKeep ? reproj_errs.size() : iKeep;
  //  Ptr<vector<DMatch> > matches = new vector<DMatch> (ikeep);
  //  for (int i = 0; i < ikeep; ++i)
  //  {
  //    DMatch match = pair.getMatches()[reproj_errs[i].second];
  //    match.distance = reproj_errs[i].first;
  //    (*matches)[i] = match;
  //  }
  //  pair.setMatches(matches);

}

void ModelFitter::fit(AtomPair& pair)
{
  fit_impl(pair);
}

const string FitPair::VERBOSE = "FitPair::VERBOSE";
const string FitPair::UBER_VERBOSE = "FitPair::UBER_VERBOSE";

FitPair::FitPair(cv::Ptr<ModelFitter> fitter, int fail_limit, cv::Ptr<std::list<AtomPair> > good_pairs,
                 CallbackEngine*cbengine) :
  fitter(fitter), failcount(new int(0)), totalcount(new int(0)), fail_limit(fail_limit), good_pairs(good_pairs),
      cbengine(cbengine)
{
}
void FitPair::operator()(AtomPair &pair)
{

  if (fail_limit > 0 && *failcount > fail_limit)
    return;
  (*totalcount)++;
  fitter->fit(pair);

  if (pair.result().success())
  {
    //    vector<Point2f> points(20);
    //    vector<Point2f> pointsH(20);
    //    Mat m_points(points);
    //    Size sz = pair.atom2()->camera().img_size();
    //    randu(m_points, Scalar(0, 0), Scalar(sz.width, sz.height));
    //
    //    Mat H = pair.homographyFromQueryToOther(pair.atom2());
    //    Mat tpoints(pointsH);
    //    perspectiveTransform(m_points, tpoints, H);
    //    Images r = pair.atom2()->images();
    //    Images l = pair.atom1()->images();
    //    r.restore();
    //    l.restore();
    //    Mat result;
    //
    //    for (int i = 0; i < points.size(); i++)
    //    {
    //      Rect imroi(0, 0, sz.width, sz.height);
    //      Rect roir(points[i], Size(9, 9));
    //      Rect roil(pointsH[i]/*.x - 3, pointsH[i].y - 3*/, Size(9, 9));
    //
    //      if (imroi.contains(roir.tl()) && imroi.contains(roir.br()) && imroi.contains(roil.tl())
    //          && imroi.contains(roil.br()))
    //      {
    //        matchTemplate(l.grey()(roil), r.grey()(roir), result, CV_TM_SQDIFF);
    //        cout << "diff: " << result.at<float>(0) << endl;
    //      }
    //
    //    }

    //    int i_inliers = pair.result().inliers();
    //    double i_err = pair.result().err();
    //    cout << "error initial " << i_err <<endl;
    //    Mat H = pair.homographyFromQueryToOther(pair.atom2());
    //    vector<DMatch> matches;
    ////    pair.atom1()->match(*pair.atom2(),matches,H, 2);
    //   // sort(matches.begin(),matches.end());
    //   // matches.resize(matches.size()/2);
    //    pair = AtomPair(pair.atom1(),pair.atom2(),matches);
    //    fitter->fit(pair);
    //
    //    double p_err = pair.result().err();
    //    cout << "error post " << p_err <<endl;
    //    if(pair.result().success())
    good_pairs->push_back(pair);
    // IFLOG(VERBOSE, waitKey(10));
  }
  if (cbengine)
    cbengine->callBack(pair, 0);

}
}

