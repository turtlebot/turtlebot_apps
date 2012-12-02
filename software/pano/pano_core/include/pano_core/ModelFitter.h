/*
 * ModelFitter.h
 *
 *  Created on: Jun 5, 2010
 *      Author: ethan
 */

#ifndef PANO_MODELFITTER_H_
#define PANO_MODELFITTER_H_

#include <iostream>

#include <string>
#include <utility>

#include <vector>
#include <list>
#include <map>
#include <set>

#include <stdexcept>
#include <exception>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pano_core/pano_interfaces.h>
#include <pano_core/ImageAtom.h>
#include <pano_core/callbacks.h>

namespace pano
{

/** The result of a fit - this is universal to to fitting two images to
 * some extrinsic model (eg. rotation)
 */
class FitterResult : public serializable
{
public:

  FitterResult();
  FitterResult(const std::vector<cv::Mat>& mats, bool success, double err, double err_thresh,
               const std::vector<uchar>& inlier_mask, size_t inliers);
  virtual ~FitterResult()
  {
  }

  bool success() const
  {
    return success_;
  }

  bool empty() const
  {
    return empty_;
  }

  const cv::Mat& mat(size_t which) const
  {
    CV_Assert(which < mats_.size())
      ;
    return mats_[which];
  }

  double err() const
  {
    return err_;
  }
  double err_thresh() const
  {
    return err_thresh_;
  }
  const std::vector<uchar>& inlier_mask() const
  {
    return inlier_mask_;
  }
  size_t inliers() const
  {
    return inliers_;
  }

  virtual int version() const
  {
    return 0;
  }
  virtual void serialize(cv::FileStorage& fs) const;
  virtual void deserialize(const cv::FileNode& fn);

  const char* getMatName(int which) const;
  void setNonStdMatName(int idx, const std::string& name);

  enum StdMats
  {
    R = 0, //!<Rotation matrix
    W_HAT = 1, //!<3 degrees of freedom representation of R Matrix
    T = 2, //!<Translation matrix
    N_MATS
  //!< number of mats (always keep this at the end of the StdMats enum
  };

  static const char* GetStdMatName(int which);
  static std::vector<cv::Mat> GenerateStdMats();

private:
  std::vector<cv::Mat> mats_;
  bool success_;
  double err_;
  double err_thresh_;
  std::vector<uchar> inlier_mask_;
  size_t inliers_; //! number of 'inliers'
  bool empty_;

  std::vector<std::string> names_;
};

struct AtomPair : public serializable
{
  AtomPair();

  AtomPair(const cv::Ptr<ImageAtom>& atom1, const cv::Ptr<ImageAtom>& atom2, const std::vector<cv::Point2f>& points1,  const std::vector<cv::Point2f>& points2);
  AtomPair(const cv::Ptr<ImageAtom>& atom1, const cv::Ptr<ImageAtom>& atom2, const std::vector<cv::DMatch>& matches);

  void setAtom1(const cv::Ptr<ImageAtom>& atom)
  {
    atom1_ = atom;
  }
  void setAtom2(const cv::Ptr<ImageAtom>& atom)
  {
    atom2_ = atom;
  }

  const cv::Ptr<ImageAtom>& atom1() const
  {
    return atom1_;
  }
  const cv::Ptr<ImageAtom>& atom2() const
  {
    return atom2_;
  }

  const cv::Ptr<ImageAtom>& other(const cv::Ptr<ImageAtom>& atom) const
  {
    return atom == atom1_ ? atom2_ : atom1_;
  }

  Extrinsics generateExtrinsics(const cv::Ptr<ImageAtom>& atom) const
  {
    Extrinsics ext = atom->extrinsics();
    ext.flag(Extrinsics::ESTIMATED) = result_->success();
    if (ext.flag(Extrinsics::ESTIMATED))
    {
      ext.mat(Extrinsics::R) = RofThis(atom);
      cv::Rodrigues(ext.mat(Extrinsics::R), ext.mat(Extrinsics::W));

      ext.val(Extrinsics::CONFIDENCE) = result().err();

      ext.val(Extrinsics::CONFIDENCE) += other(atom)->extrinsics().val(Extrinsics::CONFIDENCE);
    }

    return ext;
  }

  cv::Mat RofThis(const cv::Ptr<ImageAtom>& atom) const
  {
    cv::Mat R_12 = TMtoOther(other(atom), FitterResult::R);
    cv::Mat R_1 = other(atom)->extrinsics().mat(Extrinsics::R);
    return R_12 * R_1;
  }
  cv::Mat TMtoOther(const cv::Ptr<ImageAtom>& atom, int which) const
  {
    return atom == atom2_ ? result_->mat(which).t() : result_->mat(which);
  }

  cv::Mat homographyFromQueryToOther(const cv::Ptr<ImageAtom>& query_p) const
  {
    const ImageAtom& query = *query_p, &prior = *other(query_p);
    cv::Mat R21 = TMtoOther(query_p, Extrinsics::R);
    cv::Mat H = query.camera().K() * R21 * prior.camera().Kinv();
    return H;
  }

  void setMatches(const std::vector<cv::DMatch>& matches);
  const std::vector<cv::DMatch>& getMatches() const;
  FitterResult & result();
  const FitterResult & result() const;

  const std::vector<cv::Point2f>& pts1() const
  {
    return *pts1_;
  }
  const std::vector<cv::Point2f>& pts2() const
  {
    return *pts2_;
  }
  const std::vector<cv::Point3f>& rays1() const
  {
    return *rays1_;
  }
  const std::vector<cv::Point3f>& rays2() const
  {
    return *rays2_;
  }
  const std::vector<cv::DMatch>& matches() const
  {
    return *matches_;
  }

  bool operator==(const AtomPair& rhs) const
  {
    return atom1_ == rhs.atom1_ && atom2_ == rhs.atom2_;
  }

  virtual int version() const
  {
    return 0;
  }

  void drawMatches(cv::Mat& out) const;
  virtual void serialize(cv::FileStorage& fs) const;
  virtual void deserialize(const cv::FileNode& fn);

private:

  cv::Ptr<ImageAtom> atom1_, atom2_;
  cv::Ptr<std::vector<cv::DMatch> > matches_;
  cv::Ptr<FitterResult> result_;
  cv::Ptr<std::vector<cv::Point2f> > pts1_, pts2_;
  cv::Ptr<std::vector<cv::Point3f> > rays1_, rays2_;
};

struct AtomPairLess
{
  bool operator()(const AtomPair& lhs, const AtomPair& rhs) const
  {
    // look at std::pair for this implementation
    //    __x.first < __y.first
    //                 || (!(__y.first < __x.first) && __x.second < __y.second)
    return lhs.atom1() < rhs.atom1() || (!(rhs.atom1() < lhs.atom1()) && lhs.atom2() < rhs.atom2());
  }
};
struct AtomPairMin
{
  bool operator()(const AtomPair& lhs, const AtomPair& rhs)
  {
    return lhs.result().err() < rhs.result().err();
  }
};
struct AtomPairDist
{
  float operator()(const AtomPair& p) const
  {
    return norm(p.result().mat(FitterResult::W_HAT));
  }
  bool operator()(const AtomPair& lhs, const AtomPair& rhs) const
  {
    return (*this)(lhs) < (*this)(rhs);
  }
};

typedef std::set<AtomPair, AtomPairLess> AtomPairSet;

struct PairPointsCSV
{
  std::ostream& out;

  PairPointsCSV(std::ostream&out);

  void operator()(const std::pair<const cv::Point2f&, const cv::Point2f&>& pp);
  void operator()(const AtomPair&pair);
};

class ModelFitterParams
{
public:
  virtual ~ModelFitterParams()
  {
  }

  template<typename T>
    const T& cast() const
    {
      dynamic_cast<const T&> (*this);
    }
  template<typename T>
    T& cast()
    {
      dynamic_cast<T&> (*this);
    }
};

class ModelFitter
{
public:

  virtual ~ModelFitter()
  {
  }

  void fit(AtomPair& pair);

protected:

  virtual void fit_impl(AtomPair& pair) = 0;

};

template<typename Fitter, typename Params>
  class GenericModelFitter : public ModelFitter
  {
  public:
    GenericModelFitter() :
      fitter_(), params_()
    {
    }

    GenericModelFitter(const Params& params) :
      fitter_(), params_(params)
    {
    }

  protected:
    virtual void fit_impl(AtomPair& pair)
    {
      fitter_(params_, pair);
    }
  private:
    Fitter fitter_;
    Params params_;
  };

struct FitPair
{
  const static std::string VERBOSE;
  const static std::string UBER_VERBOSE;

  cv::Ptr<ModelFitter> fitter;
  cv::Ptr<int> failcount;
  cv::Ptr<int> totalcount;

  int fail_limit;

  cv::Ptr<std::list<AtomPair> > good_pairs;
  CallbackEngine* cbengine;

  FitPair(cv::Ptr<ModelFitter> fitter, int fail_limit, cv::Ptr<std::list<AtomPair> > good_pairs,CallbackEngine*cbengine=NULL);

  void operator()(AtomPair &pair);
};

/** \brief concatenate 3x3 R and 3x1 T into a 4x4 G matrix
 */
void convertRTtoG(const cv::Mat& R, const cv::Mat& T, cv::Mat& G);

/** gets the L2 norm between the p1 and p2 given an R
 * takes \hat{p2} = R * p1
 * return norm(p2 , \hat{p2})
 */
float calcError(const cv::Point3f& p1, const cv::Point3f& p2, const cv::Mat& R, const cv::Mat& K);

float calcError(const std::vector<cv::Point3f>& pts1, const std::vector<cv::Point3f>& pts2,
                const std::vector<uchar> & mask, const cv::Mat& R, const cv::Mat& K, int norm_type = cv::NORM_L1);

/** lift p1 to 3d, rotate points by R, reproject by K, compare to p2.
 * also takes the mask for doing this for only a subset.
 */
float calcReprojectonError(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
                           const std::vector<uchar> & mask, const cv::Mat& R, const cv::Mat& K, int norm_type =
                               cv::NORM_L1);

/** keep only a subset of points, with lower reproject error via this fitter
 */
void sparsifyMatches(const ImageAtom& atom1, const ImageAtom& atom2, std::vector<cv::Point2f>& pts1, std::vector<
    cv::Point2f>& pts2, int iKeep = 50);

/** keep only a subset of points, with lower reproject error via this fitter
 */
void sparsifyMatches(AtomPair& pair, int iKeep = 50);

inline void point2fto3dMat(const cv::Point2f& p1, const cv::Mat& Kinv, cv::Mat& p13d)
{
  p13d = Kinv * (cv::Mat_<float>(3, 1) << p1.x, p1.y, 1.0f);
  p13d /= norm(p13d);
}

inline cv::Mat point2fto3dMat(const cv::Point2f& p1, const cv::Mat& Kinv)
{
  cv::Mat p13d;
  point2fto3dMat(p1, Kinv, p13d);
  return p13d;
}
/** 'lift' 2d to 3d points via K and assumption of being on unit sphere
 */
inline cv::Mat pt2f_to_3d(const cv::Point2f& p1, const cv::Mat& K)
{
  return point2fto3dMat(p1, K.inv());
}

inline cv::Point3f point2fTo3f(const cv::Point2f& p1, const cv::Mat& Kinverse)
{
  assert(Kinverse.type() == CV_32F);
  cv::Mat p13d = Kinverse * (cv::Mat_<float>(3, 1) << p1.x, p1.y, 1.0f);
  p13d /= norm(p13d);
  return cv::Point3f(p13d.at<float> (0), p13d.at<float> (1), p13d.at<float> (2));
}

inline cv::Point2f point3fTo2f(const cv::Point3f& p1, const cv::Mat& K)
{
  assert(K.type() == CV_32F);
  cv::Mat p12d = K * (cv::Mat_<float>(3, 1) << p1.x, p1.y, p1.z);
  p12d /= p12d.at<float> (2);
  return p12d.at<cv::Point2f> (0);
}

template<typename Inserter, typename Begin, typename End>
  void points2fto3f(Begin begin, End end, Inserter it, const cv::Mat& Kinverse)
  {
    while (begin != end)
    {
      *it = point2fTo3f(*begin, Kinverse);
      ++it;
      ++begin;
    }
  }

template<typename Inserter, typename Begin, typename End>
  void points3fto2f(Begin begin, End end, Inserter it, const cv::Mat& K)
  {
    while (begin != end)
    {
      *it = point3fTo2f(*begin, K);
      ++it;
      ++begin;
    }
  }

struct SVDRSolverParams : public serializable
{
  //  params.error_thresh = 5;
  //  params.inliers_thresh = 15;
  //  params.maxiters = 30;
  //  params.nNeeded = 2;
  SVDRSolverParams() :
    maxiters(30), error_thresh(5), inliers_thresh(15), nNeeded(2)
  {

  }

  virtual int version() const
  {
    return 0;
  }
  virtual void serialize(cv::FileStorage& fs) const;
  virtual void deserialize(const cv::FileNode& fn);

  int maxiters;
  double error_thresh;
  double inliers_thresh;
  int nNeeded;
};

struct SVDFitR
{
  void operator()(SVDRSolverParams& params, AtomPair& pair) const;
};

typedef GenericModelFitter<SVDFitR, SVDRSolverParams> SVDFitter;
}
#endif /* PANO_MODELFITTER_H_ */
