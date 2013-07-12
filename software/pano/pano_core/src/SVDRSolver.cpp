/*
 * SVDRSolver.cpp
 *
 *  Created on: Oct 19, 2010
 *      Author: erublee
 */
#include "pano_core/ModelFitter.h"
#include "pano_core/feature_utils.h"
using namespace cv;

#define ROOT_2 1.41421356
#define EPS_EQ(t,val) (t >= val - 1.0e-3 && t <= val + 1.0e-3)

namespace pano
{

// SVDRSolverParams params
//int maxiters;
// double error_thresh;
// double inliers_thresh;
// int nNeeded;

void SVDRSolverParams::serialize(cv::FileStorage& fs) const
{
  fs << "{";
  fs << "error_thresh" << error_thresh;
  fs << "inliers_thresh" << inliers_thresh;
  fs << "maxiters" << maxiters;
  fs << "nNeeded" << nNeeded;
  fs << "}";
}
void SVDRSolverParams::deserialize(const cv::FileNode& node)
{
  error_thresh = (double)node["error_thresh"];
  inliers_thresh = (double)node["inliers_thresh"];
  maxiters = (int)node["maxiters"];
  nNeeded = (int)node["nNeeded"];
}

namespace
{

Mat svdsolve(const cv::Mat& correlation)
{
  //solve for r using svd
  cv::SVD svd(correlation);

  //sgn of the determinant
  int s = cv::determinant(svd.u * svd.vt) > 0 ? 1 : -1;

  cv::Mat diagm = cv::Mat::eye(3, 3, svd.u.type());

  //create a matrix with all ones but the lower right corner = S
  diagm.at<float> (2, 2) = s;

  //according to the paper TM = U * diag(1,1,s) * V^T
  return svd.u * diagm * svd.vt;
}

Mat svdcorrelation(const vector<Point3f>& pts1, const vector<Point3f>& pts2, const vector<uchar>& mask)
{
  cv::Mat r1 = cv::Mat(3, 1, CV_32F);
  cv::Mat r2 = cv::Mat(3, 1, CV_32F);
  // compute cross correlation estimate of bunch of 3-vectors
  cv::Mat correlation = cv::Mat::zeros(3, 3, cv::DataType<float>::type);
  float* r1p = r1.ptr<float> (0);
  float* r2p = r2.ptr<float> (0);
  for (size_t i = 0; i < pts1.size(); i++)
  {
    if (mask.empty() || mask[i])
    {
      memcpy(r1p, &pts1[i], 3 * sizeof(float));
      memcpy(r2p, &pts2[i], 3 * sizeof(float));
      //add to the correlation matrix C = sum( x x' )
      correlation += r2 * r1.t();

    }
  }
  return correlation;
}

Mat svdsolve(const vector<Point3f>& pts1, const vector<Point3f>& pts2, const vector<uchar>& mask)
{
  return svdsolve(svdcorrelation(pts1, pts2, mask));
}

Mat svdsolve(const vector<Point2f>& pts1, const vector<Point2f>& pts2, const vector<uchar>& mask, const Mat& K)
{
  // form the estimate covariance matrix from data points, sum( x x' )
  cv::Mat correlation = cv::Mat::zeros(3, 3, cv::DataType<float>::type);
  for (size_t i = 0; i < pts1.size(); i++)
  {
    if (mask.empty() || mask[i])
    {
      //create a 1X3 matrix for the img1 point
      cv::Mat r1 = pt2f_to_3d(pts1[i], K);

      //create a 3x1 matrix for the img2 point
      cv::Mat r2 = pt2f_to_3d(pts2[i], K);

      //add to the correlation matrix
      correlation += r2 * r1.t();
    }
  }

  return svdsolve(correlation);

}

struct NextNum
{
  int num_;
  NextNum() :
    num_(0)
  {
  }
  int operator()()
  {
    return num_++;
  }
};

void ransacSolveR(const vector<Point3f>& pts1, const vector<Point3f>& pts2, FitterResult& result, int maxiters,
                  int inliers_thresh, int nNeeded, double error_thresh, cv::Mat K, cv::Mat Kinv)
{
  float bestdist = 1e8;
  int bestinliers = 0;
  if (pts1.empty() || pts1.size() < size_t(inliers_thresh))
  {
    result = FitterResult(FitterResult::GenerateStdMats(), false, bestdist, error_thresh, vector<uchar> (), 0);
    return;
  }

  int iters = 0;

  vector<uchar> _best(pts1.size()), _mask(pts1.size());

  vector<uchar> * best = &_best;
  vector<uchar> * mask = &_mask;

  Mat bestR = Mat::eye(3, 3, DataType<float>::type);

  vector<Point3f> rpts1(nNeeded);
  vector<Point3f> rpts2(nNeeded);

  int ntotal = pts1.size();
  ;
  Mat R = cv::Mat::eye(3, 3, CV_32F);

  vector<int> idxs(pts1.size());
  generate(idxs.begin(), idxs.end(), NextNum());
//  {
//    Mat idxs_m(idxs);
//    randShuffle(idxs_m);
//  }
  int r_idx = 0;

  maxiters = std::min(int(pts1.size()),maxiters);

  while (iters++ < maxiters)
  {

    // there's no sense of whether or not we have 'converged' ?
    int numinliers = 0;
    float sumdist = 0;

    rpts1.clear();
    rpts2.clear();

    //select random subset
    while ((int)rpts1.size() < nNeeded)
    {

      if (find(rpts2.begin(), rpts2.end(), RadiusPoint(0.01, pts2[idxs[r_idx]])) == rpts2.end()
          && find(rpts1.begin(), rpts1.end(), RadiusPoint(0.01, pts1[idxs[r_idx]])) == rpts1.end())
      {
        rpts1.push_back(pts1[idxs[r_idx]]);
        rpts2.push_back(pts2[idxs[r_idx]]);

      }
      //cout << "idxs " << idxs[r_idx] << endl;
      r_idx++;
      if (r_idx >= (int)idxs.size())
      {
        break;
      }

    }

    if (r_idx >= (int)idxs.size())
    {

      r_idx = 0;
      Mat idxs_m(idxs);
      randShuffle(idxs_m);
      continue;
    }

    // fit TM to current inlier set via solver
    R = svdsolve(rpts1, rpts2, vector<uchar> ());

    std::fill(mask->begin(), mask->end(), 0);
    // for every point in the set, compute error per point
    for (int i = 0; i < ntotal; i++)
    {
      float error_point_i = calcError(pts1[i], pts2[i], R, K);
      if (error_point_i < error_thresh)
      { // if this point's error is good, keep as inlier
        numinliers++;
        (*mask)[i] = 1;
        sumdist += error_point_i;
      }
    }
    if (numinliers >= inliers_thresh)
    {

      Mat tR = svdsolve(pts1, pts2, *mask);
      float cur_dist = calcError(pts1, pts2, *mask, tR, K);
      //float cur_dist = sumdist/numinliers;
      if (cur_dist <= bestdist)
      {
        bestinliers = numinliers;
        vector<uchar> * temp = best;
        best = mask;
        mask = temp;
        bestR = tR;
        bestdist = cur_dist;
      }

    }

    if (inliers_thresh < bestinliers * 2){
     std::cout << "early break iters: " << iters << std::endl;
      break;
    }

  }

  cv::Mat omega_(3, 1, CV_32F);
  cv::Rodrigues(bestR, omega_);
  int good = bestinliers >= inliers_thresh;
  good = good * (bestdist < error_thresh);
  vector<Mat> mats = FitterResult::GenerateStdMats();

  mats[FitterResult::R] = bestR;
  mats[FitterResult::W_HAT] = omega_;

  result = FitterResult(mats, good, bestdist, error_thresh, *best, bestinliers);

}

void svdSolveR(const vector<Point3f>& rays1, const vector<Point3f>& rays2, FitterResult& result, int maxiters,
               int inliers_thresh, int nNeeded, double error_thresh, cv::Mat K, cv::Mat Kinv)
{

  // fit TM to current inlier set via solver
  Mat R = svdsolve(rays1, rays2, vector<uchar> ());
  cv::Mat omega_(3, 1, CV_32F);
  cv::Rodrigues(R, omega_);
  vector<Mat> mats = FitterResult::GenerateStdMats();

  mats[FitterResult::R] = R;
  mats[FitterResult::W_HAT] = omega_;

  result = FitterResult(mats, true, 1.0, error_thresh, vector<uchar> (1,rays1.size()), rays1.size());

}

void ransacSolveR(const vector<Point2f>& _pts1, const vector<Point2f>& _pts2, FitterResult& result, int maxiters,
               int inliers_thresh, int nNeeded, double error_thresh, cv::Mat K, cv::Mat Kinv)
{

  vector<Point3f> m_pts1, m_pts2;
  m_pts1.resize(_pts1.size());
  m_pts2.resize(_pts2.size());
  points2fto3f(_pts1.begin(), _pts1.end(), m_pts1.begin(), Kinv);
  points2fto3f(_pts2.begin(), _pts2.end(), m_pts2.begin(), Kinv);
  ransacSolveR(m_pts1, m_pts2, result, maxiters, inliers_thresh, nNeeded, error_thresh, K, Kinv);

}

}

void SVDFitR::operator ()(SVDRSolverParams& params, AtomPair& pair) const
{

  if (params.nNeeded < 0 && pair.rays1().size() >= 2)
  {
    svdSolveR(pair.rays1(), pair.rays2(), pair.result(), params.maxiters, params.inliers_thresh, params.nNeeded,
                  params.error_thresh, pair.atom1()->camera().K(), pair.atom1()->camera().Kinv());
    return;
  }
  if (pair.rays1().size() < params.inliers_thresh)
    return;
  ransacSolveR(pair.rays1(), pair.rays2(), pair.result(), params.maxiters, params.inliers_thresh, params.nNeeded,
               params.error_thresh, pair.atom1()->camera().K(), pair.atom1()->camera().Kinv());
}

}
