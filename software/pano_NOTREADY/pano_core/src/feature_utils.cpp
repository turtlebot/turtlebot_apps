/*
 * feature_utils.cpp
 *
 *  Created on: Oct 17, 2010
 *      Author: ethan
 */

#include <pano_core/feature_utils.h>
#include <iostream>
using namespace cv;
using namespace std;
namespace pano
{


void GriddedDynamicDetectorAdaptor::detectImpl(const Mat &image, vector<KeyPoint> &keypoints, const Mat &mask) const
{
  keypoints.clear();
  keypoints.reserve(maxTotalKeypoints);

  for (int i = 0; i < gridRows; ++i)
  {
    Range row_range((i * image.rows) / gridRows, ((i + 1) * image.rows) / gridRows);
    for (int j = 0; j < gridCols; ++j)
    {
      Range col_range((j * image.cols) / gridCols, ((j + 1) * image.cols) / gridCols);
      Mat sub_image = image(row_range, col_range);
      Mat sub_mask;
      if (!mask.empty())
        sub_mask = mask(row_range, col_range);

      vector<KeyPoint> sub_keypoints;
      detectors_[i * gridCols + j]->detect(sub_image, sub_keypoints, sub_mask);

      for (std::vector<cv::KeyPoint>::iterator it = sub_keypoints.begin(), end = sub_keypoints.end(); it != end; ++it)
      {
        it->pt.x += col_range.start;
        it->pt.y += row_range.start;
      }

      keypoints.insert(keypoints.end(), sub_keypoints.begin(), sub_keypoints.end());
    }
  }
}

void KeyPointsToPoints(const KeypointVector& keypts, std::vector<cv::Point2f>& pts)
{
  pts.clear();
  pts.reserve(keypts.size());
  for (size_t i = 0; i < keypts.size(); i++)
  {
    pts.push_back(keypts[i].pt);
  }
}

void PointsToKeyPoints(const std::vector<cv::Point2f>& pts, KeypointVector& kpts)
{
  kpts.clear();
  kpts.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); i++)
  {
    kpts.push_back(KeyPoint(pts[i], 6.0));
  }
}

void matches2points(const KeypointVector& train, const KeypointVector& query, const MatchesVector& matches,
                    std::vector<cv::Point2f>& pts_train, std::vector<Point2f>& pts_query)
{

  pts_train.clear();
  pts_query.clear();
  pts_train.reserve(matches.size());
  pts_query.reserve(matches.size());

  size_t i = 0;

  for (; i < matches.size(); i++)
  {

    const DMatch & dmatch = matches[i];

    if(dmatch.queryIdx > int( query.size()) || dmatch.trainIdx < 0){
      std::cerr << "bad index , query:" << dmatch.queryIdx << std::endl;
      continue;
    }
    if(dmatch.trainIdx > int( train.size()) || dmatch.trainIdx < 0){
      std::cerr << "bad index ,train:" << dmatch.trainIdx << std::endl;
      continue;
    }
    pts_query.push_back(query[dmatch.queryIdx].pt);
    pts_train.push_back(train[dmatch.trainIdx].pt);

  }

}

}


///** an adaptively adjusting detector parameters
// * shift params to get desired number of features
// */
//template<typename Adjuster>
//class DynamicDetectorAdaptor: public cv::FeatureDetector {
//public:
//
//	DynamicDetectorAdaptor(int min_features = 400, int max_features = 600,
//			int max_iters = 4, const Adjuster& a = Adjuster()) :
//		escape_iters_(max_iters), min_features_(min_features), max_features_(
//				max_features), adjuster_(a) {
//	}
//protected:
//	virtual void detectImpl(const cv::Mat& image,
//			std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask =
//					cv::Mat()) const {
//		//for oscillation testing
//		bool down = false;
//		bool up = false;
//
//		//flag for whether the correct threshhold has been reached
//		bool thresh_good = false;
//
//		//this is bad but adjuster should persist from detection to detection
//		Adjuster& adjuster = const_cast<Adjuster&> (adjuster_);
//
//		//break if the desired number hasn't been reached.
//		int iter_count = escape_iters_;
//
//		do {
//			keypoints.clear();
//
//			//the adjuster takes care of calling the detector with updated parameters
//			adjuster.detect(image, mask, keypoints);
//
//			if (int(keypoints.size()) < min_features_) {
//				down = true;
//				adjuster.tooFew(min_features_, keypoints.size());
//			} else if (int(keypoints.size()) > max_features_) {
//				up = true;
//				adjuster.tooMany(max_features_, keypoints.size());
//			} else
//				thresh_good = true;
//		} while (--iter_count >= 0 && !(down && up) && !thresh_good
//				&& adjuster.good());
//	}
//
//private:
//	int escape_iters_;
//	int min_features_, max_features_;
//	Adjuster adjuster_;
//};
//
//struct FastAdjuster {
//	FastAdjuster() :
//		thresh_(20) {
//	}
//	void detect(const cv::Mat& img, const cv::Mat& mask, std::vector<
//			cv::KeyPoint>& keypoints) const {
//		cv::FastFeatureDetector(thresh_, true).detect(img, keypoints, mask);
//	}
//	void tooFew(int min, int n_detected) {
//		//fast is easy to adjust
//		thresh_--;
//	}
//	void tooMany(int max, int n_detected) {
//		//fast is easy to adjust
//		thresh_++;
//	}
//
//	//return whether or not the threshhold is beyond
//	//a useful point
//	bool good() const {
//		return (thresh_ > 1) && (thresh_ < 200);
//	}
//	int thresh_;
//};
//
//struct StarAdjuster {
//	StarAdjuster() :
//		thresh_(30) {
//	}
//	void detect(const cv::Mat& img, const cv::Mat& mask, std::vector<
//			cv::KeyPoint>& keypoints) const {
//		cv::StarFeatureDetector detector_tmp(16, thresh_, 10, 8, 3);
//		detector_tmp.detect(img, keypoints, mask);
//	}
//	void tooFew(int min, int n_detected) {
//		thresh_ *= 0.9;
//		if (thresh_ < 1.1)
//			thresh_ = 1.1;
//	}
//	void tooMany(int max, int n_detected) {
//		thresh_ *= 1.1;
//	}
//
//	//return whether or not the threshhold is beyond
//	//a useful point
//	bool good() const {
//		return (thresh_ > 2) && (thresh_ < 200);
//	}
//	double thresh_;
//};
//
//struct SurfAdjuster {
//	SurfAdjuster() :
//		thresh_(400.0) {
//	}
//	void detect(const cv::Mat& img, const cv::Mat& mask, std::vector<
//			cv::KeyPoint>& keypoints) const {
//		cv::SurfFeatureDetector detector_tmp(thresh_);
//		detector_tmp.detect(img, keypoints, mask);
//	}
//	void tooFew(int min, int n_detected) {
//		thresh_ *= 0.9;
//		if (thresh_ < 1.1)
//			thresh_ = 1.1;
//	}
//	void tooMany(int max, int n_detected) {
//		thresh_ *= 1.1;
//	}
//
//	//return whether or not the threshhold is beyond
//	//a useful point
//	bool good() const {
//		return (thresh_ > 2) && (thresh_ < 1000);
//	}
//	double thresh_;
//};
//
//typedef DynamicDetectorAdaptor<FastAdjuster> FASTDynamicDetector;
//typedef DynamicDetectorAdaptor<StarAdjuster> StarDynamicDetector;
//typedef DynamicDetectorAdaptor<SurfAdjuster> SurfDynamicDetector;
