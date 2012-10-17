/*
 * feature_utils.h
 *
 *  Created on: Oct 17, 2010
 *      Author: ethan
 */

#ifndef FEATURE_UTILS_H_
#define FEATURE_UTILS_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <limits>
#include <string>
#include <vector>
#include <cmath>

namespace pano {
template<typename T, const int NORM_TYPE = cv::NORM_L2>
struct _RadiusPoint {
	const float r;
	const T&pt2;
	float n;
	_RadiusPoint(float r, const T&pt2) :
		r(r), pt2(pt2), n(0) {
	}
	bool cp(const T& pt1) {
		n = norm(cv::Mat(pt1), cv::Mat(pt2), NORM_TYPE);
		return r > n;
	}

};
template<typename T>
_RadiusPoint<T> RadiusPoint(float r, const T&pt2) {
	return _RadiusPoint<T> (r, pt2);
}

inline bool operator==(const cv::Point2f& pt1, _RadiusPoint<cv::Point2f> pt2) {
	return pt2.cp(pt1);
}
inline bool operator==(const cv::Point3f& pt1, _RadiusPoint<cv::Point3f> pt2) {
	return pt2.cp(pt1);
}

/** store list of key points from 'feature detection'
 */
typedef std::vector<cv::KeyPoint> KeypointVector;

/** store list of 'match points' and 'match scores' between two sets of keypoints
 */
typedef std::vector<cv::DMatch> MatchesVector;

/** convert from a vector of 'keypoints' with N-d data to 2d  xy points
 */
void KeyPointsToPoints(const KeypointVector& keypts,
		std::vector<cv::Point2f>& pts);
void PointsToKeyPoints(const std::vector<cv::Point2f>& keypts,
		KeypointVector& pts);

/** convert from a vector of 'match points' with N-d data to 2d  xy points
 */
void matches2points(const KeypointVector& train, const KeypointVector& query,
		const MatchesVector& matches, std::vector<cv::Point2f>& pts_train,
		std::vector<cv::Point2f>& pts_query);

inline float scorematch(const std::vector<cv::DMatch>& matches) {
	float s = 0;
	std::vector<cv::DMatch>::const_iterator it = matches.begin();
	while (it != matches.end()) {
		s += it->distance;
		++it;
	}
	return s / matches.size();
}


class GriddedDynamicDetectorAdaptor: public cv::FeatureDetector {
public:
	template<typename Adjuster>
	GriddedDynamicDetectorAdaptor(int max_total_keypoints, int escape_iters_per_cell, int _gridRows,
			int _gridCols, const Adjuster& adjuster) :
		maxTotalKeypoints(max_total_keypoints), gridRows(_gridRows), gridCols(
				_gridCols), detectors_(gridCols * gridRows) {
		int maxPerCell = maxTotalKeypoints / (gridRows * gridCols);
		for (int i = 0; i < (int) detectors_.size(); ++i) {
			detectors_[i] = new cv::DynamicAdaptedFeatureDetector(new Adjuster(adjuster),maxPerCell
					* 0.8, maxPerCell * 1.2,escape_iters_per_cell);
		}
	}
	virtual ~GriddedDynamicDetectorAdaptor() {
	}

protected:
	virtual void detectImpl(const cv::Mat& image,
			std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask =
					cv::Mat()) const;
	int maxTotalKeypoints;
	int gridRows;
	int gridCols;
	std::vector<cv::Ptr<FeatureDetector> > detectors_;
};

}

#endif /* FEATURE_UTILS_H_ */
