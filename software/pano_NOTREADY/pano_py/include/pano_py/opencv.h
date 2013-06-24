/*
 * opencv.h
 *
 *  Created on: Dec 19, 2010
 *      Author: ethan
 */

#ifndef PANO_PY_OPENCV_H_
#define PANO_PY_OPENCV_H_
#include <opencv2/core/core.hpp>
#include <boost/python.hpp>
namespace pano_py{
/** \brief takes a cv.cvmat or cv.iplimage from the opencv wrappings and turns it into
 * a cv::Mat for convenient use.
 *
 */
cv::Mat convertObj2Mat(boost::python::object image);

cv::Mat convertNumpy2Mat(boost::python::object np);

bool numpy_to_mat(const PyObject* o, cv::Mat& m, const char* name = "<unknown>", bool allowND=true);

int failmsg(const char *fmt, ...);



}


#endif /* OPENCV_H_ */
