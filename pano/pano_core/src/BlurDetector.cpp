/*
 * BlurDetector.cpp
 *
 *  Created on: Oct 24, 2010
 *      Author: ethan
 */

#include "pano_core/BlurDetector.h"

#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
using namespace cv;

using namespace std;

namespace pano
{

BlurDetector::BlurDetector() :
  grad_max_(125)
{
}

BlurDetector::~BlurDetector()
{
}

double BlurDetector::checkBlur(const Mat& img_grey)
{
  int blur_ROI = 200;
  Rect roi(img_grey.cols / 2 - blur_ROI, img_grey.rows / 2 - blur_ROI, 2 * blur_ROI, 2 * blur_ROI);

  if (img_grey.channels() != 1)
  {
    cv::cvtColor(img_grey(roi), img_cache, CV_RGB2GRAY);
    img_cache.convertTo(grey_, CV_8U, 1.f / 4.0);
  }
  else
  {
    img_grey(roi).convertTo(grey_, CV_8U, 1.f / 4.0);
  }
  Laplacian(grey_, grad_abs_, CV_8U);
  double max;
  cv::minMaxLoc(grad_abs_, 0, &max);

  double pofblur = 1 - (max * 0.42) / (grad_max_);

  // update our internal state, statistics of gradient
  grad_max_ = grad_max_ * 0.9 + max * 0.1;

  // if this is more than ~0.6 it is almost certainly blurred!
  return pofblur;
}

namespace
{

cv::Mat src_float = cv::Mat();

}

void sharpen_backwards_heat_equation(const cv::Mat& src, cv::Mat& dst, float alpha)
{

  // parameter alpha \in [0,1]
  // run backwards heat equation for short while
  // strongly suggested to use CV_32FC3 , need float accuracy for laplacian

  if (src.type() != CV_32FC3)
  {
    src.convertTo(src_float, CV_32FC3);
  }
  dst = cv::Mat(src_float.size(), CV_32FC3);

  int kernel_sz = 3;
  cv::Laplacian(src, dst, dst.depth(), kernel_sz);
  dst = -alpha * dst + src;

  // heat equation:  I_t = I_xx + I_yy
  // run it backwards!
  // I_0 - I_{-alpha} = I_xx + I_yy
  // I_{-alpha} = -alpha*Laplacian(I)  + I

}

}
