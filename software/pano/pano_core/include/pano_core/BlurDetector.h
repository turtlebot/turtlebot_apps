/*
 * BlurDetector.h
 *
 *  Created on: Oct 24, 2010
 *      Author: ethan
 */

#ifndef BLURDETECTOR_H_
#define BLURDETECTOR_H_

#include <opencv2/core/core.hpp>

namespace pano
{

/** apply 'unknown blur' deblurring. for efficiency, write to
    the source back in it's format.
  */
void sharpen_backwards_heat_equation( const cv::Mat& src,
                                      cv::Mat& dst, float alpha = .02 );

class BlurDetector
{
public:
  BlurDetector();
  virtual ~BlurDetector();

  /** \brief determine if image is blurry
   *  return probability of blur.  If above .6 then is > %60 chance of blur;
   */
  virtual double checkBlur(const cv::Mat& img_input);
protected:
  double grad_max_;
  cv::Mat grad_abs_;
  cv::Mat grey_;
  cv::Mat grad_x_;
  cv::Mat grad_y_;
  cv::Mat img_cache;
};

}

#endif /* BLURDETECTOR_H_ */
