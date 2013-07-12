/*
 * CaptureEngine.h
 *
 *  Created on: Oct 22, 2010
 *      Author: erublee
 */

#ifndef CAPTUREENGINE_H_
#define CAPTUREENGINE_H_

#include "pano_core/BlurDetector.h"
#include "pano_core/Camera.h"
#include "pano_core/MoleculeProcessor.h"

#include "opencv2/features2d/features2d.hpp"
#include "pano_core/callbacks.h"

namespace pano
{

class PriorTracker
{
public:
  PriorTracker(int max_fail_count = 3);
  Extrinsics track(ImageAtom& query, ModelFitter& fitter, FitterResult* result = NULL);
  void updatePrior(const ImageAtom& prior);

  /** \brief this is a function that will be called back when the match has occurred.
   *
   * The function should be able to be called the following way:
   * f(pair);
   * where pair is a const AtomPair&
   *
   */
  template<typename Function>
  void addMatchesCallback(const Function& f){
    callbacks_.addCallback<AtomPair>(MATCHES_CALLBACK,f);
  }

  /**
   *  function should take as an argument a cv::Ptr<ImageAtom>
   */
  template<typename Function>
  void addPriorUpdateCallback(const Function& f){
    callbacks_.addCallback<cv::Ptr<ImageAtom> >(PRIOR_UPDATE_CB,f);
  }

private:
  enum {
    MATCHES_CALLBACK = 0,
        PRIOR_UPDATE_CB
  };
  ImageAtom prior_;
  int max_fail_count_;
  int fail_count_;
  CallbackEngine callbacks_;

};

class CaptureEngine
{
public:
  CaptureEngine():fail_count_(1){}
  CaptureEngine(cv::Ptr<ModelFitter> fitter, cv::Ptr<cv::FeatureDetector> detector, Camera camera,
                const std::string& dirname);
  virtual ~CaptureEngine();

  virtual cv::Ptr<ImageAtom> onNewFrame(const cv::Mat& img);

  MoleculeGlob& glob()
  {
    return glob_;
  }
  const MoleculeGlob& glob() const
  {
    return glob_;
  }
  const ImageAtom& latestAtom() const{
    return atom_;
  }

  void reset();

  enum {FAIL_MAX = 4};

protected:
  cv::Ptr<ModelFitter> fitter_;
  cv::Ptr<cv::FeatureDetector> detector_;
  Camera camera_;
  ImageAtom atom_;
  MoleculeGlob glob_;
  PriorTracker prior_tracker_;
  int fail_count_;
  std::string dirname_;
  BlurDetector blur_detector_;
};
}
#endif /* CAPTUREENGINE_H_ */
