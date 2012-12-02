/*
 * CaptureEngine.cpp
 *
 *  Created on: Oct 22, 2010
 *      Author: erublee
 */

#include "pano_core/CaptureEngine.h"
#include "pano_core/panoutils.h"

#include <opencv2/highgui/highgui.hpp>
#include<opencv2/legacy/legacy.hpp>

using namespace cv;
using namespace std;
namespace pano
{
namespace
{
void onMatchCB(const AtomPair& pair)
{
  static Mat matches_img;
  pair.atom2()->images().src().copyTo(matches_img);
  if (!pair.result().success())
    bitwise_not(matches_img, matches_img);
  drawMatchesRelative(pair.atom1()->features(), pair.atom2()->features(), pair.matches(), matches_img,
                      pair.result().inlier_mask());
  imshow("prior matches", matches_img);
}
}

PriorTracker::PriorTracker(int max_fail_count) :
  max_fail_count_(max_fail_count), fail_count_(1)
{

}
Extrinsics PriorTracker::track(ImageAtom& query, ModelFitter& fitter, FitterResult* result)
{

  if (result != NULL)
  {
    *result = FitterResult();
  }

  if (prior_.features().kpts().empty())
  {
    prior_ = query;
    return query.extrinsics();
  }
  AtomPair pair = MoleculeProcessor::matchwithFitter(prior_.ptrToSelf(), query.ptrToSelf(), fitter);
  Extrinsics ext = pair.generateExtrinsics(query.ptrToSelf());

  callbacks_.callBack(pair, MATCHES_CALLBACK);

  if (pair.result().success())
  {
    if (angularDist(prior_.extrinsics(), query.extrinsics()) > CV_PI / 10)
    {
      prior_ = query;
      prior_.extrinsics() = ext;
    }
  }
  //  else
  //  {
  //    fail_count_++;
  //    if (fail_count_ > max_fail_count_)
  //    {
  //      prior_ = query;
  //    }
  //  }

  if (result != NULL)
  {
    *result = pair.result();
  }

  return ext;

}
void PriorTracker::updatePrior(const ImageAtom& prior)
{
  prior_ = prior;
}
CaptureEngine::CaptureEngine(cv::Ptr<ModelFitter> fitter, cv::Ptr<cv::FeatureDetector> detector, Camera camera,
                             const std::string& dirname) :
  fitter_(fitter), detector_(detector), camera_(camera), atom_(camera, Images()), fail_count_(1), dirname_(dirname)
{
  atom_.extrinsics() = Extrinsics(Mat::eye(3, 3, CV_32F), 1);
  atom_.extrinsics().flag(Extrinsics::ESTIMATED) = true;
  prior_tracker_.addMatchesCallback(&onMatchCB);
}

CaptureEngine::~CaptureEngine()
{

}

cv::Ptr<ImageAtom> CaptureEngine::onNewFrame(const cv::Mat& frame)
{
  if (!frame.empty())
  {
    // cout << "frame in capture engine: (W,H) = " << frame.cols << "," << frame.rows << endl;
  }
  else
  {
    cerr << "empty frame in capture engine! " << endl;
    return 0;
  }
  FitterResult fit_result;
  std::list<AtomPair> result;
  Ptr<ImageAtom> added_atom_really;

  atom_.images().load(frame);
  atom_.detect(*detector_);
  atom_.extract(BriefDescriptorExtractor(), BruteForceMatcher<Hamming> ());
  //tracker will return an estimated R by matching against a prior that it stores.
  atom_.extrinsics() = prior_tracker_.track(atom_, *fitter_, &fit_result);
  if (atom_.extrinsics().flag(Extrinsics::ESTIMATED))
  {
    //add the atom to the glob, its the first one
    if (glob_.getMolecules().empty())
    {
      glob_.addAtomToGlob(fitter_, atom_);
    }
    else
    {
      fail_count_ = 1;
      //find the closet atom in the glob
      Ptr<ImageAtom> min_atom = glob_.minDistAtom(atom_);

      //if the angle between the observation and the closest in the map is greater than some epsilon
      //try to add the observation to the map
      if (angularDist(atom_.extrinsics(), min_atom->extrinsics()) > atom_.camera().fov_max() / 4)
      {
        cout << "trying to add to map.." << endl;
        Ptr<ImageAtom> added_atom = glob_.queryAtomToGlob(fitter_, atom_, result);
        if (!result.empty())
        {
          cout << "added new image to map" << endl;
          glob_.addPrefittedPairs(result, added_atom);
          glob_.batchFindAndSetTrinsics();
          atom_.extrinsics() = added_atom->extrinsics();
          //update the prior to the image that was added, as this will be the best image
          prior_tracker_.updatePrior(*added_atom);
          added_atom_really = added_atom;
        }

      }
      else
      {
        //if the atom wasn't added then check the confidence level and update prior as necessary.
        //confidence should be roughly equivilant to an accumulated pixel error, big is worse
        if (atom_.extrinsics().val(Extrinsics::CONFIDENCE) > 20)
        {
          cout << "confidence " << atom_.extrinsics().val(Extrinsics::CONFIDENCE);//<<endl;
          cout << " switching out prior" << endl;
          prior_tracker_.updatePrior(*min_atom);
        }
        else
          prior_tracker_.updatePrior(atom_);
      }

    }

  }
  else
  {
    if (fail_count_++ % FAIL_MAX == 0)
    {
      cout << "lost localization! " << endl;

      //this function returns a pointer to an atom that might be added and fills out a list
      //of pairwise matches between matom and the glob atoms
      Ptr<ImageAtom> added_atom = glob_.queryAtomToGlob(fitter_, atom_, result);

      //this means that there are pairs that were successfully matched
      if (!result.empty())
      {
        cout << "localized" << endl;
        //if its new territory, add it to the map
        if (glob_.minDistToAtom(*added_atom) > atom_.camera().fov_max() / 4)
        {
          cout << "added new image to map" << endl;
          glob_.addPrefittedPairs(result);
          glob_.batchFindAndSetTrinsics();
          added_atom_really = added_atom;
        }
        atom_.extrinsics() = added_atom->extrinsics();
        prior_tracker_.updatePrior(*added_atom);
      }
    }

  }
  return added_atom_really;
}

void CaptureEngine::reset()
{
  glob_.reset();
  atom_.extrinsics() = Extrinsics(Mat::eye(3, 3, CV_32F), 1);
  atom_.extrinsics().flag(Extrinsics::ESTIMATED) = true;
  prior_tracker_.updatePrior(atom_);
}

}
