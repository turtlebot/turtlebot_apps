/*
 * molecule_builder.cpp
 *
 *  Created on: Oct 20, 2010
 *      Author: erublee
 */
#include <opencv2/opencv.hpp>

#include <iostream>
#include <list>

#include "pano_core/ImageMolecule.h"
#include "pano_core/MoleculeProcessor.h"
#include "pano_core/ImageAtom.h"
#include "pano_core/feature_utils.h"
#include "pano_core/ModelFitter.h"

using namespace std;
using namespace cv;
using namespace pano;

int main(int ac, char ** av)
{

  if (ac != 3)
  {
    cout << "usage: " << av[0] << " camera.yml <video device number>" << endl;
    return 1;
  }

  Camera camera;
  camera.setCameraIntrinsics(av[1]);

  FileStorage fs("camera.serial.yml", FileStorage::WRITE);
  fs << "camera";
  camera.serialize(fs);

  BriefDescriptorExtractor brief(32);

  GriddedDynamicDetectorAdaptor detector(200, 3,2, 2, cv::FastAdjuster());

  VideoCapture capture;
  capture.open(atoi(av[2]));

  if (!capture.isOpened())
  {
    cout << "capture device failed to open!" << endl;
    return 1;
  }

  cout << "following keys do stuff:" << endl;
  cout << "t : grabs a reference frame to match against" << endl;
  cout << "l : makes the reference frame new every frame" << endl;
  cout << "q : quit" << endl;

  Mat frame;

  vector<DMatch> matches;

  BruteForceMatcher<Hamming> matcher;

  bool ref_live = true;

  SVDRSolverParams params;
  params.error_thresh = 4;
  params.inliers_thresh = 12;
  params.maxiters = 10;
  params.nNeeded = 3;

  Ptr<ModelFitter> fitter(new SVDFitter(params));

  Ptr<ImageAtom> prior(new ImageAtom()), atom(new ImageAtom());

  atom->camera() = camera;
  prior->camera() = camera;
  ref_live = true;
  ImageMolecule molecule;
  int fail_count =0;
  for (;;)
  {

    capture >> frame;
    if (frame.empty())
      continue;

    atom->images().load(frame);
    atom->detect(detector);
    atom->extract(brief, matcher);

    Mat f_atom, f_prior;

    if (!prior->features().descriptors().empty() && !atom->features().kpts().empty())
    {
      AtomPair atom_pair = MoleculeProcessor::matchwithFitter(prior, atom, fitter);
      if (atom_pair.result().success())
      {


        atom->extrinsics().mat(Extrinsics::R) = atom_pair.RofThis(atom);

        atom->extrinsics().flag(Extrinsics::ESTIMATED) = true;
        atom->extrinsics().val(Extrinsics::CONFIDENCE) = atom_pair.result().err();

        drawMatchesRelative(atom_pair.atom1()->features(), atom_pair.atom2()->features(), atom_pair.matches(), frame,
                            atom_pair.result().inlier_mask());
        fail_count = 0;
      }else{
        if(fail_count++ > 3){
          atom->extrinsics().flag(Extrinsics::ESTIMATED) = false;
          atom->extrinsics().mat(Extrinsics::R) = Mat::eye(3,3,CV_32FC1);
        }
      }

    }

    imshow("frame", frame);

    if (ref_live)
    {
      swap(prior, atom);
      atom->extrinsics() = prior->extrinsics();
    }

    char key = waitKey(2);

    switch (key)
    {
      case 'l':
        ref_live = true;
        break;
      case 't':
        ref_live = false;
        swap(prior, atom);
        atom->extrinsics() = prior->extrinsics();
        break;
      case 'q':
        return 0;
        break;
    }

  }
  return 0;
}
