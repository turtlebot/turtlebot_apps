/*
 * matching_test.cpp
 *
 *  Created on: Oct 17, 2010
 *      Author: ethan
 */

#include "pano_core/ImageAtom.h"
#include "pano_core/feature_utils.h"
#include "pano_core/MoleculeProcessor.h"
#include "pano_core/CaptureEngine.h"
#include "pano_core/ModelFitter.h"
#include "pano_core/Blender.h"
#include "pano_core/panoutils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pano_core/BlurDetector.h"

#include <sstream>
#include <iostream>
#include <list>

using namespace std;
using namespace cv;
using namespace pano;

//void onMatchCB(const AtomPair& pair)
//{
//  static Mat matches_img;
//  pair.atom2()->images().src().copyTo(matches_img);
//  if (!pair.result().success())
//    bitwise_not(matches_img, matches_img);
//  drawMatchesRelative(pair.atom1()->features(), pair.atom2()->features(), pair.matches(), matches_img,
//                      pair.result().inlier_mask());
//  imshow("prior matches", matches_img);
//}

int main(int ac, char ** av)
{

  if (ac != 4)
  {
    cout << "usage: " << av[0] << " camera.yml <video device number> outputdir" << endl;
    return 1;
  }

  string output_dir = av[3];

  Camera camera;
  camera.setCameraIntrinsics(av[1]);

  BriefDescriptorExtractor brief(32);

  Ptr<FeatureDetector> detector(new GriddedDynamicDetectorAdaptor(400,3, 4, 4,FastAdjuster()));

  VideoCapture capture;
  capture.open(atoi(av[2]));
  if (!capture.isOpened())
  {
    cout << "capture device failed to open!" << endl;
    return 1;
  }

  cout << "following keys do stuff:" << endl;
  cout << "l : starts capturing the live preview ##DO THIS TO START##" << endl << endl;
  cout << "r : resets the pano" << endl;
  cout << "i/I : lower/raise inlier thresh" << endl;
  cout << "e/E : lower/raise error thresh" << endl;
  cout << "m/M : lower/raise max iterations" << endl;
  cout << "s : serialize the pano data" << endl;
  cout << "c : capture as an avi the taking of the pano" << endl;
  cout << "b : blends the pano" << endl;
  cout << "q : quit" << endl;

  Mat frame;

  bool ref_live = false;

  SVDRSolverParams params;
  params.error_thresh = 4;
  params.inliers_thresh = 10;
  params.maxiters = 25;
  params.nNeeded = 2;

  Ptr<SVDFitter> svdfitter(new SVDFitter(params));

  Ptr<ModelFitter> fitter(reinterpret_cast<const Ptr<ModelFitter>&> (svdfitter));

  Mat outimage(Size(800, 400), CV_8UC3);

  BlenderAlpha blender;

  Mat glob_out = Mat::zeros(Size(800, 400), CV_8UC3);
  Mat blended_out = Mat::zeros(Size(1500, 750), CV_8UC3);

  namedWindow("blended", CV_WINDOW_KEEPRATIO);

  VideoWriter video_writer;

  CaptureEngine capture_engine(fitter, detector, camera, output_dir);

  //the glob stores the pano graph
  //think of it as the map
  MoleculeGlob& glob = capture_engine.glob();

  BlurDetector blur_detector;

  int f_count = 0;
  double total_t;
  for (;;)
  {

    capture >> frame;
    if (frame.empty())
      continue;

    if (ref_live)
    {

      f_count++;
      double t = getTickCount();

      cv::Mat frame_;
      sharpen_backwards_heat_equation(frame.clone(), frame_);
      frame_.convertTo( frame, frame.type() );
      Ptr<ImageAtom> added = capture_engine.onNewFrame(frame);


      if (!added.empty())
      {


        blender.BlendMolecule(*glob.getBiggestMolecule(), glob_out);
      }



      //copy the map pano, so that the current image doesn't "paint" the more perfect map pano
      glob_out.copyTo(outimage);

      //draw the current observed frame to the output
      blender.blendIncremental(capture_engine.latestAtom(), outimage);

      imshow("blended", outimage);

      //capture the current output to the video file
      if (video_writer.isOpened())
        video_writer << outimage;

      //frame timing
      total_t += ((double)getTickCount() - t) / getTickFrequency();

      if (f_count % 30 == 0)
      {
        cout << "estimated W" << capture_engine.latestAtom().extrinsics().mat(Extrinsics::W) << endl;
        cout << "total tracking time per frame: " << total_t * 1000.0f / f_count << " milliseconds, averaged over "
            << f_count << " frames" << endl;
        total_t = 0;
        f_count = 0;
      }

    }
    char key = 0xFF & waitKey(2);

    switch (key)
    {
      case 'c':
        video_writer.open(output_dir + "/video_capture.avi", CV_FOURCC('H', 'F', 'Y', 'U')/*codec*/, 30/*fps*/,
                          outimage.size(), true/*color*/);
        break;
      case 'C':
        // video
        break;
      case 'l':
        ref_live = true;
        break;
      case 'b':
        cout << "blending...";
        cout.flush();
        blender.BlendMolecule(*glob.getBiggestMolecule(), blended_out);
        imwrite(output_dir + "/blended.jpg", blended_out);
        cout << " done blending. look at " << output_dir + "/blended.jpg" << endl;
        break;
      case 'r':
        cout << "reseting pano" << endl;
        glob.reset();
        glob_out = Scalar::all(0);
        break;
      case 'e':
        params.error_thresh -= 0.2;
        cout << "new error_thresh " << params.error_thresh << endl;
        break;
      case 'E':
        params.error_thresh += 0.2;
        cout << "new error_thresh " << params.error_thresh << endl;
        break;
      case 'i':
        params.inliers_thresh--;
        cout << "new inliers_thresh " << params.inliers_thresh << endl;
        break;
      case 'I':
        params.inliers_thresh++;
        cout << "new inliers_thresh " << params.inliers_thresh << endl;
        break;
      case 'm':
        params.maxiters--;
        cout << "new maxiters " << params.maxiters << endl;
        break;
      case 'M':
        params.maxiters++;
        cout << "new maxiters " << params.maxiters << endl;
        break;
      case 'g':
        //dont_check_blur = false;
        break;
      case 'G':
       //dont_check_blur = true;
        break;
      case 's':
      {
        FileStorage fs(output_dir + "/glob.yml.gz", FileStorage::WRITE);
        fs << "glob";
        glob.serialize(fs);
        fs.release();
      }
        break;
      case 27:
      case 'q':
        return 0;
        break;
      default:
        break;
    }

    //update the fitter wiht any params that changed
    (*svdfitter) = SVDFitter(params);

  }
  return 0;
}
