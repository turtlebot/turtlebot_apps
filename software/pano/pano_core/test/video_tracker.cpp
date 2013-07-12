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
#include <opencv2/video/tracking.hpp>
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
void drawPointsRelative(const vector<Point2f>& train, const vector<Point2f>& query, Mat& img,
                        const vector<unsigned char>& mask = vector<unsigned char> ())
{
  for (int i = 0; i < (int)train.size(); i++)
  {

    if (mask.empty() || mask[i])
    {
      Point2f pt_new = query[i];
      Point2f pt_old = train[i];
      Point2f dist = pt_new - pt_old;

      cv::line(img, pt_new, pt_old, Scalar(255, 255, 255), 1);
      cv::circle(img, pt_new, 2, Scalar(255, 255, 255), 1);
    }

  }

}
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

  Ptr<FeatureDetector> detector(new DynamicAdaptedFeatureDetector(new FastAdjuster(), 20, 30, 1));//(new GriddedDynamicDetectorAdaptor(20, 3, 2, 2, FastAdjuster()));

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
  params.error_thresh = 5;
  params.inliers_thresh = 5;
  params.maxiters = 100;
  params.nNeeded = 2;

  Ptr<SVDFitter> svdfitter(new SVDFitter(params));

  Ptr<ModelFitter> fitter(reinterpret_cast<const Ptr<ModelFitter>&> (svdfitter));

  Mat outimage(Size(1000, 1000), CV_8UC3);

  BlenderAlpha blender;

  Mat glob_out = Mat::zeros(Size(2000, 1000), CV_8UC3);
  Mat blended_out = Mat::zeros(Size(1200, 600), CV_8UC3);

  namedWindow("blended", CV_WINDOW_KEEPRATIO);

  VideoWriter video_writer;

  CaptureEngine capture_engine(fitter, detector, camera, output_dir);

  //the glob stores the pano graph
  //think of it as the map
  MoleculeGlob& glob = capture_engine.glob();

  BlurDetector blur_detector;

  int f_count = 0;
  double total_t = 0;
  float scale(1);

  Camera ncamera(av[1]);
  // camera.scale(scale, scale);
  ImageAtom atom(camera, Images());
  atom.extrinsics() = Extrinsics(Mat::eye(3, 3, CV_32F), 10);
  atom.extrinsics().flag(Extrinsics::ESTIMATED) = true;
  ImageAtom prior;
  MatchesVector matches;
  vector<Point2f> currentPoints, previousPoints;
  vector<uchar> status;
  Mat ftrs;
  vector<float> err;
  Mat prev_small, curr_small;
  namedWindow("frame", CV_WINDOW_KEEPRATIO);
  namedWindow("sm_frame", CV_WINDOW_KEEPRATIO);
  ImageAtom batom(ncamera, Images());
  KeypointVector keypoints;
  for (;;)
  {

    capture >> frame;
    if (frame.empty())
      continue;
    double t = getTickCount();
   // atom.images().load(frame, "", ".");
    resize(frame, curr_small, Size(), scale, scale, CV_INTER_AREA);
    //    cv::Mat frame_;
    //    sharpen_backwards_heat_equation(curr_small.clone(), frame_,0.5);
    //    frame_.convertTo(curr_small, curr_small.type());

    imshow("sm_frame", curr_small);
    detector->detect(curr_small, keypoints);

    // drawKeypoints(atom.images().grey(), atom.features().kpts(), ftrs, Scalar::all(-1));
    //imshow("ftrs", ftrs);
    f_count++;
    if (ref_live)
    {
      if (previousPoints.size())
      {
        calcOpticalFlowPyrLK(prev_small, curr_small, previousPoints, currentPoints, status, err,
                             Size(20, 20),5, cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 2, 1), 0);
        //  prior.match(atom, matches, Mat(), -1);

        //float merr =  mean(Mat(err))[0];
        for (size_t i = 0; i < status.size(); i++)
        {
          if (!status[i])
          {
            swap(status[i], status.back());
            swap(previousPoints[i], previousPoints.back());
            swap(currentPoints[i], currentPoints.back());
            status.pop_back();
            previousPoints.pop_back();
            currentPoints.pop_back();
            i--;
          }
        }

        Mat mpts(previousPoints);
        Mat mptsq(currentPoints);
        mpts *= 1 / scale;
        mptsq *= 1 / scale;
        drawPointsRelative(previousPoints, currentPoints, frame);
        imshow("frame", frame);

        AtomPair pair(prior.ptrToSelf(), atom.ptrToSelf(), previousPoints, currentPoints);
        fitter->fit(pair);
        if (pair.result().success())
        {
          atom.extrinsics() = pair.generateExtrinsics(atom.ptrToSelf());
          // cout <<"tracking good" << endl;
        }
        else
        {
          cout << "missed frame" << endl;
          atom.extrinsics().flag(Extrinsics::ESTIMATED) = false;

        }
      }
     // blender.blendIncremental(atom,glob_out);
    //  imshow("blended",glob_out);

      KeyPointsToPoints(keypoints, previousPoints);
      swap(prev_small,curr_small);
      prior = atom;

    }

    //frame timing
    total_t += ((double)getTickCount() - t) / getTickFrequency();

    if (f_count % 100 == 0)
    {
      cout << "estimated W" << atom.extrinsics().mat(Extrinsics::W) << endl;
      cout << "total tracking time per frame: " << total_t * 1000.0f / f_count << " milliseconds, averaged over "
          << f_count << " frames" << endl;
      total_t = 0;
      f_count = 0;
    }

    char key = 0xFF & waitKey(2);

    switch (key)
    {
      case 'c':
        video_writer.open(output_dir + "/video_capture.avi", CV_FOURCC('H', 'F', 'Y', 'U')/*codec*/, 30/*fps*/,
                          glob_out.size(), true/*color*/);
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

        {
          static int c = 0;
          stringstream ss;
          ss << output_dir + "/blended" << c++ << ".jpg";
          imwrite(ss.str(), glob_out);
        }
        // blender.BlendMolecule(*glob.getBiggestMolecule(), blended_out);

        cout << " done blending. look at " << output_dir + "/blended.jpg" << endl;
        break;
      case 'r':
        cout << "reseting pano" << endl;
        // glob.reset();
        prior.extrinsics().mat(Extrinsics::R) = Mat_<float>::eye(3, 3);

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
