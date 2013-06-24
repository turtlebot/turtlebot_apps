/*
 * matching_test.cpp
 *
 *  Created on: Oct 17, 2010
 *      Author: ethan
 */

#include "pano_core/ImageAtom.h"
#include "pano_core/feature_utils.h"
#include "pano_core/QuadTree.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <list>
#include <string>

using namespace std;
using namespace cv;
using namespace pano;

list<string> getImageList(istream& input)
{
  list<string> imlist;
  while (!input.eof() && input.good())
  {
    string imname;
    input >> imname;
    if (!input.eof() && input.good())
      imlist.push_back(imname);
  }
  return imlist;
}



void estimateH(const cv::Mat& w, const cv::Mat& R, const cv::Mat& K, const cv::Mat& Kinv, cv::Mat& R_w,
               cv::Mat & R_new, cv::Mat& H)
{
  Rodrigues(w, R_w);
  R_new = R_w * R;
  H = K * R_new * Kinv;
}

void getSmallImage(const cv::Mat& src, cv::Mat& dest, float factor = 10.0f)
{
  resize(src, dest, Size(src.size().width / factor, src.size().height / factor), 0, 0, CV_INTER_AREA);
}

void getGradientImage(const cv::Mat& src, cv::Mat&dest)
{
  Sobel(src, dest, CV_32F, 1, 1);
}


class ESM_data
{
public:

  ESM_data(const Mat&ref, const Mat& live, const Mat& K, const Mat& Kinv) :
    w_(Mat::zeros(3, 1, CV_32F)), R_(Mat::eye(3, 3, CV_32F)), R_new_(Mat::eye(3, 3, CV_32F)),
        H_(Mat::eye(3, 3, CV_32F)), grad_norm_(10.0e6), ref_(ref), live_(live), K_(K), Kinv_(Kinv)
  {
    getGradientImage(ref, grad_ref_);
  }

  void estimageRandH(Mat& R, Mat& H, int iters)
  {

  }
private:
  void updateW()
  {

  }
  void updateWarp()
  {
    Rodrigues(w_, R_w_,w_jacobian_);
    R_new_ = R_w_ * R_;
    H_ = K_ * R_new_ * Kinv_;
    warpPerspective(live_, warped_, H_, live_.size());
  }
  void updateGradient()
  {
     grad_diff_ = grad_ref_ - grad_live_;
     multiply(grad_diff_, grad_diff_, grad_diff_);
     grad_norm_ = sum(grad_diff_)[0];
  }

  Mat w_;
  Mat w_jacobian_;
  Mat R_w_;
  Mat R_;
  Mat R_new_;
  Mat H_;

  Mat grad_diff_;
  Mat grad_ref_;
  Mat grad_live_;
  Mat warped_;

  float grad_norm_;

  static const Mat EYE;
  const Mat ref_;
  const Mat live_;
  const Mat K_;
  const Mat Kinv_;

};

const Mat ESM_data::EYE = Mat::eye(3, 3, CV_32F);

int main(int ac, char ** av)
{

  if (ac != 3)
  {
    cerr << "usage : " << av[0] << " directory imagelist.txt" << endl;
    return 1;
  }

  std::ifstream input(av[2]);
  std::string directory = av[1];

  cout << "directory " << directory << endl;
  std::list<string> img_names = getImageList(input);

  if (img_names.empty())
  {
    cerr << av[2] << "does not contain a list of images" << endl;
    return 1;
  }
  ImageAtom atom;

  atom.camera().setCameraIntrinsics("data/camera.yml");
  atom.images().load(img_names.back(), directory);
  img_names.pop_back();

  BriefDescriptorExtractor brief(32);

  Ptr<cv::AdjusterAdapter> adapter = new FastAdjuster();
  Ptr<FeatureDetector> detector(new DynamicAdaptedFeatureDetector( adapter, 400, 800, 20) );

  BruteForceMatcher<HammingLUT> matcher;
  atom.detect( *detector );
  atom.extract(brief,matcher);


  Mat frame;

  vector<DMatch> matches;

  //  SpatialPriorMatcher matcher;
  //  matcher.setMatcher(new BruteForceMatcher<HammingLUT>(),true);


  Features train = atom.features();
  //matcher.addFeatures(&train);

  atom.draw(&frame, ImageAtom::DRAW_FEATURES);
  imshow("frame", frame);
  waitKey();

  BruteForceMatcher<HammingLUT> desc_matcher;

  Mat ref, live;

  getSmallImage(atom.images().src(), ref);

  while (!img_names.empty())
  {

    string image_name = img_names.back();
    img_names.pop_back();

    atom.images().load(image_name, directory);
    getSmallImage(atom.images().grey(), live);

    ESM_data esm(ref, live, atom.camera().K(), atom.camera().Kinv());


    waitKey(1000);

  }
  return 0;
}
