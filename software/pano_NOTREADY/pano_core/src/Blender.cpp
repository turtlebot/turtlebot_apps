/*
 * Blender.cpp
 *
 *  Created on: Jun 18, 2010
 *      Author: ethan
 */

#include "pano_core/Blender.h"
#include "pano_core/ModelFitter.h"
#include "pano_core/panoutils.h"
#if 0
#include "wavelet2d/wavelet2d.h"
#endif
#include "opencv2/highgui/highgui.hpp"
#include <iomanip>

using namespace cv;
using std::cout;
using std::endl;
using namespace pano;
namespace pano
{

BlenderAlpha::BlenderAlpha(int feather_edge) : cbe(0),
  feather_edge(feather_edge)
{

}

using namespace cv;
void initAlphaMat(const Size& sz, Mat& alpha, int feather_width)
{
  if (alpha.size() != sz || alpha.type() != CV_8UC3)
  {
    alpha = Mat::zeros(sz, CV_8UC3);
  }
  vector<uchar> _feather(feather_width);
  for (int i = 0; i < feather_width; i++)
  {
    _feather[i] = 1 + 254 * sin(i * CV_PI / (2 * feather_width));
  }

  for (int y = 0; y < alpha.rows; y++)
  {
    for (int x = 0; x < alpha.cols; x++)
    {
      int feather = min(min(x, y), min(alpha.cols - x, alpha.rows - y));
      uchar val = feather < feather_width ? _feather[feather] : 255;
      alpha.at<Vec3b> (y, x) = Vec3b(val, val, val);
    }
  }

}
BlenderAlpha::BlenderAlpha(int feather_edge, Size outputsize, Size inputsize) : cbe(0),
  feather_edge(feather_edge), outputsize(outputsize), projector(outputsize, Size(10, 5))
{
  setInputSize(inputsize);
}

void BlenderAlpha::setOutputSize(cv::Size size)
{
  if (outputsize != size)
  {
    outputsize = size;
    projector = SparseProjector(outputsize, Size(10, 5));

  }
}

void BlenderAlpha::setInputSize(cv::Size size)
{
  if (inputSize != size)
  {
    inputSize = size;
    initAlphaMat(inputSize, alpha, feather_edge);
    subtract(Scalar::all(255), alpha, one_minus_alpha);
  }
}

void BlenderAlpha::blendMolecule(const ImageMolecule& mol, cv::Size outputsize, const std::string& name_prefix)
{
  setOutputSize(outputsize);
  std::set<Ptr<ImageAtom> >::const_iterator atom = mol.getAtoms().begin();

  huge_image_.setSize(outputsize);
  output_prefix = name_prefix;
  for (; atom != mol.getAtoms().end(); ++atom)
  {

    Mat m;
    blendIncremental(**atom, m);
  }
  huge_image_.serialize("huge.yaml");
  //  Mat image = huge_image_.loadAll();
  //  //imshow("huge",image);
  //  imwrite("huge.jpg",image);
  // waitKey();
}

BlenderAlpha::~BlenderAlpha()
{

}

void alphaCompose(Mat& rgb1, const Mat& alpha, const Mat& one_minus_alpha, Mat& rgb_dest/*, const Mat &  mask = Mat()*/)
{
  multiply(rgb_dest, one_minus_alpha, rgb_dest, 1. / 255);
  multiply(rgb1, alpha, rgb1, 1. / 255);
  rgb_dest += rgb1;
}

void BlenderAlpha::blendIncremental(const ImageAtom& atom, cv::Mat& outimage)
{
  // assert(outimage.type() == atom.getImageraw().type());
  setOutputSize(outimage.size());
  assert((outimage.empty() && output_prefix.size() ) || outimage.type() == CV_8UC3);
  Mat img;
  if (atom.images().src().empty())
  {
    Images images = atom.images();
    images.restore();
    img = images.src();
  }
  else
    img = atom.images().src();

  if (cbe)
  {
    cbe->callBack(atom, 0);
  }
  setInputSize(img.size());

  Mat _img = img;
  Mat _R = atom.extrinsics().mat(Extrinsics::R);

  std::vector<int> roi_ids;

  //setup the projector
  projector.setSRandK(inputSize, _R, atom.camera().K(), roi_ids);

  std::string roi_name;
  for (int i = 0; i < (int)roi_ids.size(); i++)
  {

    int roi_id = roi_ids[i];
    Rect roi = projector.getRoi(roi_id);
    Mat roi_out;
    if (outimage.empty())
    {
      roi_name = huge_image_.addName(roi_id, output_prefix);
      huge_image_.addRoi(roi_id, roi);
      roi_out = imread(roi_name);
      if (roi_out.empty())
        roi_out = cv::Mat::zeros(roi.size(), CV_8UC3);
    }
    else
      roi_out = outimage(roi);

    //project the image onto a clean slate
    projector.projectMat(roi_id, _img, in_img, cv::BORDER_CONSTANT);

    projector.projectMat(roi_id, alpha, in_alpha, cv::BORDER_CONSTANT);

    projector.projectMat(roi_id, one_minus_alpha, in_one_minus_alpha, cv::BORDER_CONSTANT, Scalar::all(255));

    alphaCompose(in_img, in_alpha, in_one_minus_alpha, roi_out);
    if (outimage.empty())
    {
      imwrite(roi_name, roi_out);
    }
#if 0
    stringstream ss;
    ss << "roi" << roi_id;

    imshow(ss.str(), roi_out);
    waitKey(10);
#endif
  }

}

void BlenderAlpha::BlendMolecule(const ImageMolecule& molecule, cv::Mat& outimage)
{
  // will look ok with equalize pair... I think...

  // assert(outimage.type() == molecule.getAnchor()->getImageraw().type());

  assert(outimage.type() == CV_8UC3);
  setOutputSize(outimage.size());
  outimage = Scalar(0);

  std::set<cv::Ptr<ImageAtom> >::const_iterator atom = molecule.getAtoms().begin();

  for (; atom != molecule.getAtoms().end(); ++atom)
  {

    blendIncremental(**atom, outimage);
  }

}


void Blender::fillWeightsGaussian32(cv::Mat& weights, float sigma_squared)
{
  for (int y = 0; y < weights.rows; y++)
  {
    for (int x = 0; x < weights.cols; x++)
    {
      float y_h = ((float)y) / (weights.rows - 1.0) - 0.5;
      float x_h = ((float)x) / (weights.cols - 1.0) - 0.5;
      x_h *= 2; //x goes from -1 to 1
      y_h *= 2; //y "" ""
      double val = max((abs(x_h)), (abs((y_h))));
      val = exp(-(val) / (2 * sigma_squared)) * 1000.0;
      weights.at<float> (y, x) = val;
    }
  }
  double dmin, dmax;
  cv::minMaxLoc(weights, &dmin, &dmax);
  weights = weights - dmin;
  weights = (weights) / (dmax - dmin);
}

void Blender::fillWeightsGaussian64(cv::Mat& weights, double sigma_squared)
{
  for (int y = 0; y < weights.rows; y++)  {
    for (int x = 0; x < weights.cols; x++)   {
      double y_h = ((float)y) / (weights.rows - 1.0) - 0.5;
      double x_h = ((float)x) / (weights.cols - 1.0) - 0.5;
      x_h *= 2; //x goes from -1 to 1
      y_h *= 2; //y "" ""
      double val = max( ( abs(x_h) ), ( abs( ( y_h ) ) ) );
      val = exp(-(val) / (2 * sigma_squared) ) * 255.0;
      weights.at<double> (y, x) = val;
    }
  }
  double dmin, dmax;
  cv::minMaxLoc(weights, &dmin, &dmax);
  weights = weights - dmin;
  weights = (weights) / (dmax - dmin);
}
}

