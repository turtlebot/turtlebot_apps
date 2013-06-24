#include "pano_core/Blender.h"
#include "pano_core/Projector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pano_core/panoutils.h"
#include "pano_core/BlurDetector.h"
using namespace cv;
using namespace std;

namespace pano
{

BlenderSimple::BlenderSimple() :
  cbe(NULL)
{

}

BlenderSimple::~BlenderSimple()
{

}

void BlenderSimple::blendIncremental(const ImageAtom& atom, cv::Mat& outimage)
{
  CV_Assert((outimage.empty() && output_prefix.size() ) || outimage.type() == CV_32FC3)
    ;
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

  Mat _img = img;
  Mat _R = atom.extrinsics().mat(Extrinsics::R);
  std::vector<int> roi_ids;
  cv::Size inputSize = _img.size();

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

    // compose:

    if (outimage.empty())
    {
      imwrite(roi_name, roi_out);
    }

  }

}

void BlenderSimple::BlendMolecule(const ImageMolecule& molecule, cv::Mat& outimage_)
{
  Mat outimage = cv::Mat::zeros(outimage_.size(), CV_32FC3);
  if (!molecule.getAtoms().size())
  {
    cerr << "bad: empty molecule" << endl;
    return;
  }
  SparseProjector sprojector(outimage.size(), Size(10, 5));
  Mat wsum(Mat::zeros(outimage.size(), CV_32FC1)); // sum of all images' weights
  outimage = Scalar(0); // float32 version of final image

  Mat Ib; // map atom's image to projection
  Mat img;
  Mat wb;
  Mat img_tmp;
  set<Ptr<ImageAtom> >::const_iterator atom = molecule.getAtoms().begin();

  std::vector<cv::Mat> img_channels_cache(3);
  Size input_size;
  if ((*atom)->images().src().empty())
  {
    Images images = (*atom)->images();
    images.restore();
    input_size = images.src().size();

  }
  else
    input_size = (*atom)->images().src().size();

  Mat weights = Mat(input_size, CV_32F);
  Mat cweights;
  Blender::fillWeightsGaussian32(weights,0.05);
  float confidence_min = 1e-3;
#if MATLAB_VERBOSE
  ofstream matlab_file;
  matlab_file.open("omega_blend_out.m");
  int img_idx = 0;
#endif

  for (; atom != molecule.getAtoms().end(); ++atom)
  {
    float confidence = ((*atom)->extrinsics().val(Extrinsics::CONFIDENCE));

   // cout << "% atom confidence: " << confidence << endl;
    if (cbe) {
      cbe->callBack(**atom, 0);
    }
#if MATLAB_VERBOSE
    { // dump final image name <=> rotation
      matlab_file << "images_all{1+" << img_idx << "}='"
           << (*atom)->images().fname() << "';" << endl;

      matlab_file << "omega_all{1+" << img_idx << "}="
           << (*atom)->extrinsics().mat(Extrinsics::W) << ";" << endl;
      img_idx += 1;
    }
#endif
    if (confidence < confidence_min)  {
      //not blending, confidence is too low
      continue;
    }

    if ((*atom)->images().src().empty())
    {
      Images images = (*atom)->images();
      images.restore();
      images.src().convertTo(img, CV_32FC3);
    }
    else
      (*atom)->images().src().convertTo(img, CV_32FC3);

    img.copyTo(img_tmp);
    sharpen_backwards_heat_equation( img, img_tmp, .05);

    Mat R = (*atom)->extrinsics().mat(Extrinsics::R);
    Mat T = (*atom)->extrinsics().mat(Extrinsics::T);

    cweights = confidence * weights;

    std::vector<int> roi_ids;

    //get all the chips for this atom, given R and K
    sprojector.setSRandK(img.size(), R, (*atom)->camera().K(), roi_ids);

    //multiply the image by the weigths
    multiplyImageByFloatWeights(img, cweights, img, &img_channels_cache);

    for (int i = 0; i < (int)roi_ids.size(); i++)
    {
      int roi_id = roi_ids[i];
      Rect roi = sprojector.getRoi(roi_id);
      Mat out_chip = outimage(roi);
      Mat wsum_chip = wsum(roi);

      sprojector.projectMat(roi_id, cweights, wb, cv::INTER_LINEAR);
      wsum_chip += wb;
      sprojector.projectMat(roi_id, img, Ib, cv::INTER_LINEAR);
      out_chip += Ib;
    }
  }

#if MATLAB_VERBOSE
  matlab_file.close();
#endif
  divideImageByFloatWeights(outimage, wsum, outimage);
  wsum.release();
  outimage.convertTo(outimage_, outimage_.type());
  outimage.release();

}

} // end namespace vrcl


