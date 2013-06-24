#include "pano_core/Blender.h"
#include "pano_core/Projector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pano_core/panoutils.h"
#include "pano_core/BlurDetector.h"

//#include <wavelet2d/wavelet2d.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//using namespace wavelet2d;
using namespace cv;
using namespace std;

namespace pano
{

BlenderMultiband::BlenderMultiband() :
  cbe(NULL)
{

}

BlenderMultiband::~BlenderMultiband()
{

}

 

void BlenderMultiband::blendIncremental(const ImageAtom& atom, cv::Mat& outimage)
{
  CV_Assert((outimage.empty() && output_prefix.size() ) || outimage.type() == CV_64FC3)
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

void BlenderMultiband::BlendMolecule(const ImageMolecule& molecule, cv::Mat& outimage_)
{
  Mat outimage = cv::Mat::zeros(outimage_.size(), CV_64FC3);
  if (!molecule.getAtoms().size())
  {
    cerr << "bad: empty molecule" << endl;
    return;
  }
  SparseProjector sprojector(outimage.size(), Size(10, 5));
  Mat wsum(Mat::zeros(outimage.size(), CV_64F)); // sum of all images' weights
  outimage = Scalar(0); // float32 version of final image

  Mat Ib,  Ib_32; // map atom's image to projection
  Mat img, img_32;
  Mat wb,  wb_32;

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

  Mat weights   = Mat(input_size, CV_64F);
  Mat cweights, cw_32;
  Blender::fillWeightsGaussian64(weights);

  double confidence_min = 1e-3;

  ofstream matlab_file;
  matlab_file.open("omega_blend_out.m");

  int img_idx = 0;
  for (; atom != molecule.getAtoms().end(); ++atom)
  {
    double confidence = ((*atom)->extrinsics().val(Extrinsics::CONFIDENCE));

   // cout << "% atom confidence: " << confidence << endl;
    if (cbe) {
      cbe->callBack(**atom, 0);
    }

    { // dump final image name <=> rotation
      matlab_file << "images_all{1+" << img_idx << "}='"
           << (*atom)->images().fname() << "';" << endl;

      matlab_file << "omega_all{1+" << img_idx << "}="
           << (*atom)->extrinsics().mat(Extrinsics::W) << ";" << endl;
      img_idx += 1;
    }

    if (confidence < confidence_min)  {
      cout << "not blending, confidence is too low: " << confidence << endl;
      continue;
    }

    if ((*atom)->images().src().empty())
    {
      Images images = (*atom)->images();
      images.restore();
      images.src().convertTo(img, CV_64FC3);
    }
    else
      (*atom)->images().src().convertTo(img, CV_64FC3);

    sharpen_backwards_heat_equation( img.clone(), img, .05);

    Mat R = (*atom)->extrinsics().mat(Extrinsics::R);
    Mat T = (*atom)->extrinsics().mat(Extrinsics::T);

    cv::sqrt( confidence * weights, cweights );

    cweights.convertTo(cw_32, CV_32F);

    std::vector<int> roi_ids;

    //get all the chips for this atom, given R and K
    sprojector.setSRandK(img.size(), R, (*atom)->camera().K(), roi_ids);

    //multiply the image by the weigths
    multiplyImageByDoubleWeights(img, cweights, img, &img_channels_cache);
    img.convertTo( img_32, CV_32FC3 );

    assert( !cw_32.empty() && !img_32.empty() );
    for (int i = 0; i < (int)roi_ids.size(); i++)
    {
      int roi_id = roi_ids[i];
      Rect roi = sprojector.getRoi(roi_id);
      Mat out_chip = outimage(roi);
      Mat wsum_chip = wsum(roi);


      sprojector.projectMat(roi_id, cw_32, wb_32, INTER_LINEAR);   // project MS-bits
      sprojector.projectMat(roi_id, img_32, Ib_32, INTER_LINEAR);  // project MS-bits

      wb_32.convertTo(wb,CV_64F);
      Ib_32.convertTo(Ib,CV_64FC3);

      wsum_chip += wb;
      out_chip += Ib;
    }
  }
  matlab_file.close();

  divideImageByDoubleWeights(outimage, wsum, outimage);

  outimage.convertTo(outimage_, outimage_.type());
 
}

} // end namespace vrcl


