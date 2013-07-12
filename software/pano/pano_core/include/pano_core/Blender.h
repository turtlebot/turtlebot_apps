/*
 * Blender.h
 *
 *  Created on: Jun 18, 2010
 *      Author: ethan
 */

#ifndef BLENDER_H_
#define BLENDER_H_

#include <opencv2/core/core.hpp>

#include <iostream>
#include <vector>

#include "pano_core/ModelFitter.h"
#include "pano_core/ImageMolecule.h"
#include "pano_core/Projector.h"
#include "pano_core/callbacks.h"
namespace pano
{

class Blender
{ // abstract blending class, to be overridden by various ones.
public:
  virtual ~Blender()
  {
  }

  /** Draw the blend. Caller treats it as a black box.
   */
  virtual void BlendMolecule(const ImageMolecule& mol, cv::Mat& outimage) = 0;

  /** Draw the blend. Use atoms that have meta data set for geometry already.
   */
  virtual void BlendAtoms(const std::set<cv::Ptr<ImageAtom> >& atoms, cv::Mat& outimage) = 0;



  virtual void blendIncremental(const ImageAtom& image_atom, cv::Mat& outimage){ }

  static void fillWeightsGaussian32(cv::Mat& weights, float sigma_squared = 0.05);

  static void fillWeightsGaussian64(cv::Mat& weights, double sigma_squared = 0.02 );

  void set_image_path(const std::string&  path ) { image_path = path; }

protected:
  std::string image_path;

};


class BlenderNoBlend : public Blender
{
public:

  virtual ~BlenderNoBlend();

  /** Need Doxy
   */
  void BlendMolecule(const ImageMolecule& mol, cv::Mat& outimage);

  /** Need Doxy
   */
  void BlendAtoms(const std::set<cv::Ptr<ImageAtom> >& atoms, cv::Mat& outimage)
  {
  }

private:

};

class BlenderSimple : public Blender
{
public:
  BlenderSimple() ;
  virtual ~BlenderSimple();

  /** Need Doxy
   */
  void BlendMolecule(const ImageMolecule& mol, cv::Mat& outimage);

  /** Need Doxy
   */
  void BlendAtoms(const std::set<cv::Ptr<ImageAtom> >& atoms, cv::Mat& outimage)
  {
  }


  virtual void blendIncremental(const ImageAtom& image_atom, cv::Mat& outimage);

  CallbackEngine* cbe;

private:
  SparseProjector projector;

  cv::Mat in_img;
  cv::Mat in_weight;

  std::string output_prefix;

  HugeImage huge_image_;
  void setInputSize(cv::Size size);
  void setOutputSize(cv::Size size);
};

void initAlphaMat(const cv::Size& sz, cv::Mat& alpha, int feather_width);

class BlenderAlpha : public Blender
{

public:
  BlenderAlpha(int feather_edge = 30);
  BlenderAlpha(int feather_edge, cv::Size outputsize, cv::Size inputsize);
  virtual ~BlenderAlpha();

  /** Need Doxy
   */
  void BlendMolecule(const ImageMolecule& mol, cv::Mat& outimage);

  void blendMolecule(const ImageMolecule& mol, cv::Size outputsize, const std::string& name_prefix);

  /** Need Doxy
   */
  void BlendAtoms(const std::set<cv::Ptr<ImageAtom> >& atoms, cv::Mat& outimage)
  {
  }

  virtual void blendIncremental(const ImageAtom& image_atom, cv::Mat& outimage);
  CallbackEngine* cbe;

private:

  int feather_edge;
  //generate the alpha mask
  cv::Mat alpha;
  cv::Size outputsize;
  cv::Size inputSize;
  SparseProjector projector;

  cv::Mat in_img;
  cv::Mat in_alpha;
  cv::Mat in_one_minus_alpha;
  cv::Mat one_minus_alpha;

  std::string output_prefix;

  HugeImage huge_image_;
  void setInputSize(cv::Size size);
  void setOutputSize(cv::Size size);
};



class BlenderMultiband : public Blender
{
public:
  BlenderMultiband() ;
  virtual ~BlenderMultiband();

  /** Synthesize  Dihexabicylcoheptane
   */
  void BlendMolecule(const ImageMolecule& mol, cv::Mat& outimage);

  /** Need Doxy
   */
  void BlendAtoms(const std::set<cv::Ptr<ImageAtom> >& atoms, cv::Mat& outimage)
  {
  }

  virtual void blendIncremental(const ImageAtom& image_atom, cv::Mat& outimage);

  CallbackEngine* cbe;

private:
  SparseProjector projector;

  cv::Mat in_img;
  cv::Mat in_weight;

  std::string output_prefix;

  HugeImage huge_image_;
  void setInputSize(cv::Size size);
  void setOutputSize(cv::Size size);
};




}

#endif /* BLENDER_H_ */
