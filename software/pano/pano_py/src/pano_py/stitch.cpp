/*
 * stitch.cpp
 *
 *  Created on: Dec 20, 2010
 *      Author: erublee
 */
#include <pano_py/pano_py.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <pano_core/pano_core.h>
#include <pano_py/opencv.h>


using namespace pano;
using namespace cv;
namespace pano_py{
namespace{
struct CanceledException : public std::exception
{
  const char * what() const throw ()
  {
    return "Canceled!";
  }
};


class StitchProgress:public StitchProgressCallable{
public:

  void setOnProgressCallback(const bp::object& on_progress){
    py_obj_= on_progress;
  }
  virtual int onProgress(int progress){
    return bp::extract<int>(py_obj_(progress));
  }
private:
  bp::object py_obj_;
};

struct PairFitterCallback
{
  int * i;
  int total;
  int offset;
  StitchProgressCallable* callback_;
  PairFitterCallback(StitchProgressCallable* callback, int* i, int total, int offset) :
    i(i), total(total), offset(offset), callback_(callback)
  {
  }
  template<typename T>
    void operator()(const T&)
    {
      if (callback_)
      {
        (*i)++;
        int progress = (float(*i) / total) * 100 + offset;
        callback_->onProgress(progress);
      }
    }
};

void stitch_py(Options opts, const bp::object& on_progress){
  StitchProgress cb;
  cb.setOnProgressCallback(on_progress);
  stitch(opts,&cb);
}

struct StitchEngine
{
  Options opts;
  Ptr<cv::AdjusterAdapter> adapter;
  Ptr<FeatureDetector> detector;
  SVDRSolverParams params;
  Camera camera;
  Ptr<SVDFitter> svdfitter;//(new SVDFitter(params));
  Ptr<ModelFitter> fitter;//(reinterpret_cast<const Ptr<ModelFitter>&> (svdfitter));
  MoleculeGlob glob; //the glob stores the pano graph, think of it as the map
  vector<ImageAtom> atoms;
  vector<Mat> descriptors;
  Images images;
  int idx;
  StitchEngine(Options opts) :
    opts(opts), detector(new GriddedDynamicDetectorAdaptor(250, 20, 2, 2, FastAdjuster())), params(opts.fitter_params),
        camera(opts.camera),svdfitter(new SVDFitter(params)), fitter(reinterpret_cast<const Ptr<ModelFitter>&> (svdfitter)),idx(0)
  {
    camera.initUndistort();
  }

  void addNewImage(const cv::Mat& image){
    CV_Assert(image.empty() == false);
    Mat uimage;
    camera.undistort(image,uimage);
    ImageAtom atom(camera,Images(uimage));
    atom.setUid(idx++);
    Extrinsics ext(Mat::eye(3, 3, CV_32F), 200);
    atom.extrinsics() = ext;
    atom.extrinsics().flag(Extrinsics::ESTIMATED) = false;
    atom.extrinsics().val(Extrinsics::CONFIDENCE) = 200;
    atom.detect(*detector);
    atom.extract(BriefDescriptorExtractor(), BruteForceMatcher<Hamming> ());
    glob.addAtomToGlob(fitter, atom);

  }

  void stitch(cv::Mat& blended, StitchProgressCallable* callback){
      glob.batchFindAndSetTrinsics();
      CallbackEngine cbengine; 
      CV_Assert(blended.type() == CV_8UC3);
      //Mat blended(opts.stitch_size, CV_8UC3);

      Ptr<ImageMolecule> molecule;

      molecule = glob.getBiggestMolecule();

      int atom_idx = 0;
      cbengine.addCallback<ImageAtom> (0, PairFitterCallback(callback, &atom_idx, molecule->getAtoms().size(), 100));

      std::cout << ("simple stitching now") << std::endl;
      BlenderSimple blender;
      blender.cbe = &cbengine;
      blender.BlendMolecule(*molecule, blended);

  }

};

void stitch_engine_py(StitchEngine& se, cv::Mat image, const bp::object& on_progress){
  StitchProgress cb;
  cb.setOnProgressCallback(on_progress);
  se.stitch(image,&cb);
}

}

bool stitch(Options opts, StitchProgressCallable* callback)
{
  try
  {

    StitchEngine stitcher(opts);

    int i = 0;
    int t_s = opts.image_names.size();
    while (opts.image_names.size())
    {
      string imagename = opts.image_names.back();
      string imagename_full = opts.directory + "/" + imagename;
      std::cout << "reading " << imagename_full << std::endl;

      Mat img = imread(opts.directory + "/" + imagename);
      if(img.empty()){
        std::cerr << "image was not read!" << std::endl;
      }
      stitcher.addNewImage(img);

      if (callback)
      {
        callback->onProgress(int((float(i++) / t_s) * 100));
      }
      opts.image_names.pop_back();
    }
    Mat blended(opts.stitch_size,CV_8UC3);
    stitcher.stitch(blended,callback);

    vector<int> write_params(2);
    write_params[0] = CV_IMWRITE_JPEG_QUALITY;
    write_params[1] = 85; //need low size for emailing


    imwrite(opts.stitch_output, blended, write_params);
  }
  catch (CanceledException e)
  {
    std::cout << ("Canceled");
    return true;
  }
  catch (...)
  {
    std::cerr << ("Fail stitch!");
    return false;
  }
  return true;

}

void wrap_stitch(){
	bp::def("stitch",&stitch_py, "stitch a pano");
	bp::class_<StitchEngine>("StitchEngine",bp::init<Options>())
	    .def("addNewImage",&StitchEngine::addNewImage)
	    .def("stitch",&stitch_engine_py);
}
}
