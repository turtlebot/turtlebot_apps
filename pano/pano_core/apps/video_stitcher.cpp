/*
 * detector.cpp
 *
 *  Created on: Nov 17, 2010
 *      Author: erublee
 */
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <iostream>
#include <fstream>
#include <list>
#include <string>

#include <pano_core/pano_core.h>
#include <ros/ros.h>

#define LOGI ROS_INFO
#define LOGI_STREAM(x) ROS_INFO_STREAM(x)

#define LOGE ROS_ERROR
#define LOGE_STREAM(x) ROS_ERROR_STREAM(x)

using namespace pano;
using namespace std;
using namespace cv;

#define foreach         BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

namespace po = boost::program_options;

namespace
{
struct Options
{
  std::string k_file;
  std::string d_file;
  std::string directory;
  std::string stitch_output;
  std::vector<std::string> image_names;
};
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
}

int options(int ac, char ** av, Options& opts)
{
    // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message.")
  ("intrinsics,K", po::value<string>(&opts.k_file),"The camera intrinsics file, should be yaml and have atleast \"K:...\". Required.")
  ("distortion,D", po::value<string>(&opts.d_file)->default_value(""),"The camera distortion file, should be yaml and have atleast \"D:...\". optional, no distortion assumed if not given.")
  ("directory,d",po::value<string>(&opts.directory)->default_value("./"),"The directory that the images to stitch are in")
  ("stitch_output,o",po::value<string>(&opts.stitch_output)->default_value("stitched.jpg"),"the full path to the final stitch image, should have image extension. (.jpg,.png)")
  ("images", po::value< vector<string> >(&opts.image_names), "input images.");
  po::positional_options_description p;
  p.add("images", -1);
  po::variables_map vm;
  po::store(po::command_line_parser(ac, av). options(desc).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  if (!vm.count("intrinsics"))
  {
    cout << "Must supply a camera calibration file" << "\n";
    cout << desc << endl;
    return 1;
  }


  if (!vm.count("images"))
  {
    cout << "Must supply a list of input images" << "\n";
    cout << desc << endl;
    return 1;
  }
  return 0;

}
struct CanceledException : public std::exception
{
  const char * what() const throw ()
  {
    return "Canceled!";
  }
};
class StitchProgressCallable
{
public:
  virtual ~StitchProgressCallable()
  {
  }
  virtual int onProgress(int progress){
    LOGI_STREAM("Stitch Progress:"<< progress);
    return 0;
  }
};

bool solve(const std::string& directory, SVDRSolverParams params, Camera camera, std::vector<std::string> image_names,
           StitchProgressCallable* callback = NULL)
{
  try
  {
    Ptr<cv::AdjusterAdapter> adapter;
    Ptr<FeatureDetector> detector;

    int levels = 2;
    detector = new GriddedDynamicDetectorAdaptor(250, 20, levels, levels, FastAdjuster());

    Ptr<SVDFitter> svdfitter(new SVDFitter(params));

    Ptr<ModelFitter> fitter(reinterpret_cast<const Ptr<ModelFitter>&> (svdfitter));

    MoleculeGlob glob; //the glob stores the pano graph, think of it as the map

    vector<ImageAtom> atoms;
    vector<Mat> descriptors;
    Images images;

    int i = 0;
    int t_s = image_names.size();
    Mat img;
    camera.initUndistort();
    while (image_names.size())
    {
      string imagename = image_names.back();
      string imagename_full = directory + "/" + imagename;
      LOGI_STREAM("reading " << imagename_full );

      Extrinsics ext(Mat::eye(3, 3, CV_32F), 200);

      camera.undistort(imread(directory + "/" + imagename), img);
      images.load(img, imagename, directory);

      atoms.push_back(ImageAtom(camera, images));

      ImageAtom& atom = atoms.back();
      atom.setUid(i++);
      atom.extrinsics() = ext;
      atom.extrinsics().flag(Extrinsics::ESTIMATED) = false;
      atom.extrinsics().val(Extrinsics::CONFIDENCE) = 200;
      atom.detect(*detector);
      atom.extract(BriefDescriptorExtractor(), BruteForceMatcher<Hamming> ());
      //  descriptors.push_back(atom.features().descriptors());
      LOGI_STREAM( "found " << atom.features().kpts().size());
      glob.addAtomToGlob(fitter, atom)->images().clear();

      LOGI_STREAM( "detected and extracted " << i << "/" << t_s );
      if (callback)
      {
        callback->onProgress(int((float(i) / t_s) * 100));
      }
      image_names.pop_back();
    }

    glob.batchFindAndSetTrinsics();
    FileStorage fs(directory + "/glob.yaml.gz", FileStorage::WRITE);
    fs << "glob";
    glob.serialize(fs);
  }
  catch (CanceledException e)
  {
    LOGI("Canceled");
    return true;
  }
  catch (...)
  {
    LOGE("Fail stitch!");
    return false;
  }
  return true;

}
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
bool stitch(const std::string& pano_directory_name, const std::string& stitch_name, StitchProgressCallable* callback = NULL)
{
  try
  {
    CallbackEngine cbengine;
    string directory = pano_directory_name;
    FileStorage fs(directory + "/glob.yaml.gz", FileStorage::READ);

    if (!fs.isOpened())
    {
      LOGE_STREAM("failed to open : "+ directory + "/glob.yaml.gz");
      return false;
    }

    MoleculeGlob glob;
    glob.deserialize(fs["glob"]);
    glob.overideDirectory(directory);

    if (glob.getMolecules().size() < 1)
      return false;
    LOGI_STREAM("found - " << glob.getMolecules().size() << " molecules");

    Mat blended(Size(4000, 2000), CV_8UC3);

    Ptr<ImageMolecule> molecule;

    molecule = glob.getBiggestMolecule();

    int atom_idx = 0;
    cbengine.addCallback<ImageAtom> (0, PairFitterCallback(callback, &atom_idx, molecule->getAtoms().size(), 100));

    LOGI("simple stitching now");
    BlenderSimple blender;
    blender.cbe = &cbengine;
    blender.BlendMolecule(*molecule, blended);

    cbengine.addCallback<int> (0, PairFitterCallback(callback, &atom_idx, 100, 200));
    cbengine.callBack(0, 0);
    vector<int> write_params(2);
    write_params[0] = CV_IMWRITE_JPEG_QUALITY;
    write_params[1] = 85; //need low size for emailing


    imwrite(stitch_name, blended, write_params);
  }
  catch (CanceledException e)
  {
    LOGI("Canceled");
  }
  return true;
}

int main(int argc, char *argv[])
{
  Options opts;
  if (options(argc, argv, opts))
    return 1;

  Camera camera;
  FileStorage fs(opts.k_file, FileStorage::READ);
  Mat K;
  fs ["K"] >> K;
  Mat D;
  if(opts.d_file.size()){
    FileStorage dfs(opts.d_file, FileStorage::READ);
    dfs["D"] >> D;
  }
  Size sz;
  {
    Mat image = imread(opts.directory + "/" + opts.image_names[0]);
    sz = image.size();
  }
  camera.setCameraIntrinsics(K,D,sz);

  SVDRSolverParams params;
  params.error_thresh = 6;
  params.inliers_thresh = 15;
  params.maxiters = 100;
  params.nNeeded = 2;

  StitchProgressCallable pc;
  solve(opts.directory,params,camera,opts.image_names,&pc);
  stitch(opts.directory,opts.stitch_output,&pc);

  return 0;
}
