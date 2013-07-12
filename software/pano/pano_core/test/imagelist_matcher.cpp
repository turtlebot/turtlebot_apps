#include "pano_core/ImageAtom.h"
#include "pano_core/feature_utils.h"
#include "pano_core/MoleculeProcessor.h"
#include "pano_core/CaptureEngine.h"
#include "pano_core/ModelFitter.h"
#include "pano_core/Blender.h"
#include "pano_core/panoutils.h"
#include <sstream>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <list>

using namespace std;
using namespace cv;
using namespace pano;

static bool readStringList(const string& filename, vector<string>& l)
{
  l.resize(0);
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  FileNode n = fs.getFirstTopLevelNode();
  if (n.type() != FileNode::SEQ)
    return false;
  FileNodeIterator it = n.begin(), it_end = n.end();
  for (; it != it_end; ++it)
    l.push_back((string)*it);
  return true;
}

void maskMatchesByTrainImgIdx(const vector<DMatch>& matches, int trainImgIdx, vector<char>& mask)
{
  mask.resize(matches.size());
  fill(mask.begin(), mask.end(), 0);
  for (size_t i = 0; i < matches.size(); i++)
  {
    if (matches[i].imgIdx == trainImgIdx)
      mask[i] = 1;
  }
}

void condenseByTrainImgIdx(const vector<DMatch>& matches, int trainImgIdx, vector<DMatch>& tmatches)
{
  tmatches.clear();
  for (size_t i = 0; i < matches.size(); i++)
  {
    if (matches[i].imgIdx == trainImgIdx)
      tmatches.push_back(matches[i]);
  }
}
int main(int ac, char** av)
{
  if (ac != 4)
  {
    cout << "usage: " << av[0] << " camera.yml directory images.yml" << endl;
    return 1;
  }
  Camera camera;
  camera.setCameraIntrinsics(av[1]);

  string directory = av[2];
  string image_list_yaml = av[3];

  vector<std::string> image_list;
  if (!readStringList(image_list_yaml, image_list))
  {
    cerr << "could not read image list" << endl;
    return 1;
  }

  Ptr<FeatureDetector> detector(new DynamicAdaptedFeatureDetector( new FastAdjuster(),100, 110, 10));

  SVDRSolverParams params;
  params.error_thresh = 3;
  params.inliers_thresh = 12;
  params.maxiters = 50;
  params.nNeeded = 2;

  Ptr<SVDFitter> svdfitter(new SVDFitter(params));

  Ptr<ModelFitter> fitter(reinterpret_cast<const Ptr<ModelFitter>&> (svdfitter));

  //the glob stores the pano graph
  //think of it as the map
  MoleculeGlob glob;

  BlurDetector blur_detector;

  vector<ImageAtom> atoms;
  vector<Mat> descriptors;
  Images images;
  for (size_t i = 0; i < image_list.size(); i++)
  {
    images.load(image_list[i], directory);
    atoms.push_back(ImageAtom(camera, images));
    ImageAtom& atom = atoms.back();
    atom.setUid(i);
    atom.extrinsics() = Extrinsics(Mat::eye(3, 3, CV_32F), 1);
    atom.extrinsics().val(Extrinsics::ESTIMATED) = false;
    atom.detect(*detector);
    atom.extract(BriefDescriptorExtractor(), BruteForceMatcher<Hamming> ());

    atom.images().clear();
    descriptors.push_back(atom.features().descriptors());
    cout << "detected and extracted " << i + 1 << "/" << image_list.size() << endl;
  }

  BruteForceMatcher<Hamming> matcher;
  matcher.add(vector<Mat> (1, descriptors[0]));
  vector<vector<DMatch> > matches_all;
  matches_all.reserve(descriptors.size());

  vector<AtomPair> pairs;

  for (size_t i = 1; i < descriptors.size(); i++)
  {
    matches_all.push_back(vector<DMatch> ());
    matcher.match(descriptors[i], matches_all.back());
    cout << "matched descriptors: " << i + 1 << "/" << descriptors.size() << endl;

    for (size_t j = 0; j < i; j++)
    {
      vector<DMatch> matches;
      condenseByTrainImgIdx(matches_all.back(), j, matches);
      if (matches.size() >= params.inliers_thresh)
      {
        AtomPair atom_pair(atoms[j].ptrToSelf(), atoms[i].ptrToSelf(), matches);
        pairs.push_back(atom_pair);
      }
    }
    matcher.add(vector<Mat> (1, descriptors[i]));
  }

  cout << "fitting pairs, have " << pairs.size() << " to fit" << endl;
  FitPair pair_fitter(fitter, -1, new list<AtomPair> ());
  std::for_each(pairs.begin(), pairs.end(), pair_fitter);
  cout << "done fitting... found :" << pair_fitter.good_pairs->size() << " good pairs" << endl;

  glob.addPrefittedPairs(*pair_fitter.good_pairs);
  glob.batchFindAndSetTrinsics();
  FileStorage fs("glob.yaml.gz", FileStorage::WRITE);
  fs << "glob";
  glob.serialize(fs);
  cout << "found - " << glob.getMolecules().size() << " molecules" << endl;
  return 0;
}
