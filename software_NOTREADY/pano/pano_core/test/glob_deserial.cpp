/*
 * glob_deserial.cpp
 *
 *  Created on: Oct 24, 2010
 *      Author: ethan
 */

#include "pano_core/ImageAtom.h"
#include "pano_core/feature_utils.h"
#include "pano_core/QuadTree.h"
#include "pano_core/MoleculeProcessor.h"
#include "pano_core/CaptureEngine.h"
#include "pano_core/ModelFitter.h"
#include "pano_core/Blender.h"

#include <sstream>
#include <iostream>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
using namespace pano;

int main(int ac, char ** av)
{
  if (ac != 2 && ac != 3)
  {
    cout << "usage: " << av[0] << " <glob.yml or glob.yml.gz> [glob path]" << endl;
    return 1;
  }

  string out_name = "blended.deserialed.jpg";
  string glob_yml = av[1];
  string glob_path = "";
  if(ac == 3)
    glob_path = av[2];

  MoleculeGlob glob;
  FileStorage fs(glob_yml, FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cerr << "bad yml file!" << endl;
    return 1;
  }

  cout << "deserializing...";
  cout.flush();
  glob.deserialize(fs["glob"]);

  if(glob_path.size())
    glob.overideDirectory(glob_path);
  cout << " done." << endl;

  cout << "blending...";
  cout.flush();

  Ptr<ImageMolecule> molecule = glob.getMerged();

  BlenderSimple blender;
  //BlenderMultiband blender;
  Mat blended(Size(3000,1500),CV_8UC3);

  blender.BlendMolecule( *molecule, blended);
  imwrite(out_name,blended);

  cout << " done." << endl;
  namedWindow("blended",CV_WINDOW_KEEPRATIO);
  imshow("blended",blended);
  while(char(waitKey()) != 'q'){
	  cout << "press 'q' to quit" << endl;
  }

  return 0;
}
