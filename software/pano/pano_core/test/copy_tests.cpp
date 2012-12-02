/*
 * copy_tests.cpp
 *
 *  Created on: Oct 20, 2010
 *      Author: erublee
 */

#include "pano_core/ModelFitter.h"
#include "pano_core/feature_utils.h"
#include <iostream>
#include <vector>
using namespace cv;
using namespace pano;
using namespace std;
int main()
{
  ImageAtom atom;
  ImageAtom atom2;

  atom.images().load("image1.jpg","data");
  atom2.images().load("image2.jpg","data");


  vector<DMatch> matches;

  BriefDescriptorExtractor brief;
  BruteForceMatcher<HammingLUT> desc_matcher;

  FastFeatureDetector fast(20);
  atom.detect(fast);
  atom.extract(brief,desc_matcher);
  atom2.detect(fast);
  atom2.extract(brief,desc_matcher);

  BruteForceMatcher<HammingLUT> matcher;
  desc_matcher.clear();
  std::vector<Mat> d(1);
  d[0] =atom.features().descriptors();
  desc_matcher.add(d);
  desc_matcher.train();
  desc_matcher.match(atom2.features().descriptors(), matches);

  atom2 = atom;

  atom.images().load("image3.jpg","data");

  atom.detect(fast);
 // atom.extract(brief);

  desc_matcher.clear();
  d[0] = atom2.features().descriptors();
  desc_matcher.add(d);
  desc_matcher.match(atom2.features().descriptors(),matches);
  return 0;
}
