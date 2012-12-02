/*
 * utest.cpp
 *
 *  Created on: Oct 19, 2010
 *      Author: erublee
 */

// Bring in my package's API, which is what I'm testing
#include "pano_core/ModelFitter.h"
// Bring in gtest
#include <gtest/gtest.h>

#include <iostream>
#include <vector>
using namespace cv;
using namespace pano;
using namespace std;
// Declare a test
TEST(FitterResultTestSuite, Serialization)
{

  FitterResult result;
  FileStorage fs("filenode_empty.yml", FileStorage::WRITE);
  fs << "result";
  result.serialize(fs);
  fs.release();

  FitterResult de_result;
  fs = FileStorage("filenode_empty.yml", FileStorage::READ);

  de_result.deserialize(fs["result"]);
  fs.release();

  vector<Mat> mats(3);
  mats[0] = Mat::eye(3, 3, CV_32FC1);
  mats[1] = Mat::ones(4, 1, CV_32FC1);
  mats[2] = Mat::zeros(Size(4, 4), CV_8U);
  result = FitterResult(mats, true, CV_PI, 12.0, vector<unsigned char> (), 20);

  fs = FileStorage("filenode_filled.yml", FileStorage::WRITE);
  fs << "result";
  result.serialize(fs);
  fs.release();

  fs = FileStorage("filenode_filled.yml", FileStorage::READ);
  de_result.deserialize(fs["result"]);
  fs.release();

  fs = FileStorage("filenode_filled_deserialed.yml", FileStorage::WRITE);
  fs << "result";
  de_result.serialize(fs);
  fs.release();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
