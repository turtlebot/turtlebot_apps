/*
 * w2d_time.cpp
 *
 *  Created on: Oct 19, 2010
 *      Author: erublee
 */

#include <iostream>

#include <wavelet2d/wavelet2d.h>

#include "pano_core/ImageAtom.h"
#include "pano_core/feature_utils.h"
#include "pano_core/QuadTree.h"

#include <opencv2/opencv.hpp>
#include <brief_descriptor/brief.h>
#include <iostream>
#include <fstream>
#include <list>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace wavelet2d;
using namespace std;
using namespace pano;

inline void rescaleFloatImage(cv::Mat& img)
{
  using namespace cv;
  Mat flimage;
  img.convertTo(flimage, CV_32F);
  std::vector<Mat> channels;
  split(flimage, channels);
  for (size_t k = 0; k < channels.size(); k++)
  {
    Mat cc = channels[k];
    double fmin, fmax;
    cv::minMaxLoc(cc, &fmin, &fmax);
    //        if( fmax > 1.0 )
    //            fmax = 255.0 ;
    //        else
    //            fmax = 1.0;
    channels[k] = 255. * ((channels[k] - fmin) / (fmax - fmin));
  }

  merge(channels, flimage);
  flimage.convertTo(img, CV_8U);
}

list<string> getImageList(istream& input) {
  list<string> imlist;
  while (!input.eof() && input.good()) {
    string imname;
    input >> imname;
    if(!input.eof() && input.good())
      imlist.push_back(imname);
  }
  return imlist;
}


int main(int ac, char**av)
{



  if(ac != 3){
    cerr << "usage : " << av[0] << " directory imagelist.txt" << endl;
    return 1;
  }

  std::ifstream input(av[2]);
  std::string directory = av[1];

  cout << "directory " << directory << endl;
  std::list<string> img_names = getImageList(input);


  if(img_names.empty()){
    cerr << av[2] << "does not contain a list of images" << endl;
    return 1;
  }



  vector<Mat> images;

  while(!img_names.empty())
  {

    string image_name = img_names.back();
    img_names.pop_back();

    images.push_back(imread(image_name));
  }

  namedWindow("small",CV_WINDOW_KEEPRATIO);

  namedWindow("gradient",CV_WINDOW_KEEPRATIO);
  for(size_t i=0; i < images.size();i++){
    Mat img_r = images[i];
    Mat img;
    resize(img_r,img,Size(img_r.size().width/10,img_r.size().height/10),0,0,CV_INTER_AREA);

    Mat img_grad;

    Sobel(img,img_grad,CV_32F,1,1);

    Mat w,J;
    Rodrigues(Mat::eye(3,3,CV_32F),w,J);






    imshow("small",img);
    imshow("gradient",img_grad);
    waitKey(1000);

  }
  return 0;

}
