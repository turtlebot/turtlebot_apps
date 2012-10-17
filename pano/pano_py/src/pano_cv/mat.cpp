/*
 * opencv.cpp
 *
 *  Created on: Dec 17, 2010
 *      Author: erublee
 */

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <boost/foreach.hpp>
#include <pano_core/pano_core.h>
#include <pano_py/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include "wrappers.h"

using namespace pano;
namespace bp = boost::python;
using namespace cv;

namespace{

template<typename T>
  void mat_set_t(cv::Mat&m, bp::object o)
  {

    int length = bp::len(o);
    CV_Assert(length == m.size().area())
      ;
    bp::stl_input_iterator<T> begin(o), end;
    typename cv::Mat_<T>::iterator it = m.begin<T> (), itEnd = m.end<T> ();
    for (; it != itEnd; ++it)
      *it = *(begin++);
  }

void mat_set(cv::Mat& m, bp::object o, int type)
{
  //switch on the given type and use this type as the cv::Mat element type
  switch (type)
  {
    case cv::DataType<unsigned char>::type:
      mat_set_t<unsigned char> (m, o);
      break;
    case cv::DataType<int>::type:
      mat_set_t<int> (m, o);
      break;
    case cv::DataType<float>::type:
      mat_set_t<float> (m, o);
      break;
    case cv::DataType<double>::type:
      mat_set_t<double> (m, o);

      break;
  }

}
cv::Size mat_size(const cv::Mat&m){
	return m.size();
}
void mat_set(cv::Mat& m, bp::object o)
{
	//use the m.type and implicitly assume that o is of this type
	mat_set (m,o,m.type());
}

void mat_from_numpy_array(cv::Mat& m, bp::object o)
{
  pano_py::numpy_to_mat(o.ptr(),m);
}

//overloaded function pointers
void  (*mat_set_p2)(cv::Mat&, bp::object) = mat_set;
void  (*mat_set_p3)(cv::Mat&, bp::object, int) = mat_set;

}

namespace pano_py
{
void wrapMat(){
  //mat definition
   bp::class_<cv::Mat>("Mat")
       .def(bp::init<>())
       .def(bp::init<int, int, int>())
       .def_readonly("rows", &cv::Mat::rows, "the number of rows")
       .def_readonly("cols",&cv::Mat::cols, "the number of columns")
       .def("row",&cv::Mat::row, "get the row at index")
       .def("col",&cv::Mat::col, "get the column at index")
       .def("fromarray",mat_set_p2)
       .def("fromarray",mat_set_p3)
       .def("from_numpy_array",mat_from_numpy_array)
       .def("size",&mat_size)
       ;
}

}
