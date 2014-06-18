/* 
 *  opencv.cppi
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
namespace pano_py
{

namespace
{
struct memtrack_t {
  PyObject_HEAD
  void *ptr;
  Py_ssize_t size;
};


struct cvmat_t
{
  PyObject_HEAD
  CvMat *a;
  PyObject *data;
  size_t offset;
};

struct iplimage_t {
  PyObject_HEAD
  IplImage *a;
  PyObject *data;
  size_t offset;
};

cv::Mat convert_from_cvmat(PyObject *o, const char* name)
{
  Mat dest;
  cvmat_t *m = (cvmat_t*)o;
  void *buffer;
  Py_ssize_t buffer_len;

  m->a->refcount = NULL;
  if (m->data && PyString_Check(m->data))
  {
      assert(cvGetErrStatus() == 0);
      char *ptr = PyString_AsString(m->data) + m->offset;
      cvSetData(m->a, ptr, m->a->step);
      assert(cvGetErrStatus() == 0);
      dest = m->a;

  }
  else if (m->data && PyObject_AsWriteBuffer(m->data, &buffer, &buffer_len) == 0)
  {
    cvSetData(m->a, (void*)((char*)buffer + m->offset), m->a->step);
    assert(cvGetErrStatus() == 0);
    dest = m->a;
  }
  else
  {
    failmsg("CvMat argument '%s' has no data", name);
  }
  return dest;

}

cv::Mat convert_from_cviplimage(PyObject *o,const char *name)
{
  Mat dest;
  iplimage_t *ipl = (iplimage_t*)o;
  void *buffer;
  Py_ssize_t buffer_len;

  if (PyString_Check(ipl->data)) {
    cvSetData(ipl->a, PyString_AsString(ipl->data) + ipl->offset, ipl->a->widthStep);
    assert(cvGetErrStatus() == 0);
    dest = ipl->a;
  } else if (ipl->data && PyObject_AsWriteBuffer(ipl->data, &buffer, &buffer_len) == 0) {
    cvSetData(ipl->a, (void*)((char*)buffer + ipl->offset), ipl->a->widthStep);
    assert(cvGetErrStatus() == 0);
    dest = ipl->a;
  } else {
    failmsg("IplImage argument '%s' has no data", name);
  }
  return dest;
}


}//end anon namespace

////these are take from opencv/modules/python
int failmsg(const char *fmt, ...)
{
  char str[1000];

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(str, sizeof(str), fmt, ap);
  va_end(ap);

  PyErr_SetString(PyExc_TypeError, str);
  return 0;
}

cv::Mat convertObj2Mat(bp::object image)
{	
  if(strcmp(image.ptr()->ob_type->tp_name,"cv.iplimage") == 0){
    return convert_from_cviplimage(image.ptr(),image.ptr()->ob_type->tp_name);
  }else
    return convert_from_cvmat(image.ptr(), image.ptr()->ob_type->tp_name);

}

cv::Mat convertNumpy2Mat(bp::object np)
{
  Mat m;
  numpy_to_mat(np.ptr(),m);
  return m;
}

void imwrite_noargs(const std::string& window_name, const cv::Mat& image){
  imwrite(window_name,image);
}

BOOST_PYTHON_MODULE(pano_cv)
{
  //define opencv consts
  bp::object opencv = bp::scope();
  opencv.attr("CV_8UC1")  = CV_8UC1;
  opencv.attr("CV_32SC1")  = CV_32SC1;
  opencv.attr("CV_32FC1")  = CV_32FC1;
  opencv.attr("CV_64FC1")  = CV_64FC1;
  opencv.attr("CV_WINDOW_KEEPRATIO")  = int(CV_WINDOW_KEEPRATIO);
  opencv.attr("CV_WINDOW_NORMAL")  = int(CV_WINDOW_NORMAL);

  bp::class_<cv::Size>("Size")
       .def(bp::init<int, int>())
       .def_readwrite("width", &cv::Size::width)
       .def_readwrite("height",&cv::Size::height)
       .def("area",&cv::Size::area)
       ;

  wrapMat();

  bp::def("convertNumpy2Mat",convertNumpy2Mat);
  bp::def("convertCvMat2Mat",convertObj2Mat);

  bp::def("namedWindow",cv::namedWindow);
  bp::def("imshow",cv::imshow);
  bp::def("imwrite",imwrite_noargs);
  bp::def("waitKey",cv::waitKey);
}
}
