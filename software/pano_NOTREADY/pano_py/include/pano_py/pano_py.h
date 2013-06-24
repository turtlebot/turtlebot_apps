/*
 * pano_py.h
 *
 *  Created on: Dec 20, 2010
 *      Author: erublee
 */

#ifndef PANO_PY_H_
#define PANO_PY_H_
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <boost/foreach.hpp>
#include <pano_core/pano_core.h>
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/legacy.hpp>
namespace pano_py{

namespace bp = boost::python;

class Options
{
public:
  pano::Camera camera;
  cv::Size stitch_size;
  std::string directory;
  std::string stitch_output;
  std::list<std::string> image_names;
  pano::SVDRSolverParams fitter_params;
};
class StitchProgressCallable
{
public:
  virtual ~StitchProgressCallable()
  {
  }
  virtual int onProgress(int progress){
    return 0;
  }
};




template<typename T>
void container_assign(T& l, bp::object o) {
    // Turn a Python sequence into an STL input range

    bp::stl_input_iterator<typename T::value_type> begin(o), end;
    l.assign(begin, end);
}

template<typename T>
bp::object container_get(const T& l) {
    bp::list o;
    BOOST_FOREACH(const typename T::value_type & x, l){
      o.append(x);
    }
    return o;
}

bool stitch(Options opts, StitchProgressCallable* callback);

void wrap_SVDRSolverParams();
void wrap_Camera();
void wrap_Options();
void wrap_BlurDetector();
void wrap_stitch();
void wrap_glob();

}
#endif /* PANO_PY_H_ */
