#include <pano_py/pano_py.h>

#include <pano_core/pano_core.h>
#include <pano_py/opencv.h>


using namespace pano;
using namespace cv;

namespace pano_py{

namespace{
double checkBlur(BlurDetector& bd, bp::object cvmat)
{
  Mat img = convertObj2Mat(cvmat);
  CV_Assert(!img.empty());
  return bd.checkBlur(img);
}
}

void wrap_glob(){
	  bp::class_<BlurDetector>("BlurDetector")
	      .def("checkBlur",checkBlur);
}

}
