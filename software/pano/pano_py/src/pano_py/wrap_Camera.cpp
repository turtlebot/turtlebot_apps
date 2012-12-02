#include <pano_py/pano_py.h>
#include <pano_core/pano_core.h>
#include <pano_py/opencv.h>

using namespace pano;
using namespace cv;

namespace pano_py{

namespace{
void  (Camera::*sciFile)(const std::string&) = &Camera::setCameraIntrinsics;
void  (Camera::*sciMat)(const cv::Mat&, const cv::Mat&, const cv::Size&) = &Camera::setCameraIntrinsics;
}

void wrap_Camera(){
	 bp::class_<Camera>("Camera")
	      .def("setCameraIntrinsics",sciFile)
	      .def("setCameraIntrinsics",sciMat)
	      .def("write",&Camera::write)
	      ;
}

}
