#include <pano_py/pano_py.h>

#include <pano_core/pano_core.h>
#include <pano_py/opencv.h>


using namespace pano;
using namespace cv;

namespace pano_py{

void wrap_Options(){
	  bp::class_<std::list<std::string> >("list_string")
	      .def("assign", &container_assign<std::list<std::string> >)
	      .def("get", &container_get<std::list<std::string> >)
	      ;

	  bp::class_<Options>("Options")
	      .def_readwrite("camera",&Options::camera)
	      .def_readwrite("directory",&Options::directory)
	      .def_readwrite("stitch_output",&Options::stitch_output)
	      .def_readwrite("image_names",&Options::image_names)
	      .def_readwrite("stitch_size",&Options::stitch_size)
	      .def_readwrite("fitter_params",&Options::fitter_params)
	      ;

}

}
