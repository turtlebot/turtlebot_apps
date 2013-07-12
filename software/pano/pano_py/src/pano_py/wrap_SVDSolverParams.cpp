#include <pano_py/pano_py.h>

#include <pano_core/pano_core.h>
#include <pano_py/opencv.h>


using namespace pano;
using namespace cv;

namespace pano_py{

void wrap_SVDRSolverParams(){

	  bp::class_<SVDRSolverParams>("SVDRSolverParams")
	    .def_readwrite("error_thresh",&SVDRSolverParams::error_thresh, "the maximum reprojection error with which the to consider a point an inlier")
	    .def_readwrite("inliers_thresh",&SVDRSolverParams::inliers_thresh, "the number of inliers needed to be confident in the fit")
	    .def_readwrite("maxiters",&SVDRSolverParams::maxiters, "the maximum number of RANSAC iterations")
	    .def_readwrite("nNeeded",&SVDRSolverParams::nNeeded, "the minimum number of points needed for generating a hypothesis, should be around 2")
	    ;
}

}
