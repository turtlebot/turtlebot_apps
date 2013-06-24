#include <pano_py/pano_py.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <pano_core/pano_core.h>
#include <pano_py/opencv.h>


using namespace pano;
using namespace cv;

namespace pano_py{

BOOST_PYTHON_MODULE(pano_py)
{
  wrap_Options();

  wrap_SVDRSolverParams();

  wrap_stitch();

  wrap_BlurDetector();

  wrap_Camera();
}

}
