#include <iostream>
#include <turtlebot_panorama/pano_app.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pano_app");

  pano_app::PanoApp pano;
  pano.init();
  pano.spin();

  return 0;
}

