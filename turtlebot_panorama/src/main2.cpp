#include <iostream>
#include <turtlebot_panorama/pano_app2.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pano_app");

  turtlebot_panorama::PanoApp pano;
  pano.log("Panorama app Launching...");
  pano.init();
  pano.log("Panorama application initialized.");
  pano.spin();
  pano.log("Bye Bye");

  return 0;
}

