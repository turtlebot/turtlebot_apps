#ifndef PANO_APP_H_
#define PANO_APP_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pano_ros/PanoCaptureAction.h>

#include <image_transport/image_transport.h>	
#include <sensor_msgs/Image.h>		
#include <std_msgs/Empty.h> 		
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>		
#include <geometry_msgs/Twist.h>	
//#include <> // and some for modal logic
#include "geometry.h"

#include <turtlebot_panorama/TakePano.h>

namespace turtlebot_panorama {

enum State
{
  PENDING,
  SNAP,
  TURN,
  STOP,
  DONE,
};

class PanoApp
{
public:
  PanoApp();
  ~PanoApp();

  void init();
  void spin();
  void log(std::string msg);

private:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

  std::map<std::string,std::string> params;
  // panorama action server apis
  actionlib::SimpleActionClient<pano_ros::PanoCaptureAction>* ac;
  ros::Publisher pub_snap;
  ros::Publisher pub_stop;
  image_transport::Subscriber sub_stitched;

  // to control robot 
  ros::Publisher pub_cmd_vel;

  // to get the odometry of robot
  ros::Subscriber sub_odom;
  
  // public api
  ros::ServiceServer srv_pano;
  ros::Subscriber sub_pano;
  ros::Publisher pub_log;
  image_transport::Publisher  pub_stitched;

  std_msgs::Empty empty;
  geometry_msgs::Twist cmd_vel;
  float ang_vel;
  double snap_interval;

  State state;
  double angle, last_angle, given_angle;

  // to check whether it is in the progress of panoraming
  bool is_active;

  void snap();
  void turn();
  void stop();
  void done();
  bool hasReached();

  void startPano();


  bool takeService(TakePano::Request& request,TakePano::Response& response);
  void takeCb( const std_msgs::EmptyConstPtr& msg );
  void imageCb( const sensor_msgs::ImageConstPtr& msg );

  void odomCb( const nav_msgs::OdometryConstPtr& msg );

  void activeCb();
  void feedbackCb( const pano_ros::PanoCaptureFeedbackConstPtr& feedback );
  void doneCb( const actionlib::SimpleClientGoalState& state, const pano_ros::PanoCaptureResultConstPtr& result );
};

} //namespace pano_app

#endif /* PANO_APP_H_ */
