#include <turtlebot_panorama/pano_app.h>

namespace pano_app {

PanoApp::PanoApp() : ac("pano_capture", true), 
  is_active(false)
{
  ROS_INFO("Wait for Capture Server.");
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action Server \"pano_capture\" is ready.");
}

void PanoApp::init() 
{
  ros::NodeHandle nh;

  sub_pano = nh.subscribe("/take_pano", 1, &PanoApp::takeCb, this);

  pub_snap = nh.advertise<std_msgs::Empty>( "/pano_capture/snap", 100 );
  pub_stop = nh.advertise<std_msgs::Empty>( "/pano_capture/stop", 100 );

  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>( "/cmd_vel", 100 );
  sub_odom = nh.subscribe("/odom", 100, &PanoApp::odomCb, this);

  image_transport::ImageTransport it(nh);
  pub_stitched = it.advertise("/panorama", 1);
  sub_stitched = it.subscribe("/pano_capture/stitch", 1, &PanoApp::imageCb, this);

  cmd_vel.linear.x = 0.0f;
  cmd_vel.linear.y = 0.0f;
  cmd_vel.linear.z = 0.0f;
  cmd_vel.angular.x = 0.0f;
  cmd_vel.angular.y = 0.0f;
  cmd_vel.angular.z = 0.0f; 

  ROS_INFO("Panorama application initialized.");
}

void PanoApp::spin()
{
  ros::Rate loop_rate(10);
  while( ros::ok() ) {
	  switch ( state )
	  {
	  case PENDING: break;
	  case SNAP: snap(); break;
	  case TURN: turn(); break;
	  case STOP: stop(); break;
	  case DONE: done(); break;
	  default: break;
	  }  
	  ros::spinOnce();
	  loop_rate.sleep();
  }
}

void PanoApp::snap()
{
  ROS_INFO("snap");
  pub_snap.publish( empty );
  state = PENDING;
}

void PanoApp::turn()
{
  ROS_INFO("turn");
  if( hasReached() ) {
    cmd_vel.angular.z = 0.0f; 
    state = SNAP;
  } else {
    cmd_vel.angular.z = 0.33f; 
  }
  pub_cmd_vel.publish( cmd_vel );
}

bool PanoApp::hasReached()
{
  if( angle > last_angle + ecl::degrees_to_radians(30.0) ) {
    last_angle = angle;
    return true;
  } else {
    return false;
  }
}

void PanoApp::stop()
{
  ROS_INFO("stop");
  pub_stop.publish( empty );
  state = PENDING;
}

void PanoApp::done()
{
  ROS_INFO("done");
  ;
}

/********************
 * Callbacks
 ********************/
void PanoApp::takeCb( const std_msgs::EmptyConstPtr& msg )
{
  if( is_active ) return;

  //setup and send goal
  pano_ros::PanoCaptureGoal goal;
  goal.bag_filename = "/opt/pano00.bag";
  goal.camera_topic = "/camera/rgb";
  //ac.sendGoal(goal);
  ac.sendGoal(
    goal, 
    boost::bind(&PanoApp::doneCb,     this, _1, _2),
    boost::bind(&PanoApp::activeCb,   this),
    boost::bind(&PanoApp::feedbackCb, this, _1)
  );
  ROS_INFO("Goal sent.");

  angle = 0.0f;
  last_angle = 0.0f;
  is_active = true;
  //state = PENDING;
}

void PanoApp::odomCb( const nav_msgs::OdometryConstPtr& msg )
{
  static double heading_last = 0.0f;
  double heading = 0.0f;

  Eigen::AngleAxisf angle_axis(Eigen::Quaternionf(
    msg->pose.pose.orientation.w, 
    msg->pose.pose.orientation.x, 
    msg->pose.pose.orientation.y, 
    msg->pose.pose.orientation.z
  ));
  Eigen::Vector3f axis = angle_axis.axis();

  if( axis(2) > 0.0 ) heading = angle_axis.angle();
  else if( axis(2) < 0.0 ) heading = -1.0 * angle_axis.angle();

  angle += fabs(ecl::wrap_angle(heading - heading_last));
  heading_last = heading;
}

void PanoApp::imageCb ( const sensor_msgs::ImageConstPtr& msg )
{
  //if( is_active && state == DONE ) {
    ROS_INFO("repub stitched image to public channel.");
    pub_stitched.publish( msg );
    state = PENDING;
    is_active = false;
}

void PanoApp::doneCb( const actionlib::SimpleClientGoalState& state, const pano_ros::PanoCaptureResultConstPtr& result )
{
  ROS_INFO("Finished in state [%s].", state.toString().c_str() );
  ROS_INFO("pano_id: %d", 		result->pano_id );
  ROS_INFO("n_captures: %d", 	result->n_captures );
  ROS_INFO("bag_filename: %s", 	result->bag_filename.c_str() );
  this->state = DONE; //instead of shutdown.
}

void PanoApp::activeCb( )
{
  ROS_INFO("Goal just went active.");
  state = SNAP;
}

void PanoApp::feedbackCb( const pano_ros::PanoCaptureFeedbackConstPtr& feedback )
{
  ROS_INFO("Got Feedback: %d of picture captured.", (int)feedback->n_captures );

  if( /*hasRevoluted()*/angle > 2.0*M_PI || feedback->n_captures > 36/*MAX_XXX*/ )
  {
    state = STOP;
  } else {
    state = TURN;
  }
}

} //namespace pano_app
