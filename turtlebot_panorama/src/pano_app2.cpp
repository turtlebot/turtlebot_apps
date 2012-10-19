#include <turtlebot_panorama/pano_app2.h>

namespace turtlebot_panorama {

PanoApp::PanoApp() : nh(), priv_nh("~") 
{
  std::string name = ros::this_node::getName();

  ros::param::param<std::string>("~take_pano_name",params["take_pano_name"],"take_pano");
  ros::param::param<std::string>("~pub_result_name",params["pub_result_name"],"panorama");
  ros::param::param<std::string>("~cmd_vel_name",params["cmd_vel_name"],"/cmd_vel");
  ros::param::param<std::string>("~odometry_name",params["odometry_name"],"/odom");
  ros::param::param<std::string>("~pano_server_name",params["pano_server_name"],name+"/pano_server");
    params["pano_snap_name"] = params["pano_server_name"] + "/snap";
    params["pano_stop_name"] = params["pano_server_name"] + "/stop";
    params["pano_result_name"] = params["pano_server_name"] + "/stitch";
  ros::param::param<std::string>("~camera_name",params["camera_name"],"/camera/rgb");
  ros::param::param<std::string>("~bag_location",params["bag_location"],"/opt/pan00.bag");
  ros::param::param<std::string>("~log",params["log"],"log");

  params["take_pano_srv_name"] = "take_pano";

  // Redirecting log
  pub_log = priv_nh.advertise<std_msgs::String>(params["log"],100);
}

PanoApp::~PanoApp()
{
  delete ac;
}

void PanoApp::init() 
{
  ///////////////////
  // Public Channel
  ///////////////////
  // public API from this node
  srv_pano = priv_nh.advertiseService(params["take_pano_srv_name"],&PanoApp::takeService,this);
  sub_pano = priv_nh.subscribe(params["take_pano_name"], 1, &PanoApp::takeCb, this);
  image_transport::ImageTransport it(priv_nh);
  pub_stitched = it.advertise(params["pub_result_name"], 1);

  ///////////////////
  // Pano_ros 
  ///////////////////
  // Setting Action Client and wait until the server is up
  ac = new actionlib::SimpleActionClient<pano_ros::PanoCaptureAction>(params["pano_server_name"], true);
  log("Wait for Capture Server.");
  ac->waitForServer(); //will wait for infinite time
  log("Done");
  // snap and stop from pano action server
  pub_snap = priv_nh.advertise<std_msgs::Empty>(params["pano_snap_name"], 100 );
  pub_stop = priv_nh.advertise<std_msgs::Empty>( params["pano_stop_name"], 100 );
  // To receive the result
  sub_stitched = it.subscribe(params["pano_result_name"], 1, &PanoApp::imageCb, this);


  ////////////////////
  // Robot Channel
  ////////////////////
  // To roate the robot
  pub_cmd_vel =priv_nh.advertise<geometry_msgs::Twist>(params["cmd_vel_name"], 100 );
  // To check how much the robot is rotated
  sub_odom = priv_nh.subscribe(params["odometry_name"], 100, &PanoApp::odomCb, this);
  
  // default command velocity
  cmd_vel.linear.x = 0.0f;
  cmd_vel.linear.y = 0.0f;
  cmd_vel.linear.z = 0.0f;
  cmd_vel.angular.x = 0.0f;
  cmd_vel.angular.y = 0.0f;
  cmd_vel.angular.z = 0.0f; 

  is_active = false;
  
  state=PENDING;
//  log(state);

}

void PanoApp::spin()
{
  ros::Rate loop_rate(10);
  double st,fi;

  st = 0;
  while( ros::ok() ) {
    if(is_active)
    {
      turn();
      fi = ros::Time::now().toSec();
      if(fi - st > snap_interval)
      {
        snap();
        st = fi;
      }
    }
	  ros::spinOnce();
	  loop_rate.sleep();
  }
}

void PanoApp::snap()
{
  log("snap");
  pub_snap.publish( empty );
  state = PENDING;
}

void PanoApp::turn()
{
  log("turn");
  /*
  if( hasReached() ) {
    cmd_vel.angular.z = 0.0f; 
    state = SNAP;
    is_active = false;
  } else {
    cmd_vel.angular.z = ang_vel; 
  }*/
  pub_cmd_vel.publish( cmd_vel );
}

bool PanoApp::hasReached()
{
  if( angle > last_angle + ecl::degrees_to_radians(given_angle) ) {
    last_angle = angle;
    return true;
  } else {
    return false;
  }
}

void PanoApp::stop()
{
  log("stop");
  is_active = false;
  pub_stop.publish( empty );

}

void PanoApp::done()
{
  log("done");
  state= PENDING;
}

/********************
 * Callbacks
 ********************/
bool PanoApp::takeService(TakePano::Request& request,TakePano::Response& response)
{
  if(is_active) {
    if(request.angle < 0.0f)
    {
      stop();
      cmd_vel.angular.z = 0.0f;
      response.status = response.STOPPED;
    }
    else {
      response.status = response.IN_PROGRESS;
    }
  }
  else {
    given_angle = request.angle;
    cmd_vel.angular.z = request.zvel;
    snap_interval = request.snap_interval;
    startPano();
    response.status = response.STARTED;
  }

  return true;
}


void PanoApp::takeCb( const std_msgs::EmptyConstPtr& msg )
{
  if( is_active ) return;

  given_angle = 360.0f;
  cmd_vel.angular.z = 0.1f;
  snap_interval = 0.2;
  startPano();
}

void PanoApp::startPano()
{
  //setup and send goal
  pano_ros::PanoCaptureGoal goal;
  goal.bag_filename = params["bag_location"];
  goal.camera_topic = params["camera_name"];

  ac->sendGoal(
    goal, 
    boost::bind(&PanoApp::doneCb,     this, _1, _2),
    boost::bind(&PanoApp::activeCb,   this),
    boost::bind(&PanoApp::feedbackCb, this, _1)
  );
  log("Goal sent.");

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
    log("repub stitched image to public channel.");
    pub_stitched.publish( msg );
    state = PENDING;
    is_active = false;
}

void PanoApp::doneCb( const actionlib::SimpleClientGoalState& state, const pano_ros::PanoCaptureResultConstPtr& result )
{
  /*
  log("Finished in state [%s].", state.toString().c_str() );
  log("pano_id: %d", 		result->pano_id );
  log("n_captures: %d", 	result->n_captures );
  log("bag_filename: %s", 	result->bag_filename.c_str() );*/
  std::string str = "Finished in state : " + state.toString();
  log(str);
  log("Done");
  this->state = DONE; //instead of shutdown.
}

void PanoApp::activeCb( )
{
  log("Goal just went active.");
  state = SNAP;
}

void PanoApp::feedbackCb( const pano_ros::PanoCaptureFeedbackConstPtr& feedback )
{
  std::stringstream ss;
  std::string str;
  ss << "Got Feedback: " << (int)feedback->n_captures << " of picture captured.";
  ss >> str;
  log(str);

  if( /*hasRevoluted()*/angle > ecl::degrees_to_radians(given_angle))// || feedback->n_captures > 36/*MAX_XXX*/ )
  {
    stop();
    cmd_vel.angular.z = 0.0f;
  }
}

void PanoApp::log(std::string log)
{
//  std::cout << log << std::endl;
  ROS_INFO(log.c_str());
  std_msgs::String msg;
  msg.data = log;
  pub_log.publish(msg);
}

} //namespace pano_app
