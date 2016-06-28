/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /include/turtlebot_panorama/panorama.cpp
 *
 * @brief Panorama app class and ROS node implementation
 *
 * @date 08/01/2013
 *
 * @author Younghun Ju, Jihoon Lee and Marcus Liebhardt
 **/

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <turtlebot_panorama/panorama.h>

namespace turtlebot_panorama
{

PanoApp::PanoApp() : nh(), priv_nh("~")
{
  std::string name = ros::this_node::getName();

  ros::param::param<int>("~default_mode", default_mode, 1);
  ros::param::param<double>("~default_pano_angle", default_pano_angle, (2 * M_PI));
  ros::param::param<double>("~default_snap_interval", default_snap_interval, 2.0);
  ros::param::param<double>("~default_rotation_velocity", default_rotation_velocity, 0.3);

  ros::param::param<std::string>("~camera_name", params["camera_name"], "/camera/rgb");
  ros::param::param<std::string>("~bag_location", params["bag_location"], "/home/turtlebot/pano.bag");

  pub_log = priv_nh.advertise<std_msgs::String>("log", 100);
}

PanoApp::~PanoApp()
{
  delete pano_ros_client;
}

void PanoApp::init()
{
  //***************************
  // public API for the app
  //***************************
  srv_start_pano = priv_nh.advertiseService("take_pano", &PanoApp::takePanoServiceCb, this);
  sub_start_pano = priv_nh.subscribe("take_pano", 1, &PanoApp::takePanoCb, this);
  sub_stop_pano = priv_nh.subscribe("stop_pano", 1, &PanoApp::stopPanoCb, this);
  image_transport::ImageTransport it_priv(priv_nh);
  pub_stitched = it_priv.advertise("panorama", 1, true);

  //***************************
  // Robot control
  //***************************
  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  sub_odom = nh.subscribe("odom", 100, &PanoApp::odomCb, this);

  //***************************
  // pano_ros API
  //***************************
  pano_ros_client = new actionlib::SimpleActionClient<pano_ros::PanoCaptureAction>("pano_server", true);
  log("Waiting for Pano ROS server ...");
  pano_ros_client->waitForServer(); // will wait for infinite time
  log("Connected to Pano ROS server.");
  pub_action_snap = nh.advertise<std_msgs::Empty>("pano_server/snap", 100);
  pub_action_stop = nh.advertise<std_msgs::Empty>("pano_server/stop", 100);
  image_transport::ImageTransport it_pano(nh);
  sub_stitched = it_pano.subscribe("pano_server/stitch", 1, &PanoApp::stitchedImageCb, this);

  cmd_vel.linear.x = 0.0f;
  cmd_vel.linear.y = 0.0f;
  cmd_vel.linear.z = 0.0f;
  cmd_vel.angular.x = 0.0f;
  cmd_vel.angular.y = 0.0f;
  cmd_vel.angular.z = 0.0f;
  zero_cmd_vel = cmd_vel;
  is_active = false;
  continuous = false;
  ang_vel_cur = 0.0;
  given_angle = 0.0;
  angle = 0.0;
  last_angle = 0.0;
}

void PanoApp::spin()
{
  ros::Rate loop_rate(10);
  double start_time;
  start_time = 0.0;
  bool take_snapshot = false;

  while (ros::ok())
  {
    if (is_active)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Degrees to go: " << radians_to_degrees(std::abs(given_angle - angle)));
      if ((given_angle - angle) <= 0.0174) // check, if target angle is reached (< 1 degree)
      {
        snap();
        pub_cmd_vel.publish(zero_cmd_vel);
        ros::Duration(1.0).sleep(); // give the pano server some time to retrieve the last pciture
        stopPanoAction();
      }
      else
      {
        if (continuous) // then snap_interval is a duration
        {
          double now = ros::Time::now().toSec();
          if ((now - start_time) > snap_interval)
          {
            snap();
            start_time = now;
          }
          rotate();
        }
        else
        {
          if (hasReachedAngle())
          {
            pub_cmd_vel.publish(zero_cmd_vel); // stop before taking a snapshot
            take_snapshot = true;
          }
          if (take_snapshot)
          {
            if (std::abs(ang_vel_cur) <= 0.000001) // wait until robot has stopped
            {
              snap();
              take_snapshot = false;
            }
            else
            {
              std::stringstream ss;
              std::string str;
              ss << "Waiting for robot to stop ... (speed = " << ang_vel_cur << ")";
              str = ss.str();
              log(str);
            }
          }
          else
          {
            rotate();
          }
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}


void PanoApp::snap()
{
  log("snap");
  pub_action_snap.publish(empty_msg);
}

void PanoApp::rotate()
{
  log("rotate");
  pub_cmd_vel.publish(cmd_vel); // rotate a bit
}

bool PanoApp::hasReachedAngle()
{
  if (angle > last_angle + degrees_to_radians(snap_interval))
  {
    last_angle = angle;
    return true;
  }
  else
  {
    return false;
  }
}

void PanoApp::odomCb(const nav_msgs::OdometryConstPtr& msg)
{
  static double heading_last = 0.0f;
  double heading = 0.0f;

  Eigen::AngleAxisf angle_axis(Eigen::Quaternionf(msg->pose.pose.orientation.w,
                                                  msg->pose.pose.orientation.x,
                                                  msg->pose.pose.orientation.y,
                                                  msg->pose.pose.orientation.z));
  Eigen::Vector3f axis = angle_axis.axis();

  if (axis(2) > 0.0)
  {
    heading = angle_axis.angle();
  }
  else if (axis(2) < 0.0)
  {
    heading = -1.0 * angle_axis.angle();
  }

  angle += std::abs(wrap_angle(heading - heading_last));
  heading_last = heading;
  ang_vel_cur = msg->twist.twist.angular.z;
}

//*************************
// Public interface
//*************************
bool PanoApp::takePanoServiceCb(turtlebot_msgs::TakePanorama::Request& request,
                                turtlebot_msgs::TakePanorama::Response& response)
{
  if (is_active && (request.mode == request.CONTINUOUS || request.mode == request.SNAPANDROTATE))
  {
    log("Panorama creation already in progress.");
    response.status = response.IN_PROGRESS;
  }
  else if (is_active && (request.mode == request.STOP))
  {
    stopPanoAction();
    is_active = false;
    log("Panorama creation stopped.");
    response.status = response.STOPPED;
    return true;
  }
  else if (!is_active && (request.mode == request.STOP))
  {
    log("No panorama creation in progress.");
    response.status = response.STOPPED;
    return true;
  }
  else
  {
    if (request.pano_angle <= 0.0)
    {
      log("Specified panorama angle is zero or negative! Panorama creation aborted.");
      return true;
    }
    else if (request.snap_interval <= 0.0)
    {
      log("Specified snapshot interval is zero or negative! Panorama creation aborted.");
      return true;
    }
    else if (request.rot_vel == 0.0)
    {
      log("Specified rotating speed is zero! Panorama creation aborted.");
      return true;
    }
    else
    {
      given_angle = degrees_to_radians(request.pano_angle);
      snap_interval = request.snap_interval;
      cmd_vel.angular.z = request.rot_vel;
    }
    if (request.mode == turtlebot_msgs::TakePanoramaRequest::CONTINUOUS)
    {
      continuous = true;
    }
    else
    {
      continuous = false;
    }
    log("Starting panorama creation.");
    startPanoAction();
    response.status = response.STARTED;
  }
  return true;
}

void PanoApp::takePanoCb(const std_msgs::EmptyConstPtr& msg)
{
  if (is_active)
  {
    log("Panorama creation already in progress.");
    return;
  }
  if (default_mode == 0)
  {
    continuous = false;
  }
  else if (default_mode == 1)
  {
    continuous = true;
  }
  else
  {
    log("No default panorama mode set. Will use continuous mode.");
    continuous = true;
  }
  given_angle = default_pano_angle;
  snap_interval = default_snap_interval;
  cmd_vel.angular.z = default_rotation_velocity;
  log("Starting panorama creation.");
  startPanoAction();
}

void PanoApp::stopPanoCb(const std_msgs::EmptyConstPtr& msg)
{
  if (is_active)
  {
    stopPanoAction();
    is_active = false;
    log("Panorama creation stopped.");
  }
  else
  {
    log("No panorama is currently being created.");
  }
}

//***************
// Pano ROS API
//***************
void PanoApp::startPanoAction()
{
  pano_ros::PanoCaptureGoal goal;
  goal.bag_filename = params["bag_location"];
  goal.camera_topic = params["camera_name"];
  pano_ros_client->sendGoal(goal,
                            boost::bind(&PanoApp::doneCb, this, _1, _2),
                            boost::bind(&PanoApp::activeCb, this),
                            boost::bind(&PanoApp::feedbackCb, this, _1));
  log("Pano ROS action goal sent.");
  angle = 0.0;
  last_angle = 0.0;
}

void PanoApp::stopPanoAction()
{
  pub_action_stop.publish(empty_msg);
  is_active = false;
  log("Start of stitching triggered.");
}

void PanoApp::doneCb(const actionlib::SimpleClientGoalState& state, const pano_ros::PanoCaptureResultConstPtr& result)
{
  std::string str = "Pano action finished in state : " + state.toString();
  log(str);
  is_active = false;
}

void PanoApp::activeCb()
{
  log("Pano action goal just went active.");
  ros::Duration(1.0).sleep(); // Wait a bit for the server to activate the capture job
  snap(); // take first picture before start turning
  go_active = true;
}

void PanoApp::feedbackCb(const pano_ros::PanoCaptureFeedbackConstPtr& feedback)
{
  if (go_active) // first pictures has been taken, so start the control loop
  {
    is_active = true;
    go_active = false;
  }
  std::stringstream ss;
  std::string str;
  if (continuous) // then snap_interval is a duration
  {
    ss << "Got pano action feedback: " << (int)feedback->n_captures << " pictures captured.";
  }
  else // then snap_interval is an angle
  {
    ss << "Got pano action feedback: " << (int)feedback->n_captures << " of "
       << int(given_angle/degrees_to_radians(snap_interval))
       << " pictures captured.";
  }
  log(ss.str());
}

void PanoApp::stitchedImageCb(const sensor_msgs::ImageConstPtr& msg)
{
  pub_stitched.publish(msg);
  std::cout << "encoding: " << msg->encoding << std::endl;
  std::cout << "is_bigendian: " << msg->is_bigendian << std::endl;

  log("Published new panorama picture.");
}

//*************
// Logging
//*************
void PanoApp::log(std::string log)
{
  std_msgs::String msg;
  msg.data = log;
  pub_log.publish(msg);
  ROS_INFO_STREAM(log);
}

} //namespace turtlebot_panorama

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_panorama");

  turtlebot_panorama::PanoApp pano;
  pano.log("Panorama app starting...");
  pano.init();
  pano.log("Panorama application initialised.");
  pano.spin();
  pano.log("Bye, bye!");

  return 0;
}
