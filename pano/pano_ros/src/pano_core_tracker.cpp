// ROS communication
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// Msg -> OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include <boost/thread.hpp>

class PanoTrackerNode
{
  // ROS communication
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  boost::mutex image_mutex_;

  enum { WAITING, SELECTING, TRACKING };
  int state_;
  cv::Point click_point_, end_point_;
  int size_x_, size_y_;
  int iterations_; /// @todo dynamic reconfigure
  int precision_;

  static const int THICKNESS = 3;

public:

  PanoTrackerNode()
    : it_(nh_),
      state_(WAITING),
      iterations_(5),
      precision_(2)
  {
    cv::namedWindow("Pano", 0);
    cvSetMouseCallback("Pano", &PanoTrackerNode::mouseCb, this);
    cvStartWindowThread();

    image_sub_ = it_.subscribe("image", 1, &PanoTrackerNode::imageCb, this);
  }

  ~PanoTrackerNode()
  {

  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg)
  {
    image_mutex_.lock(); /// @todo Exception safety

    cv::Mat source = bridge_.imgMsgToCv(image_msg, "bgr8");

    cv::Mat display;
    if (state_ == WAITING) {
      display = source;
    }
    else if (state_ == SELECTING) {
      // Draw the currently selected ROI
      cv::cvtColor(source, display, CV_GRAY2BGR);
      cv::rectangle(display, click_point_, end_point_, CV_RGB(255,0,0), THICKNESS);
    }
    else if (state_ == TRACKING) {
//      boost::shared_ptr<imageStruct> image = toImageStruct(source);
//
//      // Track the current image
//      if (MakeTrack(&tracker_, image.get())) {
//        ROS_ERROR("MakeTrack failed");
//        display = source;
//      }
//
//      // Draw the reprojection
//      cv::cvtColor(source, display, CV_GRAY2BGR);
//      drawDetection(display);
//      ROS_INFO("ZNCC = %f", GetZNCC(&tracker_));
    }

    image_mutex_.unlock(); // Must happen before imshow!
    cv::imshow("Pano", display);
  }

  static void mouseCb(int event, int x, int y, int flags, void* param)
  {
    PanoTrackerNode* node = reinterpret_cast<PanoTrackerNode*>(param);

    if (event == CV_EVENT_LBUTTONDOWN) {
      boost::lock_guard<boost::mutex> guard(node->image_mutex_);
      node->state_ = SELECTING;
      node->click_point_ = cv::Point(x, y);
    }
    else if (node->state_ == SELECTING && event == CV_EVENT_MOUSEMOVE) {
      boost::lock_guard<boost::mutex> guard(node->image_mutex_);
      node->end_point_ = cv::Point(x, y);
    }
    else if (event == CV_EVENT_LBUTTONUP) {
      boost::lock_guard<boost::mutex> guard(node->image_mutex_);

      cv::Point pt1(node->click_point_), pt2(x, y);
//      int pos_x = std::min(pt1.x, pt2.x);
//      int pos_y = std::min(pt1.y, pt2.y);
      node->size_x_ = std::abs(pt1.x - pt2.x);
      node->size_y_ = std::abs(pt1.y - pt2.y);
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PanoTrackerNode");
  PanoTrackerNode node;
  ros::spin();
}
