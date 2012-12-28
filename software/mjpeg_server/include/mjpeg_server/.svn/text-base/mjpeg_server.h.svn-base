/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef MJPEG_SERVER_H_
#define MJPEG_SERVER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

/*
 * Maximum number of server sockets (i.e. protocol families) to listen.
 */
#define MAX_NUM_SOCKETS    100
#define IO_BUFFER     256
#define BUFFER_SIZE   1024

namespace mjpeg_server {

/* the webserver determines between these values for an answer */
typedef enum {
    A_UNKNOWN,
    A_STREAM,
    A_SNAPSHOT,
} answer_t;

/*
 * the client sends information with each request
 * this structure is used to store the important parts
 */
typedef struct {
    answer_t type;
    char *parameter;
    char *client;
    char *credentials;
} request;

/* the iobuffer structure is used to read from the HTTP-client */
typedef struct {
    int level;              /* how full is the buffer */
    char buffer[IO_BUFFER]; /* the data */
} iobuffer;

/**
 * @class ImageBuffer
 * @brief
 */
class ImageBuffer {
public:
  ImageBuffer() {}
  sensor_msgs::Image msg;
  boost::condition_variable condition_;
  boost::mutex mutex_;
};

/**
 * @class MJPEGServer
 * @brief
 */
class MJPEGServer {
public:
  /**
   * @brief  Constructor
   * @return
   */
  MJPEGServer(ros::NodeHandle& node);

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~MJPEGServer();

  /**
   * @brief  Runs whenever a new goal is sent to the move_base
   * @param goal The goal to pursue
   * @param feedback Feedback that the action gives to a higher-level monitor, in this case, the position of the robot
   * @return The result of the execution, ie: Success, Preempted, Aborted, etc.
   */
  void execute();

  /**
   * @brief  Starts the server and spins
   */
  void spin();

  /**
   * @brief  Stops the server
   */
  void stop();

  /**
   * @brief  Closes all client threads
   */
  void cleanUp();

  /**
   * @brief  Client thread function
   *         Serve a connected TCP-client. This thread function is called
   *         for each connect of a HTTP client like a webbrowser. It determines
   *         if it is a valid HTTP request and dispatches between the different
   *         response options.
   */
  void client(int fd);

private:
  typedef std::map<std::string, ImageBuffer*> ImageBufferMap;
  typedef std::map<std::string, image_transport::Subscriber> ImageSubscriberMap;
  typedef std::map<std::string, std::string> ParameterMap;

  std::string header;
  
  ImageBuffer* getImageBuffer(const std::string& topic);


  void imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& topic);

  /**
   * @brief Rotate input image by 180 degrees.
   * @param input image
   * @param output image
   */
  void invertImage(const cv::Mat& input, cv::Mat& output);

  /**
   * @brief Send an error message.
   * @param fildescriptor fd to send the answer to
   * @param error number
   * @param error message
   */
  void sendError(int fd, int which, const char *message);

  /**
   * @brief Send a complete HTTP response and a stream of JPG-frames.
   * @param fildescriptor fd to send the answer to
   * @param parameter string
   */
  void sendStream(int fd, const char *parameter);

  /**
   * @brief Send a complete HTTP response and a single JPG-frame.
   * @param fildescriptor fd to send the answer to
   * @param parameter string
   */
  void sendSnapshot(int fd, const char *parameter);

  /**
   * @brief Initializes the iobuffer structure properly
   * @param Pointer to already allocated iobuffer
   */
  void initIOBuffer(iobuffer *iobuf);

  /**
   * @brief Initializes the request structure properly
   * @param Pointer to already allocated req
   */
  void initRequest(request *req);

  /**
   * @brief If strings were assigned to the different members free them.
   *        This will fail if strings are static, so always use strdup().
   * @param req: pointer to request structure
   */
  void freeRequest(request *req);

  /**
   * @brief read with timeout, implemented without using signals
   *        tries to read len bytes and returns if enough bytes were read
   *        or the timeout was triggered. In case of timeout the return
   *        value may differ from the requested bytes "len".
   * @param fd.....: fildescriptor to read from
   * @param iobuf..: iobuffer that allows to use this functions from multiple
   *                 threads because the complete context is the iobuffer.
   * @param buffer.: The buffer to store values at, will be set to zero
                     before storing values.
   * @param len....: the length of buffer
   * @param timeout: seconds to wait for an answer
   * @return buffer.: will become filled with bytes read
   * @return iobuf..: May get altered to save the context for future calls.
   * @return func().: bytes copied to buffer or -1 in case of error
   */
  int readWithTimeout(int fd, iobuffer *iobuf, char *buffer, size_t len, int timeout);

  /**
   * @brief Read a single line from the provided fildescriptor.
   *        This funtion will return under two conditions:
   *        - line end was reached
   *        - timeout occured
   * @param fd.....: fildescriptor to read from
   * @param iobuf..: iobuffer that allows to use this functions from multiple
   *                 threads because the complete context is the iobuffer.
   * @param buffer.: The buffer to store values at, will be set to zero
                     before storing values.
   * @param len....: the length of buffer
   * @param timeout: seconds to wait for an answer
   * @return buffer.: will become filled with bytes read
   * @return iobuf..: May get altered to save the context for future calls.
   * @return func().: bytes copied to buffer or -1 in case of error
   */
  int readLineWithTimeout(int fd, iobuffer *iobuf, char *buffer, size_t len, int timeout);

  /**
   * @brief Decodes the data and stores the result to the same buffer.
   *        The buffer will be large enough, because base64 requires more
   *        space then plain text.
   * @param base64 encoded data
   * @return plain decoded data
   */
  void decodeBase64(char *data);

  /**
   * @brief Convert a hexadecimal ASCII character to integer
   * @param ASCII character
   * @return corresponding value between 0 and 15, or -1 in case of error
   */
  int hexCharToInt(char in);

  /**
   * @brief Replaces %XX with the character code it represents, URI
   * @param string to unescape
   * @return 0 if everything is ok, -1 in case of error
   */
  int unescape(char *string);

  /**
   * @brief Split string into a list of tokens
   * @param string to split
   * @return vector of tokens
   */
  void splitString(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiter = " ");

  /**
   * @brief Convert a string to an integer
   * @param string
   * @param default return value
   * @return corresponding value, default_value in case of error
   */
  int stringToInt(const std::string& str, const int default_value = 0);

  /**
   * @brief Decodes URI parameters in form of ?parameter=value
   * @param URI string
   * @return a map of parameter/value pairs
   */
  void decodeParameter(const std::string& parameter, ParameterMap& parameter_map);


  ros::NodeHandle node_;
  image_transport::ImageTransport image_transport_;
  int port_;

  int sd[MAX_NUM_SOCKETS];
  int sd_len;

  bool stop_requested_;
  char* www_folder_;

  ImageBufferMap image_buffers_;
  ImageSubscriberMap image_subscribers_;
  boost::mutex image_maps_mutex_;

};

}

#endif

