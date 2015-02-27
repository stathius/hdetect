#ifndef BAGREADER_HPP
#define BAGREADER_HPP

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <rosgraph_msgs/Clock.h>

#include "bagSubscriber.hpp"

/**
 * Opens a bag file and reads its data
 */
class bagReader
{
private:
  ros::NodeHandle nh_;

//  The clock publisher
  ros::Publisher clock_pub_;
  rosgraph_msgs::Clock clockmsg;

  std::string bagFile;

public:
  /**
   * @param fname The filename of the bag file
   */
  bagReader(std::string fname);
  ~bagReader();
  /**
   *
   * @param topics A vector of the topics to be read
   * @param cameraImage_sub_  Subscriber to handle the camera messages
   * @param cameraInfo_sub_ Subscriber to handle the camera info messages
   * @param laserScan_sub_ Subscriber to handle the laser scan messages
   */
  void loadBag(std::vector<std::string> topics,
               bagSubscriber<sensor_msgs::Image> &cameraImage_sub_,
               bagSubscriber<sensor_msgs::LaserScan> &laserScan_sub_);
};

#endif
