// It reads a bag
// Reads three specified topics from a vector of string
// Publishes clock time

#include "hdetect/lib/bagReader.hpp"


/**
 * Reads all of the messages from the bag file
 * Check what type each message is and it sends it to the appropriate subscriber
 * Three types of messages are supported sensor_msgs::Image, sensor_msgs::CameraInfo and sensor_msgs::LaserScan
 * The clock is published when a camera or a laser scan message arrives. Camera info messages have the same
 * timestamp as camera image ones so it's not needed to publish their time.
 */
void bagReader::loadBag(std::vector<std::string> topics, bagSubscriber<sensor_msgs::Image> &cameraImage_sub_,
                        bagSubscriber<sensor_msgs::LaserScan> &laserScan_sub_)
{
  ROS_INFO("[BAG_READER] Opening %s", bagFile.c_str());

  rosbag::Bag bag(bagFile, rosbag::bagmode::Read);

  // push topics we are going to read from rosbag file
  std::string cameraImageIn = topics[0];
  std::string laserScanIn = topics[1];

  ROS_INFO("[BAG_READER] topics %s %s", topics[0].c_str(),  topics[1].c_str());

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  ROS_INFO("[BAG_READER] Bag loaded for reading.");

  int count=0;

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  //for ( rosbag::View::iterator it = view.begin(); it != view.end(); it++ )
  {
    //rosbag:: MessageInstance const m = *it;

    count++;

    if (m.getTopic() == cameraImageIn)
    {
      sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
      if (img != NULL)
      {
      clockmsg.clock = m.getTime();
      clock_pub_.publish(clockmsg);
      cameraImage_sub_.newMessage(img);
      }
    }

    if (m.getTopic() == laserScanIn)
    {
      sensor_msgs::LaserScanConstPtr ls = m.instantiate<sensor_msgs::LaserScan>();
      if (ls != NULL)
      {
        clockmsg.clock = m.getTime();
        clock_pub_.publish(clockmsg);
        laserScan_sub_.newMessage(ls);
      }
    }
  }

  bag.close();
  ROS_INFO("[BAG_READER] %d messages read.", count);

  ROS_INFO("[BAG_READER] Bag is read.");
}

/// The constructor just opens the bagfile and creates a clock publisher
bagReader::bagReader(std::string fname)
{

  bagFile = fname;

  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("clock", 1);

  ROS_INFO("[BAG_READER] Object created.");
}
