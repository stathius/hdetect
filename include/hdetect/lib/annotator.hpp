#ifndef ANNOTATOR_HPP
#define ANNOTATOR_HPP

#include <sys/stat.h>
#include <sstream>

#include <rosbag/bag.h>
#include <hdetect/lib/visualizer.hpp>

/**
 * It's a class used for the annotation of the laser and the cropped image data.
 */
class annotator : public visualizer
{
private:

  std::string bagDir;

  /// File handle for CSV output.
  FILE *f;

  /// For writing to bag format.
  rosbag::Bag bag;

  void saveCSV();

  /**
   * @param image Image message
   * @param cInfo CameraInfo message
   * @param lScan LaserScan message
   */
  void saveBag(const sensor_msgs::Image::ConstPtr &image,
               const sensor_msgs::LaserScan::ConstPtr &lScan);



public:
  /**
   *
   * @param bagFile This is the name of bag file where the output data will be written
   */
  annotator(string bagFile);
  ~annotator();

  /**
   * @param image Image message
   * @param cInfo CameraInfo message
   * @param lScan LaserScan message
   */
  void annotateData(const sensor_msgs::Image::ConstPtr &image,
                    const sensor_msgs::LaserScan::ConstPtr &lScan);

};
#endif
