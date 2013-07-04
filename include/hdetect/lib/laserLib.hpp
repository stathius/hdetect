#ifndef LASERLIB_HPP
#define LASERLIB_HPP

//#include "sensor_msgs/LaserScan.h"
#include <filters/filter_chain.h>
#include "sensor_msgs/LaserScan.h"
#include "lengine.hpp"
#include <hdetect/ClusteredScan.h>


/**
 * A class used as a ROS wrapper for the lengine class.
 */
class laserLib {
private:

  /// Needed by getClusters()
  Point3D_str cogL;
  Point3D_str origin;
  geometry_msgs::Point32 pt;
  geometry_msgs::Point32 cogROS;

  /// The shadow filter to preprocess the laser
  filters::FilterChain<sensor_msgs::LaserScan> laserFilter;
  /// The filtered scan
  sensor_msgs::LaserScan filtScan;

  /// This is a people2d_engine class to hold the laser scan data for it.
  laserscan_data libScanData;

  /// people2d_engine object used to make the segmentation and compute the features.
  lengine *libEngine;

  /// Parameters of the libEngine. Initialized in constructor.
  lengine_params libEngineParams;

  ///  Vector to hold the clusters of each scan in people2d_engine format
  std::vector<Point3D_container> clusters;  // lgeometry.hpp

  /// The feature vector of the lengine format
  std::vector < std::vector <float> > descriptor;
  float angle_min;
  float angle_max;
  float angle_inc;

  // Helping variables to copy the header to the new msgs
  ros::Time stamp;
  uint32_t seq;
  std::string frame_id;

  /**
   *
   * @param ls[in] The input scan to be converted
   * @param libScan [out] The laserscan_data variable to hold the lengine compatible data
   */
  void scan2lib(sensor_msgs::LaserScan &ls, laserscan_data &libScan);

  /**
   *
   * @param features[out] Exported cluster features.
   */
  void features2ROS(hdetect::ClusteredScan &features);

public:
  /// Null constructor
  laserLib();
  ~laserLib();

  /**
   * Constructor that receiver parameters from the calling class (e.g. detector)
   * @param jumpdist The cluster segmentation distance
   * @param feature_set Feature set to be used (0 = 17, 1 = 63, 2 = 73)
   * @param laser_range The maximum trusted laser range
   */
  laserLib(double &jumpdist, int feature_set, double laser_range);

  /**
   *
   * @param features[out] Exported cluster features
   */
  void getFeatures(hdetect::ClusteredScan &features);

  /**
   *
   * @param features[out] Where the clusters are going to be exported.
   */
  void getClusters(hdetect::ClusteredScan &laserClusters);

  /**
   *
   * @param ls LaserScan to be loaded
   */
  void loadScan(sensor_msgs::LaserScan ls);
};

#endif
