#ifndef ANNOTATOR_HPP
#define ANNOTATOR_HPP

#include <sys/stat.h>
#include <sstream>
#include <deque>
#include <set>

#include <rosbag/bag.h>
#include <geometry_msgs/Point32.h>
#include <hdetect/lib/visualizer.hpp>

/**
 * It's a class used for the annotation of the laser and the cropped image data.
 */
class annotator : public visualizer
{
    public:
        /**
          *
          * @param bagFile This is the name of bag file where the output data will be written
          */
        annotator(std::string bagFile);
        ~annotator();

        /**
          * @param image Image message
          * @param cInfo CameraInfo message
          * @param lScan LaserScan message
          */
        void annotateData(const sensor_msgs::Image::ConstPtr &image,
                          const sensor_msgs::LaserScan::ConstPtr &lScan);

    private:

        /// Curent scan number
        int scanNo;

        std::string bagDir;

        /// File handle for CSV output.
        FILE *f;

        /// For writing to bag format.
        rosbag::Bag bag;

        std::deque<geometry_msgs::Point32> prev_points;

        void saveCSV();

        /**
          * @param image Image message
          * @param cInfo CameraInfo message
          * @param lScan LaserScan message
          */
        void saveBag(const sensor_msgs::Image::ConstPtr &image,
                     const sensor_msgs::LaserScan::ConstPtr &lScan);

        float calculateEucDis(geometry_msgs::Point32 &point1, geometry_msgs::Point32 &point2);
};
#endif
