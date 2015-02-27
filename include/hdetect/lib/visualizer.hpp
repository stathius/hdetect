#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

// ROS_INCLUDES
#include <std_msgs/Byte.h>
// OPENCV INCLUDES
#include <opencv2/highgui/highgui.hpp>

#include "hdetect/lib/detector.hpp"

/// A class for visualizing the segmentation and detection process of our system.
class visualizer : public detector
{
    protected:

        /// Publisher used to count the hz rate of the node
        ros::Publisher pub;
        std_msgs::Byte dummy;

        CvFont font_;

        ///  The zoom used to create the plane.
        int zoom;

        ///  Mat to draw the laser plan.
        cv::Mat laserPlane;

        /// Color image mat.
        cv::Mat colorImage;

        /// Keeps the color to draw the segments
        std::vector<cv::Scalar> pallete;

        /// Holds the color for each cluster.
        /// Based on where it is located so we have less variation in color appereance.
        cv::Scalar color;

        /// Output string stream for visualization purposes.
        std::ostringstream ss;

    public:
        visualizer();
        ~visualizer();

        /// Returns the color of point according to its position
        cv::Scalar getColor(geometry_msgs::Point32 &point);

        /// Returns the ClusteredScan from the object
//        std::vector<hdetect::ClusteredScan> &getClusteredData();

        /// Set the laserplane for visualizing purposes
        /// @param lp The computed laser plane image.
//        void setLaserPlane(cv::Mat lp);

        /**
         * @param image Image message
         * @param cInfo CameraInfo message
         * @param lScan LaserScan message
         */
        void visualizeData(const sensor_msgs::Image::ConstPtr &image,
                           const sensor_msgs::LaserScan::ConstPtr &lScan);

    private:
        /**
         * Absolute value of Point32
         */
        float distance(geometry_msgs::Point32 point);

        /**
         * Draw the laser scan in a 2d plane
         * It's a callback function for the zoom trackbar handler that's why it's a bit fuzzy.
         * The obj_class class must be a visualizer or a derived class.
         * @param zoom The zoom factor
         * @param obj The object where to get the ClusteredScan data
         */
        void plotLaser(int zoom);

        std::deque<cv::Scalar> colors;
};
#endif
