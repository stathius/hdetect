#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

// ROS_INCLUDES
#include <std_msgs/Byte.h>
// OPENCV INCLUDES
#include <opencv2/highgui/highgui.hpp>

#include <hdetect/lib/detector.hpp>

#define FALSE 0
#define TRUE 1

#define INITIAL_ZOOM 25

static const char C_WINDOW[] = "Annotation: Camera + Laser";
static const char L_WINDOW[] = "Annotation: Laser";

using namespace std;
using namespace cv;

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
	Mat laserPlane;

	/// Color image mat.
	Mat colorImage;

	/// Keeps the color to draw the segments
	vector<Scalar> pallete;

	/// Holds the color for each cluster.
	/// Based on where it is located so we have less variation in color appereance.
	Scalar color;

public:
	visualizer();
	~visualizer();

	/// Returns the color of point according to its position
	Scalar getColor(geometry_msgs::Point32 &point);

	/// Returns the ClusteredScan from the object
	hdetect::ClusteredScan getClusteredScan() { return clusterData; };

	/// Set the laserplane for visualizing purposes
	/// @param lp The computed laser plane image.
	void setLaserPlane(Mat lp) { laserPlane = lp; };

	/**
	 * @param image Image message
	 * @param cInfo CameraInfo message
	 * @param lScan LaserScan message
	 */
	void visualizeData(const sensor_msgs::Image::ConstPtr &image,
			const sensor_msgs::LaserScan::ConstPtr &lScan);
};
#endif
