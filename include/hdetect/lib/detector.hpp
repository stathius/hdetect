#ifndef DETECTOR_HPP
#define DETECTOR_HPP

// ROS
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse_yml.h>
#include <cv_bridge/cv_bridge.h>
// OPENCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>
// MY INCLUDES
#include "laserLib.hpp"
#include "projectTools.hpp"
#include <hdetect/ClusterClass.h>


/// A structure to hold all the parameters needed by the detector
struct detectorParameters {

	/// If the rectified image is used or not. For projection purposes.
	int rect;

	/// The number of features is loaded as a parameter
	/// Also used in laserLib for defining the feature set
	int no_features;

	/// The HoG group threshold is defined as a parameter (normally 2)
	int hog_group_threshold;

	/// Defines if HoG multiscale detection uses meanshift clustering (default 1)
	int hog_meanshift;

	/// The HoG SVM classifier bias (normally 0)
	int hog_hit_threshold;

	/// The camera info is loaded from the file "yaml/camera_calib.yaml"
	sensor_msgs::CameraInfo cInfo;

	/// The hz timeout for the tf (normally below the slower sensor)
	int tf_timeout;

	/// The sigmoid parameters of the laser and the camera classifiers
	/// They are provided by Platts scaling
	double laserA, laserB;
	double cameraA, cameraB;

	/// The meter to pixels ratio based on the camera sensor, the lens and the size of the ROI
	double m_to_pixels;
	/// Upper/lower body ratio
	double body_ratio;

	/// Jumping distance for the laser segmentation
	double jumpdist;

	/// Feature set (0 = 17, 1 = 63, 2 = 73)
	int feature_set;

	/// Maximum allowed laser range
	double laser_range;
};

class detector {
protected:

	/// The projection matrices in openv format
	/// Camera matrix
	cv::Mat K;
	/// Distortion coeffs
	cv::Mat D;

	/// Detector parameters
	detectorParameters params;

	/// Local node handle, used to get the file parameters, subscribers, publishers etc.
	ros::NodeHandle nh;

	/// Used to listen the transform between the laser and the camera.
	tf::TransformListener tf_listener_;
	tf::StampedTransform transform;

	///  When the image was captured.
	ros::Time acquisition_time;

	/// Curent scan number
	int scanNo;

	/// Crop box corners
	cv::Point upleft, downright;

	/// Crop box size
	int boxSize;

	/// Point projected to pixel coordinates.
	cv::Point2d prPixel;

	/// Object used to do the low lever segmentation and feature extraction from the scan.
	laserLib *laserProcessor;

	///  A pointer to the opencv converted image.
	cv_bridge::CvImagePtr cv_ptr;

	// Vectors where the class of the detector are stored
	std::vector<int> laserClass;
	std::vector<int> cameraClass;
	std::vector<int> fusionClass;

	// The adaboost detector for the laser
	CvBoost boost;

	// The laser feature matrix
	cv::Mat lFeatures;

	// The HoG detector for the image
	cv::HOGDescriptor hog;

	/// Mat where the temporary crop will be saved
	Mat crop;

	// Vectors to hold the class and the probability of the ROI
	std::vector<Rect> hogFound;
	std::vector<double> hogPred;

	// Publishes the detected humans coordinates and probability
	ros::Publisher detectionPublisher;

	/// Contains the laser clusters, annotation, features, , annotation, if it should be fused etc.
	hdetect::ClusteredScan *clusterData;

	/// Contains the coordinates, the labels and the probabilities of the detections
	hdetect::ClusterClass *detections;

	/// Does the laser segmentation, feature extraction etc into scanClusters
	void processLaser(const sensor_msgs::LaserScan::ConstPtr &lScan, hdetect::ClusteredScan *clusterData);

	/// Brings sensor_msgs::Image to an opencv accesible pointer.
	void getImage(const sensor_msgs::Image::ConstPtr &image);

	/// Returns the transform between image and lScan
	void getTF(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::LaserScan::ConstPtr &lScan);

	/// Does the rest of the laser processing, find projected and fused segments
	/// Uses directly scanClusters
	void findProjectedClusters(hdetect::ClusteredScan *clusterData);

	/// Detects if there is a pedestrian in the cluster and or ROI
	/// Gives the probabilities and the class output of each detector
	void detectFusion(hdetect::ClusteredScan *clusterData, hdetect::ClusterClass *detections);

	/// Detects if there is a pedestrian using only the camera ROI
	/// Gives the probabilities and the class output of each detector
	void detectCamera(hdetect::ClusteredScan *clusterData);

	/// Finds the class and the probability for a given sample of laser features
	void classifyLaser(std_msgs::Float32MultiArray &features);

	/// Finds the class and the probability for a given crop of the image
	void classifyCamera(geometry_msgs::Point32 &cog, double &prob);

	void saveDetection() {};

public:
	detector();
	~detector() {};

	/** Extracts all the info from an imaga/laserscan pair
	 *	First it segments the laser scan and finds the cog for each cluster.
	 *	Then translates each cog into the corresponding pixel values.
	 *	Crops the ROI from the image.
	 *
	 * @param image Image message
	 * @param lScan LaserScan message
	 */
	void detectHumans(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::LaserScan::ConstPtr &lScan);

	/**
	 * Used only for annotation purposes
	 * @param cs
	 */
	//void setClusters(hdetect::ClusteredScan cs);
	//hdetect::ClusteredScan getClusters();
};
#endif
