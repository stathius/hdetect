#ifndef PROJECTTOOLS_HPP
#define PROJECTTOOLS_HPP

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>

/**
 *
 * @param rng A random generator.
 * @return A vector of colors.
 */
std::vector<cv::Scalar> randomColors(cv::RNG& rng);


/**
 * @param[out] roi The mat to hold the cropped region
 * @param[in] image The image mat
 * @param[in] upleft Upperleft point of the crop in pixels
 * @param[in] boxSize The crop size (horizontaly)
 */
void getCrop(cv::Mat &roi, cv::Mat &image, cv::Rect &rect);

/**
 *
 * @param[in] it The laser point. Used to compute the distance of the box.
 * @param[in] pt2D The point in pixel coords.
 * @param[out] rect Rectangle of the box in pixel coords.
 * @param[in] M_TO_PIXELS The meter to pixels ration
 * @param[in] BODY_RATIO The ratio of the upper body part to the lower body part
 */
void getBox(geometry_msgs::Point32 &it, cv::Point2d &pt2D, cv::Rect &rect, double &M_TO_PIXELS, double &BODY_RATIO);

/**
 *
 * @param cInfo The camera info
 * @param upleft Upper left corner of the box in pixel coords.
 * @param downright Lower right corner of the box in pixel coords.
 * @return TRUE if the box lies into image, FALSE if not
 */
bool checkBox(sensor_msgs::CameraInfo &cInfo, cv::Rect &rect);

/**
 *
 * @param[in] pointIn Input point in cartesian x,y coords.
 * @param[out] pointOut Point projected to pixel coords.
 * @param[in] cam_info The camera info variable. Contains focal length and distortion coeffs.
 * @param[in] transform The transformation between the camera and the laser
 * @param[in] rect Set to RECT or NO_RECT if using the rectified image or not.
 *
 */
void projectPoint(geometry_msgs::Point32 &pointIn, cv::Point2d &pointOut, cv::Mat &K, cv::Mat &D,
		tf::StampedTransform &transform);

/**
 * Converts a point to the laser plane image coordinates
 * @param ptIn
 * @param ptOut
 * @param zoom
 */
void pointToPlane(geometry_msgs::Point32 &ptIn, cv::Point &ptOut, cv::Size &windowSize, int &zoom);

/**
 * Converts the camera info to opencv ready mats.
 * @param[in] cInfo
 * @param[out] K
 * @param[out] D
 */
void CameraInfo2CV(sensor_msgs::CameraInfo &cInfo, cv::Mat &K, cv::Mat &D, int &rect);

#endif
