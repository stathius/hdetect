#ifndef PROJECTTOOLS_HPP
#define PROJECTTOOLS_HPP

#include <opencv2/calib3d/calib3d.hpp>
#include "tf/transform_listener.h"
#include "sensor_msgs/CameraInfo.h"
#include <hdetect/ClusteredScan.h>

#define HEIGHT 700 // the laser window height
#define WIDTH 1000 // and width

using namespace std;
using namespace cv;

/**
 *
 * @param rng A random generator.
 * @return A vector of colors.
 */
vector<cv::Scalar> randomColors(cv::RNG& rng);


/**
 * @param[out] roi The mat to hold the cropped region
 * @param[in] image The image mat
 * @param[in] upleft Upperleft point of the crop in pixels
 * @param[in] boxSize The crop size (horizontaly)
 */
void getCrop (cv::Mat &roi, cv::Mat &image, cv::Point &upleft, int &boxSize);

/**
 *
 * @param[in] it The laser point. Used to compute the distance of the box.
 * @param[in] pt2D The point in pixel coords.
 * @param[out] boxWidth Size of the box in pixels
 * @param[out] upleft Upper left corner of the box in pixel coords.
 * @param[out] downright Lower right corner of the box in pixel coords.
 * @param[in] M_TO_PIXELS The meter to pixels ration
 * @param[in] BODY_RATIO The ratio of the upper body part to the lower body part
 */
void getBox(geometry_msgs::Point32 &it, cv::Point2d &pt2D, int &boxSize,
         cv::Point &upleft, cv::Point &downright, double &M_TO_PIXELS, double &BODY_RATIO);

 /**
  *
  * @param cInfo The camera info
  * @param upleft Upper left corner of the box in pixel coords.
  * @param downright Lower right corner of the box in pixel coords.
  * @return TRUE if the box lies into image, FALSE if not
  */
 bool checkBox(sensor_msgs::CameraInfo &cInfo, cv::Point &upleft, cv::Point &downright);

 /**
  *
  * @param[in] pointIn Input point in cartesian x,y coords.
  * @param[out] pointOut Point projected to pixel coords.
  * @param[in] cam_info The camera info variable. Contains focal length and distortion coeffs.
  * @param[in] transform The transformation between the camera and the laser
  */
 void projectPoint(geometry_msgs::Point32 &pointIn, cv::Point2d &pointOut, sensor_msgs::CameraInfo &cam_info,
                   tf::StampedTransform &transform);

 void pointToPlane(geometry_msgs::Point32 &ptIn, cv::Point &ptOut, int &zoom);

 /**
  * Draw the laser scan in a 2d plane
  * It's a callback function for the zoom trackbar handler that's why it's a bit fuzzy.
  * The obj_class class must be a visualizer or a derived class.
  * @param zoom The zoom factor
  * @param obj The object where to get the ClusteredScan data
  */
 template <class obj_class> void plotLaser(int zoom, void * obj)
 {
   // dereferencing the void object pointer
   obj_class * newObj = (obj_class*)obj; //recasted

   hdetect::ClusteredScan scanClusters = newObj->getClusteredScan();

   Mat laserPlane;
   laserPlane.create(HEIGHT, WIDTH, CV_8UC3);
   laserPlane.setTo(Scalar(255, 255, 255));

   Point pt;
   Scalar color;
   // The mat will be 1000(X)x500(Y) pixels
   // The values are computed according to the zoom value
   // The default zoom corresponds to 500 pixels = 30000 mm

   for (uint cNo = 0; cNo < scanClusters.nclusters; cNo++)
   {

     if (1) //scanClusters.fusion[clusterNo] == TRUE)
     {
       pointToPlane(scanClusters.cogs[cNo],pt,zoom);

       color = newObj->getColor(scanClusters.cogs[cNo]);

       // Number of cluster
       //cv::putText(laserPlane, boost::lexical_cast<string>(clusterNo),
       //            pt, 1, 2, color,2, 8);
       //pt.x+=30;

       // Number of points
       Scalar black(0,0,0);
       cv::putText(laserPlane, boost::lexical_cast<string>(scanClusters.clusters[cNo].points.size()),
                   pt, 1, 2, black, 2, 8);

       // Distance to cog
       pt.x+=45;
       cv::putText(laserPlane, boost::lexical_cast<string>( uint( sqrt( pow(scanClusters.cogs[cNo].x,2)
                                                                  + pow(scanClusters.cogs[cNo].y,2) ) *100) ),
                   pt, 1, 1.3, black, 1.3, 8);

       for (uint pointNo = 0; pointNo < scanClusters.clusters[cNo].points.size(); pointNo++)
       {
         // Transform x,y point to pixels
         pointToPlane(scanClusters.clusters[cNo].points[pointNo],pt,zoom);
         circle(laserPlane, pt, 2, color);
       }

       if(scanClusters.labels[cNo]==1)
       {
			//cv::circle(laserPlane, 1 , 2, Scalar(0,0,0));
       }


     }
   }

   color.val[0]=0;
   color.val[1]=255;
   color.val[2]=0;

   geometry_msgs::Point32 org;
   org.x=0;
   org.y=0;

   pointToPlane(org, pt, zoom);
   cv::putText(laserPlane, "Laser", pt, 1, 1, color, 1, 1);
   circle(laserPlane, pt, 2, color);

   newObj->setLaserPlane(laserPlane);
 }
#endif
