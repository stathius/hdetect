/*
 * test.cpp
 * test to see how the calibration works
 *  Created on: Nov 16, 2012
 *      Author: kabamaru
 *
 */
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <camera_calibration_parsers/parse_yml.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

   std::string frame_l;
   std::string frame_cam;
   std::string fname;

   tf::StampedTransform transform;

   geometry_msgs::PointStamped lSPoint;
   geometry_msgs::PointStamped lSPointOut;

   geometry_msgs::Point32 point;

   cv::Point2d pt2D;
   cv::Point3d pt3D;

   image_geometry::PinholeCameraModel cam_model_;

   sensor_msgs::CameraInfo cam_info;

void test (void) {

    ros::NodeHandle node;

    std::cout << "Give x, y ,z\n";
    std::cin >> point.x;
    std::cin >> point.y;
    //std::cin >> point.z;
    point.z = 0;

    tf::TransformListener tf_listener_;

    try {
        ros::Duration timeout(3);
        tf_listener_.waitForTransform(frame_cam, frame_l, ros::Time(0), timeout);
        tf_listener_.lookupTransform(frame_cam, frame_l, ros::Time(0), transform);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[CALTEST] TF exception:\n%s", ex.what());
      }

      // METHOD 1: ROS tf::TransformPoint + pinholeCameraModel::project3dtoPixel
      //

      lSPoint.point.x=point.x; lSPoint.point.y=point.y; lSPoint.point.z=point.z;
      lSPoint.header.frame_id=frame_l;

      //tf_listener_.transformPoint(frame_cam, ros::Time(0), lSPoint, frame_l, lSPointOut);
      lSPointOut=lSPoint;

      pt3D.x = lSPointOut.point.x;
      pt3D.y = lSPointOut.point.z;
      pt3D.z = lSPointOut.point.y;

      pt2D = cam_model_.project3dToPixel(pt3D);


      cv::Mat K(3, 3, CV_64FC1);
      cv::Mat R(3, 3, CV_64FC1);
      cv::Mat P(3, 4, CV_64FC1);

      printf("CAMERA MATRIX\n");

      int i,j;
      for(i=0; i < 3 ; i++) {
        for(j=0; j < 3; j++) {
          K.at<double>(i,j)=cam_info.K[3*i+j];
          R.at<double>(i,j)=cam_info.R[i*3+j];
          printf("%f\t",cam_info.K[3*i+j]);
        }
        printf("\n");
      }
      printf("\n");

      printf("PROJECTION MATRIX\n");

      for(i=0; i < 3 ; i++) {
        for(j=0; j < 4; j++) {
          P.at<double>(i,j)=cam_info.P[4*i+j];
          printf("%f\t",cam_info.P[4*i+j]);
        }
        printf("\n");
      }
      printf("\n");

      // METHOD 2: calib3d::projectPoints

      cv::Mat pIn(1, 3, CV_64FC1);
      cv::Mat pOut(1, 3, CV_64FC1);

      cv::Mat cvPhi = cv::Mat::zeros(3, 3, CV_64FC1);
      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

      tf::Vector3 t=transform.getOrigin();
      tvec.at<double>(0,0)= t.x(); tvec.at<double>(0,1)= t.y(); tvec.at<double>(0,2)= t.z();

      // rotation vector NOT RPY
      rvec.at<double>(0,0)= 0.0174833625050782;
      rvec.at<double>(0,1)= -0.119012558661694;
      rvec.at<double>(0,2)= 3.13528527743474;

      tf::Quaternion q;
      q = transform.getRotation();
      double roll, pitch, yaw;
      tf::Matrix3x3  phi;//(q);
      //phi.setValue( -0.999929898220757 ,     -0.00442044279230751  ,      0.0109844585550146,
      //              0.00357500223864038 ,       -0.997114460355643  ,     -0.0758285718490689,
      //               0.0112879583282635  ,     -0.0757839866673404   ,       0.99706036395074);
      //phi.setRPY(-0.075862, -0.011288, 3.138017);
      phi.setRotation(q);
      phi.getRPY(roll, pitch, yaw);
      //q.setRPY(roll, pitch, yaw);

      //phi.setEulerYPR(roll, pitch, yaw);
      // YPR 3.138017 -0.011288 -0.075862


      printf("PHI\n");
      for (i = 0; i < 3; i++) {
        for (j= 0; j < 3; j++) {
          cvPhi.at<double>(i,j) = phi.getRow(i)[j];
          printf("%f\t", cvPhi.at<double>(i,j));
        }
        printf("\n");
      }

      printf("\nRPY Q     = (%f, %f, %f, %f)\n", q.x(), q.y(), q.z(), q.w());
      phi.getRotation(q);
      printf("Rotation Q = (%f, %f, %f, %f)\n\n", q.x(), q.y(), q.z(), q.w());

      printf("Rotation RPY  = (%f, %f, %f)\n\n", roll, pitch, yaw);

      printf("Translation = (%f, %f, %f)\n\n", t.x(), t.y(), t.z());


      pIn.at<double>(0,0) = pt3D.x; pIn.at<double>(0,1) = pt3D.y; pIn.at<double>(0,2) = pt3D.z;

      // Print results

      printf("Original Point:  x %f y %f z %f\n", point.x, point.y, point.z);

      printf("Projected point: x %f y %f z %f\n\n",pIn.at<double>(0,0), pIn.at<double>(0,1), pIn.at<double>(0,2));

      //printf("Method 1: x %f y %f\n\n",pt2D.x,pt2D.y);

      cv::projectPoints(pIn, rvec, tvec, K, cam_info.D, pOut);
      printf("Hardcoded rotation\n");
      printf("Rotation vector = %f %f %f\n",rvec.at<double>(0,0),rvec.at<double>(0,1),rvec.at<double>(0,2));
      printf("Point = x %f y %f\n\n",pOut.at<double>(0,0),pOut.at<double>(0,1));

      cv::Rodrigues(cvPhi, rvec);
      cv::projectPoints(pIn, rvec, tvec, K, cam_info.D, pOut);
      printf("Computed rotation\n");
      printf("Rotation  vector = %f %f %f\n",rvec.at<double>(0,0),rvec.at<double>(0,1),rvec.at<double>(0,2));
      printf("Point = x %f y %f\n\n",pOut.at<double>(0,0),pOut.at<double>(0,1));

      cv::Mat pm = P(cv::Rect(0,0,3,3));
      cv::projectPoints(pIn, rvec, tvec, pm, cam_info.D, pOut);
      //printf("Method 2. Projection Matrix\n");
      //printf("x %f y %f\n\n",pOut.at<double>(0,0),pOut.at<double>(0,1));



}


int main(int argc, char* argv[])
{
  fname="00b09d0100aa73bb";
  frame_l="/laser";
  frame_cam="/camera";

  ros::init(argc, argv, "testcalib");
  camera_calibration_parsers::readCalibrationYml("/home/kabamaru/.ros/camera_info/matlab_calib.yaml", fname, cam_info);
  cam_model_.fromCameraInfo(cam_info);
  ROS_INFO("Start OK");
  test();
  //ros::spin();
  return 0;
}
