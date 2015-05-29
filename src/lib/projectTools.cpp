#include "hdetect/lib/projectTools.hpp"

#define HOG_WIDTH 64
#define HOG_HEIGHT 128

using namespace std;
using namespace cv;

/// Projects a point from the laser scan to a 2d top-down view plane.
void pointToPlane(geometry_msgs::Point32 &ptIn, Point &ptOut, Size &windowSize, int &zoom)
{
    int width = windowSize.width;
    int height = windowSize.height;

    ptOut.x = width - 1 - (int)((ptIn.y * width / (float)zoom) + width / 2.0);
    ptOut.y = height - 1 - (int)((ptIn.x * height / (float)zoom) + height / (float)zoom);
}

/// Function to return a selection of random generated colors
vector<Scalar> randomColors(cv::RNG& rng)
{
    vector<Scalar> colors;

    for (int i = 0; i < 250; i++)
	{
		int icolor = (unsigned)rng;
		colors.push_back(cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255));
	}

	return colors;
}

/// Returns the corresponding crop of an image
void getCrop(cv::Mat &roi, cv::Mat &image, Rect &rect)
{
    roi = image(rect);
}

/// Returns true if the box lies within the image
bool checkBox(sensor_msgs::CameraInfo &cInfo, Rect &rect)
{
    bool ret = (rect.x >= 0 && rect.x + rect.width < (int)cInfo.width) && // width inbounds
               (rect.y >= 0 && rect.y + rect.height < (int)cInfo.height) && // height inbounds
               (rect.width >= HOG_WIDTH && rect.height >= HOG_HEIGHT); // boxsize check

    return ret;
}

/// Finds the right bounding box size according to the distance
///     1.0xL
/// _______________
/// |<-upleft     |
/// |             |
/// |             |
/// |             |
/// |             |  2.0xL
/// |             |
/// |             |
/// |             |
/// |_downright->_|
///
void getBox(geometry_msgs::Point32 &it, Point2d &pt2D, Rect &rect, double &M_TO_PIXELS, double &BODY_RATIO)
{
	//boxWidth = (int)(M_TO_PIXELS / sqrt(pow(double(it.x)*M_TO_PIXELS, 2.0) + pow(double(it.y)*M_TO_PIXELS, 2.0)));
	double dist = sqrt(pow(it.x, 2.0) + pow(it.y, 2.0));
    float boxWidth = M_TO_PIXELS / dist;

    // Compute the rectangle
    rect.x = (int)(pt2D.x - (boxWidth / 2));
    rect.y = (int)(pt2D.y - BODY_RATIO * boxWidth);
    rect.width = (int)(1.0 * boxWidth);
    rect.height = (int)(2.0 * boxWidth);
}

void CameraInfo2CV(sensor_msgs::CameraInfo &cInfo, Mat &K, Mat &D, int &rect)
{
	// K Camera Matrix for Distorted images
    K = Mat::zeros(3, 3, CV_64FC1);

	int i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			K.at<double>(i, j) = cInfo.K[3 * i + j];
		}
	}

    D = Mat::zeros(1,5,CV_64FC1);

    if (rect == 0)
    {
        for (i = 0; i < 5; i++)
        {
			D.at<double>(i) = cInfo.D[i];
		}
	}


	/* Print the camera calibration parameters.
	printf("\nCamera Matrix K\n");
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			printf("%f\t",cInfo.K[3*i+j]);
		}
		printf("\n");
	}

	printf("\nRectification Matrix R\n");
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			printf("%f\t",cInfo.R[3*i+j]);
		}
		printf("\n");
	}
	printf("\nProjection Matrix P\n");
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 4; j++)
		{
			printf("%f\t",cInfo.P[4*i+j]);
		}
		printf("\n");
	}


	printf("\nDistortion coefficients D\n");
	if (rect == 0) {
		for (i = 0; i < 5; i++) {
			printf("%f ",D.at<double>(i));
		}
	}
	*/
}


void projectPoint(geometry_msgs::Point32 &pointIn, Point2d &pointOut, Mat &K, Mat &D,
                  tf::StampedTransform &transform)
{
	/*
	 * METHOD: calib3d::projectPoints
	 *
	 */

    Mat pIn(1, 3, CV_64FC1);
    Mat pOut(1, 3, CV_64FC1);

    Mat cvPhi = Mat::zeros(3, 3, CV_64FC1);
    Mat rvec = Mat::zeros(3, 1, CV_64FC1);
    Mat tvec = Mat::zeros(3, 1, CV_64FC1);

	tf::Vector3 t = transform.getOrigin();
	tvec.at<double>(0, 0) = t.x();
	tvec.at<double>(0, 1) = t.y();
	tvec.at<double>(0, 2) = t.z();
	//tf::Quaternion q;
	//q = transform.getRotation();
	tf::Matrix3x3 phi(transform.getRotation());

	// ***********************************
    // With matlab camera calib.
	// NOTICE WE EXCHANGE X --> Z, y --> X
    // ***********************************
	// ***********************************
    // pIn.at<double>(0, 0) = pointIn.y;
    // pIn.at<double>(0, 1) = pointIn.z;
    // pIn.at<double>(0, 2) = pointIn.x;
    // -----------------------------------
    // ***********************************
    // ***********************************
    // With ROS camera tf calibration
    // NOTICE WE EXCHANGE z --> x, y --> z  x --> y
    // ***********************************
    // ***********************************
    pIn.at<double>(0, 0) = pointIn.z;
    pIn.at<double>(0, 1) = pointIn.x;
    pIn.at<double>(0, 2) = pointIn.y;


	/*
   //phi.setValue( -0.999929898220757 ,     -0.00442044279230751  ,      0.0109844585550146,
   //              0.00357500223864038 ,       -0.997114460355643  ,     -0.0758285718490689,
   //               0.0112879583282635  ,     -0.0757839866673404   ,       0.99706036395074);
   //phi.setRPY(-0.075862, -0.011288, 3.138017);
   phi.setRotation(q);
   //q.setRPY(roll, pitch, yaw);
	 *

   //phi.setEulerYPR(roll, pitch, yaw);
   // YPR 3.138017 -0.011288 -0.075862
	 */
	double roll, pitch, yaw;
	phi.getRPY(roll, pitch, yaw);

	//printf("\nPHI\n");
	for (uint i = 0; i < 3; i++)
	{
		for (uint j = 0; j < 3; j++)
		{
			cvPhi.at<double>(i, j) = phi.getRow(i)[j];
			//printf("%f\t", cvPhi.at<double>(i,j));
		}
		//printf("\n");
	}

	/*
   //printf("\nRPY Q     = (%f, %f, %f, %f)\n", q.x(), q.y(), q.z(), q.w());
   //phi.getRotation(q);
   printf("Rotation Q = (%f, %f, %f, %f)\n\n", q.x(), q.y(), q.z(), q.w());

   printf("Rotation RPY  = (%f, %f, %f)\n\n", roll, pitch, yaw);

   printf("Translation = (%f, %f, %f)\n\n", t.x(), t.y(), t.z());


   // Print results
   printf("Original Point:  x %f y %f z %f\n", pt.x, pt.y, pt.z);
   printf("Projected point: x %f y %f z %f\n\n",pIn.at<double>(0,0), pIn.at<double>(0,1), pIn.at<double>(0,2));

     */
    Rodrigues(cvPhi, rvec);

	//printf("\nRvec %f %f %f",rvec.at<double>(0,0),rvec.at<double>(0,1),rvec.at<double>(0,2));
	//printf("\nTvec %f %f %f",tvec.at<double>(0,0),tvec.at<double>(0,1),tvec.at<double>(0,2));

	/*
	int i, j;
	printf("\nCamera Matrix K\n");
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			//K.at<double>(i, j) = cInfo.K[3 * i + j];
			printf("%f\t",K.at<double>(i,j));
		}
		printf("\n");
	}

	printf("\nD (kc) ");
	for (int i = 0; i < 5; i++) {
		//D.at<double>(0,i) = 0;
		printf("%f ",D.at<double>(0,i));
	}
	 */

	//pIn.at<double>(0,0)=5;
	//pIn.at<double>(0,1)=0;
	//pIn.at<double>(0,2)=10;
	//printf("\n\nPoint in  %f %f %f\n",pIn.at<double>(0,0),pIn.at<double>(0,1),pIn.at<double>(0,2));
	cv::projectPoints(pIn, rvec, tvec, K, D, pOut);
	//printf("After projectPoints");
	//printf("\nPoint out %f %f %f\n",pOut.at<double>(0,0),pOut.at<double>(0,1),pOut.at<double>(0,2));

	//      printf("Computed rotation\n");
	//      printf("Rotation  vector = %f %f %f\n",rvec.at<double>(0,0),rvec.at<double>(0,1),rvec.at<double>(0,2));
	//     printf("Point = x %f y %f\n\n",pOut.at<double>(0,0),pOut.at<double>(0,1));

	pointOut.x = pOut.at<double>(0, 0);
	pointOut.y = pOut.at<double>(0, 1);

	/*
   // rotation vector NOT RPY
   rvec.at<double>(0,0)= 0.0174833625050782;
   rvec.at<double>(0,1)= -0.119012558661694;
   rvec.at<double>(0,2)= 3.13528527743474;

   cv::projectPoints(pIn, rvec, tvec, K, cam_info->D, pOut);
   printf("Hardcoded rotation\n");
   printf("Rotation vector = %f %f %f\n",rvec.at<double>(0,0),rvec.at<double>(0,1),rvec.at<double>(0,2));
   printf("Point = x %f y %f\n\n",pOut.at<double>(0,0),pOut.at<double>(0,1));

   waitKey();
	 */
}
