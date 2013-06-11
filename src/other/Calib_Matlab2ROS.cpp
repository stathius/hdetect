// Converts the calibration from matlab to ros.
// The values are hardwired to the code.

#include <camera_calibration_parsers/parse_yml.h>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace camera_calibration_parsers;

int main(int argc,char** argv) {
	std::string name=("00b09d0100aa73bb");
	sensor_msgs::CameraInfo cam_info;
	readCalibrationYml("/home/kabamaru/.ros/camera_info/00b09d0100aa73bb.yaml", name, cam_info);

	cv::Mat ncm;     

	double matlab_K[9] =
		   {668.68462594024,	0, 			384.916944772716,
		     0, 		669.810891377069,	239.099132134278,
                     0, 		0, 			1};

	double matlab_D[5] = {   -0.413014051243856,
                                  0.182192431895544,
                                 0.0012217553319971,
                               -0.00116201628505299,
                                                  0};

	for (int i=0; i<9 ; i++) cam_info.K[i]=matlab_K[i];
	for (int i=0; i<5 ; i++) cam_info.D[i]=matlab_D[i];



        cv::Mat K(3, 3, CV_64FC1);
        cv::Mat R(3, 3, CV_64FC1);

	//float D[5]; 
	//ncm = cv.GetSubRect(self.P, (0, 0, 3, 3))
	for (int i=0;i<3 ; i++) {
		for(int j=0; j<3; j++) {
			K.at<double>(i,j)=cam_info.K[3*i+j];
			R.at<double>(i,j)=cam_info.R[i*3+j];
		}
		printf("\n");	
	}
	printf("\n");

	//for (int i=0; i<5; i++) D[i]=cam_info[

	ncm = cv::getOptimalNewCameraMatrix(K, cam_info.D, cvSize(cam_info.width,cam_info.height) , 0.0);

	// print camera matrix
	for (int i=0;i<3 ; i++) {
		for(int j=0; j<3; j++) {
			printf("%f\t",cam_info.K[3*i+j]);
		}
		printf("\n");	
	}
	printf("\n");
	
	// print the distortion coefficients
	for (int i=0; i<5 ; i++) printf("%f\t",cam_info.D[i]);
	printf("\n\n");

	// print the rectification matrix
	for (int i=0;i<3 ; i++) {
		for(int j=0; j<3; j++) {
			printf("%f\t",cam_info.R[3*i+j]);
		}
		printf("\n");	
	}
	printf("\n");
	
	// print the projection matrix
	for (int i=0;i<3 ; i++) {
		for(int j=0; j<3; j++) {
		        cam_info.P[3*i+j]=ncm.at<double>(i,j);
			printf("%f\t",cam_info.P[3*i+j]);
		}
		printf("\n");	
	}

	// Important change height and width to the original
	cam_info.width=752;
	cam_info.height=480;

	writeCalibrationYml("/home/kabamaru/.ros/camera_info/matlab_calib.yaml", name, cam_info);
	
}
