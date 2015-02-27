#ifndef RECOGNIZER_HPP
#define RECOGNIZER_HPP

// STANDARD
#include <stdio.h>
#include <string>
#include <sstream>
#include <deque>
#include <map>

// ROS
#include <image_transport/image_transport.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>

// OPENCV
#include <opencv2/highgui/highgui.hpp>

// NEWMAT
#include <newmat/newmat.h>

// MY INCLUDES
#include <hdetect/lib/detector.hpp>
#include <hdetect/lib/header.hpp>
#include <hdetect/lib/human.hpp>
#include <hdetect/lib/observation.hpp>
#include <hdetect/lib/object_tracking.hpp>
#include <hdetect/HumansFeatClass.h>

class Recognizer : public detector
{
    public:
        Recognizer();
        Recognizer(const string odomTopic, const string odomEkfTopic);
        ~Recognizer();

        void recognizeData(const sensor_msgs::Image::ConstPtr &image,
                           const sensor_msgs::LaserScan::ConstPtr &lScan);

    protected:
        image_transport::ImageTransport it_;
        image_transport::Publisher it_pub_;

        ros::Subscriber odom_sub_;
        ros::Subscriber odom_ekf_sub_;

        ros::Publisher rviz_pub_;

        cv_bridge::CvImagePtr output_cv_ptr;

        /// Raw image mat.
        cv::Mat rawImage;

        std::deque<Human> humans;
        std::deque<Observation> observations;
        std::map<int, int> pairs;

        std::deque<cv::Scalar> colors;

        // Odometry
        tf::Transform cur_odom;
        tf::Transform pre_odom;

        // Odometry Ekf
        tf::Transform cur_odom_ekf;
        tf::Transform pre_odom_ekf;

    private:

        enum Rviz
        {
            RVIZ_NONE = 0,
            RVIZ_HUMAN,
            RVIZ_ID,
            RVIZ_LINE,
            RVIZ_SCAN,
            RVIZ_TOTAL
        };

        bool with_odom;

        bool with_odom_ekf;

        int rviz_id[RVIZ_TOTAL];

        void initColor();

        void loadObservation();

        void publish(const sensor_msgs::LaserScan::ConstPtr &lScan);

        void setOdom(const nav_msgs::Odometry &odom);

        void setOdomEkf(const geometry_msgs::PoseWithCovarianceStamped &odom);

        void correctOdom(NEWMAT::ColumnVector &state);

        void correctOdomEkf(NEWMAT::ColumnVector &state);

        void setPoint(float x, float y, float z, geometry_msgs::Point &p);

        cv::Scalar getColor(int index);

        std_msgs::ColorRGBA getColorRgba(int index, float a);

        float getTimestamp();

        std::string toString(float num);

        void resetId();

        uint getId(int index, int category);

        uint getNewId();

        uint getNewId(int category);

        // Publisher variables
        ros::Publisher Humanpublisher;
        hdetect::HumansFeat HumansAux;
        std::vector<hdetect::HumansFeat> HumansVector;
        hdetect::HumansFeatClass HumansDetected;

};

#endif // RECOGNIZER_HPP
