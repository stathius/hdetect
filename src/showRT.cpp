#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <image_transport/image_transport.h>

#include <boost/bind.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <hdetect/lib/header.hpp>
#include <hdetect/ClusterClass.h>

/**
 * A node to set up all things needed for recognition.
 * Also used for the annotation.
 * The name file is given from command line.
 * @author Bang-Cheng Wang
 * @date 2013/10/01
 */

static const char WINDOW[] = "Result";

class showRT
{
    public:
        showRT();
        ~showRT();

        void showImage(const sensor_msgs::ImageConstPtr& msg);

    private:

        ros::NodeHandle nh;

        /// Subsciber to the image topic

        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
};

showRT::showRT() : nh("~"), it_(nh)
{
    // Subscibe to the corresponding topics
    image_sub_ = it_.subscribe(Header::imageTopic, 3, &showRT::showImage, this);

    cv::namedWindow(WINDOW);

    // Sleep to give time to other nodes (tf) to start
    sleep(2);
    ROS_INFO("[SHOW_RT] ShowRT running OK.");
}

showRT::~showRT()
{
    cv::destroyWindow(WINDOW);
}

void showRT::showImage(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "showRT");

    showRT vl();
    ros::spin();

    return 0;
}

