#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/bind.hpp>

#include <hdetect/lib/detector.hpp>

/**
 * A node to set up all things needed for visualization.
 * Also used for the annotation.
 * The name file is given from command line.
 * @author Stathis Fotiadis
 * @date 10/10/2012
 */
class headlessRT
{
    private:
        ros::NodeHandle nh;

        /// Subsciber to the camera image topic
        message_filters::Subscriber<sensor_msgs::Image> cameraImage_sub_;
        /// Subsciber to the laser scan topic
        message_filters::Subscriber<sensor_msgs::LaserScan>  laserScan_sub_;

        /**
          * An approximate time policy to synchronize image, camera info and laser messages.
          * This sync policy waits for all the three messages to arrive before it invokes the callback
          * from the annotator object.
          */
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> MySyncPolicy;

        /** The synchronizer based on the three messages policy */
        message_filters::Synchronizer<MySyncPolicy> *sync;

        // Declare the topics
        std::string cameraImageIn;
        std::string laserScanIn;

        /// The visualizer object that will be used for the callback
        detector myDetector;

    public:
        /**
          * Creates a synchronizer of two topics (image, laser).
          * It displays the laser data on to the image and the laser data alone in 2d.
          * @param[in] laserTopic The name of the laser topic that will be subscribed to.
          * @param[in] cameraTopic The name of the camera topic that will be subscribed to.
          */
        headlessRT(std::string cameraTopic, std::string laserTopic);

        ~headlessRT();
};

headlessRT::headlessRT(std::string cameraTopic, std::string laserTopic) : nh("~")
{
    // Subscibe to the corresponding topics
    cameraImage_sub_.subscribe(nh,cameraTopic,3);
    laserScan_sub_.subscribe(nh,laserTopic,3);

    // Initialize synchronizer
    // Future work change it to a tf::MessageFilter to include the tf transform
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), cameraImage_sub_, laserScan_sub_);

    sync->registerCallback(boost::bind(&detector::detectHumans, boost::ref(myDetector), _1, _2 ));

    // Sleep to give time to other nodes (tf) to start
    sleep(2);
    ROS_INFO("[HEADLESS_RT] headlessRT running OK.");
}

headlessRT::~headlessRT()
{

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "headlessRT");

    std::string cameraTopic(argv[1]);
    std::string laserTopic(argv[2]);

    headlessRT hdless(cameraTopic, laserTopic);
    ros::spin();

    return 0;
}
