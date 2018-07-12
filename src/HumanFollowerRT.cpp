#include <hdetect/lib/human_follower.hpp>
#include <hdetect/recognizeRT.hpp>

/**
 * A node to execute following process.
 * @author Andrés Alacid Cano
 * @date 2014/02/16
 */
 
 class HumanFollowerRT
{
    public:
        HumanFollowerRT(string cameraTopic, string laserTopic);

        ~HumanFollowerRT();

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

        HumanFollower myHumanFollower;
};

HumanFollowerRT::HumanFollowerRT(std::string cameraTopic, std::string laserTopic)
    : nh("~")
{
    // Subscibe to the corresponding topics
    cameraImage_sub_.subscribe(nh,cameraTopic,3);
    laserScan_sub_.subscribe(nh,laserTopic,3);

    // Initialize synchronizer
    // Future work change it to a tf::MessageFilter to include the tf transform
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), cameraImage_sub_, laserScan_sub_);

    sync->registerCallback(boost::bind(&HumanFollower::update, boost::ref(myHumanFollower), _1, _2));

    // Sleep to give time to other nodes (tf) to start
    sleep(2);
    ROS_INFO("[HUMANFOLLOWER_RT] HumanFollowerRT running OK.");
}

HumanFollowerRT::~HumanFollowerRT()
{
	
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "HumanFollowerRT");

    std::string cameraTopic(argv[1]);
    std::string laserTopic(argv[2]);

    HumanFollowerRT vl(cameraTopic, laserTopic);
    ros::spin();

    return 0;
}
