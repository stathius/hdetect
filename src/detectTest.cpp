/* detectBAG
 Reads the camera topic, the laser topic, and the ClusteredScan from a bag
 Then it does the detection.
 ITS
 */
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <rosgraph_msgs/Clock.h>

#include <hdetect/lib/visualizer.hpp>
#include <hdetect/ClusteredScan.h>


/** TODO A LOT
 * Makes the detection test from a bag file.
 * The name file is given from command line.
 * @author Stathis Fotiadis
 * @date 10/10/2012
 */
class detectTest : public visualizer
{
private:

	/**
	 * An approximate time policy to synchronize image, camera info and laser messages.
	 * This sync policy waits for all the three messages to arrive before it invokes the callback
	 * from the annotator object.
	 */
	//  visualizer myVzr;

	// Declare the topics
	std::string clusteredTopic;

	/** The container of the topics that will be read from the bag file. They are hardcoded. */
	std::vector<std::string> topics;

	//  The clock publisher
	ros::Publisher clock_pub_;
	rosgraph_msgs::Clock clockmsg;

public:
	/**
	 * Reads from an already annotated bag, what it needs.
	 * @param[in] bagFile The name of the bag file to be annotated
	 */
	detectTest(std::string bagFile);
	~detectTest() { };

};

detectTest::detectTest(std::string bagFile)
{
	// define the topics
	clusteredTopic = "/laser/ClusteredScan";

	topics.push_back(clusteredTopic);


	ROS_INFO("[DETECT_BAG] Obj creation OK");

	// Sleep to give time to other nodes (tf) to start
	sleep(2);
	ROS_INFO("[DETECT_BAG] Opening bag file %s", bagFile.c_str());

	rosbag::Bag bag(bagFile, rosbag::bagmode::Read);
	rosbag::View view(bag, rosbag::TopicQuery(topics));

	int count = 0;
	// Reads the topics form the bag file
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		if (m.getTopic() == clusteredTopic)
		{
			hdetect::ClusteredScan::ConstPtr mes = m.instantiate<hdetect::ClusteredScan>();
			if (mes != NULL)
			{
				// Publish the clock
				clockmsg.clock = m.getTime();
				clock_pub_.publish(clockmsg);

				sensor_msgs::Image::Ptr image_ptr = boost::make_shared<sensor_msgs::Image>(mes->image);
				sensor_msgs::LaserScan::ConstPtr laser_ptr = boost::make_shared<sensor_msgs::LaserScan>(mes->scan);

				visualizeData(image_ptr, laser_ptr);
			}
			else
			{
				ROS_INFO("[DETECT_TEST] Null message found");
			}
		}

		count++;
	}

	bag.close();
	//ROS_INFO("[BAG_READER] %d messages read.", count);

	ROS_INFO("[DETECT_BAG] Finished OK");
}

int main(int argc, char* argv[])
{
	//if (argc==1) ROS_ERROR("Give the bag filename.");

	std::string bagFile(argv[1]);

	ros::init(argc, argv, "detectBAG");

	detectTest db(bagFile);
	return 0;
}
