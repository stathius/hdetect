/* AnnotateData
 Reads the camera topic (camera/image_raw) and the laser topic from a bag
 Publishes ...
 */
#include "upm/lib/bagReader.hpp"
#include "upm/lib/annotator.hpp"

/**
 * A node to set up all things needed for the annotation.
 * The name file is given from command line.
 * @author Stathis Fotiadis
 * @date 10/10/2012
 */
class AnnotateData
{
private:

  /**
   * An approximate time policy to synchronize image, camera info and laser messages.
   * This sync policy waits for all the three messages to arrive before it invokes the callback
   * from the annotator object.
   */
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
      sensor_msgs::LaserScan> MySyncPolicy;
  /** The synchronizer based on the three messages policy */
  message_filters::Synchronizer<MySyncPolicy> *sync;

  /** Fake subscriber to the camera msgs */
  bagSubscriber<sensor_msgs::Image> cameraImage_sub_;
  /** Fake subscriber to the laser scan msgs */
  bagSubscriber<sensor_msgs::LaserScan> laserScan_sub_;

  /** The annotator class used to do the annotation of the data */
  annotator dataAnnotator;

  bagReader bagger;

// Declare the topics
  std::string cameraImageIn;
  std::string cameraInfoIn;
  std::string laserScanIn;
  /** The container of the topics that will be read from the bag file. They are hardcoded. */
  std::vector<std::string> topics;

public:
  /**
   * Sets up everything to read and annotate the given bag.
   * First it creates some fake subscribers for each one of the camera, camera info and laser topics.
   * The it sets up the approximate time policy synchronizer.
   * Then it uses bagger, a bagReader class object, to open and feed the subscribers with messages from the topics.
   * When all three messages arrive to the corresponding subscribers then the synchronizer calls
   * the annotateData() function of the annotator object. It goes on with doing the annotation
   * @param[in] bagFile The name of the bag file to be annotated
   */
  AnnotateData(std::string bagFile);
  ~AnnotateData()
  {
  }

};

AnnotateData::AnnotateData(std::string bagFile) :
    dataAnnotator(bagFile), bagger(bagFile)
{
  // define the topics
  cameraImageIn = "/camera/image_raw";
  laserScanIn = "/laser/scan";

  topics.push_back(cameraImageIn);
  topics.push_back(laserScanIn);

  // Initialize synchronizer
  sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100), cameraImage_sub_,
                                                         laserScan_sub_);

  sync->registerCallback(boost::bind(&annotator::annotateData, boost::ref(dataAnnotator), _1, _2 ));

  ROS_INFO("[ANNOTATE_LASER] Obj creation OK");

  // Sleep to give time to other nodes (tf) to start
  sleep(2);
  ROS_INFO("[ANNOTATE_LASER] Opening bag file %s", bagFile.c_str());
  bagger.loadBag(topics, cameraImage_sub_, laserScan_sub_);
  ROS_INFO("[ANNOTATE_LASER] Finished OK");
}

int main(int argc, char* argv[])
{
  //if (argc==1) ROS_ERROR("Give the bag filename.");

  std::string bagFile(argv[1]);

  ros::init(argc, argv, "AnnotateLaser");

  AnnotateData al(bagFile);
  return 0;
}
