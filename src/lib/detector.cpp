#include <hdetect/lib/detector.hpp>

//using namespace std;
using std::string;
using std::vector;
//using namespace cv;
using cv::Size;
using cv::Mat;
using cv::Rect;
using cv::HOGDescriptor;
using cv::Range;

//using namespace sensor_msgs;
using sensor_msgs::Image;
using sensor_msgs::LaserScan;
using namespace Header;

detector::detector() : nh("~")
{
  //string s = nh.getNamespace();
  //ROS_INFO("[detector::detector] my namespace %s ", s.c_str());

  if (nh.hasParam("camera_yaml") && nh.hasParam("camera_name") && nh.hasParam("boost_xml"))
  {
    string camera_yaml;
    string cname;
    string boost_xml;
    nh.getParam("camera_yaml", camera_yaml);
    nh.getParam("camera_name", cname);
    nh.getParam("boost_xml", boost_xml);

    if(!camera_calibration_parsers::readCalibrationYml(camera_yaml, cname, params.cInfo))
    {
      ROS_ERROR("[detector::detector] Failure reading camera calibration parameters.");
      return;
    }


    ROS_INFO("[detector::detector] Create Classifier");
    // create the boost classifier
    //boost = cv::ml::Boost::create();

    ROS_INFO("[detector::detector] Load Boost Classifier parameters %s", boost_xml.c_str());
    // Load the boost classifier
    boost = cv::Algorithm::load<cv::ml::Boost>(boost_xml.c_str());




    ROS_INFO("[detector::detector] Camera calibration & Boost Classifier Loaded");
    ROS_INFO("[detector::detector] Boost type: %d", boost->getBoostType());
    ROS_INFO("[detector::detector] Weak count: %d", boost->getWeakCount());
    ROS_INFO("[detector::detector] CV Folds: %d", boost->getCVFolds());
    ROS_INFO("[detector::detector] Splits: %d", (int)boost->getSplits().size());
    ROS_INFO("[detector::detector] getWeightTrimRate: %.2f", boost->getWeightTrimRate());

    //boost->save(boost_xml_2.c_str());

  }
  else
  {
    ROS_ERROR("[detector::detector] Need to set the parameters in order to load the camera calibration and the boost classifier.");
    return;
  }

  if (nh.hasParam("laser_window_width") && nh.hasParam("laser_window_height") &&
      nh.hasParam("rect") && nh.hasParam("hog_hit_threshold") && nh.hasParam("hog_group_threshold") &&
      nh.hasParam("hog_meanshift") && nh.hasParam("tf_timeout") &&
      //      nh.hasParam("cameraA") && nh.hasParam("cameraB") &&
      nh.hasParam("laserA") && nh.hasParam("laserB") &&
      nh.hasParam("m_to_pixels") && nh.hasParam("body_ratio") && nh.hasParam("jumpdist") &&
      nh.hasParam("feature_set") && nh.hasParam("laser_range") && nh.hasParam("fusion_prob") &&
      nh.hasParam("min_camera_prob") && nh.hasParam("min_laser_prob"))
  {
    nh.getParam("laser_window_width", params.laser_window_width);
    nh.getParam("laser_window_height", params.laser_window_height);
    nh.getParam("rect", params.rect);

    nh.getParam("hog_hit_threshold", params.hog_hit_threshold);
    nh.getParam("hog_group_threshold", params.hog_group_threshold);
    nh.getParam("hog_meanshift", params.hog_meanshift);
    nh.getParam("tf_timeout", params.tf_timeout);
    //		nh.getParam("cameraA", params.cameraA);
    //		nh.getParam("cameraB", params.cameraB);
    nh.getParam("laserA", params.laserA);
    nh.getParam("laserB", params.laserB);
    nh.getParam("m_to_pixels", params.m_to_pixels);
    nh.getParam("body_ratio", params.body_ratio);
    nh.getParam("jumpdist", params.jumpdist);
    nh.getParam("feature_set", params.feature_set);
    nh.getParam("laser_range", params.laser_range);
    nh.getParam("fusion_prob", params.fusion_prob);
    nh.getParam("min_camera_prob", params.min_camera_prob);
    nh.getParam("min_laser_prob", params.min_laser_prob);
    ROS_INFO("[DETECTOR] Parameters loaded.");
  }
  else
  {
    ROS_ERROR("[DETECTOR] Wrong parameters loaded.");
  }

  // Initiates the laserLib with the parameters read from the server
  laserProcessor = new laserLib(params.jumpdist, params.feature_set, params.laser_range);

  window_size = Size(params.laser_window_width, params.laser_window_height);

  // Set the default detector
  hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
  switch(params.feature_set)
  {
    case 0:
      params.no_features = FEATURE_SET_0;
      break;

    case 1:
      params.no_features = FEATURE_SET_1;
      break;

    default:
      params.no_features = FEATURE_SET_1;
      break;
  }

	// Convert camera info (cInfo) to actual needed camera matrix K and distortion coefficient D
	// ready to be used by opencv functions
	CameraInfo2CV(params.cInfo, K, D, params.rect);

  // Initializing the detector publisher
  detectionPublisher = nh.advertise<hdetect::ClusterClass>("humanDetections",1);
  ROS_INFO("[DETECTOR] Detector running OK with %d features.", params.no_features);
}

detector::~detector()
{
    delete laserProcessor;
}

/// TODO NOT SURE IF NEEDED
/*
void detector::setClusters(hdetect::ClusteredScan sd)
{
	clusterData=sd;
}

hdetect::ClusteredScan detector::getClusters()
{
	return clusterData;
}
 */

void detector::getTF(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::LaserScan::ConstPtr &lScan)
{
  // Read the transform between the laser and the camera
  // Essential for syncronising
  try
  {
    //        acquisition_time = image->header.stamp; /// Maybe need to change to ros::Time::now()
    acquisition_time = ros::Time::now();
    ros::Duration timeout(1.0 / params.tf_timeout);

    tf_listener_.waitForTransform(lScan->header.frame_id, image->header.frame_id, acquisition_time, timeout);
    tf_listener_.lookupTransform(lScan->header.frame_id, image->header.frame_id, acquisition_time, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[DETECTOR] TF exception:\n%s", ex.what());
  }
}

void detector::processLaser(const sensor_msgs::LaserScan::ConstPtr &lScan, vector<hdetect::ClusteredScan> &clusterData)
{
	// Load the scan to the processor
    laserProcessor->loadScan(*lScan);

	// Segment the laser scan
    laserProcessor->getClusters(clusterData);

    // Only process scans with valid data
    if (clusterData.size() > 0)
    {
        laserProcessor->getFeatures(clusterData);
	}
	else
    {
        ROS_WARN("[DETECTOR] No valid clusters");
    }

    initClusterData(clusterData);
    findProjectedClusters(clusterData);
}

void detector::getImage(const sensor_msgs::Image::ConstPtr &image)
{
	// Copy img to openCV format
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("[DETECTOR] cv_bridge exception: %s", e.what());
		return;
	}
}

void detector::initClusterData(vector<hdetect::ClusteredScan> &clusterData)
{
    for (uint i = 0; i < clusterData.size(); i++)
    {
        clusterData[i].cog_projected = false;
        clusterData[i].crop_projected = false;

        clusterData[i].detection_label = NOT_HUMAN;
        clusterData[i].detection_laser_prob = 0.0;
        clusterData[i].detection_camera_prob = 0.0;
        clusterData[i].detection_fusion_prob = 0.0;
        clusterData[i].label = NOT_HUMAN;
    }
}


void detector::findProjectedClusters(vector<hdetect::ClusteredScan> &clusterData)
{
  // Iterate through every cog of the scanClusters
  for (uint i = 0; i < clusterData.size(); i++)
  {
    // Convert the cog to image coordinates
    projectPoint(clusterData[i].cog, prPixel, K, D, transform);

    // If the pixel is projected within the image limits
    if (prPixel.x >= 0 && prPixel.x < cv_ptr->image.cols &&
        prPixel.y >= 0 && prPixel.y < cv_ptr->image.rows)
    {
      clusterData[i].cog_projected = true;
      // Get the box size and corners
      getBox(clusterData[i].cog, prPixel, rect, params.m_to_pixels, params.body_ratio);
//      getBox(clusterData[i].cog, prPixel, rect, m_to_pixel, params.body_ratio);
      // Check if the whole box lies inside the image
      if (checkBox(params.cInfo, rect))
      {
        // Flag the cluster as 'fusable'. Meaning they appear with a valid box on the image.
        clusterData[i].crop_projected = true;
        //ROS_INFO("Cluster %d - Ok to project", i);
      }
    }
  }
}

void detector::classifyLaser(std_msgs::Float32MultiArray &features, float &probs)
{
    // The laser feature matrix
    cv::Mat lFeatures;

    lFeatures = Mat::zeros(1, params.no_features + 1, CV_32FC1);

//    fprintf(stderr, "%d\n", features.data.size());
//    fprintf(stderr, "%d %d\n", boost.get_data()->var_all, boost.get_active_vars()->cols);

    for (int i = 0; i < (int) features.data.size(); i++)
    {
        lFeatures.at<float>(0, i + 1) = features.data[i];
    }

    if ((int)features.data.size() != params.no_features)
    {
        ROS_ERROR("[DETECTOR] Wrong number of computed features.");
    }

    // Find the prediction
    //probs = boost->predict(lFeatures, Mat(), Range::all(), false, true);
    //C++: float CvBoost::predict(const cv::Mat& sample, const cv::Mat& missing=Mat(), const cv::Range& slice=Range::all(), bool rawMode=false, bool returnSum=false ) const


//    ROS_INFO("[detector::classifyLaser] Pre - prediction");
//
//    probs = boost->predict(lFeatures);
//    ROS_INFO("[detector::classifyLaser] Prediction Result: %.2f", probs);
//    probs = boost->predict(lFeatures, cv::noArray(), cv::ml::StatModel::RAW_OUTPUT);
//    ROS_INFO("[detector::classifyLaser] Prediction Result raw: %.2f", probs);
    probs = boost->predict(lFeatures, cv::noArray(), cv::ml::Boost::PREDICT_SUM);
   // ROS_INFO("[detector::classifyLaser] Prediction Result PREDICT SUM: %.2f", probs);

	// Convert prediction to probabilty
    probs = 1 / ( 1 + exp(params.laserA * probs + params.laserB) );
    // ROS_INFO("[detector::classifyLaser] Prob: %f", probs);

	//laserProb.push_back(pred);
}

void detector::classifyCamera(geometry_msgs::Point32 &cog, float &prob)
{
//  ROS_INFO("[detector::classifyCamera] - Obtain Crop");
  // Convert the cog to image coordinates
  projectPoint(cog, prPixel, K, D, transform);

  // Get the box size and corners
  getBox(cog, prPixel, rect, params.m_to_pixels, params.body_ratio);


  // Extract the crop from the image
  getCrop(crop, cv_ptr->image, rect);

  //cv::cvtColor(src_img, mono_img, CV_RGB2GRAY);
  vector<Rect> foundM;
  vector<double> weightM;

  // We don't really care about the class so we put the threshold really low to even negative predictions
  //    hog.detectMultiScale(crop, foundM, weightM);

//  double t = (double)cv::getTickCount();
//  // run the detector with default parameters. to get a higher hit-rate
//  // (and more false alarms, respectively), decrease the hitThreshold and
//  // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
//  hog.detectMultiScale(crop, foundM, weightM, 0, Size(8,8), Size(0,0), 1.1, 0, false);
//  t = (double)cv::getTickCount() - t;
//  ROS_INFO("tdetection time = %gms", t*1000./cv::getTickFrequency());
  hog.detectMultiScale(crop, foundM, weightM, params.hog_hit_threshold,
                       cv::Size(8,8), cv::Size(0,0), 1.1,
                       params.hog_group_threshold, params.hog_meanshift);

 // ROS_INFO("[detector::classifyCamera] - weightM size %d ", (int)(foundM.size()));

	/// TODO HOW TO ACCEPT A POSITIVE (mean shift)
    if (weightM.size() > 0)
    {
      //  ROS_INFO("[DETECTOR] after hog OK, weightM size %d , %d", (int)(weightM.size()), (int)(foundM.size()));
      prob = *max_element(weightM.begin(), weightM.end());
      ROS_INFO("[detector::classifyCamera] max element %f", prob);
    }
}

void detector::detectFusion(vector<hdetect::ClusteredScan> &clusterData, hdetect::ClusterClass &detections)
{
  //ROS_INFO("[DETECTOR] clusters size %d", clusterData->clusters.size());
  // Variables where the probabilities of the detectors are stored
  // If there is no fusion the probability of the laser is taken

  float laserProb;
  float cameraProb;
  float fusionProb;

  ROS_INFO("[detector::detectFusion] - New Observation");
  for (uint i = 0; i < clusterData.size(); i++)
  {
    // put the cog into the ClusterClass

    // ONLY with camera
    laserProb = 0.0000001;//MIN_PROB;
    cameraProb = 0.0000001;//MIN_PROB;
    classifyLaser(clusterData[i].features, laserProb);

//    ROS_INFO("[detector::detectFusion] - laserProb %.2f", laserProb);

    if(laserProb < params.min_laser_prob)
    {
        clusterData[i].crop_projected == false;
        ROS_INFO("[detector::detectFusion] - Cluster %d Pos (%.2f %.2f) - Laser Prob: %f", i, clusterData[i].cog.x, clusterData[i].cog.y, laserProb);

    }

    if (clusterData[i].crop_projected == true)
    {

      ROS_INFO("[detector::detectFusion] - Cluster %d Pos (%.2f %.2f) - Laser Prob: %f", i, clusterData[i].cog.x, clusterData[i].cog.y, laserProb);

      classifyCamera(clusterData[i].cog, cameraProb);
      if (cameraProb == 0)
      {
        clusterData[i].crop_projected = false;
        // Take Down probability (Not possible to make fusion with zero)
        cameraProb = 0.0000001;
      }
    }
    //Bayesian fusion
    fusionProb = (laserProb * cameraProb) /
            ((laserProb * cameraProb) + (1.0 - laserProb) * (1.0 - cameraProb) );

    ROS_INFO("[detector::detectFusion] - FusionProb %f", fusionProb);

    clusterData[i].detection_laser_prob = laserProb;
    clusterData[i].detection_camera_prob = cameraProb;
    clusterData[i].detection_fusion_prob = fusionProb;

//    if (fusionProb > params.fusion_prob)
//    {
      if (clusterData[i].crop_projected == true)
      {
        if(cameraProb > params.min_camera_prob)
          clusterData[i].detection_label = FUSION_HUMAN;
      }
      else
      {
          if(laserProb > params.min_laser_prob)
              clusterData[i].detection_label = LASER_HUMAN;
      }
//    }
    //        ROS_INFO("[DETECTOR] cluster %d: projection %d fusion %d prob %3.3f label %d",i+1,
    //                clusterData->projected[i], clusterData->fusion[i], clusterData->detection_probs[i], clusterData->detection_labels[i]);
  }
  //    ROS_INFO("[DETECTOR] Publishing detections");
  laserProcessor->getHeader(detections.header);
  detections.clusterData = clusterData;
  detectionPublisher.publish(detections);
}


void detector::detectHumans(const Image::ConstPtr &image, const LaserScan::ConstPtr &lScan)
{
    clusterData.clear();
    detections = hdetect::ClusterClass();

    getTF(image, lScan);
    getImage(image);
    processLaser(lScan, clusterData);

    detectFusion(clusterData, detections);
}
