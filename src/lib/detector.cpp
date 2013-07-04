#include <hdetect/lib/detector.hpp>

detector::detector() : nh("~")
{
	//string s = nh.getNamespace();
	//ROS_INFO("[DETECTOR] my namespace %s ", s.c_str());

	if(nh.hasParam("camera_yaml") && nh.hasParam("camera_name") && nh.hasParam("boost_xml")) {
		string camera_yaml;
		string cname;
		string boost_xml;

		nh.getParam("camera_yaml", camera_yaml);

		nh.getParam("camera_name", cname);

		nh.getParam("boost_xml", boost_xml);

		camera_calibration_parsers::readCalibrationYml(camera_yaml, cname, params.cInfo);

		// Load the boost classifier
		boost.load( boost_xml.c_str() , "boost");

		ROS_INFO("[DETECTOR] Camera calibration & Boost Classifier Loaded");
	}
	else ROS_WARN("[DETECTOR] Need to set the parameters in order to load the camera calibration and the boost classifier.");

	if(nh.hasParam("hog_hit_threshold") && nh.hasParam("hog_group_threshold")
			&& nh.hasParam("hog_meanshift") && nh.hasParam("tf_timeout")
			//&& nh.hasParam("no_features")
			&& nh.hasParam("cameraA") && nh.hasParam("cameraB") && nh.hasParam("laserA")
			&& nh.hasParam("laserB") && nh.hasParam("m_to_pixels") && nh.hasParam("body_ratio")
			&& nh.hasParam("jumpdist") && nh.hasParam("feature_set") && nh.hasParam("laser_range")) {

		//nh.getParam("no_features", params.no_features);
		nh.getParam("hog_hit_threshold", params.hog_hit_threshold);
		nh.getParam("hog_group_threshold", params.hog_group_threshold);
		nh.getParam("hog_meanshift", params.hog_meanshift);
		nh.getParam("tf_timeout", params.tf_timeout);
		nh.getParam("tf_timeout", params.tf_timeout);
		nh.getParam("cameraA", params.cameraA);
		nh.getParam("cameraB", params.cameraB);
		nh.getParam("laserA", params.laserA);
		nh.getParam("laserB", params.laserB);
		nh.getParam("m_to_pixels", params.m_to_pixels);
		nh.getParam("body_ratio", params.body_ratio);
		nh.getParam("jumpdist", params.jumpdist);
		nh.getParam("feature_set", params.feature_set);
		nh.getParam("laser_range", params.laser_range);
		ROS_INFO("[DETECTOR] Parameters loaded.");
	}
	else
		ROS_ERROR("[DETECTOR] Wrong parameters loaded.");

	// Initiates the laserLib with the parameters read from the server
	laserProcessor = new laserLib(params.jumpdist, params.feature_set, params.laser_range);

	// Set the default detector
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

	scanNo = 0;

	switch(params.feature_set) {
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
	lFeatures = cv::Mat::zeros(params.no_features,1,CV_64F);

	ROS_INFO("[DETECTOR] Detector running OK with %d features.", params.no_features);
}

void detector::setClusters(hdetect::ClusteredScan sd)
{
	clusterData=sd;
}

hdetect::ClusteredScan detector::getClusters()
{
	return clusterData;
}

void detector::getTF(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::LaserScan::ConstPtr &lScan) {
	// Read the transform between the laser and the camera
	// Essential for syncronising
	try
	{
		acquisition_time = image->header.stamp; /// Maybe need to change to ros::Time::now()
		ros::Duration timeout(1.0 / params.tf_timeout);
		tf_listener_.waitForTransform(image->header.frame_id, lScan->header.frame_id, acquisition_time, timeout);
		tf_listener_.lookupTransform(image->header.frame_id, lScan->header.frame_id, acquisition_time, transform);
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("[DETECTOR] TF exception:\n%s", ex.what());
	}
}

void detector::getImage(const sensor_msgs::Image::ConstPtr &image) {
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

void detector::getLaser(const sensor_msgs::LaserScan::ConstPtr &lScan)
{
	// Load the scan to the processor
	laserProcessor->loadScan(*lScan);

	// Segment the laser scan
	laserProcessor->getClusters(clusterData);

	// Only process scans with valid data
	if(clusterData.clusters.size()>0) {
		laserProcessor->getFeatures(clusterData);
	}
	else
		ROS_WARN("[DETECTOR] No valid clusters. Scan %04d", scanNo);

	projectLaser();
}

void detector::projectLaser()
{

	int clusterNo = 0;
	// Iterate through every cog of the scanClusters
	for (std::vector<geometry_msgs::Point32>::iterator it = clusterData.cogs.begin();
			it != clusterData.cogs.end();
			it++)
	{

		// Convert the cog to image coordinates
		projectPoint(*it, prPixel, params.cInfo, transform);

		/// If the pixel is projected within the image limits
		if (prPixel.x >= 0 && prPixel.x < cv_ptr->image.cols && prPixel.y >= 0 && prPixel.y < cv_ptr->image.rows)
		{
			clusterData.projected[clusterNo]=1;

			// Get the box size and corners
			getBox(*it, prPixel, boxSize, upleft, downright, params.m_to_pixels, params.body_ratio);

			// Check if whole of the box lies inside the image
			if (checkBox(params.cInfo, upleft, downright))
			{
				// Flag the cluster as 'fusable'. Meaning they appear with a valid box on the image.
				clusterData.fusion[clusterNo] = 1;
			}

		}
		clusterNo++;
	}
}


void detector::detectLaser(std_msgs::Float32MultiArray &features)
{
	int i;
	for (i = 0; i < features.data.size() ; i++)
	{
		lFeatures.at<double>(i,0) = features.data[i];
	}
	if (i != params.no_features-1) ROS_ERROR("[DETECTOR] Wrong number of computed features.");

	// Find the prediction
	float pred = boost.predict(lFeatures, Mat(), Range::all(), false, true);

	if (pred >= 0.0)
		laserClass.push_back(1);
	else
		laserClass.push_back(-1);

	// Convert prediction to probabilty
	pred = 1 / ( 1 + exp(params.laserA * pred + params.laserB) );
	laserProb.push_back(pred);
}

void detector::detectCamera(geometry_msgs::Point32 &cog)
{
	// Convert the cog to image coordinates
	projectPoint(cog, prPixel, params.cInfo, transform);

	// Get the box size and corners
	getBox(cog, prPixel, boxSize, upleft, downright, params.m_to_pixels, params.body_ratio);

	// Extract the crop from the image
	getCrop (crop, cv_ptr->image, upleft, boxSize);

	//cv::cvtColor(src_img, mono_img, CV_BGR2GRAY);
	std::vector<cv::Rect> foundM;
	vector<double> weightM;

	// We don't really care about the class so we put the threshold really low to even negative predictions
	hog.detectMultiScale(crop, foundM, weightM, params.hog_hit_threshold, cv::Size(8,8), cv::Size(0,0), 1.05 , params.hog_group_threshold, params.hog_meanshift);
	/// TODO HOW TO ACCEPT A POSITIVE (mean shift)
	float pred = *max_element( weightM.begin(), weightM.end() );
	pred = 1 / ( 1 + exp(params.cameraA * pred + params.cameraB));
	cameraProb.push_back(pred);
}

void detector::detectFusion() {

	for (uint i = 0; i < clusterData.clusters.size(); i++)
	{
		detectLaser(clusterData.features[i]);

		if (clusterData.fusion[i] == 1)
		{
			detectCamera(clusterData.cogs[i]);
			// Bayesian fusion
			fusionProb.push_back( ( laserProb[i] * cameraProb[i] ) /
					( ( laserProb[i] * cameraProb[i] ) + ( 1.0 - laserProb[i] ) * ( 1.0 - cameraProb[i] ) ) ); //
		} else
		{
			fusionProb.push_back(laserProb[i]);
		}
	}
}


void detector::extractData(const sensor_msgs::Image::ConstPtr &image,
		const sensor_msgs::LaserScan::ConstPtr &lScan)
{
	getTF(image, lScan);
	getImage(image);
	getLaser(lScan);

	//detectPedestrian();
}
