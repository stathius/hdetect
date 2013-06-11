#include <upm/lib/detector.hpp>

detector::detector()
{
	if(nh.hasParam("pkg_path")) {
		std::string calib_file;
		std::string path;
		std::string name=("00b09d0100aa73bb");

		ros::param::get("/pkg_path",path);
		calib_file = path + "/yaml/camera_calib.yaml";

		camera_calibration_parsers::readCalibrationYml(calib_file, name, cInfo);

		ROS_INFO("[DETECTOR] Camera calibration file %s loaded.", calib_file.c_str());

		// Load the boost classifier
		std::string boost_file = path + "/data/trained_boost.xml";
		boost.load( boost_file.c_str() , "boost");
	}
	else ROS_WARN("[DETECTOR] Need to set the parameter /pkg_path in order to load the camera calibration and the boost classifier.");


	// Set the default detector
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

	scanNo = 0;
	clusterNo = 0;

	// It might need change to 1,17
	lFeatures = cv::Mat::zeros(17,1,CV_64F);
}

void detector::setClusters(upm::ClusteredScan sd)
{
	scanData=sd;
}

upm::ClusteredScan detector::getClusters()
{
	return scanData;
}

void detector::getTF(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::LaserScan::ConstPtr &lScan) {
	// Read the transform between the laser and the camera
	// Essential for syncronising
	try
	{
		acquisition_time = image->header.stamp; /// Maybe need to change to ros::Time::now()
		ros::Duration timeout(1.0 / 30);
		tf_listener_.waitForTransform(image->header.frame_id, lScan->header.frame_id, acquisition_time, timeout);
		tf_listener_.lookupTransform(image->header.frame_id, lScan->header.frame_id, acquisition_time, transform);
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("[EXTRACTOR] TF exception:\n%s", ex.what());
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
		ROS_ERROR("[EXTRACTOR] cv_bridge exception: %s", e.what());
		return;
	}
}

void detector::getLaser(const sensor_msgs::LaserScan::ConstPtr &lScan)
{
	// Load the scan to the processor
	laserProcessor.loadScan(*lScan);
	// Segment the laser scan
	laserProcessor.getClusters(scanData);

	// Only process scans with valid data
	if(scanData.clusters.size()>0) {
		laserProcessor.getFeatures(scanData);
	}
	else
		ROS_ERROR("[EXTRACTOR] Cluster with less than 3 points. Scan %04d  Cluster %03d", scanNo, clusterNo);
}

void detector::extractLaser()
{
	// Iterate through every cog of the scanClusters
	for (std::vector<geometry_msgs::Point32>::iterator it = scanData.cogs.begin();
			it != scanData.cogs.end();
			it++)
	{
		// Convert the cog to image coordinates
		projectPoint(*it, prPixel, cInfo, transform);

		/// If the pixel is projected within the image limits
		if (prPixel.x >= 0 && prPixel.x < cv_ptr->image.cols && prPixel.y >= 0 && prPixel.y < cv_ptr->image.rows)
		{
			scanData.projected[clusterNo]=1;

			// Get the box size and corners
			getBox(*it, prPixel, boxSize, upleft, downright);

			// Check if whole of the box lies inside the image
			if (checkBox(cInfo, upleft, downright))
			{
				// Flag the cluster as 'fusable'. Meaning they appear with a valid box on the image.
				scanData.fusion[clusterNo] = 1;
			}

		}
		clusterNo++;
	}
}


void detector::detectLaser(std_msgs::Float32MultiArray &features)
{
	uint i;
	for (i = 0; i < features.data.size() ; i++)
	{
		lFeatures.at<double>(i,0) = features.data[i];
	}
	if (i != 16) ROS_ERROR("[DETECTOR] Wrong number of computed features.");

	// Find the prediction
	float pred = boost.predict(lFeatures, Mat(), Range::all(), false, true);

	if (pred >= 0.0)
		laserClass.push_back(1);
	else
		laserClass.push_back(-1);

	// Convert prediction to probabilty
	pred = 1 / ( 1 + exp(-2.0 * pred) );
	laserProb.push_back(pred);
}

void detector::detectCamera(geometry_msgs::Point32 &cog)
{
	// Convert the cog to image coordinates
	projectPoint(cog, prPixel, cInfo, transform);

	// Get the box size and corners
	getBox(cog, prPixel, boxSize, upleft, downright);

	// Extract the crop from the image
	getCrop (crop, cv_ptr->image, upleft, boxSize);

	//cv::cvtColor(src_img, mono_img, CV_BGR2GRAY);
	std::vector<cv::Rect> foundM;
	vector<double> weightM;

	// We don't really care about the class so we put the threshold really low to even negative predictions
	hog.detectMultiScale(crop, foundM, weightM, HOG_THRESHOLD, cv::Size(8,8), cv::Size(0,0), 1.05 , 0, 1);

	float pred = *max_element( weightM.begin(), weightM.end() );
	pred = 1 / ( 1 + exp(-pred));
	cameraProb.push_back(pred);
}

void detector::detectPedestrian() {

	for (uint i = 0; i < scanData.clusters.size(); i++)
	{
		detectLaser(scanData.features[i]);

		if (scanData.fusion[i] == 1)
		{
			detectCamera(scanData.cogs[i]);
		} else
		{
			cameraProb.push_back(0.50);
		}
		// Bayesian fusion
		fusionProb.push_back( ( laserProb[i] * cameraProb[i] ) /
				( ( laserProb[i] * cameraProb[i] ) + ( 1.0 - laserProb[i] ) * ( 1.0 - cameraProb[i] ) ) ); //
	}
}


void detector::extractData(const sensor_msgs::Image::ConstPtr &image,
		const sensor_msgs::LaserScan::ConstPtr &lScan)
{
	getTF(image, lScan);
	getImage(image);
	getLaser(lScan);

	extractLaser();
	//extractVision();
	//detectPedestrian();
}
