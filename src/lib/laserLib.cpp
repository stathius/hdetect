/* LaserTranslator.cpp
Transforms the laserscan to a format that the external people2d_engine (modified) 
can read. It returns
Clusters as LaserClustersMsg
Features as ScanFeaturesMsg

Stathis Fotiadis 2012
 */
#include "hdetect/lib/laserLib.hpp"


/// Starts the laser filter. people2D_engine parameter setting.
laserLib::laserLib(double &jumpdist, int feature_set, double laser_range ) : laserFilter("sensor_msgs::LaserScan") {
	// Set the configuration
	// segmentation distance
	libEngineParams.jumpdist = jumpdist;
	libEngineParams.feature_set = feature_set; // WE SHOULD TRY CHANGING THAT TO 2
	libEngineParams.laser_range = laser_range;

	libEngineParams.sanity = 1;
	libEngineParams.segonly = 0;

	libEngine = new lengine( libEngineParams );

	// Shadow filtering parameters
	// Must have been loaded to the paramater server
	laserFilter.configure("scan_filter_chain");
}

laserLib::~laserLib(void) {

}

/** Filters and loads the laser scan.
    First it processes the laser scan with a shadow filter.
    Does conversion to people2D_engine format using scan2lib()
    Finally segments the result to clusters, still in people2D_engine format.
    To get the clusters in ROS use getClusters().
 */
void laserLib::loadScan(sensor_msgs::LaserScan ls) {

	// First apply the filter
	// WARNING THE FILTER KEEPS THE FILTERD DATA BUT WITH REVERSE RANGE (*it)
	laserFilter.update(ls, filtScan);

	/*printf("\npre  %d",ls.ranges.size());
        int filtp=0;
        for(uint cnt=0; cnt<ls.ranges.size(); cnt++)
          if(filtScan.ranges[cnt]>0.0) filtp++;
        printf("\nfilt %d\n",filtp);
	 */

	// Convert filtered scan to a library ready format into variable libScan
	scan2lib(filtScan, libScanData);

	// Load the data
	libEngine->load_scandata(libScanData);
	// Copy over the header
	seq=ls.header.seq;
	stamp=ls.header.stamp;
	frame_id=ls.header.frame_id;

	// Segment the data
	clusters.clear();
	libEngine->segmentscanJDC(clusters);
}

/** The API function used to get the features.
 *  Computes the features and saves them in the ClusteredScan message.
 *  Must use loadScan() first.
 */
void laserLib::getFeatures(hdetect::ClusteredScan &features) {

	// Compute the features
	libEngine->computeFeatures(clusters, descriptor);

	// Convert features to Float32MultiArray
	features2ROS(features);

	// Copy the header from the laser scan	
	//features.header.seq=seq;
	//features.header.stamp=stamp;
	//features.header.frame_id=frame_id;

	/*
	ROS_INFO("clusters %d", clusters.size());
	ROS_INFO("features %d", descriptor.size());
	ROS_INFO("features ROS %d", features.features.size());
	ROS_INFO("features dim %d", descriptor[0].size());
	ROS_INFO("features dim ROS %d", features.features[0].data.size());
	//*/
}

/** Converts the features to a ROS compatible ClusteredScan message
 *  The timestamp of the message is set to the timestamp of the image.
 *  Each cluster's annotation initialised to FALSE (0)
 *  Each cluster's fusion attribute initialised to FALSE (0)
 * */
void laserLib::features2ROS(hdetect::ClusteredScan &features)
{
	features.features.clear();
	features.features.resize(descriptor.size());
	features.labels.clear();
	features.fusion.clear();
	features.projected.clear();
	//features.features.resize( descriptor.size() );
	features.nclusters=descriptor.size();
	features.nfeatures=descriptor[0].size();

	int i=0;

	for (std::vector< std::vector <float> >::iterator
			it = descriptor.begin(); it!=descriptor.end(); it++)
	{
		// initialize the label to -1
		features.labels.push_back(NO_HUMAN);
		features.fusion.push_back(0);
		features.projected.push_back(0);
		for (std::vector<float>::iterator it2 = it->begin();
				it2!=it->end(); it2++)
		{
			features.features[i].data.push_back(*it2);
		}
		i++;
	}

}

/**
 *  Converts the scan from ROS to a format that the people2D_engine library understands
 *  The LaserScan is converted to a laserscan_data object */
void laserLib::scan2lib (sensor_msgs::LaserScan &ls, laserscan_data &libScan) {

	// read angle min , max and increament
	angle_min = ls.angle_min;
	angle_max = ls.angle_max;
	angle_inc = ls.angle_increment;

	// counter
	int i = 0;

	// angle
	float ang;

	// helping LSL_Point3D_str
	Point3D_str point;
	point.z=0;
	point.label=0;

	// clear the data of libScan
	libScan.data.pts.clear();


	for (std::vector<float>::iterator it = ls.ranges.begin(); it!=ls.ranges.end(); it++) 	{
		// Convert the laserscan from ROS to library format

		// Compute the x,y of the point	
		ang = angle_min + (float) i * angle_inc;

		// WARNING: WE DO THAT BECAUSE THE FILTER KEEPS THE DATA BUT WITH REVERSE RANGE (*it)
		if ( *it > 0.0 && *it <= libEngineParams.laser_range )
		{
			point.x = *it * cos(ang);
			point.y = *it * sin(ang);

			// Push it to the libScan vector if its a valid range
			libScan.data.pts.push_back(point);

		}
		//else
		//	ROS_WARN("[LASER_LIB] point out of bounds r %f - x %f - y %f ", *it, point.x, point.y);

		//ROS_INFO("i %d libScan size before %d",i, libScan.data.pts.size());
		/*
		ROS_INFO("i %d", i);
		ROS_INFO("r %f",*it);
		ROS_INFO("a %f", ang);
		ROS_INFO("x %f", point.x);
		ROS_INFO("y %f\n", point.y);
		sleep(0.1);
		//*/

		i++;

	}
	printf("[LASER_LIB] scan2lib: Laser scan Prefiltered = %d - Filtered = %d\n",ls.ranges.size(),libScan.data.pts.size());

}

/** The API function used to get the clusters.
 *  Computes the individual clusters and saves them in the ClusteredScan message.
 *  Must use loadScan() first.
 */
void laserLib::getClusters(hdetect::ClusteredScan &laserClusters) {

	// Set the header
	laserClusters.header.seq=seq;
	laserClusters.header.stamp=stamp;
	laserClusters.header.frame_id=frame_id;	


	// Clear the data for clusters and cogs
	laserClusters.clusters.clear();	
	laserClusters.clusters.resize( clusters.size() );
	laserClusters.cogs.clear();

	int i=0;
	for (std::vector<Point3D_container>::iterator 
			it = clusters.begin(); it!=clusters.end(); it++)
	{
		// compute cog
		clusters[i].compute_cog(&cogL);
		// put cluster only if it's more than 20cm away from the  laser
		//ROS_INFO("cluster %d dist %f x %f y %f",i, distance_L2_XY(&cogLSL,&origin),cogLSL.x, cogLSL.y);

		// discard cluster too close to the laser
		//if (distance_L2_XY(&cogLSL,&origin) > 0.2)
		//{
		cogROS.x=cogL.x;
		cogROS.y=cogL.y;
		cogROS.z=cogL.z;

		laserClusters.cogs.push_back(cogROS);

		for (std::vector<Point3D_str>::iterator it2 = it->pts.begin();
				it2!=it->pts.end(); it2++)
		{
			pt.x=it2->x;
			pt.y=it2->y;
			pt.z=it2->z;
			//if( sqrt( pow(pt.x,2) + pow(pt.y,2) ) > libEngineParams.laser_range )
				laserClusters.clusters[i].points.push_back(pt);
		}
		i++;
		//}
}
	//ROS_INFO("clusters %d", laserClusters.clusters.size());
	//ROS_INFO("cogs ROS %d", laserClusters.cogs.size());
}
