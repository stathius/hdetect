/* LaserTranslator.cpp
Transforms the laserscan to a format that the external people2d_engine (modified) 
can read. It returns
Clusters as LaserClustersMsg
Features as ScanFeaturesMsg

Stathis Fotiadis 2012
 */
#include "hdetect/lib/laserLib.hpp"

using namespace std;

/// Starts the laser filter. people2D_engine parameter setting.
laserLib::laserLib(double jumpdist, int feature_set, double laser_range ) : laserFilter("sensor_msgs::LaserScan") {
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

laserLib::~laserLib()
{
    delete libEngine;
}

void laserLib::getHeader(std_msgs::Header &header)
{
    header.stamp = stamp;
    header.seq = seq;
    header.frame_id = frame_id;
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
void laserLib::getFeatures(vector<hdetect::ClusteredScan> &clusterData)
{
	// Compute the features
    libEngine->computeFeatures(clusters, descriptor);

	// Convert features to Float32MultiArray
    features2ROS(clusterData);

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
void laserLib::features2ROS(vector<hdetect::ClusteredScan> &clusterData)
{
    uint descriptor_size = descriptor[0].size();

    for (uint i = 0; i < clusterData.size(); i++)
    {
        // Clear all features in cluster data
        clusterData[i].features.data.clear();

        // Insert all descriptor to cluster data
        for (uint j = 0; j < descriptor_size; j++)
        {
            clusterData[i].features.data.push_back(descriptor[i][j]);
        }
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
	//printf("[LASER_LIB] scan2lib: Laser scan Prefiltered = %d - Filtered = %d\n",ls.ranges.size(),libScan.data.pts.size());

}

/** The API function used to get the clusters.
 *  Computes the individual clusters and saves them in the ClusteredScan message.
 *  Must use loadScan() first.
 */
void laserLib::getClusters(vector<hdetect::ClusteredScan> &clusterData)
{
    // Get the amount of clusters
    uint cluster_size = clusters.size();

	// Clear the data for clusters and cogs
    clusterData.resize(cluster_size);

    for (uint i = 0; i < cluster_size; i++)
	{
		// compute cog
		clusters[i].compute_cog(&cogL);
		// put cluster only if it's more than 20cm away from the  laser
		//ROS_INFO("cluster %d dist %f x %f y %f",i, distance_L2_XY(&cogLSL,&origin),cogLSL.x, cogLSL.y);

		// discard cluster too close to the laser
		//if (distance_L2_XY(&cogLSL,&origin) > 0.2)
        //{

        clusterData[i].cog.x = cogL.x;
        clusterData[i].cog.y = cogL.y;
        clusterData[i].cog.z = cogL.z;

        // Insert all the cluster points into cluster data
        clusterData[i].clusters.points.clear();
        for (vector<Point3D_str>::iterator it2 = clusters[i].pts.begin();
             it2 != clusters[i].pts.end(); it2++)
		{
            pt32.x = it2->x;
            pt32.y = it2->y;
            pt32.z = it2->z;
			//if( sqrt( pow(pt.x,2) + pow(pt.y,2) ) > libEngineParams.laser_range )
                clusterData[i].clusters.points.push_back(pt32);
        }
		//}
}
	//ROS_INFO("clusters %d", laserClusters.clusters.size());
	//ROS_INFO("cogs ROS %d", laserClusters.cogs.size());
}
