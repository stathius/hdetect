#include "hdetect/lib/annotator.hpp"

/**
 * Creates the directory and the files to save
 */
annotator::annotator(string bagFile)
{
  ROS_INFO("[ANNOTATOR] 1");

  std::string abs_path;
  ros::NodeHandle nh;
  if(nh.hasParam("pkg_path")) {
    ros::param::get("/pkg_path",abs_path);
  }
  else ROS_ERROR("[ANNOTATOR] Parameter pkg_path (absolute package path) not found.");

  // File where the image crops will be saved
  bagFile = bagFile.substr( bagFile.find_last_of("UTMFF__") + 1,
                            bagFile.find_last_of(".") -
                            bagFile.find_last_of("UTMFF__") - 1 );

  // Create the subdirectory for the specific bag
  bagDir = abs_path+"/data/"+bagFile;
  ROS_INFO("[ANNOTATOR] Creating directory %s", bagDir.c_str());
  mkdir(bagDir.c_str(), S_IRWXG | S_IRWXO | S_IRWXU);

  // Open the CSV file for writing
  bagFile=bagDir+"/annotation.csv";
  f = fopen(bagFile.c_str(), "w");
  if (f == NULL)
  {
    ROS_ERROR("[ANNOTATOR] CSV File to write annotation data couldn't be opened");
    exit(-1);
  }
  ROS_INFO("[ANNOTATOR] Writing CSV annotation data to file %s", bagFile.c_str());

  // Opening the bag file to write everything
  bagFile=bagDir+"/everything.bag";
  bag.open(bagFile, rosbag::bagmode::Write);
  ROS_INFO("[ANNOTATOR] Writing all data to BAG file %s", bagFile.c_str());


  // Create the subdirectory of the crops of the specific bag
  bagDir = bagDir+"/crops/";
  ROS_INFO("[ANNOTATOR] Creating directory %s", bagDir.c_str());
  mkdir(bagDir.c_str(), S_IRWXG | S_IRWXO | S_IRWXU);

  ROS_INFO("[ANNOTATOR] CONSTRUCTOR END");
}

/// Closes the open files and windows.
annotator::~annotator()
{
  fclose(f);
  bag.close();
}


/**
 * Gets the annotation from the user.
 * It superimposes the clusters on the image along with their bounding boxes if available.
 * Also draws a 2d plane of the laser scan to facilitate the annotation process.
 * Asks the user to annotate the cluster. The user can only annotate clusters that can be seen on the image.
 * Saves all the data: annotation, clusters, features, laserscan, image etc in a bag.
 * The annotated clusters (only those that can be seen on the image) and their features
 * are also saved in csv format.
 * This means that the annotation of the off-screen clusters is NOT VALID.
 * If the bounding box of a projected cluster is within the image limits, a crop is also saved to a seperate file.
 * This function serves a the callback for the synchronizer in AnnotateData.cpp
 *
 */
void annotator::annotateData(const sensor_msgs::Image::ConstPtr &image,
                             const sensor_msgs::LaserScan::ConstPtr &lScan)
{
  printf("\n\nSCAN No %04d\n", scanNo);

  visualizeData(image, lScan);
  waitKey(200);

  uint annot;
  string input;

  printf("\n");
  do
  {
    printf("\nGive the number of the human clusters [enter to proceed]: ");
    //getline(cin, input);

    // if it's an accurate integer change the cluster name
    if (stringstream(input) >> annot)
      if (annot > 0 && annot < clusterData.labels.size()) // && projectedClusters[annot] == 1)
        clusterData.labels[annot] = 1;
      else
        //printf("\nInput should be between 0 and %d and it should be a projected cluster\n", scanClusters.labels.size());
        printf("\nInput should be between 0 and %d\n", clusterData.labels.size());
    else if (!input.compare(""))
      printf("\nMoving to the next scan\n");
    else
      printf("\nInput should be an integer");

  } while (input.compare(""));


  for(uint i = 0; i < clusterData.fusion.size() ; i++)
  {

    if (clusterData.fusion[i] == 1) {
      projectPoint(clusterData.cogs[i], prPixel, params.cInfo, transform);
      getBox(clusterData.cogs[i], prPixel, boxSize, upleft, downright, params.m_to_pixels, params.body_ratio);

        getCrop (crop, cv_ptr->image, upleft, boxSize);


        char cropID[30];
        if(clusterData.labels[i] == 1)
        	sprintf(cropID,"S%04d_C_%03d_1.pgm", scanNo, i);
        else
        	sprintf(cropID,"S%04d_C_%03d_0.pgm", scanNo, i);

        stringstream ss;
        ss << cropID;
        string cname;
        ss >> cname;
        cname=bagDir+cname;

        ROS_INFO("[ANNOTATOR] Saving crop : %s", cname.c_str());
        imwrite(cname, crop);
    }
  }

  for (uint i = 0; i < clusterData.labels.size(); i++)
    printf("cluster %03d\tannotation %1d\tprojected %1d\tfusion %1d\n", i, clusterData.labels[i], clusterData.projected[i],
           clusterData.fusion[i]);

  saveBag(image, lScan);

  saveCSV();

  scanNo++;

}

/// Saves to everything (image, camera info, laser scan, ClusteredScan) to a bag file
void annotator::saveBag(const sensor_msgs::Image::ConstPtr &image,
                        const sensor_msgs::LaserScan::ConstPtr &lScan)
{
//  sensor_msgs::Image new_image = *image;
//  sensor_msgs::LaserScan new_lScan = *lScan;

  // Set all the timestamps to the acquisition time (camera)
  clusterData.header.stamp = acquisition_time;
  /*
  new_image.header.stamp= acquisition_time;
  new_lScan.header.stamp= acquisition_time;

  bag.write("/camera/image", acquisition_time, new_image );
  bag.write("/laser/scan", acquisition_time, new_lScan );
  */
  clusterData.scan = *lScan;
  clusterData.image = *image;
  bag.write("/ClusteredScan", acquisition_time, clusterData );

}

/// Used to save the annotated data in CSV format, even those that are not present in the image.
void annotator::saveCSV() {

  int i,j;
  for (i = 0; i < clusterData.nclusters; i++)
  {
    //if (projectedClusters[i] == 1) { // IF THIS IS UNCOMMENTED IT ONLY SAVES VISIBLE CLUSTERS
    fprintf(f,"%04d %d %d", scanNo, clusterData.fusion[i], clusterData.labels[i]);

    for (j = 0; j < clusterData.nfeatures; j++) {
      fprintf(f,", %3.20f", clusterData.features[i].data[j]);
    }

    fprintf(f,"\n");
    //}
  }

}
