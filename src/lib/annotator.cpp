#include "hdetect/lib/annotator.hpp"

using namespace std;
using namespace cv;
using namespace Header;

/**
 * Creates the directory and the files to save
 */
annotator::annotator(string bagFile)
{
    ROS_INFO("[ANNOTATOR] Intializing");

    scanNo = 0;

    string abs_path;
    ros::NodeHandle nh;

    if (nh.hasParam("pkg_path"))
    {
        ros::param::get("/pkg_path",abs_path);
    }
    else
    {
        ROS_ERROR("[ANNOTATOR] Parameter pkg_path (absolute package path) not found.");
    }

  // File where the image crops will be saved
    bagFile = bagFile.substr(bagFile.find_last_of("/") + 1, bagFile.find_last_of(".") - bagFile.find_last_of("/") - 1);

    // Create the subdirectory for the specific bag
    bagDir = abs_path + "/data/"+bagFile;
    ROS_INFO("[ANNOTATOR] Creating directory %s", bagDir.c_str());
    mkdir(bagDir.c_str(), S_IRWXG | S_IRWXO | S_IRWXU);

    // Open the CSV file for writing
    bagFile = bagDir + "/annotation.csv";
    f = fopen(bagFile.c_str(), "w");

    if (f == NULL)
    {
        ROS_ERROR("[ANNOTATOR] CSV File to write annotation data couldn't be opened");
        exit(-1);
    }

    ROS_INFO("[ANNOTATOR] Writing CSV annotation data to file %s", bagFile.c_str());

    // Opening the bag file to write everything
    bagFile = bagDir + "/everything.bag";
    bag.open(bagFile, rosbag::bagmode::Write);
    ROS_INFO("[ANNOTATOR] Writing all data to BAG file %s", bagFile.c_str());


    // Create the subdirectory of the crops of the specific bag
    bagDir = bagDir + "/crops/";
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
    visualizeData(image, lScan);
    waitKey(200);

    int annot;
    string input;
    set<int> humans;

    // For warning message
    bool error = false;
    char warning[1000];
    sprintf(warning, "\n--------------------------------------------------------\n"
                     "     Input should be an integer between 0 and %d\n"
            "--------------------------------------------------------\n",(int)(clusterData.size() - 1));

    // Check if there are some points which have been labeled before
    for (uint i = 0; i < clusterData.size(); i++)
    {
        for (uint j = 0; j < prev_points.size(); j++)
        {
            float dis = calculateEucDis(clusterData[i].cog, prev_points[j]);

            if (dis < 0.35)
            {
                humans.insert(i);

                break;
            }
        }
    }
    do
    {
        system("clear");

        printf("\nSCAN No %04d\n", scanNo);

        printf("\nCurrently there are %d human clusters", (int)(humans.size()));

        if (humans.size() > 0)
        {
            printf(": \n");

            // For automatic labeling
            for (set<int>::iterator it = humans.begin(); it != humans.end(); it++)
            {
                printf("[%d]\n", *it);
            }
        }
        else
        {
            printf(".\n");
        }

        if (error == true)
        {
            printf("%s", warning);
        }

        printf("\nPlease give the number of the human clusters.\n"
               "[enter] to proceed, [number] to label and unlabel: ");

        getline(cin, input);

        // Check if input is not "", trim the string
        if (input.compare("") > 0)
        {
            input = input.substr(input.find_first_not_of(" "));
        }

        // If it's an accurate integer change the cluster name
        if (stringstream(input) >> annot && annot >= 0 && (uint)annot < clusterData.size())
        {
            // Label is not found
            if (humans.count(annot) == 0)
            {
                humans.insert(annot);
            }
            // Label is found
            else
            {
                humans.erase(annot);
            }

            error = false;
        }
        // If there is an error input
        else if (input.compare("") > 0)
        {
            error = true;
        }
    }
    while (input.compare(""));

    // For load the label from the set
    for (set<int>::iterator it = humans.begin(); it != humans.end(); it++)
    {
        clusterData[*it].label = LABEL_HUMAN;
    }


    // Save image crops
//    for (uint i = 0; i < clusterData.fusion.size() ; i++)
//    {
//        if (clusterData.fusion[i] == 1)
//        {
//            projectPoint(clusterData.cogs[i], prPixel, K, D, transform);
//            getBox(clusterData.cogs[i], prPixel, boxSize, upleft, downright, params.m_to_pixels, params.body_ratio);
//            getCrop (crop, cv_ptr->image, upleft, boxSize);


//            char cropID[30] = 0;

//            if (clusterData.labels[i] == 1)
//            {
//                sprintf(cropID,"S%04d_C_%03d_1.pgm", scanNo, i);
//            }
//            else
//            {
//                sprintf(cropID,"S%04d_C_%03d_0.pgm", scanNo, i);
//            }

//            stringstream ss;
//            ss << cropID;
//            string cname;
//            ss >> cname;
//            cname = bagDir + cname;

//            ROS_INFO("[ANNOTATOR] Saving crop : %s", cname.c_str());
//            imwrite(cname, crop);
//        }
//    }

//    saveBag(image, lScan);

    saveCSV();

    prev_points.clear();

//    printf("\nClusters are: \n\n");

    for (uint i = 0; i < clusterData.size(); i++)
    {
        if (clusterData[i].label == LABEL_HUMAN)
        {
//            printf("cluster %03d\tlabel %1d\tprojected %1d\tfusion %1d\n",
//                   i,
//                   clusterData[i].label,
//                   clusterData[i].cog_projected,
//                   clusterData[i].crop_projected);

            prev_points.push_back(clusterData[i].cog);
        }
    }

    scanNo++;
}

/// Saves to everything (image, camera info, laser scan, ClusteredScan) to a bag file
void annotator::saveBag(const sensor_msgs::Image::ConstPtr &image,
                        const sensor_msgs::LaserScan::ConstPtr &lScan)
{
    sensor_msgs::Image new_image = *image;
    sensor_msgs::LaserScan new_lScan = *lScan;

    // Set all the timestamps to the acquisition time (camera)
//    new_image.header.stamp = acquisition_time;
//    new_lScan.header.stamp = acquisition_time;
    detections.header.stamp = image->header.stamp;

    bag.write("/camera/image", acquisition_time, new_image );
    bag.write("/laser_top/scan", acquisition_time, new_lScan );
    bag.write("/ClusteredClass", acquisition_time, detections);
}

/// Used to save the annotated data in CSV format, even those that are not present in the image.
void annotator::saveCSV()
{
    uint cluster_size = clusterData.size();
    uint features_size = clusterData[0].features.data.size();

    for (uint i = 0; i < cluster_size; i++)
    {
    //if (projectedClusters[i] == 1) { // IF THIS IS UNCOMMENTED IT ONLY SAVES VISIBLE CLUSTERS
//        fprintf(f, "%04d %d %d", scanNo, clusterData.fusion[i], clusterData.labels[i]);
        fprintf(f, "%d", clusterData[i].label);

        for (uint j = 0; j < features_size; j++)
        {
            fprintf(f, ", %.6f", clusterData[i].features.data[j]);
        }

        fprintf(f, "\n");
    //}
    }
}

float annotator::calculateEucDis(geometry_msgs::Point32 &point1, geometry_msgs::Point32 &point2)
{
    float sum1 = (point1.x - point2.x) * (point1.x - point2.x);
    float sum2 = (point1.y - point2.y) * (point1.y - point2.y);

    return sqrt(sum1 + sum2);
}
