#include <hdetect/lib/visualizer.hpp>

//using namespace std;
using std::string;
//using namespace cv;
using cv::RNG;
using cv::namedWindow;
using cv::Scalar;
using cv::Point;
using cv::waitKey;
using cv::Mat;

#define FALSE 0
#define TRUE 1

#define INITIAL_ZOOM 30

static const char C_WINDOW[] = "Visualizer: Camera + Laser";
static const char L_WINDOW[] = "Visualizer: Laser";

visualizer::visualizer()
{
	ss << std::fixed << std::setprecision(2);

	// Random generator for the random coloring of the clusters
	RNG rng(0xFFFFFFFF);
	pallete = randomColors(rng);

	// init window and font
    namedWindow(C_WINDOW, CV_WINDOW_AUTOSIZE);
    namedWindow(L_WINDOW, CV_WINDOW_AUTOSIZE);


	cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	zoom = INITIAL_ZOOM; // in meters

	// Trackbar to control the zoom
//    createTrackbar("Zoom:", L_WINDOW, &zoom, 30, &plotLaser, this);

	pub = nh.advertise<std_msgs::Byte>("Hz", 5);
	dummy.data=1;

	ROS_INFO("[VISUALIZER] Visualizer running OK.");
    colors.push_back(Scalar(191, 0, 0));
    colors.push_back(Scalar(0, 191, 0));
    colors.push_back(Scalar(0, 0, 191));
    colors.push_back(Scalar(255, 63, 0));
    colors.push_back(Scalar(63, 255, 0));
    colors.push_back(Scalar(255, 0, 63));
    colors.push_back(Scalar(63, 0, 255));
    colors.push_back(Scalar(0, 255, 63));
    colors.push_back(Scalar(0, 63, 255));
    colors.push_back(Scalar(191, 191, 0));
    colors.push_back(Scalar(191, 0, 191));
    colors.push_back(Scalar(0, 191, 191));
}

visualizer::~visualizer()
{
	cv::destroyWindow(C_WINDOW);
	cv::destroyWindow(L_WINDOW);
}


//std::vector<hdetect::ClusteredScan>  &visualizer::getClusteredData()
//{
//    return clusterData;
//}


//void visualizer::setLaserPlane(Mat lp)
//{
//    laserPlane = lp;
//}

/**
 * It superimposes the clusters on the image along with their bounding boxes if available.
 * Also draws a 2d plane of the laser scan to facilitate the annotation process.
 */
void visualizer::visualizeData(const sensor_msgs::Image::ConstPtr &image,
		const sensor_msgs::LaserScan::ConstPtr &lScan)
{
	detector::detectHumans(image, lScan);

	// Convert image to RGB
	cvtColor(cv_ptr->image, colorImage , CV_GRAY2RGB);

	// Iterate through every cog of the scanClusters
    for (uint i = 0; i < clusterData.size() ;i++)
	{
		// If the cog is in the image save its features and then plot the cluster
        if (clusterData[i].cog_projected == true)
		{
			// Get the color index
//            color = getColor(clusterData[i].cog);
            color = colors.at(i % colors.size());

			// Draw a rectangle around each crop
			//rectangle(colorImage, upleft, downright, color, 2, 8, 0);

			/// TODO add code so when in detector mode we classify the crop/cluster or only cluster

			/// This cluster is in the image
			/// Doedsn't matter if we use fusion or only laser, we want to show the classification to the image
			/// check if fused
			/// use getCrop
			/// use a new method that computes class for cluster , cluster/crop
			/// add an if below so it draws something in either case

			// Draw the cluster number
            projectPoint(clusterData[i].cog, prPixel, K , D, transform);
            putText(colorImage, boost::lexical_cast<string>(i), prPixel, 1, 1.4, color, 1.6, 1);
            circle(colorImage, prPixel, 4, color);

			// Draw the rectangle around the ROI
            if (clusterData[i].crop_projected == true)
            {
                if (clusterData[i].detection_label == true)
                {
                    getBox(clusterData[i].cog, prPixel, rect, params.m_to_pixels, params.body_ratio);
                    rectangle(colorImage, rect, color);
                    ss << clusterData[i].detection_fusion_prob;
                    putText(colorImage, ss.str(), Point(rect.x, rect.y), 1, 1, color, 1, 1);
                    ss.str("");
				}
			}

			// This is the code to superimpose the clusters on the image
            for (uint j = 0; j < clusterData[i].clusters.points.size(); j++)
			{
				// Convert each cluster point to image coordinates
                projectPoint(clusterData[i].clusters.points[j], prPixel, K , D, transform);

				// Draw the point to the image
				if (prPixel.x >= 0 && prPixel.x < colorImage.cols && prPixel.y >= 0 && prPixel.y < colorImage.rows)
				{
					circle(colorImage, prPixel, 2, color);
				}
			}
		}
	}
	/*
  	  ROS_INFO("[ANNOTATOR] num features %d", scanClusters.nfeatures);
  	  ROS_INFO("[ANNOTATOR] size features %d", scanClusters.features[0].data.size());
  	  ROS_INFO("[ANNOTATOR] num clusters %d", scanClusters.nclusters);
  	  ROS_INFO("[ANNOTATOR] size clusters %d", scanClusters.clusters.size());
  	  ROS_INFO("[ANNOTATOR] size features %d", scanClusters.features.size());
  	  ROS_INFO("[ANNOTATOR] size labels %d", scanClusters.labels.size());
  	  ROS_INFO("[ANNOTATOR] size fusion %d", scanClusters.fusion.size());
  	  /*/

    plotLaser(zoom);
	cv::imshow(L_WINDOW, laserPlane);
	waitKey(3);
	cv::imshow(C_WINDOW, colorImage);
	waitKey(3);

	//waitKey();
	pub.publish(dummy);
	//*/
}

/// Returns a color from the pallete, based on its position
Scalar visualizer::getColor(geometry_msgs::Point32 &point)
{
    return pallete[(int) (5 * distance(point))];
}

float visualizer::distance(geometry_msgs::Point32 point)
{
    return sqrt(point.x * point.x + point.y * point.y);
}


void visualizer::plotLaser(int zoom)
{
    laserPlane = Mat(window_size, CV_8UC3);
    laserPlane.setTo(Scalar(255, 255, 255));

    Point pt;
    Scalar color;
    // The mat will be 1000(X)x500(Y) pixels
    // The values are computed according to the zoom value
    // The default zoom corresponds to 500 pixels = 30000 mm

    for (uint cNo = 0; cNo < clusterData.size(); cNo++)
    {
        if (1) //scanClusters.fusion[clusterNo] == TRUE)
        {
//            color = getColor(clusterData[cNo].cog);
            color = colors.at(cNo % colors.size());

            for (uint pointNo = 0; pointNo < clusterData[cNo].clusters.points.size(); pointNo++)
            {
                // Transform x, y point to pixels
                pointToPlane(clusterData[cNo].clusters.points[pointNo], pt, window_size, zoom);
                circle(laserPlane, pt, 2, color);
            }

            pointToPlane(clusterData[cNo].cog, pt, window_size, zoom);

            // Number of cluster
            //cv::putText(laserPlane, boost::lexical_cast<string>(clusterNo),
            //            pt, 1, 2, color,2, 8);
            //pt.x+=30;

            // Number of points
            Scalar black(0,0,0);
//            cv::putText(laserPlane, boost::lexical_cast<string>(clusterData[cNo].clusters.points.size()),
//                    pt, 1, 2, black, 2, 8);
            pt.x -= 10;
            pt.y += 5;
            cv::putText(laserPlane, boost::lexical_cast<string>(cNo),
                        pt, 1, 0.8, black, 1.6, 8);

//            // Distance to cog
//            char buf[10] = "";
//            sprintf(buf, "%.1f", distance(clusterData[cNo].cog));
//            pt.x += 35;
//            cv::putText(laserPlane, boost::lexical_cast<string>(buf),
//                        pt, 1, 0.6, black, 1.0, 8);

            if (clusterData[cNo].label == true)
            {
                //cv::circle(laserPlane, 1 , 2, Scalar(0,0,0));
            }
        }
    }

    color.val[0] = 0;
    color.val[1] = 255;
    color.val[2] = 0;

    geometry_msgs::Point32 org;
    org.x = 0;
    org.y = 0;

    pointToPlane(org, pt, window_size, zoom);
    pt.x -= 10;
    cv::putText(laserPlane, "Laser", pt, 1, 1, color, 1, 1);
    circle(laserPlane, pt, 2, color);
}
