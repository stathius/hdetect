#include <hdetect/lib/visualizer.hpp>

visualizer::visualizer() {

	// Random generator for the random coloring of the clusters
	RNG rng(0xFFFFFFFF);
	pallete = randomColors(rng);

	// init window and font
	cv::namedWindow(C_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(L_WINDOW, CV_WINDOW_AUTOSIZE);


	cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	zoom = INITIAL_ZOOM; // in meters

	// Trackbar to control the zoom
	createTrackbar("Zoom:", L_WINDOW, &zoom, 30, &plotLaser<visualizer>, this);

	pub = nh.advertise<std_msgs::Byte>("visualizer", 5);
	dummy.data=1;
}

visualizer::~visualizer() {
	cv::destroyWindow(C_WINDOW);
	cv::destroyWindow(L_WINDOW);
}

/**
 * It superimposes the clusters on the image along with their bounding boxes if available.
 * Also draws a 2d plane of the laser scan to facilitate the annotation process.
 */
void visualizer::visualizeData(const sensor_msgs::Image::ConstPtr &image,
		const sensor_msgs::LaserScan::ConstPtr &lScan)
{

	extractData(image, lScan);

	// Convert image to RGB
	cvtColor(cv_ptr->image, colorImage , CV_GRAY2RGB);

	// Iterate through every cog of the scanClusters
	for (int i=0; i < scanData.cogs.size() ;i++)
	{
		// If the cog is in the image save its features and then plot the cluster
		if (scanData.projected[i]==1)
		{
			// Get the color index
			color = getColor(scanData.cogs[i]);

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
			cv::putText(colorImage, boost::lexical_cast<string>(i), prPixel, 1, 1, color, 1, 1);

			// This is the code to superimpose the clusters on the image
			for (uint j = 0; j < scanData.clusters[i].points.size(); j++)
			{
				// Convert each cluster point to image coordinates
				projectPoint(scanData.clusters[i].points[j], prPixel, params.cInfo, transform);

				// Draw the point to the image
				if (prPixel.x >= 0 && prPixel.x < colorImage.cols && prPixel.y >= 0 && prPixel.y < colorImage.rows)
				{
					circle(colorImage, prPixel, 2, color);
					/*
          	  	  	  colorImage.at<Vec3b>(pt2D.y, pt2D.x)[0] = color.val[0]; //b
          	  	  	  colorImage.at<Vec3b>(pt2D.y, pt2D.x)[1] = color.val[1]; //g
           	  	  	  colorImage.at<Vec3b>(pt2D.y, pt2D.x)[2] = color.val[2]; //r
					 */
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
	 */
	plotLaser<visualizer>(zoom, this);
	cv::imshow(L_WINDOW, laserPlane);
	waitKey(3);
	cv::imshow(C_WINDOW, colorImage);
	waitKey(3);

	//waitKey();
	pub.publish(dummy);
}

/// Returns a color from the pallete, based on its position
Scalar visualizer::getColor(geometry_msgs::Point32 &point)
{
	return pallete[(int) (5 * sqrt(pow(point.x, 2) + pow(point.y, 2)) )];
}
