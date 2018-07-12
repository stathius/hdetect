#ifndef RECOGNIZERT_HPP
#define RECOGNIZERT_HPP

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/bind.hpp>

#include <hdetect/lib/recognizer.hpp>
#include <hdetect/lib/header.hpp>
#include <hdetect/ClusterClass.h>

/**
 * A node to set up all things needed for recognition.
 * Also used for the annotation.
 * The name file is given from command line.
 * @author Bang-Cheng Wang
 * @date 2013/10/01
 */

class recognizeRT
{
    public:
        recognizeRT(std::string cameraTopic, std::string laserTopic);

        ~recognizeRT();

    private:

        ros::NodeHandle nh;

        /// Subsciber to the camera image topic
        message_filters::Subscriber<sensor_msgs::Image> cameraImage_sub_;
        /// Subsciber to the laser scan topic
        message_filters::Subscriber<sensor_msgs::LaserScan>  laserScan_sub_;

        /**
        * An approximate time policy to synchronize image, camera info and laser messages.
        * This sync policy waits for all the three messages to arrive before it invokes the callback
        * from the annotator object.
        */
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> MySyncPolicy;

        /** The synchronizer based on the three messages policy */
        message_filters::Synchronizer<MySyncPolicy> *sync;

        // Declare the topics
        std::string cameraImageIn;
        std::string laserScanIn;

        /// The recognizer object that will be used for the callback
        Recognizer myRecognizer;
};

#endif
