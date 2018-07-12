#ifndef HUMANPOSE_HPP
#define HUMANPOSE_HPP

// STANDARD
#include <stdio.h>
#include <string>
#include <sstream>
#include <deque>
#include <map>
#include <newmat/newmat.h>
#include "cmath"

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

// Project includes
#include <hdetect/lib/human.hpp>
#include <hdetect/lib/header.hpp>
#include <hdetect/lib/recognizer.hpp>


/**
 * A node to catch the position of the closest human
 * to the robot and publish it in ROS
 * Human position in the laser frame
 * It is supposed that once an Human with a determined
 * ID is gone, this ID will not appear any more
 * @author Andrés Alacid Cano
 * @date 2014/02/15
 */

class HumanFollower : public Recognizer
{
public:
    HumanFollower();
    void scoringprocess();
    void decisionfollower();
    void goal_follower();
    void goal_frame();
    //void getHumans(deque<Human> &humans, bool odom, bool odom_combined);

    void update(const sensor_msgs::Image::ConstPtr &image,
                const sensor_msgs::LaserScan::ConstPtr &lScan);

private:

    ros::NodeHandle nh;

    typedef std::deque<Human> HumanDeque;

    // Amcl_pose subscription
    void amcl_subCallback(const geometry_msgs::PoseWithCovarianceStamped amcl_robot1_position);
    ros::Subscriber amcl_position;
    float x1amcl, y1amcl, w1amcl, z1amcl, yaw_robotamcl;

    // Odom_combined subscription
    void odom_combinedCallback (const geometry_msgs::PoseWithCovarianceStamped odom_combined_robot);
    ros::Subscriber odom_combined_sub;
    float x1odom, y1odom, w1odom, z1odom, yaw_robotodom;

    // Humans detected subscription
    void humans_detectedCallback (HumanDeque &humans_detected);
    geometry_msgs::PointStamped obj_position;

    // Variables
    bool firstData, with_odom_combined, with_amcl, firstFollow, reached, goal_sent, goal_frame_on, clockwise;  // General variables
    float px, py, x2, y2, xgoal, ygoal;  // New objective position and goal position
    float x1, y1, z1, w1, yaw_robot, yaw_first;  // Robot position
    float xdis, ydis, obj_dist, obj_mov, thetaObj;  // Objective position and movement
    float radians, radiansPI; // Angle difference
    int message_counter;  // Message counters

    //std::deque<geometry_msgs::PointStamped> HumansDQ;
    std::deque<Human> HumansIncoming;

    // Scoring variables
    float sc1, sc2, sc3;
    float k1, k2, k3;
    ros::Duration detectionDuration;
    ros::Duration followingDuration;
    float aux;
    int idMaxSc;

    // Goal publication variables
    geometry_msgs::Twist base_cmd;
    ros::Publisher cmd_vel_pub_;
    // Creation of a convenience typedef for a SimpleActionClient to allow us to communicate with actions
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    // Action Client construction
    MoveBaseClient ac;

    string frame;
};


#endif
