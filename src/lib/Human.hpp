#ifndef HUMAN_HPP
#define HUMAN_HPP

#include "ros/ros.h"
#include <newmat/newmat.h>
#include <geometry_msgs/Point.h>

class Human
{
    public:
        int id;

        float score;
        float scorefollower;

        NEWMAT::ColumnVector state;
        NEWMAT::Matrix cov;

        NEWMAT::ColumnVector preState;
        float preTimestamp;

        // This four variables are used by the following process
        ros::Time firstTimestamp; // First detection time
        ros::Time firstFollowTimestamp; // First following process time
        float dist; // Distance to the robot

        Human(int id, float score, NEWMAT::ColumnVector state, NEWMAT::Matrix cov, int preTimestamp);

        geometry_msgs::Point toPoint();
};

#endif // HUMAN_HPP
