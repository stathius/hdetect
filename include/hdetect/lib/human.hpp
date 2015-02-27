#ifndef HUMAN_HPP
#define HUMAN_HPP

#include <newmat/newmat.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>

class Human
{
    public:
        int id;

        float score;

        NEWMAT::ColumnVector state;
        NEWMAT::Matrix cov;

        NEWMAT::ColumnVector preState;
        float preTimestamp;

        // Variable for scoring process
        ros::Time firstTimestamp; // First detection time

        Human(int id, float score, NEWMAT::ColumnVector state, NEWMAT::Matrix cov, int preTimestamp);

        geometry_msgs::Point toPoint();
};

#endif // HUMAN_HPP
