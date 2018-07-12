#include <hdetect/lib/human.hpp>


//using namespace NEWMAT;
using NEWMAT::Matrix;
using NEWMAT::ColumnVector;

using geometry_msgs::Point;

Human::Human(int id, float score, ColumnVector state, Matrix cov, int preTimestamp)
{
    this->id = id;

    this->score = score;

    this->state = state;
    this->cov = cov;

    this->preState = state;
    this->preTimestamp = preTimestamp;
    this->firstTimestamp = ros::Time::now();
}

Point Human::toPoint()
{
    Point pos;

    pos.x = state(1);
    pos.y = state(2);
    pos.z = 0;

    return pos;
}
