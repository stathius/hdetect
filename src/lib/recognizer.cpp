#include <hdetect/lib/recognizer.hpp>

using namespace std;
using namespace std_msgs;
using namespace cv;
using namespace NEWMAT;
using namespace Header;

#define RVIZ_ID_SIZE 1000

Recognizer::Recognizer()
    : it_(nh)
{
    if (nh.hasParam("object_tracking"))
    {
        string object_tracking;

        nh.getParam("object_tracking", object_tracking);

        ObjectTracking::loadCfg(object_tracking);

        ROS_INFO("[Recognizer] Object Tracking Loaded");
    }
    else
    {
        ObjectTracking::loadCfg("");

        ROS_INFO("[Recognizer] Object Tracking Using Default Value");
    }

    string odom_ekf_topic;
    string odom_topic;
    string humans_detectec_topic;

    nh.param("odom_ekf_topic", odom_ekf_topic, string("/ekf_pose/odom_combined"));
    nh.param("odom_topic", odom_topic, string("/odom"));
    nh.param("humans_detectec_topic", humans_detectec_topic, string("/HumansDetected"));

    initColor();

    ROS_INFO("[RECOGNIZER] Recognizer running OK.");

    it_pub_ = it_.advertise(imageTopic, 1);

    rviz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("markers", 1);

    output_cv_ptr.reset(new cv_bridge::CvImage);
    output_cv_ptr->encoding = "rgb8";

    with_odom = false;
    with_odom_ekf = false;

    resetId();

    curTimestamp = getTimestamp();
    preTimestamp = getTimestamp();

    odom_sub_ = nh.subscribe(odom_topic, 1, &Recognizer::setOdom, this);
    odom_ekf_sub_ = nh.subscribe(odom_ekf_topic, 1, &Recognizer::setOdomEkf, this);

    Humanpublisher = nh.advertise<hdetect::HumansFeatClass>(humans_detectec_topic,1);
}


Recognizer::~Recognizer()
{
}

void Recognizer::recognizeData(const sensor_msgs::Image::ConstPtr &image,
                               const sensor_msgs::LaserScan::ConstPtr &lScan)
{
//    curTimestamp = getTimestamp();
    static double init_time =  lScan->header.stamp.sec;
    curTimestamp = lScan->header.stamp.toSec() - init_time;

    laser_frame_id = lScan->header.frame_id;

    //getTimestamp();
    //ROS_INFO("laser %f - getTimestamp %f = %f",
    //         curTimestamp, getTimestamp(), curTimestamp -getTimestamp());
    observations.clear();
    pairs.clear();

    detector::detectHumans(image, lScan);

    // Convert image to RGB
    cvtColor(cv_ptr->image, rawImage , CV_GRAY2RGB);

    loadObservation();

    //ROS_INFO("Before: Humans = %d, Observations = %d", (int)humans.size(), (int)observations.size());

    // Eliminate Untracked Human
    ObjectTracking::eliminate(humans);

    ObjectTracking::predict(humans);

    ObjectTracking::pair(humans, observations, pairs);

    ObjectTracking::update(humans, observations, pairs);

    publish(lScan);

    //ROS_INFO("After : Humans = %d, Pairs = %d\n", (int)humans.size(), (int)pairs.size());
    //changeframe();
    //tf::Transform laser_pos;
    tf::Transform odom_pos;

    // Publish Humans detected
    for(uint i=0 ; i < humans.size() ; i++)
    {
        tf::Transform laser_pos(tf::Quaternion(), tf::Vector3(humans[i].state(1), humans[i].state(2), 0));
        if(with_odom_ekf == true)
        {
          odom_pos = cur_odom_ekf * laser_pos;
        }
        else if(with_odom == true)
        {
          odom_pos = cur_odom * laser_pos;
        }
        else
        {
          odom_pos = laser_pos;
        }

        HumansAux.id = humans[i].id;
        HumansAux.x =  odom_pos.getOrigin().getX(); // humans[i].state(1);
        HumansAux.y = odom_pos.getOrigin().getY();  // humans[i].state(2);
        HumansAux.velx = humans[i].state(3);
        HumansAux.vely = humans[i].state(4);
        HumansAux.detectiontime = humans[i].firstTimestamp.sec + (humans[i].firstTimestamp.nsec * 0.000000001);
        HumansVector.push_back(HumansAux);
    }
    geometry_msgs::Vector3Stamped pos_laser;
    geometry_msgs::Vector3Stamped pos_odom;
    HumansDetected.HumansDetected = HumansVector;
    Humanpublisher.publish(HumansDetected);
    HumansVector.clear();

    preTimestamp = curTimestamp;
}

void Recognizer::initColor()
{
    colors.push_back(Scalar(191, 0, 0));
    colors.push_back(Scalar(0, 191, 0));
    colors.push_back(Scalar(0, 0, 191));
    colors.push_back(Scalar(255, 127, 0));
    colors.push_back(Scalar(127, 255, 0));
    colors.push_back(Scalar(255, 0, 127));
    colors.push_back(Scalar(127, 0, 255));
    colors.push_back(Scalar(0, 255, 127));
    colors.push_back(Scalar(0, 127, 255));
    colors.push_back(Scalar(255, 255, 0));
    colors.push_back(Scalar(255, 0, 255));
    colors.push_back(Scalar(0, 255, 255));
}

void Recognizer::loadObservation()
{

    // Iterate through every cog of the scanClusters
    for (uint i = 0; i < clusterData.size(); i++)
    {
        float prob = clusterData[i].detection_fusion_prob;
        geometry_msgs::Point32 pos = clusterData[i].cog;

        /*
         *  If the cog is in the image AND
         *  If the rectangle is inside the image AND
         *  If the prob is more than fusion_prob
         */
        if (clusterData[i].detection_label == FUSION_HUMAN)
        {
            // Below draw image
            projectPoint(pos, prPixel, K , D, transform);
            getBox(pos, prPixel, rect, params.m_to_pixels, params.body_ratio);
            getCrop (crop, rawImage, rect);

            observations.push_back(Observation(i, prob, true, pos, crop, rect));
        }
        else if (clusterData[i].detection_label == LASER_HUMAN)
        {
            observations.push_back(Observation(i, prob, false, pos));
        }
    }

    // Human position correction with odometry
    if (with_odom_ekf == true)
    {
//        fprintf(stderr, "Pre Odom Ekf: %.2f %.2f %.2f\n"
//                , pre_odom_ekf.getOrigin().getX()
//                , pre_odom_ekf.getOrigin().getY()
//                , pre_odom_ekf.getRotation().getZ() * 180 / 3.1416);

//        fprintf(stderr, "Cur Odom Ekf: %.2f %.2f %.2f\n\n"
//                , cur_odom_ekf.getOrigin().getX()
//                , cur_odom_ekf.getOrigin().getY()
//                , cur_odom_ekf.getRotation().getZ() * 180 / 3.1416);

        for (uint i = 0; i < humans.size(); i++)
        {
            correctOdomEkf(humans[i].state);
        }

        pre_odom_ekf = cur_odom_ekf;
    }
    else if (with_odom == true)
    {
//        fprintf(stderr, "Pre Odom: %.2f %.2f %.2f\n"
//                , pre_odom.getOrigin().getX()
//                , pre_odom.getOrigin().getY()
//                , pre_odom.getRotation().getZ() * 180 / 3.1416);

//        fprintf(stderr, "Cur Odom: %.2f %.2f %.2f\n\n"
//                , cur_odom.getOrigin().getX()
//                , cur_odom.getOrigin().getY()
//                , cur_odom.getRotation().getZ() * 180 / 3.1416);

        for (uint i = 0; i < humans.size(); i++)
        {
            correctOdom(humans[i].state);
        }

        pre_odom = cur_odom;
    }
}

void Recognizer::publish(const sensor_msgs::LaserScan::ConstPtr &lScan)
{
    Mat outputImage = rawImage.clone();

    for (uint i = 0; i < observations.size(); i++)
    {
        if (observations[i].camera_detected == true)
        {
            rectangle(outputImage, observations[i].rect, getColor(humans[pairs[i]].id), 2);

            std::ostringstream ss;
            ss << "Num " << humans[pairs[i]].id << "   " << observations[i].prob;
            putText(outputImage, ss.str(), Point(observations[i].rect.x, observations[i].rect.y), 1, 1, colors[9], 1, 1);
        }
    }

    output_cv_ptr->image = outputImage;
    it_pub_.publish(output_cv_ptr->toImageMsg());

    resetId();

    // Initial all scan points
    // Convert the laserscan from ROS to library format
    geometry_msgs::Point point;

    visualization_msgs::Marker temp_human;
    visualization_msgs::Marker temp_id;
    visualization_msgs::Marker temp_line;
    visualization_msgs::Marker temp_scan;
    visualization_msgs::MarkerArray rviz_markers;

    temp_human.header.frame_id = laser_frame_id;
    temp_human.header.stamp = ros::Time();
    temp_human.type = visualization_msgs::Marker::SPHERE;
    temp_human.action = visualization_msgs::Marker::ADD;
    temp_human.lifetime = ros::Duration(0.5);


    temp_id.header.frame_id = laser_frame_id;
    temp_id.header.stamp = ros::Time();
    temp_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    temp_id.action = visualization_msgs::Marker::ADD;
    temp_id.lifetime = ros::Duration(0.5);

    temp_id.scale.x = 1.0;
    temp_id.scale.y = 1.0;
    temp_id.scale.z = 1.0;

    temp_id.color.r = 0.7;
    temp_id.color.g = 0.7;
    temp_id.color.b = 0.7;
    temp_id.color.a = 0.8;


    temp_line.header.frame_id = laser_frame_id;
    temp_line.header.stamp = ros::Time();
    temp_line.type = visualization_msgs::Marker::LINE_LIST;
    temp_line.action = visualization_msgs::Marker::ADD;
    temp_line.lifetime = ros::Duration(0.2);

    temp_line.scale.x = 0.05;
    temp_line.scale.y = 0.05;
    temp_line.scale.z = 0.05;


    temp_scan.header.frame_id = laser_frame_id;
    temp_scan.header.stamp = ros::Time();
    temp_scan.type = visualization_msgs::Marker::SPHERE;
    temp_scan.action = visualization_msgs::Marker::ADD;
    temp_scan.lifetime = ros::Duration(0.2);

    temp_scan.color.r = 1.0;
    temp_scan.color.g = 1.0;
    temp_scan.color.b = 1.0;
    temp_scan.color.a = 0.8;

    temp_scan.scale.x = 0.1;
    temp_scan.scale.y = 0.1;
    temp_scan.scale.z = 0.1;


    // Draw human circle and human id
    for (uint i = 0; i < humans.size(); i++)
    {
        temp_human.id = getId(humans[i].id, RVIZ_HUMAN);

        temp_human.pose.position = humans[i].toPoint();

        temp_human.pose.orientation.w = 1.0;
        temp_human.pose.orientation.x = 0.0;
        temp_human.pose.orientation.y = 0.0;
        temp_human.pose.orientation.z = 0.0;

        temp_human.scale.x = 2 * sqrt(humans[i].cov(1, 1)) * 2;
        temp_human.scale.y = 2 * sqrt(humans[i].cov(2, 2)) * 2;
        temp_human.scale.z = 0.5;

        temp_human.color = getColorRgba(humans[i].id, 0.5);

        temp_id.id = getId(humans[i].id, RVIZ_ID);
        temp_id.text = toString(humans[i].id);

        temp_id.pose.position = humans[i].toPoint();

        rviz_markers.markers.push_back(temp_human);
        rviz_markers.markers.push_back(temp_id);
    }


    // Draw observation line
    for (uint i = 0; i < observations.size(); i++)
    {
        temp_line.id = getNewId(RVIZ_LINE);

        temp_line.points.clear();



        // Paired observation with color line
        if (pairs.count(i) == 1)
        {
            setPoint(0.0, 0.0, 0.0, point);
            temp_line.points.push_back(point);
            setPoint(observations[i].state(1), observations[i].state(2), 0.0, point);
            temp_line.points.push_back(point);
            temp_line.color = getColorRgba(humans[pairs[i]].id, 0.7);

        }
        // False observation with white line
        else
        {
//            setPoint(0.0, 0.0, 0.0, point);
//            temp_line.points.push_back(point);
//            setPoint(observations[i].state(1), observations[i].state(2), 0.0, point);
//            temp_line.points.push_back(point);
//            temp_line.color.r = 1.0;
//            temp_line.color.g = 1.0;
//            temp_line.color.b = 1.0;
//            temp_line.color.a = 0.5;
        }

        rviz_markers.markers.push_back(temp_line);
    }

    if (rviz_markers.markers.size() > 0)
    {
        rviz_pub_.publish(rviz_markers);
    }
}


void Recognizer::setOdom(const nav_msgs::Odometry &odom)
{
    tf::Vector3 vector3(odom.pose.pose.position.x,
                        odom.pose.pose.position.y,
                        odom.pose.pose.position.z);

    tf::Quaternion quaternion(odom.pose.pose.orientation.x,
                              odom.pose.pose.orientation.y,
                              odom.pose.pose.orientation.z,
                              odom.pose.pose.orientation.w);

    cur_odom = tf::Transform(quaternion, vector3);

    if (with_odom == false)
    {
        pre_odom = cur_odom;
    }

    with_odom = true;
}

void Recognizer::setOdomEkf(const geometry_msgs::PoseWithCovarianceStamped &odom_ekf)
{
    tf::Vector3 vector3(odom_ekf.pose.pose.position.x,
                        odom_ekf.pose.pose.position.y,
                        odom_ekf.pose.pose.position.z);

    tf::Quaternion quaternion(odom_ekf.pose.pose.orientation.x,
                              odom_ekf.pose.pose.orientation.y,
                              odom_ekf.pose.pose.orientation.z,
                              odom_ekf.pose.pose.orientation.w);

    cur_odom_ekf = tf::Transform(quaternion, vector3);

    if (with_odom_ekf == false)
    {
        pre_odom_ekf = cur_odom_ekf;
    }

    with_odom_ekf = true;
}

void Recognizer::correctOdom(ColumnVector &state)
{
    tf::Transform pre_pos(tf::Quaternion(), tf::Vector3(state(1), state(2), 0));

    // Translate position from previous robot coordination to global coordination
    tf::Transform global_odom = pre_odom * pre_pos;

    // Invert position from previous robot coordination to global coordination
    tf::Transform cur_pos = cur_odom.inverse() * global_odom;

    state(1) = cur_pos.getOrigin().getX();
    state(2) = cur_pos.getOrigin().getY();
}

void Recognizer::correctOdomEkf(ColumnVector &state)
{
    tf::Transform pre_pos(tf::Quaternion(), tf::Vector3(state(1), state(2), 0));

    // Translate position from previous robot coordination to global coordination
    tf::Transform global_odom = pre_odom_ekf * pre_pos;

    // Invert position from previous robot coordination to global coordination
    tf::Transform cur_pos = cur_odom_ekf.inverse() * global_odom;

    state(1) = cur_pos.getOrigin().getX();
    state(2) = cur_pos.getOrigin().getY();
}

void Recognizer::setPoint(float x, float y, float z, geometry_msgs::Point &p)
{
    p.x = x;
    p.y = y;
    p.z = z;
}

Scalar Recognizer::getColor(int index)
{
    return colors.at(index % colors.size());
}

ColorRGBA Recognizer::getColorRgba(int index, float a)
{
    ColorRGBA colorRgba;

    Scalar color = getColor(index);

    colorRgba.r = color[0] / 255.0;
    colorRgba.g = color[1] / 255.0;
    colorRgba.b = color[2] / 255.0;
    colorRgba.a = a;

    return colorRgba;
}

float Recognizer::getTimestamp()
{
    timeval t;
    gettimeofday(&t, NULL);

    static int constSec = t.tv_sec;

    return t.tv_sec - constSec + (float)t.tv_usec / 1000000;
}

string Recognizer::toString(float num)
{
    ostringstream buf;

    buf << num;

    return buf.str();
}

void Recognizer::resetId()
{
    for (uint i = 0; i < RVIZ_TOTAL; i++)
    {
        rviz_id[i] = 0;
    }
}

uint Recognizer::getId(int index, int category)
{
    return index % RVIZ_ID_SIZE + category * RVIZ_ID_SIZE;
}

uint Recognizer::getNewId()
{
    return getNewId(RVIZ_NONE);
}

uint Recognizer::getNewId(int category)
{
    int ret = rviz_id[category];

    rviz_id[category] = (rviz_id[category] + 1) % RVIZ_ID_SIZE + category * RVIZ_ID_SIZE;

    return ret;
}
