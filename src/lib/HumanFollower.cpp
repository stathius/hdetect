#include "HumanFollower.hpp"

using namespace std;
using namespace std_msgs;
using namespace NEWMAT;
using namespace Header;

/**
 * A node to catch the position of the closest human
 * to the robot and publish it in ROS
 * Human position in the laser frame
 * It is supposed that once an Human with a determined
 * ID is gone, this ID will not appear any more
 * @author Andrés Alacid Cano
 * @date 2014/02/15
 */

HumanFollower::HumanFollower(): ac("move_base", true)
{    
    // Constructor for Recognizer
    Recognizer();

    // Frame used
    frame = "/laser_top";

    // Weight scoring variables
    k1 = 0.75;
    k2 = 1.25;
    k3 = 1.25;

    // Initialize variables
    idMaxSc = 0;
    firstData = false;
    firstFollow = false;
    message_counter = 0;
    reached = false;
    goal_sent = false;
    goal_frame_on = false;
    with_amcl = false;
    with_odom_combined = false;
    x2 = 0;
    y2 = 0;

    // Robot AMCL position subscription callback
    amcl_position = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &HumanFollower::amcl_subCallback, this);

    // Robot EKF ODOM COMBINED position subscription callback
    odom_combined_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/ekf_pose/odom_combined", 10, &HumanFollower::odom_combinedCallback, this);

    // Manual turning setting
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/summit_xl_ctrl/command", 1);

    ROS_INFO("[HUMAN_FOLLOWER] Follower running OK.");
}

void HumanFollower::update(const sensor_msgs::Image::ConstPtr &image,
                           const sensor_msgs::LaserScan::ConstPtr &lScan)
{
    recognizeData(image, lScan);

    // Fill in the code you need, you can get humans directly here
    humans_detectedCallback(humans);
}


//// SCORING AND CHOOSING OBJECTIVE
//void HumanFollower::scoringprocess()
//{

//    ros::Rate rate (1.0);
//    while (nh.ok()) {

//        // Call to function that returns deque of humans
//        //HumansIncoming = myRecognizer.HumansDetected();

//        /// DEBUGGING
//        ROS_INFO("[HUMANFOLLOWER]  Humans Deque size: %d", HumansIncoming.size());


//        if (HumansIncoming.size() != 0)
//        {
//            /// DEBUGGING
//            ROS_INFO("[HUMANFOLLOWER]  First Human Position x:%f  y:%f", HumansIncoming[0].state(1), HumansIncoming[0].state(2));

//            // Scoring Humans
//            for(uint i=0 ; i < HumansIncoming.size() ; i++)
//            {
//                // Detection time scoring. Logarithmic function
//                detectionDuration = ros::Time::now() - HumansIncoming[i].firstTimestamp;
//                aux = detectionDuration.sec + (detectionDuration.nsec / 1000000000);
//                sc1 = 36.06*log(0.1333*(aux+7.50187));

//                // Distance scoring. Lineal function
//                if(HumansIncoming[i].dist == 0) sc2 = 0;
//                else
//                {
//                    sc2 = -5.405*HumansIncoming[i].dist + 108.108;
//                }

//                // Following time scoring. Logarithmic function
//                if (HumansIncoming[i].firstFollowTimestamp.sec == 0) sc3 = 0;
//                else
//                {
//                    followingDuration = ros::Time::now() - HumansIncoming[i].firstFollowTimestamp;
//                    aux = followingDuration.sec + (followingDuration.nsec / 1000000000);
//                    sc3 = 36.06*log(0.1333*(aux+7.50187));
//                }

//                // Total scoring
//                HumansIncoming[i].scorefollower = k1 * sc1 + k2 * sc2 + k3 * sc3;

//                // Take maximum score ID
//                if (idMaxSc == 0) idMaxSc = HumansIncoming[i].id;
//                else
//                {
//                    if (HumansIncoming[i].id > HumansIncoming[idMaxSc].id) idMaxSc = HumansIncoming[i].id;
//                }
//            }

//            for(uint i=0 ; i < HumansIncoming.size() ; i++)
//            {
//                if (HumansIncoming[i].id == idMaxSc)
//                {
//                    // Set follow process starting time
//                    if (HumansIncoming[i].firstFollowTimestamp.sec == 0 && HumansIncoming[i].firstFollowTimestamp.nsec ==0)
//                    {
//                        HumansIncoming[i].firstFollowTimestamp = ros::Time::now();
//                    }
//                    obj_position.point.x = HumansIncoming[i].state(1);
//                    obj_position.point.y = HumansIncoming[i].state(2);
//                    obj_position.header.seq = HumansIncoming[i].id;
//                    obj_position.header.stamp = ros::Time::now();
//                    obj_position.header.frame_id = frame;
//                }
//                else
//                {
//                    // Set to zero every Human it is not being followed
//                    HumansIncoming[i].firstFollowTimestamp.sec = 0;
//                    HumansIncoming[i].firstFollowTimestamp.nsec = 0;
//                }
//            }

//            // Reset ID maximum score
//            idMaxSc = 0;

//            // Objective position initialization with first data acquisition
//            if (!firstData) {
//                x2 = px;
//                y2 = py;
//                firstData = true;
//            }

//            decisionfollower();

//        }

//        rate.sleep();
//        ros::spinOnce();
//    }
//}


// HUMANS DETECTED CALLBACK AND SCORING PROCESS
void HumanFollower::humans_detectedCallback(HumanDeque &humans_detected)
{
    if (humans_detected.size() != 0)
    {
        /// DEBUGGING
        //ROS_INFO("[HUMANFOLLOWER]  First Human Position x:%f  y:%f", HumansIncoming[0].state(1), HumansIncoming[0].state(2));

        // Scoring Humans
        for(uint i=0 ; i < humans_detected.size() ; i++)
        {
            // Detection time scoring. Logarithmic function
            detectionDuration = ros::Time::now() - humans_detected[i].firstTimestamp;
            aux = detectionDuration.sec + (detectionDuration.nsec / 1000000000);
            sc1 = 36.06*log(0.1333*(aux+7.50187));

            // Distance scoring. Lineal function
            if(humans_detected[i].dist == 0) sc2 = 0;
            else
            {
                sc2 = -5.405*humans_detected[i].dist + 108.108;
            }

//            // Following time scoring. Logarithmic function
//            if (humans_detected[i].firstFollowTimestamp.sec == 0) sc3 = 0;
//            else
//            {
//                followingDuration = ros::Time::now() - humans_detected[i].firstFollowTimestamp;
//                aux = followingDuration.sec + (followingDuration.nsec / 1000000000);
//                sc3 = 36.06*log(0.1333*(aux+7.50187));
//            }

            // Total scoring
            humans_detected[i].scorefollower = k1 * sc1 + k2 * sc2;
            //humans_detected[i].scorefollower = k1 * sc1 + k2 * sc2 + k3 * sc3;

            // Take maximum score ID
            if (idMaxSc == 0) idMaxSc = humans_detected[i].id;
            else
            {
                if (humans_detected[i].id > humans_detected[idMaxSc].id) idMaxSc = humans_detected[i].id;
            }
        }

        for(uint i=0 ; i < humans_detected.size() ; i++)
        {
            if (humans_detected[i].id == idMaxSc)
            {
//                // Set follow process starting time
//                if (humans_detected[i].firstFollowTimestamp.sec == 0 && humans_detected[i].firstFollowTimestamp.nsec ==0)
//                {
//                    humans_detected[i].firstFollowTimestamp = ros::Time::now();
//                }
                obj_position.point.x = humans_detected[i].state(1);
                obj_position.point.y = humans_detected[i].state(2);
                obj_position.header.seq = humans_detected[i].id;
                obj_position.header.stamp = ros::Time::now();
                obj_position.header.frame_id = frame;

                // Variable change to make it easier
                px = obj_position.point.x;
                py = obj_position.point.y;
            }
//            else
//            {
//                // Set to zero every Human it is not being followed
//                humans_detected[i].firstFollowTimestamp.sec = 0;
//                humans_detected[i].firstFollowTimestamp.nsec = 0;
//            }
        }

        // Reset ID maximum score
        idMaxSc = 0;

        // Objective position initialization with first data acquisition
        if (!firstData) {
            x2 = px;
            y2 = py;
            firstData = true;
        }

        //decisionfollower();

    }
}


// ROBOT POSITION CALLBACK AMCL
void HumanFollower::amcl_subCallback(const geometry_msgs::PoseWithCovarianceStamped amcl_robot1_position)
{

    with_amcl = true;

    x1amcl = amcl_robot1_position.pose.pose.position.x;
    y1amcl = amcl_robot1_position.pose.pose.position.y;
    z1amcl = amcl_robot1_position.pose.pose.orientation.z;
    w1amcl = amcl_robot1_position.pose.pose.orientation.w;

    yaw_robotamcl = tf::getYaw(amcl_robot1_position.pose.pose.orientation);
    while(yaw_robotamcl < 0) yaw_robotamcl += 2*M_PI;
    while(yaw_robotamcl > 2*M_PI) yaw_robotamcl -= 2*M_PI;
}


// ROBOT POSITION CALLBACK ODOM_COMBINED
void HumanFollower::odom_combinedCallback(const geometry_msgs::PoseWithCovarianceStamped odom_combined_robot)
{

    with_odom_combined = true;

    x1odom = odom_combined_robot.pose.pose.position.x;
    y1odom = odom_combined_robot.pose.pose.position.y;
    z1odom = odom_combined_robot.pose.pose.orientation.z;
    w1odom = odom_combined_robot.pose.pose.orientation.w;

    yaw_robotodom = tf::getYaw(odom_combined_robot.pose.pose.orientation);
    while(yaw_robotodom < 0) yaw_robotodom += 2*M_PI;
    while(yaw_robotodom > 2*M_PI) yaw_robotodom -= 2*M_PI;


}


// DECISION-MAKING FUNCTION
void HumanFollower::decisionfollower()
{

    ros::Rate rate(1.0);
    while (ros::ok() && firstData) {

        // Robot position from appropiate frame
        if (with_amcl)
        {
            x1 = x1amcl;
            y1 = y1amcl;
            z1 = z1amcl;
            w1 = w1amcl;
            yaw_robot = yaw_robotamcl;
        }
        else if(with_odom_combined)
        {
            x1 = x1odom;
            y1 = y1odom;
            z1 = z1odom;
            w1 = w1odom;
            yaw_robot = yaw_robotodom;
        }

        // Objective distance from robot in axes
        xdis = px - x1;
        ydis = py - y1;

        // Objective distance from robot
        obj_dist = sqrt(xdis*xdis + ydis*ydis);

        // Objective movement
        obj_mov = sqrt((x2 - px)*(x2 - px) + (y2 - py)*(y2 - py));

                    // Debugging code
                //    if (message_counter2 == 0){
                //        ROS_INFO("  [DECISION MAKING] OBJ DIST = %f   OBJ MOV = %f", obj_dist, obj_mov);
                //    }
                //    if (message_counter2 == 10) message_counter2 = 0;
                //    message_counter2++;

        // First values assignment and following or framing start
        if (!firstFollow){
            //ROS_INFO("POSITION X:%f  Y:%f",px, py);
            x2 = px;
            y2 = py;
            if (obj_dist > 2.5) {
                ROS_INFO("  [DECISION MAKING] Initial goal_follower calling\n\n");
                goal_follower();
            }
            else {
                ROS_INFO("  [DECISION MAKING] Initial goal_frame calling\n\n");
                goal_frame();
            }
            firstFollow = true;
        }

        // Everytime objective moves one meter around the framing or the goal must be updated
        if(obj_mov > 1) {
            ROS_INFO("  [DECISION MAKING] Objective moved one meter around. Distance to objective %f. Function called is:", obj_dist);
            goal_frame_on = false;
            reached = false;
            if (obj_dist > 2.5) {
                ROS_INFO("  [DECISION MAKING] goal_follower.\n\n");
                goal_follower();

            }
            else {
                ROS_INFO("  [DECISION MAKING] goal_frame.\n\n");
                // Robot position updating
                // X2 and Y2 will be the real goal to follow, unless the robot moves one meter around again
                x2 = px;
                y2 = py;
                goal_frame();
            }
        }
        else if (obj_dist < 2.5 && goal_sent) {
            // Robot came into framing zone
            ROS_INFO("    [DECISION MAKING] Goal frame function starts");
            goal_frame();
        }
        // Frame funcition calling successively
        else if(goal_frame_on && !reached) {
                rate.sleep();
                goal_frame();
        }

            ros::spinOnce();
    }

}


// FOLLOWING FUNCTION
void HumanFollower::goal_follower()
{

    x2 = px;
    y2 = py;

    // THIS IS WITH THE RELATIVE OBJECTIVE POSITION
    //theta = atan2(ydis,xdis);

    //ROS_INFO("OBJECTIVE POSITION X:%f Y:%f", x2, y2);

    thetaObj = atan2(ydis,xdis);
    while(thetaObj < 0) thetaObj += 2*M_PI;
    while(thetaObj > 2*M_PI) thetaObj -= 2*M_PI;

    // Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("  [GOAL FOLLOWER] Waiting for the move_base action server to come up\n\n");
    }

    // Goal position one meter and a half before the objective, in a line that comprises the objective and the robot
    xgoal = - ((3*x2-3*x1) * sqrt (y2*y2 - 2*y1*y2 + y1*y1 + x2*x2 - 2*x1*x2 + x1*x1) + 2*(- x2*y2*y2 + 2*x2*y1*y2 - x2*y1*y1 - x2*x2*x2 + 2*x1*x2*x2 - x1*x1*x2))  /  (2*(y2*y2 - 2*y1*y2 + y1*y1 + x2*x2 - 2*x1*x2 + x1*x1));
    ygoal = - ((3*y2-3*y1) * sqrt (y2*y2 - 2*y1*y2 + y1*y1 + x2*x2 - 2*x1*x2 + x1*x1) + 2*(- y2*y2*y2 + 2*y1*y2*y2 + (- y1*y1 - x2*x2 + 2*x1*x2 - x1*x1) * y2))  /  (2*(y2*y2 - 2*y1*y2 + y1*y1 + x2*x2 - 2*x1*x2 + x1*x1));

    // INDOOR
    // Goal position half a meter before the objective, in a line that comprises the objective and the robot
    //x = - ((sqrt(2)*x2-sqrt(2)*x1) * sqrt (y2*y2 - 2*y1*y2 + y1*y1 + x2*x2 - 2*x1*x2 + x1*x1) + 2*(- x2*y2*y2 + 2*x2*y1*y2 - x2*y1*y1 - x2*x2*x2 + 2*x1*x2*x2 - x1*x1*x2))  /  (2*(y2*y2 - 2*y1*y2 + y1*y1 + x2*x2 - 2*x1*x2 + x1*x1));
    //y = - ((sqrt(2)*y2-sqrt(2)*y1) * sqrt (y2*y2 - 2*y1*y2 + y1*y1 + x2*x2 - 2*x1*x2 + x1*x1) + 2*(- y2*y2*y2 + 2*y1*y2*y2 + (- y1*y1 - x2*x2 + 2*x1*x2 - x1*x1) * y2))  /  (2*(y2*y2 - 2*y1*y2 + y1*y1 + x2*x2 - 2*x1*x2 + x1*x1));


    // Goal value assignment
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = frame;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = xgoal;
    goal.target_pose.pose.position.y = ygoal;
    goal.target_pose.pose.orientation.z = sin(thetaObj/2);
    goal.target_pose.pose.orientation.w = cos(thetaObj/2);
    ROS_INFO("  [GOAL FOLLOWER] New goal: \n X: %f    Y: %f\n\n", xgoal, ygoal);
    ac.sendGoal(goal);
    goal_sent = true;

}


// FRAMING FUNCTION
// Turn speed is determined and the angle turned is checked out in order to find out when the final objective is framed
void HumanFollower::goal_frame()
{

    // First time it entered into the framing function
    if (!goal_frame_on) {

        // Boolean to indicate framing process
        goal_frame_on = true;

        // Starting Yaw
        yaw_first = yaw_robot;

        // Angle between the robot position and the objective
        thetaObj = atan2(ydis,xdis);
        while(thetaObj < 0) thetaObj += 2*M_PI;
        while(thetaObj > 2*M_PI) thetaObj -= 2*M_PI;

        // Turn angle determined by the difference between robot angle and robot-objective angle
        radians = yaw_robot - thetaObj;

        // Check out of the angle, regarding that it is not bigger than 2*PI neither negative
        while(radians < 0) radians += 2*M_PI;
        while(radians > 2*M_PI) radians -= 2*M_PI;
        if (radians > 0 && radians < M_PI) {
            clockwise = true;
            radiansPI = radians;
            ROS_INFO("  Clockwise turn\n");
        }
        else {
            radiansPI = 2*M_PI - radians;
            clockwise = false;
            ROS_INFO("  Counterclockwise turn\n");
        }

        // Turn speed determination in rad/s. It will be proportional to the turn angle amount
        base_cmd.linear.x = base_cmd.linear.y = 0.0;
        base_cmd.angular.z = (1.5/M_PI) * radiansPI + 0.5;	// Radians is the turn angle between 0 and 180 degrees
        // Desired turn axis
        tf::Vector3 desired_turn_axis(0,0,1);

        // Turn direction
        if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;

        // If it is already close to perfect framing (+- 5 degrees), not to do framing function
        if (radiansPI < 0.09) {
            reached = true;
            goal_frame_on = false; // Framing is over
            base_cmd.angular.z = 0;
            ROS_INFO("  Framing process succeeded\n\n");
        }

        ROS_INFO("  Turn to do: %f\n", radians);

    }

    // Unless it has entered in the framing function first, cancel the current goal
    if (goal_sent) {
        ac.cancelGoal();
        ROS_INFO("  Goal cancelled\n");
        if(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {  // ERROR. It does not do the "if". Status goes from active state to state 6, and instantly goes to state 2 (PREEMPTED)
            //ROS_INFO("  Goal cancelled\n\n");
            //goal_sent = false;
        }
        goal_sent = false;
    }


    // Turn angle determined by the difference between robot angle and robot-objective angle
    thetaObj = atan2(ydis,xdis);
    while(thetaObj < 0) thetaObj += 2*M_PI;
    while(thetaObj > 2*M_PI) thetaObj -= 2*M_PI;

    // Turn angle determined by the difference between robot angle and robot-objective angle
    radians = yaw_robot - thetaObj;

    // Check out of the angle, regarding that it is not bigger than 2*PI neither negative
    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;
    if (radians < M_PI)	radiansPI = radians;
    else radiansPI = 2*M_PI - radians;

    // Algorithm to show regularly the status on the screen
    if (message_counter == 0) ROS_INFO("  Radian angle to turn: %f\n", radiansPI);
    message_counter ++;
    if (message_counter == 1) message_counter = 0;

    // Check out if framing has arrived to objective
    if (clockwise && (radians > M_PI)){
        reached = true;
        goal_frame_on = false; // Framing is over
        // Manual turning stop
        base_cmd.angular.z = 0;
        cmd_vel_pub_.publish(base_cmd);
        ROS_INFO("  Encuadre realizado\n\n");
    }
    else if (!clockwise && (radians < M_PI)){
        reached = true;
        goal_frame_on = false; // Framing is over
        // Manual turning stop
        base_cmd.angular.z = 0;
        cmd_vel_pub_.publish(base_cmd);
        ROS_INFO("  Framing process succeeded\n\n");
    }
    else {
        // Speed and direction turn determination
        base_cmd.angular.z = (1.5/M_PI) * radiansPI + 0.5;
        if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
        cmd_vel_pub_.publish(base_cmd);
    }

}
