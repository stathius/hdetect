#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


//void poseCallback(const ???camera pose??::PoseConstPtr& msg){
//}

int main(int argc, char** argv){
  ros::init(argc, argv, "upm_tf_broadcaster");

  ros::NodeHandle node;
  
  //for using later when the camera will pan and/or tilt
  //ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  static tf::TransformBroadcaster br;

	  ros::Rate rate(30.0);
	  tf::Transform world2camera_tf ;
 	   tf::Transform world2laser_tf;;
  // camera 2 laser transform. it is 10 cm above the camera and its static so it goes out of while loop
  
// we can use something like the following in the launch file but we choose this instead
  // for future use, when the camera moves
	//<node pkg="tf" type="static_transform_publisher" name="camera_to_laser"
    	//args="0.0 0.0 0.0 0.0 0.0 0.0 /camera /laser 40" />

  world2laser_tf.setOrigin( tf::Vector3( 0.0 , 0.0 , 0.0) );
  world2laser_tf.setRotation( tf::Quaternion(0, 0, 0) );
  
  while(ros::ok()) {
  //world 2 camera transform. it is 0 0 0 for the beginning

  world2camera_tf.setOrigin( tf::Vector3(0.0, 0.0 , 0.0) );
  world2camera_tf.setRotation( tf::Quaternion(0, 0, 0) );
  br.sendTransform(tf::StampedTransform(world2camera_tf, ros::Time::now(), "world", "camera"));


  br.sendTransform(tf::StampedTransform(world2laser_tf, ros::Time::now(), "world", "laser"));
 	rate.sleep();
  ros::spinOnce();
  }
  return 0;
};
