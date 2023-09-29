#include "ros/ros.h"
#include "std_msgs/String.h"
#include "leapmsg.h"
#include "JointCoordinates.h"
#include "hand_estimation.h"

// This code subscribes the leap motion topic and publishes the corresponding joint coordinates

Hand_model::Hand_estimation hand; //the hand model is described in "hand_estimation.h"
ros_zakariaem::JointCoordinates joint_coordinates;


// This callback computes the joint coordinates from the leap motion message and and update the joint_coordinates variable
void chatterCallback(const ros_zakariaem::leapmsgConstPtr& leap_msg )
{
  if (leap_msg->hands_count > 0){

  hand.compute_model_from_leap_msg(*leap_msg);
  ros_zakariaem::JointCoordinates joint_coordinates_temp;
  joint_coordinates_temp.data.insert(joint_coordinates_temp.data.begin(), hand.joint_coordinates, hand.joint_coordinates + 18);
  joint_coordinates_temp.header = leap_msg->header;
  joint_coordinates=joint_coordinates_temp;
  }
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "joint_coordinates_publisher");

 ros::NodeHandle n;

 ros::Subscriber subleap = n.subscribe("Leapmotion_raw", 1000, chatterCallback);
 ros::Publisher joint_coordinate_publisher = n.advertise<ros_zakariaem::JointCoordinates>("joint_coordinates_from_leap", 1000);

 ros::Rate loop_rate(100);

 while (ros::ok()){

	 joint_coordinate_publisher.publish(joint_coordinates);

	 ros::spinOnce();
	 loop_rate.sleep();
 }	
  return 0;
}   
