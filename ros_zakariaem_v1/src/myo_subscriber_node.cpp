#include "ros/ros.h"
#include "std_msgs/String.h"
#include "EmgArray.h"
#include "MyoArm.h"

// callback which displays the EMG data
void chatterCallback(const ros_zakariaem::EmgArrayConstPtr& myo_msg )
{ for (int i=0;i<myo_msg->data.size();i++)
  {
  ROS_INFO("sensor number [%i] = [%i]",i+1, myo_msg->data[i]);
  }
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "myo_subscriber_node");

 ros::NodeHandle n;

ros::Subscriber submyo = n.subscribe("myo_raw/myo_emg", 1000, chatterCallback);

 ros::spin();

  return 0;
}   
