#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <fstream>
#include <string>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "leapmsg.h"
#include "Imu.h"
#include "EmgArray.h"
#include "JointCoordinates.h"
#include "rosbag/bag.h"
#include "hand_estimation.h"

using namespace ros_zakariam;
using namespace message_filters;
using namespace Hand_model;

rosbag::Bag bag;	
Hand_estimation hand;
ofstream targetFile("target");
ofstream inputFile("input");

// example to synchronize the leap motion, emg signals and imu signal from myo armband
//void callback(const ros_zakariaem::leaprosConstPtr& leap_msg, const ros_eehmd::EmgArrayConstPtr& myo_msg, const ros_zakariaem::ImuConstPtr& imu_msg)

void callback(const ros_zakariaem::leapmsgConstPtr& leap_msg, const ros_zakariaem::EmgArrayConstPtr& myo_msg)
{
  if (leap_msg->hands_count > 0){
    bool flag = true;
    hand.compute_model_from_leap_msg(*leap_msg);
    ros_zakariaem::JointCoordinates joint_coordinates;
    joint_coordinates.data.insert(joint_coordinates.data.begin(), hand.joint_coordinates, hand.joint_coordinates + 18);
    for (auto& value : joint_coordinates.data){
        if (isnan(value)){
            flag=false;
        }
    }
    if (flag){
        for (auto& col : myo_msg->data) {
            inputFile << col << ',';
        }
        for (auto& col : joint_coordinates.data) {
            targetFile << col << ',';
        }
    }
    else if (!flag){
        inputFile << "error,,,,,,,,";
        targetFile << "error,,,,,,,,,,,,,,,,,,";
    }
  }
  inputFile << '\n';
  targetFile << '\n';
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "ros_to_csv");
 ros::NodeHandle n;
//load EMG topic
message_filters::Subscriber<EmgArray> submyo(n,"myo_raw/myo_emg", 100);
//load Leap Motion topic
message_filters::Subscriber<leapmsg> subleap(n,"Leapmotion_raw", 100);

//for the imu signal from myo armband
//message_filters::Subscriber<Imu> subimu(n,"myo_raw/myo_imu", 1);

// ****************Creation of the ROS synchronizer*****************************
typedef sync_policies::ApproximateTime<leapmsg, EmgArray> MySyncPolicy;
// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
// example to synchronize the leap motion, emg signals and imu signal from myo armband
//typedef sync_policies::ApproximateTime<leapros, EmgArray, Imu> MySyncPolicy;

Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000),subleap ,submyo);
// example to synchronize the leap motion, emg signals and imu signal from myo armband
//Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000),subleap ,submyo, subimu);

sync.registerCallback(boost::bind(&callback, _1, _2));
// example to synchronize the leap motion, emg signals and imu signal from myo armband
//sync.registerCallback(boost::bind(&callback, _1, _2, _3));


n.setParam("recording",false);
n.setParam("file_name","default_name");

ROS_INFO("set the parameter \"recording\" to true to write csv");
ROS_INFO("set the output file name in the \"file_name\" parameter ");



while(ros::ok())
{
        bool recording_flag;
        n.getParam("recording",recording_flag);
        if (recording_flag==true)
        {
                std::string file_name;
                n.getParam("file_name",file_name);
                targetFile.close();
                inputFile.close();
                targetFile.open(file_name + "_target.csv");
                inputFile.open(file_name + "_input.csv");
                ROS_INFO("recording");
                while (recording_flag)
                {
                        n.getParam("recording",recording_flag);
                        ros::spinOnce();
                }
                ros::Duration(1).sleep(); // sleep for a second
				ROS_INFO("end of record");
				targetFile.close();
				inputFile.close();
				ROS_INFO("files closed");
        }
}
ros::Duration(1).sleep();

return 0;
}   
