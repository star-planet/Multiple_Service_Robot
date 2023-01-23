#include <stdio.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

enum Mode{
    Serving,
    Amanity
}; static Mode mode;

void fbCallback(const std_msgs::Int16 &fbData){
    if (fbData.data == 1){
        mode = Serving;
    }

    else if (fbData.data == 2){
        mode = Amanity;
    }
}

int main(int argc, char **argv){
	ROS_INFO("Mode Change Service");

	ros::init(argc, argv, "mode_change_service");
	ros::NodeHandle nh;

    ros::ros::Subscriber subFirebase = nh.subscribe("/firebase", 1, fbCallback);
	
}