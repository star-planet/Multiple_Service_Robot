#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <thread>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h" 

using namespace std;

class Master
{
public:
Master()
{
    fnInitParam();

    pubPoseStamped = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    sub_arrival_status = nh.subscribe("/move_base/result", 1, &Master::CheckArrival, this);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ReceiveAppMsg();

        ros::spinOnce();

        loop_rate.sleep();
    }
}

