#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionResult.h">

using namespace std;

class Robot {
public:
Robot() {
    fnInitParam();

    pubPoseStamped = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    pubFirebase = nh.advertise<std_msgs::String>("firebase", 1);
    
    subFirebase = nh.subscribe("firebase", 1, &Master::checkFirebaseDatabase, this);
}

typedef struct FLAG_VALUE {
    // 서빙
    bool bflag_ServeHome = false;
    bool bflag_ServeStart = false;
    bool bflag_ServeComebackHome = false;

    // 호텔
    bool bflag_HotelHome = false;
    bool bflag_HotelStart = false;
    bool bflag_HotelComebackHome = false;
}FLAG_VALUE;
FLAG_VALUE _Flag_Value;

void fnInitParam() {
    // Hotel Home
    nh.getParam("homePose/position", target_pose_position);
    nh.getParam("homePose/orientation", target_pose_orientation);

    poseStampedHome[0].header.frame_id = "map";
    poseStampedHome[0].header.stamp = ros::Time::now();

    poseStampedHome[0].pose.position.x = target_pose_position[0];
    poseStampedHome[0].pose.position.y = target_pose_position[1];
    poseStampedHome[0].pose.position.z = target_pose_position[2];

    poseStampedHome[0].pose.orientation.x = target_pose_orientation[0];
    poseStampedHome[0].pose.orientation.y = target_pose_orientation[1];
    poseStampedHome[0].pose.orientation.z = target_pose_orientation[2];
    poseStampedHome[0].pose.orientation.w = target_pose_orientation[3];

    // Serve Home
    nh.getParam("homePose/position", target_pose_position);
    nh.getParam("homePose/orientation", target_pose_orientation);

    poseStampedHome[1].header.frame_id = "map";
    poseStampedHome[1].header.stamp = ros::Time::now();

    poseStampedHome[1].pose.position.x = target_pose_position[0];
    poseStampedHome[1].pose.position.y = target_pose_position[1];
    poseStampedHome[1].pose.position.z = target_pose_position[2];

    poseStampedHome[1].pose.orientation.x = target_pose_orientation[0];
    poseStampedHome[1].pose.orientation.y = target_pose_orientation[1];
    poseStampedHome[1].pose.orientation.z = target_pose_orientation[2];
    poseStampedHome[1].pose.orientation.w = target_pose_orientation[3];

    // Table 1
    nh.getParam("tablePose1/position", target_pose_position);
    nh.getParam("tablePose1/orientation", target_pose_orientation);

    poseStampedTable[1].header.frame_id = "map";
    poseStampedTable[1].header.stamp = ros::Time::now();

    poseStampedTable[1].pose.position.x = target_pose_position[0];
    poseStampedTable[1].pose.position.y = target_pose_position[1];
    poseStampedTable[1].pose.position.z = target_pose_position[2];

    poseStampedTable[1].pose.orientation.x = target_pose_orientation[0];
    poseStampedTable[1].pose.orientation.y = target_pose_orientation[1];
    poseStampedTable[1].pose.orientation.z = target_pose_orientation[2];
    poseStampedTable[1].pose.orientation.w = target_pose_orientation[3];

    // Table 2
    nh.getParam("tablePose1/position", target_pose_position);
    nh.getParam("tablePose1/orientation", target_pose_orientation);

    poseStampedTable[2].header.frame_id = "map";
    poseStampedTable[2].header.stamp = ros::Time::now();

    poseStampedTable[2].pose.position.x = target_pose_position[0];
    poseStampedTable[2].pose.position.y = target_pose_position[1];
    poseStampedTable[2].pose.position.z = target_pose_position[2];

    poseStampedTable[2].pose.orientation.x = target_pose_orientation[0];
    poseStampedTable[2].pose.orientation.y = target_pose_orientation[1];
    poseStampedTable[2].pose.orientation.z = target_pose_orientation[2];
    poseStampedTable[2].pose.orientation.w = target_pose_orientation[3];

    // Table 3
    nh.getParam("tablePose1/position", target_pose_position);
    nh.getParam("tablePose1/orientation", target_pose_orientation);

    poseStampedTable[3].header.frame_id = "map";
    poseStampedTable[3].header.stamp = ros::Time::now();

    poseStampedTable[3].pose.position.x = target_pose_position[0];
    poseStampedTable[3].pose.position.y = target_pose_position[1];
    poseStampedTable[3].pose.position.z = target_pose_position[2];

    poseStampedTable[3].pose.orientation.x = target_pose_orientation[0];
    poseStampedTable[3].pose.orientation.y = target_pose_orientation[1];
    poseStampedTable[3].pose.orientation.z = target_pose_orientation[2];
    poseStampedTable[3].pose.orientation.w = target_pose_orientation[3];

    // hotel 101
    nh.getParam("hotelPose1/position", target_pose_position);
    nh.getParam("hotelPose1/orientation", target_pose_orientation);

    poseStampedHotel[1].header.frame_id = "map";
    poseStampedHotel[1].header.stamp = ros::Time::now();

    poseStampedHotel[1].pose.position.x = target_pose_position[0];
    poseStampedHotel[1].pose.position.y = target_pose_position[1];
    poseStampedHotel[1].pose.position.z = target_pose_position[2];

    poseStampedHotel[1].pose.orientation.x = target_pose_orientation[0];
    poseStampedHotel[1].pose.orientation.y = target_pose_orientation[1];
    poseStampedHotel[1].pose.orientation.z = target_pose_orientation[2];
    poseStampedHotel[1].pose.orientation.w = target_pose_orientation[3];

    // hotel 102
    nh.getParam("hotelPose2/position", target_pose_position);
    nh.getParam("hotelPose2/orientation", target_pose_orientation);

    poseStampedHotel[2].header.frame_id = "map";
    poseStampedHotel[2].header.stamp = ros::Time::now();

    poseStampedHotel[2].pose.position.x = target_pose_position[0];
    poseStampedHotel[2].pose.position.y = target_pose_position[1];
    poseStampedHotel[2].pose.position.z = target_pose_position[2];

    poseStampedHotel[2].pose.orientation.x = target_pose_orientation[0];
    poseStampedHotel[2].pose.orientation.y = target_pose_orientation[1];
    poseStampedHotel[2].pose.orientation.z = target_pose_orientation[2];
    poseStampedHotel[2].pose.orientation.w = target_pose_orientation[3];

    // hotel 103
    nh.getParam("hotelPose3/position", target_pose_position);
    nh.getParam("hotelPose3/orientation", target_pose_orientation);

    poseStampedHotel[3].header.frame_id = "map";
    poseStampedHotel[3].header.stamp = ros::Time::now();

    poseStampedHotel[3].pose.position.x = target_pose_position[0];
    poseStampedHotel[3].pose.position.y = target_pose_position[1];
    poseStampedHotel[3].pose.position.z = target_pose_position[2];

    poseStampedHotel[3].pose.orientation.x = target_pose_orientation[0];
    poseStampedHotel[3].pose.orientation.y = target_pose_orientation[1];
    poseStampedHotel[3].pose.orientation.z = target_pose_orientation[2];
    poseStampedHotel[3].pose.orientation.w = target_pose_orientation[3];
}

void checkFirebaseDatabase(const std_msgs::String database) {
    if (database.data == "table1") { // 테이블1 출발
        pubPoseStamped.publish(poseStampedTable[1]);
        ROS_INFO("테이블1 출발\n");

        _Flag_Value.bflag_ServeStart = true;
    }

    if (database.data == "table2") { // 테이블2 출발
        pubPoseStamped.publish(poseStampedTable[2]);
        ROS_INFO("테이블2 출발\n");

        _Flag_Value.bflag_ServeStart = true;
    }

    if (database.data == "table3") { // 테이블3 출발
        pubPoseStamped.publish(poseStampedTable[3]);
        ROS_INFO("테이블3 출발\n");

        _Flag_Value.bflag_ServeStart = true;
    }

    if (database.data == "hotel101") { // 101호 출발
        pubPoseStamped.publish(poseStampedHotel[1]);
        ROS_INFO("101호 출발\n");

        _Flag_Value.bflag_HotelStart = true;
    }

    if (database.data == "hotel102") { // 102호 출발
        pubPoseStamped.publish(poseStampedHotel[2]);
        ROS_INFO("102호 출발\n");

        _Flag_Value.bflag_HotelStart = true;
    }

    if (database.data == "hotel103") { // 103호 출발
        pubPoseStamped.publish(poseStampedHotel[3]);
        ROS_INFO("103호 출발\n");

        _Flag_Value.bflag_HotelStart = true;
    }

    if (database.data == "servehome") { // 서빙 복귀
        pubPoseStamped.publish(poseStampedHotel[3]);
        ROS_INFO("서빙 복귀\n");

        _Flag_Value.bflag_ServeComebackHome = true;
    }

    if (database.data == "hotelhome") { // 호텔 복귀
        pubPoseStamped.publish(poseStampedHotel[3]);
        ROS_INFO("호텔 복귀\n");

        _Flag_Value.bflag_HotelComebackHome = true;
    }
}

void checkArrival(const move_base_msgs::MoveBaseActionResult arrival) {
    if (arrival.status.status == 3){
        if (_Flag_Value.bflag_ServeStart) {
            fbServeArrival = "테이블 도착";
            pubFirebase.publish(fbServeArrival);

            _Flag_Value.bflag_ServeStart = false;
        }

        if (_Flag_Value.bflag_HotelStart) {
            fbHotelArrival = "호실 도착";
            pubFirebase.publish(fbHotelArrival);

            _Flag_Value.bflag_HotelStart = false;
        }

        if (_Flag_Value.bflag_ServeComebackHome) {
            _Flag_Value.bflag_ServeComebackHome = false;
            ROS_INFO("Home 도착");
            _Flag_Value.bflag_ServeHome = true;
        }
    }
}

private:
ros::NodeHandle nh;

ros::Publisher pubPoseStamped;
ros::Publisher pubFirebase;

ros::Subscriber subFirebase;

geometry_msgs::PoseStamped poseStampedHome[2];
geometry_msgs::PoseStamped poseStampedTable[4];
geometry_msgs::PoseStamped poseStampedHotel[4];

std_msgs::String fbHotelArrival, fbServeArrival;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robotService");
    ros::NodeHandle nh;

    ROS_INFO("SYSTEM ON.\n");

    Robot rb;

    return 0;
}
