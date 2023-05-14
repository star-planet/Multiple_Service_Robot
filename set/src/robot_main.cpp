#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h" 

using namespace std;

class Service
{
public:
Service()
{
    fnInitParam();

    pubPoseStamped = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    pubUltra = nh.advertise<std_msgs::String>("/ultra", 1);
    pubPlate = nh.advertise<std_msgs::String>("/plate", 1);
    pubHotel = nh.advertise<std_msgs::String>("/hotel", 1);
    pubFirebase = nh.advertise<std_msgs::String>("/ros_to_firebase", 1);
    pubElevator1 = nh.advertise<std_msgs::String>("/elevator1", 1);
    pubElevator2 = nh.advertise<std_msgs::String>("/elevator2", 1);

    sub_arrival_status = nh.subscribe("/move_base/result", 1, &Service::CheckArrival, this);
    sub_ultra_status = nh.subscribe("/ultra", 1, &Service::CheckUltra, this);
    sub_plate_status = nh.subscribe("/plate", 1, &Service::CheckPlate, this);
    sub_firebase = nh.subscribe("firebase_to_ros", 1, &Service::CheckFirebase, this);
    sub_elevator1 = nh.subscribe("/elevator1", 1, &Service::CheckElevator1, this);
    sub_elevator2 = nh.subscribe("/elevator2", 1, &Service::CheckElevator2, this);
    sub_module_status = nh.subscribe("/module", 1, &Service::CheckModule, this);
}

enum hotelState
{
    H_WAIT,
    H_MOVING_EV,
    H_FRONT_EV,
    H_INSIDE_EV,
    H_MOVING_ROOM,
    H_ARR,
    H_RETURN,
    H_RETURN_MOVING_EV,
    H_RETURN_FRONT_EV,
    H_RETURN_INSIDE_EV,
    H_HOME,
};

enum serveState{
    S_WAIT,
    S_MOVING,
    S_ARR,
    S_RETURN,
    S_RETRIEVAL,
    S_RETRIEVAL_ARR,
    S_HOME
};

void fnInitParam()
{
    // 서빙 Home //
    nh.getParam("serve_home/position", target_pose_position);
    nh.getParam("serve_home/orientation", target_pose_orientation);

    poseStamped[0].header.frame_id = "map";
    poseStamped[0].header.stamp = ros::Time::now();

    poseStamped[0].pose.position.x = target_pose_position[0];
    poseStamped[0].pose.position.y = target_pose_position[1];
    poseStamped[0].pose.position.z = target_pose_position[2];

    poseStamped[0].pose.orientation.x = target_pose_orientation[0];
    poseStamped[0].pose.orientation.y = target_pose_orientation[1];
    poseStamped[0].pose.orientation.z = target_pose_orientation[2];
    poseStamped[0].pose.orientation.w = target_pose_orientation[3];

    // 호텔 Home //
    nh.getParam("hotel_home/position", target_pose_position);
    nh.getParam("hotel_home/orientation", target_pose_orientation);

    poseStamped[1].header.frame_id = "map";
    poseStamped[1].header.stamp = ros::Time::now();

    poseStamped[1].pose.position.x = target_pose_position[0];
    poseStamped[1].pose.position.y = target_pose_position[1];
    poseStamped[1].pose.position.z = target_pose_position[2];

    poseStamped[1].pose.orientation.x = target_pose_orientation[0];
    poseStamped[1].pose.orientation.y = target_pose_orientation[1];
    poseStamped[1].pose.orientation.z = target_pose_orientation[2];
    poseStamped[1].pose.orientation.w = target_pose_orientation[3];

    // 1번 테이블 //
    nh.getParam("serve_table/position", target_pose_position);
    nh.getParam("serve_table/orientation", target_pose_orientation);

    poseStamped[2].header.frame_id = "map";
    poseStamped[2].header.stamp = ros::Time::now();

    poseStamped[2].pose.position.x = target_pose_position[0];
    poseStamped[2].pose.position.y = target_pose_position[1];
    poseStamped[2].pose.position.z = target_pose_position[2];

    poseStamped[2].pose.orientation.x = target_pose_orientation[0];
    poseStamped[2].pose.orientation.y = target_pose_orientation[1];
    poseStamped[2].pose.orientation.z = target_pose_orientation[2];
    poseStamped[2].pose.orientation.w = target_pose_orientation[3];

    // 호텔 객실 //
    nh.getParam("hotel_room/position", target_pose_position);
    nh.getParam("hotel_room/orientation", target_pose_orientation);

    poseStamped[3].header.frame_id = "map";
    poseStamped[3].header.stamp = ros::Time::now();

    poseStamped[3].pose.position.x = target_pose_position[0];
    poseStamped[3].pose.position.y = target_pose_position[1];
    poseStamped[3].pose.position.z = target_pose_position[2];

    poseStamped[3].pose.orientation.x = target_pose_orientation[0];
    poseStamped[3].pose.orientation.y = target_pose_orientation[1];
    poseStamped[3].pose.orientation.z = target_pose_orientation[2];
    poseStamped[3].pose.orientation.w = target_pose_orientation[3];

    // 엘레베이터 앞(1층 -> 2층) //
    nh.getParam("front_of_ev1/position", target_pose_position);
    nh.getParam("front_of_ev1/orientation", target_pose_orientation);

    poseStamped[4].header.frame_id = "map";
    poseStamped[4].header.stamp = ros::Time::now();

    poseStamped[4].pose.position.x = target_pose_position[0];
    poseStamped[4].pose.position.y = target_pose_position[1];
    poseStamped[4].pose.position.z = target_pose_position[2];

    poseStamped[4].pose.orientation.x = target_pose_orientation[0];
    poseStamped[4].pose.orientation.y = target_pose_orientation[1];
    poseStamped[4].pose.orientation.z = target_pose_orientation[2];
    poseStamped[4].pose.orientation.w = target_pose_orientation[3];

    // 엘레베이터 안 (1층 -> 2층) //
    nh.getParam("inside_of_ev1/position", target_pose_position);
    nh.getParam("inside_of_ev1/orientation", target_pose_orientation);

    poseStamped[5].header.frame_id = "map";
    poseStamped[5].header.stamp = ros::Time::now();

    poseStamped[5].pose.position.x = target_pose_position[0];
    poseStamped[5].pose.position.y = target_pose_position[1];
    poseStamped[5].pose.position.z = target_pose_position[2];

    poseStamped[5].pose.orientation.x = target_pose_orientation[0];
    poseStamped[5].pose.orientation.y = target_pose_orientation[1];
    poseStamped[5].pose.orientation.z = target_pose_orientation[2];
    poseStamped[5].pose.orientation.w = target_pose_orientation[3];

    // 엘레베이터 앞(2층 -> 1층) //
    nh.getParam("front_of_ev2/position", target_pose_position);
    nh.getParam("front_of_ev2/orientation", target_pose_orientation);

    poseStamped[6].header.frame_id = "map";
    poseStamped[6].header.stamp = ros::Time::now();

    poseStamped[6].pose.position.x = target_pose_position[0];
    poseStamped[6].pose.position.y = target_pose_position[1];
    poseStamped[6].pose.position.z = target_pose_position[2];

    poseStamped[6].pose.orientation.x = target_pose_orientation[0];
    poseStamped[6].pose.orientation.y = target_pose_orientation[1];
    poseStamped[6].pose.orientation.z = target_pose_orientation[2];
    poseStamped[6].pose.orientation.w = target_pose_orientation[3];

    // 엘레베이터 안 (2층 -> 1층) //
    nh.getParam("inside_of_ev2/position", target_pose_position);
    nh.getParam("inside_of_ev2/orientation", target_pose_orientation);

    poseStamped[7].header.frame_id = "map";
    poseStamped[7].header.stamp = ros::Time::now();

    poseStamped[7].pose.position.x = target_pose_position[0];
    poseStamped[7].pose.position.y = target_pose_position[1];
    poseStamped[7].pose.position.z = target_pose_position[2];

    poseStamped[7].pose.orientation.x = target_pose_orientation[0];
    poseStamped[7].pose.orientation.y = target_pose_orientation[1];
    poseStamped[7].pose.orientation.z = target_pose_orientation[2];
    poseStamped[7].pose.orientation.w = target_pose_orientation[3];
}

void CheckModule(const std_msgs::String module)
{
    if (strcmp(module.data.c_str(), "hotel") == 0)
    {
        _HOTEL_FLAG = H_WAIT;
        ROS_INFO("호텔 모듈이 장착되었습니다. 현재 상태 : %d", _HOTEL_FLAG);
    }

    else if (strcmp(module.data.c_str(), "serving") == 0)
    {
        _SERVE_FLAG = S_WAIT;
        ROS_INFO("서빙 모듈이 장착되었습니다. 현재 상태 : %d", _SERVE_FLAG);
    }
}

void CheckFirebase(const std_msgs::String firebase)
{
    if (_HOTEL_FLAG == H_WAIT || _SERVE_FLAG == S_WAIT)
    {
        // SERVE //
        if (strcmp(firebase.data.c_str(), "테이블 1번") == 0)
        {
            ROS_INFO("[서빙] 1번 테이블로 이동합니다.");
            pubPoseStamped.publish(poseStamped[2]);
            _SERVE_FLAG = S_MOVING;
            ROS_INFO("서빙 진행 상황: %d", _SERVE_FLAG);
        }

        else if (strcmp(firebase.data.c_str(), "테이블 1번 회수") == 0)
        {
            ROS_INFO("[회수] 1번 테이블로 이동합니다.");
            pubPoseStamped.publish(poseStamped[2]);
            _SERVE_FLAG = S_RETRIEVAL;
            ROS_INFO("서빙 진행 상황: %d", _SERVE_FLAG);
        }

        else if (strcmp(firebase.data.c_str(), "서빙 복귀") == 0)
        {
            ROS_INFO("[복귀] 원래 위치로 복귀합니다.");
            pubPoseStamped.publish(poseStamped[0]);
            _SERVE_FLAG = S_RETURN;
            ROS_INFO("서빙 진행 상황: %d", _SERVE_FLAG);
        }

        else if (strcmp(firebase.data.c_str(), "테이블 1번 회수 복귀") == 0)
        {
            ROS_INFO("[회수] 원래 위치로 복귀합니다.");
            pubPoseStamped.publish(poseStamped[0]);
            _SERVE_FLAG = S_RETURN;
            ROS_INFO("서빙 진행 상황: %d", _SERVE_FLAG);
        }

        // HOTEL //
        else if (strcmp(firebase.data.c_str(), "101호") == 0)
        {
            ROS_INFO("[이동] 해당 호실로 이동합니다.");
            pubPoseStamped.publish(poseStamped[4]);
            _HOTEL_FLAG = H_MOVING_EV;
            ROS_INFO("호텔 진행 상황: %d", _HOTEL_FLAG);
        }

        else if (strcmp(firebase.data.c_str(), "호텔 복귀") == 0)
        {
            ROS_INFO("[복귀] 원래 위치로 복귀합니다.");
            pubPoseStamped.publish(poseStamped[6]);
            _HOTEL_FLAG = H_RETURN;
            ROS_INFO("호텔 진행 상황: %d", _HOTEL_FLAG);
        }
    }
}

void CheckArrival(const move_base_msgs::MoveBaseActionResult arrival)
{
    if (arrival.status.status == 3)
    {
        // HOTEL //
        if (_HOTEL_FLAG == H_MOVING_EV)
        {
            _HOTEL_FLAG = H_FRONT_EV;
            ROS_INFO("엘레베이터 앞에 도착하였습니다.");
            ROS_INFO("호텔 진행 상황: %d", _HOTEL_FLAG);
            stringstream op;
            op << "open1";
            ev1.data = op.str();
            pubElevator1.publish(ev1);
        }

        else if (_HOTEL_FLAG == H_INSIDE_EV)
        {
            ROS_INFO("엘레베이터 층간 이동 중입니다.");
            ROS_INFO("호텔 진행 상황: %d", _HOTEL_FLAG);
            stringstream cl;
            cl << "close1";
            ev1.data = cl.str();
            pubElevator1.publish(ev1);
        }

        else if (_HOTEL_FLAG == H_MOVING_ROOM)
        {
            ROS_INFO("호실 앞에 도착했습니다.");
            ROS_INFO("호텔 진행 상황: %d", _HOTEL_FLAG);
            stringstream q;
            q << "QR";
            qr.data = q.str();
            pubFirebase.publish(qr);
            stringstream ho;
            ho << "ros";
            hotel.data = ho.str();
            pubHotel.publish(hotel);
        }

        else if (_HOTEL_FLAG == H_RETURN)
        {
            _HOTEL_FLAG = H_RETURN_FRONT_EV;
            ROS_INFO("엘레베이터 앞에 도착했습니다.");
            ROS_INFO("호텔 진행 상황: %d", _HOTEL_FLAG);
            stringstream op;
            op << "open1";
            ev2.data = op.str();
            pubElevator2.publish(ev2);
        }

        else if (_HOTEL_FLAG == H_RETURN_INSIDE_EV)
        {
            ROS_INFO("엘레베이터 층간 이동 중입니다.");
            ROS_INFO("호텔 진행 상황: %d", _HOTEL_FLAG);
            stringstream cl;
            cl << "close1";
            ev2.data = cl.str();
            pubElevator2.publish(ev2);
        }

        // SERVE //
        else if (_SERVE_FLAG == S_MOVING)
        {
            _SERVE_FLAG = S_ARR;
            ROS_INFO("[서빙] 테이블에 도착했습니다");
            ROS_INFO("서빙 진행 상황: %d", _SERVE_FLAG);
            stringstream ul;
            ul << "leftFront";
            ultra.data = ul.str();
            pubUltra.publish(ultra);
        }

        else if (_SERVE_FLAG == S_RETRIEVAL)
        {
            _SERVE_FLAG = S_RETRIEVAL_ARR;
            ROS_INFO("[회수] 테이블에 도착했습니다");
            ROS_INFO("서빙 진행 상황: %d", _SERVE_FLAG);
            stringstream ul;
            ul << "leftFront";
            ultra.data = ul.str();
            pubUltra.publish(ultra);
        }

        else if (_SERVE_FLAG == S_RETURN)
        {
            _SERVE_FLAG = S_HOME;
            ROS_INFO("원위치로 복귀했습니다.");
            ROS_INFO("서빙 진행 상황: %d", _SERVE_FLAG);
        }
    }
}

void CheckUltra(const std_msgs::String ultra)
{
    if (strcmp(ultra.data.c_str(), "end") == 0)
    {
        if (_SERVE_FLAG == S_ARR)
        {
            ROS_INFO("테이블 접근 완료. 쟁반 투입");
            stringstream g;
            g << "forward";
            go.data = g.str();
            //pubPlate.publish(go);
        }

        else if (_SERVE_FLAG == S_RETRIEVAL_ARR)
        {
            ROS_INFO("테이블 접근 완료. 쟁반 회수");
            stringstream b;
            b << "backward";
            back.data = b.str();
            pubPlate.publish(back);
        }

        else if (_SERVE_FLAG == S_RETURN)
        {
            ROS_INFO("회수 완료. 원위치로 복귀합니다.");
            pubPoseStamped.publish(poseStamped[0]);
        }
    }
}

void CheckPlate(const std_msgs::String plate)
{
    if (strcmp(plate.data.c_str(), "end") == 0)
    {
        _SERVE_FLAG = S_RETURN;
        stringstream b;
        b << "backward";
        ultra.data = b.str();
        pubUltra.publish(ultra);
    }
}

// // 1층 -> 2층 //
void CheckElevator1(const std_msgs::String elevator1)
{
    if (strcmp(elevator1.data.c_str(), "First_Open") == 0)
    {
        ROS_INFO("1층 문이 열렸습니다.");
        pubPoseStamped.publish(poseStamped[5]);
        _HOTEL_FLAG = H_INSIDE_EV;
    }

    else if (strcmp(elevator1.data.c_str(), "Second_Open") == 0)
    {
        ROS_INFO("2층 문이 열렸습니다.");
        pubPoseStamped.publish(poseStamped[3]);
        _HOTEL_FLAG = H_MOVING_ROOM;
        sleep(3);
        stringstream cl;
        cl << "close2";
        ev1.data = cl.str();
        pubElevator1.publish(ev1);
    }
}

// 2층 -> 1층 //
void CheckElevator2(const std_msgs::String elevator2)
{
    if (strcmp(elevator2.data.c_str(), "First_Open") == 0)
    {
        ROS_INFO("2층 문이 열렸습니다.");
        pubPoseStamped.publish(poseStamped[7]);
        _HOTEL_FLAG = H_RETURN_INSIDE_EV;
    }

    else if (strcmp(elevator2.data.c_str(), "Second_Open") == 0)
    {
        ROS_INFO("1층 문이 열렸습니다.");
        pubPoseStamped.publish(poseStamped[1]);
        sleep(3);
        stringstream cl;
        cl << "close2";
        ev2.data = cl.str();
        pubElevator2.publish(ev2);
    }
}

private:
ros::NodeHandle nh;

ros::Publisher pubPoseStamped;
ros::Publisher pubUltra;
ros::Publisher pubPlate;
ros::Publisher pubHotel;
ros::Publisher pubElevator1;
ros::Publisher pubElevator2;
ros::Publisher pubFirebase;

ros::Subscriber sub_arrival_status;
ros::Subscriber sub_ultra_status;
ros::Subscriber sub_plate_status;
ros::Subscriber sub_firebase;
ros::Subscriber sub_elevator1;
ros::Subscriber sub_elevator2;
ros::Subscriber sub_module_status;

geometry_msgs::PoseStamped poseStamped[8];

vector<double> target_pose_position;
vector<double> target_pose_orientation;

int _HOTEL_FLAG;
int _SERVE_FLAG  ;

std_msgs::String qr, ultra, go, back, ev1, ev2, hotel;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Service");

    ROS_INFO("SYSTEM ON");

    Service service;

    ros::spin();

    return 0; 
}