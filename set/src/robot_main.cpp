#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h" 

using namespace std;

class Service {
public:
Service() {
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
};

enum serveState {
    S_WAIT,
    S_MOVING,
    S_ARR,
    S_RETURN,
    S_RETRIEVAL,
    S_RETRIEVAL_ARR,
    S_HOME
};

enum hotelState {
    H_WAIT,
    H_MOVING_EV,
    H_FRONT_EV,
    H_INSIDE_EV,
    H_MOVING_ROOM,
    H_ARR,
    H_RETURN,
    H_RETURN_INSIDE_EV,
    H_RETURN_MOVING_EV,
    H_RETURN_FRONT_EV,
    H_HOME
};

// 초기 좌표 설정
void fnInitParam() {
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

    // 엘레베이터 안 (2층 -> 1층) //
    nh.getParam("front_of_ev2/position", target_pose_position);
    nh.getParam("front_of_ev2/orientation", target_pose_orientation);

    poseStamped[5].header.frame_id = "map";
    poseStamped[5].header.stamp = ros::Time::now();

    poseStamped[5].pose.position.x = target_pose_position[0];
    poseStamped[5].pose.position.y = target_pose_position[1];
    poseStamped[5].pose.position.z = target_pose_position[2];

    poseStamped[5].pose.orientation.x = target_pose_orientation[0];
    poseStamped[5].pose.orientation.y = target_pose_orientation[1];
    poseStamped[5].pose.orientation.z = target_pose_orientation[2];
    poseStamped[5].pose.orientation.w = target_pose_orientation[3];
}

// 모듈 장착 여부 확인
void CheckModule(const std_msgs::String module) {
    if (strcmp(module.data.c_str(), "hotel") == 0) {
        m_hotel = true;
        _HOTEL_FLAG = H_WAIT;
        ROS_INFO("Hotel Module Activated.");
        sleep(0.5);
        m_serve = false;
    }

    else if (strcmp(module.data.c_str(), "serving") == 0) {
        m_serve = true;
        _SERVE_FLAG = S_WAIT;
        ROS_INFO("Serving Module Activated.");
        sleep(0.5);
        m_hotel = false;
    }
}

// 파이어베이스 확인
void CheckFirebase(const std_msgs::String firebase) {
    // 서빙 //
    if (m_serve && _SERVE_FLAG == S_WAIT) {
        if (strcmp(firebase.data.c_str(), "Serving_Start") == 0) {
            ROS_INFO("Move to Table 1.");
            pubPoseStamped.publish(poseStamped[2]);
            _SERVE_FLAG = S_MOVING;
        }
    }

    // 호텔 //
    if (m_hotel && _HOTEL_FLAG == H_WAIT) {
        if (strcmp(firebase.data.c_str(), "Success") == 0) {
            ROS_INFO("Move to 101.");
            sleep(1);
            pubPoseStamped.publish(poseStamped[4]);
            _HOTEL_FLAG = H_MOVING_EV;
        }

        else if (strcmp(firebase.data.c_str(), "Home_Success") == 0) {
            ROS_INFO("Return.");
            ultra.data = "180";
            pubUltra.publish(ultra);
            sleep(7);
            pubPoseStamped.publish(poseStamped[5]);
            _HOTEL_FLAG = H_RETURN;
        }
    }
}

// 로봇 도착 유무 확인
void CheckArrival(const move_base_msgs::MoveBaseActionResult arrival)
{
    if (arrival.status.status == 3)
    {
        if(m_hotel) {
            if (_HOTEL_FLAG == H_MOVING_EV) {
                _HOTEL_FLAG = H_FRONT_EV;
                ROS_INFO("Front of EV");
                sleep(2);
                ev.data = "open1";
                pubElevator1.publish(ev);
            }

            else if (_HOTEL_FLAG == H_MOVING_ROOM) {
                _HOTEL_FLAG = H_ARR;
                ROS_INFO("Room Arrival");
                sleep(2);
                qr.data = "QR";
                pubFirebase.publish(qr);
                sleep(0.5);
                hotel.data = "ros";
                pubHotel.publish(hotel);
            }

            else if (_HOTEL_FLAG == H_RETURN) {
                _HOTEL_FLAG = H_RETURN_FRONT_EV;
                ROS_INFO("Front of EV, Return");
                sleep(2);
                ev.data = "open1";
                pubElevator2.publish(ev);
            }

            else if (_HOTEL_FLAG == H_RETURN_INSIDE_EV) {
                ROS_INFO("Inside of EV, Return");
                sleep(2);
                ev.data = "close1";
                pubElevator2.publish(ev);
            }

            else if (_HOTEL_FLAG == H_HOME) {
                ROS_INFO("Home");
                sleep(2);
                _HOTEL_FLAG = H_WAIT;
            }
        }

        if(m_serve) {
            if (_SERVE_FLAG == S_MOVING) {
                _SERVE_FLAG = S_ARR;
                ROS_INFO("arrive to table");
                sleep(2);
                ultra.data = "leftFront";
                pubUltra.publish(ultra);
            }

            else if (_SERVE_FLAG == S_RETRIEVAL) {
                _SERVE_FLAG = S_RETRIEVAL_ARR;
                ROS_INFO("arrive to table for retrieval");
                sleep(2);
                ultra.data = "leftFront";
                pubUltra.publish(ultra);
            }

            else if (_SERVE_FLAG == S_RETURN) {
                _SERVE_FLAG = S_HOME;
                ROS_INFO("Home");
                sleep(2);
                _SERVE_FLAG = S_WAIT;
                ultra.data = "90";
                pubUltra.publish(ultra);
            }
        }
    }
}

// 초음파 센서 확인
void CheckUltra(const std_msgs::String ultra)
{
    if (m_serve) {
        if (strcmp(ultra.data.c_str(), "end") == 0) {
            // 서빙 //
            if (_SERVE_FLAG == S_ARR) {
                ROS_INFO("Finished, Plate Forward");
                sleep(1);
                direction.data = "forward";
                pubPlate.publish(direction);
            }

            else if (_SERVE_FLAG == S_RETRIEVAL_ARR) {
                ROS_INFO("Finished, Plate Backward");
                sleep(1);
                direction.data = "backward";
                pubPlate.publish(direction);
            }

            if (_SERVE_FLAG == S_RETURN) {
                ROS_INFO("Finished, Home");
                sleep(1);
                pubPoseStamped.publish(poseStamped[0]);
            }

            // 호텔 //
            if(_HOTEL_FLAG == H_INSIDE_EV) {
                ROS_INFO("Inside");
                sleep(1);
                ev.data = "close1";
                pubElevator1.publish(ev);
            }

            if(_HOTEL_FLAG == H_RETURN_INSIDE_EV) {
                ROS_INFO("Inside");
                sleep(1);
                ev.data = "close1";
                pubElevator2.publish(ev);
            }            
        }
    }

}

// 쟁반 로봇 확인
void CheckPlate(const std_msgs::String plate)
{
    if (m_serve) {
        if (strcmp(plate.data.c_str(), "end") == 0) {
            if (_SERVE_FLAG == S_ARR || _SERVE_FLAG == S_RETRIEVAL_ARR){
                _SERVE_FLAG = S_RETURN;
                ROS_INFO("Plate End");
                sleep(2);
                ultra.data = "backLeft";
                pubUltra.publish(ultra);
            }
        }
    }
}

// 1층 -> 2층 //
void CheckElevator1(const std_msgs::String elevator1)
{
    if (m_hotel){
        if (strcmp(elevator1.data.c_str(), "First_Open") == 0) {
            ROS_INFO("1 door open");
            sleep(2);
            ultra.data = "forward";
            pubUltra.publish(ultra);
            _HOTEL_FLAG = H_INSIDE_EV;
        }

        else if (strcmp(elevator1.data.c_str(), "Second_Open") == 0) {
            ROS_INFO("2 door open");
            sleep(2);
            pubPoseStamped.publish(poseStamped[3]);
            _HOTEL_FLAG = H_MOVING_ROOM;
            sleep(3);
            ev.data = "close2";
            pubElevator1.publish(ev);
        }
    }
}

// 2층 -> 1층 //
void CheckElevator2(const std_msgs::String elevator2)
{
    if (m_hotel) {
        if (strcmp(elevator2.data.c_str(), "First_Open") == 0) {
            ROS_INFO("2 door open, Return");
            sleep(2);
            ultra.data = "forward";
            pubUltra.publish(ultra);
            _HOTEL_FLAG = H_RETURN_INSIDE_EV;
        }

        else if (strcmp(elevator2.data.c_str(), "Second_Open") == 0) {
            ROS_INFO("1 door open, Return");
            sleep(2);
            pubPoseStamped.publish(poseStamped[1]);
            _HOTEL_FLAG = H_HOME;
            sleep(3);
            ev.data = "close2";
            pubElevator2.publish(ev);
        }
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

geometry_msgs::PoseStamped poseStamped[6];

vector<double> target_pose_position;
vector<double> target_pose_orientation;

int _HOTEL_FLAG, _SERVE_FLAG;

std_msgs::String qr, ultra, direction, ev, hotel;

bool m_hotel = false;
bool m_serve = false;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Service");

    ROS_INFO("[SYSTEM ON]");

    Service service;

    ros::spin();

    return 0; 
}