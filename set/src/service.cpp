#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"

#include "set/data.h"

extern mode;

enum State{
    STATE_WAIT,
    STATE_MOVING,
    STATE_ARRIVAL,
    STATE_RETURN
}; State state;

class Serving{
    public:
    Serving(){
        InitParam();

        pubPoseStamped = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        pubFirebase = nh.advertise<std_msgs::Int16>("/firebase", 1);

        subFirebase = nh.subscribe("/firebase", 11, &Serving::fbCallback, this);
        subArrival = nh.subscribe("/move_base/result", 1, &Serving::CheckArrival, this);
        subLockSensor = nh.subscribe("/lockSensor", 1, &Serving::CheckLockSensor, this);
        subPlateSensor = nh.subscribe("/plateSensor", 1, &Serving::CheckPlateSensor, this);
    }

    void fnInitParam(){
        // 1번 테이블
        nh.getParam("table_pose_first/position", target_pose_position);
        nh.getParam("table_pose_first/orientation", target_pose_orientation);

        poseStampedTable[0].header.frame_id = "map";
        poseStampedTable[0].header.stamp = ros::Time::now();

        poseStampedTable[0].pose.position.x = target_pose_position[0];
        poseStampedTable[0].pose.position.y = target_pose_position[1];
        poseStampedTable[0].pose.position.z = target_pose_position[2];

        poseStampedTable[0].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[0].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[0].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[0].pose.orientation.w = target_pose_orientation[3];

        // 2번 테이블
        nh.getParam("table_pose_second/position", target_pose_position);
        nh.getParam("table_pose_second/orientation", target_pose_orientation);

        poseStampedTable[1].header.frame_id = "map";
        poseStampedTable[1].header.stamp = ros::Time::now();

        poseStampedTable[1].pose.position.x = target_pose_position[0];
        poseStampedTable[1].pose.position.y = target_pose_position[1];
        poseStampedTable[1].pose.position.z = target_pose_position[2];

        poseStampedTable[1].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[1].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[1].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[1].pose.orientation.w = target_pose_orientation[3];

        // 3번 테이블
        nh.getParam("table_pose_third/position", target_pose_position);
        nh.getParam("table_pose_third/orientation", target_pose_orientation);

        poseStampedTable[2].header.frame_id = "map";
        poseStampedTable[2].header.stamp = ros::Time::now();

        poseStampedTable[2].pose.position.x = target_pose_position[0];
        poseStampedTable[2].pose.position.y = target_pose_position[1];
        poseStampedTable[2].pose.position.z = target_pose_position[2];

        poseStampedTable[2].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[2].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[2].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[2].pose.orientation.w = target_pose_orientation[3];

        // 초기 위치
        nh.getParam("init_pose_serving/position", target_pose_position);
        nh.getParam("init_pose_serving/orientation", target_pose_orientation);

        poseStampedInit[0].header.frame_id = "map";
        poseStampedInit[0].header.stamp = ros::Time::now();

        poseStampedInit[0].pose.position.x = target_pose_position[0];
        poseStampedInit[0].pose.position.y = target_pose_position[1];
        poseStampedInit[0].pose.position.z = target_pose_position[2];

        poseStampedInit[0].pose.orientation.x = target_pose_orientation[0];
        poseStampedInit[0].pose.orientation.y = target_pose_orientation[1];
        poseStampedInit[0].pose.orientation.z = target_pose_orientation[2];
        poseStampedInit[0].pose.orientation.w = target_pose_orientation[3];

        // 객실 101호
        nh.getParam("room_pose_101/position", target_pose_position);
        nh.getParam("room_pose_101/orientation", target_pose_orientation);

        poseStampedRoom[0].header.frame_id = "map";
        poseStampedRoom[0].header.stamp = ros::Time::now();

        poseStampedRoom[0].pose.position.x = target_pose_position[0];
        poseStampedRoom[0].pose.position.y = target_pose_position[1];
        poseStampedRoom[0].pose.position.z = target_pose_position[2];

        poseStampedRoom[0].pose.orientation.x = target_pose_orientation[0];
        poseStampedRoom[0].pose.orientation.y = target_pose_orientation[1];
        poseStampedRoom[0].pose.orientation.z = target_pose_orientation[2];
        poseStampedRoom[0].pose.orientation.w = target_pose_orientation[3];

        // 객실 102호
        nh.getParam("room_pose_102/position", target_pose_position);
        nh.getParam("room_pose_102/orientation", target_pose_orientation);

        poseStampedRoom[1].header.frame_id = "map";
        poseStampedRoom[1].header.stamp = ros::Time::now();

        poseStampedRoom[1].pose.position.x = target_pose_position[0];
        poseStampedRoom[1].pose.position.y = target_pose_position[1];
        poseStampedRoom[1].pose.position.z = target_pose_position[2];

        poseStampedRoom[1].pose.orientation.x = target_pose_orientation[0];
        poseStampedRoom[1].pose.orientation.y = target_pose_orientation[1];
        poseStampedRoom[1].pose.orientation.z = target_pose_orientation[2];
        poseStampedRoom[1].pose.orientation.w = target_pose_orientation[3];

        // 객실 103호
        nh.getParam("room_pose_103/position", target_pose_position);
        nh.getParam("room_pose_103/orientation", target_pose_orientation);

        poseStampedRoom[2].header.frame_id = "map";
        poseStampedRoom[2].header.stamp = ros::Time::now();

        poseStampedRoom[2].pose.position.x = target_pose_position[0];
        poseStampedRoom[2].pose.position.y = target_pose_position[1];
        poseStampedRoom[2].pose.position.z = target_pose_position[2];

        poseStampedRoom[2].pose.orientation.x = target_pose_orientation[0];
        poseStampedRoom[2].pose.orientation.y = target_pose_orientation[1];
        poseStampedRoom[2].pose.orientation.z = target_pose_orientation[2];
        poseStampedRoom[2].pose.orientation.w = target_pose_orientation[3];

        // 초기 위치
        nh.getParam("init_pose_hotel/position", target_pose_position);
        nh.getParam("init_pose_hotel/orientation", target_pose_orientation);

        poseStampedInit[0].header.frame_id = "map";
        poseStampedInit[0].header.stamp = ros::Time::now();

        poseStampedInit[0].pose.position.x = target_pose_position[0];
        poseStampedInit[0].pose.position.y = target_pose_position[1];
        poseStampedInit[0].pose.position.z = target_pose_position[2];

        poseStampedInit[0].pose.orientation.x = target_pose_orientation[0];
        poseStampedInit[0].pose.orientation.y = target_pose_orientation[1];
        poseStampedInit[0].pose.orientation.z = target_pose_orientation[2];
        poseStampedInit[0].pose.orientation.w = target_pose_orientation[3];
    }

    void CheckArrival(const move_base_msgs::MoveBaseActionResult arrival){
        if (mode == 0){
            if (arrival.status.status == 3){
                if (state == STATE_MOVING){
                    state = STATE_ARRIVAL;
                    pubFirebase.publish(Arrival);
                    ROS_INFO("Arrival");
                }

                else if (state == STATE_RETURN){
                    state = STATE_WAIT;
                    pubFirebase.publish(Wait);
                    ROS_INFO("Arrival");
                }
            }
        }

        else if (mode == 1){
            if (arrival.status.status == 3){
                if (state == STATE_MOVING){
                    state = STATE_ARRIVAL;
                    pubFirebase.publish(Arrival);
                    ROS_INFO("Arrival");
                }

                else if (state == STATE_RETURN){
                    state = STATE_WAIT;
                    pubFirebase.publish(Wait);
                    ROS_INFO("Arrival");
                }
            }
        }
    }

    void CheckLockSensor(const std_msgs::Int16 sensor){
        if(sensor.data == 1){
            pubFirebase.publish(Return);
        }
    }

    void CheckPlateSensor(const std_msgs::Int16 sensor){
        if(sensor.data == 1){
            pubFirebase.publish(Return);
        }
    }

    void fbCallback(const std_msgs::Int16 &fbData){
        if (mode == 0){
            if (fbData.data == 3){
                pubPoseStamped.publish(poseStampedTable[0]);
                pubFirebase.publish(Moving);
                state = STATE_MOVING;
                ROS_INFO("Table 1");
            }

            else if (fbData.data == 4){
                pubPoseStamped.publish(poseStampedTable[1]);
                pubFirebase.publish(Moving);
                state = STATE_MOVING;
                ROS_INFO("Table 2");
            }

            else if (fbData.data == 5){
                pubPoseStamped.publish(poseStampedTable[2]);
                pubFirebase.publish(Moving);
                state = STATE_MOVING;
                ROS_INFO("Table 3");
            }

            else if (fbData.data == 6){
                pubPoseStamped.publish(poseStampedInit[0]);
                pubFirebase.publish(Moving);
                state = STATE_RETURN;
                ROS_INFO("Init");
            }
        }

        if (mode == 1){
            if (fbData.data == 3){
                pubPoseStamped.publish(poseStampedTable[0]);
                pubFirebase.publish(Moving);
                state = STATE_MOVING;
                ROS_INFO("Table 1");
            }

            else if (fbData.data == 4){
                pubPoseStamped.publish(poseStampedTable[1]);
                pubFirebase.publish(Moving);
                state = STATE_MOVING;
                ROS_INFO("Table 2");
            }

            else if (fbData.data == 5){
                pubPoseStamped.publish(poseStampedTable[2]);
                pubFirebase.publish(Moving);
                state = STATE_MOVING;
                ROS_INFO("Table 3");
            }

            else if (fbData.data == 6){
                pubPoseStamped.publish(poseStampedInit[0]);
                pubFirebase.publish(Moving);
                state = STATE_RETURN;
                ROS_INFO("Init");
            }
        }
    }

    private:
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher pubPoseStamped;
    ros::Publisher pubFirebase;

    // Subscriber
    ros::Subscriber subFirebase;
    ros::Subscriber subArrival;
    ros::Subscriber subLockSensor;
    ros::Subscriber subPlateSensor;

    // Pose
    geometry_msgs::PoseStamped poseStampedTable[3];
    geometry_msgs::PoseStamped poseStampedInit[1];
    vector<double> target_pose_position;
    vector<double> target_pose_orientation;

};

int main(int argc, char** argv){
    ros::init(argc, argv, "serving");

    ROS_INFO("Serving Node On");

    Serving serving;

    ros::spin();

}