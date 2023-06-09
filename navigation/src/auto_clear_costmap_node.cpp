#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "move_base_msgs/MoveBaseActionResult.h" 

class Clear {
public:
Clear() {
	sub = nh.subscribe("/move_base/result", 1, &Clear::CheckArrival, this);
	clearClient = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
}

void CheckArrival(const move_base_msgs::MoveBaseActionResult arrival) {
	if (arrival.status.status == 3) {
		clearClient.call(srv);
	}
}

private:
	ros::NodeHandle nh;

	ros::Subscriber sub;

	ros::ServiceClient clearClient;

	std_srvs::Empty srv;
};

int main(int argc, char **argv){
	ROS_INFO("Test Clear Costmap");
	ros::init(argc, argv, "clear_costmap_service");
	ros::service::waitForService("/move_base/clear_costmaps");
}