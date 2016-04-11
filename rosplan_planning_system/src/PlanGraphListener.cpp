#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h> 
#include <fstream>
#include <iostream>
#include <string>

std::string desired_graph_id;

void graphCallback(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data.find(desired_graph_id)) {
		ROS_INFO("msg_recieved");
		std::ofstream file;
		std::stringstream ss;
		ss << desired_graph_id << ".dot";
		file.open(ss.str().c_str());
		file << msg->data;
		file.close();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "plan_graph_listener");
	desired_graph_id = "plan_0";
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/kcl_rosplan/plan_graph", 1000, graphCallback);
	ros::spin();
	return 0;
}
