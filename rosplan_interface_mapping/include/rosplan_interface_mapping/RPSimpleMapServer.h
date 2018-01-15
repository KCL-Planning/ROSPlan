#include "ros/ros.h"

#include "std_srvs/Empty.h"
#include "diagnostic_msgs/KeyValue.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"

#include "mongodb_store/message_store.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>

#ifndef KCL_simple_roadmap
#define KCL_simple_roadmap

namespace KCL_rosplan {

	struct Waypoint 
	{
		Waypoint(const std::string &id, double xCoord, double yCoord)
			: wpID(id), real_x(xCoord), real_y(yCoord) {}

		Waypoint()
			: wpID("wp_err"), real_x(0), real_y(0) {}
		
		float getDistance(const Waypoint& other) {
			return sqrt((real_x - other.real_x) * (real_x - other.real_x) + (real_y - other.real_y) * (real_y - other.real_y));
		}
		
		std::string wpID;
		double real_x;
		double real_y;
		std::vector<std::string> neighbours;
	};

	class RPSimpleMapServer
	{

	private:
		
		std::string data_path;

		// Scene database
		mongodb_store::MessageStoreProxy message_store;

		// Knowledge base
		ros::ServiceClient update_knowledge_client;

		// Roadmap
		std::map<std::string, Waypoint*> waypoints;
		std::map<std::string, std::string> db_name_map;
		void parsePose(geometry_msgs::PoseStamped &pose, std::string line);
		
		// visualisation
		std::string fixed_frame;
		ros::Publisher waypoints_pub;
		void publishWaypointMarkerArray(ros::NodeHandle nh);
		void clearMarkerArrays(ros::NodeHandle nh);

	public:

		/* constructor */
		RPSimpleMapServer(ros::NodeHandle &nh, std::string frame);

		/* service to (re)generate waypoints */
		bool setupRoadmap(std::string filename);
	};
}
#endif
