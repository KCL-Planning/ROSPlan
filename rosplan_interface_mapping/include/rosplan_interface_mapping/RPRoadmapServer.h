#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "diagnostic_msgs/KeyValue.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseStamped.h"
#include "mongodb_store/message_store.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h> 
#include <algorithm> 

#ifndef KCL_roadmap
#define KCL_roadmap

/**
 * This file defines the RPRoadmapServer class.
 * RPRoadmapServer is used to generate waypoints from a given map.
 * Currently this works throught the nav_msgs/GetMap service.
 * Waypoints are stored symbolically in the Knoweldge Base.
 * Waypoint coordinates are stored in the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	struct Waypoint 
	{
		Waypoint(const std::string &id, unsigned int xCoord, unsigned int yCoord, double resolution, geometry_msgs::Pose origin)
			: wpID(id), grid_x(xCoord), grid_y(yCoord) {

			real_x = xCoord * resolution + origin.position.x;
			real_y = yCoord * resolution + origin.position.y;
		}

		Waypoint()
			: wpID("wp_err"), grid_x(0), grid_y(0) {}

		std::string wpID;
		int grid_x;
		int grid_y;
		double real_x;
		double real_y;
		std::vector<std::string> neighbours;
	};

	struct Edge 
	{
		Edge(const std::string &s, const std::string &e)
			: start(s), end(e) {}

		std::string start;
		std::string end;
	};

	class RPRoadmapServer
	{

	private:
		
		std::string data_path;
		std::string static_map_service;
		bool use_static_map;

		// odometry
		nav_msgs::OccupancyGrid cost_map;
		geometry_msgs::PoseStamped base_odom;
		ros::ServiceClient map_client;
		tf::TransformListener tf;

		// Scene database
		mongodb_store::MessageStoreProxy message_store;

		// Knowledge base
		ros::ServiceClient update_knowledge_client;

		// Roadmap
		std::map<std::string, Waypoint> waypoints;
		std::map<std::string, std::string> db_name_map;
		std::vector<Edge> edges;

		// visualisation
		ros::Publisher waypoints_pub;
		ros::Publisher edges_pub;
		void publishWaypointMarkerArray(ros::NodeHandle nh);
		void publishEdgeMarkerArray(ros::NodeHandle nh);
		void clearMarkerArrays(ros::NodeHandle nh);

	public:

		/* constructor */
		RPRoadmapServer(ros::NodeHandle &nh);

		/* service to (re)generate waypoints */
		bool generateRoadmap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		void createPRM(nav_msgs::OccupancyGrid map, unsigned int K, double D, double R, unsigned int M);
		bool allConnected();
		void connectRecurse(std::map<std::string,bool> &connected, Waypoint &waypoint);
		bool makeConnections(unsigned int R);
		void odomCallback( const nav_msgs::OdometryConstPtr& msg );
		void costMapCallback( const nav_msgs::OccupancyGridConstPtr& msg );
	};
}
#endif
