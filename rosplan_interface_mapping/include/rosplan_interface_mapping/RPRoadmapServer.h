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
#include "rosplan_knowledge_msgs/CreatePRM.h"
#include "rosplan_knowledge_msgs/AddWaypoint.h"
#include "rosplan_knowledge_msgs/RemoveWaypoint.h"

#include "occupancy_grid_utils/coordinate_conversions.h"

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
		Waypoint(const std::string &id, unsigned int xCoord, unsigned int yCoord, const nav_msgs::MapMetaData& map_meta_data)
			: wpID(id), grid_x(xCoord), grid_y(yCoord) {
			occupancy_grid_utils::Cell cell;
			cell.x = grid_x;
			cell.y = grid_y;
			
			geometry_msgs::Point real_point = occupancy_grid_utils::cellCenter(map_meta_data, cell);
			real_x = real_point.x;
			real_y = real_point.y;
		}

		Waypoint()
			: wpID("wp_err"), grid_x(0), grid_y(0) {}
		
		/**
		 * Get the distance between this waypoint and @ref{other}.
		 * @param other The other waypoint to get the distance from.
		 * @return The distance between the waypoints.
		 */
		float getDistance(const Waypoint& other) {
			return sqrt((real_x - other.real_x) * (real_x - other.real_x) + (real_y - other.real_y) * (real_y - other.real_y));
		}
		
		/**
		 * Update the location of this waypoint such that it's no furter than @ref{max_casting_range} away from @ref{other}.
		 * @param other Waypoint this waypoint was casted from.
		 * @param max_casting_range The maximum distance this waypoint can be from the given waypoint.
		 * @param resolution The resolution of the occupancy grid.
		 * @param origin The origin of the occupancy grid.
		 */
		void update(const Waypoint& other, float max_casting_range, const nav_msgs::MapMetaData& map_meta_data) {
			float distance = getDistance(other);
			if (distance > max_casting_range) {
				float scale = max_casting_range / distance;
				
				real_x = other.real_x + (real_x - other.real_x) * scale;
				real_y = other.real_y + (real_y - other.real_y) * scale;
				geometry_msgs::Point point;
				point.x = real_x;
				point.y = real_y;
				
				occupancy_grid_utils::Cell cell = occupancy_grid_utils::pointCell(map_meta_data, point);
				grid_x = cell.x;
				grid_y = cell.y;
			}
		}

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
		std::map<std::string, Waypoint*> waypoints;
		std::map<std::string, std::string> db_name_map;
		std::vector<Edge> edges;
		bool clearWaypoint(const std::string &id);

		// visualisation
		ros::Publisher waypoints_pub;
		ros::Publisher edges_pub;
		void publishWaypointMarkerArray(ros::NodeHandle nh);
		void publishEdgeMarkerArray(ros::NodeHandle nh);
		void clearMarkerArrays(ros::NodeHandle nh);
		
		/**
		 * Check if two waypoints can be connected without colliding with any known scenery. The line should not
		 * come closer than @ref{min_width} than any known obstacle.
		 * @param w1 The first waypoint.
		 * @param w2 The second waypoint.
		 * @param threshold A value between -1 and 255 above which a cell is considered to be occupied.
		 * @return True if the waypoints can be connected, false otherwise.
		 */
		bool canConnect(const geometry_msgs::Point& w1, const geometry_msgs::Point& w2, int threshold) const;


	public:

		/* constructor */
		RPRoadmapServer(ros::NodeHandle &nh);

		/* callbacks to maps and odom */
		void odomCallback( const nav_msgs::OdometryConstPtr& msg );
		void costMapCallback( const nav_msgs::OccupancyGridConstPtr& msg );

		/* service to (re)generate waypoints */
		bool generateRoadmap(rosplan_knowledge_msgs::CreatePRM::Request &req, rosplan_knowledge_msgs::CreatePRM::Response &res);
		bool addWaypoint(rosplan_knowledge_msgs::AddWaypoint::Request &req, rosplan_knowledge_msgs::AddWaypoint::Response &res);
		bool removeWaypoint(rosplan_knowledge_msgs::RemoveWaypoint::Request &req, rosplan_knowledge_msgs::RemoveWaypoint::Response &res);

		void createPRM(nav_msgs::OccupancyGrid map, unsigned int nr_waypoints, double min_distance, double casting_distance, double connecting_distance, int occupancy_threshold, int total_attempts);
	};
}
#endif
