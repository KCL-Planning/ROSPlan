#include "ros/ros.h"
#include "rosplan_knowledge_base/RPRoadmapServer.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h> 
#include <algorithm> 
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "mongodb_store/message_store.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "diagnostic_msgs/KeyValue.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

namespace KCL_rosplan {

	/* constructor */
	RPRoadmapServer::RPRoadmapServer(ros::NodeHandle &nh)
	 : message_store(nh) {

		// config
		std::string dataPath("common/");
		std::string staticMapService("/static_map");
		nh.param("data_path", data_path, dataPath);
		nh.param("static_map_service", static_map_service, staticMapService);
		nh.param("use_static_map", use_static_map, false);

		// knowledge interface
		add_knowledge_pub = nh.advertise<rosplan_knowledge_msgs::KnowledgeItem>("/kcl_rosplan/add_knowledge", 10, true);
		remove_knowledge_pub = nh.advertise<rosplan_knowledge_msgs::KnowledgeItem>("/kcl_rosplan/remove_knowledge", 10, true);

		// visualisation
		waypoints_pub = nh.advertise<visualization_msgs::MarkerArray>("/kcl_rosplan/viz/waypoints", 10, true);
		edges_pub = nh.advertise<visualization_msgs::Marker>("/kcl_rosplan/viz/edges", 10, true);

		// map interface
		map_client = nh.serviceClient<nav_msgs::GetMap>(static_map_service);
	}

	/*------------------*/
	/* callback methods */
	/*------------------*/

	/* update the costmap */
	void RPRoadmapServer::costMapCallback( const nav_msgs::OccupancyGridConstPtr& msg ) {
		cost_map = *msg;
	}

	/* update position of the robot */
	void RPRoadmapServer::odomCallback( const nav_msgs::OdometryConstPtr& msg ) {
		//we assume that the odometry is published in the frame of the base
		base_odom.header = msg->header;
		base_odom.pose.position = msg->pose.pose.position;
		base_odom.pose.orientation = msg->pose.pose.orientation;
	}

	/*-----------*/
	/* build PRM */
	/*-----------*/

	/**
	 * Generates waypoints and stores them in the knowledge base and scene database
	 */
	bool RPRoadmapServer::generateRoadmap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		ros::NodeHandle nh("~");

		// clear previous roadmap from knowledge base
		ROS_INFO("KCL: (RPRoadmapServer) Cleaning old roadmap");
		rosplan_knowledge_msgs::KnowledgeItem clearWP;
		clearWP.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		clearWP.instance_type = "waypoint";
		remove_knowledge_pub.publish(clearWP);

		rosplan_knowledge_msgs::KnowledgeItem clearConn;
		clearConn.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		clearConn.attribute_name = "connected";
		remove_knowledge_pub.publish(clearConn);

		rosplan_knowledge_msgs::KnowledgeItem clearDist;
		clearDist.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_FUNCTION;
		clearDist.attribute_name = "distance";
		remove_knowledge_pub.publish(clearDist);

		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit)
			message_store.deleteID(db_name_map[wit->first]);
		db_name_map.clear();

		// clear from visualization
		clearMarkerArrays(nh);
 
		// read map
		nav_msgs::OccupancyGrid map;
		if(use_static_map) {
			ROS_INFO("KCL: (RPRoadmapServer) Reading in map");
			nav_msgs::GetMap mapSrv;
			map_client.call(mapSrv);
			map = mapSrv.response.map;
		} else {
			map = cost_map;
		}

		// generate waypoints
		ROS_INFO("KCL: (RPRoadmapServer) Generating roadmap");
		/* K, number of seed waypoints
		 * D, distance of random motions
		 * R, radius of random connections
		 * M, max number of waypoints */
		createPRM(map, 1, 1, 5, 10);

		// publish visualization
		publishWaypointMarkerArray(nh);
		publishEdgeMarkerArray(nh);

		// add roadmap to knowledge base and scene database
		ROS_INFO("KCL: (RPRoadmapServer) Adding knowledge");
		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {

			// instance
			rosplan_knowledge_msgs::KnowledgeItem addWP;
			addWP.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			addWP.instance_type = "waypoint";
			addWP.instance_name = wit->first;
			add_knowledge_pub.publish(addWP);

			// predicates
			for (std::vector<std::string>::iterator nit=wit->second.neighbours.begin(); nit!=wit->second.neighbours.end(); ++nit) {
				rosplan_knowledge_msgs::KnowledgeItem addConn;
				addConn.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
				addConn.attribute_name = "connected";
				diagnostic_msgs::KeyValue pairFrom;
				pairFrom.key = "from";
				pairFrom.value = wit->first;
				addConn.values.push_back(pairFrom);
				diagnostic_msgs::KeyValue pairTo;
				pairTo.key = "to";
				pairTo.value = *nit;
				addConn.values.push_back(pairTo);
				add_knowledge_pub.publish(addConn);			
			}

			// functions
			for (std::vector<std::string>::iterator nit=wit->second.neighbours.begin(); nit!=wit->second.neighbours.end(); ++nit) {
				rosplan_knowledge_msgs::KnowledgeItem addDist;
				addDist.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_FUNCTION;
				addDist.attribute_name = "distance";
				diagnostic_msgs::KeyValue pairFrom;
				pairFrom.key = "wp1";
				pairFrom.value = wit->first;
				addDist.values.push_back(pairFrom);
				diagnostic_msgs::KeyValue pairTo;
				pairTo.key = "wp2";
				pairTo.value = *nit;
				addDist.values.push_back(pairTo);

				/* TODO distance, and make optional
				double dist = sqrt(
						(wit->second.real_x - nit->second.real_x)*(wit->second.real_x - nit->second.real_x)
						+ (wit->second.real_y - nit->second.real_y)*(wit->second.real_y - nit->second.real_y));
				*/
				addDist.function_value = 1;

				add_knowledge_pub.publish(addDist);
			}

			//data
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = map.header.frame_id;
			pose.pose.position.x = wit->second.real_x;
			pose.pose.position.y = wit->second.real_y;
			pose.pose.position.z = 0.0;
			std::string id(message_store.insertNamed(wit->first, pose));
			db_name_map[wit->first] = id;
		}

		// robot start position (TODO name)
		rosplan_knowledge_msgs::KnowledgeItem addPosition;
		addPosition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		addPosition.attribute_name = "robot_at";
		diagnostic_msgs::KeyValue pair1, pair2;
		pair1.key = "v";
		pair1.value = "kenny";
		addPosition.values.push_back(pair1);
		pair2.key = "wp";
		pair2.value = "wp0";
		addPosition.values.push_back(pair2);
		add_knowledge_pub.publish(addPosition);


		ROS_INFO("KCL: (RPRoadmapServer) Done");
		return true;
	}

	/*
	 * Input:
	 * 	K, number of seed waypoints
	 * 	D, distance of random motions
	 * 	R, radius of random connections
	 * 	M, max number of waypoints
	 * Output: A roadmap G = (V, E)
	*/
	void RPRoadmapServer::createPRM(nav_msgs::OccupancyGrid map, unsigned int K, double D, double R, unsigned int M) {

		// map info
		int width = map.info.width;
		int height = map.info.height;
		double resolution = map.info.resolution; // m per cell

		if(width==0 || height==0) {
			ROS_INFO("KCL: (RPRoadmapServer) Empty map");
			return;
		}

		// V <-- empty set; E <-- empty set.
		waypoints.clear();
		edges.clear();

		// create robot start point
		geometry_msgs::PoseStamped start_pose;
		geometry_msgs::PoseStamped start_pose_transformed;
		start_pose.header = base_odom.header;
		start_pose.pose.position = base_odom.pose.position;
		start_pose.pose.orientation = base_odom.pose.orientation; 
		try {
			tf.waitForTransform( base_odom.header.frame_id, "map", ros::Time::now(), ros::Duration( 500 ) );
			tf.transformPose( "map", start_pose,  start_pose_transformed);
		} catch(tf::LookupException& ex) {
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return;
		} catch(tf::ConnectivityException& ex) {
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return;
		} catch(tf::ExtrapolationException& ex) {
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return;
		}

		int startX = (int)((start_pose_transformed.pose.position.x - map.info.origin.position.x) / resolution);
		int startY = (int)((start_pose_transformed.pose.position.y - map.info.origin.position.y) / resolution);
		Waypoint wp("wp0", startX, startY, resolution, map.info.origin);
		waypoints[wp.wpID] = wp;

		// while cardinality(V) < K do
		while(waypoints.size() < K) {

			// sample collision-free configuration at random
			int x = rand() % width;
			int y = rand() % height;

			if(map.data[ (width*y + x) ] <= 65 && map.data[ (width*y + x) ] >= 0) {
				std::stringstream ss;
				ss << "wp" << waypoints.size();
				Waypoint wp(ss.str(), x, y, resolution, map.info.origin);
				waypoints[wp.wpID] = wp;
			}
		}

		// TODO make connected proper
		while( waypoints.size() < M) { // || (!allConnected() && waypoints.size() < M)) {

			for(size_t grow=0; grow<3; grow++) {
				// get random point
				int x = rand() % width;
				int y = rand() % height;
				while(map.data[ (width*y + x) ] > 0) {
					x = rand() % width;
					y = rand() % height;
				}

				// find closest waypoint from V
				double dist = -1;
				Waypoint closest;
				for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
					Waypoint w = wit->second;
					double d = sqrt( ((x - w.grid_x)*(x - w.grid_x)) + ((y - w.grid_y)*(y - w.grid_y)) );
					if(dist < 0 || d < dist) {
						closest = w;
						dist = d;
					}
				}
				if(dist<0) ROS_INFO("KCL: (RPRoadmapServer) Error in RRT (can't find closest neighbour)");

				// new point
				int xNew = closest.grid_x + (int)((D/resolution)*(x-closest.grid_x)/dist);
				int yNew = closest.grid_y + (int)((D/resolution)*(y-closest.grid_y)/dist);
				if(xNew<0) xNew = 1;
				if(yNew<0) yNew = 1;
				if(xNew>width) xNew = width-1;
				if(yNew>height) yNew = height-1;

				// (check collision and) add to waypoints
				if(map.data[ (width*yNew + xNew) ] <= 65 && map.data[ (width*yNew + xNew) ] >= 0) {
					std::stringstream ss;
					ss << "wp" << waypoints.size();
					Waypoint wpNew(ss.str(), xNew, yNew, resolution, map.info.origin);
					waypoints[wpNew.wpID] = wpNew;
					closest.neighbours.push_back(wpNew.wpID);
					wpNew.neighbours.push_back(closest.wpID);
					Edge e(closest.wpID, wpNew.wpID);
					edges.push_back(e);

					// try and connect things up
					makeConnections(R);
				}
			}
		}
	}

	/* returns true if waypoints form a single connected component */
	bool RPRoadmapServer::allConnected() {

		if(waypoints.size()<1) return true;

		std::map<std::string,bool> connected;
		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit)
			connected[wit->first] = false;

		connectRecurse(connected, waypoints["wp0"]);
		
		int count = 0;
		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit)
			if(connected[wit->first]) count++;
		return (count == waypoints.size());
	}

	/* recursive call from allConnected */
	void RPRoadmapServer::connectRecurse(std::map<std::string,bool> &connected, Waypoint &waypoint) {
		if(connected[waypoint.wpID]) return;
		connected[waypoint.wpID] = true;
		for(size_t i=0; i<waypoint.neighbours.size(); i++)
			if(!connected[waypoints[waypoint.neighbours[i]].wpID])
				connectRecurse(connected, waypoints[waypoint.neighbours[i]]);
	}

	/* attempts to make new connections */
	bool RPRoadmapServer::makeConnections(unsigned int R) {

		bool newEdges = false;
		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
		for (std::map<std::string,Waypoint>::iterator sit=waypoints.begin(); sit!=waypoints.end(); ++sit) {
			if(wit->first.compare(sit->first)==0 || find(wit->second.neighbours.begin(), wit->second.neighbours.end(), sit->first) != wit->second.neighbours.end())
				continue;
			Waypoint w = wit->second;
			Waypoint s = sit->second;
			double d = sqrt( ((s.grid_x - w.grid_x)*(s.grid_x - w.grid_x)) + ((s.grid_y - w.grid_y)*(s.grid_y - w.grid_y)) );
			if(d<R) {
				wit->second.neighbours.push_back(s.wpID);
				sit->second.neighbours.push_back(w.wpID);
				Edge e(w.wpID, s.wpID);
				edges.push_back(e);
				newEdges = true;
			}
		}};
		return newEdges;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		// setup ros
		ros::init(argc, argv, "rosplan_roadmap_server");
		ros::NodeHandle nh("~");

		// params
		std::string costMapTopic("/move_base/local_costmap/costmap");
		nh.param("cost_map_topic", costMapTopic, costMapTopic);
		std::string odomTopic("/odom");
		nh.param("odom_topic", odomTopic, odomTopic);

		// init
		KCL_rosplan::RPRoadmapServer rms(nh);
		ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odomTopic, 1, &KCL_rosplan::RPRoadmapServer::odomCallback, &rms);
		ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(costMapTopic, 1, &KCL_rosplan::RPRoadmapServer::costMapCallback, &rms);
		ros::ServiceServer service = nh.advertiseService("/kcl_rosplan/roadmap_server", &KCL_rosplan::RPRoadmapServer::generateRoadmap, &rms);

		ROS_INFO("KCL: (RPRoadmapServer) Ready to receive");
		ros::spin();
		return 0;
	}
