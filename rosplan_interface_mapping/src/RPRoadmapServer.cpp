#include "rosplan_interface_mapping/RPRoadmapServer.h"
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/coordinate_conversions.h>

/* implementation of rosplan_interface_mapping::RPRoadmapServer */
namespace KCL_rosplan {

	/* constructor */
	RPRoadmapServer::RPRoadmapServer(ros::NodeHandle &nh)
	 : message_store(nh) {

		// config
		std::string dataPath("common/");
		std::string staticMapService("/static_map");
		nh.param("/rosplan/data_path", data_path, dataPath);
		nh.param("static_map_service", static_map_service, staticMapService);
		nh.param("use_static_map", use_static_map, false);

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		
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
	 * Check if two waypoints can be connected without colliding with any known scenery. The line should not
	 * come closer than @ref{min_width} than any known obstacle.
	 * @param w1 The first waypoint.
	 * @param w2 The second waypoint.
	 * @param threshold A value between -1 and 255 above which a cell is considered to be occupied.
	 * @return True if the waypoints can be connected, false otherwise.
	 */
	bool RPRoadmapServer::canConnect(const geometry_msgs::Point& w1, const geometry_msgs::Point& w2, int threshold) const {

		// Check if the turtlebot is going to collide with any known obstacle.
		occupancy_grid_utils::RayTraceIterRange ray_range = occupancy_grid_utils::rayTrace(cost_map.info, w1, w2);
		
		for (occupancy_grid_utils::RayTraceIterator i = ray_range.first; i != ray_range.second; ++i)
		{
			const occupancy_grid_utils::Cell& cell = *i;

			// Check if this cell is occupied.
			if (cost_map.data[cell.x + cell.y * cost_map.info.width] > threshold)
			{
				return false;
			}
		}
		return true;
	}
	
	/**
	 * Generates waypoints and stores them in the knowledge base and scene database
	 */
	bool RPRoadmapServer::generateRoadmap(rosplan_knowledge_msgs::CreatePRM::Request &req, rosplan_knowledge_msgs::CreatePRM::Response &res) {

		ros::NodeHandle nh("~");

		// clear previous roadmap from knowledge base
		ROS_INFO("KCL: (RPRoadmapServer) Cleaning old roadmap");
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		updateSrv.request.knowledge.instance_type = "waypoint";
		update_knowledge_client.call(updateSrv);

		// clear previous roadmap from scene database
		for (std::map<std::string,Waypoint*>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
			message_store.deleteID(db_name_map[wit->first]);
		}
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
		/*
		 * nr_waypoints, number of waypoints to generate before function terminates.
		 * min_distance, the minimum distance allowed between any pair of waypoints.
		 * casting_distance, the maximum distance a waypoint can be cast.
		 * connecting_distance, the maximum distance that can exists between waypoints for them to be connected.
		 * occupancy_threshold, a number between 0 and 255; determines above which value a cell is considered occupied.
		 */
		createPRM(map, req.nr_waypoints, req.min_distance, req.casting_distance, req.connecting_distance, req.occupancy_threshold, req.total_attempts);

		// publish visualization
		publishWaypointMarkerArray(nh);
		publishEdgeMarkerArray(nh);

		// add roadmap to knowledge base and scene database
		ROS_INFO("KCL: (RPRoadmapServer) Adding knowledge");
		for (std::map<std::string,Waypoint*>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {

			// instance
			rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			updateSrv.request.knowledge.instance_type = "waypoint";
			updateSrv.request.knowledge.instance_name = wit->first;
			update_knowledge_client.call(updateSrv);

			res.waypoints.push_back(wit->first);
			
			// predicates
			for (std::vector<std::string>::iterator nit=wit->second->neighbours.begin(); nit!=wit->second->neighbours.end(); ++nit) {
				rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
				updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				updatePredSrv.request.knowledge.attribute_name = "connected";
				diagnostic_msgs::KeyValue pairFrom;
				pairFrom.key = "from";
				pairFrom.value = wit->first;
				updatePredSrv.request.knowledge.values.push_back(pairFrom);
				diagnostic_msgs::KeyValue pairTo;
				pairTo.key = "to";
				pairTo.value = *nit;
				updatePredSrv.request.knowledge.values.push_back(pairTo);
				update_knowledge_client.call(updatePredSrv);	
			}

			// functions
			for (std::vector<std::string>::iterator nit=wit->second->neighbours.begin(); nit!=wit->second->neighbours.end(); ++nit) {
				rosplan_knowledge_msgs::KnowledgeUpdateService updateFuncSrv;
				updateFuncSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updateFuncSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
				updateFuncSrv.request.knowledge.attribute_name = "distance";
				diagnostic_msgs::KeyValue pairFrom;
				pairFrom.key = "wp1";
				pairFrom.value = wit->first;
				updateFuncSrv.request.knowledge.values.push_back(pairFrom);
				diagnostic_msgs::KeyValue pairTo;
				pairTo.key = "wp2";
				pairTo.value = *nit;
				updateFuncSrv.request.knowledge.values.push_back(pairTo);
				double dist = sqrt(
						(wit->second->real_x - waypoints[*nit]->real_x)*(wit->second->real_x - waypoints[*nit]->real_x)
						+ (wit->second->real_y - waypoints[*nit]->real_y)*(wit->second->real_y - waypoints[*nit]->real_y));
				updateFuncSrv.request.knowledge.function_value = dist;
				update_knowledge_client.call(updateFuncSrv);
			}

			//data
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = map.header.frame_id;
			pose.pose.position.x = wit->second->real_x;
			pose.pose.position.y = wit->second->real_y;
			pose.pose.position.z = 0.0;
			pose.pose.orientation.x = 0.0;;
			pose.pose.orientation.y = 0.0;;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;
			std::string id(message_store.insertNamed(wit->first, pose));
			db_name_map[wit->first] = id;
		}

		// robot start position (TODO remove)
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "robot_at";
		diagnostic_msgs::KeyValue pair1, pair2;
		pair1.key = "v";
		pair1.value = "kenny";
		updateSrv.request.knowledge.values.push_back(pair1);
		pair2.key = "wp";
		pair2.value = "wp0";
		updateSrv.request.knowledge.values.push_back(pair2);
		update_knowledge_client.call(updateSrv);

		ROS_INFO("KCL: (RPRoadmapServer) Done");
		return true;
	}

	/*
	 * Input:
	 * 	nr_waypoints, number of waypoints to generate before function terminates.
	 * 	min_distance, the minimum distance allowed between any pair of waypoints.
	 * 	casting_distance, the maximum distance a waypoint can be cast.
	 * 	connecting_distance, the maximum distance that can exists between waypoints for them to be connected.
	 * 	occupancy_threshold, a number between 0 and 255; determines above which value a cell is considered occupied.
	 * Output: A roadmap G = (V, E)
	 */
	void RPRoadmapServer::createPRM(nav_msgs::OccupancyGrid map, unsigned int nr_waypoints, double min_distance, double casting_distance, double connecting_distance, int occupancy_threshold, int total_attempts) {

		// map info
		int width = map.info.width;
		int height = map.info.height;
		double resolution = map.info.resolution; // m per cell

		if(width==0 || height==0) {
			ROS_INFO("KCL: (RPRoadmapServer) Empty map");
			return;
		}

		// V <-- empty set; E <-- empty set.
		for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci) {
			delete (*ci).second;
		}
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

		occupancy_grid_utils::Cell start_cell = occupancy_grid_utils::pointCell(map.info, start_pose_transformed.pose.position);
		Waypoint* start_wp = new Waypoint("wp0", start_cell.x, start_cell.y, map.info);
		waypoints[start_wp->wpID] = start_wp;

		int loop_counter = 0;
		while(waypoints.size() < nr_waypoints && ++loop_counter < total_attempts) {

			// Sample a random waypoint.
			std::map<std::string, Waypoint*>::iterator item = waypoints.begin();
			std::advance(item, rand() % waypoints.size());
			Waypoint* casting_wp = (*item).second;
			
			// sample collision-free configuration at random
			int x = rand() % width;
			int y = rand() % height;

			std::stringstream ss;
			ss << "wp" << waypoints.size();
			Waypoint* wp = new Waypoint(ss.str(), x, y, map.info);
			
			// Move the waypoint closer so it's no further than @ref{casting_distance} away from the casting_wp.
			wp->update(*casting_wp, casting_distance, map.info);
			
			// Check whether this waypoint is connected to any of the existing waypoints.
			geometry_msgs::Point p1, p2;
			p1.x = wp->real_x;
			p1.y = wp->real_y;
			
			// Ignore waypoint that are too close to existing waypoints.
			float min_distance_to_other_wp = std::numeric_limits<float>::max();
			for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci) {
				float distance = wp->getDistance(*(*ci).second);
				if (distance < min_distance_to_other_wp) 
					min_distance_to_other_wp = distance;
			}
			
			if (min_distance_to_other_wp < min_distance) {
				delete wp;
				continue;
			}
			
			for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci) {
				Waypoint* other_wp = (*ci).second;
				p2.x = other_wp->real_x;
				p2.y = other_wp->real_y;
				
				if (wp->getDistance(*other_wp) < connecting_distance && canConnect(p1, p2, occupancy_threshold)) {
					wp->neighbours.push_back(other_wp->wpID);
					other_wp->neighbours.push_back(wp->wpID);
					Edge e(wp->wpID, other_wp->wpID);
					edges.push_back(e);
				}
			}
			
			if (wp->neighbours.size() > 0) {
				waypoints[wp->wpID] = wp;
			}
		}
	}

	/**
	 * Connects a new waypoint and stores it in the knowledge base and scene database
	 */
	bool RPRoadmapServer::addWaypoint(rosplan_knowledge_msgs::AddWaypoint::Request &req, rosplan_knowledge_msgs::AddWaypoint::Response &res) {

		ros::NodeHandle nh("~");
		
		ROS_INFO("KCL: (RPRoadmapServer) Adding new waypoint");
		
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

		// map info
		int width = map.info.width;
		int height = map.info.height;
		double resolution = map.info.resolution; // m per cell

		if(width==0 || height==0) {
			ROS_INFO("KCL: (RPRoadmapServer) Empty map");
			return false;
		}

		// clear old waypoint
		clearWaypoint(req.id);

		// add new waypoint
		occupancy_grid_utils::Cell start_cell = occupancy_grid_utils::pointCell(map.info, req.waypoint.pose.position);
		Waypoint* new_wp = new Waypoint(req.id, start_cell.x, start_cell.y, map.info);
		waypoints[new_wp->wpID] = new_wp;

		// connect to neighbours
		geometry_msgs::Point p1, p2;
		p1.x = new_wp->real_x;
		p1.y = new_wp->real_y;

		for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci) {
			Waypoint* other_wp = (*ci).second;
			p2.x = other_wp->real_x;
			p2.y = other_wp->real_y;
			
			std::cout << "Try to connect " << new_wp->wpID << " to " << other_wp->wpID << "." << std::endl;
			
			if (new_wp->getDistance(*other_wp) < req.connecting_distance && canConnect(p1, p2, req.occupancy_threshold)) {
				new_wp->neighbours.push_back(other_wp->wpID);
				other_wp->neighbours.push_back(new_wp->wpID);
				Edge e(new_wp->wpID, other_wp->wpID);
				edges.push_back(e);
			} else {
				std::cout << "Do not connect these waypoints because: ";
				if (new_wp->getDistance(*other_wp) < req.connecting_distance) {
					std::cout << "collision detected." << std::endl;
				} else {
					std::cout << "the distance between them is too large. " << new_wp->getDistance(*other_wp) << ">= " <<  req.connecting_distance << "." << std::endl;
				}
			}
		}


		// instance
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		updateSrv.request.knowledge.instance_type = "waypoint";
		updateSrv.request.knowledge.instance_name = new_wp->wpID;
		if (!update_knowledge_client.call(updateSrv)) {
			ROS_ERROR("Failed to add a new waypoint instance.");
			return false;
		}
		
		// publish visualization
		publishWaypointMarkerArray(nh);
		publishEdgeMarkerArray(nh);
		
		ROS_INFO("Process the %lu neighbours of this new waypoint.", new_wp->neighbours.size());
			
		// predicates
		for (std::vector<std::string>::iterator nit=new_wp->neighbours.begin(); nit!=new_wp->neighbours.end(); ++nit) {
			// connected new->old
			rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updatePredSrv.request.knowledge.attribute_name = "connected";
			diagnostic_msgs::KeyValue pairFrom;
			pairFrom.key = "from";
			pairFrom.value = new_wp->wpID;
			updatePredSrv.request.knowledge.values.push_back(pairFrom);
			diagnostic_msgs::KeyValue pairTo;
			pairTo.key = "to";
			pairTo.value = *nit;
			updatePredSrv.request.knowledge.values.push_back(pairTo);
			update_knowledge_client.call(updatePredSrv);

			// connected old->new
			updatePredSrv.request.knowledge.values.clear();
			pairFrom.value = *nit;
			updatePredSrv.request.knowledge.values.push_back(pairFrom);
			pairTo.value = new_wp->wpID;
			updatePredSrv.request.knowledge.values.push_back(pairTo);
			update_knowledge_client.call(updatePredSrv);	
		}

		// functions
		for (std::vector<std::string>::iterator nit=new_wp->neighbours.begin(); nit!=new_wp->neighbours.end(); ++nit) {

			// distance new->old
			rosplan_knowledge_msgs::KnowledgeUpdateService updateFuncSrv;
			updateFuncSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updateFuncSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
			updateFuncSrv.request.knowledge.attribute_name = "distance";
			diagnostic_msgs::KeyValue pairFrom;
			pairFrom.key = "wp1";
			pairFrom.value = new_wp->wpID;
			updateFuncSrv.request.knowledge.values.push_back(pairFrom);
			diagnostic_msgs::KeyValue pairTo;
			pairTo.key = "wp2";
			pairTo.value = *nit;
			updateFuncSrv.request.knowledge.values.push_back(pairTo);
			double dist = sqrt(
					(new_wp->real_x - waypoints[*nit]->real_x)*(new_wp->real_x - waypoints[*nit]->real_x)
					+ (new_wp->real_y - waypoints[*nit]->real_y)*(new_wp->real_y - waypoints[*nit]->real_y));
			updateFuncSrv.request.knowledge.function_value = dist;
			update_knowledge_client.call(updateFuncSrv);

			// distance old->new
			updateFuncSrv.request.knowledge.values.clear();			
			pairFrom.value = *nit;
			updateFuncSrv.request.knowledge.values.push_back(pairFrom);
			pairTo.value = new_wp->wpID;
			updateFuncSrv.request.knowledge.values.push_back(pairTo);
			updateFuncSrv.request.knowledge.function_value = dist;
			update_knowledge_client.call(updateFuncSrv);
		}

		//data
		std::string id(message_store.insertNamed(req.id, req.waypoint));
		db_name_map[new_wp->wpID] = id;

		ROS_INFO("KCL: (RPRoadmapServer) Finished adding new waypoint");

		return true;
	}

	bool RPRoadmapServer::removeWaypoint(rosplan_knowledge_msgs::RemoveWaypoint::Request &req, rosplan_knowledge_msgs::RemoveWaypoint::Response &res) {

		ros::NodeHandle nh("~");
		if ( clearWaypoint(req.id) ) {
			// publish visualization
			publishWaypointMarkerArray(nh);
			publishEdgeMarkerArray(nh);
		}
		return true;
	}

	bool RPRoadmapServer::clearWaypoint(const std::string &name) {

		if ( db_name_map.count( name ) == 0 ) {
			return false;
		}

		message_store.deleteID(db_name_map[name]);
		db_name_map.erase(name);
		waypoints.erase(name);

		std::vector<Edge>::iterator eit = edges.begin();
		while( eit != edges.end() ) {
			if ( name == eit->start || name == eit->end ) {
				eit = edges.erase( eit );
			} else {
				eit++;
			}
		}
		// remove instance
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		updateSrv.request.knowledge.instance_type = "waypoint";
		updateSrv.request.knowledge.instance_name = name;
		if (!update_knowledge_client.call(updateSrv)) {
			ROS_INFO("Failed to remove old waypoint instance.");
		}

		return true;

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
		ros::ServiceServer createPRMService = nh.advertiseService("/kcl_rosplan/roadmap_server/create_prm", &KCL_rosplan::RPRoadmapServer::generateRoadmap, &rms);
		ros::ServiceServer addWaypointService = nh.advertiseService("/kcl_rosplan/roadmap_server/add_waypoint", &KCL_rosplan::RPRoadmapServer::addWaypoint, &rms);
		ros::ServiceServer removeWaypointService = nh.advertiseService("/kcl_rosplan/roadmap_server/remove_waypoint", &KCL_rosplan::RPRoadmapServer::removeWaypoint, &rms);

		ROS_INFO("KCL: (RPRoadmapServer) Ready to receive. Cost map topic: %s", costMapTopic.c_str());
		ros::spin();
		return 0;
	}
