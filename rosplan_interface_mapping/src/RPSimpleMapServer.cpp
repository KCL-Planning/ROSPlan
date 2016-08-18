#include "rosplan_interface_mapping/RPSimpleMapServer.h"

#include <tf/transform_listener.h>

/* implementation of rosplan_interface_mapping::RPSimpleMapServer.h */
namespace KCL_rosplan {

	/* constructor */
	RPSimpleMapServer::RPSimpleMapServer(ros::NodeHandle &nh, std::string frame)
	 : message_store(nh), fixed_frame(frame) {

		// config
		std::string dataPath("common/");
		nh.param("/rosplan/data_path", data_path, dataPath);

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		// visualisation
		waypoints_pub = nh.advertise<visualization_msgs::MarkerArray>("/kcl_rosplan/viz/waypoints", 10, true);
	}

	/*-----------*/
	/* build PRM */
	/*-----------*/

	/**
	 * Generates waypoints and stores them in the knowledge base and scene database
	 */
	bool RPSimpleMapServer::generateRoadmap(rosplan_knowledge_msgs::CreatePRM::Request &req, rosplan_knowledge_msgs::CreatePRM::Response &res) {

		ros::NodeHandle nh("~");

		// clear previous roadmap from knowledge base
		ROS_INFO("KCL: (RPSimpleMapServer) Cleaning old roadmap");
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

		// generate waypoints
		ROS_INFO("KCL: (RPSimpleMapServer) Generating roadmap");


		for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci)
			delete (*ci).second;
		waypoints.clear();

		std::srand(std::time(0));
		for(int i=0; i<req.nr_waypoints; i++) {
			int x = (rand()%15);
			int y = (rand()%15);
			std::stringstream ss;
			ss << "wp" << i;
			Waypoint* wp = new Waypoint(ss.str(), x, y);
			waypoints[wp->wpID] = wp;
		}

		// publish visualization
		publishWaypointMarkerArray(nh);

		// add roadmap to knowledge base and scene database
		ROS_INFO("KCL: (RPSimpleMapServer) Adding knowledge");
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
			pose.header.frame_id = fixed_frame;
			pose.pose.position.x = wit->second->real_x;
			pose.pose.position.y = wit->second->real_y;
			pose.pose.position.z = 0.0;
			pose.pose.orientation.x = 0.0;;
			pose.pose.orientation.y = 0.0;;
			pose.pose.orientation.z = 1.0;
			pose.pose.orientation.w = 1.0;
			std::string id(message_store.insertNamed(wit->first, pose));
			db_name_map[wit->first] = id;
		}

		ROS_INFO("KCL: (RPSimpleMapServer) Done");
		return true;
	}

	/**
	 * parses a pose with yaw from strings: "[f, f, f]"
	 */
	 void RPSimpleMapServer::parsePose(geometry_msgs::PoseStamped &pose, std::string line) {

		int curr,next;
		curr=line.find("[")+1;
		next=line.find(",",curr);

		pose.pose.position.x = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);

		pose.pose.position.y = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find("]",curr);

		float theta = atof(line.substr(curr,next-curr).c_str());
		tf::Quaternion q;
		q.setEuler(theta, 0 ,0);


		pose.pose.orientation.x = q.x();
		pose.pose.orientation.y = q.z();
		pose.pose.orientation.w = q.w();
		pose.pose.orientation.z = q.y();
	}

	bool RPSimpleMapServer::setupRoadmap(std::string filename) {

		ros::NodeHandle nh("~");

		// clear previous roadmap from knowledge base
		ros::service::waitForService("/kcl_rosplan/update_knowledge_base", ros::Duration(20));
		ROS_INFO("KCL: (RPSimpleMapServer) Loading roadmap from file");

		// load configuration file
		std::ifstream infile(filename.c_str());
		std::string line;
		int curr,next;
		while(std::getline(infile, line)) {
			// read waypoint
			curr=line.find("[");
			std::string name = line.substr(0,curr);

			// instance
			rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			updateSrv.request.knowledge.instance_type = "waypoint";
			updateSrv.request.knowledge.instance_name = name;
			update_knowledge_client.call(updateSrv);

			// data
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = fixed_frame;
			parsePose(pose, line);
			std::string id(message_store.insertNamed(name, pose));
			db_name_map[name] = id;

			// save here for viz
			Waypoint* wp = new Waypoint(name, pose.pose.position.x, pose.pose.position.y);
			waypoints[wp->wpID] = wp;
		}
		infile.close();

		// publish visualization
		publishWaypointMarkerArray(nh);
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		// setup ros
		ros::init(argc, argv, "rosplan_simple_map_server");
		ros::NodeHandle nh("~");

		// params
		std::string filename("waypoints.txt");
		std::string fixed_frame("map");
		nh.param("/waypoint_file", filename, filename);
		nh.param("fixed_frame", fixed_frame, fixed_frame);

		// init
		KCL_rosplan::RPSimpleMapServer sms(nh, fixed_frame);
		ros::ServiceServer createPRMService = nh.advertiseService("/kcl_rosplan/roadmap_server/create_prm", &KCL_rosplan::RPSimpleMapServer::generateRoadmap, &sms);
		sms.setupRoadmap(filename);

		ROS_INFO("KCL: (RPSimpleMapServer) Ready to receive.");
		ros::spin();
		return 0;
	}
