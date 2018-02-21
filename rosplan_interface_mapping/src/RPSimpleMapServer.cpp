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
		nh.param("waypoint_file", filename, filename);
		nh.param("fixed_frame", fixed_frame, fixed_frame);

		// init
		KCL_rosplan::RPSimpleMapServer sms(nh, fixed_frame);
		sms.setupRoadmap(filename);

		ROS_INFO("KCL: (RPSimpleMapServer) Ready to receive.");
		ros::spin();
		return 0;
	}
