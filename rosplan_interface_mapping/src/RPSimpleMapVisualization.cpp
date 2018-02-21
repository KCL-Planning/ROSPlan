#include "rosplan_interface_mapping/RPSimpleMapServer.h"

/* implementation of rosplan_interface_mapping::RPSimpleMapServer visualisation */
namespace KCL_rosplan {

	/* output all waypoints */
	void RPSimpleMapServer::publishWaypointMarkerArray(ros::NodeHandle nh)
	{
		visualization_msgs::MarkerArray marker_array;
		size_t counter = 0;
		for (std::map<std::string, Waypoint*>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = fixed_frame;
			marker.header.stamp = ros::Time();
			marker.ns = "mission_waypoint";
			marker.id = counter; counter++;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::MODIFY;
			marker.pose.position.x = wit->second->real_x;
			marker.pose.position.y = wit->second->real_y;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
			marker.color.a = 1.0;
			marker.color.r = 0.3;
			marker.color.g = 1.0;
			marker.color.b = 0.3;
			marker.text = wit->first;
			marker_array.markers.push_back(marker);
		}
		waypoints_pub.publish( marker_array );
	}
	/* clears all waypoints and edges */
	void RPSimpleMapServer::clearMarkerArrays(ros::NodeHandle nh)
	{
		visualization_msgs::MarkerArray marker_array;
		size_t counter = 0;
		for (std::map<std::string, Waypoint*>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = fixed_frame;
			marker.header.stamp = ros::Time();
			marker.ns = "mission_waypoint";
			marker.id = counter; counter++;
			marker.action = visualization_msgs::Marker::DELETE;
			marker_array.markers.push_back(marker);
		}
		waypoints_pub.publish( marker_array );
	}
}
