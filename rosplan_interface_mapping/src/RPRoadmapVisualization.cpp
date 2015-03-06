#include "rosplan_interface_mapping/RPRoadmapServer.h"

/* implementation of rosplan_interface_mapping::RPRoadmapServer visualisation */
namespace KCL_rosplan {

	/* output all waypoints */
	void RPRoadmapServer::publishWaypointMarkerArray(ros::NodeHandle nh)
	{
		visualization_msgs::MarkerArray marker_array;
		size_t counter = 0;
		for (std::map<std::string, Waypoint*>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
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

	/* output all edges */
	void RPRoadmapServer::publishEdgeMarkerArray(ros::NodeHandle nh)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "mission_edges";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::MODIFY;
		marker.scale.x = 0.05;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.3;
		marker.color.b = 1.0;
		for (std::vector<Edge>::iterator eit=edges.begin(); eit!=edges.end(); ++eit) {
			geometry_msgs::Point start;
			start.x = waypoints[eit->start]->real_x;
			start.y = waypoints[eit->start]->real_y;
			start.z = 0;
			marker.points.push_back(start);

			geometry_msgs::Point end;
			end.x = waypoints[eit->end]->real_x;
			end.y = waypoints[eit->end]->real_y;
			end.z = 0;
			marker.points.push_back(end);

		}
		edges_pub.publish( marker );
	}

	/* clears all waypoints and edges */
	void RPRoadmapServer::clearMarkerArrays(ros::NodeHandle nh)
	{
		visualization_msgs::MarkerArray marker_array;
		size_t counter = 0;
		for (std::map<std::string, Waypoint*>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();
			marker.ns = "mission_waypoint";
			marker.id = counter; counter++;
			marker.action = visualization_msgs::Marker::DELETE;
			marker_array.markers.push_back(marker);
		}
		waypoints_pub.publish( marker_array );

		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "mission_edges";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::DELETE;
		edges_pub.publish( marker );
	}
}
