
#include "rosplan_knowledge_base/RDDLTaskParser.h"

/* implementation of RDDLTaskParser.h */
namespace KCL_rosplan {

	/*----------------*/
	/* parsing domain */
	/*----------------*/

	/**
	 * parse the domain file
	 */
    RDDLTask *RDDLTaskParser::parseTask(const std::string &domainPath, const std::string &instancePath) {
		// only parse domain once
		if (domain_parsed) return rddlTask;
        ROS_INFO("KCL: (%s) Parsing domain: %s.", ros::this_node::getName().c_str(), domainPath.c_str());
        ROS_INFO("KCL: (%s) Parsing initial state", ros::this_node::getName().c_str());

        // parse domain
        rddlTask = RDDLParser::parseRDDLTask(domainPath, instancePath);
        if (not rddlTask) {
            ROS_ERROR("KCL: (%s) Failed to open domain file.", ros::this_node::getName().c_str());
            return nullptr;
        }
        domain_name = rddlTask->domainName;
        problem_name = rddlTask->name;
        domain_parsed = true;

		return rddlTask;
	}

} // close namespace


