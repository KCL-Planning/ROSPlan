
#include "rosplan_knowledge_base/RDDLTaskParser.h"

/* implementation of RDDLTaskParser.h */
namespace KCL_rosplan {

	/*----------------*/
	/* parsing domain */
	/*----------------*/

	/**
	 * parse the domain file
	 */
    RDDLTask *RDDLTaskParser::parseTask(const std::string &domainPath, const std::string &instancePath, bool reload) {
		// only parse domain once
		if (domain_parsed and not reload) return rddlTask;
        ROS_INFO("KCL: (%s) Parsing domain: %s.", ros::this_node::getName().c_str(), domainPath.c_str());
        if (not instancePath.empty())
            ROS_INFO("KCL: (%s) Parsing initial state: %s", ros::this_node::getName().c_str(), instancePath.c_str());
        else ROS_INFO("KCL: (%s) Parsing initial state", ros::this_node::getName().c_str());

        // parse domain
        rddlTask = RDDLParser::parseRDDLTask(domainPath, instancePath);
        if (not rddlTask) {
            ROS_ERROR("KCL: (%s) Failed to open domain file.", ros::this_node::getName().c_str());
            return nullptr;
        }
        domain_name = rddlTask->domainName;
        problem_name = rddlTask->name;

        ROS_INFO("KCL: (%s) Pre-processing domain...", ros::this_node::getName().c_str());
        uninstantiated_SACs = rddlTask->SACs;
        Instantiator instantiator(rddlTask);
        instantiator.instantiate(false);
        Preprocessor preprocessor(rddlTask, false); // ipc2018 to false
        preprocessor.preprocess(false);
        ROS_INFO("KCL: (%s) RDDL domain is ready!", ros::this_node::getName().c_str());

        domain_parsed = true;

		return rddlTask;
	}

} // close namespace


