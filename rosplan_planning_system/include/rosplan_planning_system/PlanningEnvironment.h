/**
 * This file describes the planning environment.
 * The domain is parsed from the PDDL domain file.
 * The problem instance is fetched from the Knowledge Base.
 */
#ifndef KCL_environment
#define KCL_environment

#include "ros/ros.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "../../src/VALfiles/ptree.h"
#include "FlexLexer.h"
#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include <iostream>
#include <rosplan_knowledge_base/DomainParser.h>

extern int yyparse();
extern int yydebug;

namespace VAL {
	extern analysis an_analysis;
	extern yyFlexLexer* yfl;
};

namespace KCL_rosplan
{

	class PlanningEnvironment
	{
	private:

		std::string toLowerCase(const std::string& str);
		
	public:

		DomainParser domain_parser;

		/* PDDL to Knowledge Base naming map */
		std::map<std::string,std::string> name_map;

		/* problem information */
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> domain_attributes;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> goal_attributes;

		// maps object type name to list of object instance names
		std::map<std::string,std::vector<string> > type_object_map;

		// maps instance name to type name
		std::map<std::string,std::string> object_type_map;

		/* problem instance building */
		void update(ros::NodeHandle nh);
		void clear();
	};
} // close namespace

#endif
