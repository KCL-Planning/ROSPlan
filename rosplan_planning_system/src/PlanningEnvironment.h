/**
 * This file describes the planning environment.
 * The domain is parsed from the PDDL domain file.
 * The problem instance is fetched from the Knowledge Base.
 */
#include "ros/ros.h"
#include "rosplan_knowledge_msgs/InstanceService.h"
#include "rosplan_knowledge_msgs/AttributeService.h"
#include "VALfiles/ptree.h"
#include "FlexLexer.h"
#include <string>

#ifndef KCL_environment
#define KCL_environment

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

		/* PDDL to Knowledge Base naming map */
		std::map<std::string,std::string> name_map;
		std::string toLowerCase(const std::string& str);

		/* domain information */
		std::string domainName;
		std::vector<std::string> domainTypes;
		std::map<std::string,std::vector<std::string> > domainPredicates;
		std::map<std::string,std::vector<std::string> > domainOperators;
		std::map<std::string,std::vector<std::string> > domainOperatorTypes;
		std::map<std::string,std::vector<std::string> > domainFunctions;

		// maps operator name to a list of preconditions; stored as [pred_name, label_0, label_1, ...]
		std::map<std::string, std::vector<std::vector<std::string> > > domainOperatorPreconditionMap;

		/* problem information */
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> instanceAttributes;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> domainAttributes;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> goalAttributes;

		// maps object type name to list of object instance names
		std::map<std::string,std::vector<string> > typeObjectMap;

		// maps instance name to type name
		std::map<std::string,std::string> objectTypeMap;

		/* domain parsing */
		void clearDomain();
		void parsePrecondition(const std::string &opName, const VAL::goal* goal, bool negative);
		
		public:

		/* domain parsing */		
		void parseDomain(const std::string domainPath);

		/* problem instance building */
		void update(ros::NodeHandle nh);
		void clear();
	};
} // close namespace

#endif
