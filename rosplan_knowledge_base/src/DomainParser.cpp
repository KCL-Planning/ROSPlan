#include "rosplan_knowledge_base/DomainParser.h"

/* implementation of DomainParser.h */
namespace KCL_rosplan {

	/*----------------*/
	/* parsing domain */
	/*----------------*/

	/**
	 * parse the domain file
	 */
	VAL::domain* DomainParser::parseDomain(const std::string domainPath) {

		// only parse domain once
		if(domain_parsed) return domain;
		domain_parsed = true;

		std::string domainFileName = (domainPath);
		ROS_INFO("KCL: (KB) Parsing domain: %s.", domainFileName.c_str());

		// save filename for VAL
		std::vector<char> writable(domainFileName.begin(), domainFileName.end());
		writable.push_back('\0');
		current_filename = &writable[0];

		// parse domain
		VAL::current_analysis = val_analysis = &VAL::an_analysis;
		std::ifstream domainFile;
		domainFile.open(domainFileName.c_str());
		yydebug = 0;

		VAL::yfl = new yyFlexLexer;

		if (domainFile.bad()) {
			ROS_ERROR("KCL: (KB) Failed to open domain file.");
			line_no = 0;
			VAL::log_error(VAL::E_FATAL,"Failed to open file");
		} else {
			line_no = 1;
			VAL::yfl->switch_streams(&domainFile, &std::cout);
			yyparse();

			// domain name
			domain = VAL::current_analysis->the_domain;
			domain_name = domain->name;
		}
		delete VAL::yfl;
		domainFile.close();

		return domain;

	}


	/*--------------------*/
	/* condition checking */
	/*--------------------*/

	bool PDDLGDAtomic::checkCondition(
			std::map<std::string,PDDLTypedSymbol> &parameters,
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFacts,
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFunctions) {
std::cout << "Atomic condition called" << std::endl;
		return false;		
	}

	bool PDDLGDFunction::checkCondition(
			std::map<std::string,PDDLTypedSymbol> &parameters,
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFacts,
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFunctions) {
std::cout << "Function condition called" << std::endl;
		return false;
	}

	bool PDDLGDConjunction::checkCondition(
			std::map<std::string,PDDLTypedSymbol> &parameters,
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFacts,
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFunctions) {
std::cout << "Conjunction condition called" << std::endl;

		std::vector<PDDLGoalDescription>::iterator git = goal_conditions.begin();
		switch(operand) {
			case AND:
				while(git!=goal_conditions.end()) {
					if(!git->checkCondition(parameters, modelFacts, modelFunctions)) return false;
					git++;
				}
				return true;
			case OR:
				while(git!=goal_conditions.end()) {
					if(git->checkCondition(parameters, modelFacts, modelFunctions)) return true;
					git++;
				}
				return false;
			case NOT:
				if(git!=goal_conditions.end())
					return (!git->checkCondition(parameters, modelFacts, modelFunctions));
				return false;
		}
		return false;
	}

	bool PDDLGDTimed::checkCondition(
			std::map<std::string,PDDLTypedSymbol> &parameters,
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFacts,
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFunctions) {
		return false;
	}

} // close namespace
