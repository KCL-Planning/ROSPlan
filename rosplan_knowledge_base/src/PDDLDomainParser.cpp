#include "rosplan_knowledge_base/PDDLDomainParser.h"


namespace VAL {
	yyFlexLexer* yfl;
    analysis* current_analysis=nullptr;
    parse_category* top_thing=nullptr;
};
char * current_filename;

/* implementation of PDDLDomainParser.h */
namespace KCL_rosplan {

	/*----------------*/
	/* parsing domain */
	/*----------------*/

	/**
	 * parse the domain file
	 */
	VAL::domain* PDDLDomainParser::parseDomain(const std::string domainPath) {
		// only parse domain once
		if(domain_parsed) return domain;
		domain_parsed = true;

		std::string domainFileName = (domainPath);
		ROS_INFO("KCL: (%s) Parsing domain: %s.", ros::this_node::getName().c_str(), domainFileName.c_str());

		// save filename for VAL
		std::vector<char> writable(domainFileName.begin(), domainFileName.end());
		writable.push_back('\0');
		current_filename = &writable[0];

		// parse domain
		VAL::current_analysis = val_analysis ;
		std::ifstream domainFile;
		domainFile.open(domainFileName.c_str());
		yydebug = 0;

		VAL::yfl = new yyFlexLexer;

		if (domainFile.bad()) {
			ROS_ERROR("KCL: (%s) Failed to open domain file.", ros::this_node::getName().c_str());
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

} // close namespace
