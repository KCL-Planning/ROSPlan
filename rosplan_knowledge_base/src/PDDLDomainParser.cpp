#include "rosplan_knowledge_base/PDDLDomainParser.h"


namespace VAL1_2 {
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
	VAL1_2::domain* PDDLDomainParser::parseDomain(const std::string domainPath) {
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
		VAL1_2::current_analysis = val_analysis ;
		std::ifstream domainFile;
		domainFile.open(domainFileName.c_str());
		yydebug = 0;

		VAL1_2::yfl = new yyFlexLexer;

		if (domainFile.bad()) {
			ROS_ERROR("KCL: (%s) Failed to open domain file.", ros::this_node::getName().c_str());
			line_no = 0;
			VAL1_2::log_error(VAL1_2::E_FATAL,"Failed to open file");
		} else {
			line_no = 1;
			VAL1_2::yfl->switch_streams(&domainFile, &std::cout);
			yyparse();

			// domain name
			domain = VAL1_2::current_analysis->the_domain;
			domain_name = domain->name;
		}
		delete VAL1_2::yfl;
		domainFile.close();

		return domain;

	}

} // close namespace
