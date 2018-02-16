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


} // close namespace
