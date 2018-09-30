//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 30/09/18.
//

#ifndef ROSPLAN_KNOWLEDGE_BASE_PPDDLPARSER_H
#define ROSPLAN_KNOWLEDGE_BASE_PPDDLPARSER_H

#include "ros/ros.h"
#include "PPDDLParserInterface.h"
/**
 * Domain storage and parsing.
 */
namespace KCL_rosplan {

    class PPDDLParser {
    public:
        PPDDLParser() : ppddl_domain(nullptr), domain_parsed(false), domain_name(""), problem_name("") {};
        ~PPDDLParser();

        /* RDDL task pointers */
        PPDDLInterface::Domain* ppddl_domain; // Also stores the problems

        /* domain information */
        bool domain_parsed;
        std::string domain_name;
        std::string problem_name;


        /* domain parsing */
        PPDDLInterface::Domain* parseDomainProblem(const std::string &domainPath, const std::string &problemPath);
    };
}

#endif //ROSPLAN_KNOWLEDGE_BASE_PPDDLPARSER_H
