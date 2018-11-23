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

    typedef std::shared_ptr<PPDDLInterface::p_Domain> PPDDLDomainPtr;
    typedef ppddl_parser::Problem const* PPDDLProblemPtr;
    class PPDDLParser {
    public:
        PPDDLParser() : problem(nullptr), domain(nullptr), domain_parsed(false), domain_name(""), problem_name("") {};
        ~PPDDLParser() = default;

        /* RDDL task pointers */
        PPDDLDomainPtr domain;
        PPDDLProblemPtr problem;

        /* domain information */
        bool domain_parsed;
        std::string domain_name;
        std::string problem_name;


        /* domain parsing */
        PPDDLDomainPtr parseDomainProblem(const std::string &domainPath, const std::string &problemPath);
    };
}

#endif //ROSPLAN_KNOWLEDGE_BASE_PPDDLPARSER_H
