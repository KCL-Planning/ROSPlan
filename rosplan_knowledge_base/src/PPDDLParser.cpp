//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 30/09/18.
//

#include "rosplan_knowledge_base/PPDDLParser.h"

namespace KCL_rosplan {
    PPDDLDomainPtr PPDDLParser::parseDomainProblem(const std::string &domainPath, const std::string &problemPath) {
        // only parse domain once
        if (domain_parsed) return domain;
        ROS_INFO("KCL: (%s) Parsing domain: %s.", ros::this_node::getName().c_str(), domainPath.c_str());
        ROS_INFO("KCL: (%s) Parsing initial state", ros::this_node::getName().c_str());

        // parse domain
        try {
            std::vector<std::string> problem_vect;
            if (not problemPath.empty()) problem_vect.push_back(problemPath);
            domain = PPDDLInterface::Domain(domainPath, problem_vect)._getWrappedDomain();
        }
        catch (std::runtime_error e) {
            std::string filetype = "domain";
            std::string err(e.what());
            if (err.find("Problem") != std::string::npos or err.find("problem") != std::string::npos) filetype = "problem";
            ROS_ERROR("KCL: (%s) Failed to open %s file.", ros::this_node::getName().c_str(), filetype.c_str());
            return nullptr;
        }
        if (not domain) {
            ROS_ERROR("KCL: (%s) Failed to open domain file.", ros::this_node::getName().c_str());
            return nullptr;
        }

        domain_name =  domain->name();
        problem = ppddl_parser::Problem::begin()->second;
        problem_name = problem->name();
        domain_parsed = true;

        return domain;
    }
}