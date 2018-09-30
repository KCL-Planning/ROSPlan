//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 30/09/18.
//

#include <rosplan_knowledge_base/PPDDLParser.h>

#include "rosplan_knowledge_base/PPDDLParser.h"

namespace KCL_rosplan {
    PPDDLInterface::Domain* PPDDLParser::parseDomainProblem(const std::string &domainPath, const std::string &problemPath) {
        // only parse domain once
        if (domain_parsed) return ppddl_domain;
        ROS_INFO("KCL: (%s) Parsing domain: %s.", ros::this_node::getName().c_str(), domainPath.c_str());
        ROS_INFO("KCL: (%s) Parsing initial state", ros::this_node::getName().c_str());

        // parse domain
        try {
            ppddl_domain = new PPDDLInterface::Domain(domainPath, std::vector<std::string>({problemPath}));
        }
        catch (std::runtime_error e) {
            std::string filetype = "domain";
            std::string err(e.what());
            if (err.find("Problem") != std::string::npos or err.find("problem") != std::string::npos) filetype = "problem";
            ROS_ERROR("KCL: (%s) Failed to open %s file.", ros::this_node::getName().c_str(), filetype.c_str());
            return nullptr;
        }
        if (not ppddl_domain) {
            ROS_ERROR("KCL: (%s) Failed to open domain file.", ros::this_node::getName().c_str());
            return nullptr;
        }
        
        domain_name =  ppddl_domain->getName();
        problem_name = ppddl_parser::Problem::begin()->second->name();
        domain_parsed = true;

        return ppddl_domain;
    }

    PPDDLParser::~PPDDLParser() {
        delete ppddl_domain;
    }
}