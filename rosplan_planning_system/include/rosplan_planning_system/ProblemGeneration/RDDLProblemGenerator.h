//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 26/09/18.
//

#ifndef ROSPLAN_PLANNING_SYSTEM_RDDLPROBLEMGENERATOR_H
#define ROSPLAN_PLANNING_SYSTEM_RDDLPROBLEMGENERATOR_H

#include "ProblemGenerator.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorService.h"
#include "rosplan_knowledge_msgs/GetRDDLParams.h"
#include "rosplan_knowledge_msgs/GetEnumerableTypeService.h"
#include "rosplan_knowledge_msgs/ReloadRDDLDomainProblem.h"
#include "rosplan_knowledge_msgs/GetRDDLFluentType.h"

namespace KCL_rosplan {
    class RDDLProblemGenerator : public ProblemGenerator {
    private:
        inline void makeProblem(std::ofstream &pFile) override;

        void makeNonFluents(std::ofstream &pFile, const std::set<std::string>& nonfluents);
        void makeInstance(std::ofstream &pFile, const std::set<std::string>& fluents);

        /* returns map of type -> list of instances of that type */
        std::map<std::string, std::vector<std::string>> getInstances();

        /* Returns the non-fluents after analysing the domain operations */
        void getAllFluents(const std::vector<std::string> &pred_funcs, std::set<std::string> &found_fluents,
                           std::set<std::string> &found_nonfluents);
        std::vector<std::string> getPredicatesFunctions();
        void printGenericFluentElement(std::ofstream &pFile,
                                       const std::vector<rosplan_knowledge_msgs::KnowledgeItem> &elem);
        void printGenericFluentList(std::ofstream &pFile, const std::set<std::string>& fluentlist);

        std::string _domain_name;
        std::string _non_fluents_name;
        ros::NodeHandle _nh;
        ros::ServiceClient _reload_domain;
        ros::ServiceClient _get_fluent_type;
        std::map<std::string, std::vector<std::string>> _enumeration_types;
    public:
        RDDLProblemGenerator(const std::string& kb);
        void generateProblemFile(std::string &problemPath) override;
    };
}

#endif //ROSPLAN_PLANNING_SYSTEM_RDDLPROBLEMGENERATOR_H
