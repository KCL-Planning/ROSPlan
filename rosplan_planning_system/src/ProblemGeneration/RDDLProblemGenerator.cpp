//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 26/09/18.
//

#include "rosplan_planning_system/ProblemGeneration/RDDLProblemGenerator.h"

namespace KCL_rosplan {

    RDDLProblemGenerator::RDDLProblemGenerator(const std::string& kb) : ProblemGenerator(kb) {
        // Get domain name
        ros::ServiceClient getNameClient = _nh.serviceClient<rosplan_knowledge_msgs::GetDomainNameService>(domain_name_service);
        getNameClient.waitForExistence(ros::Duration(15));
        rosplan_knowledge_msgs::GetDomainNameService nameSrv;
        if (!getNameClient.call(nameSrv)) {
            ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s. Is the Knowledge Base running?", domain_name_service.c_str());
            ros::shutdown();
        }
        _domain_name = nameSrv.response.domain_name;
        _non_fluents_name = "nf_" + _domain_name + "__generate_instance";
        _reload_domain = _nh.serviceClient<rosplan_knowledge_msgs::ReloadRDDLDomainProblem>(kb + "/reload_rddl_domain");
        _get_fluent_type = _nh.serviceClient<rosplan_knowledge_msgs::GetRDDLFluentType>(kb + "/domain/fluent_type");
    }


    /**
     * generates a RDDL problem file.
     * This file is later read by the planner.
     */
    void RDDLProblemGenerator::generateProblemFile(std::string &problemPath) {
        if (knowledge_base.size() == 0) ROS_ERROR("KCL: (%s) Knowledge base is not set!", ros::this_node::getName().c_str());
        std::ofstream pFile;
        pFile.open((problemPath).c_str());
        makeProblem(pFile);
        pFile.close();
        rosplan_knowledge_msgs::ReloadRDDLDomainProblem srv;
        srv.request.problem_path = problemPath;
        if (not _reload_domain.call(srv) or not srv.response.success) {
            ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s.", _reload_domain.getService().c_str());
        }
    }

    void RDDLProblemGenerator::makeProblem(std::ofstream &pFile) {
        std::vector<std::string> pred_funcs = getPredicatesFunctions();
        std::set<std::string> fluents, non_fluents;
        getAllFluents(pred_funcs, fluents, non_fluents);
        pFile << "// Auto-generated problem file by ROSPlan" << std::endl;
        makeNonFluents(pFile, non_fluents);
        makeInstance(pFile, fluents);
    }

    void RDDLProblemGenerator::makeNonFluents(std::ofstream &pFile, const std::set<std::string>& nonfluents) {
        pFile << "non-fluents "<< _non_fluents_name << " {" << std::endl;
        pFile << "\tdomain = " << _domain_name << ";" << std::endl;

       // Print objects
        pFile << "\tobjects {" << std::endl;

        std::string srv_name = "/" + knowledge_base + "/domain/enumerable_type";
        ros::ServiceClient getEnumerableType = _nh.serviceClient<rosplan_knowledge_msgs::GetEnumerableTypeService>(srv_name);

        std::map<std::string, std::vector<std::string>> instances = getInstances();
        for (auto it = instances.begin(); it != instances.end(); ++it) {
            if (it->second.size() > 0) {
                if ((*it->second.begin())[0] == '@') { // If it starts with @ means it's an enumeration
                    rosplan_knowledge_msgs::GetEnumerableTypeService params;
                    params.request.type_name = it->first;
                    if (!getEnumerableType.call(params)) {
                        ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s", srv_name.c_str());
                    }
                    _enumeration_types[it->first] = params.response.values;
                }
                else {
                    pFile << "\t\t" << it->first << ": " << "{";
                    bool comma = false;
                    for (auto inst_it = it->second.begin(); inst_it != it->second.end(); ++inst_it) {
                        if (comma) pFile << ", ";
                        else comma = true;
                        pFile << *inst_it;
                    }
                    pFile << "};" << std::endl;
                }
            }
        }
        pFile << "\t};" << std::endl;


        // Print non-fluents
        pFile << "\tnon-fluents {" << std::endl;
        printGenericFluentList(pFile, nonfluents);
        pFile << "\t};" << std::endl;
        pFile << "}" << std::endl;
    }


    void RDDLProblemGenerator::makeInstance(std::ofstream &pFile, const std::set<std::string> &fluents) {
        pFile << "instance " << _domain_name << "__generated_instance {" << std::endl;
        pFile << "\tdomain = " << _domain_name << ";" << std::endl;
        pFile << "\tnon-fluents = " << _non_fluents_name << ";" << std::endl;

        // Initial state
        pFile << "\tinit-state {" << std::endl;
        printGenericFluentList(pFile, fluents);
        pFile << "\t};" << std::endl;

        std::string srv_name = "/" + knowledge_base + "/state/rddl_parameters";
        ros::ServiceClient getRDDLParams = _nh.serviceClient<rosplan_knowledge_msgs::GetRDDLParams>(srv_name);
        rosplan_knowledge_msgs::GetRDDLParams params;
        if (!getRDDLParams.call(params)) {
            ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s", srv_name.c_str());
        }

        pFile << "\tmax-nondef-actions = " << params.response.max_nondef_actions << ";" << std::endl;
        pFile << "\thorizon = " << params.response.horizon << ";" << std::endl;
        pFile << "\tdiscount = " << std::fixed << std::setprecision(2) << params.response.discount_factor << ";" << std::endl;
        pFile << "}" << std::endl;

    }

    std::map<std::string, std::vector<std::string>> RDDLProblemGenerator::getInstances() {
        std::map<std::string, std::vector<std::string>> inst;
        ros::ServiceClient getInstancesClient = _nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(state_instance_service);
        ros::ServiceClient getTypesClient = _nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>(domain_type_service);

        rosplan_knowledge_msgs::GetDomainTypeService typeSrv;
        if (!getTypesClient.call(typeSrv)) {
            ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s", domain_type_service.c_str());
        }

        // get instances of each type
        for (size_t t = 0; t < typeSrv.response.types.size(); ++t) {

            rosplan_knowledge_msgs::GetInstanceService instanceSrv;
            instanceSrv.request.type_name = typeSrv.response.types[t];

            if (!getInstancesClient.call(instanceSrv)) {
                ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s: %s", state_instance_service.c_str(),
                          instanceSrv.request.type_name.c_str());
            } else {
                if (instanceSrv.response.instances.size() == 0) continue;
                inst[instanceSrv.request.type_name] = instanceSrv.response.instances;
            }
        }
        return inst;
    }

    void RDDLProblemGenerator::getAllFluents(const std::vector<std::string> &pred_funcs,
                                             std::set<std::string> &found_fluents,
                                             std::set<std::string> &found_nonfluents) {
        ros::ServiceClient getOperatorsDetailsClient = _nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(domain_operator_details_service);
        ros::ServiceClient getOperatorsClient = _nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>(domain_operator_list_service);

        rosplan_knowledge_msgs::GetDomainOperatorService op_list;
        if (!getOperatorsClient.call(op_list)) {
            ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s", domain_operator_details_service.c_str());
        }
        else {
            for (auto op = op_list.response.operators.begin(); op != op_list.response.operators.end(); ++op) {
                rosplan_knowledge_msgs::GetDomainOperatorDetailsService op_srv;
                op_srv.request.name = op->name;
                if (!getOperatorsDetailsClient.call(op_srv)) {
                    ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s",
                              domain_operator_details_service.c_str());
                } else {
                    // Check in all the effects
                    for (auto it = op_srv.response.op.at_start_add_effects.begin();
                         it != op_srv.response.op.at_start_add_effects.end(); ++it) {
                        found_fluents.insert(it->name);
                    }
                    for (auto it = op_srv.response.op.at_start_del_effects.begin();
                         it != op_srv.response.op.at_start_del_effects.end(); ++it) {
                        found_fluents.insert(it->name);
                    }
                    for (auto it = op_srv.response.op.at_end_add_effects.begin();
                         it != op_srv.response.op.at_end_add_effects.end(); ++it) {
                        found_fluents.insert(it->name);
                    }
                    for (auto it = op_srv.response.op.at_end_del_effects.begin();
                         it != op_srv.response.op.at_end_del_effects.end(); ++it) {
                        found_fluents.insert(it->name);
                    }
                    for (auto it = op_srv.response.op.at_start_assign_effects.begin();
                         it != op_srv.response.op.at_start_assign_effects.end(); ++it) {
                        found_fluents.insert(it->LHS.name);
                    }
                    for (auto it = op_srv.response.op.at_end_assign_effects.begin();
                         it != op_srv.response.op.at_end_assign_effects.end(); ++it) {
                        found_fluents.insert(it->LHS.name);
                    }
                    for (size_t i = 0; i < op_srv.response.op.probabilistic_effects.size(); ++i) {
                        for (auto it = op_srv.response.op.probabilistic_effects[i].add_effects.begin();
                             it != op_srv.response.op.probabilistic_effects[i].add_effects.end(); ++it) {
                            found_fluents.insert(it->name);
                        }
                        for (auto it = op_srv.response.op.probabilistic_effects[i].del_effects.begin();
                             it != op_srv.response.op.probabilistic_effects[i].del_effects.end(); ++it) {
                            found_fluents.insert(it->name);
                        }
                        for (auto it = op_srv.response.op.probabilistic_effects[i].assign_effects.begin();
                             it != op_srv.response.op.probabilistic_effects[i].assign_effects.end(); ++it) {
                            found_fluents.insert(it->LHS.name);
                        }
                    }
                }
            }
            // Check if the predicate appears in any of the effect lists
            for (auto pf_it = pred_funcs.begin(); pf_it != pred_funcs.end(); ++pf_it) {
                // If not found in fluents, add it to found_nonfluents
                if (found_fluents.find(*pf_it) == found_fluents.end()) found_nonfluents.insert(*pf_it);
            }
        }
    }

    std::vector<std::string> RDDLProblemGenerator::getPredicatesFunctions() {
        std::vector<std::string> pred_funcs;

        // Get all predicates and functions
        ros::ServiceClient getDomainPropsClient = _nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_predicate_service);
        ros::ServiceClient getDomainFuncsClient = _nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_function_service);

        rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;
        rosplan_knowledge_msgs::GetDomainAttributeService domainFunSrv;
        if (!getDomainPropsClient.call(domainAttrSrv) or !getDomainFuncsClient.call(domainFunSrv)) {
            ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s", domain_predicate_service.c_str());
        } else {
            for (auto it = domainAttrSrv.response.items.begin(); it != domainAttrSrv.response.items.end(); ++it) {
                pred_funcs.push_back(it->name);
            }
            for (auto it = domainFunSrv.response.items.begin(); it != domainFunSrv.response.items.end(); ++it) {
                pred_funcs.push_back(it->name);
            }
        }
        return pred_funcs;
    }

    void RDDLProblemGenerator::printGenericFluentElement(std::ofstream &pFile,
                                                         const std::vector<rosplan_knowledge_msgs::KnowledgeItem> &elem) {
        for (auto pfit = elem.begin(); pfit != elem.end(); ++pfit) {
            pFile << "\t\t" << pfit->attribute_name;

            // Parameters if any
            if (pfit->values.size() > 0) {
                pFile << "(";
                bool comma = false;
                for (auto it = pfit->values.begin(); it != pfit->values.end(); ++it) {
                    if (comma) pFile << ", ";
                    else comma = true;
                    pFile << it->value;
                }
                pFile << ")";
            }

            // Value
            pFile << " = ";
            if (pfit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {
                pFile << ((pfit->is_negative == 0)? "true" : "false");
            }
            else { // FUNCTION
                rosplan_knowledge_msgs::GetRDDLFluentType gft;
                gft.request.fluent_name = pfit->attribute_name;
                if (!_get_fluent_type.call(gft)) {
                    ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s",
                              _get_fluent_type.getService().c_str(), gft.request.fluent_name.c_str());
                    return;
                }
                auto enum_type = _enumeration_types.find(gft.response.type);
                if (enum_type != _enumeration_types.end()) {
                    pFile << enum_type->second[pfit->function_value];
                }
                else pFile << pfit->function_value;
            }
            pFile << ";" << std::endl;
        }
    }

    void RDDLProblemGenerator::printGenericFluentList(std::ofstream &pFile, const std::set<std::string> &fluentlist) {
        ros::ServiceClient getPropsClient = _nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_proposition_service);
        ros::ServiceClient getFuncsClient = _nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_function_service);
        for (auto it = fluentlist.begin(); it != fluentlist.end(); ++it) {
            rosplan_knowledge_msgs::GetAttributeService attrSrv;
            attrSrv.request.predicate_name = *it;
            if (!getPropsClient.call(attrSrv)) {
                ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s: %s",
                          state_proposition_service.c_str(), attrSrv.request.predicate_name.c_str());
                continue;
            }

            // If it was not a proposition, try functions
            if (attrSrv.response.attributes.size() == 0 and !getFuncsClient.call(attrSrv)) {
                ROS_ERROR("KCL: (RDDLProblemGenerator) Failed to call service %s: %s",
                          state_proposition_service.c_str(), attrSrv.request.predicate_name.c_str());
                continue;
            }
            printGenericFluentElement(pFile, attrSrv.response.attributes);
        }

    }

}