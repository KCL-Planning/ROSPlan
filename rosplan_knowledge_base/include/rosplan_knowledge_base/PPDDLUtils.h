//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 25/09/18.
//

#ifndef ROSPLAN_KNOWLEDGE_BASE_RDDLOPERATORUTILS_H
#define ROSPLAN_KNOWLEDGE_BASE_RDDLOPERATORUTILS_H

#define NOT_IMPLEMENTED(str) ROS_WARN_STREAM(__FILE__ << ":" << __LINE__ << ": " << str)
#define NOT_IMPLEMENTED_OPERATOR NOT_IMPLEMENTED("Unknown or unsupported operand type for the action preconditions.")

#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/DomainAssignment.h>
#include <rosplan_knowledge_msgs/ProbabilisticEffect.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include "PPDDLParser.h"

namespace KCL_rosplan {
    typedef std::vector<rosplan_knowledge_msgs::DomainFormula> vectorDF;
    typedef std::vector<rosplan_knowledge_msgs::DomainAssignment> vectorDA;
    typedef std::vector<rosplan_knowledge_msgs::ProbabilisticEffect> vectorPE;


    class PPDDLUtils {
    private:
        static rosplan_knowledge_msgs::DomainAssignment getUpdate(const ppddl_parser::UpdateEffect *pEffect, PPDDLDomainPtr domain, map<ppddl_parser::Term, string> &map);

        // Moves elements of b to the end of a
        static inline void join(std::vector<rosplan_knowledge_msgs::ExprBase>& a, std::vector<rosplan_knowledge_msgs::ExprBase>& b);

        static void fillForallGoal(const ppddl_parser::Forall *forall, PPDDLDomainPtr domain, PPDDLProblemPtr problem,
                                size_t paramid, std::vector<rosplan_knowledge_msgs::KnowledgeItem> &out_goal,
                                   std::map<ppddl_parser::Term, std::string>& var_decl, bool is_negative);

    public:

        /**
         *
         * @param precondition Expression to convert
         * @param domain Domain variable to get types and predicates from
         * @param op_head Domaiformula representing the operator header. If an exists predicate is found, the parameters are added to it
         * @param out_pos_cond Positive condition
         * @param out_neg_cond Negative conditions
         * @param var_decl [in] Declaration of variables
         * @param var_names [in] naming of variables
         */
        static void fillPreconditions(const ppddl_parser::StateFormula &precondition, PPDDLDomainPtr domain,
                                      rosplan_knowledge_msgs::DomainFormula &op_head, vectorDF &out_pos_cond,
                                      vectorDF &out_neg_cond, std::map<ppddl_parser::Term, std::string> &var_decl,
                                      std::map<std::string, int> &var_names);

        static void fillGoal(const ppddl_parser::StateFormula &goal, PPDDLDomainPtr domain, PPDDLProblemPtr problem,
                              std::vector<rosplan_knowledge_msgs::KnowledgeItem>& out_goal,
                             std::map<ppddl_parser::Term, std::string> &var_decl, bool is_negative);

        static void fillEffects(const ppddl_parser::Effect &effect, PPDDLDomainPtr domain, vectorDF &out_add_eff, vectorDF &out_del_eff,
                                vectorPE &out_pe_eff,
                                vectorDA &assign_eff, std::map<ppddl_parser::Term, std::string> &var_decl);

        static rosplan_knowledge_msgs::ExprComposite getExpression(const ppddl_parser::Expression &exp, PPDDLDomainPtr domain, std::map<ppddl_parser::Term, string> &var_decl);
        static rosplan_knowledge_msgs::DomainFormula getAtom(const ppddl_parser::Atom* a, PPDDLDomainPtr domain, std::map<ppddl_parser::Term, string> &var_decl, bool instantiate_variable=false);

    };
}

#endif //ROSPLAN_KNOWLEDGE_BASE_RDDLOPERATORUTILS_H
