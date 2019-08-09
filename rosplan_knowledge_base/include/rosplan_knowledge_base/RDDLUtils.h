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
#include "RDDLParser.h"
namespace KCL_rosplan {

    typedef std::vector<rosplan_knowledge_msgs::DomainFormula> vectorDF;
    typedef std::vector<rosplan_knowledge_msgs::DomainAssignment> vectorDA;
    typedef std::vector<rosplan_knowledge_msgs::DomainAssignment> vectorDA;
    typedef std::vector<rosplan_knowledge_msgs::ProbabilisticEffect> vectorPE;
    typedef std::vector<rosplan_knowledge_msgs::KnowledgeItem> vectorKI;
    struct PosNegDomainFormula {
        vectorDF pos; // Positive or add formulas
        vectorDF neg; // Negative or delete formulas
    };
    struct EffectDomainFormula {
        vectorDF add; // Add formulas
        vectorDF del; // Delete formulas
        vectorPE prob; // Probabilistic effects
    };

    class RDDLUtils {
    private:
        /* joints a and b, leaving the result in a. b is not valid anymore! */
        static inline void join(PosNegDomainFormula &a, PosNegDomainFormula &b);
        static inline void join(vectorDA &a, vectorDA &b);
        static inline void join(EffectDomainFormula &a, EffectDomainFormula &b);
        static inline void join(vectorKI &a, vectorKI &b);
        /* negates p by swapping positive formulas by negative ones */
        static inline void negate(PosNegDomainFormula& p);
        static inline void negate(EffectDomainFormula& p);
        /* Returns an assignment of the parameters of the op_var to the parmeters of the op_head, to match all the params for the operator */
        static std::map<std::string, std::string> getParamReplacement(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable* op_var);

        static PosNegDomainFormula toDomainFormula(const LogicalExpression *expr, const std::map<std::string, std::string>& assign);


        static PosNegDomainFormula getOperatorPrecondition(const rosplan_knowledge_msgs::DomainFormula &op_head, const LogicalExpression *SAC);
        static PosNegDomainFormula getOperatorPrecondition(const rosplan_knowledge_msgs::DomainFormula &op_head, const Connective *SAC);
        static PosNegDomainFormula getOperatorPrecondition(const rosplan_knowledge_msgs::DomainFormula &op_head, const Negation *SAC);


        static EffectDomainFormula getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const LogicalExpression *exp, std::map<std::string, std::string>& assign, bool& action_found, bool& exogenous);
        static EffectDomainFormula getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const IfThenElseExpression *exp, std::map<std::string, std::string>& assign, bool& action_found, bool& exogenous);
        static EffectDomainFormula getOperatorEffects(const ParametrizedVariable *pVariable, const BernoulliDistribution *exp, const std::map<std::string, std::string>& assign, bool& action_found, bool& exogenous);
        static EffectDomainFormula getOperatorEffects(const ParametrizedVariable *pVariable, const DiscreteDistribution *exp, const std::map<std::string, std::string>& assign, bool& action_found, bool& exogenous);
        static void fillForAllEffect(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const UniversalQuantification *exp, EffectDomainFormula& out, size_t paramid, std::map<std::string, std::string> &assign, bool& action_found, bool& exogenous);

        static vectorDA getOperatorAssignEffects(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                                 const ParametrizedVariable *pVariable, const IfThenElseExpression *exp);

        static vectorKI getGoals(const LogicalExpression *exp, bool is_negative,  std::map<std::string, std::string> &assign);
        static void fillForAllGoal(const UniversalQuantification *exp, vectorKI& out, size_t paramid, bool is_negative, std::map<std::string, std::string> &assign);

    public:
        /**
         * A precondition is only considered when it is in state-action constraint and has the form:
         *      actionfluent_name => (precondition)
         * Only conjunctions will be considered in the precondition part, which is the one that will be returned.
         * @param op_name Name of the operator
         * @param SACs Action-State Constraints
         * @return Preconditions
         */
        static PosNegDomainFormula getOperatorPreconditions(const rosplan_knowledge_msgs::DomainFormula &op_head, const std::vector<LogicalExpression *> &SACs);

        static EffectDomainFormula getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head, const std::map<ParametrizedVariable*, LogicalExpression*>& CPFs);
        static vectorDA getOperatorAssignEffects(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                                 const std::map<ParametrizedVariable *, LogicalExpression *> &CPFs);

        static vectorKI getGoals(const std::map<ParametrizedVariable*, LogicalExpression*>& CPFs);
    };
}

#endif //ROSPLAN_KNOWLEDGE_BASE_RDDLOPERATORUTILS_H
