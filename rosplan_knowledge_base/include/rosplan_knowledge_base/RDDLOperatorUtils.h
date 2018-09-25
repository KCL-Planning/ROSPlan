//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 25/09/18.
//

#ifndef ROSPLAN_KNOWLEDGE_BASE_RDDLOPERATORUTILS_H
#define ROSPLAN_KNOWLEDGE_BASE_RDDLOPERATORUTILS_H

#define NOT_IMPLEMENTED(str) ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ": " << str)
#define NOT_IMPLEMENTED_OPERATOR NOT_IMPLEMENTED("Unknown or unsupported operand type for the action preconditions.")

#include <rosplan_knowledge_msgs/DomainFormula.h>
#include "RDDLParser.h"
namespace KCL_rosplan {

    typedef std::vector<rosplan_knowledge_msgs::DomainFormula> vectorDF;
    struct PosNegDomainFormula {
        vectorDF pos;
        vectorDF neg;
    };

    class RDDLOperatorUtils {
    private:
        /* joints a and b, leaving the result in a. b is not valid anymore! */
        static inline void join(PosNegDomainFormula &a, PosNegDomainFormula &b);
        static inline void negate(PosNegDomainFormula& p);

        static PosNegDomainFormula getOperatorPrecondition(const std::string &op_name, const LogicalExpression *SAC);
        static PosNegDomainFormula getOperatorPrecondition(const std::string &op_name, const Connective *SAC);
        static PosNegDomainFormula getOperatorPrecondition(const std::string &op_name, const Negation *SAC);

        static PosNegDomainFormula toDomainFormula(const LogicalExpression *expr);

        static PosNegDomainFormula getOperatorEffects(const std::string& op_name, const ParametrizedVariable *pVariable, const LogicalExpression *exp);
        static PosNegDomainFormula getOperatorEffects(const std::string& op_name, const ParametrizedVariable *pVariable, const IfThenElseExpression *exp);


    public:
        /**
         * A precondition is only considered when it is in state-action constraint and has the form:
         *      actionfluent_name => (precondition)
         * Only conjunctions will be considered in the precondition part, which is the one that will be returned.
         * @param op_name Name of the operator
         * @param SACs Action-State Constraints
         * @return Preconditions
         */
        static PosNegDomainFormula  getOperatorPreconditions(const std::string &op_name, const std::vector<LogicalExpression *> &SACs);

        static PosNegDomainFormula getOperatorEffects(const std::string& op_name, const std::map<ParametrizedVariable*, LogicalExpression*>& CPFs);
    };
}

#endif //ROSPLAN_KNOWLEDGE_BASE_RDDLOPERATORUTILS_H
