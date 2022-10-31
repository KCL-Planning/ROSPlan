//
// Created by gerard on 21/09/18.
//

#ifndef ROSPLAN_KNOWLEDGE_BASE_RDDLEXPRUTILS_H
#define ROSPLAN_KNOWLEDGE_BASE_RDDLEXPRUTILS_H

#define NOT_IMPLEMENTED_EXPR ROS_WARN_STREAM(__FILE__ << ":" << __LINE__ << ": Unknown or unsupported operand type for the expression.")

#include <rosplan_knowledge_msgs/ExprComposite.h>
#include <rosplan_knowledge_msgs/ExprBase.h>
#include "RDDLParser.h"

class RDDLExprUtils {
private:
    // Gets a list of the expression of the quantifier (sumation or product) instantiated with all the variables of the quantifier
    static void getQuantExprRec(const Quantifier* expr, int param_id, std::map<std::string, std::string>& assign,
                                std::vector<rosplan_knowledge_msgs::ExprComposite>& result);

    // Moves elements of b to the end of a
    static inline void join(std::vector<rosplan_knowledge_msgs::ExprBase>& a, std::vector<rosplan_knowledge_msgs::ExprBase>& b);

    // Joints a list of instantiated expressions to a single polish notation opertion
    static rosplan_knowledge_msgs::ExprComposite joinQuantOperator(std::vector<rosplan_knowledge_msgs::ExprComposite> &operands,
                                                                   rosplan_knowledge_msgs::ExprBase::_op_type op_type, int op_idx=0);
public:
    /* Convert to ROSPlan expression */
    static rosplan_knowledge_msgs::ExprComposite getExpression(const Evaluatable* expr);

    static rosplan_knowledge_msgs::ExprComposite getExpression(const Connective* expr, const std::map<std::string, std::string>& assign);
    static rosplan_knowledge_msgs::ExprComposite getExpression(const Quantifier* expr, const std::map<std::string, std::string>& assign);
    static rosplan_knowledge_msgs::ExprComposite getExpression(const Negation *expr, const std::map<std::string, std::string>& assign);
    static rosplan_knowledge_msgs::ExprComposite getExpression(const NumericConstant *expr, const std::map<std::string, std::string>& assign);
    static rosplan_knowledge_msgs::ExprComposite getExpression(const ParametrizedVariable *expr, const std::map<std::string, std::string>& assign);
    static rosplan_knowledge_msgs::ExprComposite getExpression(const Parameter *expr, const std::map<std::string, std::string>& assign);
    static rosplan_knowledge_msgs::ExprComposite getExpression(const IfThenElseExpression *expr, const std::map<std::string, std::string>& assign);
    static rosplan_knowledge_msgs::ExprComposite getExpression(const LogicalExpression *expr, const std::map<std::string, std::string>& assign);
};


#endif //ROSPLAN_KNOWLEDGE_BASE_RDDLEXPRUTILS_H
