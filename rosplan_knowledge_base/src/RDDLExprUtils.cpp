//
// Created by gerard on 21/09/18.
//

#include <rosplan_knowledge_base/RDDLExprUtils.h>

#include "ros/ros.h"
#include "rosplan_knowledge_base/RDDLExprUtils.h"

rosplan_knowledge_msgs::ExprComposite RDDLExprUtils::getExpression(const Evaluatable *expr) {
    return getExpression(expr->formula, std::map<std::string, std::string>());
}

rosplan_knowledge_msgs::ExprComposite
RDDLExprUtils::getExpression(const LogicalExpression *expr, const std::map<std::string, std::string> &assign) {
    auto exp_con = dynamic_cast<const Connective*>(expr);
    if (exp_con != nullptr) return getExpression(exp_con, assign);

    auto exp_quan = dynamic_cast<const Quantifier*>(expr);
    if (exp_quan != nullptr) return getExpression(exp_quan, assign);

    auto exp_param = dynamic_cast<const ParametrizedVariable*>(expr);
    if (exp_param != nullptr) return getExpression(exp_param, assign);

    auto exp_numeric = dynamic_cast<const NumericConstant*>(expr);
    if (exp_numeric != nullptr) return getExpression(exp_numeric, assign);

    auto exp_neg = dynamic_cast<const Negation*>(expr);
    if (exp_neg != nullptr) return getExpression(exp_neg, assign);

    auto exp_parameter = dynamic_cast<const Parameter*>(expr);
    if (exp_parameter != nullptr) return getExpression(exp_parameter, assign);

    auto ite = dynamic_cast<const IfThenElseExpression*>(expr);
    if (ite != nullptr) return getExpression(ite, assign);

    NOT_IMPLEMENTED_EXPR;
    return rosplan_knowledge_msgs::ExprComposite();
}

rosplan_knowledge_msgs::ExprComposite RDDLExprUtils::getExpression(const Connective *expr, const std::map<std::string, std::string>& assign) {
    assert(expr->exprs.size() == 2); // Assuming two operands in each
    rosplan_knowledge_msgs::ExprComposite ret;

    rosplan_knowledge_msgs::ExprBase base;
    base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;

    // Check type
    if (dynamic_cast<const Multiplication*>(expr) != nullptr || dynamic_cast<const Conjunction*>(expr) != nullptr)  base.op = rosplan_knowledge_msgs::ExprBase::MUL;
    else if (dynamic_cast<const Addition*>(expr) != nullptr || dynamic_cast<const Disjunction*>(expr) != nullptr)  base.op = rosplan_knowledge_msgs::ExprBase::ADD;
    else if (dynamic_cast<const Subtraction*>(expr) != nullptr)  base.op = rosplan_knowledge_msgs::ExprBase::SUB;
    else if (dynamic_cast<const Division*>(expr) != nullptr)  base.op = rosplan_knowledge_msgs::ExprBase::DIV;
    else {
        NOT_IMPLEMENTED_EXPR;
        return ret;
    }
    ret.tokens.push_back(base);

    rosplan_knowledge_msgs::ExprComposite operand1 = getExpression(expr->exprs[0], assign);
    rosplan_knowledge_msgs::ExprComposite operand2 = getExpression(expr->exprs[1], assign);
    //ret.tokens.insert(ret.tokens.end(), std::make_move_iterator(operand1.tokens.begin()), std::make_move_iterator(operand1.tokens.end()));
    join(ret.tokens, operand1.tokens);
    //ret.tokens.insert(ret.tokens.end(), std::make_move_iterator(operand2.tokens.begin()), std::make_move_iterator(operand2.tokens.end()));
    join(ret.tokens, operand2.tokens);
    return ret;
}

rosplan_knowledge_msgs::ExprComposite RDDLExprUtils::getExpression(const Negation *expr, const std::map<std::string, std::string>& assign) {
    rosplan_knowledge_msgs::ExprComposite ret;

    rosplan_knowledge_msgs::ExprBase base;
    base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
    base.op = rosplan_knowledge_msgs::ExprBase::UMINUS;
    ret.tokens.push_back(base);

    rosplan_knowledge_msgs::ExprComposite nexp = getExpression(expr->expr, assign);
    //ret.tokens.insert(ret.tokens.end(), std::make_move_iterator(nexp.tokens.begin()), std::make_move_iterator(nexp.tokens.end()));
    join(ret.tokens, nexp.tokens);
    return ret;
}

rosplan_knowledge_msgs::ExprComposite RDDLExprUtils::getExpression(const NumericConstant *expr, const std::map<std::string, std::string>& assign) {
    rosplan_knowledge_msgs::ExprComposite ret;

    rosplan_knowledge_msgs::ExprBase base;
    base.expr_type = rosplan_knowledge_msgs::ExprBase::CONSTANT;
    base.constant = expr->value;
    ret.tokens.push_back(base);
    return ret;
}

rosplan_knowledge_msgs::ExprComposite RDDLExprUtils::getExpression(const Quantifier *expr, const std::map<std::string, std::string>& assign) {
    // Check type
    rosplan_knowledge_msgs::ExprBase::_op_type op_type = 0;
    if (dynamic_cast<const Sumation*>(expr) != nullptr)  op_type = rosplan_knowledge_msgs::ExprBase::ADD;
    else if (dynamic_cast<const Product*>(expr) != nullptr)  op_type = rosplan_knowledge_msgs::ExprBase::MUL;
    else if (dynamic_cast<const UniversalQuantification*>(expr) != nullptr)  op_type = rosplan_knowledge_msgs::ExprBase::ADD; // Treat the forall as a summation FIXME can this be problematic? Done for the goal ideally should be truncated to boolean...
    else if (dynamic_cast<const ExistentialQuantification*>(expr) != nullptr)  op_type = rosplan_knowledge_msgs::ExprBase::ADD; // Treat the exists as a summation FIXME can this be problematic? ideally should be truncated to boolean...
    else NOT_IMPLEMENTED_EXPR;

    // Get all the operands of the quantifier operation
    std::vector<rosplan_knowledge_msgs::ExprComposite> operands;
    std::map<std::string, std::string> q_assign = assign;
    getQuantExprRec(expr, 0, q_assign, operands);

    // Join all the operands in one single ExprComposite
    return joinQuantOperator(operands, op_type);
}


rosplan_knowledge_msgs::ExprComposite RDDLExprUtils::joinQuantOperator(std::vector<rosplan_knowledge_msgs::ExprComposite> &operands,
                                                                       rosplan_knowledge_msgs::ExprBase::_op_type op_type, int op_idx) {
    if (operands.size()-1 == op_idx) return operands[op_idx]; // Found last element

    rosplan_knowledge_msgs::ExprComposite expcomp;
    // Operator
    rosplan_knowledge_msgs::ExprBase base;
    base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
    base.op = op_type;
    expcomp.tokens.push_back(base);

    join(expcomp.tokens, operands[op_idx].tokens);
    auto opid_tokens = joinQuantOperator(operands, op_type, op_idx+1).tokens;
    join(expcomp.tokens, opid_tokens);
    return expcomp;
}



void RDDLExprUtils::getQuantExprRec(const Quantifier* expr, int param_id, std::map<std::string, std::string>& assign,
                                    std::vector<rosplan_knowledge_msgs::ExprComposite>& result) {

    for (auto it = expr->paramList->types[param_id]->objects.begin(); it < expr->paramList->types[param_id]->objects.end(); ++it) {
        assign[expr->paramList->params[param_id]->name] = (*it)->name;
        if (param_id == expr->paramList->params.size()-1) { // If it's the last one, instantiate and generate expression
            result.push_back(getExpression(expr->expr, assign));
        }
        else getQuantExprRec(expr, param_id+1, assign, result);
    }
}

void RDDLExprUtils::join(std::vector<rosplan_knowledge_msgs::ExprBase> &a,
                         std::vector<rosplan_knowledge_msgs::ExprBase> &b) {
    a.insert(a.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
}

rosplan_knowledge_msgs::ExprComposite
RDDLExprUtils::getExpression(const ParametrizedVariable *expr, const std::map<std::string, std::string> &assign) {
    rosplan_knowledge_msgs::ExprComposite ret;

    rosplan_knowledge_msgs::ExprBase base;
    base.expr_type = rosplan_knowledge_msgs::ExprBase::FUNCTION;

    // Fill the function
    base.function.name = expr->variableName;
    for (auto pit = expr->params.begin(); pit != expr->params.end(); ++pit) {
        diagnostic_msgs::KeyValue param;
        param.key = (*pit)->name; // Parameter name (i.e. ?r)
        size_t pos = param.key.find('?');
        if (pos != std::string::npos) param.key.erase(pos, 1); // Remove the ? if present

        auto it = assign.find((*pit)->name); // Find if there was an assignment to the variable
        if (it != assign.end()) param.value = it->second;
        else param.value = (*pit)->type->name; // Type name
        base.function.typed_parameters.push_back(param);
    }

    ret.tokens.push_back(base);
    return ret;
}

rosplan_knowledge_msgs::ExprComposite
RDDLExprUtils::getExpression(const IfThenElseExpression *expr, const std::map<std::string, std::string> &assign) {
    // Expression will be condition*bodyif + (~condition*bodyelse)
    rosplan_knowledge_msgs::ExprComposite ret;

    rosplan_knowledge_msgs::ExprBase mult;
    mult.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
    mult.op = rosplan_knowledge_msgs::ExprBase::MUL;


    auto condition = getExpression(expr->condition, assign);
    auto condition_cpy = condition; // Copy to negate it, as join moves the values and destorys the object

    // Result = + (* condition bodyif) (* (1-condition) bodyelse)
    // +
    rosplan_knowledge_msgs::ExprBase add;
    add.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
    add.op = rosplan_knowledge_msgs::ExprBase::ADD;
    ret.tokens.push_back(add);

    // * condition valiftrue
    ret.tokens.push_back(mult);
    join(ret.tokens, condition.tokens);
    auto valiftrue = getExpression(expr->valueIfTrue, assign);
    join(ret.tokens, valiftrue.tokens);

    // * ~condition valiffalse (else)
    ret.tokens.push_back(mult); // *
    rosplan_knowledge_msgs::ExprBase uminus; // 1- (negation)
    uminus.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
    uminus.op = rosplan_knowledge_msgs::ExprBase::UMINUS;
    ret.tokens.push_back(uminus);
    join(ret.tokens, condition_cpy.tokens);
    
    auto valiffalse = getExpression(expr->valueIfFalse, assign);
    join(ret.tokens, valiffalse.tokens);

    return ret;
}


rosplan_knowledge_msgs::ExprComposite
RDDLExprUtils::getExpression(const Parameter *expr, const std::map<std::string, std::string> &assign) {
    rosplan_knowledge_msgs::ExprComposite ret;

    rosplan_knowledge_msgs::ExprBase base;
    base.expr_type = base.FUNCTION;
    base.function.name = expr->name;
    ret.tokens.push_back(base);
    return ret;
}

