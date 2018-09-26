//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 25/09/18.
//

#include <rosplan_knowledge_base/RDDLOperatorUtils.h>

#include "rosplan_knowledge_base/RDDLOperatorUtils.h"
#include "ros/ros.h"

namespace KCL_rosplan {

    PosNegDomainFormula RDDLOperatorUtils::getOperatorPreconditions(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                                                    const std::vector<LogicalExpression *> &SACs) {
        PosNegDomainFormula ret;

        for (auto it = SACs.begin(); it != SACs.end(); ++it) {
            auto precs = getOperatorPrecondition(op_head, *it);
            join(ret, precs);
        }

        return ret;
    }

    PosNegDomainFormula RDDLOperatorUtils::getOperatorPrecondition(const rosplan_knowledge_msgs::DomainFormula &op_head, const LogicalExpression *SAC) {
        auto exp_con = dynamic_cast<const Connective *>(SAC);
        if (exp_con != nullptr) return getOperatorPrecondition(op_head, exp_con);

        auto exp_neg = dynamic_cast<const Negation *>(SAC);
        if (exp_neg != nullptr) return getOperatorPrecondition(op_head, exp_neg);

        auto exp_quan = dynamic_cast<const Quantifier *>(SAC);
        if (exp_quan != nullptr) return getOperatorPrecondition(op_head, exp_quan->expr);

        /*auto exp_param = dynamic_cast<const ParametrizedVariable *>(SAC);
        if (exp_param != nullptr) return getOperatorPrecondition(op_head, exp_param);

        auto exp_numeric = dynamic_cast<const NumericConstant *>(SAC);
        if (exp_numeric != nullptr) return getOperatorPrecondition(op_head, exp_numeric);*/

        return PosNegDomainFormula();
    }


    PosNegDomainFormula RDDLOperatorUtils::toDomainFormula(const LogicalExpression *expr,  const std::map<std::string, std::string>& assign) {
        auto exp_con = dynamic_cast<const Conjunction *>(expr);
        if (exp_con != nullptr) {
            assert(exp_con->exprs.size() == 2);
            PosNegDomainFormula op1, op2;
            op1 = toDomainFormula(exp_con->exprs[0], assign);
            op2 = toDomainFormula(exp_con->exprs[1], assign);
            join(op1, op2);
            return op1;
        }

        auto exp_neg = dynamic_cast<const Negation *>(expr); // FIXME may have a flaw if we have a negated conjunction!
        if (exp_neg != nullptr) {
            PosNegDomainFormula neg = toDomainFormula(exp_neg->expr, assign);
            negate(neg);
            return neg;
        }

        auto exp_var = dynamic_cast<const ParametrizedVariable *>(expr);
        if (exp_var != nullptr){
            rosplan_knowledge_msgs::DomainFormula f;
            f.name = exp_var->variableName;
            for (auto pit = exp_var->params.begin(); pit != exp_var->params.end(); ++pit) {
                diagnostic_msgs::KeyValue param;

                auto param_name = assign.find((*pit)->name);
                if (param_name != assign.end()) param.key = param_name->second; // Parameter name (i.e. ?r)
                else param.key = (*pit)->name;

                size_t pos = param.key.find('?');
                if (pos != std::string::npos) param.key.erase(pos, 1); // Remove the ? if present

                param.value = (*pit)->type->name; // Type name
                f.typed_parameters.push_back(param);
            }

            PosNegDomainFormula ret;
            ret.pos.push_back(f);
            return ret;
        }

        auto exp_dis = dynamic_cast<const Disjunction *>(expr);
        if (exp_dis != nullptr) {
            NOT_IMPLEMENTED("Disjunctions are not implemented as a precondition");
            return PosNegDomainFormula();
        }

        NOT_IMPLEMENTED_OPERATOR;
        /*auto exp_quan = dynamic_cast<const Quantifier *>(expr);
        if (exp_quan != nullptr) return getOperatorPrecondition( exp_quan->expr);*/

        return PosNegDomainFormula();
    }


    void RDDLOperatorUtils::join(PosNegDomainFormula &a, PosNegDomainFormula &b) {
        a.pos.insert(a.pos.end(), std::make_move_iterator(b.pos.begin()), std::make_move_iterator(b.pos.end()));
        a.neg.insert(a.neg.end(), std::make_move_iterator(b.neg.begin()), std::make_move_iterator(b.neg.end()));
    }


    void RDDLOperatorUtils::negate(PosNegDomainFormula &p) {
        vectorDF aux = p.pos;
        p.pos = p.neg;
        p.neg = aux;
    }

    PosNegDomainFormula RDDLOperatorUtils::getOperatorPrecondition(const rosplan_knowledge_msgs::DomainFormula &op_head, const Connective *SAC) {
        assert(SAC->exprs.size() == 2);
        auto disj = dynamic_cast<const Disjunction *>(SAC);
        if (disj != nullptr) {
            // Check if case action => precondition =(expressed as)= ¬action v precondition
            auto neg = dynamic_cast<const Negation *>(SAC->exprs[0]);
            if (neg != nullptr) {
                auto action = dynamic_cast<const ParametrizedVariable *>(neg->expr);
                if (action != nullptr and action->variableType == ParametrizedVariable::ACTION_FLUENT and
                    action->variableName == op_head.name) {
                    return toDomainFormula(disj->exprs[1], getParamReplacement(op_head, action));
                }
            }
        }

        PosNegDomainFormula op1 = getOperatorPrecondition(op_head, SAC->exprs[0]);
        PosNegDomainFormula op2 = getOperatorPrecondition(op_head, SAC->exprs[1]);
        join(op1, op2);
        return op1;
    }


    PosNegDomainFormula RDDLOperatorUtils::getOperatorPrecondition(const rosplan_knowledge_msgs::DomainFormula &op_head, const Negation *SAC) {
        // Swap the resulting precondition
        PosNegDomainFormula neg = getOperatorPrecondition(op_head, SAC->expr);
        negate(neg);
        return neg;
    }

    PosNegDomainFormula RDDLOperatorUtils::getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                                              const std::map<ParametrizedVariable *, LogicalExpression *> &CPFs) {
        PosNegDomainFormula eff_ret;
        for (auto it = CPFs.begin(); it != CPFs.end(); ++it) {
            PosNegDomainFormula eff_i = getOperatorEffects(op_head, it->first, it->second);
            join(eff_ret, eff_i);
        }
        return eff_ret;
    }

    PosNegDomainFormula
    RDDLOperatorUtils::getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const LogicalExpression *exp) {
        auto ifte = dynamic_cast<const IfThenElseExpression*>(exp);
        if (ifte != nullptr) return getOperatorEffects(op_head, pVariable, ifte);

        auto con = dynamic_cast<const Connective*>(exp);
        if (con != nullptr) {
            assert(con->exprs.size() == 2);
            PosNegDomainFormula a, b;
            a = getOperatorEffects(op_head, pVariable, con->exprs[0]);
            b = getOperatorEffects(op_head, pVariable, con->exprs[1]);
            join(a, b);
            return a;
        }

        auto exist = dynamic_cast<const ExistentialQuantification*>(exp);
        if (exist != nullptr) return getOperatorEffects(op_head, pVariable, exist->expr);

        auto neg = dynamic_cast<const Negation*>(exp);
        if (neg != nullptr) { // Swap add for del
            PosNegDomainFormula neg_expr = getOperatorEffects(op_head, pVariable, neg->expr);
            negate(neg_expr);
            return neg_expr;
        }

        auto var = dynamic_cast<const ParametrizedVariable*>(exp);
        if (var != nullptr) {
            if ((var->variableType == ParametrizedVariable::ACTION_FLUENT) and (var->variableName == op_head.name))
                return toDomainFormula(pVariable, getParamReplacement(op_head, var));
            else return PosNegDomainFormula();
        }

        // TODO numerical expressions for probabilities...

        NOT_IMPLEMENTED_OPERATOR;
        return PosNegDomainFormula();
    }

    PosNegDomainFormula
    RDDLOperatorUtils::getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const IfThenElseExpression *exp) {
        // Idea: - If condition is 1 -> add the predicate if then is 1, add ~predicate if then is 0, or join with the result
        //       - else -> add ~predicate if the condition is 1 (so if condition having the action fluent is false, then predicate is true means a negative effect)

        PosNegDomainFormula effects = getOperatorEffects(op_head, pVariable, exp->condition); // Checks if the condition has some implications on the effects of the operator
        if ((effects.neg.size() + effects.pos.size()) == 0) return effects; // If not found ignore the rest!

        PosNegDomainFormula ret = effects;
        auto iftrue = dynamic_cast<const NumericConstant*>(exp->valueIfTrue);
        if (iftrue != nullptr) {
            if (iftrue->value == 0) negate(ret);
            // else value == 1 -> return effects
        }
        else {
            PosNegDomainFormula res_if = getOperatorEffects(op_head, pVariable, iftrue);
            join(ret, res_if);
        }

        auto iffalse = dynamic_cast<const NumericConstant*>(exp->valueIfFalse);
        auto elseif = dynamic_cast<const IfThenElseExpression*>(exp->valueIfFalse);
        if (iffalse != nullptr) { // We have a true/False value
            PosNegDomainFormula cpy = effects;
            if (iffalse->value == 1)  negate(cpy); // Negate the value
            join(ret, cpy);
        }
        else if (elseif != nullptr) { // Elseif is identical to an if
            PosNegDomainFormula elseif_result = getOperatorEffects(op_head, pVariable, elseif);
            join(ret, elseif_result);
        }
        else {
            PosNegDomainFormula res_else = getOperatorEffects(op_head, pVariable, iftrue);
            join(ret, res_else);
        }

        return ret;
    }

    std::map<std::string, std::string>
    RDDLOperatorUtils::getParamReplacement(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                           const ParametrizedVariable *op_var) {
        assert(op_head.name == op_var->variableName and op_head.typed_parameters.size() == op_var->params.size());
        std::map<std::string, std::string> assign;

        for (size_t i = 0; i < op_var->params.size(); ++i) {
            assert(op_var->params[i]->type->name ==  op_head.typed_parameters[i].value);
            assign[op_var->params[i]->name] = op_head.typed_parameters[i].key;
        }

        return assign;
    }


}