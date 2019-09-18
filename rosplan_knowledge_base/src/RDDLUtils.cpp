//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 25/09/18.
//

#include <rosplan_knowledge_base/RDDLUtils.h>
#include <rosplan_knowledge_base/RDDLExprUtils.h>

#include "rosplan_knowledge_base/RDDLUtils.h"
#include "ros/ros.h"

namespace KCL_rosplan {

    PosNegDomainFormula RDDLUtils::getOperatorPreconditions(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                                                    const std::vector<LogicalExpression *> &SACs) {
        PosNegDomainFormula ret;

        for (auto it = SACs.begin(); it != SACs.end(); ++it) {
            auto precs = getOperatorPrecondition(op_head, *it);
            join(ret, precs);
        }

        return ret;
    }

    PosNegDomainFormula RDDLUtils::getOperatorPrecondition(const rosplan_knowledge_msgs::DomainFormula &op_head, const LogicalExpression *SAC) {
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


    PosNegDomainFormula RDDLUtils::toDomainFormula(const LogicalExpression *expr,  const std::map<std::string, std::string>& assign) {
        auto exp_con = dynamic_cast<const Conjunction *>(expr);
        if (exp_con != nullptr) {
            assert(exp_con->exprs.size() == 2);
            PosNegDomainFormula op1, op2;
            op1 = toDomainFormula(exp_con->exprs[0], assign);
            op2 = toDomainFormula(exp_con->exprs[1], assign);
            join(op1, op2);
            return op1;
        }

        auto exp_neg = dynamic_cast<const Negation *>(expr);
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

        auto exp_quan = dynamic_cast<const Quantifier *>(expr);
        if (exp_quan != nullptr) {
            //NOT_IMPLEMENTED("Quantifiers are not implemented as a precondition");
            return PosNegDomainFormula();
            /*return getOperatorPrecondition( exp_quan->expr);*/
        }

        auto exp_equals = dynamic_cast<const EqualsExpression *>(expr);
        if (exp_equals != nullptr) {
            //NOT_IMPLEMENTED("Equality expressions are not implemented as a precondition");
            return PosNegDomainFormula();
        }
        NOT_IMPLEMENTED_OPERATOR;

        return PosNegDomainFormula();
    }


    void RDDLUtils::join(PosNegDomainFormula &a, PosNegDomainFormula &b) {
        a.pos.insert(a.pos.end(), std::make_move_iterator(b.pos.begin()), std::make_move_iterator(b.pos.end()));
        a.neg.insert(a.neg.end(), std::make_move_iterator(b.neg.begin()), std::make_move_iterator(b.neg.end()));
    }

    void RDDLUtils::join(vectorDA &a, vectorDA &b) {
        a.insert(a.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
    }


    void RDDLUtils::join(EffectDomainFormula &a, EffectDomainFormula &b) {
        a.add.insert(a.add.end(), std::make_move_iterator(b.add.begin()), std::make_move_iterator(b.add.end()));
        a.del.insert(a.del.end(), std::make_move_iterator(b.del.begin()), std::make_move_iterator(b.del.end()));
        a.prob.insert(a.prob.end(), std::make_move_iterator(b.prob.begin()), std::make_move_iterator(b.prob.end()));
    }

    void RDDLUtils::join(vectorKI &a, vectorKI &b) {
        a.insert(a.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
    }


    void RDDLUtils::negate(PosNegDomainFormula &p) {
        vectorDF aux = p.pos;
        p.pos = p.neg;
        p.neg = aux;
    }

    void RDDLUtils::negate(EffectDomainFormula &p) {
        vectorDF aux = p.add;
        p.add = p.del;
        p.del = aux;

        for (size_t i = 0; i < p.prob.size(); ++i) {
            aux =  p.prob[i].add_effects;
            p.prob[i].add_effects = p.prob[i].del_effects;
            p.prob[i].del_effects = aux;
        }
    }

    PosNegDomainFormula RDDLUtils::getOperatorPrecondition(const rosplan_knowledge_msgs::DomainFormula &op_head, const Connective *SAC) {
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


    PosNegDomainFormula RDDLUtils::getOperatorPrecondition(const rosplan_knowledge_msgs::DomainFormula &op_head, const Negation *SAC) {
        // Swap the resulting precondition
        PosNegDomainFormula neg = getOperatorPrecondition(op_head, SAC->expr);
        negate(neg);
        return neg;
    }

    EffectDomainFormula RDDLUtils::getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                                              const std::map<ParametrizedVariable *, LogicalExpression *> &CPFs) {
        EffectDomainFormula eff_ret;
        for (auto it = CPFs.begin(); it != CPFs.end(); ++it) {
            if (it->first->valueType->name == "real" or it->first->valueType->name == "int") continue;
            std::map<std::string, std::string> assign; // Replacement for each parameter of the cpd to the parameter of the action as defined in the operator header op_head
            bool action_found = false, exogenous = true;
            EffectDomainFormula eff_i = getOperatorEffects(op_head, it->first, it->second, assign, action_found, exogenous);
            if (not action_found) { // If an action fluent was found, it will return empty lists so it means it was an exogenous event.
                if (op_head.name != "exogenous") continue;
                else if (exogenous and eff_i.add.empty() and eff_i.del.empty() and eff_i.prob.empty()) { // Action name is exogenous and we found a fluent without any related action
                    PosNegDomainFormula df = toDomainFormula(it->first, assign);
                    eff_i.add = df.pos;
                    eff_i.del = df.neg;
                }
                else if (not exogenous) { // eff_i will contain probabilistic effects if any, wrongly adding them if it
                                          // was not in an exogenous effect. Thus, we empty the list.
                    eff_i.add.clear(); eff_i.del.clear(); eff_i.prob.clear();
                }
            }
            join(eff_ret, eff_i);
        }
        return eff_ret;
    }

    vectorDA RDDLUtils::getOperatorAssignEffects(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                                         const std::map<ParametrizedVariable *, LogicalExpression *> &CPFs) {
        vectorDA ret;
        for (auto it = CPFs.begin(); it != CPFs.end(); ++it) {
            if (it->first->valueType->name != "real" and it->first->valueType->name != "int") continue;
            auto ifte = dynamic_cast<const IfThenElseExpression*>(it->second);
            if (ifte != nullptr) {
                vectorDA ass_i = getOperatorAssignEffects(op_head, it->first, ifte);
                join(ret, ass_i);
            }
        }
        return ret;
    }


    EffectDomainFormula
    RDDLUtils::getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const LogicalExpression *exp, std::map<std::string, std::string>& assign, bool& action_found, bool& exogenous) {
        auto ifte = dynamic_cast<const IfThenElseExpression*>(exp);
        if (ifte != nullptr) return getOperatorEffects(op_head, pVariable, ifte, assign, action_found, exogenous);

        auto con = dynamic_cast<const Connective*>(exp);
        if (con != nullptr) {
            assert(con->exprs.size() == 2);
            EffectDomainFormula a, b;
            a = getOperatorEffects(op_head, pVariable, con->exprs[0], assign, action_found, exogenous);
            b = getOperatorEffects(op_head, pVariable, con->exprs[1], assign, action_found, exogenous);
            join(a, b);
            return a;
        }

        auto exist = dynamic_cast<const ExistentialQuantification*>(exp);
        if (exist != nullptr) return getOperatorEffects(op_head, pVariable, exist->expr, assign, action_found, exogenous);

        auto forall = dynamic_cast<const UniversalQuantification*>(exp);
        if (forall != nullptr) {
            EffectDomainFormula forall_formula;
            fillForAllEffect(op_head, pVariable, forall, forall_formula, 0, assign, action_found, exogenous);
            return forall_formula;
        }

        auto neg = dynamic_cast<const Negation*>(exp);
        if (neg != nullptr) { // Swap add for del
            EffectDomainFormula neg_expr = getOperatorEffects(op_head, pVariable, neg->expr, assign, action_found, exogenous);
            negate(neg_expr);
            return neg_expr;
        }

        auto var = dynamic_cast<const ParametrizedVariable*>(exp);
        if (var != nullptr) {
            if (var->variableType == ParametrizedVariable::ACTION_FLUENT) {
                exogenous = false;
                if (var->variableName == op_head.name) {
                    action_found = true;
                    assign = getParamReplacement(op_head, var);
                    PosNegDomainFormula df = toDomainFormula(pVariable, assign);
                    EffectDomainFormula eff;
                    eff.add = df.pos;
                    eff.del = df.neg;
                    return eff;
                }
            }
            return EffectDomainFormula();
        }

        auto nexp = dynamic_cast<const NumericConstant*>(exp);
        if (nexp != nullptr) {
            EffectDomainFormula eff;
            if (pVariable->valueType->name == "bool") { // Assign true or false, values are handled by the getOperatorAssignEffects
                PosNegDomainFormula df = toDomainFormula(pVariable, assign);
                if (nexp->value == 0) { // false
                    eff.del = df.pos;
                    eff.add = df.neg;
                }
                else { // true
                    eff.add = df.pos;
                    eff.del = df.pos;
                }
            }
            return eff;
        }

        // Check if probabilistic effect
        auto bern = dynamic_cast<const BernoulliDistribution*>(exp);
        if (bern != nullptr) {
            return getOperatorEffects(pVariable, bern, assign, action_found, exogenous);
        }

        auto disc = dynamic_cast<const DiscreteDistribution*>(exp);
        if (disc != nullptr) {
            return getOperatorEffects(pVariable, disc, assign, action_found, exogenous);
        }

        NOT_IMPLEMENTED("Unknown or unsupported operand type for the action effects.");
        return EffectDomainFormula();
    }

    EffectDomainFormula
    RDDLUtils::getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const IfThenElseExpression *exp, std::map<std::string, std::string>& assign, bool& action_found, bool& exogenous) {
        // Idea: - If condition is 1 -> add the predicate if then is 1, add ~predicate if then is 0, or join with the result
        //       - else -> add ~predicate if the condition is 1 (so if condition having the action fluent is false, then predicate is true means a negative effect)

        EffectDomainFormula effects = getOperatorEffects(op_head, pVariable, exp->condition, assign, action_found, exogenous); // Checks if the condition has some implications on the effects of the operator

        EffectDomainFormula ret;

        // Check if numerical or nested if effect


        auto iftrue = dynamic_cast<const NumericConstant*>(exp->valueIfTrue);
        if (iftrue != nullptr) {
            if (iftrue->value == 0) {
                // ret has only probabilistic effects here, which make no sens to negate. So we add effects here
                EffectDomainFormula cpy = effects;
                negate(cpy); // Negate the value
                join(ret, cpy);
            }
            else { //value == 1 -> return effects
                join(ret, effects);
            }
        }
        else {
            EffectDomainFormula res_if = getOperatorEffects(op_head, pVariable, exp->valueIfTrue, assign, action_found, exogenous);
            join(ret, res_if);
        }

        auto iffalse = dynamic_cast<const NumericConstant*>(exp->valueIfFalse);
        auto elseif = dynamic_cast<const IfThenElseExpression*>(exp->valueIfFalse);
        if (iffalse != nullptr) { // We have a true/False value
            /*if (iffalse->value == 1)  {
                EffectDomainFormula cpy = effects;
                negate(cpy); // Negate the value
                join(ret, cpy);
            }*/
            // Note: if iffalse is a true/false value, it doesn't say anything relative to the action per se.
            //       Therefore, the best thing is to ignore the else case in that situation, as the else case will
            //       usually mean that the action was not executed, so the else clause will not provide information on
            //       the action effects.
        }
        else if (elseif != nullptr) { // Elseif is identical to an if
            EffectDomainFormula elseif_result = getOperatorEffects(op_head, pVariable, elseif, assign, action_found, exogenous);
            join(ret, elseif_result);
        }
        else {
            EffectDomainFormula res_else = getOperatorEffects(op_head, pVariable, exp->valueIfFalse, assign, action_found, exogenous);
            join(ret, res_else);
        }

        return ret;
    }

    vectorDA RDDLUtils::getOperatorAssignEffects(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                                         const ParametrizedVariable *pVariable,
                                                         const IfThenElseExpression *exp) {
        // Assumption: an assignment for an operator effect is of the form:
        //      fluent = if exists operator-fluent then expression else expression2;
        //  Here we weill only process the case in the iftrue part of the ifthenelse expression.
        //  Only will process operations of the type fluent = fluent +- (expression) <- parenthesis are important if it is a non constant expression
        vectorDA ret;
        auto exist = dynamic_cast<const ExistentialQuantification*>(exp->condition);
        if (exist != nullptr) {
            auto var = dynamic_cast<const ParametrizedVariable*>(exist->expr);
            if ((var != nullptr) and (var->variableType == ParametrizedVariable::ACTION_FLUENT) and (var->variableName == op_head.name)) {
                auto assign = getParamReplacement(op_head, var);
                PosNegDomainFormula iftrue = toDomainFormula(pVariable, assign);
                // iftrue will be the formula of the left hand side

                rosplan_knowledge_msgs::ExprComposite rhs;
                rosplan_knowledge_msgs::DomainAssignment::_assign_type_type assignment_type;

                auto numconst = dynamic_cast<const NumericConstant*>(exp->valueIfTrue);
                if (numconst == nullptr) {
                    auto addition = dynamic_cast<const Addition *>(exp->valueIfTrue);
                    auto subtr = dynamic_cast<const Subtraction *>(exp->valueIfTrue);
                    if (addition == nullptr and subtr == nullptr and numconst == nullptr) return ret; // Ignoring it!

                    if (addition != nullptr) assignment_type = rosplan_knowledge_msgs::DomainAssignment::INCREASE;
                    else assignment_type = rosplan_knowledge_msgs::DomainAssignment::DECREASE;

                    auto connective = dynamic_cast<const Connective *>(exp->valueIfTrue);

                    // check that the left hand side is actually the the fluent of the cpfs
                    auto connvar = dynamic_cast<const ParametrizedVariable *>(connective->exprs[0]);
                    if (connvar != nullptr) {
                        if (connvar->variableName != pVariable->variableName) return ret;  // Ignoring it!
                        rhs = RDDLExprUtils::getExpression(connective->exprs[1], assign);
                    }
                    else return ret;  // Ignoring it!
                }
                else {
                    rhs = RDDLExprUtils::getExpression(numconst, assign);
                    assignment_type = rosplan_knowledge_msgs::DomainAssignment::ASSIGN_CTS;
                }

                for (auto it = iftrue.pos.begin(); it != iftrue.pos.end(); ++it) {
                    rosplan_knowledge_msgs::DomainAssignment da;
                    da.assign_type = assignment_type;
                    da.grounded = false;
                    da.LHS = *it;
                    da.RHS = rhs;
                    ret.push_back(da);
                }
            }
        }
        return ret;
    }

    std::map<std::string, std::string>
    RDDLUtils::getParamReplacement(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                           const ParametrizedVariable *op_var) {
        assert(op_head.name == op_var->variableName and op_head.typed_parameters.size() == op_var->params.size());
        std::map<std::string, std::string> assign;

        for (size_t i = 0; i < op_var->params.size(); ++i) {
            assert(op_var->params[i]->type->name ==  op_head.typed_parameters[i].value);
            assign[op_var->params[i]->name] = op_head.typed_parameters[i].key;
        }

        return assign;
    }

    EffectDomainFormula
    RDDLUtils::getOperatorEffects(const ParametrizedVariable *pVariable, const BernoulliDistribution *exp,
                                          const std::map<std::string, std::string> &assign, bool& action_found, bool& exogenous) {
        EffectDomainFormula ret;
        rosplan_knowledge_msgs::ProbabilisticEffect eff;
        eff.probability = RDDLExprUtils::getExpression(exp->expr, assign);
        PosNegDomainFormula df = toDomainFormula(pVariable, assign);
        eff.add_effects = df.pos;
        eff.del_effects = df.neg;
        ret.prob.push_back(eff);
        return ret;
    }

    EffectDomainFormula
    RDDLUtils::getOperatorEffects(const ParametrizedVariable *pVariable, const DiscreteDistribution *exp,
                                          const std::map<std::string, std::string> &assign, bool& action_found, bool& exogenous) {
        // A discrete effect will be N probabilistic effects consisting of an assignment to the variable
        EffectDomainFormula ret;
        // Get Variable representation
        rosplan_knowledge_msgs::DomainFormula df = toDomainFormula(pVariable, assign).pos[0];
        for (size_t i = 0; i < exp->values.size(); ++i) {
            rosplan_knowledge_msgs::ProbabilisticEffect eff;
            rosplan_knowledge_msgs::DomainAssignment domain_assign;
            domain_assign.assign_type = rosplan_knowledge_msgs::DomainAssignment::ASSIGN;
            domain_assign.LHS = df;
            domain_assign.RHS = RDDLExprUtils::getExpression(exp->values[i], assign);
            eff.assign_effects.push_back(domain_assign);
            eff.probability = RDDLExprUtils::getExpression(exp->probabilities[i], assign);
            ret.prob.push_back(eff);
        }
        return ret;
    }

    void
    RDDLUtils::fillForAllEffect(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const UniversalQuantification *exp, EffectDomainFormula& out, size_t paramid,
                                        std::map<std::string, std::string> &assign, bool& action_found, bool& exogenous) {
        // Instantiate all the objects.
        if (paramid == exp->paramList->params.size()) {
            EffectDomainFormula inst = getOperatorEffects(op_head, pVariable, exp->expr, assign, action_found, exogenous);
            join(out, inst);
        }
        else {
            Parameter* param = exp->paramList->params[paramid];
            for (auto o = exp->paramList->types[paramid]->objects.begin(); o != exp->paramList->types[paramid]->objects.end(); ++o) {
                assign[param->name] = (*o)->name;
                fillForAllEffect(op_head, pVariable, exp, out, paramid+1, assign, action_found, exogenous);
            }
            assign.erase(param->name);
        }
    }

    std::vector<rosplan_knowledge_msgs::KnowledgeItem> RDDLUtils::getGoals(const std::map<ParametrizedVariable*, LogicalExpression*>& CPFs) {
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> ret;
        for (auto it = CPFs.begin(); it != CPFs.end(); ++it) {
            if (it->first->variableName == "goal") {
                std::map<std::string, std::string> ass;
                return getGoals(it->second, false, ass);
            }
        }
        return ret;
    }

    vectorKI RDDLUtils::getGoals(const LogicalExpression *exp, bool is_negative, std::map<std::string, std::string> &assign) {
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> ret;
        auto conj = dynamic_cast<const Conjunction*>(exp);
        if (conj != nullptr) {
            for (auto it = conj->exprs.begin(); it != conj->exprs.end(); ++it) {
                auto goals = getGoals(*it, is_negative, assign);
                join(ret, goals);
            }
            return ret;
        }

        auto disj = dynamic_cast<const Disjunction*>(exp);
        if (disj != nullptr) {
            // Assuming goal of type goal = goal | expression. Only disjunction can be between goal and something else
            for (auto it = disj->exprs.begin(); it != disj->exprs.end(); ++it) {
                auto varname = dynamic_cast<const ParametrizedVariable*>(*it);
                if (varname != nullptr and varname->variableName == "goal") continue;
                auto goals = getGoals(*it, is_negative, assign);
                join(ret, goals);
                break;
            }
            return ret;
        }

        auto forall = dynamic_cast<const UniversalQuantification*>(exp);
        if (forall != nullptr) {
            fillForAllGoal(forall, ret, 0, is_negative, assign);
            return ret;
        }

        auto exp_var = dynamic_cast<const ParametrizedVariable*>(exp);
        if (exp_var != nullptr) {
            rosplan_knowledge_msgs::KnowledgeItem ki;
            ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            ki.attribute_name = exp_var->variableName;
            ki.is_negative = is_negative;
            for (auto pit = exp_var->params.begin(); pit != exp_var->params.end(); ++pit) {
                diagnostic_msgs::KeyValue param;

                auto param_name = assign.find((*pit)->name);
                if (param_name != assign.end()) param.key = param_name->second; // Parameter name (i.e. ?r)
                else param.key = (*pit)->name;

                size_t pos = param.key.find('?');
                if (pos != std::string::npos) param.key.erase(pos, 1); // Remove the ? if present

                param.value = (*pit)->type->name; // Type name
                ki.values.push_back(param);
            }
            ret.push_back(ki);
            return ret;
        }

        auto negation = dynamic_cast<const Negation*>(exp);
        if (negation != nullptr) {
            return getGoals(negation->expr, not is_negative, assign);
        }
        NOT_IMPLEMENTED("Unknown or unsupported operand type for the goal definition.");
        return ret;
    }


    void
    RDDLUtils::fillForAllGoal(const UniversalQuantification *exp, vectorKI& out, size_t paramid, bool is_negative, std::map<std::string, std::string> &assign) {
        // Instantiate all the objects.
        if (paramid == exp->paramList->params.size()) {
            vectorKI inst = getGoals(exp->expr, is_negative, assign);
            join(out, inst);
        }
        else {
            Parameter* param = exp->paramList->params[paramid];
            for (auto o = exp->paramList->types[paramid]->objects.begin(); o != exp->paramList->types[paramid]->objects.end(); ++o) {
                assign[param->name] = (*o)->name;
                fillForAllGoal(exp, out, paramid+1, is_negative, assign);
            }
            assign.erase(param->name);
        }
    }


}