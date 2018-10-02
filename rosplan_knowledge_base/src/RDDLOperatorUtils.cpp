//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 25/09/18.
//

#include <rosplan_knowledge_base/RDDLOperatorUtils.h>
#include <rosplan_knowledge_base/RDDLExprUtils.h>

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

    void RDDLOperatorUtils::join(vectorDA &a, vectorDA &b) {
        a.insert(a.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
    }


    void RDDLOperatorUtils::join(EffectDomainFormula &a, EffectDomainFormula &b) {
        a.add.insert(a.add.end(), std::make_move_iterator(b.add.begin()), std::make_move_iterator(b.add.end()));
        a.del.insert(a.del.end(), std::make_move_iterator(b.del.begin()), std::make_move_iterator(b.del.end()));
        a.prob.insert(a.prob.end(), std::make_move_iterator(b.prob.begin()), std::make_move_iterator(b.prob.end()));
    }


    void RDDLOperatorUtils::negate(PosNegDomainFormula &p) {
        vectorDF aux = p.pos;
        p.pos = p.neg;
        p.neg = aux;
    }

    void RDDLOperatorUtils::negate(EffectDomainFormula &p) {
        vectorDF aux = p.add;
        p.add = p.del;
        p.del = aux;

        for (size_t i = 0; i < p.prob.size(); ++i) {
            aux =  p.prob[i].add_effects;
            p.prob[i].add_effects = p.prob[i].del_effects;
            p.prob[i].del_effects = aux;
        }
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

    EffectDomainFormula RDDLOperatorUtils::getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head,
                                                              const std::map<ParametrizedVariable *, LogicalExpression *> &CPFs) {
        EffectDomainFormula eff_ret;
        for (auto it = CPFs.begin(); it != CPFs.end(); ++it) {
            if (it->first->valueType->name == "real" or it->first->valueType->name == "int") continue;
            std::map<std::string, std::string> assign; // Replacement for each parameter of the cpd to the parameter of the action as defined in the operator header op_head
            EffectDomainFormula eff_i = getOperatorEffects(op_head, it->first, it->second, assign);
            join(eff_ret, eff_i);
        }
        return eff_ret;
    }

    vectorDA RDDLOperatorUtils::getOperatorAssignEffects(const rosplan_knowledge_msgs::DomainFormula &op_head,
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
    RDDLOperatorUtils::getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const LogicalExpression *exp, std::map<std::string, std::string>& assign) {
        auto ifte = dynamic_cast<const IfThenElseExpression*>(exp);
        if (ifte != nullptr) return getOperatorEffects(op_head, pVariable, ifte, assign);

        auto con = dynamic_cast<const Connective*>(exp);
        if (con != nullptr) {
            assert(con->exprs.size() == 2);
            EffectDomainFormula a, b;
            a = getOperatorEffects(op_head, pVariable, con->exprs[0], assign);
            b = getOperatorEffects(op_head, pVariable, con->exprs[1], assign);
            join(a, b);
            return a;
        }

        auto exist = dynamic_cast<const ExistentialQuantification*>(exp);
        if (exist != nullptr) return getOperatorEffects(op_head, pVariable, exist->expr, assign);

        auto neg = dynamic_cast<const Negation*>(exp);
        if (neg != nullptr) { // Swap add for del
            EffectDomainFormula neg_expr = getOperatorEffects(op_head, pVariable, neg->expr, assign);
            negate(neg_expr);
            return neg_expr;
        }

        auto var = dynamic_cast<const ParametrizedVariable*>(exp);
        if (var != nullptr) {
            if ((var->variableType == ParametrizedVariable::ACTION_FLUENT) and (var->variableName == op_head.name)) {
                assign = getParamReplacement(op_head, var);
                PosNegDomainFormula df = toDomainFormula(pVariable, assign);
                EffectDomainFormula eff;
                eff.add = df.pos;
                eff.del = df.neg;
                return eff;
            }
            else return EffectDomainFormula();
        }

        // TODO numerical expressions for probabilities...

        NOT_IMPLEMENTED_OPERATOR;
        return EffectDomainFormula();
    }

    EffectDomainFormula
    RDDLOperatorUtils::getOperatorEffects(const rosplan_knowledge_msgs::DomainFormula &op_head, const ParametrizedVariable *pVariable, const IfThenElseExpression *exp, std::map<std::string, std::string>& assign) {
        // Idea: - If condition is 1 -> add the predicate if then is 1, add ~predicate if then is 0, or join with the result
        //       - else -> add ~predicate if the condition is 1 (so if condition having the action fluent is false, then predicate is true means a negative effect)

        EffectDomainFormula effects = getOperatorEffects(op_head, pVariable, exp->condition, assign); // Checks if the condition has some implications on the effects of the operator
        if ((effects.add.size() + effects.add.size() + effects.prob.size()) == 0) return effects; // If not found ignore the rest!
        EffectDomainFormula ret = effects;

        // Check if probabilistic effect
        auto bern = dynamic_cast<const BernoulliDistribution*>(exp->valueIfTrue);
        if (bern != nullptr) {
            EffectDomainFormula b_eff = getOperatorEffects(pVariable, bern, assign);
            join(ret, b_eff); // Assign will be filled when we find an action fluent
        }
        bern = dynamic_cast<const BernoulliDistribution*>(exp->valueIfFalse);
        if (bern != nullptr) {
            EffectDomainFormula b_eff = getOperatorEffects(pVariable, bern, assign);
            join(ret, b_eff); // Assign will be filled when we find an action fluent
        }
        auto disc = dynamic_cast<const DiscreteDistribution*>(exp->valueIfTrue);
        if (disc != nullptr) {
            EffectDomainFormula b_eff = getOperatorEffects(pVariable, disc, assign);
            join(ret, b_eff); // Assign will be filled when we find an action fluent
        }
        disc = dynamic_cast<const DiscreteDistribution*>(exp->valueIfFalse);
        if (disc != nullptr) {
            EffectDomainFormula b_eff = getOperatorEffects(pVariable, disc, assign);
            join(ret, b_eff); // Assign will be filled when we find an action fluent
        }
        // Check if numerical or nested if effect


        auto iftrue = dynamic_cast<const NumericConstant*>(exp->valueIfTrue);
        if (iftrue != nullptr) {
            if (iftrue->value == 0) negate(ret);
            // else value == 1 -> return effects
        }
        else {
            EffectDomainFormula res_if = getOperatorEffects(op_head, pVariable, exp->valueIfTrue, assign);
            join(ret, res_if);
        }

        auto iffalse = dynamic_cast<const NumericConstant*>(exp->valueIfFalse);
        auto elseif = dynamic_cast<const IfThenElseExpression*>(exp->valueIfFalse);
        if (iffalse != nullptr) { // We have a true/False value
            EffectDomainFormula cpy = effects;
            if (iffalse->value == 1)  negate(cpy); // Negate the value
            join(ret, cpy);
        }
        else if (elseif != nullptr) { // Elseif is identical to an if
            EffectDomainFormula elseif_result = getOperatorEffects(op_head, pVariable, elseif, assign);
            join(ret, elseif_result);
        }
        else {
            EffectDomainFormula res_else = getOperatorEffects(op_head, pVariable, exp->valueIfFalse, assign);
            join(ret, res_else);
        }

        return ret;
    }

    vectorDA RDDLOperatorUtils::getOperatorAssignEffects(const rosplan_knowledge_msgs::DomainFormula &op_head,
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

    EffectDomainFormula
    RDDLOperatorUtils::getOperatorEffects(const ParametrizedVariable *pVariable, const BernoulliDistribution *exp,
                                          const std::map<std::string, std::string> &assign) {
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
    RDDLOperatorUtils::getOperatorEffects(const ParametrizedVariable *pVariable, const DiscreteDistribution *exp,
                                          const std::map<std::string, std::string> &assign) {
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


}