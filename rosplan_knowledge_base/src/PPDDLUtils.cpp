//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 25/09/18.
//

#include <rosplan_knowledge_base/PPDDLUtils.h>
#include "ros/ros.h"

namespace KCL_rosplan {


    void PPDDLUtils::fillPreconditions(const ppddl_parser::StateFormula &precondition, PPDDLDomainPtr domain,
                                               rosplan_knowledge_msgs::DomainFormula &op_head,
                                               vectorDF &out_pos_cond, vectorDF &out_neg_cond,
                                               std::map<ppddl_parser::Term, std::string> &var_decl,
                                               std::map<std::string, int> &var_names) {
        // check which type of condition we have
        auto a = dynamic_cast<const ppddl_parser::Atom*>(&precondition);
        if (a != nullptr) {
            rosplan_knowledge_msgs::DomainFormula df = getAtom(a, domain, var_decl);
            out_pos_cond.push_back(df);
        }

        auto n = dynamic_cast<const ppddl_parser::Negation*>(&precondition); // Change order of pos and neg list
        if (n != nullptr) fillPreconditions(n->negand(), domain, op_head, out_neg_cond, out_pos_cond, var_decl, var_names);

        auto cjt = dynamic_cast<const ppddl_parser::Conjunction*>(&precondition);
        if (cjt != nullptr) {
            for (auto it = cjt->conjuncts().begin(); it != cjt->conjuncts().end(); ++it) {
                fillPreconditions(**it, domain, op_head, out_pos_cond, out_neg_cond, var_decl, var_names);
            }
        }

        auto djt = dynamic_cast<const ppddl_parser::Disjunction*>(&precondition);
        if (djt != nullptr) NOT_IMPLEMENTED("Disjunctions are not implemented for preconditions");;

        auto ex = dynamic_cast<const ppddl_parser::Exists*>(&precondition);
        if (ex != nullptr) {
            ROS_INFO("KCL (PPDDLKnowledgeBase) Adding exists variables as action parameters.");
            // Add parameters to the operator
            ppddl_parser::VariableList params = ex->parameters();
            for (auto it = params.begin(); it != params.end(); ++it) {
                diagnostic_msgs::KeyValue param;
                param.value = domain->types().typestring(domain->terms().type(*it)); // type name
                std::string vname = param.value.substr(0, 1);
                if (var_names.find(vname) != var_names.end()) var_names[vname] = 0;
                else {
                    ++var_names[vname];
                    vname += std::to_string(var_names[vname]);
                }
                var_decl[ppddl_parser::Term(*it)] = vname;
                param.key = vname;
                op_head.typed_parameters.push_back(param);
            }

            // Process exists' body
            fillPreconditions(ex->body(), domain, op_head, out_pos_cond, out_neg_cond, var_decl, var_names);
        }

        auto fa = dynamic_cast<const ppddl_parser::Forall*>(&precondition);
        if (fa != nullptr) NOT_IMPLEMENTED("ForAll is not implemented for preconditions");;

        auto eq = dynamic_cast<const ppddl_parser::Equality*>(&precondition);
        if (eq != nullptr) NOT_IMPLEMENTED("Equality is not implemented for preconditions");

        auto cmplt = dynamic_cast<const ppddl_parser::LessThan*>(&precondition);
        if (cmplt != nullptr) NOT_IMPLEMENTED("Less than is not implemented for preconditions");

        auto cmpeq = dynamic_cast<const ppddl_parser::EqualTo*>(&precondition);
        if (cmpeq != nullptr) NOT_IMPLEMENTED("Equal to is not implemented for preconditions");

        auto cmpgte = dynamic_cast<const ppddl_parser::GreaterThanOrEqualTo*>(&precondition);
        if (cmpgte != nullptr) NOT_IMPLEMENTED("Greater Than or Equal To is not implemented for preconditions");

        auto cmpgt = dynamic_cast<const ppddl_parser::GreaterThan*>(&precondition);
        if (cmpgt != nullptr) NOT_IMPLEMENTED("Greater than is not implemented for preconditions");

        NOT_IMPLEMENTED_OPERATOR;
    }

    void PPDDLUtils::fillEffects(const ppddl_parser::Effect &effect, PPDDLDomainPtr domain, vectorDF &out_add_eff,
                                         vectorDF &out_del_eff, vectorPE &out_pr_eff,
                                         vectorDA &assign_eff, std::map<ppddl_parser::Term, std::string> &var_decl) {

        auto se = dynamic_cast<const ppddl_parser::SimpleEffect*>(&effect);
        if (se != nullptr) {
            const ppddl_parser::Atom *a = &se->atom();
            rosplan_knowledge_msgs::DomainFormula df = getAtom(a, domain, var_decl);

            auto ae = dynamic_cast<const ppddl_parser::AddEffect*>(se);
            if (ae != nullptr) out_add_eff.push_back(df);
            else out_del_eff.push_back(df);
        }

        auto ue = dynamic_cast<const ppddl_parser::UpdateEffect*>(&effect);
        if (ue != nullptr) {
            assign_eff.push_back(getUpdate(ue, domain, var_decl));
        }

        auto ce = dynamic_cast<const ppddl_parser::ConjunctiveEffect*>(&effect);
        if (ce != nullptr) {
            for (auto it = ce->conjuncts().begin(); it != ce->conjuncts().end(); ++it) {
                fillEffects(**it, domain, out_add_eff, out_del_eff, out_pr_eff, assign_eff, var_decl);
            }
        }

        auto pe = dynamic_cast<const ppddl_parser::ProbabilisticEffect*>(&effect);
        if (pe != nullptr) {
            for (size_t i = 0; i < pe->size(); ++i) {
                rosplan_knowledge_msgs::ProbabilisticEffect e;
                rosplan_knowledge_msgs::ExprBase prob;
                prob.expr_type = rosplan_knowledge_msgs::ExprBase::CONSTANT;
                prob.constant = pe->probability(i).double_value();
                e.probability.tokens.push_back(prob);
                fillEffects(pe->effect(i), domain, e.add_effects, e.del_effects, out_pr_eff, assign_eff, var_decl);
                out_pr_eff.push_back(e);
            }
        }

        auto qe = dynamic_cast<const ppddl_parser::QuantifiedEffect*>(&effect);
        if (qe != nullptr) NOT_IMPLEMENTED("Quantified Effects are not implemented.");

        auto conde = dynamic_cast<const ppddl_parser::ConditionalEffect*>(&effect);
        if (conde != nullptr) NOT_IMPLEMENTED("Conditional Effects are not implemented.");

        NOT_IMPLEMENTED_OPERATOR;
    }

    rosplan_knowledge_msgs::DomainAssignment
    PPDDLUtils::getUpdate(const ppddl_parser::UpdateEffect *ue, PPDDLDomainPtr domain, std::map<ppddl_parser::Term, string> &var_decl) {
        rosplan_knowledge_msgs::DomainAssignment assignment;
        const ppddl_parser::Update* up = &ue->update();
        if (dynamic_cast<const ppddl_parser::Assign*>(up) != nullptr) {
            assignment.assign_type = rosplan_knowledge_msgs::DomainAssignment::ASSIGN;
        }
        else if (dynamic_cast<const ppddl_parser::ScaleUp*>(up) != nullptr) {
            assignment.assign_type = rosplan_knowledge_msgs::DomainAssignment::SCALE_UP;
        }
        else if (dynamic_cast<const ppddl_parser::ScaleDown*>(up) != nullptr) {
            assignment.assign_type = rosplan_knowledge_msgs::DomainAssignment::SCALE_DOWN;
        }
        else if (dynamic_cast<const ppddl_parser::Increase*>(up) != nullptr) {
            assignment.assign_type = rosplan_knowledge_msgs::DomainAssignment::INCREASE;
        }
        else if (dynamic_cast<const ppddl_parser::Decrease*>(up) != nullptr) {
            assignment.assign_type = rosplan_knowledge_msgs::DomainAssignment::DECREASE;
        }
        assignment.LHS = getExpression(up->fluent(), domain, var_decl).tokens[0].function;
        assignment.RHS = getExpression(up->expression(), domain, var_decl);
        return assignment;
    }

    rosplan_knowledge_msgs::ExprComposite
    PPDDLUtils::getExpression(const ppddl_parser::Expression &exp, PPDDLDomainPtr domain, std::map<ppddl_parser::Term, string> &var_decl) {
        rosplan_knowledge_msgs::ExprComposite ret;
        rosplan_knowledge_msgs::ExprBase base;

        auto v = dynamic_cast<const ppddl_parser::Value*>(&exp);
        if (v != nullptr) {
            base.expr_type = rosplan_knowledge_msgs::ExprBase::CONSTANT;
            base.constant = v->value().double_value();
            ret.tokens.push_back(base);
        }

        auto f = dynamic_cast<const ppddl_parser::Fluent*>(&exp);
        if (f != nullptr) {
            base.expr_type = rosplan_knowledge_msgs::ExprBase::FUNCTION;
            rosplan_knowledge_msgs::DomainFormula df;
            df.name = domain->functions().name(f->function());
            ppddl_parser::TermList tl = f->terms();
            ppddl_parser::TypeList params = domain->functions().parameters(f->function());
            assert(tl.size() == params.size());
            for (size_t i = 0; i < tl.size(); ++i) {
                diagnostic_msgs::KeyValue p;
                p.key = var_decl[tl[i]];
                p.value = domain->types().typestring(params[i]); // type
                df.typed_parameters.push_back(p);
            }
            base.function = df;
            ret.tokens.push_back(base);
        }

        auto comp = dynamic_cast<const ppddl_parser::Computation*>(&exp);
        if (comp != nullptr) {
            auto add = dynamic_cast<const ppddl_parser::Addition *>(&exp);
            if (add != nullptr) {
                base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
                base.op = rosplan_knowledge_msgs::ExprBase::ADD;
            }

            auto sub = dynamic_cast<const ppddl_parser::Subtraction *>(&exp);
            if (sub != nullptr) {
                base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
                base.op = rosplan_knowledge_msgs::ExprBase::SUB;
            }

            auto mul = dynamic_cast<const ppddl_parser::Multiplication *>(&exp);
            if (mul != nullptr) {
                base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
                base.op = rosplan_knowledge_msgs::ExprBase::MUL;
            }

            auto div = dynamic_cast<const ppddl_parser::Division *>(&exp);
            if (div != nullptr) {
                base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
                base.op = rosplan_knowledge_msgs::ExprBase::DIV;
            }

            ret.tokens.push_back(base);
            rosplan_knowledge_msgs::ExprComposite op1 = getExpression(add->operand1(), domain, var_decl);
            rosplan_knowledge_msgs::ExprComposite op2 = getExpression(add->operand2(), domain, var_decl);

            join(ret.tokens, op1.tokens);
            join(ret.tokens, op2.tokens);
        }
        return ret;
    }

    void PPDDLUtils::join(std::vector<rosplan_knowledge_msgs::ExprBase> &a,
                                  std::vector<rosplan_knowledge_msgs::ExprBase> &b) {
            a.insert(a.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
    }


    rosplan_knowledge_msgs::DomainFormula PPDDLUtils::getAtom(const ppddl_parser::Atom* a, PPDDLDomainPtr domain, std::map<ppddl_parser::Term, string> &var_decl) {
        rosplan_knowledge_msgs::DomainFormula df;
        df.name = domain->predicates().name(a->predicate());
        ppddl_parser::TermList tl = a->terms();
        ppddl_parser::TypeList params = domain->predicates().parameters(a->predicate());
        assert(tl.size() == params.size());
        std::map<std::string, int> var_names;
        for (size_t i = 0; i < tl.size(); ++i) {
            diagnostic_msgs::KeyValue p;
            if (tl[i].object()) {
                p.value = var_decl[tl[i]];
                std::string vname = domain->types().typestring(params[i]).substr(0, 1);
                if (var_names.find(vname) != var_names.end()) var_names[vname] = 0;
                else {
                    ++var_names[vname];
                    vname += std::to_string(var_names[vname]);
                }
                p.key = vname;
            }
            else {
                p.key = var_decl[tl[i]];
                p.value = domain->types().typestring(params[i]); // type
            }
            df.typed_parameters.push_back(p);
        }
    }

    void
    PPDDLUtils::fillGoal(const ppddl_parser::StateFormula &goal, PPDDLDomainPtr domain, PPDDLProblemPtr problem, std::vector<rosplan_knowledge_msgs::KnowledgeItem>& out_goal,
                         std::map<ppddl_parser::Term, std::string> &var_decl, bool is_negative) {

        // check which type of condition we have
        auto a = dynamic_cast<const ppddl_parser::Atom*>(&goal);
        if (a != nullptr) {
            rosplan_knowledge_msgs::DomainFormula df = getAtom(a, domain, var_decl);
            rosplan_knowledge_msgs::KnowledgeItem ki;
            ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            ki.attribute_name = df.name;
            ki.values = df.typed_parameters;
            ki.is_negative = is_negative;
            out_goal.push_back(ki);
        }

        auto n = dynamic_cast<const ppddl_parser::Negation*>(&goal); // Change order of pos and neg list
        if (n != nullptr) fillGoal(n->negand(), domain, problem, out_goal, var_decl, true);

        auto cjt = dynamic_cast<const ppddl_parser::Conjunction*>(&goal);
        if (cjt != nullptr) {
            for (auto it = cjt->conjuncts().begin(); it != cjt->conjuncts().end(); ++it) {
                fillGoal(**it, domain, problem, out_goal, var_decl, is_negative);
            }
        }

        auto djt = dynamic_cast<const ppddl_parser::Disjunction*>(&goal);
        if (djt != nullptr) NOT_IMPLEMENTED("Disjunctions are not implemented for goals");;

        auto ex = dynamic_cast<const ppddl_parser::Exists*>(&goal);
        if (ex != nullptr) NOT_IMPLEMENTED("Exists is not implemented for goals");

        auto fa = dynamic_cast<const ppddl_parser::Forall*>(&goal);
        if (fa != nullptr) { // Add all the instantiations
            std::map<std::string, int> var_names;
            for (auto it = fa->parameters().begin(); it != fa->parameters().end(); ++it) {
                // Iterate over the the objects
                ppddl_parser::ObjectList olist = problem->terms().compatible_objects(domain->terms().type(ppddl_parser::Term(*it)));
                for (auto o = olist.begin(); o != olist.end(); ++o) {
                    diagnostic_msgs::KeyValue param;
                    param.value = domain->terms().get_name(ppddl_parser::Term(*it));//domain->types().typestring(domain->terms().type(*it)); // type name
                    std::string vname = param.value.substr(0, 1);
                    if (var_names.find(vname) != var_names.end()) var_names[vname] = 0;
                    else {
                        ++var_names[vname];
                        vname += std::to_string(var_names[vname]);
                    }
                    var_decl[ppddl_parser::Term(*it)] = vname;
                    param.key = vname;

                    // Process forall' body
                    fillGoal(ex->body(), domain, problem, out_goal, var_decl, is_negative);
                }
            }
        }

        auto eq = dynamic_cast<const ppddl_parser::Equality*>(&goal);
        if (eq != nullptr) NOT_IMPLEMENTED("Term equality is not implemented");

        auto cmplt = dynamic_cast<const ppddl_parser::LessThan*>(&goal);
        if (cmplt != nullptr){
            rosplan_knowledge_msgs::DomainInequality di;
            di.comparison_type = rosplan_knowledge_msgs::DomainInequality::LESS;
            di.LHS = getExpression(cmplt->expr1(), domain, var_decl);
            di.RHS = getExpression(cmplt->expr2(), domain, var_decl);
            rosplan_knowledge_msgs::KnowledgeItem ki;
            ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INEQUALITY;
            ki.ineq = di;
            out_goal.push_back(ki);
        }
        auto cmplte = dynamic_cast<const ppddl_parser::LessThanOrEqualTo*>(&goal);
        if (cmplte != nullptr){
            rosplan_knowledge_msgs::DomainInequality di;
            di.comparison_type = rosplan_knowledge_msgs::DomainInequality::LESSEQ;
            di.LHS = getExpression(cmplte->expr1(), domain, var_decl);
            di.RHS = getExpression(cmplte->expr2(), domain, var_decl);
            rosplan_knowledge_msgs::KnowledgeItem ki;
            ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INEQUALITY;
            ki.ineq = di;
            out_goal.push_back(ki);
        }

        auto cmpeq = dynamic_cast<const ppddl_parser::EqualTo*>(&goal);
        if (cmpeq != nullptr) {
            rosplan_knowledge_msgs::DomainInequality di;
            di.comparison_type = rosplan_knowledge_msgs::DomainInequality::EQUALS;
            di.LHS = getExpression(cmpeq->expr1(), domain, var_decl);
            di.RHS = getExpression(cmpeq->expr2(), domain, var_decl);
            rosplan_knowledge_msgs::KnowledgeItem ki;
            ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INEQUALITY;
            ki.ineq = di;
            out_goal.push_back(ki);
        }

        auto cmpgte = dynamic_cast<const ppddl_parser::GreaterThanOrEqualTo*>(&goal);
        if (cmpgte != nullptr) {
            rosplan_knowledge_msgs::DomainInequality di;
            di.comparison_type = rosplan_knowledge_msgs::DomainInequality::GREATEREQ;
            di.LHS = getExpression(cmpgte->expr1(), domain, var_decl);
            di.RHS = getExpression(cmpgte->expr2(), domain, var_decl);
            rosplan_knowledge_msgs::KnowledgeItem ki;
            ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INEQUALITY;
            ki.ineq = di;
            out_goal.push_back(ki);
        }

        auto cmpgt = dynamic_cast<const ppddl_parser::GreaterThan*>(&goal);
        if (cmpgt != nullptr) {
            rosplan_knowledge_msgs::DomainInequality di;
            di.comparison_type = rosplan_knowledge_msgs::DomainInequality::GREATER;
            di.LHS = getExpression(cmpgt->expr1(), domain, var_decl);
            di.RHS = getExpression(cmpgt->expr2(), domain, var_decl);
            rosplan_knowledge_msgs::KnowledgeItem ki;
            ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INEQUALITY;
            ki.ineq = di;
            out_goal.push_back(ki);
        }

        NOT_IMPLEMENTED_OPERATOR;
    }
}