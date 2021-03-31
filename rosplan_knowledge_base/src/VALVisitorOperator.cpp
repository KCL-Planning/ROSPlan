#include "rosplan_knowledge_base/VALVisitorOperator.h"

/* implementation of rosplan_knowledge_base::VALVisitorOperator */
namespace KCL_rosplan {

    /* encoding state */
    bool cond_neg;
    bool eff_neg;
    VAL1_2::time_spec cond_time;
    VAL1_2::time_spec eff_time;
    VAL1_2::comparison_op comparison_op;

    /*-----------*/
    /* operators */
    /*-----------*/

    /**
     * Visit an operator to pack into ROS message
     */
    void VALVisitorOperator::visit_operator_(VAL1_2::operator_ * op) {

        msg.formula.name = op->name->symbol::getName();
        msg.formula.typed_parameters.clear();

        // parameters
        for (VAL1_2::var_symbol_list::const_iterator vi = op->parameters->begin(); vi != op->parameters->end(); vi++) {
            const VAL1_2::var_symbol* var = *vi;
            diagnostic_msgs::KeyValue param;
            param.key = var->pddl_typed_symbol::getName();
            if (var->type != nullptr) param.value = var->type->getName();
            else param.value = "object";
            msg.formula.typed_parameters.push_back(param);
        }

        // prepare message
        msg.at_start_add_effects.clear();
        msg.at_start_del_effects.clear();
        msg.at_end_add_effects.clear();
        msg.at_end_del_effects.clear();
        msg.at_start_simple_condition.clear();
        msg.over_all_simple_condition.clear();
        msg.at_end_simple_condition.clear();
        msg.at_start_neg_condition.clear();
        msg.over_all_neg_condition.clear();
        msg.at_end_neg_condition.clear();
        msg.at_start_comparison.clear();
        msg.at_end_comparison.clear();
        msg.over_all_comparison.clear();

        // effects
        op->effects->visit(this);

        // conditions
        if (op->precondition) op->precondition->visit(this);
    }

    /*--------------*/
    /* propositions */
    /*--------------*/

    /**
     * Visit an prop to pack into ROS message
     */
    void VALVisitorOperator::visit_proposition(VAL1_2::proposition *p) {

        last_prop.typed_parameters.clear();

        // predicate name
        rosplan_knowledge_msgs::DomainFormula formula;
        last_prop.name = p->head->symbol::getName();

        // predicate variables
        for (VAL1_2::parameter_symbol_list::const_iterator vi = p->args->begin(); vi != p->args->end(); vi++) {
            const VAL1_2::parameter_symbol* var = *vi;
            diagnostic_msgs::KeyValue param;
            param.key = var->pddl_typed_symbol::getName();
            if (var->type != nullptr)  param.value = var->type->getName();
            else param.value = "object";
            last_prop.typed_parameters.push_back(param);
        }

    };

    /*-------*/
    /* goals */
    /*-------*/

    void VALVisitorOperator::visit_conj_goal(VAL1_2::conj_goal *c) {
        c->getGoals()->visit(this);
    }

    void VALVisitorOperator::visit_timed_goal(VAL1_2::timed_goal *c) {
        cond_time = c->getTime();
        c->getGoal()->visit(this);
    }

    void VALVisitorOperator::visit_neg_goal(VAL1_2::neg_goal *c) {
        cond_neg = !cond_neg;
        c->getGoal()->visit(this);
        cond_neg = !cond_neg;
    }

    void VALVisitorOperator::visit_simple_goal(VAL1_2::simple_goal *c) {

        c->getProp()->visit(this);

        if(cond_neg) {
            switch(cond_time) {
            case VAL1_2::E_AT_START: msg.at_start_neg_condition.push_back(last_prop); break;
            case VAL1_2::E_AT_END: msg.at_end_neg_condition.push_back(last_prop); break;
            case VAL1_2::E_OVER_ALL: msg.over_all_neg_condition.push_back(last_prop); break;
            }
        } else {
            switch(cond_time) {
            case VAL1_2::E_AT_START: msg.at_start_simple_condition.push_back(last_prop); break;
            case VAL1_2::E_AT_END: msg.at_end_simple_condition.push_back(last_prop); break;
            case VAL1_2::E_OVER_ALL: msg.over_all_simple_condition.push_back(last_prop); break;
            }
        }
    }

    void VALVisitorOperator::visit_comparison(VAL1_2::comparison * c) {

        rosplan_knowledge_msgs::DomainInequality ineq;
        ineq.grounded = false;

        // assignment left hand side
        last_expr.tokens.clear();
        c->getLHS()->visit(this);
        ineq.LHS = last_expr;

        // assignment right hand side
        last_expr.tokens.clear();
        c->getRHS()->visit(this);
        ineq.RHS = last_expr;

        // assignment operator
        switch(c->getOp()) {
            case VAL1_2::E_GREATER: ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::GREATER; break;
            case VAL1_2::E_GREATEQ: ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::GREATEREQ; break;
            case VAL1_2::E_LESS:    ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::LESS; break;
            case VAL1_2::E_LESSEQ:  ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::LESSEQ; break;
            case VAL1_2::E_EQUALS:  ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::EQUALS; break;
        }

        switch(cond_time) {
            case VAL1_2::E_AT_START: msg.at_start_comparison.push_back(ineq); break;
            case VAL1_2::E_AT_END: msg.at_end_comparison.push_back(ineq); break;
            case VAL1_2::E_OVER_ALL: msg.over_all_comparison.push_back(ineq); break;
        }
    }

    void VALVisitorOperator::visit_qfied_goal(VAL1_2::qfied_goal *) {}

    void VALVisitorOperator::visit_disj_goal(VAL1_2::disj_goal *) {}

    void VALVisitorOperator::visit_imply_goal(VAL1_2::imply_goal *) {}

    /*---------*/
    /* effects */
    /*---------*/

    void VALVisitorOperator::visit_effect_lists(VAL1_2::effect_lists * e) {

        eff_neg = false;
        e->add_effects.pc_list<VAL1_2::simple_effect*>::visit(this);
        eff_neg = true;
        e->del_effects.pc_list<VAL1_2::simple_effect*>::visit(this);
        eff_neg = false;

        e->forall_effects.pc_list<VAL1_2::forall_effect*>::visit(this);
        e->cond_effects.pc_list<VAL1_2::cond_effect*>::visit(this);
        e->cond_assign_effects.pc_list<VAL1_2::cond_effect*>::visit(this);
        e->assign_effects.pc_list<VAL1_2::assignment*>::visit(this);
        e->timed_effects.pc_list<VAL1_2::timed_effect*>::visit(this);
    }

    void VALVisitorOperator::visit_timed_effect(VAL1_2::timed_effect * e) {
        eff_time = e->ts;
        e->effs->visit(this);
    };

    void VALVisitorOperator::visit_simple_effect(VAL1_2::simple_effect * e) {

        e->prop->visit(this);

        if(eff_neg) {
            switch(eff_time) {
            case VAL1_2::E_AT_START: msg.at_start_del_effects.push_back(last_prop); break;
            case VAL1_2::E_AT_END: msg.at_end_del_effects.push_back(last_prop); break;
            }
        } else {
            switch(eff_time) {
            case VAL1_2::E_AT_START: msg.at_start_add_effects.push_back(last_prop); break;
            case VAL1_2::E_AT_END: msg.at_end_add_effects.push_back(last_prop); break;
            }
        }
    }

    void VALVisitorOperator::visit_assignment(VAL1_2::assignment * e) {

        rosplan_knowledge_msgs::DomainAssignment ass;
        ass.grounded = false;

        // assignment left hand side
        e->getFTerm()->visit(this);
        ass.LHS = last_func;

        // assignment right hand side
        last_expr.tokens.clear();
        e->getExpr()->visit(this);
        ass.RHS = last_expr;

        // assignment operator
        switch(e->getOp()) {
            case VAL1_2::E_ASSIGN:      ass.assign_type = rosplan_knowledge_msgs::DomainAssignment::ASSIGN;     break;
            case VAL1_2::E_INCREASE:    ass.assign_type = rosplan_knowledge_msgs::DomainAssignment::INCREASE;   break;
            case VAL1_2::E_DECREASE:    ass.assign_type = rosplan_knowledge_msgs::DomainAssignment::DECREASE;   break;
            case VAL1_2::E_SCALE_UP:    ass.assign_type = rosplan_knowledge_msgs::DomainAssignment::SCALE_UP;   break;
            case VAL1_2::E_SCALE_DOWN:  ass.assign_type = rosplan_knowledge_msgs::DomainAssignment::SCALE_DOWN; break;
            case VAL1_2::E_ASSIGN_CTS:  ass.assign_type = rosplan_knowledge_msgs::DomainAssignment::ASSIGN_CTS; break;
        }

        // add assigment effect
        switch(eff_time) {
            case VAL1_2::E_AT_START: msg.at_start_assign_effects.push_back(ass); break;
            case VAL1_2::E_AT_END:   msg.at_end_assign_effects.push_back(ass);   break;
        }
    }

    void VALVisitorOperator::visit_forall_effect(VAL1_2::forall_effect * e) {std::cout << "not implemented forall" << std::endl;};
    void VALVisitorOperator::visit_cond_effect(VAL1_2::cond_effect * e) {std::cout << "not implemented cond" << std::endl;};

    /*-------------*/
    /* expressions */
    /*-------------*/

    void VALVisitorOperator::visit_plus_expression(VAL1_2::plus_expression * s) {

        rosplan_knowledge_msgs::ExprBase op;
        op.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
        op.op = rosplan_knowledge_msgs::ExprBase::ADD;
        last_expr.tokens.push_back(op);

        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
    }

    void VALVisitorOperator::visit_minus_expression(VAL1_2::minus_expression * s) {

        rosplan_knowledge_msgs::ExprBase op;
        op.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
        op.op = rosplan_knowledge_msgs::ExprBase::SUB;
        last_expr.tokens.push_back(op);

        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
    }

    void VALVisitorOperator::visit_mul_expression(VAL1_2::mul_expression * s) {

        rosplan_knowledge_msgs::ExprBase op;
        op.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
        op.op = rosplan_knowledge_msgs::ExprBase::MUL;
        last_expr.tokens.push_back(op);

        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
    }

    void VALVisitorOperator::visit_div_expression(VAL1_2::div_expression * s) {

        rosplan_knowledge_msgs::ExprBase op;
        op.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
        op.op = rosplan_knowledge_msgs::ExprBase::DIV;
        last_expr.tokens.push_back(op);

        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
    }

    void VALVisitorOperator::visit_uminus_expression(VAL1_2::uminus_expression * s) {
        rosplan_knowledge_msgs::ExprBase op;
        op.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
        op.op = rosplan_knowledge_msgs::ExprBase::UMINUS;
        last_expr.tokens.push_back(op);
    }

    void VALVisitorOperator::visit_int_expression(VAL1_2::int_expression * s) {
        rosplan_knowledge_msgs::ExprBase op;
        op.expr_type = rosplan_knowledge_msgs::ExprBase::CONSTANT;
        op.constant = s->double_value();
        last_expr.tokens.push_back(op);
    }

    void VALVisitorOperator::visit_float_expression(VAL1_2::float_expression * s) {
        rosplan_knowledge_msgs::ExprBase op;
        op.expr_type = rosplan_knowledge_msgs::ExprBase::CONSTANT;
        op.constant = s->double_value();
        last_expr.tokens.push_back(op);
    }

    void VALVisitorOperator::visit_special_val_expr(VAL1_2::special_val_expr * s) {
        rosplan_knowledge_msgs::ExprBase op;
        op.expr_type = rosplan_knowledge_msgs::ExprBase::SPECIAL;
        switch(s->getKind()) {
            case VAL1_2::E_HASHT: op.special_type = rosplan_knowledge_msgs::ExprBase::HASHT; break;
            case VAL1_2::E_DURATION_VAR: op.special_type = rosplan_knowledge_msgs::ExprBase::DURATION; break;
            case VAL1_2::E_TOTAL_TIME: op.special_type = rosplan_knowledge_msgs::ExprBase::TOTAL_TIME; break;
            }
        last_expr.tokens.push_back(op);
    }

    void VALVisitorOperator::visit_func_term(VAL1_2::func_term * s) {
        rosplan_knowledge_msgs::ExprBase op;
        op.expr_type = rosplan_knowledge_msgs::ExprBase::FUNCTION;

        // func_term name
        last_func.name = s->getFunction()->getName();

        // func_term variables
        last_func.typed_parameters.clear();
        for (VAL1_2::parameter_symbol_list::const_iterator vi = s->getArgs()->begin(); vi != s->getArgs()->end(); vi++) {
            diagnostic_msgs::KeyValue param;
            param.key = (*vi)->pddl_typed_symbol::getName();
            param.value = (*vi)->type->getName();
            last_func.typed_parameters.push_back(param);
        }

        op.function = last_func;
        last_expr.tokens.push_back(op);
    }

    /*-------*/
    /* extra */
    /*-------*/

    void VALVisitorOperator::visit_derivation_rule(VAL1_2::derivation_rule * o) {}

} // close namespace
