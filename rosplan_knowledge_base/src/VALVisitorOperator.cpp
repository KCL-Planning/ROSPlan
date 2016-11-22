#include "rosplan_knowledge_base/VALVisitorOperator.h"

/* implementation of rosplan_knowledge_base::VALVisitorOperator */
namespace KCL_rosplan {

	/* encoding state */
	bool cond_neg;
	bool eff_neg;
	VAL::time_spec cond_time;
	VAL::time_spec eff_time;
	VAL::comparison_op comparison_op;

	/*-----------*/
	/* operators */
	/*-----------*/

	/**
	 * Visit an operator to pack into ROS message
	 */
	void VALVisitorOperator::visit_operator_(VAL::operator_ * op) {

		msg.formula.name = op->name->symbol::getName();

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
		msg.formula.typed_parameters.clear();

		// parameters
		for (VAL::var_symbol_list::const_iterator vi = op->parameters->begin(); vi != op->parameters->end(); vi++) {
			const VAL::var_symbol* var = *vi;
			diagnostic_msgs::KeyValue param;
			param.key = var->pddl_typed_symbol::getName();
			param.value = var->type->getName();
			msg.formula.typed_parameters.push_back(param);
		}

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
	void VALVisitorOperator::visit_proposition(VAL::proposition *p) {

		last_prop.typed_parameters.clear();

		// predicate name
		rosplan_knowledge_msgs::DomainFormula formula;
		last_prop.name = p->head->symbol::getName();

		// predicate variables
		for (VAL::parameter_symbol_list::const_iterator vi = p->args->begin(); vi != p->args->end(); vi++) {
			const VAL::parameter_symbol* var = *vi;
			diagnostic_msgs::KeyValue param;
			param.key = var->pddl_typed_symbol::getName();
			param.value = var->type->getName();
			last_prop.typed_parameters.push_back(param);
		}

	};

	/*-------*/
	/* goals */
	/*-------*/

	void VALVisitorOperator::visit_conj_goal(VAL::conj_goal *c){
		c->getGoals()->visit(this);
	}

	void VALVisitorOperator::visit_timed_goal(VAL::timed_goal *c){
		cond_time = c->getTime();
        c->getGoal()->visit(this);
	}

	void VALVisitorOperator::visit_neg_goal(VAL::neg_goal *c) {
		cond_neg = !cond_neg;
        c->getGoal()->visit(this);
		cond_neg = !cond_neg;
	}

	void VALVisitorOperator::visit_simple_goal(VAL::simple_goal *c){

		c->getProp()->visit(this);

		if(cond_neg) {
			switch(cond_time) {
			case VAL::E_AT_START: msg.at_start_neg_condition.push_back(last_prop); break;
			case VAL::E_AT_END: msg.at_end_neg_condition.push_back(last_prop); break;
			case VAL::E_OVER_ALL: msg.over_all_neg_condition.push_back(last_prop); break;
			}
		} else {
			switch(cond_time) {
			case VAL::E_AT_START: msg.at_start_simple_condition.push_back(last_prop); break;
			case VAL::E_AT_END: msg.at_end_simple_condition.push_back(last_prop); break;
			case VAL::E_OVER_ALL: msg.over_all_simple_condition.push_back(last_prop); break;
			}
		}
	}

	void VALVisitorOperator::visit_comparison(VAL::comparison * c){}
	void VALVisitorOperator::visit_qfied_goal(VAL::qfied_goal *){}
	void VALVisitorOperator::visit_disj_goal(VAL::disj_goal *){}
	void VALVisitorOperator::visit_imply_goal(VAL::imply_goal *){}

	/*---------*/
	/* effects */
	/*---------*/

	void VALVisitorOperator::visit_effect_lists(VAL::effect_lists * e) {

		eff_neg = false;
		e->add_effects.pc_list<VAL::simple_effect*>::visit(this);
		eff_neg = true;
		e->del_effects.pc_list<VAL::simple_effect*>::visit(this);
		eff_neg = false;

		e->forall_effects.pc_list<VAL::forall_effect*>::visit(this);
		e->cond_effects.pc_list<VAL::cond_effect*>::visit(this);
		e->cond_assign_effects.pc_list<VAL::cond_effect*>::visit(this);
		e->assign_effects.pc_list<VAL::assignment*>::visit(this);
		e->timed_effects.pc_list<VAL::timed_effect*>::visit(this);
	}

	void VALVisitorOperator::visit_timed_effect(VAL::timed_effect * e) {
		eff_time = e->ts;
		e->effs->visit(this);
	};

	void VALVisitorOperator::visit_simple_effect(VAL::simple_effect * e) {

		e->prop->visit(this);

		if(eff_neg) {
			switch(eff_time) {
			case VAL::E_AT_START: msg.at_start_del_effects.push_back(last_prop); break;
			case VAL::E_AT_END: msg.at_end_del_effects.push_back(last_prop); break;
			}
		} else {
			switch(eff_time) {
			case VAL::E_AT_START: msg.at_start_add_effects.push_back(last_prop); break;
			case VAL::E_AT_END: msg.at_end_add_effects.push_back(last_prop); break;
			}
		}
	}

	void VALVisitorOperator::visit_assignment(VAL::assignment * e) {
		// e->getFTerm()
	}

	void VALVisitorOperator::visit_forall_effect(VAL::forall_effect * e) {std::cout << "not implemented forall" << std::endl;};
	void VALVisitorOperator::visit_cond_effect(VAL::cond_effect * e) {std::cout << "not implemented cond" << std::endl;};

	/*-------*/
	/* extra */
	/*-------*/

	void VALVisitorOperator::visit_derivation_rule(VAL::derivation_rule * o){}

} // close namespace
