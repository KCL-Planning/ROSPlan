#include "rosplan_knowledge_base/VALVisitorProblem.h"
#include <iostream>

/* implementation of rosplan_knowledge_base::VALVisitorPredicate */
namespace KCL_rosplan {

	/*----------------*/
	/* return methods */
	/*----------------*/

	std::map <std::string, std::vector<std::string> > VALVisitorProblem::returnInstances() {
		VAL::const_symbol_list *c = problem->objects;
		for (VAL::const_symbol_list::const_iterator symbolListIterator = c->begin();
			symbolListIterator != c->end(); symbolListIterator++) {
			const VAL::const_symbol *object = *symbolListIterator;
			instances[object->type->getName()].push_back(object->pddl_typed_symbol::getName());
		}
		return instances;
	}

	std::vector<rosplan_knowledge_msgs::KnowledgeItem> VALVisitorProblem::returnFacts(){
		if(!effects_read) {
			visit_effect_lists(problem->initial_state);
			effects_read = true;
		}
		return facts;
	}

	std::vector<rosplan_knowledge_msgs::KnowledgeItem> VALVisitorProblem::returnFunctions(){
		if(!effects_read) {
			visit_effect_lists(problem->initial_state);
			effects_read = true;
		}
		return functions;
	}

	std::vector<rosplan_knowledge_msgs::KnowledgeItem> VALVisitorProblem::returnGoals() {
		problem->the_goal->visit(this);
		return goals;
	}

	rosplan_knowledge_msgs::KnowledgeItem VALVisitorProblem::returnMetric() {
		problem->metric->visit(this);
		return metric;
	}


	/*--------------*/
	/* propositions */
	/*--------------*/

	/**
	 * Visit a proposition to pack into ROS message
	 */
	void VALVisitorProblem::visit_proposition(VAL::proposition *p) {

		last_prop.typed_parameters.clear();
		last_prop.name = p->head->getName();

		std::vector<std::string> predicateLabels;

		// parse domain for predicates
		VAL::pred_decl_list *predicates = domain->predicates;
		for (VAL::pred_decl_list::const_iterator predicateIterator = predicates->begin(); predicateIterator != predicates->end(); predicateIterator++) {

			VAL::pred_decl *tempPredicate = *predicateIterator;

			// compare pedicate name with last proposition name
			if (tempPredicate->getPred()->symbol::getName() == last_prop.name) {

				// iterate the predicate symbols
				for (VAL::var_symbol_list::const_iterator varSymbolIterator = tempPredicate->getArgs()->begin();
						 varSymbolIterator != tempPredicate->getArgs()->end(); varSymbolIterator++) {

					// add labels to predicateLabels
					const VAL::var_symbol *tempVarSymbol = *varSymbolIterator;
					predicateLabels.push_back(tempVarSymbol->pddl_typed_symbol::getName());
				}
			}
		}

		// pack parameters
		int index = 0;
		for (VAL::parameter_symbol_list::const_iterator vi = p->args->begin(); vi != p->args->end(); vi++) {

			const VAL::parameter_symbol* var = *vi;

			diagnostic_msgs::KeyValue param;
			param.key = predicateLabels[index];
			param.value = var->pddl_typed_symbol::getName();
			last_prop.typed_parameters.push_back(param);

			++index;
		}
	}


	/*---------*/
	/* Effects */
	/*---------*/

	void VALVisitorProblem::visit_effect_lists(VAL::effect_lists * e) {

		problem_eff_neg = false;
		e->add_effects.pc_list<VAL::simple_effect*>::visit(this);

		problem_eff_neg = true;
		e->del_effects.pc_list<VAL::simple_effect*>::visit(this);
		problem_eff_neg = false;

		e->forall_effects.pc_list<VAL::forall_effect*>::visit(this);
		e->cond_effects.pc_list<VAL::cond_effect*>::visit(this);
		e->cond_assign_effects.pc_list<VAL::cond_effect*>::visit(this);
		e->assign_effects.pc_list<VAL::assignment*>::visit(this);
		e->timed_effects.pc_list<VAL::timed_effect*>::visit(this);
	}

	void VALVisitorProblem::visit_timed_effect(VAL::timed_effect * e) {
		problem_eff_time = e->ts;
		e->effs->visit(this);
	}

	void VALVisitorProblem::visit_simple_effect(VAL::simple_effect * e) {

		e->prop->visit(this);

		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = 1;
		item.attribute_name = last_prop.name;
		item.is_negative = problem_eff_neg;

		for(unsigned int a = 0; a < last_prop.typed_parameters.size(); a++) {
			diagnostic_msgs::KeyValue param;
			param.key = last_prop.typed_parameters[a].key;
			param.value = last_prop.typed_parameters[a].value;
			item.values.push_back(param);
		}

		facts.push_back(item);
	}

	void VALVisitorProblem::visit_assignment(VAL::assignment *e) {

		e->getFTerm()->visit(this);

		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = 1;
		item.attribute_name = last_func_term.name;
		item.is_negative = false;

		for(unsigned int a = 0; a < last_func_term.typed_parameters.size(); a++) {
			diagnostic_msgs::KeyValue param;
			param.key = last_func_term.typed_parameters[a].key;
			param.value = last_func_term.typed_parameters[a].value;
			item.values.push_back(param);
		}

		expression.str("");
		e->getExpr()->visit(this);
		item.function_value = std::atof(expression.str().c_str());

		functions.push_back(item);
	}

	void VALVisitorProblem::visit_forall_effect(VAL::forall_effect * e) {
		ROS_ERROR("Not yet implemented forall effects in intial state parser.");
	}

	void VALVisitorProblem::visit_cond_effect(VAL::cond_effect * e) {
		ROS_ERROR("Not yet implemented conditional effects in intial state parser.");	
	}

	/*-------*/
	/* Goals */
	/*-------*/

	void VALVisitorProblem::visit_conj_goal(VAL::conj_goal * g){
		g->getGoals()->visit(this);
	}

	void VALVisitorProblem::visit_timed_goal(VAL::timed_goal *c){
		problem_cond_time = c->getTime();
		c->getGoal()->visit(this);
	}

	void VALVisitorProblem::visit_neg_goal(VAL::neg_goal *c) {
		problem_cond_neg = !problem_cond_neg;
		c->getGoal()->visit(this);
		problem_cond_neg = !problem_cond_neg;
	}

	void VALVisitorProblem::visit_simple_goal(VAL::simple_goal* g) {

		g->getProp()->visit(this);

		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = 1;
		item.attribute_name = last_prop.name;
		item.is_negative = problem_cond_neg;

		for(unsigned int a = 0; a < last_prop.typed_parameters.size(); a++) {
			diagnostic_msgs::KeyValue param;
			param.key = last_prop.typed_parameters[a].key;
			param.value = last_prop.typed_parameters[a].value;
			item.values.push_back(param);
		}

		goals.push_back(item);
	}

	void VALVisitorProblem::visit_qfied_goal(VAL::qfied_goal *) {}
	void VALVisitorProblem::visit_disj_goal(VAL::disj_goal *) {}
	void VALVisitorProblem::visit_imply_goal(VAL::imply_goal *) {}
	void VALVisitorProblem::visit_comparison(VAL::comparison *c) {}


	/*---------*/
	/* metrics */
	/*---------*/

	void VALVisitorProblem::visit_metric_spec(VAL::metric_spec * s){

		expression.str("");
		s->expr->visit(this);

		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = 2;
		diagnostic_msgs::KeyValue param;

		if (s->opt == 0){
			param.key = "minimize";
		}
		else {
			param.key = "maximize";
		}

		param.value = expression.str();
		item.values.push_back(param);
		metric = item;
	}


	/*-------------*/
	/* expressions */
	/*-------------*/

	void VALVisitorProblem::visit_plus_expression(VAL::plus_expression * s){
		s->getLHS()->visit(this);
		s->getRHS()->visit(this);
	}
	void VALVisitorProblem::visit_minus_expression(VAL::minus_expression * s){
		s->getLHS()->visit(this);
		s->getRHS()->visit(this);
	}
	void VALVisitorProblem::visit_mul_expression(VAL::mul_expression * s){
		s->getLHS()->visit(this);
		s->getRHS()->visit(this);
	}
	void VALVisitorProblem::visit_div_expression(VAL::div_expression * s){
		s->getLHS()->visit(this);
		s->getRHS()->visit(this);
	}
	void VALVisitorProblem::visit_uminus_expression(VAL::uminus_expression * s){
		s->getExpr()->visit(this);

	}
	void VALVisitorProblem::visit_int_expression(VAL::int_expression * s){
		expression << s->double_value();
	}
	void VALVisitorProblem::visit_float_expression(VAL::float_expression * s){
		expression << s->double_value();
	}
	void VALVisitorProblem::visit_special_val_expr(VAL::special_val_expr * s){
        string tempExpression;
        switch(s->getKind()) {
            case 0:
                tempExpression = "hasht";
                break;
            case 1:
                tempExpression = "duration-var";
                break;
            case 2:
                tempExpression = "total-time";
        }
		expression << tempExpression;
	}

	void VALVisitorProblem::visit_violation_term(VAL::violation_term * v){
		expression << v->getName();
	}

	void VALVisitorProblem::visit_func_term(VAL::func_term * s) {

		last_func_term.typed_parameters.clear();

		// func_term name
		last_func_term.name = s->getFunction()->getName();

		expression << last_func_term.name;


		std::vector<std::string> parameterLabels;

		// parse domain for parameter labels
		for (VAL::func_decl_list::const_iterator fit = domain->functions->begin(); fit != domain->functions->end(); fit++) {
			if ((*fit)->getFunction()->symbol::getName() == last_func_term.name) {
				VAL::var_symbol_list::const_iterator vit = (*fit)->getArgs()->begin();
				for(; vit != (*fit)->getArgs()->end(); vit++) {
					parameterLabels.push_back((*vit)->pddl_typed_symbol::getName());
				}
			}
		}

		// func_term variables
		int index = 0;
		for (VAL::parameter_symbol_list::const_iterator vi = s->getArgs()->begin(); vi != s->getArgs()->end(); vi++) {

			diagnostic_msgs::KeyValue param;
			param.key = parameterLabels[index];
			param.value = (*vi)->getName();
			last_func_term.typed_parameters.push_back(param);
			index++;

			expression << " " << (*vi)->getName();
		}
		expression << ")";
	}


	/*----------------------*/
	/* unused visit methods */
	/*----------------------*/

	void VALVisitorProblem::visit_operator_(VAL::operator_ *) {}
	void VALVisitorProblem::visit_derivation_rule(VAL::derivation_rule *o) {}

} // close namespace
