#include "rosplan_knowledge_base/VALVisitorProblem.h"
#include <iostream>

/* implementation of rosplan_knowledge_base::VALVisitorPredicate */
namespace KCL_rosplan {

	/*----------------*/
	/* return methods */
	/*----------------*/

	std::map <std::string, std::vector<std::string> > VALVisitorProblem::returnInstances() {
		VAL1_2::const_symbol_list *c = problem->objects;
		if (c)
		{
			for (VAL1_2::const_symbol_list::const_iterator symbolListIterator = c->begin();
				symbolListIterator != c->end(); symbolListIterator++) {
				const VAL1_2::const_symbol *object = *symbolListIterator;
				instances[object->type->getName()].push_back(object->pddl_typed_symbol::getName());
			}
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

	std::vector<rosplan_knowledge_msgs::KnowledgeItem> VALVisitorProblem::returnTimedKnowledge() {
		if(!effects_read) {
			visit_effect_lists(problem->initial_state);
			effects_read = true;
		}
		return timed_initial_literals;
	}

	/*--------------*/
	/* propositions */
	/*--------------*/

	/**
	 * Visit a proposition to pack into ROS message
	 */
	void VALVisitorProblem::visit_proposition(VAL1_2::proposition *p) {

		last_prop.typed_parameters.clear();
		last_prop.name = p->head->getName();

		std::vector<std::string> predicateLabels;

		// parse domain for predicates
		VAL1_2::pred_decl_list *predicates = domain->predicates;
		for (VAL1_2::pred_decl_list::const_iterator predicateIterator = predicates->begin(); predicateIterator != predicates->end(); predicateIterator++) {

			VAL1_2::pred_decl *tempPredicate = *predicateIterator;

			// compare pedicate name with last proposition name
			if (tempPredicate->getPred()->symbol::getName() == last_prop.name) {

				// iterate the predicate symbols
				for (VAL1_2::var_symbol_list::const_iterator varSymbolIterator = tempPredicate->getArgs()->begin();
						 varSymbolIterator != tempPredicate->getArgs()->end(); varSymbolIterator++) {

					// add labels to predicateLabels
					const VAL1_2::var_symbol *tempVarSymbol = *varSymbolIterator;
					predicateLabels.push_back(tempVarSymbol->pddl_typed_symbol::getName());
				}
			}
		}

		// pack parameters
		int index = 0;
		for (VAL1_2::parameter_symbol_list::const_iterator vi = p->args->begin(); vi != p->args->end(); vi++) {

			const VAL1_2::parameter_symbol* var = *vi;

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

	void VALVisitorProblem::visit_effect_lists(VAL1_2::effect_lists * e) {

		problem_eff_neg = false;
		e->add_effects.pc_list<VAL1_2::simple_effect*>::visit(this);

		problem_eff_neg = true;
		e->del_effects.pc_list<VAL1_2::simple_effect*>::visit(this);
		problem_eff_neg = false;

		e->forall_effects.pc_list<VAL1_2::forall_effect*>::visit(this);
		e->cond_effects.pc_list<VAL1_2::cond_effect*>::visit(this);
		e->cond_assign_effects.pc_list<VAL1_2::cond_effect*>::visit(this);
		e->assign_effects.pc_list<VAL1_2::assignment*>::visit(this);
		e->timed_effects.pc_list<VAL1_2::timed_effect*>::visit(this);
	}

	void VALVisitorProblem::visit_timed_initial_literal(VAL1_2::timed_initial_literal * s) {
		problem_eff_time = s->time_stamp;
		s->effs->visit(this);
		problem_eff_time = 0;
	}

	void VALVisitorProblem::visit_simple_effect(VAL1_2::simple_effect * e) {

		e->prop->visit(this);

		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		item.attribute_name = last_prop.name;
		item.is_negative = problem_eff_neg;

		for(unsigned int a = 0; a < last_prop.typed_parameters.size(); a++) {
			diagnostic_msgs::KeyValue param;
			param.key = last_prop.typed_parameters[a].key;
			param.value = last_prop.typed_parameters[a].value;
			item.values.push_back(param);
		}

		if(problem_eff_time > 0) {
			item.initial_time = ros::Time::now() + ros::Duration(problem_eff_time);
			timed_initial_literals.push_back(item);
		} else {
			item.initial_time = ros::Time::now();
			facts.push_back(item);
		}
	}

	void VALVisitorProblem::visit_assignment(VAL1_2::assignment *e) {

		e->getFTerm()->visit(this);

		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
		item.attribute_name = last_func_term.name;
		item.is_negative = false;

		for(unsigned int a = 0; a < last_func_term.typed_parameters.size(); a++) {
			diagnostic_msgs::KeyValue param;
			param.key = last_func_term.typed_parameters[a].key;
			param.value = last_func_term.typed_parameters[a].value;
			item.values.push_back(param);
		}

		last_expr.tokens.clear();
		e->getExpr()->visit(this);
		item.function_value = KnowledgeComparitor::evaluateExpression(last_expr, functions);

		if(problem_eff_time > 0) {
			item.initial_time = ros::Time::now() + ros::Duration(problem_eff_time);
			timed_initial_literals.push_back(item);
		} else {
			item.initial_time = ros::Time::now();
			functions.push_back(item);
		}
	}

	void VALVisitorProblem::visit_forall_effect(VAL1_2::forall_effect * e) {
		ROS_ERROR("KCL: (%s) Not yet implemented forall effects in intial state parser.", ros::this_node::getName().c_str());
	}

	void VALVisitorProblem::visit_cond_effect(VAL1_2::cond_effect * e) {
		ROS_ERROR("KCL: (%s) Not yet implemented conditional effects in intial state parser.", ros::this_node::getName().c_str());
	}

	void VALVisitorProblem::visit_timed_effect(VAL1_2::timed_effect * e) {
		ROS_WARN("KCL: (%s) Timed effects not a part of PDDL problem parsing.", ros::this_node::getName().c_str());
	}

	/*-------*/
	/* Goals */
	/*-------*/

	void VALVisitorProblem::visit_conj_goal(VAL1_2::conj_goal * g){
		g->getGoals()->visit(this);
	}

	void VALVisitorProblem::visit_neg_goal(VAL1_2::neg_goal *g) {
		problem_cond_neg = !problem_cond_neg;
		g->getGoal()->visit(this);
		problem_cond_neg = !problem_cond_neg;
	}

	void VALVisitorProblem::visit_simple_goal(VAL1_2::simple_goal* g) {

		g->getProp()->visit(this);

		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
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

    void VALVisitorProblem::visit_qfied_goal(VAL1_2::qfied_goal *g) {
        //g->getQuantifier()->visit(this);
        g->getVars()->visit(this);
        //g->getSymTab()->visit(this);
        g->getGoal()->visit(this);

    }
    void VALVisitorProblem::visit_disj_goal(VAL1_2::disj_goal *g) {
        g->getGoals()->visit(this);
    }
    void VALVisitorProblem::visit_imply_goal(VAL1_2::imply_goal *g) {
        g->getAntecedent()->visit(this);
        g->getConsequent()->visit(this);
    }
    void VALVisitorProblem::visit_comparison(VAL1_2::comparison *c) {

        rosplan_knowledge_msgs::KnowledgeItem item;
        item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INEQUALITY;
        item.is_negative = problem_cond_neg;

        rosplan_knowledge_msgs::DomainInequality ineq;
        
        switch( c->getOp() )
        {
            case VAL1_2::E_GREATER: ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::GREATER; break;
            case VAL1_2::E_GREATEQ: ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::GREATEREQ; break;
            case VAL1_2::E_LESS: ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::LESS; break;
            case VAL1_2::E_LESSEQ: ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::LESSEQ; break;
            case VAL1_2::E_EQUALS: ineq.comparison_type = rosplan_knowledge_msgs::DomainInequality::EQUALS; break;
        }

		last_expr.tokens.clear();
        c->getLHS()->visit(this);
		ineq.LHS.tokens = last_expr.tokens;

        last_expr.tokens.clear();
        c->getRHS()->visit(this);
        ineq.RHS.tokens = last_expr.tokens;

        item.ineq = ineq;
        goals.push_back(item);

    }

	void VALVisitorProblem::visit_timed_goal(VAL1_2::timed_goal *c){
		ROS_WARN("KCL: (%s) Timed goal not a part of PDDL problem parsing.", ros::this_node::getName().c_str());	
	}

	/*---------*/
	/* metrics */
	/*---------*/

	void VALVisitorProblem::visit_metric_spec(VAL1_2::metric_spec * s){

		metric.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::EXPRESSION;

		// parse expression
		last_expr.tokens.clear();
		s->expr->visit(this);
		metric.expr = last_expr;

		// parse optimization
		switch (s->opt) {
			case VAL1_2::E_MINIMIZE: metric.optimization = "minimize"; break;
			case VAL1_2::E_MAXIMIZE: metric.optimization = "maximize"; break;
		}
	}


	/*-------------*/
	/* expressions */
	/*-------------*/

	void VALVisitorProblem::visit_plus_expression(VAL1_2::plus_expression * s){

		rosplan_knowledge_msgs::ExprBase base;
		base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
		base.op = rosplan_knowledge_msgs::ExprBase::ADD;
		last_expr.tokens.push_back(base);

		s->getLHS()->visit(this);
		s->getRHS()->visit(this);
	}

	void VALVisitorProblem::visit_minus_expression(VAL1_2::minus_expression * s){

		rosplan_knowledge_msgs::ExprBase base;
		base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
		base.op = rosplan_knowledge_msgs::ExprBase::SUB;
		last_expr.tokens.push_back(base);

		s->getLHS()->visit(this);
		s->getRHS()->visit(this);
	}

	void VALVisitorProblem::visit_mul_expression(VAL1_2::mul_expression * s){

		rosplan_knowledge_msgs::ExprBase base;
		base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
		base.op = rosplan_knowledge_msgs::ExprBase::MUL;
		last_expr.tokens.push_back(base);

		s->getLHS()->visit(this);
		s->getRHS()->visit(this);
	}

	void VALVisitorProblem::visit_div_expression(VAL1_2::div_expression * s){

		rosplan_knowledge_msgs::ExprBase base;
		base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
		base.op = rosplan_knowledge_msgs::ExprBase::DIV;
		last_expr.tokens.push_back(base);

		s->getLHS()->visit(this);
		s->getRHS()->visit(this);
	}

	void VALVisitorProblem::visit_uminus_expression(VAL1_2::uminus_expression * s){

		rosplan_knowledge_msgs::ExprBase base;
		base.expr_type = rosplan_knowledge_msgs::ExprBase::OPERATOR;
		base.op = rosplan_knowledge_msgs::ExprBase::SUB;
		last_expr.tokens.push_back(base);

		base.expr_type = rosplan_knowledge_msgs::ExprBase::CONSTANT;
		base.constant = 0;
		last_expr.tokens.push_back(base);

		s->getExpr()->visit(this);
	}

	void VALVisitorProblem::visit_int_expression(VAL1_2::int_expression * s){
		rosplan_knowledge_msgs::ExprBase base;
		base.expr_type = rosplan_knowledge_msgs::ExprBase::CONSTANT;
		base.constant = s->double_value();
		last_expr.tokens.push_back(base);
	}

	void VALVisitorProblem::visit_float_expression(VAL1_2::float_expression * s){
		rosplan_knowledge_msgs::ExprBase base;
		base.expr_type = rosplan_knowledge_msgs::ExprBase::CONSTANT;
		base.constant = s->double_value();
		last_expr.tokens.push_back(base);
	}

	void VALVisitorProblem::visit_special_val_expr(VAL1_2::special_val_expr * s){

		rosplan_knowledge_msgs::ExprBase base;
		base.expr_type = rosplan_knowledge_msgs::ExprBase::SPECIAL;

		switch(s->getKind()) {
			case VAL1_2::E_HASHT:			base.special_type = rosplan_knowledge_msgs::ExprBase::HASHT; break;
			case VAL1_2::E_DURATION_VAR:	base.special_type = rosplan_knowledge_msgs::ExprBase::DURATION; break;
			case VAL1_2::E_TOTAL_TIME:		base.special_type = rosplan_knowledge_msgs::ExprBase::TOTAL_TIME; break;
		}

		last_expr.tokens.push_back(base);
	}

	void VALVisitorProblem::visit_func_term(VAL1_2::func_term * s) {

		rosplan_knowledge_msgs::ExprBase base;
		base.expr_type = rosplan_knowledge_msgs::ExprBase::FUNCTION;
		last_func_term.typed_parameters.clear();

		// func_term name
		last_func_term.name = s->getFunction()->getName();

		// parse domain for parameter labels
		std::vector<std::string> parameterLabels;
		for (VAL1_2::func_decl_list::const_iterator fit = domain->functions->begin(); fit != domain->functions->end(); fit++) {
			if ((*fit)->getFunction()->symbol::getName() == last_func_term.name) {
				VAL1_2::var_symbol_list::const_iterator vit = (*fit)->getArgs()->begin();
				for(; vit != (*fit)->getArgs()->end(); vit++) {
					parameterLabels.push_back((*vit)->pddl_typed_symbol::getName());
				}
			}
		}

		// func_term variables
		VAL1_2::parameter_symbol_list::const_iterator vi = s->getArgs()->begin();
		int index = 0;
		for (; vi != s->getArgs()->end() && index < parameterLabels.size(); vi++) {
			diagnostic_msgs::KeyValue param;
			param.key = parameterLabels[index];
			param.value = (*vi)->getName();
			last_func_term.typed_parameters.push_back(param);
			index++;
		}

		base.function = last_func_term;
		last_expr.tokens.push_back(base);
	}


	/*----------------------*/
	/* unused visit methods */
	/*----------------------*/

	void VALVisitorProblem::visit_operator_(VAL1_2::operator_ *) {}
	void VALVisitorProblem::visit_derivation_rule(VAL1_2::derivation_rule *o) {}

} // close namespace
