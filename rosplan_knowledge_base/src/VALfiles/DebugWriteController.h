/************************************************************************
 * Copyright 2008, Strathclyde Planning Group,
 * Department of Computer and Information Sciences,
 * University of Strathclyde, Glasgow, UK
 * http://planning.cis.strath.ac.uk/
 *
 * Maria Fox, Richard Howey and Derek Long - VAL
 * Stephen Cresswell - PDDL Parser
 *
 * This file is part of VAL, the PDDL validator.
 *
 * VAL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * VAL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with VAL.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/

#ifndef __DWRITECONTROLLER_1_2
#define __DWRITECONTROLLER_1_2

#include "WriteController.h"

namespace VAL1_2 {

class DebugWriteController : public WriteController {
protected: 
	int indent;
public:
	DebugWriteController() : indent(0) {};
	virtual ~DebugWriteController() {};

	virtual void write_symbol(ostream & o,const symbol *);
	virtual void write_const_symbol(ostream & o,const const_symbol *);
	virtual void write_var_symbol(ostream & o,const var_symbol *);
	virtual void write_pddl_typed_symbol(ostream & o,const pddl_typed_symbol *);
	virtual void write_plus_expression(ostream & o,const plus_expression *);
	virtual void write_minus_expression(ostream & o,const minus_expression *);
	virtual void write_mul_expression(ostream & o,const mul_expression *);
	virtual void write_div_expression(ostream & o,const div_expression *);
	virtual void write_uminus_expression(ostream & o,const uminus_expression *);
	virtual void write_int_expression(ostream & o,const int_expression *);
	virtual void write_float_expression(ostream & o,const float_expression *);
	virtual void write_special_val_expr(ostream & o,const special_val_expr *);
	virtual void write_func_term(ostream & o,const func_term *);
	virtual void write_assignment(ostream & o,const assignment *);
	virtual void write_goal_list(ostream & o,const goal_list *);
	virtual void write_simple_goal(ostream & o,const simple_goal *);
	virtual void write_qfied_goal(ostream & o,const qfied_goal *);
	virtual void write_conj_goal(ostream & o,const conj_goal *);
	virtual void write_disj_goal(ostream & o,const disj_goal *);
	virtual void write_timed_goal(ostream & o,const timed_goal *);
	virtual void write_imply_goal(ostream & o,const imply_goal *);
	virtual void write_neg_goal(ostream & o,const neg_goal *);
	virtual void write_comparison(ostream & o,const comparison *);
	virtual void write_proposition(ostream & o,const proposition *);
	virtual void write_pred_decl_list(ostream & o,const pred_decl_list *);
	virtual void write_func_decl_list(ostream & o,const func_decl_list *);
	virtual void write_pred_decl(ostream & o,const pred_decl *);
	virtual void write_func_decl(ostream & o,const func_decl *);
	virtual void write_simple_effect(ostream & o,const simple_effect *);
	virtual void write_forall_effect(ostream & o,const forall_effect *);
	virtual void write_cond_effect(ostream & o,const cond_effect *);
	virtual void write_timed_effect(ostream & o,const timed_effect *);
	virtual void write_timed_initial_literal(ostream & o,const timed_initial_literal *);
	virtual void write_effect_lists(ostream & o,const effect_lists *);
	virtual void write_operator_list(ostream & o,const operator_list *);
	virtual void write_derivations_list(ostream & o,const derivations_list * d);
	virtual void write_derivation_rule(ostream & o,const derivation_rule * d);
	virtual void write_operator_(ostream & o,const operator_ *);
	virtual void write_action(ostream & o,const action *);
	virtual void write_event(ostream & o,const event *);
	virtual void write_process(ostream & o,const process *);
	virtual void write_durative_action(ostream & o,const durative_action *);
	virtual void write_domain(ostream & o,const domain *);
	virtual void write_metric_spec(ostream & o,const metric_spec *);
	virtual void write_length_spec(ostream & o,const length_spec *);
	virtual void write_problem(ostream & o,const problem *);
	virtual void write_plan_step(ostream & o,const plan_step *);
	virtual void write_preference(ostream & o,const preference *);
	virtual void write_violation_term(ostream & o,const violation_term *);
	virtual void write_constraint_goal(ostream & o,const constraint_goal *);
};

};


#endif
