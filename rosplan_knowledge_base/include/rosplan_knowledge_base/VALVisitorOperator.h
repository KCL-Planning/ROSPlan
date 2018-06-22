/**
 * This file describes VALVisitorOperator class.
 * KMS VALVisitor classes are used to package PDDL things into ROS messages.
 */
#include <sstream>
#include <string>
#include <vector>

#include "ptree.h"
#include "VisitController.h"

#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include "rosplan_knowledge_msgs/ExprBase.h"
#include "rosplan_knowledge_msgs/ExprComposite.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#ifndef KCL_valvisitor_operator
#define KCL_valvisitor_operator

namespace KCL_rosplan {

	class VALVisitorOperator : public VAL::VisitController
	{
	private:

	public:

		/* message */
		rosplan_knowledge_msgs::ExprComposite last_expr;
		rosplan_knowledge_msgs::DomainFormula last_prop;
		rosplan_knowledge_msgs::DomainFormula last_func;
		rosplan_knowledge_msgs::DomainOperator msg;

		/* visitor methods */
		virtual void visit_proposition(VAL::proposition *);
		virtual void visit_operator_(VAL::operator_ *);

		virtual void visit_simple_goal(VAL::simple_goal *);
		virtual void visit_qfied_goal(VAL::qfied_goal *);
		virtual void visit_conj_goal(VAL::conj_goal *);
		virtual void visit_disj_goal(VAL::disj_goal *);
		virtual void visit_timed_goal(VAL::timed_goal *);
		virtual void visit_imply_goal(VAL::imply_goal *);
		virtual void visit_neg_goal(VAL::neg_goal *);

		virtual void visit_assignment(VAL::assignment * e);
		virtual void visit_simple_effect(VAL::simple_effect * e);
		virtual void visit_forall_effect(VAL::forall_effect * e);
		virtual void visit_cond_effect(VAL::cond_effect * e);
		virtual void visit_timed_effect(VAL::timed_effect * e);
		virtual void visit_effect_lists(VAL::effect_lists * e);

		virtual void visit_comparison(VAL::comparison * c);

        virtual void visit_plus_expression(VAL::plus_expression * s);
        virtual void visit_minus_expression(VAL::minus_expression * s);
        virtual void visit_mul_expression(VAL::mul_expression * s);
        virtual void visit_div_expression(VAL::div_expression * s);
        virtual void visit_uminus_expression(VAL::uminus_expression * s);
        virtual void visit_int_expression(VAL::int_expression * s);
        virtual void visit_float_expression(VAL::float_expression * s);
        virtual void visit_special_val_expr(VAL::special_val_expr * s);
        virtual void visit_func_term(VAL::func_term * s);

		virtual void visit_derivation_rule(VAL::derivation_rule * o);
	};

} // close namespace

#endif
