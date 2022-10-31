/**
 * This file describes VALVisitorProblem class.
 * KMS VALVisitor classes are used to package PDDL things into ROS messages.
 */
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include "VALfiles/ptree.h"
#include "VALfiles/VisitController.h"

#include "rosplan_knowledge_base/KnowledgeComparitor.h"

#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/ExprComposite.h"
#include "rosplan_knowledge_msgs/ExprBase.h"

#ifndef KCL_valvisitor_problem
#define KCL_valvisitor_problem

namespace KCL_rosplan {

	class VALVisitorProblem : public VAL1_2::VisitController
	{
	private:

		/* encoding state */
		bool problem_cond_neg;
		bool problem_eff_neg;
		double problem_eff_time;

		VAL1_2::domain* domain;
		VAL1_2::problem* problem;

		bool effects_read;

	public:

		/* constructor */
		VALVisitorProblem(VAL1_2::domain* inputDomain, VAL1_2::problem* inputProblem){

			domain = inputDomain;
			problem = inputProblem;

			problem_eff_time = 0;

			effects_read = false;
			problem_cond_neg = false;
			problem_eff_neg = false;
		}

		/* messages */
		rosplan_knowledge_msgs::DomainFormula last_prop;
		rosplan_knowledge_msgs::ExprComposite last_expr;
		rosplan_knowledge_msgs::DomainFormula last_func_term;

		std::map<std::string, std::vector<std::string> > instances;
		std::map<std::string, std::vector<std::string> > constants;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> facts;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> functions;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> goals;
		rosplan_knowledge_msgs::KnowledgeItem metric;

		/* timed initial literals */
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> timed_initial_literals;

		/* return methods */
        std::map<std::string, std::vector<std::string> > returnInstances();
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> returnFacts();
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> returnFunctions();
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> returnGoals();
		rosplan_knowledge_msgs::KnowledgeItem returnMetric();

		std::vector<rosplan_knowledge_msgs::KnowledgeItem> returnTimedKnowledge();

		/* visitor methods */
		virtual void visit_proposition(VAL1_2::proposition *);
		virtual void visit_operator_(VAL1_2::operator_ *);

		virtual void visit_simple_goal(VAL1_2::simple_goal *);
		virtual void visit_qfied_goal(VAL1_2::qfied_goal *);
		virtual void visit_conj_goal(VAL1_2::conj_goal *);
		virtual void visit_disj_goal(VAL1_2::disj_goal *);
		virtual void visit_timed_goal(VAL1_2::timed_goal *);
		virtual void visit_imply_goal(VAL1_2::imply_goal *);
		virtual void visit_neg_goal(VAL1_2::neg_goal *);

		virtual void visit_assignment(VAL1_2::assignment * e);
		virtual void visit_simple_effect(VAL1_2::simple_effect * e);
		virtual void visit_forall_effect(VAL1_2::forall_effect * e);
		virtual void visit_cond_effect(VAL1_2::cond_effect * e);
		virtual void visit_timed_effect(VAL1_2::timed_effect * e);
		virtual void visit_timed_initial_literal(VAL1_2::timed_initial_literal * s);
		virtual void visit_effect_lists(VAL1_2::effect_lists * e);

		virtual void visit_comparison(VAL1_2::comparison * c);

		virtual void visit_derivation_rule(VAL1_2::derivation_rule * o);

		virtual void visit_metric_spec(VAL1_2:: metric_spec * s);

        virtual void visit_plus_expression(VAL1_2::plus_expression * s);
        virtual void visit_minus_expression(VAL1_2::minus_expression * s);
        virtual void visit_mul_expression(VAL1_2::mul_expression * s);
        virtual void visit_div_expression(VAL1_2::div_expression * s);
        virtual void visit_uminus_expression(VAL1_2::uminus_expression * s);
        virtual void visit_int_expression(VAL1_2::int_expression * s);
        virtual void visit_float_expression(VAL1_2::float_expression * s);
        virtual void visit_special_val_expr(VAL1_2::special_val_expr * s);
        virtual void visit_func_term(VAL1_2::func_term * s);
    };

} // close namespace

#endif
