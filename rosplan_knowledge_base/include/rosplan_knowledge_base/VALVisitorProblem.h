/**
 * This file describes VALVisitorProblem class.
 * KMS VALVisitor classes are used to package PDDL things into ROS messages.
 */
#include <sstream>
#include <string>
#include <vector>

#include "ptree.h"
#include "VisitController.h"

#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#ifndef KCL_valvisitor_problem
#define KCL_valvisitor_problem

namespace KCL_rosplan {

	class VALVisitorProblem : public VAL::VisitController
	{
	private:

        VAL::domain* domain;
        VAL::problem* problem;

	public:

        VALVisitorProblem(VAL::domain* inputDomain, VAL::problem* inputProblem){
            domain = inputDomain;
            problem = inputProblem;
        }

		/* message */
        rosplan_knowledge_msgs::DomainFormula last_prop;

        std::map<std::string, std::vector<std::string> > instances;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> facts;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> goals;



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

        std::map<std::string, std::vector<std::string> > returnInstances();
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> returnFacts();
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> returnGoals();

		virtual void visit_assignment(VAL::assignment * e);
		virtual void visit_simple_effect(VAL::simple_effect * e);
		virtual void visit_forall_effect(VAL::forall_effect * e);
		virtual void visit_cond_effect(VAL::cond_effect * e);
		virtual void visit_timed_effect(VAL::timed_effect * e);
		virtual void visit_effect_lists(VAL::effect_lists * e);

		virtual void visit_comparison(VAL::comparison * c);

		virtual void visit_derivation_rule(VAL::derivation_rule * o);


	};

} // close namespace

#endif
