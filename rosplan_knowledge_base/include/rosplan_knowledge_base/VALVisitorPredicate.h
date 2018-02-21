/**
 * This file describes VALVisitorPredicate class.
 * KMS VALVisitor classes are used to package PDDL things into ROS messages.
 */
#include <sstream>
#include <string>
#include <vector>

#include "ptree.h"
#include "VisitController.h"

#include "rosplan_knowledge_msgs/DomainFormula.h"

#ifndef KCL_valvisitor_predicate
#define KCL_valvisitor_predicate

namespace KCL_rosplan {

	class VALVisitorPredicate : public VAL::VisitController
	{
	private:

	public:

		/* message */
		rosplan_knowledge_msgs::DomainFormula msg;

		/* visitor methods */
		virtual void visit_pred_decl(VAL::pred_decl *);
	};

} // close namespace

#endif
