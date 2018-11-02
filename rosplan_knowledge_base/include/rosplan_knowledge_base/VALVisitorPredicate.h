/**
 * This file describes VALVisitorPredicate class.
 * KMS VALVisitor classes are used to package PDDL things into ROS messages.
 */
#include <sstream>
#include <string>
#include <vector>

#include "VALfiles/ptree.h"
#include "VALfiles/VisitController.h"

#include "rosplan_knowledge_msgs/DomainFormula.h"

#ifndef KCL_valvisitor_predicate
#define KCL_valvisitor_predicate

namespace KCL_rosplan {

	class VALVisitorPredicate : public VAL1_2::VisitController
	{
	private:

	public:

		/* message */
		rosplan_knowledge_msgs::DomainFormula msg;

		/* visitor methods */
		virtual void visit_pred_decl(VAL1_2::pred_decl *);
	};

} // close namespace

#endif
