#include "rosplan_knowledge_base/VALVisitorPredicate.h"

/* implementation of rosplan_knowledge_base::VALVisitorPredicate */
namespace KCL_rosplan {

	/*--------------*/
	/* propositions */
	/*--------------*/

	/**
	 * Visit an prop to pack into ROS message
	 */
	void VALVisitorPredicate::visit_pred_decl(VAL::pred_decl *p) {

		msg.typed_parameters.clear();

		// predicate name
		msg.name = p->getPred()->symbol::getName();

		// predicate variables
		for (VAL::var_symbol_list::const_iterator vi = p->getArgs()->begin(); vi != p->getArgs()->end(); vi++) {
			const VAL::var_symbol* var = *vi;
			diagnostic_msgs::KeyValue param;
			param.key = var->pddl_typed_symbol::getName();
			param.value = var->type->getName();
			msg.typed_parameters.push_back(param);
		}

	};

} // close namespace
