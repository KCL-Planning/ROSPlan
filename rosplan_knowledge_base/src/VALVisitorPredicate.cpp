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
		std::cout << "[VALVisitorPredicate::visit_pred_decl]" << std::endl;
		if (p == NULL)
		{
			std::cout << "[VALVisitorPredicate::visit_pred_decl] p is NULL!" << std::endl;
			if (p->getPred() == NULL)
			{
				std::cout << "No predicate found!" << std::endl;
			}
		}
		
		msg.typed_parameters.clear();

		// predicate name
		msg.name = p->getPred()->symbol::getName();

		std::cout << "[VALVisitorPredicate::visit_pred_decl] " << msg.name << std::endl;
		
		// predicate variables
		for (VAL::var_symbol_list::const_iterator vi = p->getArgs()->begin(); vi != p->getArgs()->end(); vi++) {
			const VAL::var_symbol* var = *vi;
			
			std::cout << "[VALVisitorPredicate::visit_pred_decl] Get symbols." << std::endl;
			if (var == NULL)
			{
				std::cout << "Var is NULL!" << std::endl;
				if (var->type == NULL)
					std::cout << "Var: " << var->pddl_typed_symbol::getName() << " has no type!" << std::endl;
			}
			
			diagnostic_msgs::KeyValue param;
			param.key = var->pddl_typed_symbol::getName();
			param.value = var->type->getName();
			msg.typed_parameters.push_back(param);
		}

	};

} // close namespace
