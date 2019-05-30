#include <ros/ros.h>
#include <boost/tokenizer.hpp>

#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "diagnostic_msgs/KeyValue.h"

#ifndef KCL_action_interface
#define KCL_action_interface

/**
 * This file defines the RPActionInterface header.
 * This header defines a standard action interface for ROSPlan.
 * This interface will link a PDDL action to some implementation, most
 * commonly as an actionlib client.
 */
namespace KCL_rosplan {

	class RPActionInterface
	{

	private:
	protected:

		/* PDDL info and publisher */
		std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates;
		rosplan_knowledge_msgs::DomainFormula params;
		rosplan_knowledge_msgs::DomainOperator op;
		ros::Publisher pddl_action_parameters_pub;

		/* action feedback to planning system */
		ros::Publisher action_feedback_pub;

		/* service handle to PDDL knowledge base */
		ros::ServiceClient update_knowledge_client;

		/* action status */
		bool action_success;
		bool action_cancelled;

	public:

		/* main loop for action interface */
		void runActionInterface();

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		
		/* perform or call real action implementation */
		virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) =0;
	};
}
#endif
