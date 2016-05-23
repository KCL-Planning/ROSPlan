#include "PlanDispatcher.h"

#ifndef KCL_simple_dispatcher
#define KCL_simple_dispatcher

namespace KCL_rosplan
{

	class SimplePlanDispatcher: public PlanDispatcher
	{
	private:

		/* check preconditions are true */
		bool checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg);

		/* simple dispatch modes */
		bool dispatch_on_completion;
		bool dispatch_concurrent;

	public:

		/* constructor */
		SimplePlanDispatcher();

		/* access */
		int getCurrentAction();
		void setCurrentAction(size_t freeActionID);
		void reset();

		/* action dispatch methods */
		bool dispatchPlan(const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actionList, double missionStart, double planStart);

		/* action feedback methods */
		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
		void actionFeedback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
	};
}

#endif
