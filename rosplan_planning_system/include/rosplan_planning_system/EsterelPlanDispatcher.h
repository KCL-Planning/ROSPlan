#include "PlanDispatcher.h"

#ifndef KCL_esterel_dispatcher
#define KCL_esterel_dispatcher

namespace KCL_rosplan
{
	struct StrlEdge
	{
		int signal_type;
		std::string edge_name;
		bool active;
	};

	struct StrlNode
	{
		std::string node_name;
		std::vector<std::string> input;
		std::vector<std::string> output;

		std::vector<bool> await_input;
		bool dispatched;
		bool completed;
	};

	class EsterelPlanDispatcher: public PlanDispatcher
	{
	private:

		/* Esterel filename */
		std::string strl_file;
		bool readEsterelFile(std::string strlFile);

		/* plan description in Esterel */
		std::map<std::string,StrlNode> plan_description;
		std::map<std::string,StrlEdge> plan_edges;

	public:

		/* constructor */
		EsterelPlanDispatcher();

		/* access */
		int getCurrentAction();
		void reset();

		/* action dispatch methods */
		bool dispatchPlan(const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actionList, double missionStart, double planStart);

		/* action feedback methods */
		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
		void actionFeedback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
	};
}

#endif
