#include "PlanDispatcher.h"
#include "EsterelPlan.h"
#include "CFFPlanParser.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#ifndef KCL_esterel_dispatcher
#define KCL_esterel_dispatcher

namespace KCL_rosplan
{
	class EsterelPlanDispatcher: public PlanDispatcher
	{
	private:

		bool printPlan();

		/* mapping PDDL conditions and esterel inputs */
		ros::ServiceClient query_knowledge_client;
		std::map<std::string,rosplan_knowledge_msgs::KnowledgeItem> condition_mapping;
		void preparePDDLCondition(std::string edgeName);

		/* plan description from parser */
		CFFPlanParser *cff_pp;

		/* plan description from Esterel file */
		std::string strl_file;
		bool readEsterelFile(std::string strlFile);

	public:

		/* constructor */
		EsterelPlanDispatcher(CFFPlanParser &parser);

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
