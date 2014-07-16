/**
 * This file contains the parsing of the ActionFeedback topic.
 * The feedback from actions can be verbose, and it is here that the data is parsed.
 * 
 * However: the discovery of new information, and consequent reasoning, should be
 * passed through knowledge messages (see the Plan Filter.)  In this way reasoning does
 * not need to wait for actions to be terminated, but can continue in parallel and
 * possibly interrupt action execution.
 *
 * ROS calls feedbackCallback() in its own thread.
 * Mesages are passed onto actionFeedback based on action type.
 */
namespace KCL_rosplan {

	/*---------------------------*/
	/* Specific action responses */
	/*---------------------------*/

	/**
	 * processes single action feedback message.
	 * This method serves as the hook for defining more interesting behaviour on action feedback.
	 */
	void actionFeedback(const planning_dispatch_msgs::ActionFeedback::ConstPtr& msg) {
	
	}

	/*------------------*/
	/* general feedback */
	/*------------------*/

	/**
	 * listen to and process actionFeedback topic.
	 */
	void feedbackCallback(const planning_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		ROS_INFO("KCL: Feedback received [%i,%s]", msg->action_id, msg->status.c_str());

		// create error if the action is unrecognised
		if(KCL_rosplan::currentAction != (unsigned int)msg->action_id)
			ROS_INFO("KCL: Unexpected action ID: %d; current action: %zu", msg->action_id, KCL_rosplan::currentAction);

		// action enabled
		if(!KCL_rosplan::actionReceived[msg->action_id] && (0 == msg->status.compare("action enabled")))
			KCL_rosplan::actionReceived[msg->action_id] = true;
		
		// action completed (successfuly)
		if(!KCL_rosplan::actionCompleted[msg->action_id] && 0 == msg->status.compare("action achieved"))
			KCL_rosplan::actionCompleted[msg->action_id] = true;

		// more specific feedback
		actionFeedback(msg);
	}

} // close namespace
