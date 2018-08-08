---
title: Tutorial 10 Action Interface
layout: documentation
permalink: /tutorials/tutorial_10
---

## 1. Description

This tutorial will cover how to link ROSPlan with lower level control with an Action Interface. The action interface will subscribe to the action dispatch topic and listen for PDDL action messages dispatched by a Plan Dispatch node. There are two methods to implement this interface

1. Extending the **Action Interface** node
2. Implementing an interface from scratch. 

The action interface class provides a template for servicing a PDDL action. This class can be extended, and a *concreteCallback* method implemented with the code that services the action. The superclass performs the following process:

```
IF action name matches THEN:
  check action for malformed parameters
  update knowledge base (at start effects)
  publish (action enabled)
  success = concreteCallback()
  IF success THEN:
      update knowledge base (at end effects)
      publish (action achieved)
  ELSE
      publish (action failed)
  RETURN success
```

### When you should extend Action Interface to service the action:

If you want to follow the above algorithm exactly for a single PDDL action, this is the most convenient way. The superclass will handle:

- Subscription and publication to the action dispatch topics.
- Fetching operator details for checking parameters, conditions, and effects.
- Updating the knowledge base with the effects of the action.

### When you should start from scratch:

If you want anything different. For example:

- If you don't want the action interface to update the knowledge base. For example, a robot's current position is updated by passive sensing/odometry, not automatically as the effect of a movement action.
- If you care less about the action parameters, or action name.

## 2. Prior Setup

This tutorial assumes that you have have already followed [Tutorial 04: Simulated Actions](tutorial_04), and will use the same launch files and scripts. Copy the launch file you created in Tutorial 04, calling the copy *tutorial_10.launch* and remove the following lines:

```
	<include file="$(find rosplan_planning_system)/launch/includes/simulated_action.launch" >
		<arg name="pddl_action_name" value="undock" />
	</include>
```

```
	<include file="$(find rosplan_planning_system)/launch/includes/simulated_action.launch" >
		<arg name="pddl_action_name" value="localise" />
	</include>
```

This has removed the simulated execution of the `undock` and `localise` actions. In this tutorial you will implement action interfaces for these two PDDL actions.

Follow the ROS tutorials for creating and building a new package: [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials). In this tutorial you will implement some new nodes that will be put into this package.

## 3. Extending Action Interface


```
#include <ros/ros.h>

#include "rosplan_action_interface/RPActionInterface.h"

#ifndef KCL_tutorial_10
#define KCL_tutorial_10

/**
 * This file defines the action interface written in Tutorial 10.
 */
namespace KCL_rosplan {

	class RPTutorialInterface: public RPActionInterface
	{

	private:

		ros::NodeHandle node_handle;

	public:

		/* constructor */
		RPTutorialInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
```

### pt 1
- implementing a new action interface that pubs/subs to action topics
- code copy
- subscriptions to dispatch
- printing the message
- publication of action achieved

### pt 2
- description of the action interface
-- parameter checking
-- knowledge updates (pre/post)
-- design pattern (abstract pattern with concrete callback)

- code copy


### discussion
- should I extend the action interface?
