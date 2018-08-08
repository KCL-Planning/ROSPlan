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

## 3. Extending Action Interface

For this tutorial, we will create a new action interface directly in the ROSPlan package. This means creating two new files, a header file and source file, and editing CMakeLists.txt. Normally it is better to create new code in a separate package, but for the purposes of this tutorial it will keep things simpler.

### 3.1 Header File

Create a new file in the directory: *rosplan/rosplan_planning_system/include/rosplan_action_interface/* and call it *RPTutorial10.h*. Copy in the following code:

```
#include <ros/ros.h>
#include <vector>

#include "rosplan_action_interface/RPActionInterface.h"

#ifndef KCL_tutorial_10
#define KCL_tutorial_10

/**
 * This file defines an action interface created in tutorial 10.
 */
namespace KCL_rosplan {

	class RPTutorialInterface: public RPActionInterface
	{

	private:

	public:

		/* constructor */
		RPTutorialInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
```

This header file defines a new class, called *RPTutorialInterface*, which extends the *RPActionInterface* class. You can take a look at the header for this class in the same directory. Note that there is one virtual method, which will be implemented by *RPTutorialInterface*.

```
/* perform or call real action implementation */
virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) =0;
```

### 3.2 Source File

Create a new file in the directory: *rosplan/rosplan_planning_system/src/ActionInterface/* and call it *RPTutorial10.cpp*. Copy in the following code:

```
#include "rosplan_action_interface/RPTutorial.h"

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

	/* constructor */
	RPTutorialInterface::RPTutorialInterface(ros::NodeHandle &nh) {
		// perform setup
	}

	/* action dispatch callback */
	bool RPTutorialInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.

		// complete the action
		ROS_INFO("KCL: (%s) TUTORIAL Action completing.", msg->name.c_str());
		return true;
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_tutorial_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPTutorialInterface rpti(nh);

		rpti.runActionInterface();

		return 0;
	}
```

This file contains for the header file created above. The important method is the *concreteCallback* method, which implements the action. Right now this method simply returns **true** -- indicating that the action has succeeded.

### 3.3 CmakeLists.txt

Finally, in order to compile the node, it must be added to the CMakeLists. Open the file *CmakeLists.txt* in the directory *rosplan/rosplan_planning_system/*. Find the following line:

```
## Declare cpp executables
add_executable(problemInterface src/ProblemGeneration/ProblemInterface.cpp src/ProblemGeneration/PDDLProblemGenerator.cpp)
```

Add the following lines just above:

```
## Declare action interface for Tutorial 10
add_executable(tutorialInterface src/ActionInterface/RPTutorial.cpp src/ProblemGeneration/PDDLProblemGenerator.cpp src/ActionInterface/RPActionInterface.cpp)
add_dependencies(tutorialInterface ${catkin_EXPORTED_TARGETS})
target_link_libraries(tutorialInterface ${catkin_LIBRARIES})
```

Now recompile rosplan. If you have catkin tools, you can use the command:

```
catkin build rosplan_planning_system
```

### 3.4 Launching the Action Interface

Open the launch file *tutorial_10.launch* and add the following lines after the simulated action interfaces:

```
	<!-- dock action interface -->
	<node name="rosplan_interface_undock" pkg="rosplan_planning_system" type="tutorialInterface" respawn="false" output="screen">
		<param name="knowledge_base"		value="rosplan_knowledge_base" />
		<param name="pddl_action_name"		value="undock" />
		<param name="action_dispatch_topic" value="/rosplan_plan_dispatcher/action_dispatch" />
		<param name="action_feedback_topic" value="/rosplan_plan_dispatcher/action_feedback" />
	</node>
```

This will launch a new node of type *tutorialInterface*. The parameters are a part of the *ActionInterface* superclass, and specify:

- The prefix for the knowledge base services, which will be called to update the state with the action's effects.
- The PDDL action name. Actions published with this name will be handled by this interface and others ignored.
- The action dispatch and feedback topics which will be subscribed and published to respectively.

From the terminal, launch the file:

```
roslaunch tutorial_10.launch
```

You should see the output from the simulated actions, and the tutorial interface, like this:

```
[ INFO] [1528191989.417351283]: KCL: (goto_waypoint) Ready to receive
[ INFO] [1528191989.414754601]: KCL: (undock) Ready to receive
[ INFO] [1528191989.410538907]: KCL: (dock) Ready to receive
```

Open a second terminal, source the workspace, and look at the contents of three of those messages using `rostopic echo`:

```
rostopic echo /rosplan_knowledge_base/pddl_action_parameters -n 3
```

This topic is publishing the parameter labels and types of the PDDL operator:

```
name: undock
typed_parameters: 
  - 
    key: "v"
    value: "robot"
  - 
    key: "wp"
    value: "waypoint"
```

### 3.5 Using the Action Interface

In the second terminal, using the script written for [Tutorial 04: Simulated Actions](tutorial_04), start problem generation, planning, and plan dispatch.

```
./tutorial.bash
```

you should see the lines similar to:

```
KCL: (/rosplan_plan_dispatcher) Dispatching plan.
KCL: (/rosplan_plan_dispatcher) Dispatching action [0, undock, 0.460119, 10.000000]
KCL: (undock) action received
KCL: (/rosplan_knowledge_base) Removing domain attribute (docked)
KCL: (undock) TUTORIAL Action completing.
KCL: (/rosplan_knowledge_base) Adding fact (undocked kenny)
KCL: (/rosplan_plan_dispatcher) Feedback received [0, action enabled]
KCL: (/rosplan_plan_dispatcher) Feedback received [0, action achieved]
KCL: (/rosplan_plan_dispatcher) 0: action undock completed
KCL: (/rosplan_plan_dispatcher) Dispatching action [1, localise, 10.461119, 60.000000]
```

You should see that the tutorial interface we just wrote is pickng up the first *undock* action, and that the Knowledge Base is modified accordingly. The execution then hangs on the localisation action, since we removed that simulated interface. You will have to interrupt the script with `ctrl+c`. You can also cancel the plan dispatch by calling a service:

```
rosservice call /rosplan_plan_dispatcher/cancel_dispatch
```

However, since the knowledge base has been modified, it is best to reset everything by killing the launch file as well.
