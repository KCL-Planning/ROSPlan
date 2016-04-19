^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan_planning_system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2016-04-19)
------------------
* added missing compile dependency for planning lib
* changed maintainer to not sending emails to the wrong people
* compilation error solved
* test build
* install tags
* library
* Added plan parsing for Contingent FF
* Esterel dispatcher
* Added GLUT dependency so that you can use rosdep to install it.
* Added flex dependency so that you can use rosdep to install the prerequisites.
* Bug fix in PlanParser::generateFilter.
  When handling a "plan" command on a domain which has 0-parameters
  predicates, the code in generateFilter would access memory past the end
  of filter_objects[i] array. This would cause several unexpected
  behaviors (most of which end with a segfault).
  The fix simply puts that parts of code that handle predicate parameter
  under the condition that there actually are parameters to handle.
* x
* Pause and Cancel commands
* Minor change to ROS_INFO
* Update UI added precondition false status
* Domain fetching services
  Cleaner knowledge item names
  Removal of instance attributes
* Domain parsing in the knowledge base. New domain structure.
* Better knowledge fetching
* Planning system status
* Updated
* Planning commands
* Complete plan publishing
* Predicate checking completed
* Prints
* Fixed labelling
* Erroneous parameter assignments
* Precondition check correctly labelled
* Replaced dispatcher
* Merge branch 'master' of https://github.com/KCL-Planning/rosplan
  Conflicts:
  rosplan_planning_system/src/PlanDispatcher.cpp
* Additional output in dispatcher on precondition check
* Correct flag as online
* Checking for preconditions updated
  Fixed bug in query knowledge service
* Fixed concurrency flag check
* Fixed flags for dispatch
* Now checking for preconditions during dispatch
  Added knowledge query to knowledge base
  Fixed initial values for dispatch flags
* Removed temp file
* Dispatch strategies
* test commiting
* testing commit
* Update domain.pddl
* Fix little bug introduced in 6840478eacb4ead59f20c7f394bbee899f3c00ec
* Fixed bug in parser when action has no parameters
* Added turtlebot test files
* Update CMakeLists.txt
* Removed incorrect run dependencies (and commit before)
* cola2 interface and simplemapserver
* Fixed minor misnamings in CMakeLists
* Readme for fle, package.xml for mongodb
* Removed useless files
* Merge branch 'master' of https://github.com/KCL-Planning/ROSPlan
* Save plans
* mission filter properly separated from planning filter
* Added return value
* Removed GLUT from CMakeLists.txt
* Fixed roadmap launch file
* Update CMakeLists.txt
* Moved roadmap into own package
* VAL fix (from mzillich)
* Updated Planning system to use proper service names;
  Added turtebot demo
  Knowledge base now clears the scene database
* Altered Knowledge base updates to be services.
* Numerous bug fixes and plan parser; rewrite complete.
  A number of to-dos are left.
* Rewrite of PDDLProblem generation.
  Fixed all ROS INFO prints.
* Updated roadmap server to use costmaps; modified launch file to match.
* Getting rid of catkin warnings. Everything so tidy.
* Moved headers to include
* Continued rewrite of planning loop; planning environment fixed.
  Very minor changes elsewhere.
  Domain simplified to only movebase compatible.
* Fixed names in launch file
* Merge branch 'master' of https://github.com/KCL-Planning/ROSPlan into scratch
  Conflicts:
  planning_system/launch/planning_system.launch
  rosplan_knowledge_msgs/msg/KnowledgeItem.msg
  rosplan_knowledge_msgs/srv/AttributeService.srv
  rosplan_planning_system/CMakeLists.txt
  rosplan_planning_system/src/ActionFeedback.cpp
  rosplan_planning_system/src/PlanningEnvironment.h
  rosplan_planning_system/src/PlanningLoop.cpp
  rosplan_planning_system/src/PostProcess.cpp
* Started rewrite of planning system.
  Added timed dispatch from PANDORA.
  Added interfaces and knowledge base from SQUIRREL.
* Contributors: Emresav, Marc Hanheide, Michael, Michael Cashmore, Neowizard, Simon Vernhes, buildbot-squirrel, fsuarez6, ipa-nhg, m312z, michael
