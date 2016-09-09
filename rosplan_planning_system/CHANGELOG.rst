^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan_planning_system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2016-09-09)
------------------
* Exposing domain_path in launch files.
* Fixing wrong parameter handling
* 'common' of package ros_planning_system does not exist any more and is should not be installed.
* Fixing launch files (includes setting working dir by default to tmp/) and reintroducing correct parameter parsing that was lost in merge.
* Merge branch 'master' of https://github.com/KCL-Planning/ROSPlan into merge-upstream
* Added getDomainPredicateDetails service.
  Service is used by the sample action interface to know the correct parameter labels with which to update the state.
* fixed parameters. Problem was not loaded correctly; no extra rosplan namespace inside node namespace. param->getParam
* Merge remote-tracking branch 'origin/devel'
* updated code and launch files to use "/rosplan/" as prefix of configuration parameters instead nodes names.
* created "rosplan_config" package to store common information as MongoDB and planner binaries.
* moved domain files used in demos to 'ros_demos' package.
* added wait for service in action clientwq
* fixed string parsing bug in POPFEsterelPlanParser
* using esterel dispatch in ROSPlan
* POPF to Esterel
* minor comment fix
* fixed problem generation
* added problem publisher
* updated launch files to reflect new parameters
* status goes back to READY when a plan fails, max planning attempts
* Merge branch 'master' into devel
  Conflicts:
  rosplan_planning_system/CMakeLists.txt
* fixed bug in RPActionInterface
* updating squirrel branch to fix bug in turtle demo
* Updated action interface to make KB changes
* action interface automatically updates knwoledge base
* action feedback and parameter check moved into super
* added missing include
* action interface
* made default working_dir /tmp
* returned to global params
* added mongdb_store dependency
* added missing compile dependency for planning lib
* Revert "changed maintainer to not sending emails to the wrong people"
  This reverts commit a041b5defb4c41ac131b209a72df57ff6358f7fe.
* updated esterel dispatch to only check conditions of already activated nodes. Only prints plan when something changes.
* basically nothing
* plan graph listener
* Merge pull request `#23 <https://github.com/LCAS/ROSPlan/issues/23>`_ from KCL-Planning/master
  Update squirrel branch
* Merge branch 'squirrel' of https://github.com/kcl-planning/ROSPlan into squirrel
* Fixed some fixes in the esterel dispatcher.
  Nodes are no longer stay 'completed' after executing.
* removed odd line from RPMoveBase and added plan to graph
* putting the DOT graph onto a topic instead of a file
* removed some extra INFO prints
* Added jump conditions in the CFFParser
  Update EsterelPlanDispatcher to dispatch actions on (OR(action edge) and AND(condition edge))
* Merge branch 'squirrel' of https://github.com/kcl-planning/ROSPlan into squirrel
* Esterel dispatches now uses the global data path so it can be used by other planning instances (only relevant for the d3 visualiser).
* return failure on a failure to plan
* Increased font size
* changes for dispatching from prewritten plan
* Write the DOT to different files.
  Give the planner 60 seconds timeout.
* Added ff
* Changed the interal data structures that represent an Esterel program from mappings from names to instances to pointers.
  Contingent plans are parsed properly now (shed actions were not handled correctly before).
* ROSPlan can now dispatch contingent plans.
  Observations are not checked properly.
  KnowledgeItems can now be negative
* condition checking in esterel dispatch
* CFF parsing direct to Esterel, removed duplicated plan store
* * Fixed CFF parser and Exterel Plan Dispatches + assorted bug fixes.
* Generate contingent problem calls
* Updated DOT output
* Knowledge Array and some parsing
* info prints update
* the two bugs
* Last changes from the integration meeting.
* Planning as an action
* new service
* esterel edge checking
* start planning with specific ID
* problem gen as service
* Linking contingent domains.
* Contributors: Bram, Bram Ridder, Christian Dondrup, Diego Escudero, Marc Hanheide, Michael Cashmore, m312z

0.0.4 (2016-04-21)
------------------
* removed ~ which is incorrectly introduced
* Contributors: Marc Hanheide

0.0.3 (2016-04-20)
------------------
* Install target and paths (`#3 <https://github.com/LCAS/ROSPlan/issues/3>`_)
  * added install targets (fixes `#2 <https://github.com/LCAS/ROSPlan/issues/2>`_)
  * made parameters private and included package path prefix for defaults
  * created absolute path for pre-compiled planner binary `popf`
* Contributors: Marc Hanheide

0.0.2 (2016-04-19)
------------------
* added mongdb_store dependency
* Contributors: Marc Hanheide

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
