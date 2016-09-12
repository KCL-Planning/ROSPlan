^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.6 (2016-09-12)
------------------

0.0.5 (2016-09-09)
------------------
* fixed parameters. Problem was not loaded correctly; no extra rosplan namespace inside node namespace. param->getParam
* Merge remote-tracking branch 'origin/devel'
* created "rosplan_config" package to store common information as MongoDB and planner binaries.
* moved domain files used in demos to 'ros_demos' package.
* added wait for service in action clientwq
* added fixed frame to mc2_turtlebot launch
* Merge branch 'master' into devel
  Conflicts:
  rosplan_planning_system/CMakeLists.txt
* updating squirrel branch to fix bug in turtle demo
* updated turtlebot package name in mc2 launch
* Contributors: Diego Escudero, m312z

0.0.4 (2016-04-21)
------------------

0.0.3 (2016-04-20)
------------------
* Install target and paths (`#3 <https://github.com/LCAS/ROSPlan/issues/3>`_)
  * added install targets (fixes `#2 <https://github.com/LCAS/ROSPlan/issues/2>`_)
  * made parameters private and included package path prefix for defaults
  * created absolute path for pre-compiled planner binary `popf`
* Contributors: Marc Hanheide

0.0.2 (2016-04-19)
------------------
* changed maintainer
* Contributors: Marc Hanheide

0.0.1 (2016-04-19)
------------------
* Update turtlebot_explore.bash
* updated turtlebot package name in mc2 launch
* altered sample waypoints.txt
* Changed playground to world in launch files
* Fixes for the NGCUV demo
* Update turtlebot_explore.bash
* Update turtlebot_explore.bash
* Domain parsing in the knowledge base. New domain structure.
* Updated
* Fixed parameter definition
* Added turtlebot test files
* Update turtlebot_explore.bash
* Update turtlebot_explore.bash
* Updated cola2 demo
* cola2 interface and simplemapserver
* Updated Planning system to use proper service names;
  Added turtebot demo
  Knowledge base now clears the scene database
* Contributors: Michael, Michael Cashmore, m312z, michael
