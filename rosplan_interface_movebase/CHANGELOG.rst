^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan_interface_movebase
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2016-04-20)
------------------
* Install target and paths (`#3 <https://github.com/LCAS/ROSPlan/issues/3>`_)
  * added install targets (fixes `#2 <https://github.com/LCAS/ROSPlan/issues/2>`_)
  * made parameters private and included package path prefix for defaults
  * created absolute path for pre-compiled planner binary `popf`
* Contributors: Marc Hanheide

0.0.2 (2016-04-19)
------------------

0.0.1 (2016-04-19)
------------------
* changed maintainer to not sending emails to the wrong people
* Exported target dependiencies
* Clear costmaps on goto_waypoint action failure
* Updated movebase interface to use facts
* Added predicates to movement
* Fixed minor misnamings in CMakeLists
* Removed space; best update
* Updated Planning system to use proper service names;
  Added turtebot demo
  Knowledge base now clears the scene database
* Altered Knowledge base updates to be services.
* Getting rid of catkin warnings. Everything so tidy.
* Added movebase interface
* Contributors: Marc Hanheide, Michael, michael
