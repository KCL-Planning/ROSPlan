^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan_interface_mapping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.9 (2016-09-14)
------------------

0.0.8 (2016-09-12)
------------------

0.0.7 (2016-09-12)
------------------

0.0.6 (2016-09-12)
------------------

0.0.5 (2016-09-09)
------------------
* Merge branch 'master' of https://github.com/KCL-Planning/ROSPlan into merge-upstream
* Merge remote-tracking branch 'origin/devel'
* updated code and launch files to use "/rosplan/" as prefix of configuration parameters instead nodes names.
* created "rosplan_config" package to store common information as MongoDB and planner binaries.
* added wait for service in action clientwq
* returned to global params
* Revert "changed maintainer to not sending emails to the wrong people"
  This reverts commit a041b5defb4c41ac131b209a72df57ff6358f7fe.
* Fix setting the quaternion.
* Contributors: Diego Escudero, Marc Hanheide, Senka212, m312z

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

0.0.1 (2016-04-19)
------------------
* changed maintainer to not sending emails to the wrong people
* Update RPRoadmapServer.cpp
* Fix RPSimpleMapServer who added a supplementary waypoint with a empty string as a name
* Fixes for the NGCUV demo
* Mapping dependencies
* Domain fetching services
  Cleaner knowledge item names
  Removal of instance attributes
* Now checking for preconditions during dispatch
  Added knowledge query to knowledge base
  Fixed initial values for dispatch flags
* Fixed parameter definition
* Closed file
* Fixed frame ID
* Frame ID of visualisation
* x
* Viz loaded waypoints
* Reading waypoints from file
* Update rosplan_roadmap_server.launch
* cola2 interface and simplemapserver
* Fixed minor misnamings in CMakeLists
* roadmap viz update
* Update RPRoadmapServer.cpp
* Added "add waypoint" service to mapping interface
* Cleaned code.
  The maximum iterations of the PRM loop is now a variable set in the CreatePRM service.
* Fixed a bug where waypoints were updated incorrectly if they were sampled outside the max casting range.
* Points are properly scaled now and respect the casting_range parameter.
  The orientation (including rotation) of the map is now respected.
* Reimplemented part of the PRM, it now uses service parameters to guide search.
  Collision is now properly checked when creating edges between waypoints.
* x
* Moved roadmap into own package
* Contributors: Bram Ridder, Marc Hanheide, Michael, Michael Cashmore, Simon Vernhes, michael
