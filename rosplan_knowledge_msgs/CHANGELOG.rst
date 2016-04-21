^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan_knowledge_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2016-04-21)
------------------

0.0.3 (2016-04-20)
------------------

0.0.2 (2016-04-19)
------------------

0.0.1 (2016-04-19)
------------------
* changed maintainer to not sending emails to the wrong people
* Domain fetching services
  Cleaner knowledge item names
  Removal of instance attributes
* Remove goal service
* Added service message
* Now checking for preconditions during dispatch
  Added knowledge query to knowledge base
  Fixed initial values for dispatch flags
* Waypoint service
* Cleaned code.
  The maximum iterations of the PRM loop is now a variable set in the CreatePRM service.
* Reimplemented part of the PRM, it now uses service parameters to guide search.
  Collision is now properly checked when creating edges between waypoints.
* Altered Knowledge base updates to be services.
* Getting rid of catkin warnings. Everything so tidy.
* Started rewrite of planning system.
  Added timed dispatch from PANDORA.
  Added interfaces and knowledge base from SQUIRREL.
* Contributors: Bram Ridder, Marc Hanheide, Michael, michael
