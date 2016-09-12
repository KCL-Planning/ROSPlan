^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan_knowledge_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.7 (2016-09-12)
------------------

0.0.6 (2016-09-12)
------------------

0.0.5 (2016-09-09)
------------------
* Added getDomainPredicateDetails service.
  Service is used by the sample action interface to know the correct parameter labels with which to update the state.
* Added domain operator details service
* homogenised versions to 0.0.0
* Revert "changed maintainer to not sending emails to the wrong people"
  This reverts commit a041b5defb4c41ac131b209a72df57ff6358f7fe.
* ROSPlan can now dispatch contingent plans.
  Observations are not checked properly.
  KnowledgeItems can now be negative
* updated query service to return bool array
* Generate contingent problem calls
* Updated DOT output
* Merge branch 'squirrel' of https://github.com/KCL-Planning/rosplan into squirrel
* Knowledge Array and some parsing
* problem gen as service
* Contributors: Bram Ridder, Marc Hanheide, m312z

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
