^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan_interface_mapping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
