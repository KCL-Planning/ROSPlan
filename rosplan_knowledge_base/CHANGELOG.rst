^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan_knowledge_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2016-04-19)
------------------
* changed maintainer to not sending emails to the wrong people
* Merge branch 'master' of https://github.com/KCL-Planning/rosplan
* clear knowledge service in knowledge base
* Added empty file for database dir
* Added flex dependency so that you can use rosdep to install the prerequisites.
* Fixed some domain parsing bugs
* Merge branch 'master' of https://github.com/m312z/ROSPlan
  Conflicts:
  rosplan_knowledge_base/include/rosplan_knowledge_base/DomainParser.h
* Domain fetching services
  Cleaner knowledge item names
  Removal of instance attributes
* Domain parsed setup
* Domain parsing in the knowledge base. New domain structure.
* Removed test robot
* Rewrite of Knowledgebase, preparing for domain parsing
* x
* Fetch all instances
* Remove goal service
* Better knowledge fetching
* Updated filter
* Fix for removal loops
* Fixed same/contains knowledge function
* Checking for preconditions updated
  Fixed bug in query knowledge service
* Now checking for preconditions during dispatch
  Added knowledge query to knowledge base
  Fixed initial values for dispatch flags
* Reading waypoints from file
* Update CMakeLists.txt
* Fixed minor misnamings in CMakeLists
* Fixed bug in domain attribute removal
* mission filter properly separated from planning filter
* Moved roadmap into own package
* Updated Planning system to use proper service names;
  Added turtebot demo
  Knowledge base now clears the scene database
* Altered Knowledge base updates to be services.
* Bug fixes in domain functions
* Updated roadmap server to use costmaps; modified launch file to match.
* Getting rid of catkin warnings. Everything so tidy.
* Continued rewrite of planning loop; planning environment fixed.
  Very minor changes elsewhere.
  Domain simplified to only movebase compatible.
* Started rewrite of planning system.
  Added timed dispatch from PANDORA.
  Added interfaces and knowledge base from SQUIRREL.
* Contributors: Marc Hanheide, Michael, Michael Cashmore, fsuarez6, m312z, michael
