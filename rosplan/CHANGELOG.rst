^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosplan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.8 (2016-09-12)
------------------

0.0.7 (2016-09-12)
------------------

0.0.6 (2016-09-12)
------------------

0.0.5 (2016-09-09)
------------------
* Merge branch 'master' of https://github.com/KCL-Planning/ROSPlan into merge-upstream
* Adding rosplan_rqt to meta package (`#4 <https://github.com/LCAS/ROSPlan/issues/4>`_)
  Not sure if this is supposed to be in there but it is currently missing and hence not install when installing `ros-indigo-rosplan`. Please, just reject if it should not be installed by default.
* created "rosplan_config" package to store common information as MongoDB and planner binaries.
* Revert "changed maintainer to not sending emails to the wrong people"
  This reverts commit a041b5defb4c41ac131b209a72df57ff6358f7fe.
* Contributors: Christian Dondrup, Diego Escudero, Marc Hanheide

0.0.4 (2016-04-21)
------------------

0.0.3 (2016-04-20)
------------------

0.0.2 (2016-04-19)
------------------

0.0.1 (2016-04-19)
------------------
* changed maintainer to not sending emails to the wrong people
* Update package.xml
* q
* Fixed minor misnamings in CMakeLists
* Moved roadmap into own package
* Updated Planning system to use proper service names;
  Added turtebot demo
  Knowledge base now clears the scene database
* Continued rewrite of planning loop; planning environment fixed.
  Very minor changes elsewhere.
  Domain simplified to only movebase compatible.
* Started rewrite of planning system.
  Added timed dispatch from PANDORA.
  Added interfaces and knowledge base from SQUIRREL.
* Contributors: Marc Hanheide, Michael, Michael Cashmore, michael
