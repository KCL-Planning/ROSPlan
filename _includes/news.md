## News and Events

#### Virtual Machine
A Virtual Machine with ROSPlan installed is now available! [LINK]()  

#### New Features in the Latest Version

- New **tutorials** and **documentation** has been written to walk a user through each component of ROSPlan.  
- The Knowledge Base now keeps track of **timed-initial literals and functions**; **metrics**; and **numeric expressions**.
- An **initial state** can be loaded into the Knowledge Base from a PDDL problem file.  
- Multiple Knowledge Bases can now be run in parallel for systems which use multiple domains, or multiple states.
- New implementations of the Planner Interface: FF, Metric-FF, Contingent-FF, LPG, Fast Downward, TFD, SMTPlan, and UPMurphi.
- The ESTEREL plan formalism now fully supports **temporal plans with concurrent actions and timed-initial literals**.
- The new **simulated action node** can be used to test a planning and execution system with actions competing with a user defined probability.

#### ICAPS 2017 Tutorial
A tutorial on AI Planning for Robotics and Human Interaction will be given at ICAPS 2017. Jun 18, Pittsburgh.  
[http://kcl-planning.github.io/ROSPlan/tutorials/tutorialICAPS2017](http://kcl-planning.github.io/ROSPlan/tutorials/tutorialICAPS2017)
		
#### Support for Contingency Planning
The pnp branch of ROSPlan now supports conditional planning through Contingent-FF.
	
#### Tool support improvements
The update from #### timn####  allows popf to be used in "one-shot" mode, without the "-n" flag.  
[pull request link](https://github.com/KCL-Planning/ROSPlan/pull/42)

#### Qt5 support
The update from #### timn####  introduces Qt5 compatibility for the rqt plugins.  
[pull request link](https://github.com/KCL-Planning/ROSPlan/pull/39)

