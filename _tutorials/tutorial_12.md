---
title: Tutorial 12 RDDL and Probabilistic Planning
layout: documentation
permalink: /tutorials/tutorial_11
---
## 1.  Description
This tutorial covers the use of probabilistic planning with ROSPlan. We will mainly cover thr RDDL extension, although PPDDL is also supported.

### Background
For this tutorial we assume you are famliar with the RDDL language by Scott Sanner. If you are not, you can follow the following resources to learn about it:

- RDDLSim software repository: [github.com/ssanner/rddlsim](https://github.com/ssanner/rddlsim)
- RDDL Tutorial: [sites.google.com/site/rddltutorial/](https://sites.google.com/site/rddltutorial/)
- RDDL Language description: [sites.google.com/site/rddltutorial/rddl-language-discription](https://sites.google.com/site/rddltutorial/rddl-language-discription)

## 2. Initial notes
In these examples we will use the [PROST](https://bitbucket.org/tkeller/prost/wiki/Home) planner by Keller _et al._ as it was the winner in the last [IPPC](https://ipc2018-probabilistic.bitbucket.io/#results) competition. However, you should be able to use **any** RDDL-based planner that uses the IPPC client-server protocol. The RDDL version of ROSPlan comes with an online dispatcher that emulates the protocol, as well as an offline version using RDDLSim. For it, we provide a RDDLSim planner interface which parses the output from RDDLSim, also making it usable with any IPPC-abled planner.

### Disclaimer: PDDL Structure
When extending ROSPlan to RDDL and probabilistic planners, we wanted to **keep the interface unmodified**. 
To do so, we needed to adapt the RDDL structure to PDDL-like structure of ROSPlan's messages. This forced some assumptions to be made, as detailed in [ROSPlan's probabilistic extension paper](https://link.springer.com/chapter/10.1007%2F978-3-030-23807-0_20). This tutorial will also go over them.

However, it may be that you see ROSPlan warnings such as `Unknown or unsupported operand type for the expression.` Such warnings mean that some part of the RDDL domain will not be represented in ROSPlan's structures.  Still, this shouldn't be a big issue as the planner will receive the full RDDL domain and will plan with all the information, even when some structures are not present in ROSPlan.

### Disclaimer: RDDL parser
Our RDDL parser was originally taken from PROST. The parser was forked [here](https://github.com/gerardcanal/rddl_parser) and we will try to keep it updated and synced with PROST's one. However, changes or delays may be expected.

Given that we depend on this RDDL parser, issues in it will also be present in ROSPlan. One of the problems you may encounter is the Knowledge Base suddenly dying when planning. This is probably due to an error in your domain description that causes the parser to segfault (see [PROST issue](https://bitbucket.org/tkeller/prost/issues/72/rddl-parser-segfaults-on-instance)). Before opening a ROSPlan issue, please make sure that this is a ROSPlan issue.
<span style="color:red">**We strongly recommend that you test your domains with prost (without rosplan) to make sure they work**.</span>

## 3. The RDDL Knowledge Base
In order to handle RDDL, a new Knowledge Base has been developed. 
It uses the same interface as the PDDL one, so their use should be straightforward. However, there are some new things we menton here.

### New services
The RDDL Knowledge Base provides some new services (apart from the usual ones). Those are:

- `/rosplan_knowledge_base/reload_rddl_domain`: This services reloads the RDDL domain from the file.
- `/rosplan_knowledge_base/state/rddl_parameters`: Returns the domain parameters such as horizon, discount factor and max_nondef_actions.
- `/rosplan_knowledge_base/state/set_rddl_discount_factor`: Sets the discount_factor parameter.
- `/rosplan_knowledge_base/state/set_rddl_horizon`: Sets the horizon parameter.
- `/rosplan_knowledge_base/state/set_rddl_max_nondef_actions`: Sets the max_nondef_actions parameter.

### Launching the KB
The Knowledge Base to be executed is determined by the specified domain extension. Thus, if the `domain_path`paramter ends in `.rddl` the RDDL KB will be launched. If it has extension `.ppddl` the PPDDL one will be started. `.pddl` starts the standard PDDL one.

Apart from that, the RDDL KB has the following parameters (which are equivalent to the set_ services above, but set at launch time):
- `discount_factor`: Sets the discount_factor parameter.
- `horizon`: Sets the horizon parameter.
- `max_nondef_actions`: Sets the max_nondef_actions parameter.

## 4. Online dispatcher
We provide a new dispatcher which works online.
This means that it will keep asking to the planner for the next action rather than getting a whole plan before executing. Thus, planning and execution will be interleaved. 
This also means that the whole plan will not be published, but will keep being updated. It also means there should be no need to replanning (at least not until the defined horizon has been reached). 

Therefore, there is **no need** of calling the `/parse_plan` service when using the online dispatcher, as there is no plan to be parsed. Rather, the `/dispach_plan` service will start the planning proces and the dispatcher at the same time. 

Example of launch file running the online dispatcher:
```
<node name="rosplan_plan_dispatcher" pkg="rosplan_planning_system" type="online_plan_dispatcher" output="screen">
			<param name="knowledge_base"        value="$(arg knowledge_base)" />
			<param name="plan_topic"            value="/rosplan_parsing_interface/$(arg plan_topic)" />
			<param name="action_dispatch_topic" value="$(arg action_dispatch_topic)" />
			<param name="action_feedback_topic" value="$(arg action_feedback_topic)" />
			<param name="ippc_server_port" 		value="$(arg ippc_server_port)" />
            <param name="compute_rewards" 		value="$(arg compute_rewards)" />
</node>
```
In it, the parameters are the following:
- knowledge_base: The name of the knowledge base node to be used. i.e. `rosplan_knowledge_base`
- (**new behavior**) plan_topic: The name of the topic to which the plan will be **published**. As the Online Dispatcher will be interacting with the planner, it will be the getting the plan and publishing it in the plan_topic. The plan will be published after each dispatched action, thus building it incrementally. , i.e. `complete_plan`
- action_dispatch_topic: The topic to which the actions will be dispatched. i.e. `action_dispatch`
- action_feedback_topic: The topic to which the Action Interfaces will publish the feedback. i.e.  `action_feedback`
- (**new**) ippc_server_port: Port to which the planner will connect. The IPPC server will be started in the specified port. It defaults to 3234.
- (**new**) compute_rewards: If true, the current state reward will be compued and the server will send it back to the planner. It may slow down the process as the reward needs to be computed. Recommended use if the planner uses the immediate rewards computed at each time step.

## 5. Testing the domains without ROSPlan

## 6. RDDL interpretation


## Acknowledgement
We would like to thank Prof. Sanner and Dr. Keller for their support.

If you use the the probabilistic extension or RDDL for ROSPlan for any research project, please cite the following paper:
```bibtex
@inproceedings{Canal_taros2019,  
    author = "Canal, Gerard and Cashmore, Michael and Krivi{\'c}, Senka and Aleny{\`a}, Guillem and Magazzeni, Daniele and Torras, Carme",  
    title = "{Probabilistic Planning for Robotics with ROSPlan}",  
    booktitle = "Towards Autonomous Robotic Systems",  
    year = "2019",  
    publisher = "Springer International Publishing",  
    pages = "236--250",  
    isbn = "978-3-030-23807-0",  
    doi = "10.1007/978-3-030-23807-0\_20"  
}
```