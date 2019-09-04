---
title: Tutorial 12 RDDL and Probabilistic Planning
layout: documentation
permalink: /tutorials/tutorial_12
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
- `rosservice call /rosplan_knowledge_base/domain/fluent_type`: Returns the type of a fluent.
- `/rosplan_knowledge_base/domain/enumerable_type`: Returns the values inside an enumerable type.

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
            <param name="ippc_server_port"      value="$(arg ippc_server_port)" />
            <param name="compute_rewards"       value="$(arg compute_rewards)" />
</node>
```
In it, the parameters are the following:
- knowledge_base: The name of the knowledge base node to be used. i.e. `rosplan_knowledge_base`
- (**new behavior**) plan_topic: The name of the topic to which the plan will be **published**. As the Online Dispatcher will be interacting with the planner, it will be the getting the plan and publishing it in the plan_topic. The plan will be published after each dispatched action, thus building it incrementally. , i.e. `complete_plan`
- action_dispatch_topic: The topic to which the actions will be dispatched. i.e. `action_dispatch`
- action_feedback_topic: The topic to which the Action Interfaces will publish the feedback. i.e.  `action_feedback`
- (**new**) ippc_server_port: Port to which the planner will connect. The IPPC server will be started in the specified port. It defaults to 3234.
- (**new**) compute_rewards: If true, the current state reward will be compued and the server will send it back to the planner. It may slow down the process as the reward needs to be computed. Recommended use if the planner uses the immediate rewards computed at each time step.

### Running the planner with the online dispatcher
The planner node is the same one as in the deterministic/PDDL version of ROSPlan. However, the planner command is a bit different:

```
<arg name="planner_command"         value="$(find rosplan_planning_system)/common/bin/prost/run_prost_online.sh instance.rddl &quot;[PROST -s 1 -se [IPPC2014]]&quot;" />
```

Where "[PROST -s 1 -se [IPPC2014]]" is PROST's search configuration (you can set your preferred one, see [PROST's help on search configurations](https://bitbucket.org/tkeller/prost/src/default/src/search/main.cc) for more information. Notice the `&quot;` which is used to escape the quote character.

## 5. Plan generation with RDDLSim (to use the simple or Esterel dispatcher)
Standard dispatchers (simple or Esterel) can also be used with RDDL and probabilistic planners. In that case, the plan must be generated beforehand. To do so, we rely on [RDDLSim](https://github.com/ssanner/rddlsim) to simulate the action outcomes. We then parse the plan and dispatch it as usual.

### Running the planner with the standard dispatchers
The planner_command changes a bit when using the standard dispatchers. In this case, instead of runnnig the `run_prost_online.sh` script, we will run the `run_prost.sh` one (Not a big change, huh?)

Thus, similarly as done before, you set the command as:

```
<arg name="planner_command"         value="$(find rosplan_planning_system)/common/bin/prost/run_prost.sh instance.rddl &quot;[PROST -s 1 -se [IPPC2014]]&quot;" />
```

**IMPORTANT**:  If using the standard dispatchers, the planner_interface needs to be run (as the plan needs to be parsed). In this case, the `rddlsim_planner_interface` is the planner_interface to be used. 

## 6. Testing the domains without ROSPlan
As said above, we **strongly recommend** to test the domains without rosplan for RDDL first, as the parser may fail in some cases. To do so, you can use the `run_prost` script that can be found in `rosplan_planning_system/common/bin/prost`. You can run it as follows:

```
./run_prost.sh domain.rddl instance.rddl "[PROST -s 1 -se [IPPC2014]]"
```
If the script is able to produce a plan, then it means that the domain is correct. If you don't, you probably need to fix some issues in the domain (such as typos in fluent names or wrongly used fluents).

## 7. RDDL interpretation
As already said, we process the RDDL in  a concrete way. Thus, if the domain is written in such a way it will be easier to work with ROSPlan.

#### Fluents
Fluents are handled as follows:
- RDDL's boolean fluents are converted to PDDL predicates.
- Real or int fluents are converted to PDDL functions.
- Enumerable types are also supported. They are returned by the `kb/domain/types` service. Their values are represented as a function inside ROSPlan. Thus, you can retrieve the value of an enumerable type by calling `kb/state/functions `, which will return a KnowledgeItem of type function, and its function value will be the enumerable value (which will be the index of the value in the enumerable type list). To recover the enumerable value name, you can call `/rosplan_knowledge_base/domain/enumerable_type` with the type name and it will return the list of values for this type. Thus, if you have an enumerable type defined as:  `t_mytype: { @low, @med, @high};`, getting a value of 0 will refer to `@low`, while a value of 2 will refer to `@high`.

### Action preconditions
Action preconditions are extracted from the action-preconditions block of the RDDL domain.
We expect preconditions to have the form `action => formula`, for which we add `formula` as a precondition of the action. An example is:

```
forall_{?r: robot, ?wf: waypoint, ?wt: waypoint} [goto_waypoint(?r, ?wf, ?wt) => (robot_at(?r, ?wf) ^ localised(?r) ^ undocked(?r))];
```
That specifies that "robot_at and localised and undocked" are preconditions to the goto_waypoint action (with the apropriate parameters set).

Any other rule inside the block with a different format will be ignored when accessing through ROSPlan, but the full domain **will be sent to the planner**, so it will plan with all the constraints (although they may not be accessible via ROSPlan yet).

### Action effects
Action effects are obtained from the cpfs block. 
A fluent appearing in the left hand size of a cpfs formula is added to the effects of an action `a`.
We add the effect when the action fluent `a` appears in the formula, alone, inside a quantified expression or along with other expressions (in which case we only keep the action fluent, and the other state fluents are ignored). 

In the case of an If-then-else, we add an effect when the action fluent `a` appears the formula in the following cases:
 - It appears in the condition of the if clause and the value is `true`.
 - It appears in the condition of the if clause and the value is `false`, in which case we add the negated proposition as the effect.
 - It appears in the else clause: we also add it negated.
 - If the "then" part of the if is not the values `true`, `false`, `Bernoulli` or `Discrete` (probability distributions, see below), the effect will be ignored. Therefore, if writing an if-then-else with a state fluent as the result, we suggest to add the state-fluent in an conjunctive clause with the action fluent in the if part, which will lead to the same result.

Similarly, assign effects will be added when the state-fluent of the LHS of the formula has real or integer type. In this case, we will consider also formulas of the type `fluent = fluent ± expression`, which will be translated to increas/decrease expressions of the fluent value. In the case of the if-then-else clauses, a constant value will be added as an effect, or when it has the form  `fluent ± expression`.

#### Probabilistic effects
This version of ROSPlan added probabilistic effects.
Those were added to the default `rosplan_knowledge_msgs/DomainOperator` message, without changing the already available interface. The message now includes a list of called `probabilistic_effects`, besides the add, del and assign effects. This list has type `rosplan_knowledge_msgs/ProbabilisticEffect`, which is defined as:
```
rosplan_knowledge_msgs/ExprComposite   probability
rosplan_knowledge_msgs/DomainFormula[] add_effects # Add effects
rosplan_knowledge_msgs/DomainFormula[] del_effects # Del effects
rosplan_knowledge_msgs/DomainAssignment[] assign_effects # Assign effects, mainly used for RDDL Discrete distribution
```
Thus, a probabilistic effect is just the typical list of effects but accompanied by a probability value, expressed as a `rosplan_knowledge_msgs/ExprComposite` message. 

Probabilistic effects are processed in the same way as standard effects, with the difference that when the result of the cpfs formula is depending on the probability distibution, we add such effect to the probabilistic effects list. 

**Note:** as per now, we only consider probabilistic effects of type Bernoulli and Discrete. 

#### Exogenous effects
Exogenous effects are those effects that happen in the domain without the robot's intervention. 
ROSPlan is able to find exogenous effects in a RDDL domain and provides its information. To do so, it creates a new virtual operator called "exogenous", which can be queried through the `/rosplan_knowledge_base/domain/operator_details` service, providing a list of effects that happen in the domain without bein related to any action.

Note that this action is not added in the domain and it doesn't exist for the planner as such (though the planner should probably take exogenous events into account). 

### Goals
RDDL doesn't have a goal definition. 
However, there are ways to define the domain such that it goes towards the completion of a goal which consists in defining a fluent which becomes true at the goal state, being the maximum reward only provided in such case.

Although we can parse this as a goal state in ROSPlan, this is currently not handled and it is not active until further tests can be performed.


## 8. Using other IPPC-enabled planners
We dispatch rosplan with PROST, but using any other RDDL-based planner is easier than ever!
As we use the IPPC protocol, any planner using it can be integrated into ROSPlan without the need of coding any interface.

The only difference is the call to the planner, which we do in PROST using the `run_prost.sh` scripts. Those scripts provide an easy interface to PROST and RDDLSim, making it easier to call them. We suggest anyone interested in adding other planners to take a look at those scripts.

More specifically, you would only need to modify the lines 78-84 of `run_prost_online.sh` to call your planner with the online dispatcher, or lines 97-101 of `run_prost.sh` to run it along with RDDLSim for offline dispatching of the actions.

## Acknowledgement
We would like to thank Prof. Sanner and Dr. Keller for their support.

Please, send any suggestion or encountered issues through the Github's issues page.

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