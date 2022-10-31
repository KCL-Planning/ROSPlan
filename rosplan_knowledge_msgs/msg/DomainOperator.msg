# A message used to represent an ungrounded operator in the domain.

# (1) name and parameters
rosplan_knowledge_msgs/DomainFormula formula

# (2) duration constraint


# (3) effect lists
rosplan_knowledge_msgs/DomainFormula[] at_start_add_effects
rosplan_knowledge_msgs/DomainFormula[] at_start_del_effects
rosplan_knowledge_msgs/DomainFormula[] at_end_add_effects
rosplan_knowledge_msgs/DomainFormula[] at_end_del_effects
rosplan_knowledge_msgs/DomainAssignment[] at_start_assign_effects
rosplan_knowledge_msgs/DomainAssignment[] at_end_assign_effects
rosplan_knowledge_msgs/ProbabilisticEffect[] probabilistic_effects

# (4) conditions
rosplan_knowledge_msgs/DomainFormula[] at_start_simple_condition
rosplan_knowledge_msgs/DomainFormula[] over_all_simple_condition
rosplan_knowledge_msgs/DomainFormula[] at_end_simple_condition
rosplan_knowledge_msgs/DomainFormula[] at_start_neg_condition
rosplan_knowledge_msgs/DomainFormula[] over_all_neg_condition
rosplan_knowledge_msgs/DomainFormula[] at_end_neg_condition
rosplan_knowledge_msgs/DomainInequality[] at_start_comparison
rosplan_knowledge_msgs/DomainInequality[] at_end_comparison
rosplan_knowledge_msgs/DomainInequality[] over_all_comparison
