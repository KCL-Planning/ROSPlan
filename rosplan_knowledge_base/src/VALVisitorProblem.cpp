#include "rosplan_knowledge_base/VALVisitorProblem.h"

/* implementation of rosplan_knowledge_base::VALVisitorPredicate */
namespace KCL_rosplan {


    /* encoding state */
    bool problem_cond_neg;
    bool problem_eff_neg;
    VAL::time_spec problem_cond_time;
    VAL::time_spec problem_eff_time;

    /*--------------*/
    /* propositions */
    /*--------------*/

    /**
     * Visit an prop to pack into ROS message
     */
    void VALVisitorProblem::visit_proposition(VAL::proposition *p) {

        last_prop.typed_parameters.clear();

        // predicate name
        rosplan_knowledge_msgs::DomainFormula formula;
        last_prop.name = p->head->symbol::getName();
        // predicate variables


        for (VAL::parameter_symbol_list::const_iterator vi = p->args->begin(); vi != p->args->end(); vi++) {
            const VAL::parameter_symbol* var = *vi;
            diagnostic_msgs::KeyValue param;
            param.key = var->pddl_typed_symbol::getName();
            param.value = var->type->getName();
            last_prop.typed_parameters.push_back(param);
        }

    };



    /**
     * Effects
     */
	void VALVisitorProblem::visit_effect_lists(VAL::effect_lists * e) {
		problem_eff_neg = false;
		e->add_effects.pc_list<VAL::simple_effect*>::visit(this);
        problem_eff_neg = true;
		e->del_effects.pc_list<VAL::simple_effect*>::visit(this);
        problem_eff_neg = false;

		e->forall_effects.pc_list<VAL::forall_effect*>::visit(this);
        e->timed_effects.pc_list<VAL::timed_effect*>::visit(this);
	}

    void VALVisitorProblem::visit_timed_effect(VAL::timed_effect * e) {
        problem_eff_time = e->ts;
        e->effs->visit(this);
    };

	void VALVisitorProblem::visit_simple_effect(VAL::simple_effect * e) {

        e->prop->visit(this);

        // parse domain for predicates
        VAL::pred_decl_list *predicates = domain->predicates;
        for (VAL::pred_decl_list::const_iterator predicateIterator = predicates->begin(); predicateIterator != predicates->end(); predicateIterator++) {

            VAL::pred_decl* tempPredicate = *predicateIterator;

            // compare pedicate name with last proposition name
            if (tempPredicate->getPred()->symbol::getName() == last_prop.name) {

                rosplan_knowledge_msgs::KnowledgeItem item;
                item.knowledge_type = 1;
                item.attribute_name = last_prop.name;

                // iterate the predicate symbols
                for (VAL::var_symbol_list::const_iterator varSymbolIterator = tempPredicate->getArgs()->begin();
                     varSymbolIterator != tempPredicate->getArgs()->end(); varSymbolIterator++) {

                    const VAL::var_symbol * tempVarSymbol = *varSymbolIterator;

                    for ( unsigned int a = 0; a < last_prop.typed_parameters.size(); a++){

                        // compare predicate symbol name to match the value of the typed_parameter
                        if (tempVarSymbol->type->getName() == last_prop.typed_parameters[a].value) {

                            diagnostic_msgs::KeyValue param;
                            param.key = tempVarSymbol->pddl_typed_symbol::getName();
                            param.value = last_prop.typed_parameters[a].key;
                            item.values.push_back(param);
                        }
                    }
                }

                item.is_negative = problem_eff_neg;
                facts.push_back(item);
            }
        }

    }

    void VALVisitorProblem::visit_forall_effect(VAL::forall_effect * e) {std::cout << "not implemented forall" << std::endl;};
    void VALVisitorProblem::visit_cond_effect(VAL::cond_effect * e) {std::cout << "not implemented cond" << std::endl;};

    std::vector<rosplan_knowledge_msgs::KnowledgeItem> VALVisitorProblem::returnFacts(){
        visit_effect_lists(problem->initial_state);
        return facts;
    }


    /**
     * Goals
     */

    void VALVisitorProblem::visit_conj_goal(VAL::conj_goal * g){

        g->getGoals()->visit(this);

    }

    void VALVisitorProblem::visit_timed_goal(VAL::timed_goal *c){
        problem_cond_time = c->getTime();
        c->getGoal()->visit(this);
    }

    void VALVisitorProblem::visit_neg_goal(VAL::neg_goal *c) {
        problem_cond_neg = !problem_cond_neg;
        c->getGoal()->visit(this);
        problem_cond_neg = !problem_cond_neg;
    }

    void VALVisitorProblem::visit_simple_goal(VAL::simple_goal* g) {

        g->getProp()->visit(this);

        // parse domain for predicates
        VAL::pred_decl_list *predicates = domain->predicates;
        for (VAL::pred_decl_list::const_iterator predicateIterator = predicates->begin();
             predicateIterator != predicates->end(); predicateIterator++) {

            VAL::pred_decl *tempPredicate = *predicateIterator;

            // compare pedicate name with last proposition name
            if (tempPredicate->getPred()->symbol::getName() == last_prop.name) {

                rosplan_knowledge_msgs::KnowledgeItem item;
                item.knowledge_type = 1;
                item.attribute_name = last_prop.name;

                // iterate the predicate symbols
                for (VAL::var_symbol_list::const_iterator varSymbolIterator = tempPredicate->getArgs()->begin();
                     varSymbolIterator != tempPredicate->getArgs()->end(); varSymbolIterator++) {


                    const VAL::var_symbol *tempVarSymbol = *varSymbolIterator;

                    for (unsigned int a = 0; a < last_prop.typed_parameters.size(); a++) {

                        // compare predicate symbol name to match the value of the typed_parameter
                        if (tempVarSymbol->type->getName() == last_prop.typed_parameters[a].value) {

                            diagnostic_msgs::KeyValue param;
                            param.key = tempVarSymbol->pddl_typed_symbol::getName();
                            param.value = last_prop.typed_parameters[a].key;
                            item.values.push_back(param);

                        }
                    }
                }
                goals.push_back(item);
            }
        }
    }

        std::vector <rosplan_knowledge_msgs::KnowledgeItem> VALVisitorProblem::returnGoals() {

            problem->the_goal->visit(this);
            return goals;
        }


        /**
         * Instances
         */
        std::map <std::string, std::vector<std::string> > VALVisitorProblem::returnInstances() {
            VAL::const_symbol_list *c = problem->objects;
            for (VAL::const_symbol_list::const_iterator symbolListIterator = c->begin();
                 symbolListIterator != c->end(); symbolListIterator++) {
                const VAL::const_symbol *object = *symbolListIterator;
                instances[object->type->getName()].push_back(object->pddl_typed_symbol::getName());
            }
            return instances;
        }


        void VALVisitorProblem::visit_operator_(VAL::operator_ *) {}

        void VALVisitorProblem::visit_qfied_goal(VAL::qfied_goal *) {}
        void VALVisitorProblem::visit_disj_goal(VAL::disj_goal *) {}
        void VALVisitorProblem::visit_imply_goal(VAL::imply_goal *) {}

        void VALVisitorProblem::visit_assignment(VAL::assignment *e) {}
        void VALVisitorProblem::visit_comparison(VAL::comparison *c) {}
        void VALVisitorProblem::visit_derivation_rule(VAL::derivation_rule *o) {}




} // close namespace
