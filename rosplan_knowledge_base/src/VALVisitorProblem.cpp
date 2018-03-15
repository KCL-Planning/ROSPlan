#include "rosplan_knowledge_base/VALVisitorProblem.h"
#include <iostream>

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
     * Visit a proposition to pack into ROS message
     */
    void VALVisitorProblem::visit_proposition(VAL::proposition *p) {

        last_prop.typed_parameters.clear();
        last_prop.labels.clear();

        last_prop.name = p->head->getName();

        std::vector<std::string> predicateLabels;

        // parse domain for predicates
        VAL::pred_decl_list *predicates = domain->predicates;
        for (VAL::pred_decl_list::const_iterator predicateIterator = predicates->begin(); predicateIterator != predicates->end(); predicateIterator++) {

            VAL::pred_decl *tempPredicate = *predicateIterator;

            // compare pedicate name with last proposition name
            if (tempPredicate->getPred()->symbol::getName() == last_prop.name) {

                // iterate the predicate symbols
                for (VAL::var_symbol_list::const_iterator varSymbolIterator = tempPredicate->getArgs()->begin();
                     varSymbolIterator != tempPredicate->getArgs()->end(); varSymbolIterator++) {

                    // add labels to predicateLabels
                    const VAL::var_symbol *tempVarSymbol = *varSymbolIterator;
                    predicateLabels.push_back(tempVarSymbol->pddl_typed_symbol::getName());
                }
            }
        }

        int index = 0;
        for (VAL::parameter_symbol_list::const_iterator vi = p->args->begin(); vi != p->args->end(); vi++) {
            const VAL::parameter_symbol* var = *vi;
            diagnostic_msgs::KeyValue param;
            param.key = var->pddl_typed_symbol::getName();
            param.value = var->type->getName();
            last_prop.typed_parameters.push_back(param);
            last_prop.labels.push_back(predicateLabels[index]);

            // check for supertype and add prop with supertype if it exists
            if (var->type->type->getName() != "object"){
                param.value = var->type->type->getName();
                last_prop.typed_parameters.push_back(param);
                last_prop.labels.push_back(predicateLabels[index]);
            }
            ++index;
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
                        if (tempVarSymbol->type->getName() == last_prop.typed_parameters[a].value && tempVarSymbol->pddl_typed_symbol::getName() == last_prop.labels[a]) {

                            diagnostic_msgs::KeyValue param;
                            param.key = last_prop.labels[a];
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

            VAL::pred_decl* tempPredicate = *predicateIterator;

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
                        if (tempVarSymbol->type->getName() == last_prop.typed_parameters[a].value && tempVarSymbol->pddl_typed_symbol::getName() == last_prop.labels[a]) {

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

    /**
     * Metric
     */

    void VALVisitorProblem::visit_plus_expression(VAL::plus_expression * s){
        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
        cout << "!!!!!!!!!!!!!!  plus_expression\n";
    }
    void VALVisitorProblem::visit_minus_expression(VAL::minus_expression * s){
        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
        cout << "!!!!!!!!!!!!!!  minus_expression\n";
    }
    void VALVisitorProblem::visit_mul_expression(VAL::mul_expression * s){
        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
        cout << "!!!!!!!!!!!!!!  mul_expression\n";
    }
    void VALVisitorProblem::visit_div_expression(VAL::div_expression * s){
        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
        cout << "!!!!!!!!!!!!!!  div_expression\n";
    }
    void VALVisitorProblem::visit_uminus_expression(VAL::uminus_expression * s){
        s->getExpr()->visit(this);

    }
    void VALVisitorProblem::visit_int_expression(VAL::int_expression * s){
        expression << s->double_value();
        cout << "!!!!!!!!!!!!!!  int_expression\n";
    }
    void VALVisitorProblem::visit_float_expression(VAL::float_expression * s){
        cout << "!!!!!!!!!!!!!!  float_expression\n";
        expression << s->double_value();
    }
    void VALVisitorProblem::visit_special_val_expr(VAL::special_val_expr * s){
        cout << "!!!!!!!!!!!!!!  special_val_expr\n";
        expression << s->getKind();
    }
    void VALVisitorProblem::visit_violation_term(VAL::violation_term * v){
        cout << "!!!!!!!!!!!!!!  violation_term\n";
        expression << v->getName();
    }
    void VALVisitorProblem::visit_func_term(VAL::func_term * s) {
        // s->getFunction()->visit(this);
        // s->getArgs()->visit(this);


        cout << "!!!!!!!!!!!!!!  func_term\n";
        last_func_term.typed_parameters.clear();

        // func_term name
        last_func_term.name = s->getFunction()->getName();
        expression << last_func_term.name;

        // func_term variables
        const VAL::parameter_symbol_list* param_list = s->getArgs();
        for (VAL::parameter_symbol_list::const_iterator vi = param_list->begin(); vi != param_list->end(); vi++) {
            const VAL::parameter_symbol* var = *vi;
            diagnostic_msgs::KeyValue param;
            cout << "!!!!!!!!!!!!!!  loop\n";
            expression << var->pddl_typed_symbol::getName();
            param.value = var->type->getName();
            last_func_term.typed_parameters.push_back(param);
        }
    }

    void VALVisitorProblem::visit_metric_spec(VAL::metric_spec * s){

        s->expr->visit(this);

        cout << "!!!!!!!!!!!!!!  metric_spec\n";
        rosplan_knowledge_msgs::KnowledgeItem item;

        item.knowledge_type = 2;
        diagnostic_msgs::KeyValue param;

        if (s->opt == 0){
            param.key = "minimize";
        }
        else {
            param.key = "maximize";
        }

        // option 1
        // something with lasp_prop
        // param.value = last_prop.typed_parameters[a].value;

        // option 2
        param.value = expression.str();
        cout << "metric key: "+param.key+" metric value: "+param.value+"\n";

        item.values.push_back(param);
        metric = item;

    }

    rosplan_knowledge_msgs::KnowledgeItem VALVisitorProblem::returnMetric() {
        problem->metric->visit(this);
        return metric;
    }


} // close namespace
