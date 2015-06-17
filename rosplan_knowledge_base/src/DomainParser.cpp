#include "rosplan_knowledge_base/DomainParser.h"

/* implementation of DomainParser.h */
namespace KCL_rosplan {

	/*----------------*/
	/* parsing domain */
	/*----------------*/

	/**
	 * parse the domain file
	 */
	void DomainParser::parseDomain(const std::string domainPath) {
		
		// only parse domain once
		if(domain_parsed) return;
		domain_parsed = true;

		std::string domainFileName = (domainPath);
		ROS_INFO("KCL: (KB) Parsing domain: %s.", domainFileName.c_str());

		// save filename for VAL
		std::vector<char> writable(domainFileName.begin(), domainFileName.end());
		writable.push_back('\0');
		current_filename = &writable[0];

		// parse domain
		VAL::current_analysis = &VAL::an_analysis;
		std::ifstream domainFile;
		domainFile.open(domainFileName.c_str());
		yydebug = 0;

		VAL::yfl = new yyFlexLexer;

		if (domainFile.bad()) {
			ROS_ERROR("KCL: (KB) Failed to open domain file.");
			line_no = 0;
			VAL::log_error(VAL::E_FATAL,"Failed to open file");
		} else {
			line_no = 1;
			VAL::yfl->switch_streams(&domainFile, &std::cout);
			yyparse();

			// domain name
			VAL::domain* domain = VAL::current_analysis->the_domain;
			domain_name = domain->name;

			// types
			VAL::pddl_type_list* types = domain->types;
			for (VAL::pddl_type_list::const_iterator ci = types->begin(); ci != types->end(); ci++) {
				const VAL::pddl_type* type = *ci;
				domain_types.push_back(type->getName());
			}

			// predicates
			VAL::pred_decl_list* predicates = domain->predicates;
			if(predicates) {
				for (VAL::pred_decl_list::const_iterator ci = predicates->begin(); ci != predicates->end(); ci++) {
					const VAL::pred_decl* predicate = *ci;

					// predicate name
					PDDLAtomicFormula pred;
					pred.name = predicate->getPred()->symbol::getName();
					domain_predicates[pred.name] = pred;

					// predicate variables
					for (VAL::var_symbol_list::const_iterator vi = predicate->getArgs()->begin(); vi != predicate->getArgs()->end(); vi++) {

						const VAL::var_symbol* var = *vi;
						PDDLTypedSymbol sym;
						sym.type = var->type->getName();
						sym.name = var->pddl_typed_symbol::getName();
						domain_predicates[pred.name].vars.push_back(sym);
					}
				}
			}

			// functions
			VAL::func_decl_list* functions = domain->functions;
			if(functions) {
				for (VAL::func_decl_list::const_iterator ci = functions->begin(); ci != functions->end(); ci++) {
					const VAL::func_decl* function = *ci;
					// function name
					PDDLAtomicFormula func;
					func.name = function->getFunction()->symbol::getName();
					domain_functions[func.name] = func;

					// parameters
					for (VAL::var_symbol_list::const_iterator vi = function->getArgs()->begin(); vi != function->getArgs()->end(); vi++) {
						const VAL::var_symbol* var = *vi;
						PDDLTypedSymbol sym;
						sym.type = var->type->getName();
						sym.name = var->pddl_typed_symbol::getName();
						domain_functions[func.name].vars.push_back(sym);
					}
				}
			}

			// operators
			VAL::operator_list* operators = domain->ops;
			for (VAL::operator_list::const_iterator ci = operators->begin(); ci != operators->end(); ci++) {			
				const VAL::operator_* op = *ci;

				// name
				std::string name = op->name->symbol::getName();

				// condition
				const VAL::goal* precondition = op->precondition;
				PDDLGoalDescription c = parseCondition(precondition);

				// duration
				PDDLFunctionNumber n(0);
				PDDLDuration d(n, EQ);
				const VAL::durative_action* da = dynamic_cast<const VAL::durative_action*>(op);
				if (da) {
					// TODO get the real duration from VAL
					PDDLFunctionNumber n(1);
					d = PDDLDuration(n, EQ);
				}

				// operator
				PDDLOperator oper(d, c);
				oper.name = name;

				// parameters
				for (VAL::var_symbol_list::const_iterator vi = op->parameters->begin(); vi != op->parameters->end(); vi++) {
					const VAL::var_symbol* var = *vi;
					PDDLTypedSymbol sym;
					sym.type = var->type->getName();
					sym.name = var->pddl_typed_symbol::getName();
					oper.parameters.push_back(sym);
				}

				domain_operators.insert( std::pair<std::string,PDDLOperator>(name, oper) );
			}
		}
		domainFile.close();
		delete VAL::yfl;
	}

	/**
	 * parse an expression recursively.
	 */
	PDDLFunction DomainParser::parseExpression(const VAL::expression* exp) {

		// number
		const VAL::num_expression* nex = dynamic_cast<const VAL::num_expression*>(exp);
		if (nex) {
			PDDLFunctionNumber func(nex->double_value());
			return func;
		}

		// unit minus
		const VAL::uminus_expression* ume = dynamic_cast<const VAL::uminus_expression*>(exp);
		if (ume) {
			PDDLFunctionNumber lhs(0);
			PDDLFunction rhs = parseExpression(ume->getExpr());
			PDDLFunctionOperator func(lhs, rhs, MINUS);
			return func;
		}

		// function
		const VAL::func_term* ft = dynamic_cast<const VAL::func_term*>(exp);
		if (ft) {
			PDDLAtomicFormula prop;
			prop.name = ft->getFunction()->symbol::getName();
	        for (VAL::parameter_symbol_list::const_iterator ci = ft->getArgs()->begin(); ci != ft->getArgs()->begin(); ci++) {
				const VAL::parameter_symbol* param = *ci;
				PDDLTypedSymbol sym;
				sym.type = param->type->getName();
				sym.name = param->pddl_typed_symbol::getName();
				prop.vars.push_back(sym);
			}
			PDDLFunctionAtomic pgfa(prop);
			return pgfa;
		}

		// binary expressions
		const VAL::binary_expression* be = dynamic_cast<const VAL::binary_expression*>(exp);
		if (be) {

			PDDLFunction lhs = parseExpression(be->getLHS());
			PDDLFunction rhs = parseExpression(be->getRHS());

			// plus
			PDDLBinaryOperator bop = ADD;
			const VAL::plus_expression* pe = dynamic_cast<const VAL::plus_expression*>(exp);
			if (pe) bop = ADD;

			// minus
			const VAL::minus_expression* mie = dynamic_cast<const VAL::minus_expression*>(exp);
			if (mie) bop = MINUS;

			// multiply
			const VAL::mul_expression* mue = dynamic_cast<const VAL::mul_expression*>(exp);
			if (mue) bop = MULT;

			// divide
			const VAL::div_expression* de = dynamic_cast<const VAL::div_expression*>(exp);
			if (de) bop = DIV;

			PDDLFunctionOperator func(lhs, rhs, bop);
			return func;
		}

		// "?duration"
		const VAL::special_val_expr* spe = dynamic_cast<const VAL::special_val_expr*>(exp);
		if (spe) {
			PDDLFunctionDuration pfd;
			return pfd;
		}
	}

	/**
	 * parse a condition recursively.
	 */
	PDDLGoalDescription DomainParser::parseCondition(const VAL::goal* goal) {
		
		// simple proposition (base case 1)
		const VAL::simple_goal* sg = dynamic_cast<const VAL::simple_goal*>(goal);
		if (sg) {
			const VAL::proposition* valprop = sg->getProp();
			PDDLAtomicFormula prop;
			prop.name = valprop->head->symbol::getName();
	        for (VAL::parameter_symbol_list::const_iterator ci = valprop->args->begin(); ci != valprop->args->end(); ci++) {
				const VAL::parameter_symbol* param = *ci;
				PDDLTypedSymbol sym;
				sym.type = param->type->getName();
				sym.name = param->pddl_typed_symbol::getName();
				prop.vars.push_back(sym);
			}
			PDDLGDAtomic pgda(prop);
			return pgda;
		}

		// function inequality (base case 2)
		const VAL::comparison* co = dynamic_cast<const VAL::comparison*>(goal);
		if (co) {
			PDDLFunctionInequality bin_expression = EQ;
			switch(co->getOp()) {
				case VAL::E_GREATER: bin_expression = GT; break;
				case VAL::E_GREATEQ: bin_expression = EQGT; break;
				case VAL::E_LESS: bin_expression = LT; break;
				case VAL::E_LESSEQ: bin_expression = EQLT; break;
			};
			PDDLFunction lhs = parseExpression(co->getLHS());
			PDDLFunction rhs = parseExpression(co->getRHS());
			PDDLGDFunction pgdf(lhs, rhs, bin_expression);
			return pgdf;
		}

		// negative condition
		const VAL::neg_goal* ng = dynamic_cast<const VAL::neg_goal*>(goal);
		if (ng) {
			PDDLGDConjunction pgda(NOT);
			PDDLGoalDescription inner = parseCondition(ng->getGoal());
			pgda.goal_conditions.push_back(inner);
			return pgda;
		}

		// conjunctive condition
		const VAL::conj_goal* cg = dynamic_cast<const VAL::conj_goal*>(goal);
		if (cg) {
			PDDLGDConjunction pgda(AND);
	        const VAL::goal_list* goals = cg->getGoals();
	        for (VAL::goal_list::const_iterator ci = goals->begin(); ci != goals->end(); ci++) {
				PDDLGoalDescription inner = parseCondition((*ci));
				pgda.goal_conditions.push_back(inner);
			}
			return pgda;
		}

		// disjunctive condition
		const VAL::disj_goal* dg = dynamic_cast<const VAL::disj_goal*>(goal);
		if (dg) {
			PDDLGDConjunction pgda(OR);
	        const VAL::goal_list* goals = dg->getGoals();
	        for (VAL::goal_list::const_iterator ci = goals->begin(); ci != goals->end(); ci++) {
				PDDLGoalDescription inner = parseCondition((*ci));
				pgda.goal_conditions.push_back(inner);
			}
			return pgda;
		}

		// timed condition
		const VAL::timed_goal* tg = dynamic_cast<const VAL::timed_goal*>(goal);
		if (tg) {
			PDDLTimeSpecifier time_spec = START;
			switch(tg->getTime()) {
				case VAL::E_AT_END: time_spec = END; break;
				case VAL::E_OVER_ALL: time_spec = ALL; break;
			};
			PDDLGDTimed pgdt(time_spec);
			PDDLGoalDescription inner = parseCondition(tg->getGoal());
			pgdt.goal_conditions.push_back(inner);
			return pgdt;
		}
	}

} // close namespace
