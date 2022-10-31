
#include <iostream>

#include <string>
#include "State.h"
#include "Plan.h"
#include "Validator.h"
#include "typecheck.h"
#include "RobustAnalyse.h"

#include <cstdio>
#include <iostream>
#include <fstream>
#include "ptree.h"
#include "FlexLexer.h"
#include "Utils.h"
#include "Validator.h"
#include "LaTeXSupport.h"

extern int yyparse();
extern int yydebug;

namespace VAL1_2
{

extern parse_category* top_thing;

analysis an_analysis;
extern analysis* current_analysis;

extern yyFlexLexer* yfl;
bool Silent;
int errorCount;
extern bool Verbose;
bool ContinueAnyway;
bool ErrorReport;
bool InvariantWarnings;
bool LaTeX;

//ostream * report = &cout;
extern ostream * report;

bool makespanDefault;

typedef map<double, vector<string> > Ranking;

plan * getPlan (std::stringstream& plan_to_validate, TypeChecker & tc, vector<string> & failed, string & name)
{
	plan * the_plan;

	/*	std::ifstream planFile (argv[argcount++]);
		if (!planFile)
		{
			failed.push_back (name);
			*report << "Bad plan file!\n";
			the_plan = 0; return the_plan;
		};

		yfl = new yyFlexLexer (&planFile, &cout);
	*/
	yfl = new yyFlexLexer (&plan_to_validate, &cout);
	yyparse();
	delete yfl;

	the_plan = dynamic_cast<plan*> (top_thing);

	if (!the_plan || !tc.typecheckPlan (the_plan))
	{
		failed.push_back (name);

		*report << "Bad plan description!\n";
		delete the_plan;
		the_plan = 0; return the_plan;
	};

	if (the_plan->getTime() >= 0)
	{
		name += " - Planner run time: "; name += toString (the_plan->getTime());
	};

	return the_plan;

};

vector<plan_step *> getTimedInitialLiteralActions()
{

	vector<plan_step *> timedIntitialLiteralActions;

	if (an_analysis.the_problem->initial_state->timed_effects.size() != 0)
	{
		int count = 1;
		for (pc_list<timed_effect*>::const_iterator e = an_analysis.the_problem->initial_state->timed_effects.begin(); e != an_analysis.the_problem->initial_state->timed_effects.end(); ++e)
		{
			operator_symbol * timed_initial_lit = an_analysis.op_tab.symbol_put ("Timed Initial Literal Action " + toString (count++));

			action  * timed_initial_lit_action = new action (timed_initial_lit, new var_symbol_list(), new conj_goal (new goal_list()), (*e)->effs, new var_symbol_table());

			plan_step * a_plan_step =  new plan_step (timed_initial_lit, new const_symbol_list());
			a_plan_step->start_time_given = true;
			a_plan_step->start_time = dynamic_cast<const timed_initial_literal *> (*e)->time_stamp;

			a_plan_step->duration_given = false;

			timedIntitialLiteralActions.push_back (a_plan_step);
			an_analysis.the_domain->ops->push_back (timed_initial_lit_action);
		};
	};

	return timedIntitialLiteralActions;
};

void deleteTimedIntitialLiteralActions (vector<plan_step *> tila)
{
	for (vector<plan_step *>::iterator i = tila.begin(); i != tila.end(); ++i)
	{
		delete *i;
	};
};


void executePlan (std::stringstream& plan_to_validate, TypeChecker & tc, const DerivationRules * derivRules, double tolerance, bool lengthDefault, bool giveAdvice)
{
	Ranking rnk;
	Ranking rnkInv;
	vector<string> failed;
	vector<string> queries;

	std::string name = "The master plan";

	plan * the_plan = getPlan (plan_to_validate, tc, failed, name);
	if (the_plan == 0) return;

	plan * copythe_plan = new plan (*the_plan);
	plan * planNoTimedLits = new plan();
	vector<plan_step *> timedInitialLiteralActions = getTimedInitialLiteralActions();
	double deadLine = 101;

	//add timed initial literals to the plan from the problem spec
	for (vector<plan_step *>::iterator ps = timedInitialLiteralActions.begin(); ps != timedInitialLiteralActions.end(); ++ps)
	{
		the_plan->push_back (*ps);
	};

	//add actions that are not to be moved to the timed intitial literals otherwise to the plan to be repaired
	//i.e. pretend these actions are timed initial literals
	for (pc_list<plan_step*>::const_iterator i = copythe_plan->begin(); i != copythe_plan->end(); ++i)
	{
		planNoTimedLits->push_back (*i);
	};

	copythe_plan->clear(); delete copythe_plan;

	PlanRepair pr (timedInitialLiteralActions, deadLine, derivRules, tolerance, tc, an_analysis.the_domain->ops,
	               an_analysis.the_problem->initial_state,
	               the_plan, planNoTimedLits, an_analysis.the_problem->metric, lengthDefault,
	               an_analysis.the_domain->isDurative(), an_analysis.the_problem->the_goal, current_analysis);

	if (Verbose)
		pr.getValidator().displayPlan();



	try
	{

		if (pr.getValidator().execute())
		{
			if (!Silent) cout << "Plan executed successfully - checking goal\n";

			if (pr.getValidator().checkGoal (an_analysis.the_problem->the_goal))

			{
				if (! (pr.getValidator().hasInvariantWarnings()))
				{
					rnk[pr.getValidator().finalValue() ].push_back (name);
					if (!Silent) *report << "Plan valid\n";
					if (!Silent) *report << "Final value: " << pr.getValidator().finalValue() << "\n";
				}
				else
				{
					rnkInv[pr.getValidator().finalValue() ].push_back (name);
					if (!Silent) *report << "Plan valid (subject to further invariant checks)\n";
					if (!Silent) *report << "Final value: " << pr.getValidator().finalValue();
				};
				if (Verbose)
				{
					pr.getValidator().reportViolations();
				};
			}
			else
			{
				failed.push_back (name);
				*report << "Goal not satisfied\n";

				*report << "Plan invalid\n";
				++errorCount;
			};

		}
		else
		{
			failed.push_back (name);
			++errorCount;
			if (ContinueAnyway)
			{
				cout << "\nPlan failed to execute - checking goal\n";

				if (!pr.getValidator().checkGoal (an_analysis.the_problem->the_goal)) *report << "\nGoal not satisfied\n";

			}

			else *report << "\nPlan failed to execute\n";


		};

		if (pr.getValidator().hasInvariantWarnings())
		{
			cout << "\n\n";
			*report << "This plan has the following further condition(s) to check:";
			cout << "\n\n";

			pr.getValidator().displayInvariantWarnings();
		};
	}
	catch (exception & e)
	{
		cout << "Error occurred in validation attempt:\n  " << e.what() << "\n";
		queries.push_back (name);

	};

	//display error report and plan repair advice
	if (giveAdvice && (Verbose || ErrorReport))
	{
		pr.firstPlanAdvice();
	};

	planNoTimedLits->clear(); delete planNoTimedLits;
	delete the_plan;

	if (!rnk.empty())
	{
		if (!Silent) cout << "\nSuccessful plans:";


		if (an_analysis.the_problem->metric &&
		    an_analysis.the_problem->metric->opt == E_MINIMIZE)
		{
			if (!Silent) for_each (rnk.begin(), rnk.end(), showList());

		}
		else
		{
			if (!Silent) for_each (rnk.rbegin(), rnk.rend(), showList());
		};



		*report << "\n";
	};

	if (!rnkInv.empty())
	{
		if (!Silent) cout << "\nSuccessful Plans Subject To Further Invariant Checks:";


		if (an_analysis.the_problem->metric &&
		    an_analysis.the_problem->metric->opt == E_MINIMIZE)
		{
			for_each (rnkInv.begin(), rnkInv.end(), showList());
		}
		else
		{
			for_each (rnkInv.rbegin(), rnkInv.rend(), showList());
		};



		*report << "\n";
	};

	if (!failed.empty())
	{
		cout << "\n\nFailed plans:\n ";
		copy (failed.begin(), failed.end(), ostream_iterator<string> (cout, " "));
		*report << "\n";
	};

	if (!queries.empty())
	{
		cout << "\n\nQueries (validator failed):\n ";
		copy (queries.begin(), queries.end(), ostream_iterator<string> (cout, " "));
		*report << "\n";
	};

};


bool checkPlan (const std::string& domain_file, const std::string& problem_file, std::stringstream& plan)
{
	try
	{
		current_analysis = &an_analysis;
		//an_analysis.const_tab.symbol_put(""); //for events - undefined symbol
		Silent = false;
		errorCount = 0;
		Verbose = false;
		ContinueAnyway = false;
		ErrorReport = false;
		InvariantWarnings = false;
		makespanDefault = false;
		bool CheckDPs = true;
		bool giveAdvice = true;

		double tolerance = 0.01;
		bool lengthDefault = true;

		string s;

		std::ifstream domainFile (domain_file.c_str());
		if (!domainFile)
		{
			std::cerr << "Bad domain file!\n";
			exit (1);
		};

		yfl = new yyFlexLexer (&domainFile, &cout);

		yydebug = 0;
		yyparse();
		delete yfl;

		if (!an_analysis.the_domain)
		{
			std::cerr << "Problem in domain definition!\n";
			exit (1);
		};

		TypeChecker tc (current_analysis);

		bool typesOK = tc.typecheckDomain();

		if (!typesOK)
		{
			std::cerr << "Type problem in domain description!\n";
			exit (1);
		};

		std::ifstream problemFile (problem_file.c_str());
		if (!problemFile)
		{
			std::cerr << "Bad problem file!\n";
			exit (1);
		};

		yfl = new yyFlexLexer (&problemFile, &cout);
		yyparse();
		delete yfl;

		if (!tc.typecheckProblem())
		{
			std::cerr << "Type problem in problem specification!\n";
			exit (1);
		};

		const DerivationRules * derivRules = new DerivationRules (an_analysis.the_domain->drvs, an_analysis.the_domain->ops);

		if (CheckDPs && !derivRules->checkDerivedPredicates())
		{
			exit (1);
		};

		executePlan (plan, tc, derivRules, tolerance, lengthDefault, giveAdvice);


		delete derivRules;

	}
	catch (exception & e)
	{
		std::cerr << "Error: " << e.what() << "\n";
		an_analysis.error_list.report();
		return 2;
	};

	return errorCount == 0;
};

};
