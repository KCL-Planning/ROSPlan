#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "../../src/VALfiles/ptree.h"
#include "FlexLexer.h"

#ifndef KCL_domainparser
#define KCL_domainparser

extern int yyparse();
extern int yydebug;

namespace VAL {
	extern analysis an_analysis;
	extern yyFlexLexer* yfl;
};

/**
 * Domain storage and parsing. Currently supports the requirements:
 *   strips, typing
 *   negative-preconditions
 *   disjunctive-preconditions
 *   fluents
 *   durative-actions
 *   duration-inequalities
 */
namespace KCL_rosplan {

	/*------------------*/
	/* basic structures */
	/*------------------*/

	struct PDDLTypedSymbol
	{
		std::string type;
		std::string name;
	};

	struct PDDLAtomicFormula
	{
		std::string name;
		std::vector<PDDLTypedSymbol> vars;
	};

	/*-----------*/
	/* functions */
	/*-----------*/

	enum PDDLFunctionType { NUMBER, OPERATOR, ATOMIC, DURATION };
	enum PDDLBinaryOperator { ADD, MINUS, MULT, DIV };
	enum PDDLFunctionInequality { GT, LT, EQ, EQLT, EQGT };

	struct PDDLFunction
	{
		PDDLFunction(PDDLFunctionType t) : type(t) {}
		PDDLFunctionType type;
	};

	struct PDDLFunctionNumber : PDDLFunction
	{
		PDDLFunctionNumber(double n)
				: PDDLFunction(NUMBER), number(n) {}
		double number;
	};

	struct PDDLFunctionOperator : PDDLFunction
	{
		PDDLFunctionOperator(PDDLFunction a, PDDLFunction b, PDDLBinaryOperator op)
				: PDDLFunction(OPERATOR), first(a), second(b), function_operator(op) {}
		PDDLBinaryOperator function_operator;
		PDDLFunction first;
		PDDLFunction second;
	};

	struct PDDLFunctionAtomic : PDDLFunction
	{
		PDDLFunctionAtomic(PDDLAtomicFormula f)
				: PDDLFunction(ATOMIC), atomic_function(f) {}
		PDDLAtomicFormula atomic_function;
	};

	struct PDDLFunctionDuration : PDDLFunction
	{
		PDDLFunctionDuration()
				: PDDLFunction(DURATION) {}
	};

	/*-------------------*/
	/* goal descriptions */
	/*-------------------*/

	enum PDDLTimeSpecifier { START, END, ALL };
	enum PDDLGoalDescriptionType { GD_TIMED, GD_ATOMIC, GD_CONJUNCTION, GD_FUNCTION_COMPARISON };
	enum PDDLGoalDescriptionOperand { AND, OR, NOT };

	struct PDDLGoalDescription
	{
		PDDLGoalDescription(PDDLGoalDescriptionType t) : type(t) {}
		PDDLGoalDescriptionType type;
	};

	struct PDDLGDAtomic : PDDLGoalDescription
	{
		PDDLGDAtomic(PDDLAtomicFormula g)
				: PDDLGoalDescription(GD_ATOMIC), goal_condition(g) {}
		PDDLAtomicFormula goal_condition;
	};

	struct PDDLGDTimed : PDDLGoalDescription
	{
		PDDLGDTimed(PDDLTimeSpecifier ts)
				: PDDLGoalDescription(GD_TIMED), time_specifier(ts) {}
		PDDLTimeSpecifier time_specifier;
		std::vector<PDDLGoalDescription> goal_conditions;
	};

	struct PDDLGDConjunction : PDDLGoalDescription
	{
		PDDLGDConjunction(PDDLGoalDescriptionOperand o)
				: PDDLGoalDescription(GD_CONJUNCTION), operand(o) {}
		PDDLGoalDescriptionOperand operand;
		std::vector<PDDLGoalDescription> goal_conditions;
	};

	struct PDDLGDFunction : PDDLGoalDescription
	{
		PDDLGDFunction(PDDLFunction a, PDDLFunction b, PDDLFunctionInequality fi)
				: PDDLGoalDescription(GD_FUNCTION_COMPARISON), first(a), second(b), function_inequality(fi) {}
		PDDLFunctionInequality function_inequality;
		PDDLFunction first;
		PDDLFunction second;
	};

	/*-----------*/
	/* durations */
	/*-----------*/

	struct PDDLDuration
	{
		PDDLDuration(PDDLFunction f, PDDLFunctionInequality fi)
				: duration(f), function_inequality(fi) {}
		// only EQ, EQLT, or EQGT
		PDDLFunctionInequality function_inequality;
		PDDLFunction duration;
	};

	/*------------------*/
	/* durative actions */
	/*------------------*/

	struct PDDLOperator
	{
		PDDLOperator(PDDLDuration d, PDDLGoalDescription c)
				: duration(d), condition(c) {}
		std::string name;
		std::vector<PDDLTypedSymbol> parameters;
		PDDLDuration duration;
		PDDLGoalDescription condition;
	};

	/*--------*/
	/* parser */
	/*--------*/

	class DomainParser
	{
	private:

		/* domain parsing methods */
		PDDLGoalDescription parseCondition(const VAL::goal* goal);
		PDDLFunction parseExpression(const VAL::expression* exp);

	public:

		/* domain information */
		bool domain_parsed;
		std::string domain_name;
		std::vector<std::string> domain_types;
		std::map< std::string, PDDLAtomicFormula> domain_predicates;
		std::map< std::string, PDDLAtomicFormula> domain_functions;
		std::map< std::string, PDDLOperator> domain_operators;

		/* domain parsing */		
		void parseDomain(const std::string domainPath);
	};
}
#endif
