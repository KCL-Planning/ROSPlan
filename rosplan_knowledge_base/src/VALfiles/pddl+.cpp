/* A Bison parser, made by GNU Bison 2.3.  */

/* Skeleton implementation for Bison's Yacc-like parsers in C

   Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
   Free Software Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "2.3"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Using locations.  */
#define YYLSP_NEEDED 0



/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     OPEN_BRAC = 258,
     CLOSE_BRAC = 259,
     OPEN_SQ = 260,
     CLOSE_SQ = 261,
     DEFINE = 262,
     PDDLDOMAIN = 263,
     REQS = 264,
     EQUALITY = 265,
     STRIPS = 266,
     ADL = 267,
     NEGATIVE_PRECONDITIONS = 268,
     TYPING = 269,
     DISJUNCTIVE_PRECONDS = 270,
     EXT_PRECS = 271,
     UNIV_PRECS = 272,
     QUANT_PRECS = 273,
     COND_EFFS = 274,
     FLUENTS = 275,
     OBJECTFLUENTS = 276,
     NUMERICFLUENTS = 277,
     ACTIONCOSTS = 278,
     TIME = 279,
     DURATIVE_ACTIONS = 280,
     DURATION_INEQUALITIES = 281,
     CONTINUOUS_EFFECTS = 282,
     DERIVED_PREDICATES = 283,
     TIMED_INITIAL_LITERALS = 284,
     PREFERENCES = 285,
     CONSTRAINTS = 286,
     ACTION = 287,
     PROCESS = 288,
     EVENT = 289,
     DURATIVE_ACTION = 290,
     DERIVED = 291,
     CONSTANTS = 292,
     PREDS = 293,
     FUNCTIONS = 294,
     TYPES = 295,
     ARGS = 296,
     PRE = 297,
     CONDITION = 298,
     PREFERENCE = 299,
     START_PRE = 300,
     END_PRE = 301,
     EFFECTS = 302,
     INITIAL_EFFECT = 303,
     FINAL_EFFECT = 304,
     INVARIANT = 305,
     DURATION = 306,
     AT_START = 307,
     AT_END = 308,
     OVER_ALL = 309,
     AND = 310,
     OR = 311,
     EXISTS = 312,
     FORALL = 313,
     IMPLY = 314,
     NOT = 315,
     WHEN = 316,
     WHENEVER = 317,
     EITHER = 318,
     PROBLEM = 319,
     FORDOMAIN = 320,
     INITIALLY = 321,
     OBJECTS = 322,
     GOALS = 323,
     EQ = 324,
     LENGTH = 325,
     SERIAL = 326,
     PARALLEL = 327,
     METRIC = 328,
     MINIMIZE = 329,
     MAXIMIZE = 330,
     HASHT = 331,
     DURATION_VAR = 332,
     TOTAL_TIME = 333,
     INCREASE = 334,
     DECREASE = 335,
     SCALE_UP = 336,
     SCALE_DOWN = 337,
     ASSIGN = 338,
     GREATER = 339,
     GREATEQ = 340,
     LESS = 341,
     LESSEQ = 342,
     Q = 343,
     COLON = 344,
     NUMBER = 345,
     ALWAYS = 346,
     SOMETIME = 347,
     WITHIN = 348,
     ATMOSTONCE = 349,
     SOMETIMEAFTER = 350,
     SOMETIMEBEFORE = 351,
     ALWAYSWITHIN = 352,
     HOLDDURING = 353,
     HOLDAFTER = 354,
     ISVIOLATED = 355,
     BOGUS = 356,
     NAME = 357,
     FUNCTION_SYMBOL = 358,
     INTVAL = 359,
     FLOATVAL = 360,
     AT_TIME = 361,
     PLUS = 362,
     HYPHEN = 363,
     DIV = 364,
     MUL = 365,
     UMINUS = 366
   };
#endif
/* Tokens.  */
#define OPEN_BRAC 258
#define CLOSE_BRAC 259
#define OPEN_SQ 260
#define CLOSE_SQ 261
#define DEFINE 262
#define PDDLDOMAIN 263
#define REQS 264
#define EQUALITY 265
#define STRIPS 266
#define ADL 267
#define NEGATIVE_PRECONDITIONS 268
#define TYPING 269
#define DISJUNCTIVE_PRECONDS 270
#define EXT_PRECS 271
#define UNIV_PRECS 272
#define QUANT_PRECS 273
#define COND_EFFS 274
#define FLUENTS 275
#define OBJECTFLUENTS 276
#define NUMERICFLUENTS 277
#define ACTIONCOSTS 278
#define TIME 279
#define DURATIVE_ACTIONS 280
#define DURATION_INEQUALITIES 281
#define CONTINUOUS_EFFECTS 282
#define DERIVED_PREDICATES 283
#define TIMED_INITIAL_LITERALS 284
#define PREFERENCES 285
#define CONSTRAINTS 286
#define ACTION 287
#define PROCESS 288
#define EVENT 289
#define DURATIVE_ACTION 290
#define DERIVED 291
#define CONSTANTS 292
#define PREDS 293
#define FUNCTIONS 294
#define TYPES 295
#define ARGS 296
#define PRE 297
#define CONDITION 298
#define PREFERENCE 299
#define START_PRE 300
#define END_PRE 301
#define EFFECTS 302
#define INITIAL_EFFECT 303
#define FINAL_EFFECT 304
#define INVARIANT 305
#define DURATION 306
#define AT_START 307
#define AT_END 308
#define OVER_ALL 309
#define AND 310
#define OR 311
#define EXISTS 312
#define FORALL 313
#define IMPLY 314
#define NOT 315
#define WHEN 316
#define WHENEVER 317
#define EITHER 318
#define PROBLEM 319
#define FORDOMAIN 320
#define INITIALLY 321
#define OBJECTS 322
#define GOALS 323
#define EQ 324
#define LENGTH 325
#define SERIAL 326
#define PARALLEL 327
#define METRIC 328
#define MINIMIZE 329
#define MAXIMIZE 330
#define HASHT 331
#define DURATION_VAR 332
#define TOTAL_TIME 333
#define INCREASE 334
#define DECREASE 335
#define SCALE_UP 336
#define SCALE_DOWN 337
#define ASSIGN 338
#define GREATER 339
#define GREATEQ 340
#define LESS 341
#define LESSEQ 342
#define Q 343
#define COLON 344
#define NUMBER 345
#define ALWAYS 346
#define SOMETIME 347
#define WITHIN 348
#define ATMOSTONCE 349
#define SOMETIMEAFTER 350
#define SOMETIMEBEFORE 351
#define ALWAYSWITHIN 352
#define HOLDDURING 353
#define HOLDAFTER 354
#define ISVIOLATED 355
#define BOGUS 356
#define NAME 357
#define FUNCTION_SYMBOL 358
#define INTVAL 359
#define FLOATVAL 360
#define AT_TIME 361
#define PLUS 362
#define HYPHEN 363
#define DIV 364
#define MUL 365
#define UMINUS 366




/* Copy the first part of user declarations.  */
#line 17 "pddl+.yacc"

/*
Error reporting:
Intention is to provide error token on most bracket expressions,
so synchronisation can occur on next CLOSE_BRAC.
Hence error should be generated for innermost expression containing error.
Expressions which cause errors return a NULL values, and parser
always attempts to carry on.
This won't behave so well if CLOSE_BRAC is missing.

Naming conventions:
Generally, the names should be similar to the PDDL2.1 spec.
During development, they have also been based on older PDDL specs,
older PDDL+ and TIM parsers, and this shows in places.

All the names of fields in the semantic value type begin with t_
Corresponding categories in the grammar begin with c_
Corresponding classes have no prefix.

PDDL grammar       yacc grammar      type of corresponding semantic val.  

thing+             c_things          thing_list
(thing+)           c_thing_list      thing_list

*/

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <ctype.h>

// This is now copied locally to avoid relying on installation 
// of flex++.

//#include "FlexLexer.h"
//#include <FlexLexer.h>

#include "ptree.h"
#include "parse_error.h"

#define YYDEBUG 1 

int yyerror(char *);

#ifndef YY_
# if YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", ((char *)msgid))
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) ((char *) msgid)
# endif
#endif

extern int yylex();

using namespace VAL1_2;



/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* Enabling the token table.  */
#ifndef YYTOKEN_TABLE
# define YYTOKEN_TABLE 0
#endif

#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
#line 79 "pddl+.yacc"
{
    parse_category* t_parse_category;

    effect_lists* t_effect_lists;
    effect* t_effect;
    simple_effect* t_simple_effect;
    cond_effect*   t_cond_effect;
    forall_effect* t_forall_effect;
    timed_effect* t_timed_effect;

    quantifier t_quantifier;
    metric_spec*  t_metric;
    optimization t_optimization;

    symbol* t_symbol;
    var_symbol*   t_var_symbol;
    pddl_type*    t_type;
    pred_symbol*  t_pred_symbol;
    func_symbol*  t_func_symbol;
    const_symbol* t_const_symbol;

    parameter_symbol_list* t_parameter_symbol_list;
    var_symbol_list* t_var_symbol_list;
    const_symbol_list* t_const_symbol_list;
    pddl_type_list* t_type_list;

    proposition* t_proposition;
    pred_decl* t_pred_decl;
    pred_decl_list* t_pred_decl_list;
    func_decl* t_func_decl;
    func_decl_list* t_func_decl_list;

    goal* t_goal;
    con_goal * t_con_goal;
    goal_list* t_goal_list;

    func_term* t_func_term;
    assignment* t_assignment;
    expression* t_expression;
    num_expression* t_num_expression;
    assign_op t_assign_op;
    comparison_op t_comparison_op;

    structure_def* t_structure_def;
    structure_store* t_structure_store;

    action* t_action_def;
    event* t_event_def;
    process* t_process_def;
    durative_action* t_durative_action_def;
    derivation_rule* t_derivation_rule;

    problem* t_problem;
    length_spec* t_length_spec;

    domain* t_domain;    

    pddl_req_flag t_pddl_req_flag;

    plan* t_plan;
    plan_step* t_step;

    int ival;
    double fval;

    char* cp;
    int t_dummy;

    var_symbol_table * vtab;
}
/* Line 187 of yacc.c.  */
#line 451 "pddl+.cpp"
	YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif



/* Copy the second part of user declarations.  */


/* Line 216 of yacc.c.  */
#line 464 "pddl+.cpp"

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#elif (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
typedef signed char yytype_int8;
#else
typedef short int yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(e) ((void) (e))
#else
# define YYUSE(e) /* empty */
#endif

/* Identity function, used to suppress warnings about constant conditions.  */
#ifndef lint
# define YYID(n) (n)
#else
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static int
YYID (int i)
#else
static int
YYID (i)
    int i;
#endif
{
  return i;
}
#endif

#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#     ifndef _STDLIB_H
#      define _STDLIB_H 1
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (YYID (0))
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined _STDLIB_H \
       && ! ((defined YYMALLOC || defined malloc) \
	     && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef _STDLIB_H
#    define _STDLIB_H 1
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
	 || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss;
  YYSTYPE yyvs;
  };

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  YYSIZE_T yyi;				\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (YYID (0))
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack)					\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack, Stack, yysize);				\
	Stack = &yyptr->Stack;						\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (YYID (0))

#endif

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  17
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   946

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  112
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  124
/* YYNRULES -- Number of rules.  */
#define YYNRULES  340
/* YYNRULES -- Number of states.  */
#define YYNSTATES  789

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   366

#define YYTRANSLATE(YYX)						\
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    60,    61,    62,    63,    64,
      65,    66,    67,    68,    69,    70,    71,    72,    73,    74,
      75,    76,    77,    78,    79,    80,    81,    82,    83,    84,
      85,    86,    87,    88,    89,    90,    91,    92,    93,    94,
      95,    96,    97,    98,    99,   100,   101,   102,   103,   104,
     105,   106,   107,   108,   109,   110,   111
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const yytype_uint16 yyprhs[] =
{
       0,     0,     3,     5,     7,     9,    15,    20,    23,    26,
      29,    32,    35,    38,    40,    45,    50,    55,    58,    59,
      62,    64,    69,    73,    75,    77,    79,    81,    84,    85,
      91,    95,    98,    99,   101,   106,   111,   113,   117,   118,
     123,   128,   130,   133,   134,   137,   138,   143,   148,   150,
     153,   157,   158,   160,   162,   164,   166,   171,   173,   175,
     178,   179,   182,   183,   190,   193,   196,   199,   200,   205,
     208,   211,   214,   215,   217,   219,   221,   223,   225,   230,
     232,   234,   236,   238,   241,   244,   247,   248,   253,   258,
     263,   271,   277,   283,   285,   287,   290,   291,   296,   301,
     307,   313,   317,   323,   329,   333,   338,   346,   352,   354,
     357,   358,   363,   365,   367,   369,   371,   374,   377,   380,
     381,   387,   393,   399,   405,   411,   417,   423,   428,   431,
     432,   434,   437,   439,   441,   447,   453,   459,   465,   470,
     477,   487,   497,   499,   501,   503,   505,   508,   509,   514,
     516,   521,   523,   531,   537,   543,   549,   555,   561,   567,
     572,   578,   584,   590,   596,   598,   600,   606,   612,   614,
     616,   618,   623,   628,   630,   635,   640,   642,   644,   646,
     648,   650,   652,   654,   659,   667,   671,   674,   679,   685,
     690,   698,   700,   705,   711,   716,   724,   727,   729,   734,
     740,   742,   745,   747,   752,   760,   765,   770,   775,   781,
     786,   792,   798,   805,   812,   818,   820,   825,   830,   835,
     841,   849,   857,   863,   866,   868,   871,   873,   875,   877,
     882,   887,   892,   897,   902,   907,   912,   917,   922,   927,
     932,   935,   937,   939,   941,   943,   945,   947,   949,   955,
     968,   973,   986,   991,  1004,  1009,  1021,  1026,  1030,  1034,
    1035,  1037,  1042,  1045,  1046,  1051,  1056,  1061,  1067,  1072,
    1074,  1076,  1078,  1080,  1082,  1084,  1086,  1088,  1090,  1092,
    1094,  1096,  1098,  1100,  1102,  1104,  1106,  1108,  1110,  1112,
    1114,  1116,  1118,  1120,  1125,  1130,  1143,  1149,  1152,  1155,
    1158,  1161,  1164,  1167,  1170,  1171,  1176,  1181,  1183,  1188,
    1194,  1199,  1207,  1213,  1219,  1221,  1223,  1227,  1229,  1231,
    1233,  1238,  1242,  1246,  1250,  1254,  1258,  1260,  1263,  1265,
    1268,  1271,  1275,  1279,  1280,  1284,  1286,  1291,  1293,  1298,
    1300
};

/* YYRHS -- A `-1'-separated list of the rules' RHS.  */
static const yytype_int16 yyrhs[] =
{
     113,     0,    -1,   114,    -1,   218,    -1,   231,    -1,     3,
       7,   116,   115,     4,    -1,     3,     7,   116,     1,    -1,
     117,   115,    -1,   217,   115,    -1,   216,   115,    -1,   198,
     115,    -1,   199,   115,    -1,   200,   115,    -1,   202,    -1,
       3,     8,   102,     4,    -1,     3,     9,   118,     4,    -1,
       3,     9,     1,     4,    -1,   118,   215,    -1,    -1,   120,
     119,    -1,   120,    -1,     3,   121,   128,     4,    -1,     3,
       1,     4,    -1,   102,    -1,    69,    -1,   102,    -1,   102,
      -1,   124,   125,    -1,    -1,     3,   127,   128,     4,   126,
      -1,     3,     1,     4,    -1,   108,    90,    -1,    -1,   102,
      -1,   129,   108,   141,   128,    -1,   129,   108,   139,   128,
      -1,   129,    -1,    88,   135,   129,    -1,    -1,   132,   108,
     141,   130,    -1,   132,   108,   139,   130,    -1,   132,    -1,
     137,   131,    -1,    -1,   138,   132,    -1,    -1,   142,   108,
     141,   133,    -1,   142,   108,   139,   133,    -1,   142,    -1,
     134,   137,    -1,   134,    88,   136,    -1,    -1,   102,    -1,
     102,    -1,   102,    -1,   102,    -1,     3,    63,   143,     4,
      -1,   102,    -1,   102,    -1,   142,   140,    -1,    -1,   143,
     141,    -1,    -1,   144,     3,    69,   180,   179,     4,    -1,
     144,   173,    -1,   144,   172,    -1,   144,   145,    -1,    -1,
       3,   106,   144,     4,    -1,   148,   146,    -1,   175,   146,
      -1,   174,   146,    -1,    -1,   151,    -1,   171,    -1,   170,
      -1,   175,    -1,   174,    -1,     3,    55,   150,     4,    -1,
     149,    -1,   170,    -1,   171,    -1,   176,    -1,   150,   170,
      -1,   150,   171,    -1,   150,   176,    -1,    -1,     3,    55,
     146,     4,    -1,     3,    55,     1,     4,    -1,     3,    55,
     153,     4,    -1,     3,   193,     3,   128,     4,   152,     4,
      -1,     3,    61,   211,   152,     4,    -1,     3,    62,   190,
     156,     4,    -1,   154,    -1,   176,    -1,   153,   152,    -1,
      -1,     3,    52,   158,     4,    -1,     3,    53,   158,     4,
      -1,     3,    79,   180,   178,     4,    -1,     3,    80,   180,
     178,     4,    -1,     3,     1,     4,    -1,     3,    79,   180,
     178,     4,    -1,     3,    80,   180,   178,     4,    -1,     3,
       1,     4,    -1,     3,    55,   157,     4,    -1,     3,   193,
       3,   128,     4,   156,     4,    -1,     3,    62,   190,   156,
       4,    -1,   155,    -1,   157,   156,    -1,    -1,     3,    55,
     160,     4,    -1,   159,    -1,   170,    -1,   171,    -1,   161,
      -1,   160,   170,    -1,   160,   171,    -1,   160,   161,    -1,
      -1,     3,    83,   180,   164,     4,    -1,     3,    79,   180,
     164,     4,    -1,     3,    80,   180,   164,     4,    -1,     3,
      81,   180,   164,     4,    -1,     3,    82,   180,   164,     4,
      -1,     3,    79,   180,   178,     4,    -1,     3,    80,   180,
     178,     4,    -1,     3,    55,   163,     4,    -1,   163,   162,
      -1,    -1,   165,    -1,    88,    77,    -1,   179,    -1,   180,
      -1,     3,   107,   164,   164,     4,    -1,     3,   108,   164,
     164,     4,    -1,     3,   110,   164,   164,     4,    -1,     3,
     109,   164,   164,     4,    -1,     3,    55,   169,     4,    -1,
       3,   167,    88,    77,   168,     4,    -1,     3,    52,     3,
     167,    88,    77,   168,     4,     4,    -1,     3,    53,     3,
     167,    88,    77,   168,     4,     4,    -1,    87,    -1,    85,
      -1,    69,    -1,   177,    -1,   169,   166,    -1,    -1,     3,
      60,   195,     4,    -1,   195,    -1,     3,    60,   197,     4,
      -1,   197,    -1,     3,   193,     3,   128,     4,   147,     4,
      -1,     3,    61,   190,   146,     4,    -1,     3,    83,   180,
     177,     4,    -1,     3,    79,   180,   177,     4,    -1,     3,
      80,   180,   177,     4,    -1,     3,    81,   180,   177,     4,
      -1,     3,    82,   180,   177,     4,    -1,     3,   108,   177,
       4,    -1,     3,   107,   177,   177,     4,    -1,     3,   108,
     177,   177,     4,    -1,     3,   110,   177,   177,     4,    -1,
       3,   109,   177,   177,     4,    -1,   179,    -1,   180,    -1,
       3,   110,    76,   177,     4,    -1,     3,   110,   177,    76,
       4,    -1,    76,    -1,   104,    -1,   105,    -1,     3,   103,
     134,     4,    -1,     3,   102,   134,     4,    -1,   103,    -1,
       3,   103,   134,     4,    -1,     3,   102,   134,     4,    -1,
     103,    -1,    84,    -1,    85,    -1,    86,    -1,    87,    -1,
      69,    -1,   187,    -1,     3,    55,   191,     4,    -1,     3,
     193,     3,   128,     4,   183,     4,    -1,     3,    55,     4,
      -1,     3,     4,    -1,     3,    44,   189,     4,    -1,     3,
      44,   102,   189,     4,    -1,     3,    55,   186,     4,    -1,
       3,   193,     3,   128,     4,   185,     4,    -1,   189,    -1,
       3,    44,   189,     4,    -1,     3,    44,   102,   189,     4,
      -1,     3,    55,   186,     4,    -1,     3,   193,     3,   128,
       4,   185,     4,    -1,   186,   184,    -1,   184,    -1,     3,
      44,   190,     4,    -1,     3,    44,   102,   190,     4,    -1,
     190,    -1,   188,   189,    -1,   189,    -1,     3,    55,   188,
       4,    -1,     3,   193,     3,   128,     4,   189,     4,    -1,
       3,    53,   190,     4,    -1,     3,    91,   190,     4,    -1,
       3,    92,   190,     4,    -1,     3,    93,   179,   190,     4,
      -1,     3,    94,   190,     4,    -1,     3,    95,   190,   190,
       4,    -1,     3,    96,   190,   190,     4,    -1,     3,    97,
     179,   190,   190,     4,    -1,     3,    98,   179,   179,   190,
       4,    -1,     3,    99,   179,   190,     4,    -1,   195,    -1,
       3,    60,   190,     4,    -1,     3,    55,   192,     4,    -1,
       3,    56,   192,     4,    -1,     3,    59,   190,   190,     4,
      -1,     3,   193,     3,   128,     4,   190,     4,    -1,     3,
     194,     3,   128,     4,   190,     4,    -1,     3,   182,   177,
     177,     4,    -1,   191,   183,    -1,   183,    -1,   192,   190,
      -1,   190,    -1,    58,    -1,    57,    -1,     3,   122,   134,
       4,    -1,     3,   122,   128,     4,    -1,     3,   123,   134,
       4,    -1,     3,    38,   119,     4,    -1,     3,    38,     1,
       4,    -1,     3,    39,   124,     4,    -1,     3,    39,     1,
       4,    -1,     3,    31,   189,     4,    -1,     3,    31,     1,
       4,    -1,     3,    31,   184,     4,    -1,     3,    31,     1,
       4,    -1,   202,   203,    -1,   203,    -1,   206,    -1,   207,
      -1,   208,    -1,   209,    -1,   205,    -1,    36,    -1,     3,
     204,   196,   190,     4,    -1,     3,    32,   102,   214,     3,
     128,     4,    42,   183,    47,   147,     4,    -1,     3,    32,
       1,     4,    -1,     3,    34,   102,   214,     3,   128,     4,
      42,   190,    47,   147,     4,    -1,     3,    34,     1,     4,
      -1,     3,    33,   102,   214,     3,   128,     4,    42,   190,
      47,   162,     4,    -1,     3,    33,     1,     4,    -1,     3,
      35,   102,   214,     3,   128,     4,    51,   166,   210,     4,
      -1,     3,    35,     1,     4,    -1,   210,    47,   152,    -1,
     210,    43,   211,    -1,    -1,   213,    -1,     3,    55,   212,
       4,    -1,   212,   211,    -1,    -1,     3,    52,   190,     4,
      -1,     3,    53,   190,     4,    -1,     3,    54,   190,     4,
      -1,     3,    44,   102,   213,     4,    -1,     3,    44,   213,
       4,    -1,    41,    -1,    10,    -1,    11,    -1,    14,    -1,
      13,    -1,    15,    -1,    16,    -1,    17,    -1,    19,    -1,
      20,    -1,    25,    -1,    24,    -1,    23,    -1,    21,    -1,
      22,    -1,    12,    -1,    18,    -1,    26,    -1,    27,    -1,
      28,    -1,    29,    -1,    30,    -1,    31,    -1,   102,    -1,
       3,    37,   130,     4,    -1,     3,    40,   133,     4,    -1,
       3,     7,     3,    64,   102,     4,     3,    65,   102,     4,
     219,     4,    -1,     3,     7,     3,    64,     1,    -1,   117,
     219,    -1,   220,   219,    -1,   221,   219,    -1,   223,   219,
      -1,   201,   219,    -1,   224,   219,    -1,   225,   219,    -1,
      -1,     3,    67,   130,     4,    -1,     3,    66,   144,     4,
      -1,    68,    -1,     3,   222,   183,     4,    -1,     3,    73,
     226,   227,     4,    -1,     3,    73,     1,     4,    -1,     3,
      70,    71,   104,    72,   104,     4,    -1,     3,    70,    71,
     104,     4,    -1,     3,    70,    72,   104,     4,    -1,    74,
      -1,    75,    -1,     3,   228,     4,    -1,   181,    -1,   179,
      -1,    78,    -1,     3,   100,   102,     4,    -1,     3,    78,
       4,    -1,   107,   227,   229,    -1,   108,   227,   227,    -1,
     110,   227,   230,    -1,   109,   227,   227,    -1,   227,    -1,
     227,   229,    -1,   227,    -1,   227,   230,    -1,   232,   231,
      -1,    24,   105,   231,    -1,    24,   104,   231,    -1,    -1,
     235,    89,   233,    -1,   233,    -1,   234,     5,   235,     6,
      -1,   234,    -1,     3,   102,   131,     4,    -1,   105,    -1,
     104,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,   252,   252,   253,   254,   258,   264,   271,   272,   273,
     274,   276,   278,   280,   283,   288,   295,   302,   303,   308,
     310,   315,   317,   325,   333,   335,   343,   348,   350,   354,
     356,   363,   363,   366,   379,   388,   397,   409,   411,   417,
     426,   436,   441,   442,   446,   447,   455,   462,   471,   477,
     479,   481,   488,   494,   498,   502,   506,   511,   518,   523,
     525,   529,   531,   535,   548,   550,   552,   555,   559,   565,
     566,   568,   570,   579,   580,   581,   582,   583,   587,   588,
     592,   594,   596,   603,   604,   605,   607,   611,   613,   621,
     623,   631,   636,   641,   644,   651,   652,   656,   658,   660,
     664,   668,   674,   678,   682,   688,   690,   698,   703,   709,
     710,   714,   715,   719,   721,   723,   730,   731,   732,   734,
     739,   741,   743,   745,   747,   752,   758,   764,   769,   770,
     774,   775,   777,   778,   782,   784,   786,   788,   793,   795,
     798,   801,   807,   808,   809,   817,   821,   824,   828,   833,
     840,   845,   850,   855,   860,   862,   864,   866,   868,   873,
     875,   877,   879,   881,   883,   884,   888,   890,   892,   898,
     899,   902,   905,   907,   925,   927,   929,   935,   936,   937,
     938,   939,   951,   958,   960,   964,   965,   969,   971,   973,
     975,   979,   984,   986,   988,   990,   997,   999,  1004,  1006,
    1010,  1015,  1017,  1022,  1024,  1027,  1029,  1031,  1033,  1035,
    1037,  1039,  1041,  1043,  1045,  1050,  1052,  1056,  1058,  1061,
    1064,  1067,  1070,  1076,  1078,  1083,  1085,  1095,  1102,  1109,
    1114,  1119,  1124,  1126,  1133,  1135,  1142,  1144,  1151,  1153,
    1160,  1161,  1165,  1166,  1167,  1168,  1169,  1173,  1179,  1188,
    1199,  1206,  1217,  1223,  1233,  1239,  1254,  1261,  1263,  1265,
    1269,  1271,  1276,  1279,  1283,  1285,  1287,  1289,  1294,  1299,
    1304,  1305,  1307,  1308,  1310,  1312,  1313,  1314,  1315,  1316,
    1318,  1321,  1324,  1325,  1327,  1336,  1339,  1342,  1344,  1346,
    1348,  1350,  1352,  1358,  1362,  1367,  1379,  1386,  1387,  1388,
    1389,  1390,  1392,  1393,  1394,  1397,  1400,  1403,  1406,  1410,
    1412,  1419,  1422,  1426,  1433,  1434,  1439,  1440,  1441,  1442,
    1443,  1445,  1449,  1450,  1451,  1452,  1456,  1457,  1462,  1463,
    1469,  1472,  1474,  1477,  1481,  1485,  1491,  1495,  1501,  1509,
    1510
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "OPEN_BRAC", "CLOSE_BRAC", "OPEN_SQ",
  "CLOSE_SQ", "DEFINE", "PDDLDOMAIN", "REQS", "EQUALITY", "STRIPS", "ADL",
  "NEGATIVE_PRECONDITIONS", "TYPING", "DISJUNCTIVE_PRECONDS", "EXT_PRECS",
  "UNIV_PRECS", "QUANT_PRECS", "COND_EFFS", "FLUENTS", "OBJECTFLUENTS",
  "NUMERICFLUENTS", "ACTIONCOSTS", "TIME", "DURATIVE_ACTIONS",
  "DURATION_INEQUALITIES", "CONTINUOUS_EFFECTS", "DERIVED_PREDICATES",
  "TIMED_INITIAL_LITERALS", "PREFERENCES", "CONSTRAINTS", "ACTION",
  "PROCESS", "EVENT", "DURATIVE_ACTION", "DERIVED", "CONSTANTS", "PREDS",
  "FUNCTIONS", "TYPES", "ARGS", "PRE", "CONDITION", "PREFERENCE",
  "START_PRE", "END_PRE", "EFFECTS", "INITIAL_EFFECT", "FINAL_EFFECT",
  "INVARIANT", "DURATION", "AT_START", "AT_END", "OVER_ALL", "AND", "OR",
  "EXISTS", "FORALL", "IMPLY", "NOT", "WHEN", "WHENEVER", "EITHER",
  "PROBLEM", "FORDOMAIN", "INITIALLY", "OBJECTS", "GOALS", "EQ", "LENGTH",
  "SERIAL", "PARALLEL", "METRIC", "MINIMIZE", "MAXIMIZE", "HASHT",
  "DURATION_VAR", "TOTAL_TIME", "INCREASE", "DECREASE", "SCALE_UP",
  "SCALE_DOWN", "ASSIGN", "GREATER", "GREATEQ", "LESS", "LESSEQ", "Q",
  "COLON", "NUMBER", "ALWAYS", "SOMETIME", "WITHIN", "ATMOSTONCE",
  "SOMETIMEAFTER", "SOMETIMEBEFORE", "ALWAYSWITHIN", "HOLDDURING",
  "HOLDAFTER", "ISVIOLATED", "BOGUS", "NAME", "FUNCTION_SYMBOL", "INTVAL",
  "FLOATVAL", "AT_TIME", "PLUS", "HYPHEN", "DIV", "MUL", "UMINUS",
  "$accept", "mystartsymbol", "c_domain", "c_preamble", "c_domain_name",
  "c_domain_require_def", "c_reqs", "c_pred_decls", "c_pred_decl",
  "c_new_pred_symbol", "c_pred_symbol", "c_init_pred_symbol",
  "c_func_decls", "c_func_decl", "c_ntype", "c_new_func_symbol",
  "c_typed_var_list", "c_var_symbol_list", "c_typed_consts",
  "c_const_symbols", "c_new_const_symbols", "c_typed_types",
  "c_parameter_symbols", "c_declaration_var_symbol", "c_var_symbol",
  "c_const_symbol", "c_new_const_symbol", "c_either_type",
  "c_new_primitive_type", "c_primitive_type", "c_new_primitive_types",
  "c_primitive_types", "c_init_els", "c_timed_initial_literal",
  "c_effects", "c_effect", "c_a_effect", "c_p_effect", "c_p_effects",
  "c_conj_effect", "c_da_effect", "c_da_effects", "c_timed_effect",
  "c_cts_only_timed_effect", "c_da_cts_only_effect",
  "c_da_cts_only_effects", "c_a_effect_da", "c_p_effect_da",
  "c_p_effects_da", "c_f_assign_da", "c_proc_effect", "c_proc_effects",
  "c_f_exp_da", "c_binary_expr_da", "c_duration_constraint", "c_d_op",
  "c_d_value", "c_duration_constraints", "c_neg_simple_effect",
  "c_pos_simple_effect", "c_init_neg_simple_effect",
  "c_init_pos_simple_effect", "c_forall_effect", "c_cond_effect",
  "c_assignment", "c_f_exp", "c_f_exp_t", "c_number", "c_f_head",
  "c_ground_f_head", "c_comparison_op", "c_pre_goal_descriptor",
  "c_pref_con_goal", "c_pref_goal", "c_pref_con_goal_list",
  "c_pref_goal_descriptor", "c_constraint_goal_list", "c_constraint_goal",
  "c_goal_descriptor", "c_pre_goal_descriptor_list", "c_goal_list",
  "c_forall", "c_exists", "c_proposition", "c_derived_proposition",
  "c_init_proposition", "c_predicates", "c_functions_def",
  "c_constraints_def", "c_constraints_probdef", "c_structure_defs",
  "c_structure_def", "c_rule_head", "c_derivation_rule", "c_action_def",
  "c_event_def", "c_process_def", "c_durative_action_def", "c_da_def_body",
  "c_da_gd", "c_da_gds", "c_timed_gd", "c_args_head", "c_require_key",
  "c_domain_constants", "c_type_names", "c_problem", "c_problem_body",
  "c_objects", "c_initial_state", "c_goals", "c_goal_spec",
  "c_metric_spec", "c_length_spec", "c_optimization", "c_ground_f_exp",
  "c_binary_ground_f_exp", "c_binary_ground_f_pexps",
  "c_binary_ground_f_mexps", "c_plan", "c_step_t_d", "c_step_d", "c_step",
  "c_float", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
   token YYLEX-NUM.  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285,   286,   287,   288,   289,   290,   291,   292,   293,   294,
     295,   296,   297,   298,   299,   300,   301,   302,   303,   304,
     305,   306,   307,   308,   309,   310,   311,   312,   313,   314,
     315,   316,   317,   318,   319,   320,   321,   322,   323,   324,
     325,   326,   327,   328,   329,   330,   331,   332,   333,   334,
     335,   336,   337,   338,   339,   340,   341,   342,   343,   344,
     345,   346,   347,   348,   349,   350,   351,   352,   353,   354,
     355,   356,   357,   358,   359,   360,   361,   362,   363,   364,
     365,   366
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,   112,   113,   113,   113,   114,   114,   115,   115,   115,
     115,   115,   115,   115,   116,   117,   117,   118,   118,   119,
     119,   120,   120,   121,   122,   122,   123,   124,   124,   125,
     125,   126,   126,   127,   128,   128,   128,   129,   129,   130,
     130,   130,   131,   131,   132,   132,   133,   133,   133,   134,
     134,   134,   135,   136,   137,   138,   139,   140,   141,   142,
     142,   143,   143,   144,   144,   144,   144,   144,   145,   146,
     146,   146,   146,   147,   147,   147,   147,   147,   148,   148,
     149,   149,   149,   150,   150,   150,   150,   151,   151,   152,
     152,   152,   152,   152,   152,   153,   153,   154,   154,   154,
     154,   154,   155,   155,   155,   156,   156,   156,   156,   157,
     157,   158,   158,   159,   159,   159,   160,   160,   160,   160,
     161,   161,   161,   161,   161,   162,   162,   162,   163,   163,
     164,   164,   164,   164,   165,   165,   165,   165,   166,   166,
     166,   166,   167,   167,   167,   168,   169,   169,   170,   171,
     172,   173,   174,   175,   176,   176,   176,   176,   176,   177,
     177,   177,   177,   177,   177,   177,   178,   178,   178,   179,
     179,   180,   180,   180,   181,   181,   181,   182,   182,   182,
     182,   182,   183,   183,   183,   183,   183,   184,   184,   184,
     184,   184,   185,   185,   185,   185,   186,   186,   187,   187,
     187,   188,   188,   189,   189,   189,   189,   189,   189,   189,
     189,   189,   189,   189,   189,   190,   190,   190,   190,   190,
     190,   190,   190,   191,   191,   192,   192,   193,   194,   195,
     196,   197,   198,   198,   199,   199,   200,   200,   201,   201,
     202,   202,   203,   203,   203,   203,   203,   204,   205,   206,
     206,   207,   207,   208,   208,   209,   209,   210,   210,   210,
     211,   211,   212,   212,   213,   213,   213,   213,   213,   214,
     215,   215,   215,   215,   215,   215,   215,   215,   215,   215,
     215,   215,   215,   215,   215,   215,   215,   215,   215,   215,
     215,   215,   215,   216,   217,   218,   218,   219,   219,   219,
     219,   219,   219,   219,   219,   220,   221,   222,   223,   224,
     224,   225,   225,   225,   226,   226,   227,   227,   227,   227,
     227,   227,   228,   228,   228,   228,   229,   229,   230,   230,
     231,   231,   231,   231,   232,   232,   233,   233,   234,   235,
     235
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     1,     1,     1,     5,     4,     2,     2,     2,
       2,     2,     2,     1,     4,     4,     4,     2,     0,     2,
       1,     4,     3,     1,     1,     1,     1,     2,     0,     5,
       3,     2,     0,     1,     4,     4,     1,     3,     0,     4,
       4,     1,     2,     0,     2,     0,     4,     4,     1,     2,
       3,     0,     1,     1,     1,     1,     4,     1,     1,     2,
       0,     2,     0,     6,     2,     2,     2,     0,     4,     2,
       2,     2,     0,     1,     1,     1,     1,     1,     4,     1,
       1,     1,     1,     2,     2,     2,     0,     4,     4,     4,
       7,     5,     5,     1,     1,     2,     0,     4,     4,     5,
       5,     3,     5,     5,     3,     4,     7,     5,     1,     2,
       0,     4,     1,     1,     1,     1,     2,     2,     2,     0,
       5,     5,     5,     5,     5,     5,     5,     4,     2,     0,
       1,     2,     1,     1,     5,     5,     5,     5,     4,     6,
       9,     9,     1,     1,     1,     1,     2,     0,     4,     1,
       4,     1,     7,     5,     5,     5,     5,     5,     5,     4,
       5,     5,     5,     5,     1,     1,     5,     5,     1,     1,
       1,     4,     4,     1,     4,     4,     1,     1,     1,     1,
       1,     1,     1,     4,     7,     3,     2,     4,     5,     4,
       7,     1,     4,     5,     4,     7,     2,     1,     4,     5,
       1,     2,     1,     4,     7,     4,     4,     4,     5,     4,
       5,     5,     6,     6,     5,     1,     4,     4,     4,     5,
       7,     7,     5,     2,     1,     2,     1,     1,     1,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       2,     1,     1,     1,     1,     1,     1,     1,     5,    12,
       4,    12,     4,    12,     4,    11,     4,     3,     3,     0,
       1,     4,     2,     0,     4,     4,     4,     5,     4,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     4,     4,    12,     5,     2,     2,     2,
       2,     2,     2,     2,     0,     4,     4,     1,     4,     5,
       4,     7,     5,     5,     1,     1,     3,     1,     1,     1,
       4,     3,     3,     3,     3,     3,     1,     2,     1,     2,
       2,     3,     3,     0,     3,     1,     4,     1,     4,     1,
       1
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
   STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint16 yydefact[] =
{
     333,     0,     0,   340,   339,     0,     2,     3,     4,   333,
     335,   337,     0,     0,    43,   333,   333,     1,     0,   330,
       0,     0,     0,     0,    54,     0,    43,   332,   331,     0,
     334,     0,     0,     6,     0,     0,     0,     0,     0,     0,
      13,   241,   246,   242,   243,   244,   245,     0,     0,   338,
      42,   336,     0,   296,     0,     0,     0,     0,     0,     0,
       0,   247,    45,     0,     0,    60,     0,     5,     7,    10,
      11,    12,     0,   240,     9,     8,    14,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,    55,     0,    41,    45,     0,     0,     0,    20,     0,
       0,     0,    48,     0,     0,     0,    16,    15,   270,   271,
     284,   273,   272,   274,   275,   276,   285,   277,   278,   282,
     283,   281,   280,   279,   286,   287,   288,   289,   290,   291,
     292,    17,   237,     0,     0,   227,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,   236,   250,   269,     0,
     254,     0,   252,     0,   256,     0,   293,     0,    44,   233,
       0,    23,    38,   232,    19,   235,     0,   234,    27,   294,
      57,     0,    59,    24,    25,    38,     0,     0,   215,     0,
       0,     0,   202,     0,     0,   169,   170,     0,     0,     0,
       0,     0,     0,     0,    38,    38,    38,    38,    38,     0,
      58,    45,    45,    22,     0,     0,    36,     0,    33,    38,
      60,    60,     0,     0,     0,   228,     0,     0,   181,   177,
     178,   179,   180,    51,     0,     0,     0,   248,     0,   205,
     203,   201,   206,   207,     0,   209,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,    62,    40,    39,    52,
      38,    21,     0,    30,     0,    47,    46,   230,   226,     0,
       0,     0,     0,     0,     0,   173,     0,   164,   165,    38,
      38,   304,   208,   210,   211,     0,     0,   214,     0,     0,
       0,     0,     0,     0,    37,    38,    38,    32,   217,   225,
     218,     0,   216,   229,     0,    49,    51,    51,     0,     0,
       0,     0,     0,     0,     0,     0,   304,   304,     0,   304,
     304,   304,   304,   304,   212,   213,     0,     0,     0,     0,
       0,    56,    61,    35,    34,     0,    29,   219,    53,    50,
       0,     0,     0,     0,     0,     0,   222,     0,     0,     0,
      67,    45,   307,     0,     0,     0,   297,   301,   295,   298,
     299,   300,   302,   303,   204,     0,     0,   182,   200,     0,
       0,     0,   259,    31,   172,   171,     0,   159,     0,     0,
       0,     0,     0,     0,     0,     0,   191,     0,     0,     0,
       0,     0,   314,   315,     0,     0,   186,     0,     0,     0,
       0,     0,     0,     0,     0,   147,   144,   143,   142,     0,
       0,   160,   161,   163,   162,   220,   221,   239,     0,     0,
       0,   238,     0,   306,    66,    65,    64,   151,   305,     0,
       0,   310,     0,   319,   176,   318,   317,     0,   308,     0,
       0,   185,   224,   200,     0,    38,     0,     0,    73,    75,
      74,    77,    76,   149,     0,     0,     0,     0,     0,     0,
       0,   255,     0,     0,     0,     0,   197,     0,   191,    38,
       0,     0,    26,    67,    51,   312,     0,   313,     0,     0,
      51,    51,     0,     0,     0,     0,     0,   309,     0,   198,
     183,   223,     0,     0,     0,     0,     0,   249,   129,     0,
       0,   253,   251,     0,     0,   138,   146,     0,     0,   258,
     260,     0,   257,    93,    94,     0,   187,   189,   196,     0,
       0,     0,     0,     0,     0,     0,     0,   321,     0,     0,
       0,     0,     0,     0,     0,   316,   199,     0,     0,     0,
       0,    72,    79,    80,    81,    72,    72,    82,     0,     0,
      72,    38,     0,     0,     0,     0,     0,     0,   145,     0,
       0,     0,     0,   263,     0,     0,     0,    96,     0,     0,
       0,     0,     0,     0,     0,     0,   188,     0,   150,     0,
      68,   231,   311,   320,   175,   174,   326,   322,   323,   325,
     328,   324,     0,     0,    88,    86,     0,     0,    87,    69,
      71,    70,   148,     0,     0,   127,   128,     0,   168,     0,
       0,     0,     0,   139,     0,     0,     0,     0,     0,     0,
       0,   101,     0,     0,   112,   115,   113,   114,     0,     0,
       0,     0,     0,     0,     0,     0,     0,    38,     0,     0,
      63,   327,   329,   184,     0,     0,     0,   153,     0,     0,
     125,   126,     0,     0,     0,   268,   264,   265,   266,   261,
     262,   119,     0,     0,     0,     0,     0,    97,    98,    89,
      95,     0,     0,   108,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,   190,     0,    78,
      83,    84,    85,     0,     0,     0,     0,     0,   267,     0,
       0,     0,     0,     0,     0,    91,     0,   110,     0,     0,
       0,     0,    92,     0,   155,    99,   156,   100,   157,   158,
     154,     0,     0,     0,     0,    38,   152,     0,     0,   140,
     141,     0,   111,   118,   116,   117,     0,     0,     0,   130,
     132,   133,     0,     0,     0,     0,   104,     0,     0,     0,
       0,    38,     0,     0,     0,   192,   194,     0,   166,   167,
       0,     0,     0,     0,   131,   121,   122,   123,   124,   120,
     105,   109,     0,     0,     0,     0,    90,   193,     0,     0,
       0,     0,     0,   107,   102,   103,     0,     0,     0,     0,
       0,     0,     0,   195,   134,   135,   137,   136,   106
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     5,     6,    35,    23,   306,    79,    97,    98,   162,
     223,   464,   100,   168,   326,   209,   205,   206,    92,    25,
      93,   101,   263,   250,   329,   295,    94,   201,   172,   202,
     102,   283,   377,   414,   530,   437,   531,   532,   634,   438,
     502,   619,   503,   663,   664,   737,   613,   614,   689,   615,
     445,   542,   728,   729,   362,   399,   547,   449,   533,   534,
     415,   416,   535,   536,   537,   548,   599,   267,   268,   426,
     224,   356,   456,   629,   457,   357,   181,   316,   358,   434,
     259,   486,   226,   178,   104,   417,    37,    38,    39,   307,
      40,    41,    66,    42,    43,    44,    45,    46,   400,   499,
     610,   500,   149,   131,    47,    48,     7,   308,   309,   310,
     345,   311,   312,   313,   384,   576,   476,   577,   581,     8,
       9,    10,    11,    12
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -531
static const yytype_int16 yypact[] =
{
      30,    22,   199,  -531,  -531,    74,  -531,  -531,  -531,    37,
    -531,    27,   -10,   105,   107,    37,    37,  -531,   119,  -531,
     250,   226,    43,    66,  -531,   242,   107,  -531,  -531,   248,
    -531,   156,    64,  -531,   540,   261,   265,   265,   265,   265,
     281,  -531,  -531,  -531,  -531,  -531,  -531,   265,   265,  -531,
    -531,  -531,   291,  -531,   294,   488,    96,    65,    69,    70,
      83,  -531,   198,   286,   277,  -531,   305,  -531,  -531,  -531,
    -531,  -531,   613,  -531,  -531,  -531,  -531,   321,   325,   649,
     336,   648,   339,   373,   356,   395,   356,   411,   356,   424,
     356,  -531,   443,   345,   198,   463,    95,   473,   478,   482,
     330,   522,   147,   -16,   485,   465,  -531,  -531,  -531,  -531,
    -531,  -531,  -531,  -531,  -531,  -531,  -531,  -531,  -531,  -531,
    -531,  -531,  -531,  -531,  -531,  -531,  -531,  -531,  -531,  -531,
    -531,  -531,  -531,   485,   532,  -531,   485,   485,   270,   485,
     485,   485,   270,   270,   270,   541,  -531,  -531,  -531,   548,
    -531,   584,  -531,   594,  -531,   596,  -531,    54,  -531,  -531,
     600,  -531,   524,  -531,  -531,  -531,   100,  -531,  -531,  -531,
    -531,    54,  -531,  -531,  -531,   524,   309,   618,  -531,   515,
     625,   378,  -531,   626,   627,  -531,  -531,   485,   646,   485,
     485,   485,   270,   485,   524,   524,   524,   524,   524,   589,
    -531,   198,   198,  -531,   549,   650,   547,   653,  -531,   524,
    -531,  -531,   678,   485,   485,  -531,   485,   485,   151,  -531,
    -531,  -531,  -531,  -531,    49,   680,   681,  -531,   682,  -531,
    -531,  -531,  -531,  -531,   683,  -531,   684,   685,   485,   485,
     694,   695,   696,   698,   700,   703,  -531,  -531,  -531,  -531,
     524,  -531,    54,  -531,   704,  -531,  -531,  -531,  -531,   399,
     410,   485,   705,   191,   456,  -531,    49,  -531,  -531,   524,
     524,   690,  -531,  -531,  -531,   706,   707,  -531,   532,   643,
     670,   671,   663,   195,  -531,   524,   524,   612,  -531,  -531,
    -531,   717,  -531,  -531,   620,  -531,  -531,  -531,    49,    49,
      49,    49,   719,   720,   721,   290,   690,   690,   722,   690,
     690,   690,   690,   690,  -531,  -531,   723,   734,   485,   485,
     746,  -531,  -531,  -531,  -531,   660,  -531,  -531,  -531,  -531,
     200,   229,    49,    46,    49,    49,  -531,   485,   485,   343,
    -531,   198,  -531,   355,   189,   734,  -531,  -531,  -531,  -531,
    -531,  -531,  -531,  -531,  -531,   483,   709,  -531,  -531,   713,
     714,   369,  -531,  -531,  -531,  -531,   748,  -531,   750,   774,
     777,   778,   779,   784,   545,   785,  -531,   441,   789,   697,
     699,   790,  -531,  -531,    60,   796,  -531,    59,   448,   799,
     801,   802,   801,   803,   804,  -531,  -531,  -531,  -531,   724,
     258,  -531,  -531,  -531,  -531,  -531,  -531,  -531,   103,   805,
     806,  -531,   340,  -531,  -531,  -531,  -531,  -531,  -531,    32,
     807,  -531,   677,  -531,  -531,  -531,  -531,   809,  -531,   485,
     810,  -531,  -531,  -531,   469,   524,   190,   811,  -531,  -531,
    -531,  -531,  -531,  -531,     3,   812,   813,   153,   153,   472,
     733,  -531,   815,   816,   532,   817,  -531,   487,  -531,   524,
     819,    44,  -531,  -531,  -531,  -531,   716,  -531,   820,   725,
    -531,  -531,    60,    60,    60,    60,   821,  -531,   822,  -531,
    -531,  -531,   824,   358,   826,   485,   827,  -531,  -531,    44,
      44,  -531,  -531,   735,   743,  -531,  -531,    49,   493,  -531,
    -531,   379,  -531,  -531,  -531,   828,  -531,  -531,  -531,   829,
     732,   831,   333,   270,   490,   230,   832,  -531,   833,   237,
     240,    60,    60,    60,    60,  -531,  -531,   734,   834,   636,
     835,   837,  -531,  -531,  -531,   837,   837,  -531,   -16,   838,
     837,   524,   492,    14,    14,   764,   766,   840,  -531,   140,
     485,   485,   485,  -531,   841,   843,   843,  -531,   815,   485,
      44,    44,    44,    44,    44,   844,  -531,   845,  -531,   846,
    -531,  -531,  -531,  -531,  -531,  -531,    60,  -531,  -531,  -531,
      60,  -531,   847,   778,  -531,  -531,    44,    44,  -531,  -531,
    -531,  -531,  -531,   848,   849,  -531,  -531,   739,  -531,   850,
     851,    49,    49,  -531,   297,   853,   854,   855,   856,   857,
     521,  -531,   526,   858,  -531,  -531,  -531,  -531,   859,   525,
     816,   861,    72,    72,    49,    49,    49,   524,   637,   862,
    -531,  -531,  -531,  -531,   530,    49,    49,  -531,   801,   110,
    -531,  -531,   863,   864,   865,  -531,  -531,  -531,  -531,  -531,
    -531,  -531,    44,    44,    44,    44,    44,  -531,  -531,  -531,
    -531,   866,   268,  -531,   867,   656,   868,   869,   870,   871,
     872,   873,   874,   875,   141,   805,   877,  -531,   688,  -531,
    -531,  -531,  -531,   878,    49,   781,   879,   880,  -531,   557,
     120,   120,   120,   120,   120,  -531,   881,  -531,   485,    44,
      44,   883,  -531,   110,  -531,  -531,  -531,  -531,  -531,  -531,
    -531,   816,   532,   884,   579,   524,  -531,   885,   886,  -531,
    -531,   693,  -531,  -531,  -531,  -531,   689,   788,   887,  -531,
    -531,  -531,   888,   889,   890,   891,  -531,   588,   861,    14,
      14,   524,   127,   892,   893,  -531,  -531,   894,  -531,  -531,
     120,   120,   120,   120,  -531,  -531,  -531,  -531,  -531,  -531,
    -531,  -531,   895,   896,   897,   898,  -531,  -531,   845,   120,
     120,   120,   120,  -531,  -531,  -531,   861,   899,   900,   901,
     902,   903,   904,  -531,  -531,  -531,  -531,  -531,  -531
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -531,  -531,  -531,   484,  -531,   432,  -531,   783,  -531,  -531,
     808,  -531,  -531,  -531,  -531,  -531,  -153,   659,  -181,   905,
     793,   383,  -270,  -531,  -531,    50,  -531,    39,  -531,  -152,
    -531,  -531,   447,  -531,  -463,  -378,  -531,  -531,  -531,  -531,
    -474,  -531,  -531,  -531,  -510,  -531,   357,  -531,  -531,   223,
     372,  -531,  -287,  -531,   466,   154,     9,  -531,  -382,  -367,
    -531,  -531,  -381,  -377,  -441,  -206,  -520,  -137,  -380,  -531,
    -531,  -329,  -335,   148,   243,  -531,  -531,   -56,  -102,  -531,
     708,   -78,  -531,  -344,  -531,   457,  -531,  -531,  -531,  -531,
    -531,   906,  -531,  -531,  -531,  -531,  -531,  -531,  -531,  -530,
    -531,  -387,   322,  -531,  -531,  -531,  -531,   314,  -531,  -531,
    -531,  -531,  -531,  -531,  -531,  -354,  -531,   344,   341,   300,
    -531,   907,  -531,   909
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If zero, do what YYDEFACT says.
   If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -73
static const yytype_int16 yytable[] =
{
      82,   187,   177,   145,   375,   191,   192,   193,   439,   441,
     439,   441,   504,   442,   446,   442,   385,   597,   266,   211,
     247,   248,   212,   440,   600,   440,   330,   331,   620,    13,
     427,   180,    20,     1,   183,   184,   465,   188,   189,   190,
      18,   241,   242,   243,   244,   245,   443,   512,   443,   264,
     367,    31,   264,   173,     2,   239,   254,   199,   488,   432,
     302,     2,   176,   422,    26,    53,    83,    33,   589,    34,
      85,    87,   590,   591,    17,   665,    26,   593,   182,    21,
     650,   513,   489,   490,    89,   234,   174,   236,   237,   238,
     598,   240,   332,   333,   334,   335,   160,    80,   225,    81,
     286,   207,   667,   669,   466,   481,    81,    32,    22,   543,
     544,   258,   258,   264,   261,   262,   303,   304,   521,   522,
     523,   524,   508,   726,    14,   231,   366,   368,   369,   370,
     264,   322,   323,   324,     3,     4,   275,   276,   423,   443,
     539,     3,     4,   604,    81,   660,   661,   265,   598,   265,
     185,   186,   265,   185,   186,   -24,   200,   289,   289,   291,
     378,   429,   606,   424,   185,   186,    54,    84,   578,   579,
     580,    86,    88,   616,   616,   265,   185,   186,   504,   504,
     622,   623,   624,   625,   626,    90,   684,   443,   617,   617,
     381,   443,   443,   682,   515,   293,   443,   161,   582,   321,
     519,   520,   208,   718,   364,   454,   635,   636,   727,    24,
     210,   443,   443,   265,   185,   186,   359,   360,   644,   763,
     764,    14,   396,   265,   185,   186,   580,   761,   762,    18,
     265,   185,   186,   365,   571,   371,   372,   743,   397,   -24,
     398,   574,   605,   712,   575,   483,    49,   425,   135,   170,
     484,   485,   680,   -24,    51,   171,   439,   441,    52,   173,
     683,   442,   451,   382,   383,    67,   782,   681,    34,   696,
     504,   440,   690,   691,   692,   693,   694,   389,    99,   294,
     -28,   -28,   482,   376,    72,   430,   433,    95,   294,    96,
     443,   285,   174,    24,   443,    76,   410,   200,    77,    55,
      91,   452,    24,    15,    16,   453,   509,   724,   103,    19,
     731,   731,   731,   731,   731,    27,    28,   294,   294,   739,
     740,   339,   725,   697,   105,   294,   135,   478,   294,   106,
     698,    24,    24,   166,   167,   425,   425,   425,   425,    24,
     132,   549,    24,   146,   373,   443,   374,   699,   700,   550,
     551,   552,   455,   458,     3,     4,   340,   341,   342,   528,
     343,   529,   -72,   344,   213,   214,   215,   135,   216,   217,
     731,   731,   731,   731,   185,   186,   569,   147,   218,   508,
     554,    81,   230,   540,   425,   425,   425,   425,   594,   731,
     731,   731,   731,   219,   220,   221,   222,   148,   505,   150,
     460,   376,   176,   288,   732,   733,   734,   735,   151,   461,
     153,   174,   155,   176,   290,   152,   666,   668,   670,   671,
     672,   393,   394,   565,   395,   583,   379,   380,   154,   666,
     668,   555,   556,   685,   557,   296,   297,   135,   396,   425,
     558,   559,   462,   425,   412,   413,   463,   156,   607,   608,
     609,   355,   431,   157,   397,    36,   398,   621,   560,   561,
     562,   563,   564,   769,   770,   771,   772,   159,    36,    36,
      36,    36,   355,   480,   673,   361,   495,   163,   717,    36,
      36,    96,   778,   779,   780,   781,   165,   386,   176,    78,
     374,   507,   -18,   412,   570,   444,   595,   742,   -18,   -18,
     -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,
     -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,   -18,
      68,    69,    70,    71,   498,   649,   169,   387,   501,   659,
     179,    74,    75,   678,   679,    81,   370,   549,   388,   214,
     215,   135,   216,   217,   194,   550,   551,   552,   553,    55,
     676,   195,   218,   730,   730,   730,   730,   730,   296,   297,
     721,   722,   747,   298,   299,   300,   301,   219,   220,   221,
     222,    56,    57,    58,    59,    60,    61,    62,    63,    64,
      65,   651,   374,   746,   701,   174,   484,   196,   765,   408,
     -18,   662,   760,   255,   256,   173,   738,   197,   133,   198,
     409,   493,   494,   135,   203,   652,   653,   654,   655,   656,
     642,   643,   204,   730,   730,   730,   730,   228,   713,   458,
     346,   347,   227,   349,   350,   351,   352,   353,   174,   229,
     232,   233,   730,   730,   730,   730,   136,   137,   138,   139,
     140,   141,   142,   143,   144,    57,    58,    59,    60,    61,
     235,   249,   246,   107,   251,   252,   744,   253,   376,   108,
     109,   110,   111,   112,   113,   114,   115,   116,   117,   118,
     119,   120,   121,   122,   123,   124,   125,   126,   127,   128,
     129,   674,   257,   269,   270,   317,   271,   272,   273,   274,
     133,   585,   675,   305,   135,   135,   484,   485,   277,   278,
     279,   133,   280,   134,   281,   173,   135,   282,   287,   292,
     314,   315,   318,   319,   320,   586,   587,   562,   563,   564,
     325,   327,   328,   336,   337,   338,   348,   354,   136,   137,
     138,   139,   140,   141,   142,   143,   144,   355,   174,   136,
     137,   138,   139,   140,   141,   142,   143,   144,   484,   361,
     363,   130,   401,   484,   402,   468,   390,   173,   296,   297,
     391,   392,   173,   298,   299,   300,   703,   586,   587,   562,
     563,   564,   652,   653,   654,   655,   656,   469,   403,   470,
     471,   404,   405,   406,   472,   473,   474,   475,   407,   411,
     174,   296,   297,   418,   421,   174,   750,   751,   752,   753,
     428,   419,   435,   420,   436,   444,   447,   448,   374,   459,
     497,   467,   450,   477,   479,   487,   491,   492,   498,   501,
     516,   506,   510,   545,   517,   525,   526,   518,   527,   538,
     541,   546,   566,   567,   462,   568,   572,   573,   584,   588,
     529,   601,   592,   602,   603,   611,   612,   627,   628,   639,
     630,   633,   637,   638,   640,   641,   604,   718,   645,   646,
     647,   648,   657,   658,   662,   754,   677,   686,   687,   688,
     695,   702,   704,   705,   706,   707,   708,   709,   710,   711,
     715,   164,   716,   719,   720,   736,   741,   158,   745,   748,
     749,   755,   756,   757,   758,   759,   766,   767,   768,   773,
     774,   775,   776,   783,   784,   785,   786,   787,   788,   284,
     514,   175,   723,   618,   596,   496,   777,   511,   714,     0,
     631,   632,   260,     0,     0,     0,     0,     0,    30,    29,
       0,    50,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,    73
};

static const yytype_int16 yycheck[] =
{
      56,   138,   104,    81,   339,   142,   143,   144,   390,   390,
     392,   392,   453,   390,   392,   392,   345,     3,   224,   171,
     201,   202,   175,   390,   544,   392,   296,   297,   558,     7,
     384,   133,     5,     3,   136,   137,     4,   139,   140,   141,
       3,   194,   195,   196,   197,   198,   390,     3,   392,     3,
       4,     8,     3,    69,    24,   192,   209,     3,    55,   388,
     266,    24,     3,     3,    14,     1,     1,     1,   531,     3,
       1,     1,   535,   536,     0,     3,    26,   540,   134,    89,
     610,   461,    79,    80,     1,   187,   102,   189,   190,   191,
      76,   193,   298,   299,   300,   301,     1,     1,   176,     3,
     252,     1,   622,   623,    72,   434,     3,    64,     3,   489,
     490,   213,   214,     3,   216,   217,   269,   270,   472,   473,
     474,   475,   457,     3,   102,   181,   332,   333,   334,   335,
       3,   283,   285,   286,   104,   105,   238,   239,    78,   483,
     484,   104,   105,     3,     3,   619,   620,   103,    76,   103,
     104,   105,   103,   104,   105,     4,   102,   259,   260,   261,
     341,   102,   549,   103,   104,   105,   102,   102,   522,   523,
     524,   102,   102,   555,   556,   103,   104,   105,   619,   620,
     560,   561,   562,   563,   564,   102,    76,   531,   555,   556,
       1,   535,   536,   634,   464,     4,   540,   102,   527,     4,
     470,   471,   102,    76,     4,   102,   586,   587,    88,   102,
     171,   555,   556,   103,   104,   105,   318,   319,   605,   739,
     740,   102,    69,   103,   104,   105,   580,   737,   738,     3,
     103,   104,   105,     4,     4,   337,   338,   711,    85,    88,
      87,     4,   102,   102,     4,    55,     4,   384,    58,   102,
      60,    61,   634,   102,     6,   108,   638,   638,   102,    69,
     638,   638,     4,    74,    75,     4,   776,   634,     3,     1,
     711,   638,   652,   653,   654,   655,   656,   355,     1,    88,
       3,     4,   435,   339,     3,   387,   388,     1,    88,     3,
     634,   252,   102,   102,   638,     4,   374,   102,     4,     9,
     102,    43,   102,   104,   105,    47,   459,   689,     3,     9,
     690,   691,   692,   693,   694,    15,    16,    88,    88,   699,
     700,    31,   689,    55,     3,    88,    58,   429,    88,     4,
      62,   102,   102,     3,     4,   472,   473,   474,   475,   102,
       4,    44,   102,     4,     1,   689,     3,    79,    80,    52,
      53,    54,   408,   409,   104,   105,    66,    67,    68,     1,
      70,     3,     4,    73,    55,    56,    57,    58,    59,    60,
     750,   751,   752,   753,   104,   105,   513,     4,    69,   714,
       1,     3,     4,   485,   521,   522,   523,   524,   541,   769,
     770,   771,   772,    84,    85,    86,    87,    41,   454,     4,
      60,   457,     3,     4,   691,   692,   693,   694,    86,    69,
      88,   102,    90,     3,     4,     4,   622,   623,   624,   625,
     626,    52,    53,   501,    55,   527,    71,    72,     4,   635,
     636,    52,    53,   639,    55,   102,   103,    58,    69,   576,
      61,    62,   102,   580,     3,     4,   106,     4,   550,   551,
     552,     3,     4,   108,    85,    23,    87,   559,    79,    80,
      81,    82,    83,   750,   751,   752,   753,     4,    36,    37,
      38,    39,     3,     4,   627,     3,     4,     4,   684,    47,
      48,     3,   769,   770,   771,   772,     4,     4,     3,     1,
       3,     4,     4,     3,     4,     3,     4,   703,    10,    11,
      12,    13,    14,    15,    16,    17,    18,    19,    20,    21,
      22,    23,    24,    25,    26,    27,    28,    29,    30,    31,
      36,    37,    38,    39,     3,     4,     4,    44,     3,     4,
      65,    47,    48,     3,     4,     3,   742,    44,    55,    56,
      57,    58,    59,    60,     3,    52,    53,    54,    55,     9,
     628,     3,    69,   690,   691,   692,   693,   694,   102,   103,
       3,     4,   715,   107,   108,   109,   110,    84,    85,    86,
      87,    31,    32,    33,    34,    35,    36,    37,    38,    39,
      40,    55,     3,     4,   662,   102,    60,     3,   741,    44,
     102,     3,     4,   210,   211,    69,   698,     3,    53,     3,
      55,   447,   448,    58,     4,    79,    80,    81,    82,    83,
     601,   602,    88,   750,   751,   752,   753,   102,   674,   675,
     306,   307,     4,   309,   310,   311,   312,   313,   102,     4,
       4,     4,   769,   770,   771,   772,    91,    92,    93,    94,
      95,    96,    97,    98,    99,    32,    33,    34,    35,    36,
       4,   102,    63,     4,     4,   108,   712,     4,   714,    10,
      11,    12,    13,    14,    15,    16,    17,    18,    19,    20,
      21,    22,    23,    24,    25,    26,    27,    28,    29,    30,
      31,    44,     4,     3,     3,    42,     4,     4,     4,     4,
      53,    55,    55,     3,    58,    58,    60,    61,     4,     4,
       4,    53,     4,    55,     4,    69,    58,     4,     4,     4,
       4,     4,    42,    42,    51,    79,    80,    81,    82,    83,
     108,     4,   102,     4,     4,     4,     4,     4,    91,    92,
      93,    94,    95,    96,    97,    98,    99,     3,   102,    91,
      92,    93,    94,    95,    96,    97,    98,    99,    60,     3,
      90,   102,     4,    60,     4,    78,    47,    69,   102,   103,
      47,    47,    69,   107,   108,   109,   110,    79,    80,    81,
      82,    83,    79,    80,    81,    82,    83,   100,     4,   102,
     103,     4,     4,     4,   107,   108,   109,   110,     4,     4,
     102,   102,   103,     4,     4,   102,   107,   108,   109,   110,
       4,   104,     3,   104,     3,     3,     3,     3,     3,     3,
      77,     4,    88,     4,     4,     4,     4,     4,     3,     3,
     104,     4,     3,    88,     4,     4,     4,   102,     4,     3,
       3,    88,     4,     4,   102,     4,     4,     4,     4,     4,
       3,    77,     4,    77,     4,     4,     3,     3,     3,   110,
       4,     4,     4,     4,     4,     4,     3,    76,     4,     4,
       4,     4,     4,     4,     3,    77,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       3,    98,     4,     4,     4,     4,     3,    94,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,   250,
     463,   103,   689,   556,   542,   449,   768,   460,   675,    -1,
     576,   580,   214,    -1,    -1,    -1,    -1,    -1,    21,    20,
      -1,    26,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
      -1,    -1,    -1,    -1,    -1,    -1,    40
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,     3,    24,   104,   105,   113,   114,   218,   231,   232,
     233,   234,   235,     7,   102,   104,   105,     0,     3,   231,
       5,    89,     3,   116,   102,   131,   137,   231,   231,   235,
     233,     8,    64,     1,     3,   115,   117,   198,   199,   200,
     202,   203,   205,   206,   207,   208,   209,   216,   217,     4,
     131,     6,   102,     1,   102,     9,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,   204,     4,   115,   115,
     115,   115,     3,   203,   115,   115,     4,     4,     1,   118,
       1,     3,   189,     1,   102,     1,   102,     1,   102,     1,
     102,   102,   130,   132,   138,     1,     3,   119,   120,     1,
     124,   133,   142,     3,   196,     3,     4,     4,    10,    11,
      12,    13,    14,    15,    16,    17,    18,    19,    20,    21,
      22,    23,    24,    25,    26,    27,    28,    29,    30,    31,
     102,   215,     4,    53,    55,    58,    91,    92,    93,    94,
      95,    96,    97,    98,    99,   193,     4,     4,    41,   214,
       4,   214,     4,   214,     4,   214,     4,   108,   132,     4,
       1,   102,   121,     4,   119,     4,     3,     4,   125,     4,
     102,   108,   140,    69,   102,   122,     3,   190,   195,    65,
     190,   188,   189,   190,   190,   104,   105,   179,   190,   190,
     190,   179,   179,   179,     3,     3,     3,     3,     3,     3,
     102,   139,   141,     4,    88,   128,   129,     1,   102,   127,
     139,   141,   128,    55,    56,    57,    59,    60,    69,    84,
      85,    86,    87,   122,   182,   193,   194,     4,   102,     4,
       4,   189,     4,     4,   190,     4,   190,   190,   190,   179,
     190,   128,   128,   128,   128,   128,    63,   130,   130,   102,
     135,     4,   108,     4,   128,   133,   133,     4,   190,   192,
     192,   190,   190,   134,     3,   103,   177,   179,   180,     3,
       3,     4,     4,     4,     4,   190,   190,     4,     4,     4,
       4,     4,     4,   143,   129,   139,   141,     4,     4,   190,
       4,   190,     4,     4,    88,   137,   102,   103,   107,   108,
     109,   110,   177,   128,   128,     3,   117,   201,   219,   220,
     221,   223,   224,   225,     4,     4,   189,    42,    42,    42,
      51,     4,   141,   128,   128,   108,   126,     4,   102,   136,
     134,   134,   177,   177,   177,   177,     4,     4,     4,    31,
      66,    67,    68,    70,    73,   222,   219,   219,     4,   219,
     219,   219,   219,   219,     4,     3,   183,   187,   190,   190,
     190,     3,   166,    90,     4,     4,   177,     4,   177,   177,
     177,   190,   190,     1,     3,   184,   189,   144,   130,    71,
      72,     1,    74,    75,   226,   183,     4,    44,    55,   193,
      47,    47,    47,    52,    53,    55,    69,    85,    87,   167,
     210,     4,     4,     4,     4,     4,     4,     4,    44,    55,
     193,     4,     3,     4,   145,   172,   173,   197,     4,   104,
     104,     4,     3,    78,   103,   179,   181,   227,     4,   102,
     190,     4,   183,   190,   191,     3,     3,   147,   151,   170,
     171,   174,   175,   195,     3,   162,   147,     3,     3,   169,
      88,     4,    43,    47,   102,   189,   184,   186,   189,     3,
      60,    69,   102,   106,   123,     4,    72,     4,    78,   100,
     102,   103,   107,   108,   109,   110,   228,     4,   190,     4,
       4,   183,   128,    55,    60,    61,   193,     4,    55,    79,
      80,     4,     4,   167,   167,     4,   166,    77,     3,   211,
     213,     3,   152,   154,   176,   189,     4,     4,   184,   128,
       3,   197,     3,   180,   144,   134,   104,     4,   102,   134,
     134,   227,   227,   227,   227,     4,     4,     4,     1,     3,
     146,   148,   149,   170,   171,   174,   175,   176,     3,   195,
     190,     3,   163,   180,   180,    88,    88,   168,   177,    44,
      52,    53,    54,    55,     1,    52,    53,    55,    61,    62,
      79,    80,    81,    82,    83,   193,     4,     4,     4,   179,
       4,     4,     4,     4,     4,     4,   227,   229,   227,   227,
     227,   230,   183,   190,     4,    55,    79,    80,     4,   146,
     146,   146,     4,   146,   128,     4,   162,     3,    76,   178,
     178,    77,    77,     4,     3,   102,   213,   190,   190,   190,
     212,     4,     3,   158,   159,   161,   170,   171,   158,   153,
     211,   190,   180,   180,   180,   180,   180,     3,     3,   185,
       4,   229,   230,     4,   150,   180,   180,     4,     4,   110,
       4,     4,   168,   168,   213,     4,     4,     4,     4,     4,
     211,    55,    79,    80,    81,    82,    83,     4,     4,     4,
     152,   152,     3,   155,   156,     3,   177,   178,   177,   178,
     177,   177,   177,   128,    44,    55,   193,     4,     3,     4,
     170,   171,   176,   147,    76,   177,     4,     4,     4,   160,
     180,   180,   180,   180,   180,     4,     1,    55,    62,    79,
      80,   193,     4,   110,     4,     4,     4,     4,     4,     4,
       4,     4,   102,   189,   186,     3,     4,   177,    76,     4,
       4,     3,     4,   161,   170,   171,     3,    88,   164,   165,
     179,   180,   164,   164,   164,   164,     4,   157,   190,   180,
     180,     3,   177,   152,   189,     4,     4,   128,     4,     4,
     107,   108,   109,   110,    77,     4,     4,     4,     4,     4,
       4,   156,   156,   178,   178,   128,     4,     4,     4,   164,
     164,   164,   164,     4,     4,     4,     4,   185,   164,   164,
     164,   164,   156,     4,     4,     4,     4,     4,     4
};

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrorlab


/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */

#define YYFAIL		goto yyerrlab

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)					\
do								\
  if (yychar == YYEMPTY && yylen == 1)				\
    {								\
      yychar = (Token);						\
      yylval = (Value);						\
      yytoken = YYTRANSLATE (yychar);				\
      YYPOPSTACK (1);						\
      goto yybackup;						\
    }								\
  else								\
    {								\
      yyerror (YY_("syntax error: cannot back up")); \
      YYERROR;							\
    }								\
while (YYID (0))


#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K])
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)				\
    do									\
      if (YYID (N))                                                    \
	{								\
	  (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
	  (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
	  (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
	  (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
	}								\
      else								\
	{								\
	  (Current).first_line   = (Current).last_line   =		\
	    YYRHSLOC (Rhs, 0).last_line;				\
	  (Current).first_column = (Current).last_column =		\
	    YYRHSLOC (Rhs, 0).last_column;				\
	}								\
    while (YYID (0))
#endif


/* YY_LOCATION_PRINT -- Print the location on the stream.
   This macro was not mandated originally: define only if we know
   we won't break user code: when these are the locations we know.  */

#ifndef YY_LOCATION_PRINT
# if YYLTYPE_IS_TRIVIAL
#  define YY_LOCATION_PRINT(File, Loc)			\
     fprintf (File, "%d.%d-%d.%d",			\
	      (Loc).first_line, (Loc).first_column,	\
	      (Loc).last_line,  (Loc).last_column)
# else
#  define YY_LOCATION_PRINT(File, Loc) ((void) 0)
# endif
#endif


/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (YYLEX_PARAM)
#else
# define YYLEX yylex ()
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
do {						\
  if (yydebug)					\
    YYFPRINTF Args;				\
} while (YYID (0))

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)			  \
do {									  \
  if (yydebug)								  \
    {									  \
      YYFPRINTF (stderr, "%s ", Title);					  \
      yy_symbol_print (stderr,						  \
		  Type, Value); \
      YYFPRINTF (stderr, "\n");						  \
    }									  \
} while (YYID (0))


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_value_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# else
  YYUSE (yyoutput);
# endif
  switch (yytype)
    {
      default:
	break;
    }
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (yytype < YYNTOKENS)
    YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
  else
    YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_stack_print (yytype_int16 *bottom, yytype_int16 *top)
#else
static void
yy_stack_print (bottom, top)
    yytype_int16 *bottom;
    yytype_int16 *top;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (; bottom <= top; ++bottom)
    YYFPRINTF (stderr, " %d", *bottom);
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
do {								\
  if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
} while (YYID (0))


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_reduce_print (YYSTYPE *yyvsp, int yyrule)
#else
static void
yy_reduce_print (yyvsp, yyrule)
    YYSTYPE *yyvsp;
    int yyrule;
#endif
{
  int yynrhs = yyr2[yyrule];
  int yyi;
  unsigned long int yylno = yyrline[yyrule];
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
	     yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      fprintf (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr, yyrhs[yyprhs[yyrule] + yyi],
		       &(yyvsp[(yyi + 1) - (yynrhs)])
		       		       );
      fprintf (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)		\
do {					\
  if (yydebug)				\
    yy_reduce_print (yyvsp, Rule); \
} while (YYID (0))

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static YYSIZE_T
yystrlen (const char *yystr)
#else
static YYSIZE_T
yystrlen (yystr)
    const char *yystr;
#endif
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static char *
yystpcpy (char *yydest, const char *yysrc)
#else
static char *
yystpcpy (yydest, yysrc)
    char *yydest;
    const char *yysrc;
#endif
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
	switch (*++yyp)
	  {
	  case '\'':
	  case ',':
	    goto do_not_strip_quotes;

	  case '\\':
	    if (*++yyp != '\\')
	      goto do_not_strip_quotes;
	    /* Fall through.  */
	  default:
	    if (yyres)
	      yyres[yyn] = *yyp;
	    yyn++;
	    break;

	  case '"':
	    if (yyres)
	      yyres[yyn] = '\0';
	    return yyn;
	  }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into YYRESULT an error message about the unexpected token
   YYCHAR while in state YYSTATE.  Return the number of bytes copied,
   including the terminating null byte.  If YYRESULT is null, do not
   copy anything; just return the number of bytes that would be
   copied.  As a special case, return 0 if an ordinary "syntax error"
   message will do.  Return YYSIZE_MAXIMUM if overflow occurs during
   size calculation.  */
static YYSIZE_T
yysyntax_error (char *yyresult, int yystate, int yychar)
{
  int yyn = yypact[yystate];

  if (! (YYPACT_NINF < yyn && yyn <= YYLAST))
    return 0;
  else
    {
      int yytype = YYTRANSLATE (yychar);
      YYSIZE_T yysize0 = yytnamerr (0, yytname[yytype]);
      YYSIZE_T yysize = yysize0;
      YYSIZE_T yysize1;
      int yysize_overflow = 0;
      enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
      char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
      int yyx;

# if 0
      /* This is so xgettext sees the translatable formats that are
	 constructed on the fly.  */
      YY_("syntax error, unexpected %s");
      YY_("syntax error, unexpected %s, expecting %s");
      YY_("syntax error, unexpected %s, expecting %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s");
# endif
      char *yyfmt;
      char const *yyf;
      static char const yyunexpected[] = "syntax error, unexpected %s";
      static char const yyexpecting[] = ", expecting %s";
      static char const yyor[] = " or %s";
      char yyformat[sizeof yyunexpected
		    + sizeof yyexpecting - 1
		    + ((YYERROR_VERBOSE_ARGS_MAXIMUM - 2)
		       * (sizeof yyor - 1))];
      char const *yyprefix = yyexpecting;

      /* Start YYX at -YYN if negative to avoid negative indexes in
	 YYCHECK.  */
      int yyxbegin = yyn < 0 ? -yyn : 0;

      /* Stay within bounds of both yycheck and yytname.  */
      int yychecklim = YYLAST - yyn + 1;
      int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
      int yycount = 1;

      yyarg[0] = yytname[yytype];
      yyfmt = yystpcpy (yyformat, yyunexpected);

      for (yyx = yyxbegin; yyx < yyxend; ++yyx)
	if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
	  {
	    if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
	      {
		yycount = 1;
		yysize = yysize0;
		yyformat[sizeof yyunexpected - 1] = '\0';
		break;
	      }
	    yyarg[yycount++] = yytname[yyx];
	    yysize1 = yysize + yytnamerr (0, yytname[yyx]);
	    yysize_overflow |= (yysize1 < yysize);
	    yysize = yysize1;
	    yyfmt = yystpcpy (yyfmt, yyprefix);
	    yyprefix = yyor;
	  }

      yyf = YY_(yyformat);
      yysize1 = yysize + yystrlen (yyf);
      yysize_overflow |= (yysize1 < yysize);
      yysize = yysize1;

      if (yysize_overflow)
	return YYSIZE_MAXIMUM;

      if (yyresult)
	{
	  /* Avoid sprintf, as that infringes on the user's name space.
	     Don't have undefined behavior even if the translation
	     produced a string with the wrong number of "%s"s.  */
	  char *yyp = yyresult;
	  int yyi = 0;
	  while ((*yyp = *yyf) != '\0')
	    {
	      if (*yyp == '%' && yyf[1] == 's' && yyi < yycount)
		{
		  yyp += yytnamerr (yyp, yyarg[yyi++]);
		  yyf += 2;
		}
	      else
		{
		  yyp++;
		  yyf++;
		}
	    }
	}
      return yysize;
    }
}
#endif /* YYERROR_VERBOSE */


/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yymsg, yytype, yyvaluep)
    const char *yymsg;
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  YYUSE (yyvaluep);

  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  switch (yytype)
    {

      default:
	break;
    }
}


/* Prevent warnings from -Wmissing-prototypes.  */

#ifdef YYPARSE_PARAM
#if defined __STDC__ || defined __cplusplus
int yyparse (void *YYPARSE_PARAM);
#else
int yyparse ();
#endif
#else /* ! YYPARSE_PARAM */
#if defined __STDC__ || defined __cplusplus
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */



/* The look-ahead symbol.  */
int yychar;

/* The semantic value of the look-ahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*----------.
| yyparse.  |
`----------*/

#ifdef YYPARSE_PARAM
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void *YYPARSE_PARAM)
#else
int
yyparse (YYPARSE_PARAM)
    void *YYPARSE_PARAM;
#endif
#else /* ! YYPARSE_PARAM */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{
  
  int yystate;
  int yyn;
  int yyresult;
  /* Number of tokens to shift before error messages enabled.  */
  int yyerrstatus;
  /* Look-ahead token as an internal (translated) token number.  */
  int yytoken = 0;
#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

  /* Three stacks and their tools:
     `yyss': related to states,
     `yyvs': related to semantic values,
     `yyls': related to locations.

     Refer to the stacks thru separate pointers, to allow yyoverflow
     to reallocate them elsewhere.  */

  /* The state stack.  */
  yytype_int16 yyssa[YYINITDEPTH];
  yytype_int16 *yyss = yyssa;
  yytype_int16 *yyssp;

  /* The semantic value stack.  */
  YYSTYPE yyvsa[YYINITDEPTH];
  YYSTYPE *yyvs = yyvsa;
  YYSTYPE *yyvsp;



#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  YYSIZE_T yystacksize = YYINITDEPTH;

  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;


  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY;		/* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */

  yyssp = yyss;
  yyvsp = yyvs;

  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack.  Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	yytype_int16 *yyss1 = yyss;


	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  This used to be a
	   conditional around just the two extra args, but that might
	   be undefined if yyoverflow is a macro.  */
	yyoverflow (YY_("memory exhausted"),
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),

		    &yystacksize);

	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
	goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
	yystacksize = YYMAXDEPTH;

      {
	yytype_int16 *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyexhaustedlab;
	YYSTACK_RELOCATE (yyss);
	YYSTACK_RELOCATE (yyvs);

#  undef YYSTACK_RELOCATE
	if (yyss1 != yyssa)
	  YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;


      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
		  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
	YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     look-ahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to look-ahead token.  */
  yyn = yypact[yystate];
  if (yyn == YYPACT_NINF)
    goto yydefault;

  /* Not known => get a look-ahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid look-ahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = YYLEX;
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yyn == 0 || yyn == YYTABLE_NINF)
	goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the look-ahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token unless it is eof.  */
  if (yychar != YYEOF)
    yychar = YYEMPTY;

  yystate = yyn;
  *++yyvsp = yylval;

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 2:
#line 252 "pddl+.yacc"
    {top_thing= (yyvsp[(1) - (1)].t_domain); current_analysis->the_domain= (yyvsp[(1) - (1)].t_domain);;}
    break;

  case 3:
#line 253 "pddl+.yacc"
    {top_thing= (yyvsp[(1) - (1)].t_problem); current_analysis->the_problem= (yyvsp[(1) - (1)].t_problem);;}
    break;

  case 4:
#line 254 "pddl+.yacc"
    {top_thing= (yyvsp[(1) - (1)].t_plan); ;}
    break;

  case 5:
#line 259 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(4) - (5)].t_domain); (yyval.t_domain)->name= (yyvsp[(3) - (5)].cp);delete [] (yyvsp[(3) - (5)].cp);
	if (types_used && !types_defined) {
		yyerrok; log_error(E_FATAL,"Syntax error in domain - no :types section, but types used in definitions."); 
	}
	;}
    break;

  case 6:
#line 265 "pddl+.yacc"
    {yyerrok; (yyval.t_domain)=static_cast<domain*>(NULL);
       	log_error(E_FATAL,"Syntax error in domain"); ;}
    break;

  case 7:
#line 271 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); (yyval.t_domain)->req= (yyvsp[(1) - (2)].t_pddl_req_flag);;}
    break;

  case 8:
#line 272 "pddl+.yacc"
    {types_defined = true; (yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); (yyval.t_domain)->types= (yyvsp[(1) - (2)].t_type_list);;}
    break;

  case 9:
#line 273 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); (yyval.t_domain)->constants= (yyvsp[(1) - (2)].t_const_symbol_list);;}
    break;

  case 10:
#line 274 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); 
                                       (yyval.t_domain)->predicates= (yyvsp[(1) - (2)].t_pred_decl_list); ;}
    break;

  case 11:
#line 276 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain); 
                                       (yyval.t_domain)->functions= (yyvsp[(1) - (2)].t_func_decl_list); ;}
    break;

  case 12:
#line 278 "pddl+.yacc"
    {(yyval.t_domain)= (yyvsp[(2) - (2)].t_domain);
   										(yyval.t_domain)->constraints = (yyvsp[(1) - (2)].t_con_goal);;}
    break;

  case 13:
#line 280 "pddl+.yacc"
    {(yyval.t_domain)= new domain((yyvsp[(1) - (1)].t_structure_store)); ;}
    break;

  case 14:
#line 283 "pddl+.yacc"
    {(yyval.cp)=(yyvsp[(3) - (4)].cp);;}
    break;

  case 15:
#line 289 "pddl+.yacc"
    {
	// Stash in analysis object --- we need to refer to it during parse
	//   but domain object is not created yet,
	current_analysis->req |= (yyvsp[(3) - (4)].t_pddl_req_flag);
	(yyval.t_pddl_req_flag)=(yyvsp[(3) - (4)].t_pddl_req_flag);
    ;}
    break;

  case 16:
#line 296 "pddl+.yacc"
    {yyerrok; 
       log_error(E_FATAL,"Syntax error in requirements declaration.");
       (yyval.t_pddl_req_flag)= 0; ;}
    break;

  case 17:
#line 302 "pddl+.yacc"
    { (yyval.t_pddl_req_flag)= (yyvsp[(1) - (2)].t_pddl_req_flag) | (yyvsp[(2) - (2)].t_pddl_req_flag); ;}
    break;

  case 18:
#line 303 "pddl+.yacc"
    { (yyval.t_pddl_req_flag)= 0; ;}
    break;

  case 19:
#line 309 "pddl+.yacc"
    {(yyval.t_pred_decl_list)=(yyvsp[(2) - (2)].t_pred_decl_list); (yyval.t_pred_decl_list)->push_front((yyvsp[(1) - (2)].t_pred_decl));;}
    break;

  case 20:
#line 311 "pddl+.yacc"
    {  (yyval.t_pred_decl_list)=new pred_decl_list;
           (yyval.t_pred_decl_list)->push_front((yyvsp[(1) - (1)].t_pred_decl)); ;}
    break;

  case 21:
#line 316 "pddl+.yacc"
    {(yyval.t_pred_decl)= new pred_decl((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_var_symbol_list),current_analysis->var_tab_stack.pop());;}
    break;

  case 22:
#line 318 "pddl+.yacc"
    {yyerrok; 
        // hope someone makes this error someday
        log_error(E_FATAL,"Syntax error in predicate declaration.");
	(yyval.t_pred_decl)= NULL; ;}
    break;

  case 23:
#line 326 "pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_put((yyvsp[(1) - (1)].cp));
           current_analysis->var_tab_stack.push(
           				current_analysis->buildPredTab());
           delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 24:
#line 333 "pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_ref("="); 
	      requires(E_EQUALITY); ;}
    break;

  case 25:
#line 335 "pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 26:
#line 343 "pddl+.yacc"
    { (yyval.t_pred_symbol)=current_analysis->pred_tab.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 27:
#line 349 "pddl+.yacc"
    {(yyval.t_func_decl_list)=(yyvsp[(1) - (2)].t_func_decl_list); (yyval.t_func_decl_list)->push_back((yyvsp[(2) - (2)].t_func_decl));;}
    break;

  case 28:
#line 350 "pddl+.yacc"
    { (yyval.t_func_decl_list)=new func_decl_list; ;}
    break;

  case 29:
#line 355 "pddl+.yacc"
    {(yyval.t_func_decl)= new func_decl((yyvsp[(2) - (5)].t_func_symbol),(yyvsp[(3) - (5)].t_var_symbol_list),current_analysis->var_tab_stack.pop());;}
    break;

  case 30:
#line 357 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in functor declaration.");
	 (yyval.t_func_decl)= NULL; ;}
    break;

  case 31:
#line 363 "pddl+.yacc"
    {(yyval.t_dummy) = NULL;;}
    break;

  case 32:
#line 363 "pddl+.yacc"
    {(yyval.t_dummy)=NULL;;}
    break;

  case 33:
#line 367 "pddl+.yacc"
    { (yyval.t_func_symbol)=current_analysis->func_tab.symbol_put((yyvsp[(1) - (1)].cp));
           current_analysis->var_tab_stack.push(
           		current_analysis->buildFuncTab()); 
           delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 34:
#line 380 "pddl+.yacc"
    {  
      (yyval.t_var_symbol_list)= (yyvsp[(1) - (4)].t_var_symbol_list);
      (yyval.t_var_symbol_list)->set_types((yyvsp[(3) - (4)].t_type));           /* Set types for variables */
      (yyval.t_var_symbol_list)->splice((yyval.t_var_symbol_list)->end(),*(yyvsp[(4) - (4)].t_var_symbol_list));   /* Join lists */ 
      delete (yyvsp[(4) - (4)].t_var_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
      types_used = true;
   ;}
    break;

  case 35:
#line 389 "pddl+.yacc"
    {  
      (yyval.t_var_symbol_list)= (yyvsp[(1) - (4)].t_var_symbol_list);
      (yyval.t_var_symbol_list)->set_either_types((yyvsp[(3) - (4)].t_type_list));    /* Set types for variables */
      (yyval.t_var_symbol_list)->splice((yyval.t_var_symbol_list)->end(),*(yyvsp[(4) - (4)].t_var_symbol_list));   /* Join lists */ 
      delete (yyvsp[(4) - (4)].t_var_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
      types_used = true;
   ;}
    break;

  case 36:
#line 398 "pddl+.yacc"
    {
       (yyval.t_var_symbol_list)= (yyvsp[(1) - (1)].t_var_symbol_list);
   ;}
    break;

  case 37:
#line 410 "pddl+.yacc"
    {(yyval.t_var_symbol_list)=(yyvsp[(3) - (3)].t_var_symbol_list); (yyvsp[(3) - (3)].t_var_symbol_list)->push_front((yyvsp[(2) - (3)].t_var_symbol)); ;}
    break;

  case 38:
#line 411 "pddl+.yacc"
    {(yyval.t_var_symbol_list)= new var_symbol_list; ;}
    break;

  case 39:
#line 418 "pddl+.yacc"
    {  
      (yyval.t_const_symbol_list)= (yyvsp[(1) - (4)].t_const_symbol_list);
      (yyvsp[(1) - (4)].t_const_symbol_list)->set_types((yyvsp[(3) - (4)].t_type));           /* Set types for constants */
      (yyvsp[(1) - (4)].t_const_symbol_list)->splice((yyvsp[(1) - (4)].t_const_symbol_list)->end(),*(yyvsp[(4) - (4)].t_const_symbol_list)); /* Join lists */ 
      delete (yyvsp[(4) - (4)].t_const_symbol_list);                   /* Delete (now empty) list */
      requires(E_TYPING);
      types_used = true;
   ;}
    break;

  case 40:
#line 427 "pddl+.yacc"
    {  
      (yyval.t_const_symbol_list)= (yyvsp[(1) - (4)].t_const_symbol_list);
      (yyvsp[(1) - (4)].t_const_symbol_list)->set_either_types((yyvsp[(3) - (4)].t_type_list));
      (yyvsp[(1) - (4)].t_const_symbol_list)->splice((yyvsp[(1) - (4)].t_const_symbol_list)->end(),*(yyvsp[(4) - (4)].t_const_symbol_list));
      delete (yyvsp[(4) - (4)].t_const_symbol_list);
      requires(E_TYPING);
      types_used = true;
   ;}
    break;

  case 41:
#line 436 "pddl+.yacc"
    {(yyval.t_const_symbol_list)= (yyvsp[(1) - (1)].t_const_symbol_list);;}
    break;

  case 42:
#line 441 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(2) - (2)].t_const_symbol_list); (yyvsp[(2) - (2)].t_const_symbol_list)->push_front((yyvsp[(1) - (2)].t_const_symbol));;}
    break;

  case 43:
#line 442 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=new const_symbol_list;;}
    break;

  case 44:
#line 446 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(2) - (2)].t_const_symbol_list); (yyvsp[(2) - (2)].t_const_symbol_list)->push_front((yyvsp[(1) - (2)].t_const_symbol));;}
    break;

  case 45:
#line 447 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=new const_symbol_list;;}
    break;

  case 46:
#line 456 "pddl+.yacc"
    {  
       (yyval.t_type_list)= (yyvsp[(1) - (4)].t_type_list);
       (yyval.t_type_list)->set_types((yyvsp[(3) - (4)].t_type));           /* Set types for constants */
       (yyval.t_type_list)->splice((yyval.t_type_list)->end(),*(yyvsp[(4) - (4)].t_type_list)); /* Join lists */ 
       delete (yyvsp[(4) - (4)].t_type_list);                   /* Delete (now empty) list */
   ;}
    break;

  case 47:
#line 463 "pddl+.yacc"
    {  
   // This parse needs to be excluded, we think (DPL&MF: 6/9/01)
       (yyval.t_type_list)= (yyvsp[(1) - (4)].t_type_list);
       (yyval.t_type_list)->set_either_types((yyvsp[(3) - (4)].t_type_list));
       (yyval.t_type_list)->splice((yyvsp[(1) - (4)].t_type_list)->end(),*(yyvsp[(4) - (4)].t_type_list));
       delete (yyvsp[(4) - (4)].t_type_list);
   ;}
    break;

  case 48:
#line 472 "pddl+.yacc"
    { (yyval.t_type_list)= (yyvsp[(1) - (1)].t_type_list); ;}
    break;

  case 49:
#line 478 "pddl+.yacc"
    {(yyval.t_parameter_symbol_list)=(yyvsp[(1) - (2)].t_parameter_symbol_list); (yyval.t_parameter_symbol_list)->push_back((yyvsp[(2) - (2)].t_const_symbol)); ;}
    break;

  case 50:
#line 480 "pddl+.yacc"
    {(yyval.t_parameter_symbol_list)=(yyvsp[(1) - (3)].t_parameter_symbol_list); (yyval.t_parameter_symbol_list)->push_back((yyvsp[(3) - (3)].t_var_symbol)); ;}
    break;

  case 51:
#line 481 "pddl+.yacc"
    {(yyval.t_parameter_symbol_list)= new parameter_symbol_list;;}
    break;

  case 52:
#line 488 "pddl+.yacc"
    { (yyval.t_var_symbol)= current_analysis->var_tab_stack.top()->symbol_put((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 53:
#line 494 "pddl+.yacc"
    { (yyval.t_var_symbol)= current_analysis->var_tab_stack.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 54:
#line 498 "pddl+.yacc"
    { (yyval.t_const_symbol)= current_analysis->const_tab.symbol_get((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp); ;}
    break;

  case 55:
#line 502 "pddl+.yacc"
    { (yyval.t_const_symbol)= current_analysis->const_tab.symbol_put((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 56:
#line 507 "pddl+.yacc"
    { (yyval.t_type_list)= (yyvsp[(3) - (4)].t_type_list); ;}
    break;

  case 57:
#line 512 "pddl+.yacc"
    { (yyval.t_type)= current_analysis->pddl_type_tab.symbol_ref((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 58:
#line 519 "pddl+.yacc"
    { (yyval.t_type)= current_analysis->pddl_type_tab.symbol_ref((yyvsp[(1) - (1)].cp)); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 59:
#line 524 "pddl+.yacc"
    {(yyval.t_type_list)= (yyvsp[(1) - (2)].t_type_list); (yyval.t_type_list)->push_back((yyvsp[(2) - (2)].t_type));;}
    break;

  case 60:
#line 525 "pddl+.yacc"
    {(yyval.t_type_list)= new pddl_type_list;;}
    break;

  case 61:
#line 530 "pddl+.yacc"
    {(yyval.t_type_list)= (yyvsp[(1) - (2)].t_type_list); (yyval.t_type_list)->push_back((yyvsp[(2) - (2)].t_type));;}
    break;

  case 62:
#line 531 "pddl+.yacc"
    {(yyval.t_type_list)= new pddl_type_list;;}
    break;

  case 63:
#line 536 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (6)].t_effect_lists);
	  (yyval.t_effect_lists)->assign_effects.push_back(new assignment((yyvsp[(4) - (6)].t_func_term),E_ASSIGN,(yyvsp[(5) - (6)].t_num_expression)));  
          if((yyvsp[(4) - (6)].t_func_term)->getFunction()->getName()=="total-cost")
          {
          	requires(E_ACTIONCOSTS); 
          	// Should also check that $5 is 0...
		  }
          else
          {
          	requires(E_NFLUENTS); 
          }
	;}
    break;

  case 64:
#line 549 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[(2) - (2)].t_simple_effect)); ;}
    break;

  case 65:
#line 551 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[(2) - (2)].t_simple_effect)); ;}
    break;

  case 66:
#line 553 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[(2) - (2)].t_timed_effect)); ;}
    break;

  case 67:
#line 555 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;;}
    break;

  case 68:
#line 560 "pddl+.yacc"
    { requires(E_TIMED_INITIAL_LITERALS); 
   		(yyval.t_timed_effect)=new timed_initial_literal((yyvsp[(3) - (4)].t_effect_lists),(yyvsp[(2) - (4)].fval));;}
    break;

  case 69:
#line 565 "pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(2) - (2)].t_effect_lists); (yyval.t_effect_lists)->append_effects((yyvsp[(1) - (2)].t_effect_lists)); delete (yyvsp[(1) - (2)].t_effect_lists);;}
    break;

  case 70:
#line 566 "pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(2) - (2)].t_effect_lists); (yyval.t_effect_lists)->cond_effects.push_front((yyvsp[(1) - (2)].t_cond_effect)); 
                                      requires(E_COND_EFFS);;}
    break;

  case 71:
#line 568 "pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(2) - (2)].t_effect_lists); (yyval.t_effect_lists)->forall_effects.push_front((yyvsp[(1) - (2)].t_forall_effect));
                                      requires(E_COND_EFFS);;}
    break;

  case 72:
#line 570 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists(); ;}
    break;

  case 73:
#line 579 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (1)].t_effect_lists);;}
    break;

  case 74:
#line 580 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 75:
#line 581 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 76:
#line 582 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->cond_effects.push_front((yyvsp[(1) - (1)].t_cond_effect));;}
    break;

  case 77:
#line 583 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->forall_effects.push_front((yyvsp[(1) - (1)].t_forall_effect));;}
    break;

  case 78:
#line 587 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 79:
#line 588 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (1)].t_effect_lists);;}
    break;

  case 80:
#line 593 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 81:
#line 595 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 82:
#line 597 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[(1) - (1)].t_assignment));
         requires(E_NFLUENTS);;}
    break;

  case 83:
#line 603 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 84:
#line 604 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 85:
#line 605 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->assign_effects.push_back((yyvsp[(2) - (2)].t_assignment));
                                     requires(E_NFLUENTS); ;}
    break;

  case 86:
#line 607 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 87:
#line 612 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists); ;}
    break;

  case 88:
#line 614 "pddl+.yacc"
    {yyerrok; (yyval.t_effect_lists)=NULL;
	 log_error(E_FATAL,"Syntax error in (and ...)");
	;}
    break;

  case 89:
#line 622 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists); ;}
    break;

  case 90:
#line 627 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; 
          (yyval.t_effect_lists)->forall_effects.push_back(
	       new forall_effect((yyvsp[(6) - (7)].t_effect_lists), (yyvsp[(4) - (7)].t_var_symbol_list), current_analysis->var_tab_stack.pop())); 
          requires(E_COND_EFFS);;}
    break;

  case 91:
#line 632 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->cond_effects.push_back(
	       new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)));
          requires(E_COND_EFFS); ;}
    break;

  case 92:
#line 637 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->cond_assign_effects.push_back(
	       new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)));
          requires(E_COND_EFFS); ;}
    break;

  case 93:
#line 642 "pddl+.yacc"
    { (yyval.t_effect_lists)=new effect_lists;
          (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[(1) - (1)].t_timed_effect)); ;}
    break;

  case 94:
#line 645 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[(1) - (1)].t_assignment));
          requires(E_NFLUENTS); ;}
    break;

  case 95:
#line 651 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyvsp[(1) - (2)].t_effect_lists)->append_effects((yyvsp[(2) - (2)].t_effect_lists)); delete (yyvsp[(2) - (2)].t_effect_lists); ;}
    break;

  case 96:
#line 652 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 97:
#line 657 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect((yyvsp[(3) - (4)].t_effect_lists),E_AT_START);;}
    break;

  case 98:
#line 659 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect((yyvsp[(3) - (4)].t_effect_lists),E_AT_END);;}
    break;

  case 99:
#line 661 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 100:
#line 665 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 101:
#line 669 "pddl+.yacc"
    {yyerrok; (yyval.t_timed_effect)=NULL;
	log_error(E_FATAL,"Syntax error in timed effect"); ;}
    break;

  case 102:
#line 675 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 103:
#line 679 "pddl+.yacc"
    {(yyval.t_timed_effect)=new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_timed_effect)->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 104:
#line 683 "pddl+.yacc"
    {yyerrok; (yyval.t_timed_effect)=NULL;
	log_error(E_FATAL,"Syntax error in conditional continuous effect"); ;}
    break;

  case 105:
#line 689 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists); ;}
    break;

  case 106:
#line 694 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; 
          (yyval.t_effect_lists)->forall_effects.push_back(
	       new forall_effect((yyvsp[(6) - (7)].t_effect_lists), (yyvsp[(4) - (7)].t_var_symbol_list), current_analysis->var_tab_stack.pop())); 
          requires(E_COND_EFFS);;}
    break;

  case 107:
#line 699 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists;
	  (yyval.t_effect_lists)->cond_assign_effects.push_back(
	       new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)));
          requires(E_COND_EFFS); ;}
    break;

  case 108:
#line 704 "pddl+.yacc"
    { (yyval.t_effect_lists)=new effect_lists;
          (yyval.t_effect_lists)->timed_effects.push_back((yyvsp[(1) - (1)].t_timed_effect)); ;}
    break;

  case 109:
#line 709 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyvsp[(1) - (2)].t_effect_lists)->append_effects((yyvsp[(2) - (2)].t_effect_lists)); delete (yyvsp[(2) - (2)].t_effect_lists); ;}
    break;

  case 110:
#line 710 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 111:
#line 714 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 112:
#line 715 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (1)].t_effect_lists);;}
    break;

  case 113:
#line 720 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->del_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 114:
#line 722 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->add_effects.push_front((yyvsp[(1) - (1)].t_simple_effect));;}
    break;

  case 115:
#line 724 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; (yyval.t_effect_lists)->assign_effects.push_front((yyvsp[(1) - (1)].t_assignment));
         requires(E_NFLUENTS);;}
    break;

  case 116:
#line 730 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->del_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 117:
#line 731 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->add_effects.push_back((yyvsp[(2) - (2)].t_simple_effect));;}
    break;

  case 118:
#line 732 "pddl+.yacc"
    {(yyval.t_effect_lists)= (yyvsp[(1) - (2)].t_effect_lists); (yyval.t_effect_lists)->assign_effects.push_back((yyvsp[(2) - (2)].t_assignment));
                                     requires(E_NFLUENTS); ;}
    break;

  case 119:
#line 734 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 120:
#line 740 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_ASSIGN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 121:
#line 742 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 122:
#line 744 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 123:
#line 746 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_UP,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 124:
#line 748 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_DOWN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 125:
#line 753 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; 
         timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_effect_lists)->timed_effects.push_front(te);
         te->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 126:
#line 759 "pddl+.yacc"
    {(yyval.t_effect_lists)=new effect_lists; 
         timed_effect * te = new timed_effect(new effect_lists,E_CONTINUOUS);
         (yyval.t_effect_lists)->timed_effects.push_front(te);
         te->effs->assign_effects.push_front(
	     new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression))); ;}
    break;

  case 127:
#line 765 "pddl+.yacc"
    {(yyval.t_effect_lists) = (yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 128:
#line 769 "pddl+.yacc"
    { (yyval.t_effect_lists)=(yyvsp[(1) - (2)].t_effect_lists); (yyvsp[(1) - (2)].t_effect_lists)->append_effects((yyvsp[(2) - (2)].t_effect_lists)); delete (yyvsp[(2) - (2)].t_effect_lists); ;}
    break;

  case 129:
#line 770 "pddl+.yacc"
    { (yyval.t_effect_lists)= new effect_lists; ;}
    break;

  case 130:
#line 774 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_expression);;}
    break;

  case 131:
#line 775 "pddl+.yacc"
    {(yyval.t_expression)= new special_val_expr(E_DURATION_VAR);
                    requires( E_DURATION_INEQUALITIES );;}
    break;

  case 132:
#line 777 "pddl+.yacc"
    { (yyval.t_expression)=(yyvsp[(1) - (1)].t_num_expression); ;}
    break;

  case 133:
#line 778 "pddl+.yacc"
    { (yyval.t_expression)= (yyvsp[(1) - (1)].t_func_term); ;}
    break;

  case 134:
#line 783 "pddl+.yacc"
    { (yyval.t_expression)= new plus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 135:
#line 785 "pddl+.yacc"
    { (yyval.t_expression)= new minus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 136:
#line 787 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 137:
#line 789 "pddl+.yacc"
    { (yyval.t_expression)= new div_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 138:
#line 794 "pddl+.yacc"
    { (yyval.t_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list)); ;}
    break;

  case 139:
#line 796 "pddl+.yacc"
    { (yyval.t_goal)= new timed_goal(new comparison((yyvsp[(2) - (6)].t_comparison_op),
        			new special_val_expr(E_DURATION_VAR),(yyvsp[(5) - (6)].t_expression)),E_AT_START); ;}
    break;

  case 140:
#line 799 "pddl+.yacc"
    { (yyval.t_goal) = new timed_goal(new comparison((yyvsp[(4) - (9)].t_comparison_op),
					new special_val_expr(E_DURATION_VAR),(yyvsp[(7) - (9)].t_expression)),E_AT_START);;}
    break;

  case 141:
#line 802 "pddl+.yacc"
    { (yyval.t_goal) = new timed_goal(new comparison((yyvsp[(4) - (9)].t_comparison_op),
					new special_val_expr(E_DURATION_VAR),(yyvsp[(7) - (9)].t_expression)),E_AT_END);;}
    break;

  case 142:
#line 807 "pddl+.yacc"
    {(yyval.t_comparison_op)= E_LESSEQ; requires(E_DURATION_INEQUALITIES);;}
    break;

  case 143:
#line 808 "pddl+.yacc"
    {(yyval.t_comparison_op)= E_GREATEQ; requires(E_DURATION_INEQUALITIES);;}
    break;

  case 144:
#line 809 "pddl+.yacc"
    {(yyval.t_comparison_op)= E_EQUALS; ;}
    break;

  case 145:
#line 817 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_expression); ;}
    break;

  case 146:
#line 822 "pddl+.yacc"
    { (yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal)); ;}
    break;

  case 147:
#line 824 "pddl+.yacc"
    { (yyval.t_goal_list)= new goal_list; ;}
    break;

  case 148:
#line 829 "pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(3) - (4)].t_proposition)); ;}
    break;

  case 149:
#line 834 "pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(1) - (1)].t_proposition)); ;}
    break;

  case 150:
#line 841 "pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(3) - (4)].t_proposition)); ;}
    break;

  case 151:
#line 846 "pddl+.yacc"
    { (yyval.t_simple_effect)= new simple_effect((yyvsp[(1) - (1)].t_proposition)); ;}
    break;

  case 152:
#line 851 "pddl+.yacc"
    { (yyval.t_forall_effect)= new forall_effect((yyvsp[(6) - (7)].t_effect_lists), (yyvsp[(4) - (7)].t_var_symbol_list), current_analysis->var_tab_stack.pop());;}
    break;

  case 153:
#line 856 "pddl+.yacc"
    { (yyval.t_cond_effect)= new cond_effect((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_effect_lists)); ;}
    break;

  case 154:
#line 861 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_ASSIGN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 155:
#line 863 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_INCREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 156:
#line 865 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_DECREASE,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 157:
#line 867 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_UP,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 158:
#line 869 "pddl+.yacc"
    { (yyval.t_assignment)= new assignment((yyvsp[(3) - (5)].t_func_term),E_SCALE_DOWN,(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 159:
#line 874 "pddl+.yacc"
    { (yyval.t_expression)= new uminus_expression((yyvsp[(3) - (4)].t_expression)); requires(E_NFLUENTS); ;}
    break;

  case 160:
#line 876 "pddl+.yacc"
    { (yyval.t_expression)= new plus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_NFLUENTS); ;}
    break;

  case 161:
#line 878 "pddl+.yacc"
    { (yyval.t_expression)= new minus_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_NFLUENTS); ;}
    break;

  case 162:
#line 880 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_NFLUENTS); ;}
    break;

  case 163:
#line 882 "pddl+.yacc"
    { (yyval.t_expression)= new div_expression((yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); requires(E_NFLUENTS); ;}
    break;

  case 164:
#line 883 "pddl+.yacc"
    { (yyval.t_expression)=(yyvsp[(1) - (1)].t_num_expression); ;}
    break;

  case 165:
#line 884 "pddl+.yacc"
    { (yyval.t_expression)= (yyvsp[(1) - (1)].t_func_term); requires(E_NFLUENTS); ;}
    break;

  case 166:
#line 889 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression(new special_val_expr(E_HASHT),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 167:
#line 891 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(3) - (5)].t_expression), new special_val_expr(E_HASHT)); ;}
    break;

  case 168:
#line 893 "pddl+.yacc"
    { (yyval.t_expression)= new special_val_expr(E_HASHT); ;}
    break;

  case 169:
#line 898 "pddl+.yacc"
    { (yyval.t_num_expression)=new int_expression((yyvsp[(1) - (1)].ival));   ;}
    break;

  case 170:
#line 899 "pddl+.yacc"
    { (yyval.t_num_expression)=new float_expression((yyvsp[(1) - (1)].fval)); ;}
    break;

  case 171:
#line 903 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 172:
#line 906 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 173:
#line 908 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(1) - (1)].cp)),
                            new parameter_symbol_list); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 174:
#line 926 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 175:
#line 928 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(2) - (4)].cp)), (yyvsp[(3) - (4)].t_parameter_symbol_list)); delete [] (yyvsp[(2) - (4)].cp); ;}
    break;

  case 176:
#line 930 "pddl+.yacc"
    { (yyval.t_func_term)=new func_term( current_analysis->func_tab.symbol_get((yyvsp[(1) - (1)].cp)),
                            new parameter_symbol_list); delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 177:
#line 935 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_GREATER; ;}
    break;

  case 178:
#line 936 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_GREATEQ; ;}
    break;

  case 179:
#line 937 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_LESS; ;}
    break;

  case 180:
#line 938 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_LESSEQ; ;}
    break;

  case 181:
#line 939 "pddl+.yacc"
    { (yyval.t_comparison_op)= E_EQUALS; ;}
    break;

  case 182:
#line 952 "pddl+.yacc"
    {(yyval.t_goal)= (yyvsp[(1) - (1)].t_goal);;}
    break;

  case 183:
#line 959 "pddl+.yacc"
    {(yyval.t_goal) = new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 184:
#line 962 "pddl+.yacc"
    {(yyval.t_goal)= new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_goal),current_analysis->var_tab_stack.pop());
        requires(E_UNIV_PRECS);;}
    break;

  case 185:
#line 964 "pddl+.yacc"
    {(yyval.t_goal) = new conj_goal(new goal_list);;}
    break;

  case 186:
#line 965 "pddl+.yacc"
    {(yyval.t_goal) = new conj_goal(new goal_list);;}
    break;

  case 187:
#line 970 "pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (4)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 188:
#line 972 "pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (5)].cp),(yyvsp[(4) - (5)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 189:
#line 974 "pddl+.yacc"
    {(yyval.t_con_goal) = new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 190:
#line 977 "pddl+.yacc"
    {(yyval.t_con_goal)= new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_con_goal),current_analysis->var_tab_stack.pop());
                requires(E_UNIV_PRECS);;}
    break;

  case 191:
#line 980 "pddl+.yacc"
    {(yyval.t_con_goal) = (yyvsp[(1) - (1)].t_con_goal);;}
    break;

  case 192:
#line 985 "pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (4)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 193:
#line 987 "pddl+.yacc"
    {(yyval.t_con_goal) = new preference((yyvsp[(3) - (5)].cp),(yyvsp[(4) - (5)].t_con_goal));requires(E_PREFERENCES);;}
    break;

  case 194:
#line 989 "pddl+.yacc"
    {(yyval.t_con_goal) = new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 195:
#line 992 "pddl+.yacc"
    {(yyval.t_con_goal)= new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_con_goal),current_analysis->var_tab_stack.pop());
                requires(E_UNIV_PRECS);;}
    break;

  case 196:
#line 998 "pddl+.yacc"
    {(yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyvsp[(1) - (2)].t_goal_list)->push_back((yyvsp[(2) - (2)].t_con_goal));;}
    break;

  case 197:
#line 1000 "pddl+.yacc"
    {(yyval.t_goal_list)= new goal_list; (yyval.t_goal_list)->push_back((yyvsp[(1) - (1)].t_con_goal));;}
    break;

  case 198:
#line 1005 "pddl+.yacc"
    {(yyval.t_goal)= new preference((yyvsp[(3) - (4)].t_goal)); requires(E_PREFERENCES);;}
    break;

  case 199:
#line 1007 "pddl+.yacc"
    {(yyval.t_goal)= new preference((yyvsp[(3) - (5)].cp),(yyvsp[(4) - (5)].t_goal)); requires(E_PREFERENCES);;}
    break;

  case 200:
#line 1011 "pddl+.yacc"
    {(yyval.t_goal)=(yyvsp[(1) - (1)].t_goal);;}
    break;

  case 201:
#line 1016 "pddl+.yacc"
    {(yyval.t_goal_list) = (yyvsp[(1) - (2)].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[(2) - (2)].t_con_goal));;}
    break;

  case 202:
#line 1018 "pddl+.yacc"
    {(yyval.t_goal_list) = new goal_list; (yyval.t_goal_list)->push_back((yyvsp[(1) - (1)].t_con_goal));;}
    break;

  case 203:
#line 1023 "pddl+.yacc"
    {(yyval.t_con_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 204:
#line 1025 "pddl+.yacc"
    {(yyval.t_con_goal) = new qfied_goal(E_FORALL,(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_con_goal),current_analysis->var_tab_stack.pop());
        requires(E_UNIV_PRECS);;}
    break;

  case 205:
#line 1028 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ATEND,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 206:
#line 1030 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ALWAYS,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 207:
#line 1032 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_SOMETIME,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 208:
#line 1034 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_WITHIN,(yyvsp[(4) - (5)].t_goal),NULL,(yyvsp[(3) - (5)].t_num_expression)->double_value(),0.0);delete (yyvsp[(3) - (5)].t_num_expression);;}
    break;

  case 209:
#line 1036 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ATMOSTONCE,(yyvsp[(3) - (4)].t_goal));;}
    break;

  case 210:
#line 1038 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_SOMETIMEAFTER,(yyvsp[(4) - (5)].t_goal),(yyvsp[(3) - (5)].t_goal));;}
    break;

  case 211:
#line 1040 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_SOMETIMEBEFORE,(yyvsp[(4) - (5)].t_goal),(yyvsp[(3) - (5)].t_goal));;}
    break;

  case 212:
#line 1042 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_ALWAYSWITHIN,(yyvsp[(5) - (6)].t_goal),(yyvsp[(4) - (6)].t_goal),(yyvsp[(3) - (6)].t_num_expression)->double_value(),0.0);delete (yyvsp[(3) - (6)].t_num_expression);;}
    break;

  case 213:
#line 1044 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_HOLDDURING,(yyvsp[(5) - (6)].t_goal),NULL,(yyvsp[(4) - (6)].t_num_expression)->double_value(),(yyvsp[(3) - (6)].t_num_expression)->double_value());delete (yyvsp[(3) - (6)].t_num_expression);delete (yyvsp[(4) - (6)].t_num_expression);;}
    break;

  case 214:
#line 1046 "pddl+.yacc"
    {(yyval.t_con_goal) = new constraint_goal(E_HOLDAFTER,(yyvsp[(4) - (5)].t_goal),NULL,0.0,(yyvsp[(3) - (5)].t_num_expression)->double_value());delete (yyvsp[(3) - (5)].t_num_expression);;}
    break;

  case 215:
#line 1051 "pddl+.yacc"
    {(yyval.t_goal)= new simple_goal((yyvsp[(1) - (1)].t_proposition),E_POS);;}
    break;

  case 216:
#line 1053 "pddl+.yacc"
    {(yyval.t_goal)= new neg_goal((yyvsp[(3) - (4)].t_goal));simple_goal * s = dynamic_cast<simple_goal *>((yyvsp[(3) - (4)].t_goal));
       if(s && s->getProp()->head->getName()=="=") {requires(E_EQUALITY);} 
       else{requires(E_NEGATIVE_PRECONDITIONS);};;}
    break;

  case 217:
#line 1057 "pddl+.yacc"
    {(yyval.t_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list));;}
    break;

  case 218:
#line 1059 "pddl+.yacc"
    {(yyval.t_goal)= new disj_goal((yyvsp[(3) - (4)].t_goal_list));
        requires(E_DISJUNCTIVE_PRECONDS);;}
    break;

  case 219:
#line 1062 "pddl+.yacc"
    {(yyval.t_goal)= new imply_goal((yyvsp[(3) - (5)].t_goal),(yyvsp[(4) - (5)].t_goal));
        requires(E_DISJUNCTIVE_PRECONDS);;}
    break;

  case 220:
#line 1066 "pddl+.yacc"
    {(yyval.t_goal)= new qfied_goal((yyvsp[(2) - (7)].t_quantifier),(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_goal),current_analysis->var_tab_stack.pop());;}
    break;

  case 221:
#line 1069 "pddl+.yacc"
    {(yyval.t_goal)= new qfied_goal((yyvsp[(2) - (7)].t_quantifier),(yyvsp[(4) - (7)].t_var_symbol_list),(yyvsp[(6) - (7)].t_goal),current_analysis->var_tab_stack.pop());;}
    break;

  case 222:
#line 1071 "pddl+.yacc"
    {(yyval.t_goal)= new comparison((yyvsp[(2) - (5)].t_comparison_op),(yyvsp[(3) - (5)].t_expression),(yyvsp[(4) - (5)].t_expression)); 
        requires(E_NFLUENTS);;}
    break;

  case 223:
#line 1077 "pddl+.yacc"
    {(yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyvsp[(1) - (2)].t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal));;}
    break;

  case 224:
#line 1079 "pddl+.yacc"
    {(yyval.t_goal_list)= new goal_list; (yyval.t_goal_list)->push_back((yyvsp[(1) - (1)].t_goal));;}
    break;

  case 225:
#line 1084 "pddl+.yacc"
    {(yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyvsp[(1) - (2)].t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal));;}
    break;

  case 226:
#line 1086 "pddl+.yacc"
    {(yyval.t_goal_list)= new goal_list; (yyval.t_goal_list)->push_back((yyvsp[(1) - (1)].t_goal));;}
    break;

  case 227:
#line 1096 "pddl+.yacc"
    {(yyval.t_quantifier)=E_FORALL; 
        current_analysis->var_tab_stack.push(
        		current_analysis->buildForallTab());;}
    break;

  case 228:
#line 1103 "pddl+.yacc"
    {(yyval.t_quantifier)=E_EXISTS;
        current_analysis->var_tab_stack.push(
        	current_analysis->buildExistsTab());;}
    break;

  case 229:
#line 1110 "pddl+.yacc"
    {(yyval.t_proposition)=new proposition((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_parameter_symbol_list));;}
    break;

  case 230:
#line 1115 "pddl+.yacc"
    {(yyval.t_proposition) = new proposition((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_var_symbol_list));;}
    break;

  case 231:
#line 1120 "pddl+.yacc"
    {(yyval.t_proposition)=new proposition((yyvsp[(2) - (4)].t_pred_symbol),(yyvsp[(3) - (4)].t_parameter_symbol_list));;}
    break;

  case 232:
#line 1125 "pddl+.yacc"
    {(yyval.t_pred_decl_list)= (yyvsp[(3) - (4)].t_pred_decl_list);;}
    break;

  case 233:
#line 1127 "pddl+.yacc"
    {yyerrok; (yyval.t_pred_decl_list)=NULL;
	 log_error(E_FATAL,"Syntax error in (:predicates ...)");
	;}
    break;

  case 234:
#line 1134 "pddl+.yacc"
    {(yyval.t_func_decl_list)= (yyvsp[(3) - (4)].t_func_decl_list);;}
    break;

  case 235:
#line 1136 "pddl+.yacc"
    {yyerrok; (yyval.t_func_decl_list)=NULL;
	 log_error(E_FATAL,"Syntax error in (:functions ...)");
	;}
    break;

  case 236:
#line 1143 "pddl+.yacc"
    {(yyval.t_con_goal) = (yyvsp[(3) - (4)].t_con_goal);;}
    break;

  case 237:
#line 1145 "pddl+.yacc"
    {yyerrok; (yyval.t_con_goal)=NULL;
      log_error(E_FATAL,"Syntax error in (:constraints ...)");
      ;}
    break;

  case 238:
#line 1152 "pddl+.yacc"
    {(yyval.t_con_goal) = (yyvsp[(3) - (4)].t_con_goal);;}
    break;

  case 239:
#line 1154 "pddl+.yacc"
    {yyerrok; (yyval.t_con_goal)=NULL;
      log_error(E_FATAL,"Syntax error in (:constraints ...)");
      ;}
    break;

  case 240:
#line 1160 "pddl+.yacc"
    { (yyval.t_structure_store)=(yyvsp[(1) - (2)].t_structure_store); (yyval.t_structure_store)->push_back((yyvsp[(2) - (2)].t_structure_def)); ;}
    break;

  case 241:
#line 1161 "pddl+.yacc"
    { (yyval.t_structure_store)= new structure_store; (yyval.t_structure_store)->push_back((yyvsp[(1) - (1)].t_structure_def)); ;}
    break;

  case 242:
#line 1165 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_action_def); ;}
    break;

  case 243:
#line 1166 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_event_def); requires(E_TIME); ;}
    break;

  case 244:
#line 1167 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_process_def); requires(E_TIME); ;}
    break;

  case 245:
#line 1168 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_durative_action_def); requires(E_DURATIVE_ACTIONS); ;}
    break;

  case 246:
#line 1169 "pddl+.yacc"
    { (yyval.t_structure_def)= (yyvsp[(1) - (1)].t_derivation_rule); requires(E_DERIVED_PREDICATES);;}
    break;

  case 247:
#line 1173 "pddl+.yacc"
    {(yyval.t_dummy)= 0; 
    	current_analysis->var_tab_stack.push(
    					current_analysis->buildRuleTab());;}
    break;

  case 248:
#line 1184 "pddl+.yacc"
    {(yyval.t_derivation_rule) = new derivation_rule((yyvsp[(3) - (5)].t_proposition),(yyvsp[(4) - (5)].t_goal),current_analysis->var_tab_stack.pop());;}
    break;

  case 249:
#line 1196 "pddl+.yacc"
    { (yyval.t_action_def)= current_analysis->buildAction(current_analysis->op_tab.symbol_put((yyvsp[(3) - (12)].cp)),
			(yyvsp[(6) - (12)].t_var_symbol_list),(yyvsp[(9) - (12)].t_goal),(yyvsp[(11) - (12)].t_effect_lists),
			current_analysis->var_tab_stack.pop()); delete [] (yyvsp[(3) - (12)].cp); ;}
    break;

  case 250:
#line 1200 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in action declaration.");
	 (yyval.t_action_def)= NULL; ;}
    break;

  case 251:
#line 1213 "pddl+.yacc"
    {(yyval.t_event_def)= current_analysis->buildEvent(current_analysis->op_tab.symbol_put((yyvsp[(3) - (12)].cp)),
		   (yyvsp[(6) - (12)].t_var_symbol_list),(yyvsp[(9) - (12)].t_goal),(yyvsp[(11) - (12)].t_effect_lists),
		   current_analysis->var_tab_stack.pop()); delete [] (yyvsp[(3) - (12)].cp);;}
    break;

  case 252:
#line 1218 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in event declaration.");
	 (yyval.t_event_def)= NULL; ;}
    break;

  case 253:
#line 1230 "pddl+.yacc"
    {(yyval.t_process_def)= current_analysis->buildProcess(current_analysis->op_tab.symbol_put((yyvsp[(3) - (12)].cp)),
		     (yyvsp[(6) - (12)].t_var_symbol_list),(yyvsp[(9) - (12)].t_goal),(yyvsp[(11) - (12)].t_effect_lists),
                     current_analysis->var_tab_stack.pop()); delete [] (yyvsp[(3) - (12)].cp);;}
    break;

  case 254:
#line 1234 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in process declaration.");
	 (yyval.t_process_def)= NULL; ;}
    break;

  case 255:
#line 1246 "pddl+.yacc"
    { (yyval.t_durative_action_def)= (yyvsp[(10) - (11)].t_durative_action_def);
      (yyval.t_durative_action_def)->name= current_analysis->op_tab.symbol_put((yyvsp[(3) - (11)].cp));
      (yyval.t_durative_action_def)->symtab= current_analysis->var_tab_stack.pop();
      (yyval.t_durative_action_def)->parameters= (yyvsp[(6) - (11)].t_var_symbol_list);
      (yyval.t_durative_action_def)->dur_constraint= (yyvsp[(9) - (11)].t_goal); 
      delete [] (yyvsp[(3) - (11)].cp);
    ;}
    break;

  case 256:
#line 1255 "pddl+.yacc"
    {yyerrok; 
	 log_error(E_FATAL,"Syntax error in durative-action declaration.");
	 (yyval.t_durative_action_def)= NULL; ;}
    break;

  case 257:
#line 1262 "pddl+.yacc"
    {(yyval.t_durative_action_def)=(yyvsp[(1) - (3)].t_durative_action_def); (yyval.t_durative_action_def)->effects=(yyvsp[(3) - (3)].t_effect_lists);;}
    break;

  case 258:
#line 1264 "pddl+.yacc"
    {(yyval.t_durative_action_def)=(yyvsp[(1) - (3)].t_durative_action_def); (yyval.t_durative_action_def)->precondition=(yyvsp[(3) - (3)].t_goal);;}
    break;

  case 259:
#line 1265 "pddl+.yacc"
    {(yyval.t_durative_action_def)= current_analysis->buildDurativeAction();;}
    break;

  case 260:
#line 1270 "pddl+.yacc"
    { (yyval.t_goal)=(yyvsp[(1) - (1)].t_goal); ;}
    break;

  case 261:
#line 1272 "pddl+.yacc"
    { (yyval.t_goal)= new conj_goal((yyvsp[(3) - (4)].t_goal_list)); ;}
    break;

  case 262:
#line 1277 "pddl+.yacc"
    { (yyval.t_goal_list)=(yyvsp[(1) - (2)].t_goal_list); (yyval.t_goal_list)->push_back((yyvsp[(2) - (2)].t_goal)); ;}
    break;

  case 263:
#line 1279 "pddl+.yacc"
    { (yyval.t_goal_list)= new goal_list; ;}
    break;

  case 264:
#line 1284 "pddl+.yacc"
    {(yyval.t_goal)= new timed_goal((yyvsp[(3) - (4)].t_goal),E_AT_START);;}
    break;

  case 265:
#line 1286 "pddl+.yacc"
    {(yyval.t_goal)= new timed_goal((yyvsp[(3) - (4)].t_goal),E_AT_END);;}
    break;

  case 266:
#line 1288 "pddl+.yacc"
    {(yyval.t_goal)= new timed_goal((yyvsp[(3) - (4)].t_goal),E_OVER_ALL);;}
    break;

  case 267:
#line 1290 "pddl+.yacc"
    {timed_goal * tg = dynamic_cast<timed_goal *>((yyvsp[(4) - (5)].t_goal));
		(yyval.t_goal) = new timed_goal(new preference((yyvsp[(3) - (5)].cp),tg->clearGoal()),tg->getTime());
			delete tg;
			requires(E_PREFERENCES);;}
    break;

  case 268:
#line 1295 "pddl+.yacc"
    {(yyval.t_goal) = new preference((yyvsp[(3) - (4)].t_goal));requires(E_PREFERENCES);;}
    break;

  case 269:
#line 1299 "pddl+.yacc"
    {(yyval.t_dummy)= 0; current_analysis->var_tab_stack.push(
    				current_analysis->buildOpTab());;}
    break;

  case 270:
#line 1304 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_EQUALITY;;}
    break;

  case 271:
#line 1305 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_STRIPS;;}
    break;

  case 272:
#line 1307 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_TYPING;;}
    break;

  case 273:
#line 1309 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_NEGATIVE_PRECONDITIONS;;}
    break;

  case 274:
#line 1311 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_DISJUNCTIVE_PRECONDS;;}
    break;

  case 275:
#line 1312 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_EXT_PRECS;;}
    break;

  case 276:
#line 1313 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_UNIV_PRECS;;}
    break;

  case 277:
#line 1314 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_COND_EFFS;;}
    break;

  case 278:
#line 1315 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_NFLUENTS | E_OFLUENTS;;}
    break;

  case 279:
#line 1317 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_DURATIVE_ACTIONS;;}
    break;

  case 280:
#line 1318 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_TIME |
                      E_NFLUENTS |
                      E_DURATIVE_ACTIONS; ;}
    break;

  case 281:
#line 1321 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)=E_ACTIONCOSTS | E_NFLUENTS;;}
    break;

  case 282:
#line 1324 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)=E_OFLUENTS;;}
    break;

  case 283:
#line 1325 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)=E_NFLUENTS;;}
    break;

  case 284:
#line 1327 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_STRIPS |
		      E_TYPING | 
		      E_NEGATIVE_PRECONDITIONS |
		      E_DISJUNCTIVE_PRECONDS |
		      E_EQUALITY |
		      E_EXT_PRECS |
		      E_UNIV_PRECS |
		      E_COND_EFFS;;}
    break;

  case 285:
#line 1336 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_EXT_PRECS |
		      E_UNIV_PRECS;;}
    break;

  case 286:
#line 1340 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_DURATION_INEQUALITIES;;}
    break;

  case 287:
#line 1343 "pddl+.yacc"
    {(yyval.t_pddl_req_flag)= E_CONTINUOUS_EFFECTS;;}
    break;

  case 288:
#line 1345 "pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_DERIVED_PREDICATES;;}
    break;

  case 289:
#line 1347 "pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_TIMED_INITIAL_LITERALS;;}
    break;

  case 290:
#line 1349 "pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_PREFERENCES;;}
    break;

  case 291:
#line 1351 "pddl+.yacc"
    {(yyval.t_pddl_req_flag) = E_CONSTRAINTS;;}
    break;

  case 292:
#line 1353 "pddl+.yacc"
    {log_error(E_WARNING,"Unrecognised requirements declaration ");
       (yyval.t_pddl_req_flag)= 0; delete [] (yyvsp[(1) - (1)].cp);;}
    break;

  case 293:
#line 1359 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(3) - (4)].t_const_symbol_list);;}
    break;

  case 294:
#line 1363 "pddl+.yacc"
    {(yyval.t_type_list)=(yyvsp[(3) - (4)].t_type_list); requires(E_TYPING);;}
    break;

  case 295:
#line 1373 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(11) - (12)].t_problem); (yyval.t_problem)->name = (yyvsp[(5) - (12)].cp); (yyval.t_problem)->domain_name = (yyvsp[(9) - (12)].cp);
		if (types_used && !types_defined) {
			yyerrok; log_error(E_FATAL,"Syntax error in problem file - types used, but no :types section in domain file."); 
		}

	;}
    break;

  case 296:
#line 1380 "pddl+.yacc"
    {yyerrok; (yyval.t_problem)=NULL;
       	log_error(E_FATAL,"Syntax error in problem definition."); ;}
    break;

  case 297:
#line 1386 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->req= (yyvsp[(1) - (2)].t_pddl_req_flag);;}
    break;

  case 298:
#line 1387 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->objects= (yyvsp[(1) - (2)].t_const_symbol_list);;}
    break;

  case 299:
#line 1388 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->initial_state= (yyvsp[(1) - (2)].t_effect_lists);;}
    break;

  case 300:
#line 1389 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->the_goal= (yyvsp[(1) - (2)].t_goal);;}
    break;

  case 301:
#line 1391 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->constraints = (yyvsp[(1) - (2)].t_con_goal);;}
    break;

  case 302:
#line 1392 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->metric= (yyvsp[(1) - (2)].t_metric);;}
    break;

  case 303:
#line 1393 "pddl+.yacc"
    {(yyval.t_problem)=(yyvsp[(2) - (2)].t_problem); (yyval.t_problem)->length= (yyvsp[(1) - (2)].t_length_spec);;}
    break;

  case 304:
#line 1394 "pddl+.yacc"
    {(yyval.t_problem)=new problem;;}
    break;

  case 305:
#line 1397 "pddl+.yacc"
    {(yyval.t_const_symbol_list)=(yyvsp[(3) - (4)].t_const_symbol_list);;}
    break;

  case 306:
#line 1400 "pddl+.yacc"
    {(yyval.t_effect_lists)=(yyvsp[(3) - (4)].t_effect_lists);;}
    break;

  case 307:
#line 1403 "pddl+.yacc"
    {(yyval.vtab) = current_analysis->buildOpTab();;}
    break;

  case 308:
#line 1406 "pddl+.yacc"
    {(yyval.t_goal)=(yyvsp[(3) - (4)].t_goal);delete (yyvsp[(2) - (4)].vtab);;}
    break;

  case 309:
#line 1411 "pddl+.yacc"
    { (yyval.t_metric)= new metric_spec((yyvsp[(3) - (5)].t_optimization),(yyvsp[(4) - (5)].t_expression)); ;}
    break;

  case 310:
#line 1413 "pddl+.yacc"
    {yyerrok; 
        log_error(E_FATAL,"Syntax error in metric declaration.");
        (yyval.t_metric)= NULL; ;}
    break;

  case 311:
#line 1420 "pddl+.yacc"
    {(yyval.t_length_spec)= new length_spec(E_BOTH,(yyvsp[(4) - (7)].ival),(yyvsp[(6) - (7)].ival));;}
    break;

  case 312:
#line 1423 "pddl+.yacc"
    {(yyval.t_length_spec) = new length_spec(E_SERIAL,(yyvsp[(4) - (5)].ival));;}
    break;

  case 313:
#line 1427 "pddl+.yacc"
    {(yyval.t_length_spec) = new length_spec(E_PARALLEL,(yyvsp[(4) - (5)].ival));;}
    break;

  case 314:
#line 1433 "pddl+.yacc"
    {(yyval.t_optimization)= E_MINIMIZE;;}
    break;

  case 315:
#line 1434 "pddl+.yacc"
    {(yyval.t_optimization)= E_MAXIMIZE;;}
    break;

  case 316:
#line 1439 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(2) - (3)].t_expression);;}
    break;

  case 317:
#line 1440 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_func_term);;}
    break;

  case 318:
#line 1441 "pddl+.yacc"
    {(yyval.t_expression)= (yyvsp[(1) - (1)].t_num_expression);;}
    break;

  case 319:
#line 1442 "pddl+.yacc"
    { (yyval.t_expression)= new special_val_expr(E_TOTAL_TIME); ;}
    break;

  case 320:
#line 1444 "pddl+.yacc"
    {(yyval.t_expression) = new violation_term((yyvsp[(3) - (4)].cp));;}
    break;

  case 321:
#line 1445 "pddl+.yacc"
    { (yyval.t_expression)= new special_val_expr(E_TOTAL_TIME); ;}
    break;

  case 322:
#line 1449 "pddl+.yacc"
    { (yyval.t_expression)= new plus_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 323:
#line 1450 "pddl+.yacc"
    { (yyval.t_expression)= new minus_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 324:
#line 1451 "pddl+.yacc"
    { (yyval.t_expression)= new mul_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 325:
#line 1452 "pddl+.yacc"
    { (yyval.t_expression)= new div_expression((yyvsp[(2) - (3)].t_expression),(yyvsp[(3) - (3)].t_expression)); ;}
    break;

  case 326:
#line 1456 "pddl+.yacc"
    {(yyval.t_expression) = (yyvsp[(1) - (1)].t_expression);;}
    break;

  case 327:
#line 1458 "pddl+.yacc"
    {(yyval.t_expression) = new plus_expression((yyvsp[(1) - (2)].t_expression),(yyvsp[(2) - (2)].t_expression));;}
    break;

  case 328:
#line 1462 "pddl+.yacc"
    {(yyval.t_expression) = (yyvsp[(1) - (1)].t_expression);;}
    break;

  case 329:
#line 1464 "pddl+.yacc"
    {(yyval.t_expression) = new mul_expression((yyvsp[(1) - (2)].t_expression),(yyvsp[(2) - (2)].t_expression));;}
    break;

  case 330:
#line 1470 "pddl+.yacc"
    {(yyval.t_plan)= (yyvsp[(2) - (2)].t_plan); 
         (yyval.t_plan)->push_front((yyvsp[(1) - (2)].t_step)); ;}
    break;

  case 331:
#line 1473 "pddl+.yacc"
    {(yyval.t_plan) = (yyvsp[(3) - (3)].t_plan);(yyval.t_plan)->insertTime((yyvsp[(2) - (3)].fval));;}
    break;

  case 332:
#line 1475 "pddl+.yacc"
    {(yyval.t_plan) = (yyvsp[(3) - (3)].t_plan);(yyval.t_plan)->insertTime((yyvsp[(2) - (3)].ival));;}
    break;

  case 333:
#line 1477 "pddl+.yacc"
    {(yyval.t_plan)= new plan;;}
    break;

  case 334:
#line 1482 "pddl+.yacc"
    {(yyval.t_step)=(yyvsp[(3) - (3)].t_step); 
         (yyval.t_step)->start_time_given=1; 
         (yyval.t_step)->start_time=(yyvsp[(1) - (3)].fval);;}
    break;

  case 335:
#line 1486 "pddl+.yacc"
    {(yyval.t_step)=(yyvsp[(1) - (1)].t_step);
	 (yyval.t_step)->start_time_given=0;;}
    break;

  case 336:
#line 1492 "pddl+.yacc"
    {(yyval.t_step)= (yyvsp[(1) - (4)].t_step); 
	 (yyval.t_step)->duration_given=1;
         (yyval.t_step)->duration= (yyvsp[(3) - (4)].fval);;}
    break;

  case 337:
#line 1496 "pddl+.yacc"
    {(yyval.t_step)= (yyvsp[(1) - (1)].t_step);
         (yyval.t_step)->duration_given=0;;}
    break;

  case 338:
#line 1502 "pddl+.yacc"
    {(yyval.t_step)= new plan_step( 
              current_analysis->op_tab.symbol_get((yyvsp[(2) - (4)].cp)), 
	      (yyvsp[(3) - (4)].t_const_symbol_list)); delete [] (yyvsp[(2) - (4)].cp);
      ;}
    break;

  case 339:
#line 1509 "pddl+.yacc"
    {(yyval.fval)= (yyvsp[(1) - (1)].fval);;}
    break;

  case 340:
#line 1510 "pddl+.yacc"
    {(yyval.fval)= (float) (yyvsp[(1) - (1)].ival);;}
    break;


/* Line 1267 of yacc.c.  */
#line 4334 "pddl+.cpp"
      default: break;
    }
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;


  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (YY_("syntax error"));
#else
      {
	YYSIZE_T yysize = yysyntax_error (0, yystate, yychar);
	if (yymsg_alloc < yysize && yymsg_alloc < YYSTACK_ALLOC_MAXIMUM)
	  {
	    YYSIZE_T yyalloc = 2 * yysize;
	    if (! (yysize <= yyalloc && yyalloc <= YYSTACK_ALLOC_MAXIMUM))
	      yyalloc = YYSTACK_ALLOC_MAXIMUM;
	    if (yymsg != yymsgbuf)
	      YYSTACK_FREE (yymsg);
	    yymsg = (char *) YYSTACK_ALLOC (yyalloc);
	    if (yymsg)
	      yymsg_alloc = yyalloc;
	    else
	      {
		yymsg = yymsgbuf;
		yymsg_alloc = sizeof yymsgbuf;
	      }
	  }

	if (0 < yysize && yysize <= yymsg_alloc)
	  {
	    (void) yysyntax_error (yymsg, yystate, yychar);
	    yyerror (yymsg);
	  }
	else
	  {
	    yyerror (YY_("syntax error"));
	    if (yysize != 0)
	      goto yyexhaustedlab;
	  }
      }
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse look-ahead token after an
	 error, discard it.  */

      if (yychar <= YYEOF)
	{
	  /* Return failure if at end of input.  */
	  if (yychar == YYEOF)
	    YYABORT;
	}
      else
	{
	  yydestruct ("Error: discarding",
		      yytoken, &yylval);
	  yychar = YYEMPTY;
	}
    }

  /* Else will try to reuse look-ahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule which action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;	/* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (yyn != YYPACT_NINF)
	{
	  yyn += YYTERROR;
	  if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
	    {
	      yyn = yytable[yyn];
	      if (0 < yyn)
		break;
	    }
	}

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
	YYABORT;


      yydestruct ("Error: popping",
		  yystos[yystate], yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  *++yyvsp = yylval;


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#ifndef yyoverflow
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEOF && yychar != YYEMPTY)
     yydestruct ("Cleanup: discarding lookahead",
		 yytoken, &yylval);
  /* Do not reclaim the symbols of the rule which action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
		  yystos[*yyssp], yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  /* Make sure YYID is used.  */
  return YYID (yyresult);
}


#line 1513 "pddl+.yacc"


#include <cstdio>
#include <iostream>
int line_no= 1;
using std::istream;
#include "lex.yy.cc"

namespace VAL1_2 {
extern yyFlexLexer* yfl;
};


int yyerror(char * s)
{
    return 0;
}

int yylex()
{
    return yfl->yylex();
}

