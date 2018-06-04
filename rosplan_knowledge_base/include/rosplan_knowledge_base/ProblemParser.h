#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "FlexLexer.h"
#include "ptree.h"
#include "VisitController.h"
#include "VALVisitorProblem.h"


#ifndef KCL_problemparser
#define KCL_problemparser

extern int yyparse();
extern int yydebug;

namespace VAL {
    extern analysis an_analysis;
    extern yyFlexLexer* yfl;
};

/**
 * Problem file storage and parsing. Uses VAL parser and storage.
 */
namespace KCL_rosplan {

    /*--------*/
    /* parser */
    /*--------*/

    class ProblemParser
    {
    private:

    public:

        /* VAL pointers */
        VAL::analysis * val_analysis;
        VAL::problem* problem;

        /* Problem information */
        bool problem_parsed;
        std::string problem_name;

        /* Problem parsing */
        VAL::problem* parseProblem(const std::string ProblemPath);


    };
}
#endif
