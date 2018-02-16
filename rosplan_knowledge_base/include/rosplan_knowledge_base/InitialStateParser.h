#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "FlexLexer.h"
#include "ptree.h"
#include "VisitController.h"
#include "VALVisitorProblem.h"


#ifndef KCL_initialStateparser
#define KCL_initialStateparser

extern int yyparse();
extern int yydebug;

namespace VAL {
    extern analysis an_analysis;
    extern yyFlexLexer* yfl;
};

/**
 * Initial State storage and parsing. Uses VAL parser and storage.
 */
namespace KCL_rosplan {

    /*--------*/
    /* parser */
    /*--------*/

    class InitialStateParser
    {
    private:

    public:

        /* VAL pointers */
        VAL::analysis * val_analysis;
        VAL::problem* problem;

        /* initialState information */
        bool initialState_parsed;
        std::string initialState_name;

        /* initialState parsing */
        VAL::problem* parseInitialState(const std::string initialStatePath);


    };
}
#endif
