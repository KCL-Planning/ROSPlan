#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "FlexLexer.h"
#include "VALfiles/ptree.h"
#include "VisitController.h"
#include "VALVisitorProblem.h"


#ifndef KCL_problemparser
#define KCL_problemparser


/**
 * Problem file storage and parsing. Uses VAL parser and storage.
 */
namespace KCL_rosplan {

    /*--------*/
    /* parser */
    /*--------*/

    class PDDLProblemParser
    {
    private:

    public:

        /* VAL pointers */
        VAL1_2::analysis * val_analysis;
        VAL1_2::problem* problem;

        /* Problem information */
        bool problem_parsed;
        std::string problem_name;

        /* Problem parsing */
        VAL1_2::problem* parseProblem(const std::string ProblemPath);


    };
}
#endif
