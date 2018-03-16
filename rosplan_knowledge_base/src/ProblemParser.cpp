#include "rosplan_knowledge_base/ProblemParser.h"

/* implementation of ProblemParser.h */
namespace KCL_rosplan {

    /*-----------------------*/
    /* parsing problem */
    /*-----------------------*/

    /**
     * parse the problem file
     */
    VAL::problem* ProblemParser::parseProblem(const std::string ProblemPath) {

        // only parse Problem once
        if(problem_parsed) return problem;
        problem_parsed = true;

        std::string ProblemFileName = (ProblemPath);
        ROS_INFO("KCL: (KB) Parsing Problem File: %s.", ProblemFileName.c_str());

        // save filename for VAL
        std::vector<char> writable(ProblemFileName.begin(), ProblemFileName.end());
        writable.push_back('\0');
        current_filename = &writable[0];

        // parse Problem
        VAL::current_analysis = val_analysis = &VAL::an_analysis;  // use the same analysis got from the domain
        std::ifstream ProblemFile;
        ProblemFile.open(ProblemFileName.c_str());
        yydebug = 0;

        VAL::yfl = new yyFlexLexer;

        if (ProblemFile.bad()) {
            ROS_ERROR("KCL: (KB) Failed to open problem file.");
            line_no = 0;
            VAL::log_error(VAL::E_FATAL,"Failed to open file");
        } else {
            line_no = 1;
            VAL::yfl->switch_streams(&ProblemFile, &std::cout);
            yyparse();

            // Problem name
            problem = VAL::current_analysis->the_problem;
            //problem_name = problem->name;

        }
        delete VAL::yfl;
        ProblemFile.close();

        return problem;

    }
} // close namespace
