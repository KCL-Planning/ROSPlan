#include "rosplan_knowledge_base/InitialStateParser.h"

/* implementation of InitialStateParser.h */
namespace KCL_rosplan {

    /*-----------------------*/
    /* parsing initial state */
    /*-----------------------*/

    /**
     * parse the initial state file
     */
    VAL::problem* InitialStateParser::parseInitialState(const std::string InitialStatePath) {

        // only parse InitialState once
        if(initialState_parsed) return problem;
        initialState_parsed = true;

        std::string InitialStateFileName = (InitialStatePath);
        ROS_INFO("KCL: (KB) Parsing Initial State: %s.", InitialStateFileName.c_str());

        // save filename for VAL
        std::vector<char> writable(InitialStateFileName.begin(), InitialStateFileName.end());
        writable.push_back('\0');
        current_filename = &writable[0];

        // parse InitialState
        VAL::current_analysis = val_analysis = &VAL::an_analysis;  // use the same analysis got from the domain
        std::ifstream InitialStateFile;
        InitialStateFile.open(InitialStateFileName.c_str());
        yydebug = 0;

        VAL::yfl = new yyFlexLexer;

        if (InitialStateFile.bad()) {
            ROS_ERROR("KCL: (KB) Failed to open Initial State file.");
            line_no = 0;
            VAL::log_error(VAL::E_FATAL,"Failed to open file");
        } else {
            line_no = 1;
            VAL::yfl->switch_streams(&InitialStateFile, &std::cout);
            yyparse();

            // InitialState name
            problem = VAL::current_analysis->the_problem;
            //problem_name = problem->name;

        }
        delete VAL::yfl;
        InitialStateFile.close();

        return problem;

    }
} // close namespace
