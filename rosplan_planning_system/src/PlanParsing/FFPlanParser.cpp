#include "rosplan_planning_system/FFPlanParser.h"

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>
#include <algorithm>
#include <ctype.h>
#include <string>

/* implementation of rosplan_planning_system::FFPlanParser.h */
namespace KCL_rosplan {
         
    /*---------------------*/
    /* string manipulation */
    /*---------------------*/
        
    namespace str_utils {

        void toLowerCase(std::string &str) {
            std::transform(str.begin(), str.end(), str.begin(), tolower);
        }

        unsigned int split(const std::string &txt, std::vector<std::string> &strs, char ch) {
            unsigned int pos = txt.find( ch );
            unsigned int initialPos = 0;
            strs.clear();
            // Decompose statement
            while( pos != std::string::npos && pos < txt.length()) {
                if(txt.substr( initialPos, pos - initialPos + 1 ) !=" ") {
                    std::string s = txt.substr( initialPos, pos - initialPos + 1 );
                    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
                    strs.push_back(s);
                }
                initialPos = pos + 1;
                pos = txt.find( ch, initialPos );
            }
            // Add the last one
            strs.push_back( txt.substr( initialPos, txt.size() - initialPos ) );
            return strs.size();
        }

    }

    /*---------------------*/
    /* Fast Forward parser */
    /*---------------------*/
    
    FFPlanParser::~FFPlanParser() {}

    void FFPlanParser::reset() {
        filter_objects.clear();
        filter_attributes.clear();
        knowledge_filter.clear();
        action_list.clear();
    }
    
    /**
    * processes the parameters of a single PDDL action into an ActionDispatch message
    */
    void FFPlanParser::processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params, PlanningEnvironment &environment) {
        
        // find the correct PDDL operator definition
        std::map<std::string,std::vector<std::string> >::iterator ait;
        ait = environment.domain_operators.find(msg.name);
        if(ait != environment.domain_operators.end()) {

            // add the PDDL parameters to the action dispatch
            for(size_t i=0; i<ait->second.size(); i++) {
                diagnostic_msgs::KeyValue pair;
                pair.key = ait->second[i];
                pair.value = params[i];
                msg.parameters.push_back(pair);

                // prepare object existence for the knowledge filter
                bool toBeAdded = true;
                for(size_t j=0; j<filter_objects.size(); j++)
                    if(0==filter_objects[j].compare(params[i])) toBeAdded = false;
                if(toBeAdded) filter_objects.push_back(params[i]);
            }

            // prepare object attributes for the knowledge filter
            for(size_t i=0; i<environment.domain_operator_precondition_map[msg.name].size(); i++) {
                std::vector<std::string> filterAttribute;
                std::vector<std::string> precondition = environment.domain_operator_precondition_map[msg.name][i];
                filterAttribute.push_back(precondition[0]);
                for(size_t j=1; j<precondition.size(); j++) {
                    if(j>1) filterAttribute.push_back(precondition[j]);
                    // instance name
                    for(size_t k=0;k<ait->second.size();k++) {
                        if(0==ait->second[k].compare(precondition[j]))
                            filterAttribute.push_back(params[k]);
                    }
                }
                filter_attributes.push_back(filterAttribute);
            }
        } // end of operator
    }

    /**
    * Parse a plan written by FF
    */
    void FFPlanParser::preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) {

        ROS_INFO("KCL: (FFPlanParser) Loading plan from file: %s. Initial action ID: %zu", dataPath.c_str(), freeActionID);
                
        std::string filePath;

        if(dataPath.rfind("/") == dataPath.length()-1) {
            filePath = dataPath + "plan.pddl";
        }
        else {
            filePath = dataPath;
        }
        
        std::ifstream infile(filePath.c_str());
        std::string line;
        bool isPlanParsed = false;        
        bool isPlanFound = false;       
        size_t planActionId = 0;
        planActionId = freeActionID;
                
        reset();
        while(!infile.eof() && !isPlanParsed) {

            std::getline(infile, line);

            std::string whitespaces(" \t\f\v\n\r");
            line.erase(line.find_last_not_of(whitespaces)+1);      
            str_utils::toLowerCase(line);

            // search actions of the plan
            if(!isPlanFound) {
                // loop until plan is printed             
                if(line.compare("ff: found legal plan as follows") != 0) {
                    continue;                    
                }                

                // remove empty lines after plan header
                bool isScanning = true;
                std::streampos lastPos;                
                while(isScanning) {                    
                    lastPos = infile.tellg();                
                    std::getline(infile, line);
                    str_utils::toLowerCase(line);                                        
                    isScanning = !infile.eof() && line.empty();                    
                }
                
                // plan is found
                if(!infile.eof()) {
                    isPlanFound = true;                
                    infile.seekg(lastPos);
                }
            }            
            // parse plan actions once header is found                    
            else {
                // plan action
                if(!line.empty()) {

                    rosplan_dispatch_msgs::ActionDispatch msg;
                    std::vector<std::string> s;
                    size_t idx = 0;
                                        
                    // actions look like this:
                    // step    0: got_place C1
                    //         1: find_object V1 C1 
                    str_utils::split(line, s, ' ');
                    if(s[0] == "step") { idx = 1; }

                    unsigned int action_id = std::atoi(s[idx].substr(0,s[idx].size()-1).c_str());
                    std::string operator_name = s[idx+1];                    

                    msg.action_id = planActionId;
                    msg.name = operator_name;
                    msg.dispatch_time = 0.0;

                    // collect parameters
                    std::vector<std::string> params;
                    for(size_t pdx=idx+2; pdx<s.size(); pdx++) {                        
                        params.push_back(s[pdx]);
                        pdx++;
                    }
                    
                    if(params.size() > 0) {
                        processPDDLParameters(msg, params, environment);
                    }
                    
                    action_list.push_back(msg);
                    planActionId++;
                    
                }
                // no more actions, so parsing is done
                else                    
                {
                    isPlanParsed = true;
                }
            }
            
        }

        infile.close();
    }

    /**
    * populates the knowledge filter messages
    */
    void FFPlanParser::generateFilter(PlanningEnvironment &environment) {

        knowledge_filter.clear();

		// populate filter message with objects
		for(size_t i=0; i<filter_objects.size(); i++) {
			rosplan_knowledge_msgs::KnowledgeItem filterItem;
			filterItem.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			filterItem.instance_type = environment.object_type_map[filter_objects[i]];
			filterItem.instance_name = filter_objects[i];
			knowledge_filter.push_back(filterItem);
		}

		// populate filter message with attributes, only statics, not all preconditions.
		for(size_t i=0; i<filter_attributes.size(); i++) {
			rosplan_knowledge_msgs::KnowledgeItem filterItem;
			filterItem.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			filterItem.attribute_name = filter_attributes[i][0];
			if (filter_attributes[i].size() > 1)
			{
				filterItem.instance_type = environment.object_type_map[filter_attributes[i][1]];
				filterItem.instance_name = filter_attributes[i][1];
				for(size_t j=2; j<filter_attributes[i].size()-1; j+=2) {
					diagnostic_msgs::KeyValue pair;
					pair.key = filter_attributes[i][j];
					pair.value = filter_attributes[i][j+1];
					filterItem.values.push_back(pair);
				}
			} 
			knowledge_filter.push_back(filterItem);
		}
	}

} // close namespace
