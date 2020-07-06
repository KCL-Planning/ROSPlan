#include "rosplan_planning_system/PlannerInterface/CFFPlannerInterface.h"

#include <ctime>
#include <string>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <algorithm>

using namespace KCL_rosplan;

namespace {

    unsigned int split_string(const std::string &txt, std::vector<std::string> &strs, char ch) {
        size_t pos = txt.find( ch );
        unsigned int initialPos = 0;
        strs.clear();
                
        while( pos != std::string::npos && pos < txt.length()) {
            if(txt.substr( initialPos, pos - initialPos + 1 ) !=" ") {
                std::string s = txt.substr( initialPos, pos - initialPos + 1 );
                s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
                strs.push_back(s);
            }
            initialPos = pos + 1;
            pos = txt.find( ch, initialPos );
        }
        
        strs.push_back( txt.substr( initialPos, txt.size() - initialPos ) );
        return strs.size();
    }
}

CFFPlannerInterface::CFFPlannerInterface(ros::NodeHandle& nh) {
    
    node_handle = &nh;
    plan_server = 
        new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>((*node_handle), 
                                                                             "start_planning", 
                                                                             boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), 
                                                                             false);

    std::string plannerTopic = "planner_output";
    node_handle->getParam("planner_topic", plannerTopic);
    plan_publisher = node_handle->advertise<std_msgs::String>(plannerTopic, 1, true);

    plan_server->start();
}

CFFPlannerInterface::~CFFPlannerInterface() {
    delete plan_server;
}

void CFFPlannerInterface::clearPreviousPlan() {
    planner_output.clear();    
}

void CFFPlannerInterface::saveProblemToFileIfNeeded() {
    
    ROS_INFO("KCL: (%s) Need to write problem file: %d", ros::this_node::getName().c_str(), 
             use_problem_topic && problem_instance_received);
    
    if(use_problem_topic && problem_instance_received) {        
        ROS_INFO("KCL: (%s) Writing problem to file: %s", ros::this_node::getName().c_str(), problem_name.c_str());
    
        std::ofstream dest;
        dest.open((problem_path).c_str());
        dest << problem_instance;
        dest.close();
    }
}

std::string CFFPlannerInterface::runCommand(std::string cmd) {
    
    ROS_INFO("KCL: (%s) Command: %s", ros::this_node::getName().c_str(), cmd.c_str());
    
    std::string data;
    FILE *stream;
    char buffer[1000];
        
    stream = popen(cmd.c_str(), "r");
    while (fgets(buffer, 1000, stream) != NULL) {            
        data.append(buffer);
    }
    pclose(stream);
    
    return data;
}

void CFFPlannerInterface::callExternalPlanner() {      
    
    // prepare the planner command line
    std::string str = planner_command;
    std::size_t dit = str.find("DOMAIN");
    if(dit!=std::string::npos) str.replace(dit,6,domain_path);
    std::size_t pit = str.find("PROBLEM");
    if(pit!=std::string::npos) str.replace(pit,7,problem_path);
    std::string commandString = str + " > " + data_path + "plan.cff";

    // call the planer
    ROS_INFO("KCL: (%s) (%s) Running planner: %s", ros::this_node::getName().c_str(), problem_name.c_str(),  commandString.c_str());
    std::string plan = runCommand(commandString.c_str());
    ROS_INFO("KCL: (%s) (%s) Planning complete", ros::this_node::getName().c_str(), problem_name.c_str());
}

bool CFFPlannerInterface::isPlanSolved(std::ifstream &plan_file) {                

    bool solved = false;
    const std::string CFF_SOLVED_PLAN_PATTERN = "ff: found plan as follows";            
    
    std::string line;
    while (not solved and std::getline(plan_file, line)) {
        if (line.find(CFF_SOLVED_PLAN_PATTERN) != line.npos) {            
            solved = true;
        }
    }

    ROS_INFO("KCL: (%s) Is plan solved? %d", ros::this_node::getName().c_str(), solved);    
    return solved;
}


void CFFPlannerInterface::savePlanInPopfFormatToFile() {
    
    std::string plan_file = data_path + "plan.pddl";    
    ROS_INFO("KCL: (%s) Writing converted plan (pddl) to file: %s", ros::this_node::getName().c_str(), planner_output.c_str());

    std::ofstream dest;
    dest.open(plan_file.c_str());
    dest << planner_output;
    dest.close();
}

void CFFPlannerInterface::convertPlanToPopfFormat(std::ifstream &plan_file) {              
    
    ROS_INFO("KCL: (%s) cff plan to popf format", ros::this_node::getName().c_str());   
                
    std::string line;
    bool isPlanParsed = false;        
    
    while(!plan_file.eof() && !isPlanParsed) {
        
        std::getline(plan_file, line);                    
        
        if(!line.empty()) {
            
            // actions look like this:            
            //   17||0 --- FIND_OBJECT C1 ITEM_0 --- SON: 18||0
            //   18||0 --- PICKUP_OBJECT C1 ITEM_0 --- SON: 19||0
                        
            if (!(line.compare("-------------------------------------------------") == 0)) {
    
                // extract elements from line                
                std::vector<std::string> tokens;                
                split_string(line, tokens, ' ');
                
                std::string action_id = tokens[0].substr(0, tokens[0].find("||"));
                std::string operator_name = tokens[2];                    

                typedef std::vector<std::string> OperatorParams;
                OperatorParams params;
                
                int idx = 3;
                while (idx < tokens.size() && tokens[idx] != "---") {
                    params.push_back(tokens[idx]);
                    idx++;
                }
                
                // add action based on elements of the line
                std::string operator_parameters;
                for (OperatorParams::iterator it = params.begin(); it != params.end(); ++it) {                        
                    if(std::next(it) != params.end()) {
                        operator_parameters = operator_parameters + (*it) + " ";
                    } else {
                        operator_parameters = operator_parameters + (*it);
                    }
                }
                std::string action = action_id + ": " 
                    + "(" + operator_name + " " + operator_parameters + ") [0.001]\n";               

                planner_output += action;                
                ROS_INFO("KCL: (%s) Action: { %s }", ros::this_node::getName().c_str(), action.c_str());   
            }
        }
        else {
            isPlanParsed = true;
        }
    }    
    
    if (isPlanParsed)
    {
        std::transform(planner_output.begin(), planner_output.end(), planner_output.begin(), ::tolower);   
    }
    
    ROS_INFO("KCL: (%s) Plan converted to popf format: %d", ros::this_node::getName().c_str(), isPlanParsed);       
}

bool CFFPlannerInterface::parsePlan() {       
    
    bool solved = false;
        
    std::string file_path = data_path + "plan.cff";    
    ROS_INFO("KCL: (%s) Plan to parse: %s", ros::this_node::getName().c_str(), file_path.c_str());    
    
    std::ifstream plan_file;
    plan_file.open(file_path.c_str());    
        
    solved = isPlanSolved(plan_file);
    if (solved) {        
        convertPlanToPopfFormat(plan_file);
        savePlanInPopfFormatToFile();
    }    
    plan_file.close();        
    
    return solved;
}

bool CFFPlannerInterface::runPlanner() {        
    bool success = false;
    
    clearPreviousPlan();        
    saveProblemToFileIfNeeded();
    
    callExternalPlanner();        
    success = parsePlan();            
    
    if(!success) { 
        ROS_INFO("KCL: (%s) (%s) Plan was unsolvable.", ros::this_node::getName().c_str(), problem_name.c_str());
    }
    else {
        ROS_INFO("KCL: (%s) (%s) Plan was solved.", ros::this_node::getName().c_str(), problem_name.c_str());
    }

    return success;
}

int main(int argc, char **argv) {

    srand(static_cast <unsigned> (time(0)));

    ros::init(argc,argv,"rosplan_planner_interface");
    ros::NodeHandle nh("~");
    KCL_rosplan::CFFPlannerInterface pi(nh);

    std::string problem_topic_name = "problem_instance";
    nh.getParam("problem_topic", problem_topic_name);
    
    ros::Subscriber problem_subcriber 
        = nh.subscribe(problem_topic_name, 1, &KCL_rosplan::PlannerInterface::problemCallback, 
                       dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

    ros::ServiceServer planning_service 
        = nh.advertiseService("planning_server", &KCL_rosplan::PlannerInterface::runPlanningServerDefault, 
                              dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));
    ros::ServiceServer planning_with_params_service 
        = nh.advertiseService("planning_server_params", &KCL_rosplan::PlannerInterface::runPlanningServerParams, 
                              dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
