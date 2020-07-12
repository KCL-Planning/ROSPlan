#include "rosplan_planning_system/PlannerInterface/CFFPlannerInterface.h"

#include <ctime>
#include <string>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <algorithm>

using namespace KCL_rosplan;

namespace {

    typedef std::vector<std::string> OperatorParams;
    
    unsigned int split_string(const std::string &text, std::vector<std::string> &tokens, char separator) {
        size_t pos = text.find(separator);
        unsigned int initialPos = 0;
        tokens.clear();
                
        while(pos != std::string::npos && pos < text.length()) {
            if(text.substr(initialPos, pos - initialPos + 1) !=" ") {
                std::string s = text.substr(initialPos, pos - initialPos + 1);
                s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
                tokens.push_back(s);
            }
            initialPos = pos + 1;
            pos = text.find(separator, initialPos);
        }
        
        tokens.push_back(text.substr(initialPos, text.size() - initialPos));
        return tokens.size();
    }

    void extractElementsFromLine(const std::string &line, std::string &action_id, std::string &operator_name, OperatorParams &operator_params) {
                    
        // Example of actions of Contigent-FF:            
        //   17||0 --- FIND_OBJECT C1 ITEM_0 --- SON: 18||0
        //   18||0 --- PICKUP_OBJECT C1 ITEM_0 --- SON: 19||0
        //
            
        std::vector<std::string> tokens;                
        
        split_string(line, tokens, ' ');        
        action_id = tokens[0].substr(0, tokens[0].find("||"));
        operator_name = tokens[2];                    

        int idx = 3;
        while (idx < tokens.size() && tokens[idx] != "---") {
            operator_params.push_back(tokens[idx]);
            idx++;
        }
        
        ROS_DEBUG("KCL: (%s) Elements in line: { %s, %s, etc. }", ros::this_node::getName().c_str(), action_id.c_str(), operator_name.c_str());  
    }
    
    void createAction(const std::string &action_id, const std::string &operator_name, const OperatorParams &operator_params, std::string &action) {
        
        std::string operator_parameters;
        for (OperatorParams::const_iterator it = operator_params.begin(); it != operator_params.end(); ++it) {                        
            if(std::next(it) != operator_params.end()) {
                operator_parameters = operator_parameters + (*it) + " ";
            } else {
                operator_parameters = operator_parameters + (*it);
            }
        }
        
        action = action_id + ": "  + "(" + operator_name + " " + operator_parameters + ")  [0.001]\n";
        ROS_DEBUG("KCL: (%s) Action: %s", ros::this_node::getName().c_str(), action.c_str());  
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
        ROS_DEBUG("KCL: (%s) Writing problem to file: %s", ros::this_node::getName().c_str(), problem_name.c_str());
    
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
    
    std::string command_template = planner_command;
    
    std::size_t dit = command_template.find("DOMAIN");
    if(dit != std::string::npos) {
        command_template.replace(dit, 6, domain_path);
    }    
    
    std::size_t pit = command_template.find("PROBLEM");
    if(pit != std::string::npos) {
        command_template.replace(pit, 7, problem_path);
    }    
    
    std::string command = command_template + " > " + data_path + "plan.cff";

    ROS_INFO("KCL: (%s) (%s) Running planner: %s", ros::this_node::getName().c_str(), problem_name.c_str(),  command.c_str());
    std::string plan = runCommand(command.c_str());
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
    
    ROS_INFO("KCL: (%s) cff plan to POPF format", ros::this_node::getName().c_str());   
    
    bool isParseCompleted = false;   
    std::string line;     
    
    while(!plan_file.eof() && !isParseCompleted) {
        
        std::getline(plan_file, line);                    
        
        if (!line.empty()) {  
           if (!(line.compare("-------------------------------------------------") == 0)) {
        
                std::vector<std::string> tokens;                
                split_string(line, tokens, ' ');

                std::string action_id;
                std::string operator_name;                    
                OperatorParams operator_params;                
                extractElementsFromLine(line, action_id, operator_name, operator_params);
                
                std::string action; 
                createAction(action_id, operator_name, operator_params, action);
        
                planner_output += action;                
                ROS_INFO("KCL: (%s) Action: { %s }", ros::this_node::getName().c_str(), action.c_str());   
           }
        } else {
            isParseCompleted = true;
        }
    }    
    
    if (isParseCompleted)
    {
        std::transform(planner_output.begin(), planner_output.end(), planner_output.begin(), ::tolower);   
    }
    
    ROS_INFO("KCL: (%s) Plan converted to popf format: %d", ros::this_node::getName().c_str(), isParseCompleted);  
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
    
    bool solved = false;
    
    clearPreviousPlan();        
    saveProblemToFileIfNeeded();
    
    callExternalPlanner();        
    solved = parsePlan();            
    
    ROS_INFO("KCL: (%s) (%s) Was plan solved? %d", ros::this_node::getName().c_str(), problem_name.c_str(), solved);
    return solved;
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
