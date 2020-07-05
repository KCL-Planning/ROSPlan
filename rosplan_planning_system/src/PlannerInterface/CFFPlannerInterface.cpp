#include "rosplan_planning_system/PlannerInterface/CFFPlannerInterface.h"

#include <fstream>
#include <sstream>
#include <ctime>
#include <string>
#include <streambuf>

using namespace KCL_rosplan;

CFFPlannerInterface::CFFPlannerInterface(ros::NodeHandle& nh)
{
    node_handle = &nh;

    plan_server = new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>((*node_handle), "start_planning", boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false);

    std::string plannerTopic = "planner_output";
    node_handle->getParam("planner_topic", plannerTopic);
    plan_publisher = node_handle->advertise<std_msgs::String>(plannerTopic, 1, true);

    node_handle->param<bool>("use_ffha", this->use_ffha, false);

    plan_server->start();
}

CFFPlannerInterface::~CFFPlannerInterface()
{
    delete plan_server;
}

void CFFPlannerInterface::clearPreviousPlan() {
    planner_output.clear();    
}

void CFFPlannerInterface::saveProblemToFileIfNeeded() {
    if(use_problem_topic && problem_instance_received) {
        ROS_INFO("KCL: (%s) (%s) Writing problem to file.", ros::this_node::getName().c_str(), problem_name.c_str());
    
        std::ofstream dest;
        dest.open((problem_path).c_str());
        dest << problem_instance;
        dest.close();
    }
}

std::string CFFPlannerInterface::runCommand(std::string cmd) {
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
    std::string commandString = str + " > " + data_path + "plan.pddl";

    // call the planer
    ROS_INFO("KCL: (%s) (%s) Running: %s", ros::this_node::getName().c_str(), problem_name.c_str(),  commandString.c_str());
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

    return solved;
}

void CFFPlannerInterface::convertPlanToPopfFormat(std::ifstream &plan_file) {              
    std::string line;
            
    if (this->use_ffha) {
        std::getline(plan_file, line); //go to first action line
    } else {
    // actions look like this:
    // step    0: got_place C1
    //         1: find_object V1 C1
    // plan cost: XX
        while (std::getline(plan_file, line)) { // Move to the beginning of the plan
            if (line.substr(0, 4) == "step") {
                line = line.substr(4); // Remove the step
                break;
            }
        }
    }

    // First iteration line will be like   0: got_place C1
    while (line.find("Total cost") == line.npos and line.find("time spend") == line.npos and line.size() > 0) {
        std::stringstream ss(line); // To trim whitespaces
        std::string aux;
        // Read the action number X:
        ss >> aux;
        std::string add = (use_ffha) ? " " : " (";
        planner_output += aux + add; // Add parenthesis before the action
        while (ss >> aux) { // Read the rest
            planner_output += aux;
            if (ss.good()) planner_output += " "; // Add a whitespace unless we have processed all the line
        }
        add = (use_ffha) ? "  [0.001]\n" : ")  [0.001]\n";
        planner_output += add; // Close parenthesis and add duration
        std::getline(plan_file, line);
    }
    // Convert to lowercase as FF prints all the actions and parameters in uppercase
    std::transform(planner_output.begin(), planner_output.end(), planner_output.begin(), ::tolower);    
}

bool CFFPlannerInterface::parsePlan() {       
    std::ifstream plan_file;
    plan_file.open((data_path + "plan.pddl").c_str());
    
    bool solved = isPlanSolved(plan_file);
    if (solved) {
        convertPlanToPopfFormat(plan_file);
    }
    
    plan_file.close();        
}

bool CFFPlannerInterface::runPlanner() {        
    bool success = false;
    
    clearPreviousPlan();        
    saveProblemToFileIfNeeded();
    
    callExternalPlanner();        
    success = parsePlan();            
    
    if(!success) ROS_INFO("KCL: (%s) (%s) Plan was unsolvable.", ros::this_node::getName().c_str(), problem_name.c_str());
    else ROS_INFO("KCL: (%s) (%s) Plan was solved.", ros::this_node::getName().c_str(), problem_name.c_str());

    return success;
}

int main(int argc, char **argv) {

    srand (static_cast <unsigned> (time(0)));

    ros::init(argc,argv,"rosplan_planner_interface");
    ros::NodeHandle nh("~");

    KCL_rosplan::CFFPlannerInterface pi(nh);

    // subscribe to problem instance
    std::string problemTopic = "problem_instance";
    nh.getParam("problem_topic", problemTopic);
    ros::Subscriber problem_sub = nh.subscribe(problemTopic, 1, &KCL_rosplan::PlannerInterface::problemCallback, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

    // start the planning services
    ros::ServiceServer service1 = nh.advertiseService("planning_server", &KCL_rosplan::PlannerInterface::runPlanningServerDefault, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));
    ros::ServiceServer service2 = nh.advertiseService("planning_server_params", &KCL_rosplan::PlannerInterface::runPlanningServerParams, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
