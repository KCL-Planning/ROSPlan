#include "rosplan_planning_system/PlannerInterface/TFDPlannerInterface.h"

namespace KCL_rosplan {

    /*-------------*/
    /* constructor */
    /*-------------*/

    TFDPlannerInterface::TFDPlannerInterface(ros::NodeHandle& nh)
    {
        node_handle = &nh;

        plan_server = new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>((*node_handle), "start_planning", boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false);

        // publishing raw planner output
        std::string plannerTopic = "planner_output";
        node_handle->getParam("planner_topic", plannerTopic);
        plan_publisher = node_handle->advertise<std_msgs::String>(plannerTopic, 1, true);

        // start planning action server
        plan_server->start();
    }

    TFDPlannerInterface::~TFDPlannerInterface()
    {
        delete plan_server;
    }

    /**
     * Runs external commands
     */
    std::string TFDPlannerInterface::runCommand(std::string cmd) {
        std::string data;
        FILE *stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while ( fgets(buffer, 1000, stream) != NULL )
            data.append(buffer);
        pclose(stream);
        return data;
    }

    /*------------------*/
    /* Plan and process */
    /*------------------*/

    /**
     * passes the problem to the Planner; the plan to post-processing.
     */
    bool TFDPlannerInterface::runPlanner() {
        // path is based on the default installation of the Temporal Fast Downward
        std::string plannerPlanPath = data_path + "../../rosplan_planning_system/common/bin/tfd-src-0.4/downward/";
        std::string tfdOutputName = "tfdplan";

        // save problem to file for TFD
        if(use_problem_topic && problem_instance_received) {
            ROS_INFO("KCL: (%s) (%s) Writing problem to file.", ros::this_node::getName().c_str(), problem_name.c_str());
            std::ofstream dest;
            dest.open((problem_path).c_str());
            dest << problem_instance;
            dest.close();
        }

        // prepare the planner command line
        std::string str = planner_command;
        std::size_t dit = str.find("DOMAIN");
        if(dit!=std::string::npos) str.replace(dit,6,domain_path);
        std::size_t pit = str.find("PROBLEM");
        if(pit!=std::string::npos) str.replace(pit,7,problem_path);

        // delete old plans before running the planner
        ROS_INFO("KCL: (%s) Removing old TFD plans with (%s) name.", ros::this_node::getName().c_str(), tfdOutputName.c_str());
        std::string removeOldPlans = "cd " + plannerPlanPath + " && rm " + tfdOutputName + ".*";
        runCommand(removeOldPlans);

        // call the planer
        ROS_INFO("KCL: (%s) (%s) Running: %s", ros::this_node::getName().c_str(), problem_name.c_str(),  str.c_str());
        std::string plan = runCommand(str.c_str());
        ROS_INFO("KCL: (%s) (%s) Planning complete", ros::this_node::getName().c_str(), problem_name.c_str());

        // get the most optimal plan in case there are many (tfdplan.[the highest number])
        std::string bestPlanCommand = "cd " + plannerPlanPath + " && ls | grep " + tfdOutputName + " | tail -1";
        std::string bestPlan = runCommand(bestPlanCommand.c_str());
        // get rid of carriage return
        if (bestPlan[bestPlan.size() - 1] == '\n') {
            bestPlan.erase(bestPlan.size() - 1);
        }

        // prepare command to copy plan
        std::string updatePlanCommand = "cp " + plannerPlanPath + bestPlan + " " + data_path + "plan.pddl";
        runCommand(updatePlanCommand.c_str());

        // check the planner solved the problem
        std::ifstream planfile;
        planfile.open((data_path + "plan.pddl").c_str());
        std::string line;
        std::stringstream ss;

        int curr, next;
        bool solved = false;
        double planDuration;

        while (std::getline(planfile, line)) {

            if (line.find("0.0", 0) != std::string::npos){
                solved = true;
            }
            planDuration = 0;
            ss.str("");
            do {
                if (line.length()<2)
                    break;
                ss << line << std::endl;
            } while (std::getline(planfile, line));
            
            planner_output = ss.str();

        }
        planfile.close();

        if(!solved) ROS_INFO("KCL: (%s) (%s) Plan was unsolvable.", ros::this_node::getName().c_str(), problem_name.c_str());
        else ROS_INFO("KCL: (%s) (%s) Plan was solved.", ros::this_node::getName().c_str(), problem_name.c_str());

        return solved;
    }

} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    srand (static_cast <unsigned> (time(0)));

    ros::init(argc,argv,"rosplan_planner_interface");
    ros::NodeHandle nh("~");

    KCL_rosplan::TFDPlannerInterface pi(nh);

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
