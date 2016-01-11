#include "rosplan_planning_system/CFFPlanParser.h"

/* implementation of rosplan_interface_mapping::CFFPlanParser.h */
namespace KCL_rosplan {

	/* constructor */
	CFFPlanParser::CFFPlanParser(ros::NodeHandle &nh) : message_store(nh) {

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	}

	void CFFPlanParser::reset() {
		plan.clear();
	}

	void CFFPlanParser::generateFilter(PlanningEnvironment &environment) {
		// do nothing yet
	}

	/*-----------*/
	/* build PRM */
	/*-----------*/

	/**
	 * Parse a plan
	 */
	void CFFPlanParser::preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) {

		ros::NodeHandle nh("~");
		ROS_INFO("KCL: (CFFPlanParser) Loading plan from file: %s", ((dataPath + "plan.pddl").c_str()));

		// prepare plan
		plan.clear();

		// load plan file
		std::ifstream infile((dataPath + "plan.pddl").c_str());
		std::string line;
		int curr,next,nodeCount;
		bool planFound = false;
		bool planRead = false;
		while(!infile.eof()) {
			std::getline(infile, line);

			if (line.compare("ff: found legal plan as follows") == 0) {
				planFound = true;
			} else if (!planFound) {
				//consume useless lines
			} else if (!planRead) {

				bool observeAction = false;
				bool shedAction = false;
				nodeCount = 0;
				std::vector<int> parentStack;

				while(!infile.eof()) {

					std::getline(infile, line);

					if(line.substr(0,10).compare("time spent")==0)
						break;

					if (line.length()<10)
						continue;

					// action name
					curr = line.find(":");
					std::string name = line.substr(curr+2).c_str();

					// deal with branches
					if("RAMINIFICATE" == name && observeAction) {

						// branch
						parentStack.push_back(nodeCount-1);
						observeAction = true;
					} else if("SHED" == name.substr(0,4)) {

						// shed
						observeAction = false;
						shedAction = true;

					} else if("POP" == name.substr(0,3)) {

						// pop
						if(parentStack.size()>1) {
							std::swap(
								parentStack[parentStack.size()-1],
								parentStack[parentStack.size()-2]);
							observeAction = false;
						} else {
							ROS_INFO("KCL: (CFFPlanParser) Error parsing plan (POP)");
						}

					} else if("RAMINIFICATE" == name) {
						// skip administration actions
						observeAction = true;
					} else if("ASSUME" == name.substr(0,6)) {
						// skip administration actions
						observeAction = false;

					} else {

						PlanNode node(nodeCount,name);

						// incoming edge(s)
						if(parentStack.size()>0) {
							node.inc_edges.push_back(parentStack.back());
							parentStack.pop_back();
						}
						if(shedAction && parentStack.size()>0) {
							node.inc_edges.push_back(parentStack.back());
							parentStack.pop_back();
							shedAction = false;
						}

						// save this parent
						parentStack.push_back(nodeCount);
						nodeCount++;

						// prepare message
						node.dispatch_msg.action_id = node.id;
						node.dispatch_msg.duration = 0.1;
						node.dispatch_msg.dispatch_time = 0;
						node.dispatch_msg.name = name;

						// check for parameters
						curr = line.find(":")+2;
						bool paramsExist = (line.find(" ",curr) != std::string::npos);
						if(paramsExist) {

							// name
							next = line.find(" ",curr);
							node.dispatch_msg.name = line.substr(curr,next-curr).c_str();

							// parameters
							std::vector<std::string> params;
							while(next < line.length()) {
								curr = next + 1;
								next = line.find(" ",curr);
								if(next == std::string::npos)
									next = line.length();
								diagnostic_msgs::KeyValue pair;
								pair.key = "";
								pair.value = line.substr(curr,next-curr);
								node.dispatch_msg.parameters.push_back(pair);
							}
						}
						plan.push_back(node);
						observeAction = true;
					}
				}
				planRead = true;
			}
		}
		printPlan(plan);
		produceEsterel(plan);
		infile.close();
	}

	/*
	 * output a plan as a dot graph
	 */
	bool CFFPlanParser::printPlan(std::vector<PlanNode> &plan) {

		// output file
		std::ofstream dest;
		dest.open("plan.dot");

		dest << "digraph plan {" << std::endl;

		for(int i=0;i<plan.size();i++) {
			for(int j=0;j<plan[i].inc_edges.size();j++) {
				if(plan[i].inc_edges[j] >= 0)
					dest << "\"[" <<  plan[i].inc_edges[j] << "] " << plan[plan[i].inc_edges[j]].action_name
						<< "\" -> \"[" << plan[i].id <<	 "] " << plan[i].action_name << "\"" << std::endl;
			}
		}

		dest << "}" << std::endl;
		dest.close();
	}

	/*
	 * output a plan as an Esterel controller
	 */
	bool CFFPlanParser::produceEsterel(std::vector<PlanNode> &plan) {

		// output file
		std::ofstream dest;
		dest.open("plan.strl");

		// main module
		dest << "module plan:" << std::endl;
		dest << "input SOURCE";
		for(int i=0;i<plan.size();i++) {
			dest << ", a" << plan[i].id << "_complete";
		}
		dest << std::endl;

		dest << "output SINK";
		for(int i=0;i<plan.size();i++) {
			dest << ", a" << plan[i].id << "_dispatch";
		}
		dest << std::endl;

		dest << "signal e" << plan[0].id;
		for(int i=1;i<plan.size();i++) {
			dest << ", e" << plan[i].id;
		}
		dest << " in" << std::endl;
		dest << "run action" << plan[0].id << std::endl;
		for(int i=1;i<plan.size();i++) {
			dest << " || action" << plan[i].id << std::endl;
		}
		dest << "end" << std::endl;
		dest << "end module" << std::endl << std::endl;

		// action modules
		for(int i=0;i<plan.size();i++) {

			dest << "module action" << plan[i].id << ":" << std::endl;

			if(plan[i].inc_edges.size() > 0) {
				dest << "input ";
				for(int j=0;j<plan[i].inc_edges.size();j++) {
					if(j>0) dest << ", ";
					dest << "e" << plan[plan[i].inc_edges[j]].id;
				}
				dest << ";" << std::endl;
			}

			dest << "output e" << plan[i].id << ";" << std::endl;

			dest << "  await ";
			for(int j=0;j<plan[i].inc_edges.size();j++) {
				if(j>0) dest << " or ";
				dest << "e" << plan[plan[i].inc_edges[j]].id;
			}
			dest << ";" << std::endl;
			dest << "  emit action" << plan[i].id << "_dispatch;" << std::endl;
			dest << "  await action" << plan[i].id << "_complete;" << std::endl;
			dest << "  emit e" << plan[i].id << ";" << std::endl;
			dest << "end module" << std::endl << std::endl;
		}
		dest.close();
	}
} // close namespace
