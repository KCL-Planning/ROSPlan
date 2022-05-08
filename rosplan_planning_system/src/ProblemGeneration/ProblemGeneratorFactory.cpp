//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 26/09/18.
//

#include <rosplan_planning_system/ProblemGeneration/ProblemGeneratorFactory.h>

#include "rosplan_planning_system/ProblemGeneration/ProblemGeneratorFactory.h"

namespace KCL_rosplan {
    ProblemGeneratorPtr ProblemGeneratorFactory::createProblemGenerator(ProblemGeneratorFactory::ProbGen pg_type, const std::string& kb) {
        if (pg_type == ProbGen::PDDL) return ProblemGeneratorPtr(new PDDLProblemGenerator(kb));
        else if (pg_type == ProbGen::RDDL) return ProblemGeneratorPtr(new RDDLProblemGenerator(kb));
        else if (pg_type == ProbGen::CHIMP) return ProblemGeneratorPtr(new CHIMPProblemGenerator(kb));
        else {
            ROS_ERROR("KCL: (%s) Unknown Problem Generator type.", ros::this_node::getName().c_str());
            ros::shutdown();
        }
    }
}