//
// Created by Gerard Canal on 24/09/18.
//

#include "rosplan_knowledge_base/KnowledgeBaseFactory.h"

namespace KCL_rosplan {
    std::unique_ptr<KCL_rosplan::KnowledgeBase>
    KnowledgeBaseFactory::createKB(KCL_rosplan::KnowledgeBaseFactory::KB kb_type, ros::NodeHandle& n) {
        if (kb_type == KB::PDDL) return KnowledgeBasePtr(new PDDLKnowledgeBase(n));
        else if (kb_type == KB::PPDDL) return KnowledgeBasePtr(new PPDDLKnowledgeBase(n));
        else if (kb_type == KB::RDDL) return KnowledgeBasePtr(new RDDLKnowledgeBase(n));
        else if (kb_type == KB::HDDL or kb_type == KB::CHIMP) return KnowledgeBasePtr(new HDDLKnowledgeBase(n));
        else {
            ROS_ERROR("KCL: (%s) Unknown Knowledge Base type.", ros::this_node::getName().c_str());
            ros::shutdown();
        }
    }
}
