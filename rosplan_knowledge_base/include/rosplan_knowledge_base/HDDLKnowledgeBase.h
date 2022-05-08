#include <ros/ros.h>
#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <diagnostic_msgs/KeyValue.h>

#include "hddl_parser.h"
#include "KnowledgeBase.h"

namespace KCL_rosplan {

    class HDDLKnowledgeBase : public KnowledgeBase {
      public:
        HDDLKnowledgeBase(ros::NodeHandle& n);

        /* parse domain and probelm files */
        void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) override;

        /* add the initial state to the knowledge base */
        void addInitialState() override;
        void addConstants() override;
        
        /* service methods for fetching the domain details */
        bool getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request  &req, rosplan_knowledge_msgs::GetDomainNameService::Response &res) override;
        bool getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res) override;
        bool getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) override;
        bool getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) override;
        bool getFunctionPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) override;
        bool getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res) override;
        bool getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) override;

      private:

        HDDLParser hddl_parser_;
        std::string problem_file_path_;
    };
}
