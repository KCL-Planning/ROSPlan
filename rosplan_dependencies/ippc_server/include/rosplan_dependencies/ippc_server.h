/**
 * This code is based on the IPPC client code from glutton/gourmand
 * http://www.cs.washington.edu/ai/planning/gourmand.html
 *
 * It was obtained from REX-D https://bitbucket.org/dmartinezm/rex-d
 * And modified to adapt it to ROSPlan by Gerard Canal <gcanal@iri.upc.edu>
 */

#ifndef CLIENT_H
#define CLIENT_H
#define ACTION_NAME_NOOP "noop"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "base64.h"
#include <fstream>

#define HAVE_SSTREAM 1
#if HAVE_SSTREAM
#include <sstream>
#else
#include <strstream>
namespace std {
typedef std::ostrstream ostringstream;
}
#endif

#include "strxml.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

/*******************************************************************************
 *
 * XML Server
 *
 ******************************************************************************/

/*
  This class is responsible for communicating the policy produced by the planner
  to the server. The server sends to the client the descriptions of states for
  which it wants to know an action to execute, and the client sends back the
  descriptions of actions recommended for these states by the planner's policy.
*/

template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

class XMLServer_t
{
private:
    int DEBUG;
    int sockfd, newsockfd;
    int planner_socket;
    long time_allowed;
    int num_rounds;
    int rounds_left;
    int session_id;

    std::string client_name;
    std::string problem_name;

    /*
      Based on the description of a state received from the server, constructs
      that state in the planner's representation.
    */
    const std::string getAction( const XMLNode* actionNode, float& planning_result );

    void connect(int portno);
    void disconnect();
    void error(const char *msg);

public:
    XMLServer_t(  );
    ~XMLServer_t();

    /*
      Carries out a problem-solving session with the server.
    */
    std::string start_session(uint port, const std::string &domain_path, const std::string &instance_path);
    void end_session();

    std::string get_action(const std::vector<rosplan_knowledge_msgs::KnowledgeItem>& state, float& planning_result, double reward=0);

    std::string get_encoded_task(const std::string &domain_path, const std::string &instance_path);

    void start_round();
    void end_round();
};

#endif // CLIENT_H
