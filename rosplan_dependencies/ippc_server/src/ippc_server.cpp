#include "rosplan_dependencies/ippc_server.h"
#include <cstring>

# define DEBUG_VALUE 0

static int
sessionRequestInfo(const XMLNode* node, std::ostream &os,
                   std::string& client_name, std::string& problem_name)
{
    std::string s;
    if( !node->dissect("client-name", client_name ) ) return( 0 );

    if( !node->dissect("problem-name", problem_name ) ) return( 0 );

    if( DEBUG_VALUE > 0 )
    {
        os << "<client>: session start: client name = " << client_name << std::endl;
        os << "<client>: session start: problem name = " << problem_name << std::endl;
    }

    return( 1 );
}


/*******************************************************************************
 *
 * XML client
 *
 ******************************************************************************/

XMLServer_t::XMLServer_t()
{
    // 1 = basic, 2 = info, 3 = sent messages
    DEBUG = DEBUG_VALUE;
}

XMLServer_t::~XMLServer_t()
{
    disconnect();
}

void XMLServer_t::error(const char *msg)
{
    perror(msg);
    exit(1);
}

void
XMLServer_t::connect(int portno)
{
    socklen_t clilen;
//         char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
//         int n;
//         if (argc < 2) {
//                 fprintf(stderr,"ERROR, no port provided\n");
//                 exit(1);
//         }
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        error("ERROR opening socket");
    }

    // enable reuse address
    // http://stackoverflow.com/questions/2147592/bind-error-address-already-in-use
    // UPDATE: does not work, still can not use that port to establish a connection to the last place it connected to
    // http://hea-www.harvard.edu/~fine/Tech/addrinuse.html
//     int on = 1;
//     if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
//         error("setsockopt(SO_REUSEADDR) failed");
//     }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0) {
        std::cerr << "ERROR on binding... retrying in 2 seconds..." << std::endl;
        sleep(2);
        if (bind(sockfd, (struct sockaddr *) &serv_addr,
                 sizeof(serv_addr)) < 0) {
            error("ERROR on binding");
        }
    }
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd,
                       (struct sockaddr *) &cli_addr,
                       &clilen);
    if (newsockfd < 0)
        error("ERROR on accept");

    if (DEBUG > 0)
        std::cout << "IppcServer::" << "Connected!" << std::endl;
    // real code
    this->planner_socket = newsockfd;
}

void
XMLServer_t::disconnect()
{
    close(newsockfd);
    close(sockfd);
}


std::string
XMLServer_t::start_session(uint port, const std::string &domain_path, const std::string &instance_path)
{
    std::ostringstream oss;

    connect(port);

    /*
     * Read session-request
     * session-request => "<session-request>"  <name><problem>  "</session-request>"
     * name => "<client-name>"  <WORD>  "</client-name>"
     * problem => "<problem-name>"  <WORD>  "</problem-name>"
     * Ignoring all data
     */

    if (DEBUG > 0)
        std::cout << "IppcServer::" << "Reading init" << std::endl;
    const XMLNode* sessionInitNode = read_node(planner_socket);
    delete sessionInitNode;
    sessionInitNode = read_node(planner_socket);

    if (DEBUG > 0)
        std::cout << "IppcServer::" << "Node is " << sessionInitNode << std::endl;
    if( !sessionRequestInfo( sessionInitNode, std::cout, client_name, problem_name) ) {
        if (sessionInitNode != 0)
        {
            std::cout << "<client>: ERROR: could not extract session info from the server's response to the session request. The server's response was: " << sessionInitNode << std::endl;
            delete sessionInitNode;
        }
        else
        {
            std::cout << "<client>: ERROR: could not parse the server's response to the session request."<< std::endl;
        }

        return "";
    }

    /*
     * Write session parameters -- the number of rounds to execute a policy
     * for the requested problem on the server and the amount of time
     * allowed for each round
     *
     * -Session init
     * session-init => "<session-init>"  <sessionID><numrounds><timeallowed>  "</session-init>"
     * sessionID => "<session-id>"  <WORD>  "</session-id>"
     * numrounds => "<num-rounds>"  <WORD>  "</num-rounds>"
     * timeallowed => "<time-allowed>"  <WORD>  "</time-allowed"
     * task => "<task>" <TASK_DESCRIPTION> "</task>"s
     */

    time_allowed = 50000;
    num_rounds = 1;
    rounds_left = num_rounds;
    session_id = 1;
    oss.str("");
    oss << "<session-init>"
        <<  "<session-id>" << session_id << "</session-id>"
        <<  "<num-rounds>" << num_rounds << "</num-rounds>"
        <<  "<time-allowed>" << time_allowed << "</time-allowed>"
        << "<task>" << get_encoded_task(domain_path, instance_path) << "</task>"
        <<  "<no-header/>"
        << "</session-init>"
        << '\0';
#if !HAVE_SSTREAM
    oss << '\0';
#endif
    int res = write(planner_socket, oss.str().c_str(), oss.str().length());
    if (res != (int)oss.str().length()) {
        std::cerr << "Error writing to IPPC client! (" << res << ").";
    }
    return client_name;
}


void
XMLServer_t::start_round()
{
    /** *************************************************************
     * ***************************************************************
     * LOOP *
     * ***************************************************************
     */
    std::ostringstream oss;

//     // execute the specified number of rounds; in this implementation, we ignore round_time
//     for( int rounds_left = num_rounds; rounds_left > 0; --rounds_left )
//     {
//         std::cout << "***********************************************" << std::endl;
//         std::cout << ">>> ROUND " << num_rounds - rounds_left + 1 << " OUT OF " << num_rounds << std::endl;
//         std::cout << "***********************************************" << std::endl;



    /*
     * read round request
     *
     * -Round request
     * round-request => "<round-request/>"
     */

    const XMLNode* roundInitNode = read_node(planner_socket);
    roundInitNode = read_node(planner_socket);
    if( !roundInitNode || (roundInitNode->getName() != "round-request") )
    {
        if (roundInitNode != 0)
        {
            std::cout << "<client>: ERROR: could not extract info from the clients's init session request. The client's response was: "  << roundInitNode << std::endl;
            delete roundInitNode;
        }
        else
        {
            std::cout << "<client>: ERROR: could not parse the clients's init session request." << std::endl;
        }

        return;
    }

    /*
     * write Round init
     *
     * -Round init
     * round-init => "<round-init>"  <round><time-left><rounds-left><sessionID>  "</round-init>"
     * round => "<round-num>"  <WORD>  "</round-num>"
     * time-left => "<time-left>"  <WORD>  "</time-left>"
     * rounds-left => "<rounds-left>"  <WORD>  "</rounds-left>"
     */

    oss.str("");
    oss << "<round-init>"
        <<  "<round-num>" << num_rounds - rounds_left + 1 << "</session-id>"
        <<  "<time-left>" << time_allowed << "</num-rounds>"
        <<  "<rounds-left>" << rounds_left - 1 << "</time-allowed>"
        <<  "</round-init>"
        << '\0';
#if !HAVE_SSTREAM
    oss << '\0';
#endif
    int res = write(planner_socket, oss.str().c_str(), oss.str().length());
    if (res != (int)oss.str().length()) {
        std::cerr << "Error writing to IPPC client! (" << res << ").";
    }

    rounds_left--;
}


std::string
XMLServer_t::get_action(const std::vector<rosplan_knowledge_msgs::KnowledgeItem>& state, float& planning_result, double immediate_reward)
{
    std::ostringstream oss;
    /** ************************************************************************
     * ************************************************************************
     * Should be a loop
     * ************************************************************************
     */
    std::string  action_string;
//         for (int i = 0; i < 3; i++) {
    /* Sending:
    * - Send state
    * - Read action
    * - Send round-end
    * - Send session-end
    */

    /*
    * Write State (and turn response)
    * state => "<turn>" <turn-num> <time-left> <immediate-reward> <observed-fluent>* "</turn>" | 
    *     "<turn>" <turn-num> <time-left> <immediate-reward> <no-observed-fluents> "</turn>" |
    *     <end-round> | <end-session>
    * turn-num => "<turn-num>" <WORD> ">/turn-num>"
    * time-left => "<time-left>" <WORD> "</time-left>"
    * immediate-reward => "<immediate-reward> <WORD> "</immediate-reward>"
    * observed-fluent => "<observed-fluent>" <fluent-name> <fluent-arg>* <fluent-value> "</observed-fluent>"
    * fluent-name => "<fluent-name>" <WORD> "</fluent-name>"
    * fluent-arg => "<fluent-arg>" <WORD> "</fluent-arg>"
    * fluent-value => "<fluent-value>" <WORD> "</fluent-value>"
    * no-observed-fluents => "<no-observed-fluents/>"
    */

    oss.str("");
    oss << "<turn><turn-num>1</turn-num><time-left>" << time_allowed << "</time-left><immediate-reward>"<< immediate_reward <<"</immediate-reward>";
    for (auto predicate = state.begin(); predicate != state.end(); ++predicate) {
        if (DEBUG > 1)
            std::cout << "IppcServer::" << "Sending predicate: " << *predicate << std::endl;
        oss <<  "<observed-fluent>" << "<fluent-name>" << predicate->attribute_name << "</fluent-name>";
        for (auto arg = predicate->values.begin(); arg != predicate->values.end(); ++arg) {
            oss <<  "<fluent-arg>" << arg->value << "</fluent-arg>";
        }
        if (predicate->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT)
            oss << "<fluent-value>" << ((predicate->is_negative)? "false" : "true") << "</fluent-value>";
        else if (predicate->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT)
            oss << "<fluent-value>" << std::to_string(predicate->function_value) << "</fluent-value>";
        oss << "</observed-fluent>";
    }
    oss <<  "</turn>"
        << '\0';
#if !HAVE_SSTREAM
    oss << '\0';
#endif
    if (DEBUG > 2)
        std::cout << "IppcServer::" << "Writting: " << oss.str() << std::endl;
    int res = write(planner_socket, oss.str().c_str(), oss.str().length());
    if (res != (int)oss.str().length()) {
        std::cerr << "Error writing to IPPC client! (" << res << ").";
    }
    /*
    * Read action
    *
    * -action spec
    * action spec => "<actions>" <act>* "</actions>"
    * act => "<action>"  <name><arg>*<value>  "</action>"
    * name => "<action-name>"  <WORD>  "</action-name>
    * arg => "<action-arg>"  <WORD>  "</action-arg>"
    * value => "<action-value>" <WORD> "</action-value>"
    */

    const XMLNode *response = NULL;
    response = read_node(planner_socket);
    response = read_node(planner_socket);
    if( !response )
    {
        std::cout << "<client>: ERROR: no state response!! "<< std::endl;
//                 return "error";

    }

    // check if the message is actions
    if (DEBUG > 0)
        std::cout << "IppcServer::" << "Actions name " << response->getName() << std::endl;
    if (response == nullptr) {
        throw std::runtime_error("Could not obtain the planner response");
    }
    if( response->getName() == "actions" )
    {
        action_string = getAction( response, planning_result );
        if (action_string.size() < 3) { // No plan
            if (DEBUG > 0)
                std::cerr << "<client>: ERROR: invalid action response from the client! The response was: " << response << std::endl;
            return "";
        }

        delete response;
        response = NULL;
    }
    else {
        delete response;
        response = NULL;
    }

    if (DEBUG > 0)
        std::cout << "IppcServer::" << "Reading action string: " << action_string << std::endl;

    return action_string;
}

void
XMLServer_t::end_round()
{
    std::ostringstream oss;

    // finished
    if (DEBUG > 0)
        std::cout << "IppcServer::" << "Writting round end" << std::endl;

    /*
     * Finishing...
     * 
     * -End round
     * end-round => "<round-end>" <instance-name> <client-name> <round-num> <round-reward> <turns-used> <time-used> <time-left> <immediate-reward> "</round-end>"
     * instance-name => "<instance-name>" <WORD> "</instance-name>"
     * client-name => "<client-name>" <WORD> "</client-name>"
     * round-num => "<round-num>" <WORD> "</round-num>"
     * round-reward => "<rount-reward>" <WORD> "</round-reward>"
     * turns-used => "<turns-used>" <WORD> "</turns-used>"
     * time-used => "<time-used>" <WORD> "</time-used>"
     * time-left => "<time-left>" <WORD> "</time-left>"
     * immediate-reward => "<immediate-reward>" <WORD> "</immediate-reward>"
     */

    /**
     * <round-end>
     *  <instance-name>domain_inst_mdp__1</instance-name>
     *  <client-name>prost</client-name>
     *  <round-num>2</round-num>
     *  <round-reward>0</round-reward>
     *  <turns-used>1</turns-used>
     *  <time-used>10</time-used>
     *  <time-left>0</time-left>
     * </round-end>
     */

    oss.str("");
    oss << "<round-end>"
        // WARNING G-Pack looks for round-reward for some reason... (note: not present in the protocol)
        << "<instance-name>" << problem_name << "</instance-name>"
        << "<client-name>" << client_name << "</client-name>"
        << "<round-num>" << num_rounds - rounds_left + 1 << "</round-num>"
        << "<round-reward>" << 0 << "</round-reward>"
        << "<turns-used>" << 1 << "</turns-used>"
        << "<time-used>" << time_allowed << "</time-used>"
        << "<time-left>" << /*time_allowed*/ 0 << "</time-left>"
        << "<immediate-reward>" << 0 << "</immediate-reward>"
        <<  "</round-end>"
        << '\0';
#if !HAVE_SSTREAM
    oss << '\0';
#endif
    if (DEBUG > 2)
        std::cout << "IppcServer::" << "Writting: " << oss.str() << std::endl;
    int res = write(planner_socket, oss.str().c_str(), oss.str().length());
    if (res != (int)oss.str().length()) {
        std::cerr << "Error writing to IPPC client! (" << res << ").";
    }
}


void
XMLServer_t::end_session()
{
    std::ostringstream oss;

    if (DEBUG > 0)
        std::cout << "IppcServer::" << "Writting session end" << std::endl;

    /*
     * Finishing...
     *
     * -End session
     * end-session => "<session-end>" <instance-name> <total-reward> <rounds-used> <time-used> <client-name> <session-id> <time-left> "</session-end>"
     * instance-name => "<instance-name>" <WORD> "</instance-name>"
     * total-reward => "<total-reward>"  <WORD>  "</total-reward>"
     * rounds-used => "<rounds-used>"  <WORD>  "</rounds-used>"
     * time-used => "<time-used>" <WORD> "</time-used>"
     * client-name => "<client-name>" <WORD> "</client-name>"
     * session-id => "<session-id>" <WORD> "</session-id>"
     * time-left => "<time-left>"  <WORD>  "</time-left>"
    */

    oss.str("");
    oss << "<session-end>"
        << "<instance-name>" << problem_name << "</instance-name>"
        << "<total-reward>" << 1 << "</total-reward>"
        << "<rounds-used>" << 1 << "</rounds-used>"
        << "<time-used>" << time_allowed << "</time-used>"
        << "<client-name>" << client_name << "</client-name>"
        << "<session-id>" << session_id << "</session-id>"
        << "<time-left>" << 0 << "</time-left>"
        <<  "</session-end>"
        << '\0';
#if !HAVE_SSTREAM
    oss << '\0';
#endif
    if (DEBUG > 2)
        std::cout << "IppcServer::" << "Writting: " << oss.str() << std::endl;
    int res = write(planner_socket, oss.str().c_str(), oss.str().length());
    if (res != (int)oss.str().length()) {
        std::cerr << "Error writing to IPPC client! (" << res << ").";
    }


}


const std::string
XMLServer_t::getAction( const XMLNode* actionNode, float& planning_result)
{
    /*
     * -action spec
     * action spec => "<actions>" <act>* "</actions>"
     * act => "<action>"  <name><arg>*<value>  "</action>"
     * name => "<action-name>"  <WORD>  "</action-name>
     * arg => "<action-arg>"  <WORD>  "</action-arg>"
     * value => "<action-value>" <WORD> "</action-value>"
     */
    if( !actionNode || (actionNode->getName() != "actions") )
        return( NULL );

    if (actionNode->size() == 2
        && actionNode->getChild(1)->getName() == ACTION_NAME_NOOP)
    {
        if (DEBUG > 0)
            std::cout << "IppcServer::" << "No actions received.\n" << std::endl;
        return NULL;
    }
    else
    {
        std::string action;
        std::string action_name;
        std::string action_value;

        // construct the state based on the description of its atoms
        for (int i = 0; i < actionNode->size(); i++)
        {
            std::vector<std::string> action_args;
            if (i > 0) action += ";";
            const XMLNode* cn = actionNode->getChild(i);
            if (cn->getName() == "action")
            {

                if (!cn->dissect("action-name", action_name))
                {
                    std::cout<<"<client> ERROR: There is something wrong with the action's name"<<std::endl;
                    return NULL;
                }
                if (!cn->dissect("action-value", action_value))
                {
                    std::cout<<"<client> ERROR: There is something wrong with the action's value"<<std::endl;
                    return NULL;
                }

                for (int i = 0; i < cn->size(); i++)
                {
                    const XMLNode* cn_arg = cn->getChild(i);
                    if (cn_arg->getName() == "action-arg")
                    {
                        std::string arg;
                        arg = cn_arg->getText();
                        action_args.push_back(arg);
                    }
                }

            }
            action_name += "(";
            for (size_t i = 0; i < action_args.size(); ++i) {
                action_name+=action_args[i];
                if ((i+1) < action_args.size())
                    action_name+=",";
            }
            action_name += ")";
            action += action_name;
            planning_result = atof(action_value.c_str());
            if (DEBUG > 0)
                std::cout << "IppcServer::" << "Action is " << action_name << " with value " << planning_result << std::endl;
        }
        return( action );
    }
}

std::string XMLServer_t::get_encoded_task(const std::string &domain_path, const std::string &instance_path) {
    // Based on https://stackoverflow.com/questions/2912520/read-file-contents-into-a-string-in-c
    std::ifstream ifs(domain_path);
    std::string task_str;
    if (ifs.good()) {
        task_str.assign((std::istreambuf_iterator<char>(ifs)),
                            (std::istreambuf_iterator<char>()));
    }
    else {
        std::cerr << "IppcServer:: " << "Could not open domain file " << domain_path << std::endl;
        return "";
    }

    task_str += "\n\n";

    ifs = std::ifstream(instance_path);
    if (ifs.good()) {
        task_str.append((std::istreambuf_iterator<char>(ifs)),
                       (std::istreambuf_iterator<char>()));
    }
    else {
        std::cerr << "IppcServer:: " << "Could not open instance file " << instance_path << std::endl;
        return "";
    }

    return base64_encode(reinterpret_cast<const unsigned char*>(task_str.c_str()), task_str.length());
}
