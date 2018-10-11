/*
 * Copyright 2003-2005 Carnegie Mellon University and Rutgers University
 * Copyright 2007 H�kan Younes
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "rosplan_dependencies/strxml.h"
#if HAVE_SSTREAM
#include <sstream>
#else
#include <strstream>
namespace std {
typedef std::ostrstream ostringstream;
}
#endif
#include <stack>
#include <unistd.h>


struct PSink {
    int error;
    const XMLNode* top;

    PSink() : error(0), top(0) {}

    void pushNode(const std::string& name, const str_pair_vec& params);
    void popNode(const std::string& name);
    void pushText(const std::string& text);

    void formaterror() {
        error=1;
    }
    void streamerror() {
        error=2;
    }

private:
    std::stack<XMLParent*> s;
};


static const std::string EMPTY_STRING;


static std::string next_token(int fd) {
    static char last_char = 0;

    std::string res;
    if (last_char) {
        res += last_char;
    }

    if (last_char == '<') {
        last_char = 0;
        return res;
    }

    if (last_char == '>') {
        last_char = 0;
        return res;
    }

    char next_char;
    while (1) {
        if (read(fd, &next_char, 1) != 1) {
            return EMPTY_STRING;
        }
        if (next_char == '>' || next_char == '<') {
            if (res.empty()) {
                res += next_char;
                last_char = 0;
                return res;
            }
            last_char = next_char;
            break;
        }
        res += next_char;
    }
//   std::cout << "Read: " << res << std::endl;
    return res;
}


static int token_type(char c) {
    if (c == '=')
        return 2;
    if (c == '"')
        return 3;
    if (c == '/')
        return 4;
    if (isspace(c))
        return 0;
    return 1;
}


static str_vec tokenize_string(const std::string& str) {
    str_vec v;
    std::string last;
    int t_type = 0;
    for (const char *s = str.c_str(); *s; s++) {
        int n_type = token_type(*s);

        if (t_type != n_type && !last.empty()) {
            if (t_type) {
                v.push_back(last);
            }
            last.erase();
        }
        last += *s;
        t_type = n_type;
    }
    if (last.length()) {
        v.push_back(last);
    }
    return v;
}


static int do_node(const std::string& token, PSink& ps) {
    str_vec node_tokens = tokenize_string(token);

    if (node_tokens.empty()) {
        return -2;
    }

    if (node_tokens[0] == "/") {
        ps.popNode(node_tokens[1]);
        return -1;
    }
    std::string name = node_tokens[0];
    str_pair_vec v;
    for (size_t i=1; i<node_tokens.size(); i+=5) {
        if (node_tokens[i] == "/") {
            ps.pushNode(name, v);
            ps.popNode(name);
            return 0;
        }
        if (i+5 > node_tokens.size() ||
            node_tokens[i+1] != "=" ||
            node_tokens[i+2] != "\"" ||
            node_tokens[i+4] != "\"") {
            return -2;
        }
        str_pair p(node_tokens[i], node_tokens[i+3]);
        v.push_back(p);
    }
    ps.pushNode(name, v);
    return 1;
}


static bool parse_node(int fd, PSink& ps) {
    std::string token = next_token(fd);
    int depth = 0;
    while (!token.empty()) {
        if (token == "<") {
            int delta = do_node(next_token(fd), ps);
            if (delta == -2) {
                ps.formaterror();
                return false;
            }
            depth += delta;
            token = next_token(fd);
            if (token != ">") {
                ps.formaterror();
                return false;
            }
            if (depth == 0) {
                return true;
            }
        } else {
            ps.pushText(token);
        }
        token = next_token(fd);
    }
    ps.streamerror();
    return false;
}


void PSink::pushNode(const std::string& name, const str_pair_vec& params) {
    XMLParent *p = new XMLParent(name);
    for (size_t i=0; i<params.size(); i++) {
        p->params[params[i].first] = params[i].second;
    }
    if (s.empty()) {
        top = p;
    } else {
        s.top()->children.push_back(p);
    }
    s.push(p);
}


void PSink::popNode(const std::string& name) {
    //XMLParent *p = s.top();
    s.pop();
    //delete p;
}


void PSink::pushText(const std::string& text) {
    XMLText *t = new XMLText(text);
    if (s.size() == 0) {
        top = t;
    } else {
        s.top()->children.push_back(t);
    }
}


/* ====================================================================== */
/* XMLNode */

/* Puts the text for the child node with the given in the
   destination string.  Returns false if no child node with the
   given name exists. */
bool XMLNode::dissect(const std::string& child,
                      std::string& destination) const {
    const XMLNode* c = getChild(child);
    if (c != 0) {
        destination = c->getText();
        return true;
    } else {
        return false;
    }
}


/* Output operator for XML nodes. */
std::ostream& operator<<(std::ostream& os, const XMLNode& xn) {
    xn.print(os);
    return os;
}


/* Output operator for XML node pointers. */
std::ostream& operator<<(std::ostream& os, const XMLNode* xn) {
    if (xn != 0) {
        os << *xn;
    }
    return os;
}


/* Reads an XML node from the given file descriptor. */
const XMLNode* read_node(int fd) {
    PSink ps;
    if (parse_node(fd, ps)) {
        return ps.top;
    } else {
        return 0;
    }
}


/* ====================================================================== */
/* XMLText */

/* Returns the text for this XML node. */
std::string XMLText::getText() const {
    return text;
}


/* Returns the name for this XML node. */
const std::string& XMLText::getName() const {
    return text;
}


/* Returns the parameter of this XML node with the given name. */
const std::string& XMLText::getParam(std::string name) const {
    return EMPTY_STRING;
}


/* Prints this object on the given stream. */
void XMLText::print(std::ostream& os) const {
    os << text;
}


/* ====================================================================== */
/* XMLParent */

/* Deletes this XML parent node. */
XMLParent::~XMLParent() {
    for (node_vec::const_iterator ni = children.begin();
         ni != children.end(); ni++) {
        delete *ni;
    }
}


/* Returns the ith child of this XML node. */
const XMLNode* XMLParent::getChild(int i) const {
    return children[i];
}


/* Returns the child of this XML node with the given name. */
const XMLNode* XMLParent::getChild(const std::string& name) const {
    for (node_vec::const_iterator ni = children.begin();
         ni != children.end(); ni++)  {
        const XMLParent* p = dynamic_cast<const XMLParent*>(*ni);
        if (p != 0 && p->name == name) {
            return p;
        }
    }
    return 0;
}


/* Returns the size of this XML node. */
int XMLParent::size() const {
    return children.size();
}


/* Returns the text for this XML node. */
std::string XMLParent::getText() const {
    std::ostringstream os;
    for (node_vec::const_iterator ni = children.begin();
         ni != children.end(); ni++) {
        os << *ni;
    }
#if !HAVE_SSTREAM
    os << '\0';
#endif
    return os.str();
}


/* Returns the name for this XML node. */
const std::string& XMLParent::getName() const {
    return name;
}


/* Returns the parameter of this XML node with the given name. */
const std::string& XMLParent::getParam(std::string name) const {
    str_str_map::const_iterator pi = params.find(name);
    return (pi != params.end()) ? (*pi).second : EMPTY_STRING;
}


/* prints this object on the given stream. */
void XMLParent::print(std::ostream& os) const {
    os << "<" << name;
    for (str_str_map::const_iterator itr = params.begin();
         itr != params.end(); itr++) {
        os << " " << itr->first << "=\"" << itr->second << "\"";
    }
    os << ">";
    for (node_vec::const_iterator ni = children.begin();
         ni != children.end(); ni++) {
        os << *ni;
    }
    os << "</" << name << ">";
}