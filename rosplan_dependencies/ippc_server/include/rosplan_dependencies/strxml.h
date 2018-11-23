/* -*-C++-*- */
/*
 * XML.
 *
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
 *
 * NOTE: This file is ignored by doxygen config.
 */
#ifndef _STRXML_H
#define _STRXML_H

/* Comment out the following define if you get complaints with sstream */
#define HAVE_SSTREAM 1

#include <iostream>
#include <map>
#include <string>
#include <vector>


/* ====================================================================== */
/* XMLNode */

/*
 * An abstract XML node.
 */
struct XMLNode {
    /* Deletes this XML node. */
#if __cplusplus > 199711L // C++11
    virtual ~XMLNode() = default;
#else
    virtual ~XMLNode() {}
#endif

    /* Returns the ith child of this XML node. */
    virtual const XMLNode* getChild(int i) const {
        return 0;
    }

    /* Returns the child of this XML node with the given name. */
    virtual const XMLNode* getChild(const std::string& name) const {
        return 0;
    }

    /* Returns the size of this XML node. */
    virtual int size() const {
        return 0;
    }

    /* Returns the text for this XML node. */
    virtual std::string getText() const = 0;

    /* Returns the name for this XML node. */
    virtual const std::string& getName() const = 0;

    /* Returns the parameter of this XML node with the given name. */
    virtual const std::string& getParam(std::string name) const = 0;

    /* Puts the text for the child node with the given in the
       destination string.  Returns false if no child node with the
       given name exists. */
    bool dissect(const std::string& child, std::string& destination) const;

protected:
    /* Prints this object on the given stream. */
    virtual void print(std::ostream& os) const = 0;

    friend std::ostream& operator<<(std::ostream& os, const XMLNode& xn);
};

/* Output operator for XML nodes. */
std::ostream& operator<<(std::ostream& os, const XMLNode& xn);

/* Output operator for XML node pointers. */
std::ostream& operator<<(std::ostream& os, const XMLNode* xn);

/* Reads an XML node from the given file descriptor. */
const XMLNode* read_node(int fd);


typedef std::pair<std::string, std::string> str_pair;
typedef std::vector<str_pair> str_pair_vec;
typedef std::vector<std::string> str_vec;
typedef std::map<std::string, std::string> str_str_map;
typedef std::vector<const XMLNode*> node_vec;


/* ====================================================================== */
/* XMLText */

/*
 * An XML text node.
 */
struct XMLText : public XMLNode {
    /* The text of this node. */
    std::string text;

    /* Constructs an XML text node. */
    XMLText(const std::string& text) : text(text) {}

    /* Returns the text for this XML node. */
    virtual std::string getText() const;

    /* Returns the name for this XML node. */
    virtual const std::string& getName() const;

    /* Returns the parameter of this XML node with the given name. */
    virtual const std::string& getParam(std::string name) const;

protected:
    /* Prints this object on the given stream. */
    virtual void print(std::ostream& os) const;
};


/* ====================================================================== */
/* XMLParent */

/*
 * An XML parent node.
 */
struct XMLParent : public XMLNode {
    /* Name of this node. */
    std::string name;
    /* Parameters of this node. */
    str_str_map params;
    /* Children of this node. */
    node_vec children;

    /* Constructs an XML parent node. */
    XMLParent(const std::string& name) : name(name) {}

    /* Deletes this XML parent node. */
    virtual ~XMLParent();

    /* Returns the ith child of this XML node. */
    virtual const XMLNode* getChild(int i) const;

    /* Returns the child of this XML node with the given name. */
    virtual const XMLNode* getChild(const std::string& name) const;

    /* Returns the size of this XML node. */
    virtual int size() const;

    /* Returns the text for this XML node. */
    virtual std::string getText() const;

    /* Returns the name for this XML node. */
    virtual const std::string& getName() const;

    /* Returns the parameter of this XML node with the given name. */
    virtual const std::string& getParam(std::string name) const;

protected:
    /* prints this object on the given stream. */
    virtual void print(std::ostream& os) const;
};


#endif /* _STRXML_H */