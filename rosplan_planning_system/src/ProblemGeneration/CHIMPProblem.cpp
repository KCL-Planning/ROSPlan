#include "rosplan_planning_system/ProblemGeneration/CHIMPProblem.h"
#include <iostream>
#include <fstream>
#include <sstream>

using std::string;
using std::vector;

namespace KCL_rosplan
{

CHIMPProblem::CHIMPProblem()
{
}

CHIMPProblem::~CHIMPProblem()
{
}

void CHIMPProblem::addFluent(const CHIMPFluent& fluent)
{
    fluents_.push_back(fluent);
}

void CHIMPProblem::addFluents(const std::vector<CHIMPFluent>& add_fluents)
{
    fluents_.insert(fluents_.end(), add_fluents.begin(), add_fluents.end());
}

void CHIMPProblem::addTask(const CHIMPFluent& task)
{
    tasks_.push_back(task);
}

void CHIMPProblem::addArgumentSymbol(const std::string& symbol)
{
    symbols_.insert(symbol);
}

void CHIMPProblem::addArgumentSymbols(const std::vector<std::string>& add_symbols)
{
    symbols_.insert(add_symbols.begin(), add_symbols.end());
}

void CHIMPProblem::generateProblem(std::ostream &os)
{
    extractFluentsArguments();
    os << "(Problem\n\n";
    writeArgumentSymbols(os);
    os << "\n";
    writeFluents(os);
    os << "\n";
    writeTasks(os);
    os << ")\n";
}

string idStr(const CHIMPFluent &fluent)
{
    string prefix = "f";
    if (fluent.type_ == CHIMPFluent::FluentType::TASK) {
        prefix = "t";
    }
    std::stringstream ss;
    ss << prefix << fluent.id_;
    return ss.str();
}

std::ostream &operator<<(std::ostream &os, const CHIMPFluent &fluent)
{
    if (fluent.type_ == CHIMPFluent::FluentType::STATE)
    {
        os << "(Fluent ";
    }
    else
    {
        os << "(Task ";
    }
    os << idStr(fluent) << " ";
    os << fluent.name_ << "(";
    if (fluent.arguments_.size() > 0)
    {
        os << fluent.arguments_.at(0);
        for (std::size_t i = 1; i < fluent.arguments_.size(); ++i)
        {
            os << " " << fluent.arguments_.at(i);
        }
    }
    os << "))";
    return os;
}

string formatConstraint(string constraint, string id_str, int earliest, int latest)
{
    std::stringstream ss;
    ss << "(Constraint " << constraint << "[" << earliest << "," << latest << "]";
    ss << "(" << id_str << ")";
    ss << ")";
    return ss.str();
}

void CHIMPProblem::writeArgumentSymbols(std::ostream &os)
{
    os << "(ArgumentSymbols\n";
    for (string arg : symbols_)
    {
        os << arg << "\n";
    }
    os << "n)\n";
}

void CHIMPProblem::writeFluents(std::ostream &os)
{
    for (CHIMPFluent fluent : fluents_)
    {
        os << fluent << "\n";
        os << formatConstraint("Release", idStr(fluent), 0, 0) << "\n";
    }
}

void CHIMPProblem::writeTasks(std::ostream &os)
{
    for (CHIMPFluent task : tasks_)
    {
        os << task << "\n";
    }
}

void CHIMPProblem::extractFluentsArguments()
{
    for (CHIMPFluent fluent : fluents_)
    {
        symbols_.insert(fluent.arguments_.begin(), fluent.arguments_.end());
    }
    for (CHIMPFluent fluent : tasks_)
    {
        symbols_.insert(fluent.arguments_.begin(), fluent.arguments_.end());
    }
}

std::vector<CHIMPFluent> CHIMPProblem::getFluents()
{
    return fluents_;
}

} // namespace KCL_rosplan