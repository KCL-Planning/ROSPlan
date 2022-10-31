#ifndef CHIMPPROBLEM_H
#define CHIMPPROBLEM_H

#include "CHIMPFluent.h"

#include <string>
#include <vector>
#include <set>
#include <iostream>

namespace KCL_rosplan 
{

class CHIMPProblem
{

  public:
    CHIMPProblem();
    ~CHIMPProblem();

    void addFluent(const CHIMPFluent& fluent);
    void addFluents(const std::vector<CHIMPFluent>& add_fluents);
    void addTask(const CHIMPFluent& task);
    void addArgumentSymbol(const std::string& symbol);
    void addArgumentSymbols(const std::vector<std::string>& symbols);
    void generateProblem(std::ostream& os);
    std::vector<CHIMPFluent> getFluents();

  private:
    std::vector<CHIMPFluent> fluents_;
    std::vector<CHIMPFluent> tasks_;
    std::set<std::string> symbols_;

    void extractFluentsArguments();
    void writeFluents(std::ostream &os);
    void writeTasks(std::ostream &os);
    void writeArgumentSymbols(std::ostream &os);
};

} // namespace KCL_rosplan

#endif // CHIMPPROBLEM_H