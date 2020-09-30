#ifndef CHIMPPROBLEMGENERATOR_H
#define CHIMPPROBLEMGENERATOR_H

/**
 * This class is responsible for generating the CHIMP problem instance (pdl).
 * This is done by using the objects requested from Knowedge services.
 */
#include "CHIMPProblem.h"
#include "ProblemGenerator.h"
#include "ros/ros.h"

#include <string>
#include <vector>

namespace KCL_rosplan
{
class CHIMPProblemGenerator : public ProblemGenerator
{
private:
  void addInstances(CHIMPProblem& problem);
  std::vector<std::string> queryAllInstances();

  void addInitialState(CHIMPProblem& problem);
  void addGoals(CHIMPProblem& problem);

  void makeProblem(std::ofstream &pFile);

public:
  CHIMPProblemGenerator(const std::string &kb) : ProblemGenerator(kb){};
};
}  // namespace KCL_rosplan

#endif
