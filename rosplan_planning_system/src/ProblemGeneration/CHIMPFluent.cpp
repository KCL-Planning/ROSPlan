#include "rosplan_planning_system/ProblemGeneration/CHIMPFluent.h"

#include <sstream>

using std::string;

namespace KCL_rosplan
{

int CHIMPFluent::nextId = 0;

CHIMPFluent::CHIMPFluent(FluentType type, string name, const std::vector<std::string> &arguments)
    : id_(++nextId),
      type_(type),
      name_(name),
      arguments_(arguments)
{
}

CHIMPFluent::~CHIMPFluent()
{
}

std::string CHIMPFluent::chimpFormatRepr() const
{
  std::stringstream ss;
  ss << name_ << "(";
  for (size_t i = 0; i < arguments_.size(); ++i)
  {
    if (i > 0)
      ss << " ";
    ss << arguments_[i];
  }
  ss << ")";
  return ss.str();
}


} // namespace