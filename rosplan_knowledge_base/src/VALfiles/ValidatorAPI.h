#ifndef VAL_API
#define VAL_API

#include <sstream>

namespace VAL {

bool checkPlan(const std::string& domain_file, const std::string& problem_file, std::stringstream& plan);

};

#endif
