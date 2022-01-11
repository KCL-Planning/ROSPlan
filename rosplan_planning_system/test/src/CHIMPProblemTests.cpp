// #include <gtest/gtest.h>
#include <gtest/gtest.h>
#include "rosplan_planning_system/ProblemGeneration/CHIMPProblem.h"
#include <sstream>

using namespace KCL_rosplan;

GTEST_TEST(CHIMPProblemTests, Test1_generate_problem)
{
    CHIMPFluent::FluentType state_type = CHIMPFluent::FluentType::STATE;
    
    CHIMPProblem problem;
    CHIMPFluent on1_fluent(state_type, "On", {"mug1", "table1"});
    problem.addFluent(on1_fluent);

    CHIMPFluent screw_task(CHIMPFluent::FluentType::TASK, "get_object", {"mug1"});
    problem.addTask(screw_task);

    std::stringstream problem_stream;
    problem.generateProblem(problem_stream);

    const char* expected = R"problem((Problem

(ArgumentSymbols
mug1
table1
n)

(Fluent f1 On(mug1 table1))
(Constraint Release[0,0](f1))

(Task t2 get_object(mug1))
)
)problem";

    ASSERT_EQ(problem_stream.str(), expected);

}

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}