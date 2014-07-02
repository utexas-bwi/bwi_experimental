///
//   Unit test for making two plans concurrently.

#include <string>
#include <vector>
#include <gtest/gtest.h>
#include "../src/plan_concurrently.h"

#if 0  // commented out
using namespace bwi_actexec;

std::string g_test_action = "test_action";
std::string g_test_param = "test_parameter";
std::vector<std::string> g_test_parameters(1, g_test_param);

class TestAction: public Action
{
public:
  int paramNumber() const {return 1;}
  std::string getName() const {return std::string(g_test_action);}
  void run() {}
  bool hasFinished() const {return false;}
  Action *clone() const {return NULL;}
private:
  std::vector<std::string> getParameters() const {return g_test_parameters;}
};

Action *test_act = new TestAction();
#endif  // commented out

std::list<int> empty_plan()
{
  return std::list<int>();
}

std::list<int> immediate_plan()
{
  std::list<int> plan;
  plan.push_back(42);
  return plan;
}

TEST(PlanConcurrently, planFailure)
{
  std::list<int> plan = plan_concurrently<int>(empty_plan, empty_plan);
  EXPECT_TRUE(plan.empty());
}

TEST(PlanConcurrently, useFirstPlan)
{
  std::list<int> plan = plan_concurrently<int>(immediate_plan, empty_plan);
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(plan.front(), 42);
}

TEST(PlanConcurrently, useSecondPlan)
{
  std::list<int> plan = plan_concurrently<int>(empty_plan, immediate_plan);
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(plan.front(), 42);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
