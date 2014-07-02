///
//   Unit test for making two plans concurrently.

#include <string>
#include <vector>
#include <unistd.h>
#include <boost/bind.hpp>
#include <gtest/gtest.h>
#include "../src/plan_concurrently.h"

std::list<int> empty_plan()
{
  return std::list<int>();
}

std::list<int> immediate_plan(int result)
{
  std::list<int> plan;
  plan.push_back(result);
  return plan;
}

std::list<int> delayed_plan(int result, int usecs)
{
  usleep(usecs);
  std::list<int> plan;
  plan.push_back(result);
  return plan;
}

TEST(PlanConcurrently, planFailure)
{
  std::list<int> plan = plan_concurrently<int>(empty_plan, empty_plan);
  EXPECT_TRUE(plan.empty());
}

TEST(PlanConcurrently, useFirstPlan)
{
  std::list<int> plan = plan_concurrently<int>
    (boost::bind(immediate_plan, 42),
     empty_plan);
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(plan.front(), 42);
}

TEST(PlanConcurrently, useSecondPlan)
{
  std::list<int> plan = plan_concurrently<int>
    (empty_plan,
     boost::bind(immediate_plan, 42));
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(plan.front(), 42);
}

TEST(PlanConcurrently, useDelayedPlan)
{
  std::list<int> plan = plan_concurrently<int>
    (empty_plan,
     boost::bind(delayed_plan, 17, 100000));
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(plan.front(), 17);
}

TEST(PlanConcurrently, useLessDelayedPlan)
{
  std::list<int> plan = plan_concurrently<int>
    (boost::bind(delayed_plan, 7, 50000),
     boost::bind(delayed_plan, 17, 100000));
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(plan.front(), 7);
}

TEST(PlanConcurrently, useImmediatePlan)
{
  std::list<int> plan = plan_concurrently<int>
    (boost::bind(immediate_plan, 42),
     boost::bind(delayed_plan, 17, 100000));
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(plan.front(), 42);
}

// These tests fail with the serial implementation, they requires
// actual concurrency.  Save it for the genuine implementation.
#if 0
TEST(PlanConcurrently, useSecondImmediatePlan)
{
  std::list<int> plan = plan_concurrently<int>
    (boost::bind(delayed_plan, 17, 100000),
     boost::bind(immediate_plan, 42));
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(plan.front(), 42);
}

TEST(PlanConcurrently, useLessDelayedSecondPlan)
{
  std::list<int> plan = plan_concurrently<int>
    (boost::bind(delayed_plan, 17, 100000),
     boost::bind(delayed_plan, 7, 50000));
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(plan.front(), 7);
}
#endif

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
