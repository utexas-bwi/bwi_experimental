// -*- c-file-style: bsd -*-

///
//   Unit test for making two plans concurrently.

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "../src/plan_concurrently.h"

typedef PlanConcurrently::Plan Plan;

Plan empty_plan()
{
  return Plan();
}

TEST(PlanConcurrently, planFailure)
{
  PlanConcurrently p;
  Plan plan = p.plan(empty_plan, empty_plan);
  EXPECT_TRUE(plan.empty());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_plan_concurrently");
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
