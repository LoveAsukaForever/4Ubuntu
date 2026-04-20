// std
#include <cmath>
#include <thread>

// gtest
#include <gtest/gtest.h>

// any worker
#include "any_worker/Rate.hpp"

// time/time
#define RATE_TEST_TOL 0.05       // 5%
#define RATE_TEST_TOL_ABS 0.001  // 1ms

/*!
 * Simulate some processing which takes a certain amount of time.
 * @param duration Processing duration in seconds.
 */
void doSomething(const double duration) {
  std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1e9 * duration)));
}

#define EXPECT_NEAR_TEST_TOL(A, B) EXPECT_NEAR((A), (B), (RATE_TEST_TOL) * (B) + RATE_TEST_TOL_ABS)

TEST(RateTest, Initialization) {  // NOLINT
  const std::string name = "Test";
  const double timeStep = 0.1;
  any_worker::Rate rate(name, timeStep);

  EXPECT_EQ(rate.getOptions().name_, name);
  EXPECT_EQ(rate.getOptions().timeStep_, timeStep);
  EXPECT_EQ(rate.getOptions().maxTimeStepFactorWarning_, 1.0);
  EXPECT_EQ(rate.getOptions().maxTimeStepFactorError_, 10.0);
  EXPECT_EQ(rate.getOptions().enforceRate_, true);
  EXPECT_EQ(rate.getOptions().clockId_, CLOCK_MONOTONIC);
  EXPECT_EQ(rate.getNumTimeSteps(), 0u);
  EXPECT_EQ(rate.getNumWarnings(), 0u);
  EXPECT_EQ(rate.getNumErrors(), 0u);
  EXPECT_TRUE(std::isnan(rate.getAwakeTime()));
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeMean()));
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));
}

TEST(RateTest, Reset) {  // NOLINT
  const double timeStep = 0.1;
  const double processingTime = 0.05;

  // Run for one time step and reset.
  any_worker::Rate rate("Test", timeStep);
  doSomething(processingTime);
  rate.sleep();
  rate.reset();

  EXPECT_EQ(rate.getNumTimeSteps(), 0u);
  EXPECT_EQ(rate.getNumWarnings(), 0u);
  EXPECT_EQ(rate.getNumErrors(), 0u);
  EXPECT_TRUE(std::isnan(rate.getAwakeTime()));
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeMean()));
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));
}

TEST(RateTest, DISABLED_SleepWithEnforceRate) {  // NOLINT
  const double timeStep = 0.01;
  any_worker::Rate rate("Test", timeStep);
  rate.getOptions().enforceRate_ = true;

  timespec start{};
  timespec end{};
  const double processingTime = 0.005;
  std::vector<double> processingTimes;
  std::vector<double> summedStepTimes;

  // Test sleep() without processing.
  clock_gettime(CLOCK_MONOTONIC, &start);
  rate.reset();
  rate.sleep();
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), timeStep);

  // Test sleep() with processing additionally.
  clock_gettime(CLOCK_MONOTONIC, &start);
  rate.reset();
  doSomething(processingTime);
  rate.sleep();
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), timeStep);

  // Test sleep() with where one step takes too long, recovery within one step.
  processingTimes = {0.002, 0.002, 0.015, 0.002, 0.002};
  summedStepTimes = {0.0, 0.01, 0.02, 0.035, 0.04, 0.05};
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processingTimes.size(); i++) {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[i]);
    doSomething(processingTimes[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()]);

  // Test sleep() with where one step takes too long, recovery within two steps.
  processingTimes = {0.002, 0.002, 0.019, 0.002, 0.002, 0.002};
  summedStepTimes = {0.00, 0.01, 0.02, 0.039, 0.041, 0.05, 0.06};
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processingTimes.size(); i++) {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[i]);
    doSomething(processingTimes[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()]);

  // Test sleep() with where two steps take too long, recovery within one step.
  processingTimes = {0.002, 0.002, 0.012, 0.012, 0.002, 0.002};
  summedStepTimes = {0.00, 0.01, 0.02, 0.032, 0.044, 0.05, 0.06};
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processingTimes.size(); i++) {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[i]);
    doSomething(processingTimes[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()]);

  // Test sleep() with where two steps take too long, recovery within two steps.
  processingTimes = {0.002, 0.002, 0.012, 0.012, 0.008, 0.002, 0.002};
  summedStepTimes = {0.00, 0.01, 0.02, 0.032, 0.044, 0.052, 0.06, 0.07};
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processingTimes.size(); i++) {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[i]);
    doSomething(processingTimes[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()]);
}

TEST(RateTest, DISABLED_SleepWithoutEnforceRate) {  // NOLINT
  const double timeStep = 0.1;
  any_worker::Rate rate("Test", timeStep);
  rate.getOptions().enforceRate_ = false;

  timespec start{};
  timespec end{};
  const double processingTime = 0.05;
  std::vector<double> processingTimes;
  std::vector<double> summedStepTimes;

  // Test sleep() without processing.
  clock_gettime(CLOCK_MONOTONIC, &start);
  rate.reset();
  rate.sleep();
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), timeStep);

  // Test sleep() with processing.
  clock_gettime(CLOCK_MONOTONIC, &start);
  rate.reset();
  doSomething(processingTime);
  rate.sleep();
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), timeStep);

  // Test sleep() with where one step takes too long.
  processingTimes = {0.02, 0.02, 0.15, 0.02, 0.02};
  summedStepTimes = {0.0, 0.1, 0.2, 0.35, 0.45, 0.55};
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processingTimes.size(); i++) {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[i]);
    doSomething(processingTimes[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()]);

  // Test sleep() with where two steps take too long.
  processingTimes = {0.02, 0.02, 0.12, 0.12, 0.02, 0.02};
  summedStepTimes = {0.0, 0.1, 0.2, 0.32, 0.44, 0.54, 0.64};
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processingTimes.size(); i++) {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[i]);
    doSomething(processingTimes[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR_TEST_TOL(any_worker::Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()]);
}

TEST(RateTest, WarningsAndErrors) {  // NOLINT
  const double timeStep = 0.1;
  any_worker::Rate rate("Test", timeStep);
  doSomething(0.5 * timeStep);  // Ok
  rate.sleep();
  doSomething(2.0 * timeStep);  // Warning
  rate.sleep();
  doSomething(3.0 * timeStep);  // Warning
  rate.sleep();
  doSomething(11.0 * timeStep);  // Error
  rate.sleep();

  EXPECT_EQ(rate.getNumTimeSteps(), 4u);
  EXPECT_EQ(rate.getNumWarnings(), 2u);
  EXPECT_EQ(rate.getNumErrors(), 1u);
}

TEST(RateTest, DISABLED_StatisticsWithEnforceRate) {  // NOLINT
  const double timeStep = 0.1;
  any_worker::Rate rate("Test", timeStep);
  rate.getOptions().enforceRate_ = true;

  // Test 1 time step.
  const double processingTime = 0.05;
  rate.reset();
  doSomething(processingTime);
  rate.sleep();

  EXPECT_EQ(rate.getNumTimeSteps(), 1u);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTime(), processingTime);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeMean(), processingTime);
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));

  // Test 10 time steps with similar processing times.
  const unsigned int numTimeSteps = 10;
  rate.reset();
  for (unsigned int i = 0; i < numTimeSteps; i++) {
    doSomething(processingTime);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), numTimeSteps);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTime(), processingTime);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeMean(), processingTime);
  EXPECT_LE(rate.getAwakeTimeStdDev(), RATE_TEST_TOL);
  EXPECT_FALSE(rate.getAwakeTimeStdDev() == 0.0);  // If it is 0.0 something is fishy.

  // Test 9 time steps with different processing times.
  const std::vector<double> processingTimes = {0.04, 0.02, 0.04, 0.07, 0.05, 0.05, 0.04, 0.09, 0.05};
  rate.reset();
  for (double processingTime : processingTimes) {
    doSomething(processingTime);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), processingTimes.size());
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTime(), *processingTimes.rbegin());
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeMean(), 0.05);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeStdDev(), 0.02);

  // Test again with time step violation.
  rate.getOptions().timeStep_ = 0.035;
  rate.reset();
  for (double processingTime : processingTimes) {
    doSomething(processingTime);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), processingTimes.size());
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTime(), *processingTimes.rbegin());
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeMean(), 0.05);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeStdDev(), 0.02);
}

TEST(RateTest, DISABLED_StatisticsWithoutEnforceRate) {  // NOLINT
  const double timeStep = 0.1;
  any_worker::Rate rate("Test", timeStep);
  rate.getOptions().enforceRate_ = false;

  // Test 1 time step.
  const double processingTime = 0.05;
  rate.reset();
  doSomething(processingTime);
  rate.sleep();

  EXPECT_EQ(rate.getNumTimeSteps(), 1u);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTime(), processingTime);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeMean(), processingTime);
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));

  // Test 10 time steps with similar processing times.
  const unsigned int numTimeSteps = 10;
  rate.reset();
  for (unsigned int i = 0; i < numTimeSteps; i++) {
    doSomething(processingTime);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), numTimeSteps);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTime(), processingTime);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeMean(), processingTime);
  EXPECT_LE(rate.getAwakeTimeStdDev(), RATE_TEST_TOL);
  EXPECT_FALSE(rate.getAwakeTimeStdDev() == 0.0);  // If it is 0.0 something is fishy.

  // Test 9 time steps with different processing times.
  const std::vector<double> processingTimes = {0.04, 0.02, 0.04, 0.07, 0.05, 0.05, 0.04, 0.09, 0.05};
  rate.reset();
  for (double processingTime : processingTimes) {
    doSomething(processingTime);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), processingTimes.size());
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTime(), *processingTimes.rbegin());
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeMean(), 0.05);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeStdDev(), 0.02);

  // Test again with time step violation.
  rate.getOptions().timeStep_ = 0.035;
  rate.reset();
  for (double processingTime : processingTimes) {
    doSomething(processingTime);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), processingTimes.size());
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTime(), *processingTimes.rbegin());
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeMean(), 0.05);
  EXPECT_NEAR_TEST_TOL(rate.getAwakeTimeStdDev(), 0.02);
}
