/**
 * @file test_gdb.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 测试GDB调试的具体流程
 * @version 1.0
 * @date 2025/10/26
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <iostream>
#include <ros/ros.h>

struct Student_t {
  int age;
  char name[100];
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_gdb");
  ros::NodeHandle n;
  double display = 0;
  int waitChange = 0;
  int tick = 0;
  struct Student_t HM_ZC = {10, "ZhongChi"};

  while (1) {
    if (tick++ > 1000) {
      display += 0.001;
      tick = 0;
      std::cout << display << std::endl;
    }
  }

  return 0;
}
