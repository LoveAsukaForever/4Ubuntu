# Gazebo 仿真（TurtleBot3 替身 + `/robot2/cmd_vel`）

本仓库没有自有 URDF 时，用 **TurtleBot3** 提供可动的差速模型；与 `serial_hw`、`move_base` 一致，**速度指令统一走 `/robot2/cmd_vel`**（见 `run_my_planner.launch` 与 `run_serial_hw.launch`）。

## 依赖（Ubuntu / WSL + ROS Noetic 示例）

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-turtlebot3-gazebo \
  ros-noetic-turtlebot3-description \
  ros-noetic-topic-tools \
  ros-noetic-teleop-twist-keyboard
```

## 一键：Gazebo + 键盘遥控

```bash
source ~/badminton_ws/devel/setup.bash   # 按你的 catkin 路径改
export TURTLEBOT3_MODEL=burger            # 或与 launch 默认一致
roslaunch move_control sim_tb3_teleop.launch
```

终端里用键盘发速度 → 话题为 `/robot2/cmd_vel` → `relay` → TurtleBot3 的 `/cmd_vel`。

## 一键：Gazebo + 录包（第 4 章）

另开终端：

```bash
source ~/badminton_ws/devel/setup.bash
roslaunch move_control sim_tb3_record.launch bag:=/tmp/tb3_exp1
```

会录制 `/robot2/cmd_vel`、`/cmd_vel`、`/odom`、`/joint_states`。论文中说明：**仿真验证运动学与 ROS 调度；实机验证 STM32 协议与执行器。**

## 与 move_base 联调（可选）

```bash
roslaunch move_control sim_tb3_with_move_base.launch
```

无激光/地图时可能无法规划，仅用于检查节点与 remap 是否正常。

## 与 `pj_batmin` 下位机的关系

仿真栈**不打开串口**；实机实验再 `roslaunch serial_hw run_serial_hw.launch`。两者共用同一套 **话题命名习惯**（尤其 `/robot2/cmd_vel`）。
