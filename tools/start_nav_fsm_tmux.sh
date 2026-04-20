#!/usr/bin/env bash
set -euo pipefail

SESSION="navfsm"

source /opt/ros/noetic/setup.bash
source ~/badminton_ws/devel/setup.bash

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION"
  echo "Attach with: tmux attach -t $SESSION"
  exit 0
fi

tmux new-session -d -s "$SESSION" -n core

# 1) Gazebo（三全向）
tmux send-keys -t "$SESSION:core" \
"source /opt/ros/noetic/setup.bash; source ~/badminton_ws/devel/setup.bash; roslaunch omni3ros_pkg urdf_gazebo_view.launch" C-m

# 2) SLAM
tmux split-window -v -t "$SESSION:core"
tmux send-keys -t "$SESSION:core.1" \
"source /opt/ros/noetic/setup.bash; source ~/badminton_ws/devel/setup.bash; rosrun gmapping slam_gmapping scan:=/scan _base_frame:=base_footprint _odom_frame:=odom" C-m

# 3) move_base (MyPlanner)
tmux split-window -h -t "$SESSION:core.0"
tmux send-keys -t "$SESSION:core.2" \
"source /opt/ros/noetic/setup.bash; source ~/badminton_ws/devel/setup.bash; roslaunch move_control run_my_planner.launch" C-m

# 4) nav relay: /robot2/cmd_vel -> /cmd_vel_nav
tmux split-window -h -t "$SESSION:core.1"
tmux send-keys -t "$SESSION:core.3" \
"source /opt/ros/noetic/setup.bash; source ~/badminton_ws/devel/setup.bash; python3 - <<'PY'
import rospy
from geometry_msgs.msg import Twist
pub=rospy.Publisher('/cmd_vel_nav', Twist, queue_size=10)
def cb(m): pub.publish(m)
rospy.init_node('robot2_to_cmd_vel_nav_relay', anonymous=True)
rospy.Subscriber('/robot2/cmd_vel', Twist, cb, queue_size=10)
rospy.spin()
PY" C-m

# 5) twist_mux
tmux new-window -t "$SESSION" -n mux
tmux send-keys -t "$SESSION:mux" \
"source /opt/ros/noetic/setup.bash; source ~/badminton_ws/devel/setup.bash; rosrun twist_mux twist_mux /cmd_vel_out:=/cmd_vel _yaml_cfg_file:=$(rospack find move_control)/config/twist_mux_fsm_nav.yaml" C-m

# 6) FSM (remap to /cmd_vel_fsm)
tmux new-window -t "$SESSION" -n fsm
tmux send-keys -t "$SESSION:fsm" \
"source /opt/ros/noetic/setup.bash; source ~/badminton_ws/devel/setup.bash; rosrun move_control robot_fsm_node cmd_vel:=/cmd_vel_fsm" C-m

# 7) RViz（可选）
tmux new-window -t "$SESSION" -n rviz
tmux send-keys -t "$SESSION:rviz" \
"source /opt/ros/noetic/setup.bash; source ~/badminton_ws/devel/setup.bash; rviz" C-m

echo "Started tmux session: $SESSION"
echo "Attach with: tmux attach -t $SESSION"

