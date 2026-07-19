#!/bin/bash
# Set up the Kinova Gen3 6DOF mock-hardware sim (MoveIt2 + RViz) on ROS2 Jazzy.
# Should be safe to re-run. Requires sudo (apt + patching /opt files).
set -euo pipefail

XACRO="/opt/ros/jazzy/share/kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro"
LAUNCH="/opt/ros/jazzy/share/kinova_gen3_6dof_robotiq_2f_85_moveit_config/launch/robot.launch.py"

echo "==> [1/4] Installing packages (moveit config pulls deps; joint-state-broadcaster is missing from them)"
sudo apt update
sudo apt install -y "ros-jazzy-kinova-gen3-6dof-robotiq-2f-85-moveit-config" "ros-jazzy-joint-state-broadcaster"

echo "==> [2/4] Patching $XACRO"
if [ -f "$XACRO.bak" ]; then
  echo "    .bak already exists -- assuming patched, skipping."
else
  sudo cp "$XACRO" "$XACRO.bak"
  # Fix 1: version skew -- kortex 0.2.6 passes params robotiq 0.0.1's macro doesn't have.
  # Fix 2: disable the gripper's ros2_control (mimic joints abort controller_manager on Jazzy).
  sudo sed -i \
    -e 's/mock_sensor_commands="${fake_sensor_commands}"/fake_sensor_commands="${fake_sensor_commands}"/' \
    -e 's/sim_gazebo="${sim_gazebo}"/sim_ignition="${sim_gazebo}"/' \
    -e 's#sim_isaac="${sim_isaac}"#sim_isaac="${sim_isaac}">#' \
    -e '/isaac_joint_commands=/d' \
    -e '/isaac_joint_states=/d' \
    -e 's/include_ros2_control="${include_ros2_control}"/include_ros2_control="false"/' \
    "$XACRO"
  echo "    patched (backup at $XACRO.bak)"
fi

echo "==> [3/4] Patching $LAUNCH"
if [ -f "$LAUNCH.bak" ]; then
  echo "    .bak already exists -- assuming patched, skipping."
else
  sudo cp "$LAUNCH" "$LAUNCH.bak"
  # Give joints 1/4/6 finite +/-2pi limits (use_external_cable) instead of 'continuous'.
  # Continuous joints must stay in [-pi,pi] or MoveIt's CheckStartStateBounds aborts plans
  # (START_STATE_INVALID, -26) once the unconstrained wrist drifts past pi.
  sudo sed -i 's/        "dof": "6",/&\n        "use_external_cable": "true",/' "$LAUNCH"
  echo "    patched (backup at $LAUNCH.bak)"
fi

echo "==> [3b/4] Registering PILZ pipeline in $LAUNCH (PTP = minimal-joint-change moves)"
# move_group only loads pipelines listed here; stock launch has ompl only, so PILZ
# PTP goals return code=0. Content-guarded (not .bak) so it applies on re-runs too.
if grep -q 'pilz_industrial_motion_planner' "$LAUNCH"; then
  echo "    already registered, skipping."
else
  sudo sed -i 's/pipelines=\["ompl"\]/pipelines=["ompl", "pilz_industrial_motion_planner"]/' "$LAUNCH"
  echo "    registered."
fi

echo "==> [4/4] Done. Launch with:"
cat <<EOF

  ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.1.10 use_fake_hardware:=true

EOF
