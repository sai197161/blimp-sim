"""Launch Gazebo Fortress with the blimp test world + ROS 2 bridge.

Run without building a package:
  source /opt/ros/humble/setup.bash
  ros2 launch ~/sai/sim_ws/src/blimp-sim/launch/blimp_sim.launch.py
Headless (no GUI, e.g. over ssh):
  ros2 launch .../blimp_sim.launch.py headless:=true
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODELS = os.path.join(ROOT, "models")
WORLDS = os.path.join(ROOT, "worlds")
WORLD = os.path.join(WORLDS, "blimp_test.sdf")
BRIDGE_CFG = os.path.join(ROOT, "config", "bridge.yaml")


def _append(var, path):
    old = os.environ.get(var, "")
    return path + (os.pathsep + old if old else "")


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    res = MODELS + os.pathsep + WORLDS
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        # Fortress reads IGN_GAZEBO_RESOURCE_PATH; newer reads GZ_SIM_RESOURCE_PATH. Set both.
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", _append("IGN_GAZEBO_RESOURCE_PATH", res)),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _append("GZ_SIM_RESOURCE_PATH", res)),

        ExecuteProcess(cmd=["ign", "gazebo", "-r", "-v", "3", WORLD],
                       output="screen", condition=UnlessCondition(headless)),
        ExecuteProcess(cmd=["ign", "gazebo", "-r", "-s", "--headless-rendering", "-v", "3", WORLD],
                       output="screen", condition=IfCondition(headless)),

        Node(package="ros_gz_bridge", executable="parameter_bridge", name="blimp_bridge",
             parameters=[{"config_file": BRIDGE_CFG, "use_sim_time": True}], output="screen"),
    ])
