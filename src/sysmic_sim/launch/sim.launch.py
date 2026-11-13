"""Start the whole SYSMIC V2 simulation with one command.

    ros2 launch sysmic_sim sim.launch.py
    ros2 launch sysmic_sim sim.launch.py layout:=5 style:=12
    ros2 launch sysmic_sim sim.launch.py rviz:=false
    ros2 launch sysmic_sim sim.launch.py viewer:=false        # headless simulation

Starts four things:
  1. the RoboCasa simulation, using the Python of the "robocasa" conda env
  2. the ROS 2 bridge (/clock, /joint_states, /odom, /tf, /cmd_vel, /sysmic/joint_command)
  3. robot_state_publisher with the SYSMIC URDF
  4. RViz

Do not activate conda in the terminal you launch from. The launch file finds the
env's Python by itself; pass conda_python:=/path/to/envs/robocasa/bin/python if it
is installed somewhere unusual.

Closing the simulation viewer (or Ctrl+C) stops everything.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def find_conda_python(env_name):
    """Path of the conda env's python, or '' if it cannot be found."""
    home = os.path.expanduser("~")
    roots = []
    conda_exe = os.environ.get("CONDA_EXE")  # set by `conda init`, even when no env is active
    if conda_exe:
        roots.append(os.path.dirname(os.path.dirname(conda_exe)))
    roots += [os.path.join(home, d) for d in
              ("miniconda3", "miniforge3", "mambaforge", "anaconda3")]
    roots.append("/opt/conda")
    for root in roots:
        python = os.path.join(root, "envs", env_name, "bin", "python")
        if os.path.isfile(python):
            return python
    return ""


def robot_description(share):
    """The URDF with mesh paths made absolute, so RViz can load them."""
    description = os.path.join(share, "description")
    with open(os.path.join(description, "sysmic_v2_gazebo.urdf")) as f:
        urdf = f.read()
    return urdf.replace('filename="models/', f'filename="file://{description}/models/')


def launch_setup(context):
    share = get_package_share_directory("sysmic_sim")
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    env_name = arg("conda_env")
    python = arg("conda_python") or find_conda_python(env_name)
    if not python or not os.path.isfile(python):
        return [
            LogInfo(msg=(
                f"\n[sysmic_sim] Could not find the Python of conda env '{env_name}'.\n"
                "  Install it with:  bash install_sim.sh   (in the workspace folder)\n"
                "  or pass:          conda_python:=/path/to/envs/robocasa/bin/python\n")),
            EmitEvent(event=Shutdown(reason="conda env not found")),
        ]

    simulation = ExecuteProcess(
        cmd=[python, os.path.join(share, "sim", "sysmic_sim_server.py"),
             "--layout", arg("layout"), "--style", arg("style"), "--viewer", arg("viewer")],
        # The simulation runs on the conda Python (3.11). Keep the ROS Python 3.10
        # packages off its path, and make its prints show up immediately.
        additional_env={"PYTHONPATH": "", "PYTHONUNBUFFERED": "1"},
        output="screen",
    )
    sim_time = {"use_sim_time": True}
    actions = [
        LogInfo(msg=f"[sysmic_sim] simulation python: {python}"),
        simulation,
        Node(package="sysmic_sim", executable="ros2_bridge.py", output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(robot_description(share),
                                                              value_type=str)}, sim_time],
             output="screen"),
        # closing the simulation ends the whole launch
        RegisterEventHandler(OnProcessExit(
            target_action=simulation,
            on_exit=[EmitEvent(event=Shutdown(reason="simulation closed"))])),
    ]
    if arg("rviz").lower() in ("true", "1", "yes"):
        actions.append(Node(package="rviz2", executable="rviz2",
                            arguments=["-d", os.path.join(share, "rviz", "sysmic.rviz")],
                            parameters=[sim_time], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true",
                              description="Show the MuJoCo viewer window"),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
        DeclareLaunchArgument("layout", default_value="1",
                              description="RoboCasa kitchen layout id"),
        DeclareLaunchArgument("style", default_value="1",
                              description="RoboCasa kitchen style id"),
        DeclareLaunchArgument("conda_env", default_value="robocasa",
                              description="Name of the conda env with RoboCasa"),
        DeclareLaunchArgument("conda_python", default_value="",
                              description="Full path to that env's python (auto-detected if empty)"),
        OpaqueFunction(function=launch_setup),
    ])
