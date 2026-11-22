"""Start the SYSMIC 2.0 RoboCasa simulation with one command.

    ros2 launch sysmic_v2_sim sim.launch.py
    ros2 launch sysmic_v2_sim sim.launch.py layout:=5 style:=12
    ros2 launch sysmic_v2_sim sim.launch.py rviz:=false
    ros2 launch sysmic_v2_sim sim.launch.py viewer:=false        # headless simulation
    ros2 launch sysmic_v2_sim sim.launch.py sensors:=/path/to/my_sensors.yaml

Starts four things:
  1. the RoboCasa simulation, using the Python of the "robocasa" conda env
  2. the ROS 2 bridge (/clock, /joint_states, /odom, /tf, /cmd_vel, /sysmic/joint_command)
  3. robot_state_publisher with the SYSMIC URDF
  4. RViz

Do not activate conda in the terminal you launch from. The launch file finds the
env's Python by itself; pass conda_python:=/path/to/envs/robocasa/bin/python if it
is installed somewhere unusual, or set SYSMIC_SIM_PYTHON (the Docker image does).

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


def urdf_path(share):
    """sysmic.urdf, installed from the package's urdf/ into share/sysmic_v2_sim/urdf."""
    return os.path.join(share, "urdf", "sysmic.urdf")


def load_sensors_module(share):
    """sysmic_v2_robocasa/sysmic_sensors.py (needs only numpy + PyYAML here)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "sysmic_sensors", os.path.join(share, "sysmic_v2_robocasa", "sysmic_sensors.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def robot_description(share, sensors_cfg=None, sensors_module=None):
    """The URDF for RViz: absolute mesh paths, the same link colours and "SYSMIC"
    lettering as the RoboCasa model (sim/sysmic_colors.py), and a fixed frame for
    every enabled sensor in sensors.yaml. sysmic.urdf itself is not edited."""
    import copy
    import importlib.util
    import xml.etree.ElementTree as ET

    spec = importlib.util.spec_from_file_location(
        "sysmic_colors", os.path.join(share, "sim", "sysmic_colors.py"))
    colors = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(colors)

    path = urdf_path(share)
    folder = os.path.dirname(os.path.realpath(path))
    root = ET.parse(path).getroot()
    for mesh in root.iter("mesh"):
        f = mesh.get("filename", "")
        if f.startswith("meshes/"):
            mesh.set("filename", f"file://{folder}/{f}")
    for link in root.findall("link"):
        name = link.get("name", "")
        for vis in link.findall("visual"):
            if vis.find("material") is not None:
                continue
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                parts = [(None, colors.link_color(name))]
            else:
                parts = colors.split_decals(name, mesh.get("filename")[len("file://"):])
            if parts[0][0] is not None:
                mesh.set("filename", f"file://{parts[0][0]}")
            mat = ET.SubElement(vis, "material", name=f"{name}_mat")
            ET.SubElement(mat, "color", rgba=parts[0][1])
            # A decal ("SYSMIC" lettering) goes on its own link, fixed to this one:
            # RViz colours every visual of a link with the first visual's material.
            for mesh_file, rgba in parts[1:]:
                tag = os.path.splitext(os.path.basename(mesh_file))[0]
                decal = ET.SubElement(root, "link", name=tag)
                v = copy.deepcopy(vis)
                v.find("geometry/mesh").set("filename", f"file://{mesh_file}")
                v.find("material").set("name", f"{tag}_mat")
                v.find("material/color").set("rgba", rgba)
                decal.append(v)
                joint = ET.SubElement(root, "joint", name=f"{tag}_joint", type="fixed")
                ET.SubElement(joint, "parent", link=name)
                ET.SubElement(joint, "child", link=tag)
                ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
    # sensor frames (<sensor>_link and camera optical frames), from sensors.yaml
    if sensors_cfg is not None:
        boxes = {f"{k}_link": v for k, v in sensors_module.bodies(sensors_cfg).items()}
        for parent, child, xyz, rpy in sensors_module.tf_frames(sensors_cfg):
            link = ET.SubElement(root, "link", name=child)
            if child in boxes:  # `show: true`: the sensor's box, as in the simulation
                b = boxes[child]
                vis = ET.SubElement(link, "visual")
                ET.SubElement(vis, "origin", xyz=" ".join(f"{v:.6f}" for v in b["centre_in_sensor"]),
                              rpy="0 0 0")
                ET.SubElement(ET.SubElement(vis, "geometry"), "box",
                              size=" ".join(f"{2 * v:.6f}" for v in b["half"]))
                mat = ET.SubElement(vis, "material", name=f"{child}_mat")
                ET.SubElement(mat, "color", rgba=b["rgba"])
            joint = ET.SubElement(root, "joint", name=f"{child}_joint", type="fixed")
            ET.SubElement(joint, "parent", link=parent)
            ET.SubElement(joint, "child", link=child)
            ET.SubElement(joint, "origin", xyz=" ".join(f"{v:.6f}" for v in xyz),
                          rpy=" ".join(f"{v:.6f}" for v in rpy))
    return ET.tostring(root, encoding="unicode")


def desktop_env():
    """This environment without the variables a snap app (e.g. VS Code's terminal) sets.

    Snap VS Code points GTK / GIO / GSettings at its own folders. The system RViz then
    aborts with "GLib-GIO-ERROR: Settings schema ... does not contain a key named
    'antialiasing'". Restore the desktop's values for RViz only.
    """
    env = dict(os.environ)
    if "SNAP" not in env:
        return env
    for key in ("GTK_PATH", "GTK_EXE_PREFIX", "GTK_IM_MODULE_FILE", "GIO_MODULE_DIR",
                "GSETTINGS_SCHEMA_DIR", "LOCPATH", "GIO_LAUNCHED_DESKTOP_FILE",
                "GIO_LAUNCHED_DESKTOP_FILE_PID"):
        env.pop(key, None)
    orig = env.pop("XDG_DATA_DIRS_VSCODE_SNAP_ORIG", None)
    if orig is not None:
        env["XDG_DATA_DIRS"] = orig
    else:
        env["XDG_DATA_DIRS"] = ":".join(
            d for d in env.get("XDG_DATA_DIRS", "").split(":") if "/snap/" not in d
        ) or "/usr/local/share:/usr/share"
    return env


def launch_setup(context):
    share = get_package_share_directory("sysmic_v2_sim")
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    env_name = arg("conda_env")
    # conda_python:=..., else $SYSMIC_SIM_PYTHON (set in the Docker image), else the conda env
    python = (arg("conda_python") or os.environ.get("SYSMIC_SIM_PYTHON")
              or find_conda_python(env_name))
    if not python or not os.path.isfile(python):
        return [
            LogInfo(msg=(
                f"\n[sysmic_v2_sim] Could not find the Python of conda env '{env_name}'.\n"
                "  Install it with:  bash install_sim.sh   (in the workspace folder)\n"
                "  or pass:          conda_python:=/path/to/envs/robocasa/bin/python\n")),
            EmitEvent(event=Shutdown(reason="conda env not found")),
        ]

    sensors_module = load_sensors_module(share)
    sensors_path = os.path.abspath(os.path.expanduser(
        arg("sensors") or os.path.join(share, "sysmic_v2_robocasa", "sensors.yaml")))
    sensors_cfg = sensors_module.load(sensors_path)

    simulation = ExecuteProcess(
        cmd=[python, os.path.join(share, "sim", "sysmic_sim_server.py"),
             "--layout", arg("layout"), "--style", arg("style"), "--viewer", arg("viewer")],
        # The simulation runs on the conda Python (3.11). Keep the ROS Python 3.10
        # packages off its path, and make its prints show up immediately.
        additional_env={"PYTHONPATH": "", "PYTHONUNBUFFERED": "1",
                        "SYSMIC_URDF": os.path.realpath(urdf_path(share)),
                        "SYSMIC_SENSORS": sensors_path},
        output="screen",
    )
    sim_time = {"use_sim_time": True}
    actions = [
        LogInfo(msg=f"[sysmic_v2_sim] simulation python: {python}"),
        LogInfo(msg=f"[sysmic_v2_sim] sensors: {sensors_path}"),
        simulation,
        Node(package="sysmic_v2_sim", executable="ros2_bridge.py", output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(
                 robot_description(share, sensors_cfg, sensors_module), value_type=str)},
                 sim_time],
             output="screen"),
        # closing the simulation ends the whole launch
        RegisterEventHandler(OnProcessExit(
            target_action=simulation,
            on_exit=[EmitEvent(event=Shutdown(reason="simulation closed"))])),
    ]
    if arg("rviz").lower() in ("true", "1", "yes"):
        actions.append(Node(package="rviz2", executable="rviz2",
                            arguments=["-d", os.path.join(share, "rviz", "sysmic.rviz")],
                            parameters=[sim_time], env=desktop_env(), output="screen"))
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
        DeclareLaunchArgument("sensors", default_value="",
                              description="sensors.yaml to use (default: the package's "
                                          "sysmic_v2_robocasa/sensors.yaml)"),
        DeclareLaunchArgument("conda_env", default_value="robocasa",
                              description="Name of the conda env with RoboCasa"),
        DeclareLaunchArgument("conda_python", default_value="",
                              description="Full path to that env's python (auto-detected if empty)"),
        OpaqueFunction(function=launch_setup),
    ])
