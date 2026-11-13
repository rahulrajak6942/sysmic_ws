"""Where things live. Shared by the simulation scripts."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# Robot description: URDF + meshes. Never modified by the scripts.
DESCRIPTION = os.path.normpath(os.path.join(HERE, "..", "description"))
URDF = os.path.join(DESCRIPTION, "sysmic_v2_gazebo.urdf")

# Generated MuJoCo / robosuite models. Rebuilt automatically when the URDF or
# the scripts change, so this folder can be deleted at any time.
GENERATED = os.environ.get("SYSMIC_SIM_CACHE") or os.path.join(
    os.path.expanduser("~"), ".cache", "sysmic_sim")
ROBOT_XML = os.path.join(GENERATED, "sysmic_v2.xml")
SCENE_XML = os.path.join(GENERATED, "sysmic_v2_scene.xml")
ROBOSUITE_DIR = os.path.join(GENERATED, "robosuite_sysmic")
