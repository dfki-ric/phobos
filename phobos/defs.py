from .commandline_logging import get_logger
log = get_logger(__name__)

EULER_CONVENTION = 'xyz'
RPY_CONVENTION = 'xyz'
MESH_TYPES = ["stl", "obj", "bobj", "dae"]
EXPORT_TYPES = ["smurf", "urdf", "sdf", "joint_limits", "pdf"]
IMPORT_TYPES = ["smurf", "urdf", "sdf"]
KINEMATIC_TYPES = ["urdf", "sdf"]
SCENE_TYPES = ["smurfs", "sdf"]

HYRODYN_AVAILABLE = False
BASE_LOG_LEVEL = None

try:
    import hyrodyn
    import os
    if "AUTOPROJ_CURRENT_ROOT" in os.environ.keys():
        HYRODYN_AVAILABLE = True
        log.info("HyRoDyn tests available.")
    else:
        HYRODYN_AVAILABLE = False
        log.info("HyRoDyn can't be used as the autoproj env hasn't be sourced.")
    del os
except ImportError:
    log.info("HyRoDyn tests not available.")

BPY_AVAILABLE = False
try:
    import bpy
    BPY_AVAILABLE = True
    log.info("Blender-Python (bpy) available.")
except ImportError:
    log.info("Blender-Python (bpy) not available.")

PYBULLET_AVAILABLE = False
try:
    import pybullet as pb
    PYBULLET_AVAILABLE = True
    log.info("Pybullet tests available.")
except ImportError:
    log.info("Pybullet tests not available.")


def dump_json(obj, **kwargs):
    import json
    if "default_flow_style" in kwargs:
        kwargs.pop("default_flow_style")
    if "default_style" in kwargs:
        kwargs.pop("default_style")
    if "indent" not in kwargs:
        kwargs["indent"] = 2
    if "sort_keys" not in kwargs:
        kwargs["sort_keys"] = True
    return json.dumps(obj, **kwargs)


YAML_AVAILABLE = False
try:
    from yaml import safe_load as load_json, safe_dump as dump_yaml
    YAML_AVAILABLE = True
    log.info("YAML available (backwards compatibility).")
except ImportError:
    from json import loads as load_json, dumps
    dump_yaml = dump_json
    log.info("YAML not available (backwards compatibility).")

del get_logger
del log