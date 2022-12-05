EULER_CONVENTION = 'xyz'
RPY_CONVENTION = 'xyz'
MESH_TYPES = ["dae", "stl", "obj", "mars_obj", "bobj"]
EXPORT_TYPES = ["smurf", "urdf", "sdf", "joint_limits", "pdf"]
IMPORT_TYPES = ["smurf", "urdf", "sdf"]
KINEMATIC_TYPES = ["urdf", "sdf"]

HYRODYN_AVAILABLE = False
BASE_LOG_LEVEL = "WARNING"
LOG_FILE_CONVENTION = None

try:
    import hyrodyn
    import os
    if "AUTOPROJ_CURRENT_ROOT" in os.environ.keys():
        HYRODYN_AVAILABLE = True
        print("HyRoDyn tests available.")
    else:
        HYRODYN_AVAILABLE = False
        print("HyRoDyn can't be used as the autoproj env hasn't be sourced.")
    del os
except ImportError:
    print("HyRoDyn tests not available.")

BPY_AVAILABLE = False
try:
    import bpy
    BPY_AVAILABLE = True
    print("Blender-Python (bpy) available.")
except ImportError:
    print("Blender-Python (bpy) not available.")

PYBULLET_AVAILABLE = False
try:
    import pybullet as pb
    PYBULLET_AVAILABLE = True
    print("Pybullet tests available.")
except ImportError:
    print("Pybullet tests not available.")

DEIMOS_AVAILABLE = False
try:
    from deimos.deimos import Deimos
    DEIMOS_AVAILABLE = True
    print("Deimos available.")
except ImportError:
    print("Deimos not available.")

XDBI_AVAILABLE = False
try:
    import xtypes_py
    import xdbi_py
    XDBI_AVAILABLE = True
    print("XDBI available.")
except ImportError:
    print("XDBI not available.")


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
    print("YAML available (backwards compatibility).")
except ImportError:
    from json import loads as load_json, dumps
    dump_yaml = dump_json
    print("YAML not available (backwards compatibility).")
