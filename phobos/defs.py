
EULER_CONVENTION = 'xyz'
RPY_CONVENTION = 'xyz'

HYRODYN_AVAILABLE = False
BASE_LOG_LEVEL = "WARNING"
LOG_FILE_CONVENTION = None

try:
    import hyrodyn
    import os
    if "AUTOPROJ_CURRENT_ROOT" in os.environ.keys():
        HYRODYN_AVAILABLE = True
        print("HyRoDyn tests available.", flush=True)
    else:
        HYRODYN_AVAILABLE = False
        print("HyRoDyn can't be used as the autoproj env hasn't be sourced.", flush=True)
    del os
except ImportError:
    print("HyRoDyn tests not available.", flush=True)

BPY_AVAILABLE = False
try:
    import bpy
    BPY_AVAILABLE = True
    print("Blender-Python (bpy) available.", flush=True)
except ImportError:
    print("Blender-Python (bpy) not available.", flush=True)

PYBULLET_AVAILABLE = False
try:
    import pybullet as pb
    PYBULLET_AVAILABLE = True
    print("Pybullet tests available.", flush=True)
except ImportError:
    print("Pybullet tests not available.", flush=True)

DEIMOS_AVAILABLE = False
try:
    from deimos.deimos import Deimos
    DEIMOS_AVAILABLE = True
    print("Deimos available.", flush=True)
except ImportError:
    print("Deimos not available.", flush=True)


def dump_json(obj, **kwargs):
    import json
    if "default_flow_style" in kwargs:
        kwargs.pop("default_flow_style")
    if "default_style" in kwargs:
        kwargs.pop("default_style")
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

MESH_TYPES = ["dae", "stl", "obj", "mars_obj", "bobj"]
ENTITY_TYPES = ["smurf", "urdf", "sdf", "srdf", "joint_limits", "pdf"]