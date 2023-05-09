import json
import os
from .misc import merge_default
from ..defs import BPY_AVAILABLE


def get_resources_path(*filepath):
    path = None
    try:
        import pkg_resources
        path = pkg_resources.resource_filename("phobos", os.path.join("data", *filepath))
    except ImportError:
        path = os.path.join(os.path.dirname(__file__), "..", "data")
        if len(filepath) > 0:
            path = os.path.join(path, *filepath)
    return os.path.normpath(path)


def get_user_resources_path():
    path = os.getenv("PHOBOS_CONFIG_PATH", None)
    if BPY_AVAILABLE:
        # [Todo v2.1.0] Re-add user config dir in phobos settings and use that path here
        pass
    if path is not None:
        return os.path.normpath(path)
    return None


with open(get_resources_path("defaults.json"), "r") as f:
    DEFAULTS = json.load(f)
if get_user_resources_path() is not None:
    with open(get_user_resources_path(), "r") as f:
        DEFAULTS = merge_default(json.load(f), DEFAULTS)


def get_sensor(sensor_category, sensor_type = "default"):
    if len(sensor_category) == 0:
        return {}
    if len(sensor_type) == 0:
        sensor_type = "default"
    try:
        return DEFAULTS["sensors"][sensor_category][sensor_type]
    except KeyError:
        return {}


def get_sensor_info(sensor_category):
    result = {
        "blender_type": "Ray_sensor"
    }
    cat = DEFAULTS["sensors"][sensor_category]
    info = [] if "" not in cat else cat[""]
    for key in info:
        result[key] = info[key]
    return result


def get_sensor_categories():
    return list(DEFAULTS["sensors"].keys())


def get_sensor_types(sensor_category):
    sensors = DEFAULTS["sensors"][sensor_category].keys()
    result = []
    for key in sensors:
        if len(key) > 0:
            result.append(key)
    return sensors


def get_default_motor(motor_definition="default"):
    return DEFAULTS["motors"][motor_definition.lower()]


def get_motor_defaults():
    return list(DEFAULTS["motors"].keys())


def get_default_joint(joint_type):
    return DEFAULTS["joints"][joint_type.lower()]


def get_default_ci_test_definition():
    return DEFAULTS["ci_test"]


def get_default_ci_deploy_definition():
    return DEFAULTS["ci_deploy"]


def get_default_export_config(version="default"):
    return DEFAULTS["export_config"][version]


def get_default_rel_mesh_pathes():
    return DEFAULTS["rel_mesh_pathes"]


def get_blender_resources_path(*filepath):
    return get_resources_path("blender", *filepath)
