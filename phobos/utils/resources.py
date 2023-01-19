import os
import json

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


with open(get_resources_path("defaults.json"), "r") as f:
    DEFAULTS = json.load(f)


def get_default_sensor(sensor_type):
    return DEFAULTS["sensors"][sensor_type]


def get_default_motor(motor_definition="default"):
    return DEFAULTS["motors"][motor_definition.lower()]


def get_default_joint(joint_type):
    return DEFAULTS["joints"][joint_type.lower()]


def get_default_ci_test_definition():
    return DEFAULTS["ci_test"]


def get_default_export_config(version="default"):
    return DEFAULTS["export_config"][version]


def get_blender_resources_path(*filepath):
    return get_resources_path("blender", *filepath)
