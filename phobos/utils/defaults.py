import pkg_resources
import json

with open(pkg_resources.resource_filename("phobos", "data/defaults.json"), "r") as f:
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
