#!/usr/bin/python
# coding=utf-8

import bpy
from phobos.phoboslog import log
from phobos.utils.blender import createPreview


def exportPreview(model, path):
    """This function exports a given robot model to a specified filepath as YAML.

    Args:
      model(dict -- the generated robot model dictionary): The robot model to export
      path(str): The filepath to export the robot to. *WITH filename!*

    Returns:

    """
    log("Phobos Thumbnail export: Creating thumbnail in " + path, "INFO")
    visuals = []
    for linkname in model['links']:
        for visualname in model['links'][linkname]['visual']:
            visuals.append(bpy.data.objects[visualname])
    createPreview(visuals, path, model['name'])


# registering export functions of types with Phobos
entity_type_dict = {'thumbnails': {'export': exportPreview,
                                   'extensions': ('png', 'jpg')}
                    }
