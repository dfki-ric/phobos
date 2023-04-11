#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Parses the scene entities at import and provides them as :data:`scene_types`.
"""

import os
import importlib.util

scene_types = dict()
structure_export_folders = []

# this creates a dict entry for every python file in this subfolder
for filename in os.listdir(os.path.dirname(__file__)):
    mod_name, file_ext = os.path.splitext(os.path.split(filename)[-1])

    # only take .py files and ignore this __init__ file
    if (filename != os.path.split(__file__)[-1]) and (file_ext.lower() == '.py'):
        modpath = os.path.join(os.path.dirname(__file__), filename)

        # load the module from file and source it to access its data
        spec = importlib.util.spec_from_file_location(filename, modpath)
        py_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(py_mod)

        # try reading the entity dictionary and add it to the existing entities
        if hasattr(py_mod, 'scene_type_dict'):
            scene_types.update(py_mod.scene_type_dict.copy())
            print('Registered scene plugin:', list(py_mod.scene_type_dict.keys()))
        else:
            print(
                'ERROR in scenes/__init__: "' + filename + '" has no valid scene plugin interface.'
            )
