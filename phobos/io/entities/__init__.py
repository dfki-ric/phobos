#!/usr/bin/python3.5
# coding=utf-8

"""
..module:: phobos.io.entities
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

..moduleauthor:: Kai von Szadowski, Ole Schwiegert, Stefan Rahms, Malte Langosz, Sebastian Klemp, Simon Reichel

Copyright 2017, University of Bremen & DFKI GmbH Robotics Innovation Center

This file is part of Phobos, a Blender Add-On to edit robot models.

Phobos is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License
as published by the Free Software Foundation, either version 3
of the License, or (at your option) any later version.

Phobos is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Phobos.  If not, see <http://www.gnu.org/licenses/>.

File phobos.io.entitites.__init__.py
"""

import os
import importlib.util

entity_types = dict()

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
        if hasattr(py_mod, 'entity_type_dict'):
            entity_types.update(py_mod.entity_type_dict.copy())
            print('Registered entity plugin:', list(py_mod.entity_type_dict.keys()))
        else:
            print('ERROR in entities/__init__: "' +
                  filename + '" has no valid entity plugin interface.')
