#!/usr/bin/python
# coding=utf-8

"""
..module:: phobos
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

..moduleauthor:: Kai von Szadowski, Ole Schwiegert, Stefan Rahms, Malte Langosz, Sebastian Klemp
Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

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

File __init__.py

Created on 6 Jan 2014

"""

import sys
import os.path

bl_info = {
    "name": "Phobos",
    "description": "A set of tools to enable editing of MARS robot models in Blender.",
    "author": "Kai von Szadkowski, Ole Schwiegert, Stefan Rahms, Malte Langosz",
    "version": (0, 7),
    "blender": (2, 69, 0),
    "location": "Phobos adds a number of custom tool panels.",
    "warning": "",
    "wiki_url": "",
    "support": "COMMUNITY",
    "tracker_url": "https://github.com/rock-simulation/phobos/issues",
    "category": "Development"
}

yamlconfpath = os.path.dirname(__file__) + "/yamlpath.conf"
if os.path.isfile(yamlconfpath):
    f = open(yamlconfpath)
    path = f.read()
    f.close()
    if (path == "v" or path == "i"):
        print("There is no YAML installation for python 3.4 or greater on this computer")
    else:
        print("Importing yaml module")
        sys.path.insert(0, path)
        import yaml
else:
    print("Could not find yamlpath.conf")
    print("Using distributed package instead!")
    sys.path.insert(0, sys.path[0] + "/phobos")
    import yaml

    print("Importing yaml module")

# prepare defs module
import phobos.defs as defs
import os

# Add custom YAML (de-)serializer
def bool_representer(dumper, data):
    if data == '$true':
        return dumper.represent_bool(True)
    elif data == '$false':
        return dumper.represent_bool(False)
    else:
        return dumper.represent_str(str(data))


yaml.add_representer(str, bool_representer)


def bool_constructor(self, node):
    value = self.construct_yaml_bool(node)
    return '$true' if value else '$false'


yaml.Loader.add_constructor(u'tag:yaml.org,2002:bool', bool_constructor)
yaml.SafeLoader.add_constructor(u'tag:yaml.org,2002:bool', bool_constructor)

if "bpy" in locals():
    import importlib
    importlib.reload(phobos.defs)
    print("Using following folder for defs: " + os.path.dirname(__file__) + "/definitions")
    defs.updateDefs(os.path.dirname(__file__) + "/definitions")
    importlib.reload(phobos.utils.validation)
    importlib.reload(phobos.utils.selection)
    importlib.reload(phobos.utils.io)
    importlib.reload(phobos.utils.general)
    importlib.reload(phobos.utils.naming)
    importlib.reload(phobos.utils.blender)
    importlib.reload(phobos.testing)
    importlib.reload(phobos.logging)
    importlib.reload(phobos.phobosgui)
    importlib.reload(phobos.model.controllers)
    importlib.reload(phobos.model.joints)
    importlib.reload(phobos.model.materials)
    importlib.reload(phobos.model.poses)
    importlib.reload(phobos.model.inertia)
    importlib.reload(phobos.model.sensors)
    importlib.reload(phobos.model.models)
    importlib.reload(phobos.model.links)
    importlib.reload(phobos.model.geometries)
    importlib.reload(phobos.model.lights)
    importlib.reload(phobos.io.meshes.meshes)
    importlib.reload(phobos.io.entities.smurf)
    importlib.reload(phobos.io.entities.primitive)
    importlib.reload(phobos.io.entities.light)
    importlib.reload(phobos.io.entities.urdf)
    importlib.reload(phobos.io.entities.heightmap)
    importlib.reload(phobos.io.entities.generic)
    importlib.reload(phobos.io.scenes.smurfs)
    importlib.reload(phobos.operators.misc)
    importlib.reload(phobos.operators.editing)
    importlib.reload(phobos.operators.selection)
    importlib.reload(phobos.operators.io)
    importlib.reload(phobos.operators.naming)
    print("Reloading Phobos.")
else:

    print("Using following folder for defs: " + os.path.dirname(__file__) + "/definitions")
    defs.updateDefs(os.path.dirname(__file__) + "/definitions")
    import phobos.utils.validation
    import phobos.utils.selection
    import phobos.utils.io
    import phobos.utils.general
    import phobos.utils.naming
    import phobos.utils.blender
    import phobos.testing
    import phobos.logging
    import phobos.phobosgui
    import phobos.model.controllers
    import phobos.model.joints
    import phobos.model.materials
    import phobos.model.poses
    import phobos.model.inertia
    import phobos.model.sensors
    import phobos.model.models
    import phobos.model.links
    import phobos.model.geometries
    import phobos.model.lights
    import phobos.io.meshes.meshes
    import phobos.io.entities.smurf
    import phobos.io.entities.primitive
    import phobos.io.entities.light
    import phobos.io.entities.urdf
    import phobos.io.entities.heightmap
    import phobos.io.entities.generic
    import phobos.io.scenes.smurfs
    import phobos.operators.misc
    import phobos.operators.editing
    import phobos.operators.selection
    import phobos.operators.io
    import phobos.operators.naming
    print("Importing Phobos modules.")

import bpy

def register():
    """This function registers all modules to blender.

    :return: Nothing

    """
    phobos.defs.register()
    phobos.phobosgui.register()
    bpy.utils.register_module(__name__)


def unregister():
    """This function unregisters all modules to blender.

    :return: Nothing

    """
    phobos.defs.unregister()
    phobos.phobosgui.unregister()
    bpy.utils.unregister_module(__name__)

# if __name__ == "__main__":
#    register()
