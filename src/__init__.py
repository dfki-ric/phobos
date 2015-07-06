#!/usr/bin/python

"""
..module:: phobos
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

..moduleauthor:: Kai von Szadowski, Ole Schwiegert
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

bl_info = {
    "name": "Phobos",
    "description": "A set of tools to enable editing of MARS robot models in Blender.",
    "author": "Kai von Szadkowski, Malte Langosz, Stefan Rahms, Ole Schwiegert",
    "version": (0, 3),
    "blender": (2, 69, 0),
    "location": "Phobos adds a number of custom tool panels.",
    "warning": "",
    "wiki_url": "",
    "category": "Development"
}

import sys
import os.path

yamlconfpath = os.path.dirname(__file__) + "/yamlpath.conf"
if (os.path.isfile(yamlconfpath)):
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
from . import defs
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

print("Using following folder for defs: " + os.path.dirname(__file__) + "/definitions")
defs.updateDefs(os.path.dirname(__file__) + "/definitions")

if "bpy" in locals():
    import imp

    imp.reload(phobos.robotupdate)
    imp.reload(phobos.robotdictionary)
    imp.reload(phobos.controllers)
    imp.reload(phobos.exporter)
    imp.reload(phobos.importer)
    imp.reload(phobos.phobosgui)
    imp.reload(phobos.joints)
    imp.reload(phobos.links)
    imp.reload(phobos.sensors)
    imp.reload(phobos.logging)
    imp.reload(phobos.utils.blender)
    imp.reload(phobos.utils.general)
    imp.reload(phobos.utils.selection)
    imp.reload(phobos.utils.naming)
    imp.reload(phobos.operators.io)
    imp.reload(phobos.operators.editing)
    imp.reload(phobos.operators.misc)
    imp.reload(phobos.operators.naming)
    imp.reload(phobos.operators.selection)
    imp.reload(phobos.inertia)
    imp.reload(phobos.marssceneexport)
    imp.reload(phobos.defs)
    print("Reloading Phobos.")
else:
    import phobos.robotupdate, phobos.links, phobos.marssceneexport, phobos.robotdictionary, phobos.controllers, \
        phobos.exporter, phobos.importer, phobos.joints, phobos.sensors, phobos.inertia, \
        phobos.phobosgui, phobos.utils.naming, phobos.utils.blender, phobos.utils.general, phobos.utils.selection, \
        phobos.operators.io, phobos.operators.editing, phobos.operators.misc, phobos.operators.naming, \
        phobos.operators.selection, phobos.logging, phobos.defs

    print("Importing Phobos modules.")

import bpy


def register():
    """This function registers all modules to blender.

    :return: Nothing

    """
    phobos.links.register()
    phobos.controllers.register()
    phobos.exporter.register()
    phobos.phobosgui.register()
    phobos.importer.register()
    phobos.joints.register()
    phobos.sensors.register()
    phobos.inertia.register()
    bpy.utils.register_module(__name__)


def unregister():
    """This function unregisters all modules to blender.

    :return: Nothing

    """
    phobos.links.unregister()
    phobos.controllers.unregister()
    phobos.exporter.unregister()
    phobos.phobosgui.unregister()
    phobos.importer.unregister()
    phobos.joints.unregister()
    phobos.sensors.unregister()
    phobos.inertia.register()
    bpy.utils.unregister_module(__name__)

# if __name__ == "__main__":
#    register()
