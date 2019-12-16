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
Handles different import attempts to cope with Blender's *Reload script* functionality.

Contains also some adjustments to make YAML imports deal with booleans appropriately.
"""

import sys
import os.path
import importlib
import pkgutil

# TODO double import of basemodule?
import bpy
import phobos


def import_submodules(package, recursive=True, verbose=False):
    """Import all submodules of a module, recursively, including subpackages.
        If a module is already imported it is reloaded instead.
        Recursion can be turned off.
        The imported modules are returned as dictionary.

    Args:
      package(str | module): package (name or actual module)
      recursive(bool, optional): recursion active (Default value = True)
      verbose(bool, optional): import feedback active (Default value = False)

    Returns:

    """
    modules = sys.modules

    # when using string import initial module first
    if isinstance(package, str):
        package = importlib.import_module(package)

    results = {}
    # iterate over all modules in package path
    for loader, name, is_pkg in pkgutil.walk_packages(package.__path__):
        full_name = package.__name__ + '.' + name

        # reload already imported modules
        if full_name in modules.keys():
            if verbose:
                print("RELOAD: ", full_name)
            results[full_name] = importlib.reload(modules[full_name])
        # otherwise import them
        else:
            if verbose:
                print("IMPORT: ", full_name)
            results[full_name] = importlib.import_module(full_name)

        # recursion on submodules
        if recursive and is_pkg:
            results.update(import_submodules(full_name))
    return results


bl_info = {
    "name": "Phobos",
    "description": "A toolbox to enable editing of robot models in Blender.",
    "author": "Kai von Szadkowski, Ole Schwiegert, Stefan Rahms, Malte Langosz, Simon Reichel",
    "version": (1, 0, 1),
    "blender": (2, 79, 0),
    "location": "Phobos adds a number of custom tool panels.",
    "warning": "",
    "wiki_url": "https://github.com/dfki-ric/phobos/wiki",
    "support": "COMMUNITY",
    "tracker_url": "https://github.com/dfki-ric/phobos/issues",
    "category": "Development",
}

# TODO rework yaml import: loading module twice if yaml is not found...
yamlconfpath = os.path.dirname(__file__) + "/python_dist_packages.conf"
if os.path.isfile(yamlconfpath):
    f = open(yamlconfpath)
    distpath = f.read().replace('\n', '')
    f.close()
    sys.path.insert(0, os.path.normpath(distpath))
    import yaml

    # OPT here we could add additional required imports
# stop execution, when yaml cannot be imported
else:
    raise FileNotFoundError('No python_dist_packages.conf file found. Please reinstall phobos.')


# Add custom YAML (de-)serializer
def bool_representer(dumper, data):
    """

    Args:
      dumper: 
      data: 

    Returns:

    """
    if data == '$true':
        return dumper.represent_bool(True)
    elif data == '$false':
        return dumper.represent_bool(False)
    else:
        return dumper.represent_str(str(data))


yaml.add_representer(str, bool_representer)


def bool_constructor(self, node):
    """

    Args:
      node: 

    Returns:

    """
    value = self.construct_yaml_bool(node)
    return '$true' if value else '$false'


yaml.Loader.add_constructor(u'tag:yaml.org,2002:bool', bool_constructor)
yaml.SafeLoader.add_constructor(u'tag:yaml.org,2002:bool', bool_constructor)

# Recursively import all submodules
print("Importing phobos")
import_submodules(phobos, verbose=True)


def register():
    """This function registers all modules to blender.
    
    :return: Nothing

    Args:

    Returns:

    """
    phobos.phobosgui.register()
    bpy.utils.register_module(__name__)


def unregister():
    """This function unregisters all modules in Blender."""
    print("\n" + "-" * 100)
    print("Unregistering Phobos...")
    # TODO delete all imported modules to resolve reregistration conflicts
    phobos.phobosgui.unregister()
    bpy.utils.unregister_module(__name__)
