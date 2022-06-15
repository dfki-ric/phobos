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
"""

bl_info = {
    "name": "Phobos",
    "description": "A toolbox to enable editing of robot models in Blender.",
    "author": "Kai von Szadkowski, Malte Langosz, Henning Wiedemann, Julius Martensen, Ole Schwiegert, Stefan Rahms, Simon Reichel",
    "version": (2, 0, 0),
    "blender": (3, 2, 0),
    "location": "Phobos adds a number of custom tool panels.",
    "warning": "",
    "wiki_url": "https://github.com/dfki-ric/phobos/wiki",
    "support": "COMMUNITY",
    "tracker_url": "https://github.com/dfki-ric/phobos/issues",
    "category": "Development",
}

requirements = {
        "yaml": "pyyaml",
        "networkx": "networkx",  # optional for blender
        "numpy": "numpy",
        # "pybullet": "pybullet",  # optional for blender
        # "open3d": "open3d",  # optional for blender
        "scipy": "scipy",
        "trimesh": "trimesh",  # optional for blender
        #"python-fcl": "python-fcl",  # optional for blender
        "pkg_resources": "setuptools"
    }


def install_requirement(package_name):
    import sys
    import os

    py_exec = str(sys.executable)
    # Get lib directory
    lib = None
    for path in sys.path:
        if "modules" in path and ("Roaming" in path or ".config" in path or "Users" in path):
            lib = path
            break
    # Ensure pip is installed
    os.system(" ".join([py_exec, "-m", "ensurepip", "--user"]))
    # Update pip (not mandatory)
    os.system(" ".join([py_exec, "-m", "pip", "install", "--upgrade", "pip"]))
    # Install package
    os.system(" ".join([py_exec, "-m", "pip", "install", f"--target={str(lib)}", package_name]))
    print("Installing required package", package_name, "to", lib, flush=True)


def check_requirements():
    import importlib
    for import_name, req_name in requirements.items():
        print("Checking", import_name, flush=True)
        if importlib.util.find_spec(import_name) is None:
            install_requirement(req_name)
    importlib.invalidate_caches()


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
        import sys
        import pkgutil
        import importlib

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


def register():
    """This function registers all modules to blender.

    :return: Nothing

    Args:

    Returns:

    """
    check_requirements()
    from . import defs
    from . import utils
    from . import ci
    from . import geometry
    from . import smurf
    from . import scripts
    from . import core
    from . import io
    from . import scenes

    # Recursively import all submodules
    from . import blender
    print("Importing phobos")
    import_submodules(blender, verbose=True)

    # bpy.utils.register_module(__name__)
    blender.operators.selection.register()
    blender.operators.io.register()
    blender.operators.editing.register()
    blender.operators.generic.register()
    blender.operators.naming.register()
    blender.operators.poses.register()
    blender.phobosgui.register()


def unregister():
    """This function unregisters all modules in Blender."""
    print("\n" + "-" * 100)
    print("Unregistering Phobos...")
    # TODO delete all imported modules to resolve reregistration conflicts
    from . import blender
    blender.phobosgui.unregister()


try:
    import bpy
except ImportError:
    from pkg_resources import get_distribution, DistributionNotFound

    try:
        # Change here if project is renamed and does not equal the package name
        dist_name = __name__
        __version__ = get_distribution(dist_name).version
    except DistributionNotFound:
        __version__ = ".".join([str(x) for x in bl_info["version"]])
    finally:
        del get_distribution, DistributionNotFound

    print("Future import in pure python scripts.")

from . import defs
from . import utils
from . import ci
from . import geometry
from . import smurf
from . import scripts
from . import core
from . import io
from . import scenes

