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
import sys

REQUIREMENTS_HAVE_BEEN_CHECKED = False
BPY_AVAILABLE = False
try:
    import bpy
    BPY_AVAILABLE = True
except ImportError:
    pass

bl_info = {
    "name": "Phobos",
    "description": "A toolbox to enable editing of robot models in Blender.",
    "author": "Kai von Szadkowski, Henning Wiedemann, Malte Langosz, Simon Reichel, Julius Martensen, et. al.",
    "version": (2, 0, 0),
    "blender": (3, 3, 1),
    "location": "Phobos adds a number of custom tool panels.",
    "warning": "",
    "wiki_url": "https://github.com/dfki-ric/phobos/wiki",
    "support": "COMMUNITY",
    "tracker_url": "https://github.com/dfki-ric/phobos/issues",
    "category": "Development",
}

# [TODO v2.1.0] fill this automatically
requirements = {
    "yaml": "pyyaml",
    "numpy": "numpy",
    "scipy": "scipy",
    "pkg_resources": "setuptools",
    "collada": "pycollada",
    "pydot": "pydot"
}

optional_requirements = {
    "lxml": "lxml",
    "networkx": "networkx",  # optional for blender
    "trimesh": "trimesh",  # optional for blender
}

extra_requirements = {
    "pybullet": "pybullet",  # optional for blender
    "open3d": "open3d",  # optional for blender
    "python-fcl": "python-fcl"  # optional for blender
}


def install_requirement(package_name, upgrade_pip=False, lib=None, ensure_pip=True):
    import subprocess
    import sys
    if lib is None and BPY_AVAILABLE:
        lib = bpy.utils.user_resource("SCRIPTS", path="modules")
    if ensure_pip:
        # Ensure pip is installed
        subprocess.check_call([sys.executable, "-m", "ensurepip", "--user"])
    # Update pip (not mandatory)
    if upgrade_pip:
        print("  Upgrading pip...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])
    # Install package
    print("  Installing package", package_name)
    if lib is None:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", package_name])
    else:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", f"--target={str(lib)}", package_name])


def check_requirements(optional=False, extra=False, force=False, upgrade_pip=False, lib=None):
    global REQUIREMENTS_HAVE_BEEN_CHECKED
    if REQUIREMENTS_HAVE_BEEN_CHECKED and not force:
        return
    print("Checking requirements:")
    import importlib
    import subprocess
    import sys
    # Ensure pip is installed
    subprocess.check_call([sys.executable, "-m", "ensurepip", "--user"])
    reqs = [requirements]
    if optional:
        reqs += [optional_requirements]
    if extra:
        reqs += [extra_requirements]
    if upgrade_pip:
        print("  Upgrading pip...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])
    for r in reqs:
        for import_name, req_name in r.items():
            print("  Checking", import_name)
            try:
                if importlib.util.find_spec(import_name) is None:
                    install_requirement(req_name, upgrade_pip=False, lib=lib, ensure_pip=False)
            except AttributeError:  # when using importlib before v3.4
                loader = importlib.find_loader(import_name)
                if not issubclass(type(loader), importlib.machinery.SourceFileLoader):
                    install_requirement(req_name, upgrade_pip=False, lib=lib, ensure_pip=False)
            except subprocess.CalledProcessError as e:
                if import_name in optional_requirements.keys():
                    print(f"Couldn't install optional_requirement {import_name} ({req_name})")
                else:
                    raise e
    importlib.invalidate_caches()
    REQUIREMENTS_HAVE_BEEN_CHECKED = True


def register():
    """This function registers all modules to blender.

    :return: Nothing

    Args:

    Returns:

    """
    import sys
    if not sys.platform.startswith("win"):
        check_requirements(optional=True, upgrade_pip=True)
    from . import defs
    from . import io
    from . import core
    from . import geometry
    from . import scenes
    from . import utils
    from . import ci
    # from . import scripts

    # Recursively import all submodules
    from . import blender

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
    # [TODO v2.1.0] delete all imported modules to resolve reregistration conflicts
    from . import blender
    blender.phobosgui.unregister()


if not "blender" in sys.executable.lower() and not BPY_AVAILABLE:
    from pkg_resources import get_distribution, DistributionNotFound

    try:
        # Change here if project is renamed and does not equal the package name
        dist_name = __name__
        __version__ = get_distribution(dist_name).version
    except DistributionNotFound:
        __version__ = ".".join([str(x) for x in bl_info["version"]])
    finally:
        del get_distribution, DistributionNotFound
else:
    if not sys.platform.startswith("win"):
        check_requirements(optional=True, upgrade_pip=True)

from . import defs
from . import io
from . import core
from . import geometry
from . import scenes
from . import utils
from . import ci
from . import scripts

del sys
