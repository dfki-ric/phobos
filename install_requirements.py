#!/usr/bin/python3

import importlib
import subprocess
import sys

REQUIREMENTS_HAVE_BEEN_CHECKED = False
BPY_AVAILABLE = False
try:
    import bpy
    BPY_AVAILABLE = True
except ImportError:
    pass


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
    "python-fcl": "python-fcl",  # optional for blender,
    "PIL": "Pillow"  # optional for blender,
}


def install_requirement(package_name, upgrade_pip=False, lib=None, ensure_pip=True):
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
                if import_name in list(optional_requirements.keys()) + list(extra_requirements.keys()):
                    print(f"Couldn't install optional requirement {import_name} ({req_name})")
                else:
                    raise e
    importlib.invalidate_caches()
    REQUIREMENTS_HAVE_BEEN_CHECKED = True


if __name__ == "__main__":
    print("---\nYou are executing this file with:", sys.executable)
    extra = False
    if not BPY_AVAILABLE:
        print("To install the requirements for the installation of the Phobos Blender-Add-on run this file with Blender's python:")
        print("${YOUR_BLENDER_EXECUTABLE} -b --python install_requirements.py")
        extra = True
    print("This script will try to install and update pip and the following python packages.")
    print("  Required:", list(requirements.values()))
    if extra:
        print("  Optional:", list(optional_requirements.values()) + list(extra_requirements.values()))
    else:
        print("  Optional:", list(optional_requirements.values()))
    proceed = None
    while proceed not in ["y", "n"]:
        proceed = input("Do you want to proceed? [y/n]>")
    if proceed == "y":
        check_requirements(optional=True, upgrade_pip=True, extra=extra)
    else:
        print("Aborted!")

