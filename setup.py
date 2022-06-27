#!/usr/bin/python

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------
import json
import os
import sys
import shutil
import setuptools
import subprocess
from pathlib import Path
from copy import deepcopy


# utilities
def get_git_revision_hash():
    return subprocess.check_output(['git', 'rev-parse', 'HEAD']).strip().decode("utf-8")


def get_git_revision_short_hash():
    return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).strip().decode("utf-8")


def get_git_branch():
    return subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).strip().decode("utf-8")


def check_blender_agreement(answer):
    if not answer.lower() in ["y", "yes"]:
        print("Blender installation cancelled!")
        print("If you want to install the phobos blender "
              "add-on later or manually follow the following steps:\n"
              "1) Zip the phobos directory in this repository\n"
              "2) Open blender\n"
              "3) Go to preferences->Add-ons\n"
              "4) Click 'Install' and select the freshly zipped phobos.zip file\n"
              "   Blender will now install the phobos add-on with all its dependencies.\n"
              "5) Click the checkbox to activate the phobos-add-on")
        sys.exit()
    return True


def exec_shell_cmd(command):
    if sys.platform.startswith("linux"):
        command = [" ".join(str(x) for x in command)]
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    (out, err) = proc.communicate()
    return out.decode().strip()


def install_requirement(py_exec, package_name, upgrade_pip=False, lib=None):
    # Ensure pip is installed
    exec_shell_cmd([py_exec, "-m", "ensurepip", "--user"])
    # Update pip (not mandatory)
    if upgrade_pip:
        print("  Upgrading pip...")
        exec_shell_cmd([py_exec, "-m", "pip", "install", "--upgrade", "pip"])
    # Install package
    print("  Installing package", package_name, flush=True)
    if lib is None:
        exec_shell_cmd([py_exec, "-m", "pip", "install", package_name])
    else:
        exec_shell_cmd([py_exec, "-m", "pip",  "install", f"--target={str(lib)}", package_name])


def check_requirements(py_exec, optional=False, upgrade_pip=False, lib=None):
    print("Checking requirements:")
    import importlib
    reqs = [requirements]
    if optional:
        reqs += [optional_requirements]
    if upgrade_pip:
        print("  Upgrading pip...")
        exec_shell_cmd([py_exec, "-m", "pip", "install", "--upgrade", "pip"])
    for r in reqs:
        for import_name, req_name in r.items():
            print("  Checking", import_name, flush=True)
            install_requirement(py_exec, req_name, upgrade_pip=False, lib=lib)
            # try:
            #     if importlib.util.find_spec(import_name) is None:
            #         install_requirement(py_exec, req_name, upgrade_pip=False, lib=lib)
            # except AttributeError:
            #     loader = importlib.find_loader(import_name)
            #     if not issubclass(type(loader), importlib.machinery.SourceFileLoader):
            #         install_requirement(py_exec, req_name, upgrade_pip=False, lib=lib)
    importlib.invalidate_caches()


def fetch_path_from_blender_script(blender_exec, script, keyword="blender"):
    out = exec_shell_cmd([blender_exec, "--background", "--python", script])
    path = None
    for line in out.split("\n"):
        line = line.strip()
        if line == "":
            continue
        if line.endswith("\r"):
            line = line.replace("\r", "")
        if line.startswith("/") and keyword in line.lower():
            path = Path(line)
            break
    return path


# data
with open("README.md", "r") as fh:
    long_description = fh.read()
codemeta = json.load(open("codemeta.json", "r"))
deps = json.load(open("blender_requirements.json", "r"))
requirements = deps["requirements"]
optional_requirements = deps["optional_requirements"]
this_dir = Path(__file__).parent

##################
# python package #
##################
kwargs = {
    "name": codemeta["title"].lower(),  # Replace with your own username
    "version": codemeta["version"],
    "author":",  ".join(codemeta["author"]),
    "author_email": codemeta["maintainer"],
    "description": codemeta["description"] + " Revision:" + get_git_branch() + "-"
                + get_git_revision_short_hash(),
    "long_description": long_description,
    "long_description_content_type":" text/markdown",
    "url": codemeta["codeRepository"],
    "packages": setuptools.find_packages(),
    "include_package_data": True,
    "package_data":{'':  [os.path.join("data", x) for x in os.listdir(this_dir/"phobos"/"data")],},
    "classifiers": [
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.10",
        "Operating System :: OS Independent",
    ],
    "python_requires": '>= 3.6',
    "entry_points": {
        'console_scripts': [
            'phobos=phobos.scripts.phobos:main',
        ]
    }
}
# # autoproj handling
# if not "AUTOPROJ_CURRENT_ROOT" in os.environ:
#     kwargs["install_requires"] = list(requirements.values()) #+ list(optional_requirements.keys())

setuptools.setup(
    **kwargs
)

#################
# blender addon #
#################
# autoproj installation can not handle user inputs
if "AUTOPROJ_CURRENT_ROOT" in os.environ:
    sys.exit()

blender_path = None
print("Thanks for installing phobos!\n\nTrying now to install blender addon")
check_blender_agreement(input("Do you want to proceed? (y/n)>").strip())
auto_found = False
for cmd in ["which", "where", "Get-Command"]:
    out = Path(exec_shell_cmd([cmd, "blender"]))
    if out.exists() and "blender" in str(out).lower():
        if input(f"Take this blender version '{out}'? (y/n)>").lower() in ["y", "yes"]:
            auto_found = True
        break
while not auto_found or not out.exists() or not "blender" in str(out).lower():
    auto_found = True
    print("Could not retrieve blender executable from this path:", out)
    out = Path(input("Please provide the path to your blender executable: ").strip())
blender_path = deepcopy(out)
scripts_path = fetch_path_from_blender_script(blender_path, this_dir/"get_blender_path.py")
modules_path = scripts_path / "modules"
addons_path = scripts_path / "addons"
print("Found the following blender installation directories:")
print(scripts_path)
print(modules_path)
print(addons_path)
if check_blender_agreement(input("Do you want to proceed? (y/n)>").strip()):
    #install phobos
    target = addons_path/"phobos"
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(this_dir/"phobos", target)
    # install requirements
    python_path = fetch_path_from_blender_script(blender_path, this_dir/"get_blender_python.py")
    check_requirements(python_path, upgrade_pip=True, lib=modules_path)
