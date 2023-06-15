#!/usr/bin/python3

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------
import json
import os
import setuptools
import subprocess
from pathlib import Path


# utilities
def get_git_revision_hash():
    return subprocess.check_output(['git', 'rev-parse', 'HEAD']).strip().decode("utf-8")


def get_git_revision_short_hash():
    return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).strip().decode("utf-8")


def get_git_branch():
    return subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).strip().decode("utf-8")


def main(args):
    # data
    with open("README.md", "r") as fh:
        long_description = fh.read()
    codemeta = json.load(open("codemeta.json", "r"))
    requirements = {
        "yaml": "pyyaml",
        "networkx": "networkx",  # optional for blender
        "numpy": "numpy",
        "scipy": "scipy",
        "trimesh": "trimesh",  # optional for blender
        "pkg_resources": "setuptools",
        "collada": "pycollada",
        "pydot": "pydot"
    }
    optional_requirements = {
        "yaml": "pyyaml",
        "pybullet": "pybullet",  # optional for blender
        "open3d": "open3d",  # optional for blender
        "python-fcl": "python-fcl",  # optional for blender
    }
    this_dir = Path(__file__).parent

    ##################
    # python package #
    ##################
    kwargs = {
        "name": codemeta["title"].lower(),  # Replace with your own username
        "version": codemeta["version"],
        "author": ",  ".join(codemeta["author"]),
        "author_email": codemeta["maintainer"],
        "description": codemeta["description"] + " Revision:" + get_git_branch() + "-" + get_git_revision_short_hash(),
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
    # autoproj handling
    if not "AUTOPROJ_CURRENT_ROOT" in os.environ:
        kwargs["install_requires"] = list(requirements.values())
        kwargs["extras_require"] = {'console_scripts': list(optional_requirements.keys())}

    setuptools.setup(
        **kwargs
    )


if __name__ == "__main__":
    import sys
    main(sys.argv)