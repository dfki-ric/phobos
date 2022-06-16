#!/usr/bin/python

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------
import json
import setuptools
import subprocess


def get_git_revision_hash():
    return subprocess.check_output(['git', 'rev-parse', 'HEAD']).strip().decode("utf-8")


def get_git_revision_short_hash():
    return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).strip().decode("utf-8")


def get_git_branch():
    return subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).strip().decode("utf-8")


with open("README.md", "r") as fh:
    long_description = fh.read()

print("Attention: You are currently installing phobos for CLI usage only. If you want to use phobos the phobos blender "
      "add-on follow the following steps:\n"
      "1) Zip the phobos directory in this repository\n"
      "2) Open blender\n"
      "3) Go to preferences->Add-ons\n"
      "4) Click 'Install' and select the freshly zipped phobos.zip file\n"
      "   Blender will now install the phobos add-on with all its dependencies.\n"
      "5) Click the checkbox to activate the phobos-add-on")

codemeta = json.load(open("codemeta.json", "r"))

setuptools.setup(
    name=codemeta["title"].lower(),  # Replace with your own username
    version=codemeta["version"],
    author=", ".join(codemeta["author"]),
    author_email=codemeta["maintainer"],
    description=codemeta["description"] + " Revision:" + get_git_branch() + "-"
                + get_git_revision_short_hash(),
    long_description=long_description,
    long_description_content_type="text/markdown",
    url=codemeta["codeRepository"],
    packages=setuptools.find_packages(),
    include_package_data=True,
    package_data={'': [
        'data/floatingbase.urdf', 'data/manifest.xml.in', 'data/ROSCMakeLists.txt.in',
        'data/ROSpackage.xml.in', 'data/README.md.in', 'data/GitLFS_README.md.in',
        'data/assembly_config.yml.in', 'data/xml_formats.json'
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.8",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    # This currently leads to issues with autoproj
    # install_requires=[
    #     "numpy",
    #     "scipy",
    #     "pyyaml",
    #     "trimesh",
    #     "networkx",
    #     "open3d",
    #     "python-fcl",
    #     "pybullet",
    # ],
    entry_points={
        'console_scripts': [
            'phobos=phobos.scripts.phobos:main',
        ]
    },
)
