># **NOTE: This is the beta version of phobos2.0.0. Please test and report all issues [here](https://github.com/dfki-ric/phobos/issues). Thanks for your contribution!**

[![latest-release](https://img.shields.io/github/tag/dfki-ric/phobos.svg?label=version&style=flat)](https://github.com/dfki-ric/phobos/releases)
[![DOI](https://joss.theoj.org/papers/10.21105/joss.01326/status.svg)](https://doi.org/10.21105/joss.01326)
[![license](https://img.shields.io/github/license/dfki-ric/phobos.svg?style=flat)](https://github.com/dfki-ric/phobos/blob/master/COPYING)
[![made-with-sphinx-doc](https://img.shields.io/badge/Made%20with-Sphinx-1f425f.svg)](https://www.sphinx-doc.org/)

![Phobos](https://github.com/dfki-ric/phobos/wiki/img/phobos_logo_small.png)

Phobos is both a CLI tool and add-on for the open-source 3D modeling software
[Blender v3.3LTS](https://www.blender.org/download/lts/3-3/) to support your robot model creation and editing.

The Blender add-on enables the creation of WYSIWYG robot
models for use in robot frameworks like [ROS](http://wiki.ros.org/) and
[ROCK](https://github.com/rock-core) or in real-time simulations such as
[MARS](https://github.com/rock-simulation/mars) or
[Gazebo](http://gazebosim.org/). Phobos exports formats such as **URDF**,
**SDF** or **SMURF** and common mesh formats (**Stereolithography** (.stl),
**Wavefront** (.obj) or **Collada** (.dae)).

Phobos was initiated and is currently developed at the [Robotics Innovation
Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German
Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in
Bremen, together with the [Robotics
Group](http://www.informatik.uni-bremen.de/robotik/index_en.php) of the
[University of Bremen](http://www.uni-bremen.de/en.html).

Please contact [Henning Wiedemann](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/hewi04.html)
for any inquiries, or any questions and feedback not suited for the issues
page.

## Version 2.0.0
With version 2.0.0 we did a refactoring of Phobos and its now possible to use phobos as a normal python package and command line tool (see below).

When running the new Phobos on a model created with an older version of Phobos, make sure to have a backup.
For most cases you should be able to update your model by simply exporting it to smurf and then importing it again from that smurf file.
Due to the changes between Blender 2 to 3 it might be necessary to check whether your materials already use the Specular BSDF or Principled BSDF shaders, if not you'd have to update this manually.

If you encounter any problems with this new version please do not hesitate to open an issue [here](https://github.com/dfki-ric/phobos/issues).

## Questions or Ideas you want to discuss?
Please have a look at our [GitHub discussions](https://github.com/dfki-ric/phobos/discussions).

## Found a bug or want to request a feature?
Please let us know by opening an issue [here](https://github.com/dfki-ric/phobos/issues).

## Documentation
- User documentation: [Phobos Wiki](https://github.com/dfki-ric/phobos/wiki)
- Source documentation: [Phobos' Github Page](http://dfki-ric.github.io/phobos).

## Citing

Phobos has been published in the [Journal of Open Source Software](https://doi.org/10.21105/joss.01326).
We ask users to cite the use of Phobos, as it allows us to keep the project alive.

When citing, please provide this information:

  - Phobos version you were using (see the [wiki](https://github.com/dfki-ric/phobos/wiki/Installation#versions-and-branching) for information about versions)
  - If you were using additional Phobos plugins or configurations.
  - The general [Phobos paper](https://doi.org/10.21105/joss.01326).

If you are on the hunt for a BiBTeX entry, check out the [FAQ section](https://github.com/dfki-ric/phobos/wiki/FAQ#how-do-i-cite-phobos).

## Installation

### Blender
>UPDATING: If you already have phobos installed and want to update.
> You have to remove the old version of Phobos first and close Blender.
> Then proceed with the installation steps explained below.

>NOTE (WINDOWS): If you are using blender under Windows, make sure you have the latest version of [Microsoft Visual C++ Redistributable](https://learn.microsoft.com/en-US/cpp/windows/latest-supported-vc-redist?view=msvc-170) installed.  Otherwise Blender's python won't work properly.

To install Phobos in blender download the phobos.zip of the release or zip the phobos subdirectory (e.g. `zip -r phobos.zip phobos`)
Phobos has several python dependencies, those have to be installed in blender before you can use phobos.
There are two ways of installing Phobos:

1. Recommended:
  a. Directly install the phobos.zip in blender: `Blender->Edit->Preferences->Addons->Install` and activate it.
  b. Restart Blender.
  c. Activate Phobos Add-on again.
2. (Offers inspection of the requirements before installing them) Before installing the phobos.zip you can run the script install_requirements.py with blender's python.
  a. ```bash
     ${BLENDER_EXECUTABLE} -b --python install_requirements.py
     ```
  b. Install the phobos.zip in blender: `Blender->Edit->Preferences->Addons->Install` and activate it.

After installation the phobos main menu can be found on the right hand side of the 3D Viewport.
If not already visible, one can find a very small arrow to open the Blender toolbar (purple circle showing it in the image).

![Small arrow to open the phobos toolbar widget.](https://github.com/dfki-ric/phobos/wiki/img/blender_phobos_menu_open.png)

Phobos is currently tested and running with Blender v3.3 LTS.

### CLI
Install the requirements by executing `install_requirements.py` with the python you want to install phobos to:
```bash
cd phobos
python3 install_requirements.py
```

Then just install it using pip:
```bash
cd phobos
pip install .
```
or with autoproj:
1) Add the package to your buildconf/package_set
2) Install via `amake`

## Overview

### Blender

![Model of the SpaceClimber robot in Blender, next to the Phobos toolbar
displayed on the
left.](https://github.com/dfki-ric/phobos/wiki/img/phobos_spaceclimber.png)

*Model of the
[SpaceClimber](http://robotik.dfki-bremen.de/en/research/projects/spaceclimber-1.html)
robot in Blender, next to the Phobos toolbar displayed on the left.*

Phobos makes use of Blender's hierarchical object graph and its bone objects.
These objects, normally used for animating 3D characters, allow to store 3D
coordinate systems and apply constraints to their movements, for instance to
restrict the movement of an object to a certain range on a specific axis. This
allows to replicate the links and joints defined in a **URDF** model and together
with the hierarchical tree of parent and child objects, the complete, branching
kinematic chain of a robot can be represented. By attaching meshes or
primitives to the bones, Phobos allows to add visual and collision objects to
a model. Additional objects allow storing further information, e.g. centers of
mass of each part of a robot, thus refining the physical representation. Sensor
objects can be added to correctly place and orient devices such as laser
scanners, cameras or contact sensors. Making use of Blender's custom object
properties, any necessary information can be added to the model, from inertia
tensors to opening angles of cameras.

![Decomposition of the different elements from which Phobos models are composed
in Blender.](https://github.com/dfki-ric/phobos/wiki/img/phobos_elements.png)

*Decomposition of the different elements from which Phobos models are composed
in Blender. These elements can be arranged in Blender on different layers, thus
avoiding confusion or obstruction of view when editing very complex models.*

### CLI

You can either use CLI-phobos as a normal python package or use the scripts provided with it.

For the latter do `phobos --help` to get a list of the currently available scripts.
It will also tell you which dependencies are missing for some scripts.

## Features

- WYSIWYG editor for robot models using Blender
- CLI tools for fast and easy model handling and inspection
- CI tool to run phobos headless in your CI-pipeline for atomated model processing and maintenance
- Import and export of **URDF**, **SDF** **SMURF** and other
  [formats](https://github.com/dfki-ric/phobos/wiki/Formats)
- Easy definition of robot kinematics (links and joints)
- Visualisation of different model components, even joint constraints
- Numerous tools for fast editing:
  - Batch editing of object properties
  - Auto-generation of collision objects
  - Auto-generation of inertia tensors from mass and shape
  - Calculation of merged inertia for complex links
  - Verbose logging
- Saving and loading of model poses
- Annotation of objects from motors/sensors to joints/links
- Save/load different export configurations for the same model
- Export with defined floating point precision
- Model integrity checks
- Tools for maintaining your own model database
- Library containing Python examples for automatic model adaption
- All the cool features Blender already provides (rendering, animation, etc.)

## License

Phobos is distributed under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).
