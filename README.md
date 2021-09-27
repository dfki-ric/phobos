[![latest-release](https://img.shields.io/github/tag/dfki-ric/phobos.svg?label=version&style=flat)](https://github.com/dfki-ric/phobos/releases)
[![DOI](https://joss.theoj.org/papers/10.21105/joss.01326/status.svg)](https://doi.org/10.21105/joss.01326)
[![Code style:
black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/ambv/black)
[![license](https://img.shields.io/github/license/dfki-ric/phobos.svg?style=flat)](https://github.com/dfki-ric/phobos/blob/master/COPYING)
[![made-with-python](https://img.shields.io/badge/Made%20with-Python-1f425f.svg)](https://www.python.org/)
[![made-with-sphinx-doc](https://img.shields.io/badge/Made%20with-Sphinx-1f425f.svg)](https://www.sphinx-doc.org/)
[![Build Status](https://travis-ci.org/dfki-ric/phobos.svg?branch=master)](https://travis-ci.org/dfki-ric/phobos)

![Phobos](https://github.com/dfki-ric/phobos/wiki/img/phobos_logo_small.png)

Phobos is an add-on for the open-source 3D modeling software
[Blender](http://www.blender.org) that enables the creation of WYSIWYG robot
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

Please contact [Kai von Szadkowski](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/kavo01.html)
for any inquiries, or any questions and feedback not suited for the issues
page.

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

Just zip the `phobos` subfolder: `zip -r phobos.zip phobos` and install it via Blender: `blender->edit->preferences->addons->install`

## Overview

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


## Features

- WYSIWYG editor for robot models using Blender
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
+ Library containing Python examples for automatic model adaption
- All the cool features Blender already provides (rendering, animation, etc.)

## License

Phobos is distributed under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).
