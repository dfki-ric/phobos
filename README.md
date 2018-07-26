![Phobos](https://github.com/dfki-ric/phobos/wiki/img/phobos_logo_small.png)

Phobos is an add-on for the open-source 3D modeling software [Blender](http://www.blender.org) that enables the creation of WYSIWYG robot models for use in robot frameworks like ROS and ROCK or in real-time simulations such as MARS or Gazebo. Phobos exports formats such as URDF or SMURF and common mesh formats(STereoLithography (.stl), Wavefront (.obj) or Collada (.dae)).

Phobos was initiated and is currently developed at the [Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen, together with the [Robotics Group](http://www.informatik.uni-bremen.de/robotik/index_en.php) of the [University of Bremen](http://www.uni-bremen.de/en.html).

Please contact [Kai von Szadkowski](http://robotik.dfki-bremen.de/en/about-us/staff/kavo01.html) for any inquiries, or any questions and feedback not suited for the issues page.

## Documentation

- User documentation: [Phobos Wiki](https://github.com/dfki-ric/phobos/wiki)
- Source documentation: [Phobos' Github Page](http://dfki-ric.github.io/phobos).

## Installation

Please refer to the Wiki's [installation page](https://github.com/dfki-ric/phobos/wiki/Installation).

## Overview

![Model of the SpaceClimber robot in Blender, next to the Phobos toolbar displayed on the left.](https://github.com/dfki-ric/phobos/wiki/img/phobos_spaceclimber.png)

*Model of the [SpaceClimber](http://robotik.dfki-bremen.de/en/research/projects/spaceclimber-1.html) robot in Blender, next to the Phobos toolbar displayed on the left.*

Phobos makes use of Blender's hierarchical object graph and its bone objects. These objects, normally used for animating 3D characters, allow to store 3D coordinate systems and apply constraints to their movements, for instance to restrict the movement of an object to a certain range on a specific axis. This allows to replicate the links and joints defined in a URDF model and together with the hierarchical tree of parent and child objects, the complete, branching kinematic chain of a robot can be represented.
By attaching meshes or primitives to the bones, Phobos allows to add visual and collision objects to a model. Additional objects allow storing further information, e.g. centers of mass of each part of a robot, thus refining the physical representation. Sensor objects can be added to correctly place and orient devices such as laser scanners, cameras or contact sensors. Making use of Blender's custom object properties, any necessary information can be added to the model, from inertia tensors to opening angles of cameras.

![Decomposition of the different elements from which Phobos models are composed in Blender.](https://github.com/dfki-ric/phobos/wiki/img/phobos_elements.png)

*Decomposition of the different elements from which Phobos models are composed in Blender. These elements can be arranged in Blender on different layers, thus avoiding confusion or obstruction of view when editing very complex models.*


## Features

- WYSIWYG editor for robot models using Blender
- Definition of robot kinematics (links and joints)
- Annotation of motors/sensors to joints/links
- Visualization of joint constraints
- Numerous tools for fast editing:
  - Batch editing of object properties
  - Auto-generation of collision objects
  - Auto-generation of inertia tensors from mass and shape
  - Calculation of merged inertia for complex links
  - Verbose logging
- Import and export of URDF, SMURF and other formats
- Export with defined floating point precision

## License

Phobos is distributed under the [GNU Lesser General Public License](https://www.gnu.org/licenses/lgpl.html).
