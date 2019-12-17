---
title: 'Phobos: A tool for creating complex robot models'
tags:
  - robotics
  - Blender
  - Python
  - URDF
  - SDF
  - model
authors:
  - name: Kai von Szadkowski
    orcid: 0000-0002-1814-8463
    affiliation: 1
  - name: Simon Reichel
    orcid: 0000-0002-0190-9780
    affiliation: 1
affiliations:
 - name: German Research Center for Artificial Intelligence GmbH - Robotics Innovation Center, Bremen
   index: 1
date: 01 February 2019
bibliography: paper.bib
---

# Summary

The rise of task specific requirements for robots leads to an increasing
complexity of the affiliated system. To ensure a seamless, simulation based
development of both hard- and software a virtual twin is necessary, which is
able to reflect different aspects of the mechatronic system. Most description
formats for these systems are based on [**XML** ](https://www.xml.com/)
(e**X**tensible **M**arkup **L**anguage), which encodes the necessary data for
simulation, e.g. mechanical structure, inertial data, visual and collision
representations. However, the initial modeling or continuous refinement of the
model based on design iterations, sensor and actor integration can be a time
consuming, challenging task for the modeler. Also, tiny mistakes in the model
description can lead to huge differences in the modeled system. With increasing
model complexity the model files become even more difficult to debug.

![Universal Robots UR5 (https://www.universal-robots.com/products/ur5-robot/) within ``Phobos``](PhobosUR5.png)

``Phobos`` is an add-on for the open-source 3D modeling software
[Blender](https://www.blender.org/) that allows users to edit models of robots
and their environments for robotics research and applications.

It integrates seamlessly in Blender's GUI and adds functionality for creating
accurate representations of robotic systems with clearly defined kinematic
chains, actuators and sensors in a WYSIWYG (**W**hat **Y**ou **S**ee **I**s
**W**hat **Y**ou **G**et) environment.

To simplify common workflows further, all components of ``Phobos`` models in
Blender can be annotated with custom data fields both individually or in
groups. Such data can become part of exported model formats that support them
(e.g. to define new types of sensors and other hardware devices) or kept for
internal use, enabling the user to manipulate multiple components of a model
using custom [Python](https://www.python.org/) scripts. As ``Phobos`` is built
on Blender's internal Python scripting
[API](https://docs.blender.org/api/current/), it is easy to create scripts that
work with both Blender- and ``Phobos``-specific data.

Current features of ``Phobos`` include:

+ support for a number of common description formats for robotic devices, such
  as:
  - [URDF](http://wiki.ros.org/urdf) (**U**nified **R**obot **D**escription
    **F**ormat)
  - [SDF](http://sdformat.org) (**S**imulation **D**escription **F**ormat)
  - [SMURF](https://github.com/dfki-ric/phobos/wiki/smurf) (**S**upplementable
    **M**ostly **U**niversal **R**obot **F**ormat).
- visual representation of many different components, such as model links and
  joints, inertias, motors etc.
- hierarchical overview over the model
- useful batch tools, like renaming, reorientation of the model tree etc.
- automated mass and inertia calculation
- saving and loading of different model poses
- tools for maintaining your own model database
- creation of submodels which can be incorporated into other models
+ definition of export configurations, where the high fidelity model is either
  used as a template or can be simplified
+ internal checking of associated data for integrity
+ an additional library containing Python examples for automatic model adaption

Furthermore, the user can tap into  Blenders capabilities to visualize joint
limits via the Pose Editing Mode, simplify associated visual and collision
meshes or write Python scripts to modifiy the model objects. Since import and
export are plugin-based, adding another format is as simple as adding a Python
script to the respective subfolder. Phobos uses a Python dictionary in which
all model information is stored for I/O purposes, thus new developers do not
have to program against Blender's representations of model elements.

Phobos was developed at the German Research Center for Artificial
Intelligence's Robotics Innovation Center
([DFKI-RIC](https://robotik.dfki-bremen.de/en/startpage.html) in Bremen in
collaboration with the University of Bremen's [Robotics
Group](https://www.uni-bremen.de/en/fachbereich-3/robo/). The goal behind its
development is to simplify the task of creating accurate descriptions of
robotic systems in an intuitive and less error-prone way than editing them by
hand in a text editor, and to provide an open-source alternative to exporting
such models from proprietary CAD systems.

It has been used recently to provide the models for seriell-parallel hybrid
robots modelling [@hyrodyn].

Phobos can be found at Github [@phobosrepo].

# Acknowledgements

We acknowledge contributions from Stefan Rahms, Ole Schwiegert und Malte Langosz.

``Phobos`` was carried out in the projects:

+ LIMES, a collaboration between the DFKI Robotics Inno- vation Center and the
  University of Bremen, funded by the German Space Agency (DLR, Grant numbers:
  50RA1218, 50RA1219) with federal funds of the Federal Ministry of Economics
  and Technology (BMWi) in accordance with the parliamentary resolution of the
  German Parliament.
+ D-RoCK, funded by the German Aerospace Center (DLR) with federal funds from
  the Federal Ministry of Education and Research (BMBF) (Grant Numbers:
  01IW15001)
+ TransFit, which is funded by the German Federal Ministry of Economics and
  Technology (BMWi) according to a resolution of the German Bundestag, grant
  no. 50RA1701, 50RA1702 and 50RA1703.

# References
