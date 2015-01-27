Phobos Blender Add-On {#phobos}
===================

Phobos is an open-source Add-On for Blender which simplifies creating and editing robot models and environments to be used in the MARS simulation. Specifically, Phobos allows the user to create URDF files as well as SMURF robot descriptions in a visual, interactive user interface, providing a number of import and export options.

Check out the Phobos [Roadmap](roadmap.md) if you're interested in its further development.

As of the current version of Phobos (0.5), this documentation is still far from extensive. There are probably a lot of difficulties for Blender beginners that we have overlooked and there are certainly features in Phobos that we have not covered at all or at least not comprehensively enough. We therefore appreciate if you contact as about issues you have with Phobos. Either we have not explained it well enough or you have stumbled upon a bug - both cases are something we would like to know.

## Contents

### Documentation 

- [Overview](#overview)
- [Installation](installation.md)
- [Blender](blender.md)
- [Structure of robot models in Phobos](robotmodels.md)
- [Editing robot models](editmodels.md)
- [Import and Export](importexport.md)

#### Operators

- [Overview of operators](operators.md)

### Tutorials

- [Basic modelling](tutorial_basicmodelling.md)
- [Editing masses and inertia](tutorial_massandinertia.md)

## Conventions

Throughout this documentation, when referring to working with Phobos in Blender, we will use **this** format for buttons, e.g. **A** means hitting the 'A' key. Mouse buttons are referred to as **LMB** and **RMB** for left and right mouse buttons.

## Overview <a name="overview"></a>

Creating adequate simulation models of a robot is a difficult task that especially in the world of open source and research oftentimes comes down to editing complex custom data (XML) files by hand. This is not only error-prone and slow, it also forces the robotics researcher or enthusiast to give up any visual context and painstakingly re-iterate construction using a text editor. While there are some remedies for this problem such as macro languages (e.g. XACRO) for robot definition or plugins for CAD-software allowing to export the structure of a robot, none of these allow the user to intuitively create robot models from scratch or customize the models used for construction for use in simulation with ease. This is a gap that Phobos is supposed to fill and we hope that many users out there will find it useful for their needs.

### Phobos, MARS and SMURF

Phobos was created to create models for MARS, a realtime physics simulation that has been developed at the German Research Center for Artificial Intelligence's Robotics Innovation Center (DFKI-RIC) in Bremen in collaboration with the University of Bremen. It uses the [Open Dynamics Engine (ODE)](http://www.ode.org/) to calculate rigid body physics and [OpenSceneGraph (OSG)](http://www.openscenegraph.org/) for visualization. MARS is very easily extendable through its plugin architecture and can be used within other applications, as it is by the [Rock robotics framework](http://rock-robotics.org/).

As Phobos, MARS is open-source and has its own [repository](https://github.com/rock-simulation/mars) on GitHub.

MARS is compatible with two different representations of robot models: its classic "MARS scene" and the newly-developed (S)upplementable, (M)ostly (U)niversal (R)obot (F)ormat (SMURF). While MARS scene files as described in the MARS documentation are still valid and will continue to function in the future, the future development of MARS will focus on SMURF as its preferred format and thus Phobos is being developed mainly for editing of SMURF-models. For this reason, no future feature implementations related to MARS scenes are planned. However, a lot of the functionality provided by Phobos (such as batch-editing) can be used for editing old blender files of MARS scenes as well.

#### URDF

The SMURF format uses at its heart [URDF](http://wiki.ros.org/urdf) (the Unified Robot Description Format) for defining the kinematic model of a robot. This format was created in the ROS world and contains only sparse information beyond the "mechanical skeleton" of a robot. Thus, the SMURF format encompasses additional files adding all information that is needed by MARS and in fact any custom-defined data one wants to assign to arbitrary parts of the robot. Thus Phobos is designed to assist the user in building a viable URDF robot model in Blender and annotate all additional information in SMURF that enables to simulate the robot in MARS or use any custom-tailored tools.
Notably, as URDF serves as the format for kinematic data, it is entirely possible to use Phobos for creating and editing only URDF and ignore the additional functionality provided by SMURF. We therefore hope that many people working with ROS will find Phobos interesting as well.

### Blender

Blender is a feature-rich, mature open-source 3D modelling and animation software that comes with an extensive and flexible Python scripting API. Using this API, it is possible to script almost any feature available from the GUI, thus allowing to automate repetitive tasks or simplify complex editing procedures.

### Phobos Add-On

Phobos is built upon a set of scripts which were originally created to use the power of Blender for modelling MARS scenes, i.e. the original scene format used in the MARS simulation. With the decision to switch our kinematics representation to URDF and embed it in a new hierarchical format to store all other relevant information, we also decided to take our approach to harnessing the power of Blender one step further and create a fully-fledged addon with its own GUI for Blender.

