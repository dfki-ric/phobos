Phobos Blender Add-On {#phobos}
===================

Phobos is an open-source Add-On for Blender which simplifies creating and editing robot models and environments to be used in the MARS simulation. Specifically, Phobos allows the user to create URDF files as well as SMURF robot descriptions in a visual, interactive user interface, providing a number of import and export options.

Check out the Phobos [Roadmap](roadmap.md)

## Installation

See [Phobos installation](installation.md).

## Tutorials

All tutorials will also be linked in the relevant sections of this documentation.

## Overview

Creating adequate simulation models of a robot is a difficult task that especially in the world of open source and research oftentimes comes down to editing complex custom data (XML) files by hand. This is not only error-prone and slow, it also forces the robotics researcher or enthusiast to give up any visual context and painstakingly re-iterate construction using a text editor. While there are some remedies for this problem such as macro languages (e.g. XACRO) for robot definition or plugins for CAD-software allowing to export the structure of a robot, none of these allow the user to intuitively create robot models from scratch or customize the models used for construction for use in simulation with ease. This is a gap that Phobos is supposed to fill and we hope that many users out there will find it useful for their needs.

### The MARS simulation framework

MARS is a realtime physics simulation that has been developed at the German Research Center for Artificial Intelligence's Robotics Innovation Center (DFKI-RIC) in Bremen in collaboration with the University of Bremen. It uses the [Open Dynamics Engine (ODE)](http://www.ode.org/) to calculate rigid body physics and [OpenSceneGraph (OSG)](http://www.openscenegraph.org/) for visualization. MARS is very easily extendable through its plugin architecture and can be used within other applications, as it is by the [Rock robotics framework](http://rock-robotics.org/).

As Phobos, MARS is open-source and has its own [repository](https://github.com/rock-simulation/mars) on GitHub.

### Blender

Blender is a feature-rich, mature open-source 3D modelling and animation software that comes with an extensive and flexible Python scripting API. Using this API, it is possible to script almost every feature available from the GUI, thus allowing to automate repetitive tasks or simplify complex editing procedures.

### Phobos Add-On

Phobos is built upon a set of scripts which were originally created to use the power of Blender for modelling MARS scenes, i.e. the original scene format used in the MARS simulation. With the decision to switch our kinematics representation to URDF and embed it in a new hierarchical format to store all other relevant information, we also decided to take our approach to harnessing the power of Blender one step further and create a fully-fledged addon with its own GUI for Blender.

## Robot Model

### Formats

MARS is compatible with two different representations of robot models: its classic "MARS scene" and the newly-developed (S)upplementable, (M)ostly (U)niversal (R)obot (F)ormat (SMURF). While MARS scene files as described in the MARS documentation are still valid and will continue to function in the future, the future development of MARS will focus on SMURF as its preferred format and thus Phobos is being developed mainly for editing of SMURF-models. For this reason, no future feature implementations related to MARS scenes are planned. However, a lot of the functionality provided by Phobos (such as batch-editing) can be used for editing old blender fiels of MARS scenes as well.

SMURF uses at its heart URDF (the Unified Robot Description Format) for defining the kinematic model of a robot. This format was created in the ROS world and contains only sparse information beyond the "mechanical skeleton" of a robot. Thus, the SMURF format encompasses additional files adding all information that is needed by MARS and in fact any custom-defined data one wants to assign to arbitrary parts of the robot. Thus Phobos is designed to assist the user in building a viable URDF robot model in Blender and annotate all additional information in SMURF that enables to simulate the robot in MARS.
Notably, as URDF serves as the format for kinematic data, it is entirely possible to use Phobos for creating and editing only URDF and ignore the additional functionality provided by SMURF. We therefore hope that many people working with ROS will find our tools interesting as well.

### Structure of robot models

URDF is a very well-documented format and before reading on, you might want to have a look at its [documentation](http://wiki.ros.org/urdf/XML/model) to get an overview of how it works. We'll give a short overview here nonetheless.

URDF models consist of *links* defining coordinate frames relative to which objects for visualisation and collision detection can be arranged. These links themselves are placed relative to one another as defined in the accompanying *joints*, one joint connecting two links at a time. Thus a tree is built from these links interconnected by joints which at the same time forms a hierarchical, branching kinematic chain (thus the name *link*). Fortunately, Blender also arranges 3D objects in a hierarchy, enabling essentially a WYSIWYG (What-you-see-is-what-you-get) approach when emulating URDF. However, there are some limitations, the most important one being that *links* and *joints* are not represented as separate objects. This is due to the simple fact that a Blender objects always contain a transformation relative to their parent object and thus the transformation information of a joint can be incorporated in an object that already represents a *link*.

Such "superlinks" containing their parent joint's information are represented in Blender using *armature* objects. Each such armature contains a single *bone* which in turn saves the contraints defined for the related joint. By establishing parent-child relationships between links and thus placing the child link relative to the parent link in its parent's local coordinate frame, we can implicitly specify the corresponding joint transformation. Every link in turn contains its own child objects, which are regular blender mesh objects representing the robot's shape both visually (*visual* objects) and for purposes of collision simulation (*collision* objects). It is further possible to place additional *inertial* objects as children of links, thus re-defining their simulated center of mass and containing information on the inertia of a link. This is necessary as a link can contain multiple visual and collision objects, such that the inertia of the link has to be calculated from a number of collision objects and the center of mass is most likely not at the same position as the link itself.

Beyond the information contained in URDF that can be directly represented in Blender, there are other data fields which have no direct parallel in Blender models. However, Blender allows to assign custom properties to every object in a scene, allowing to store any piece of information in an object that URDF specifies. Moreover, this also enables the definition of further variables for use in SMURF.

We conclude this section with a short note on armatures and bones. Blender's armatures are designed for building a skeleton of an animated character, consisting of multiple *bones* which can be linked to one another and constrained in their movement. Due to a number of technical details, we chose not to use one armature per model and one bone per "superlink" - as may be intuitive -, but instead one armature object per link, with each armature containing one single bone. This among other things simplifies copying and pasting of entire limbs with their attached visual and collision objects. However it also complicates certain tasks, such as encoding and switching between robot poses. Using multiple armatures was the most effective solution for the feature set wet implemented initially, but we may well change this design in the future.

### Joints

In URDF, there are a number of different joint types, listed as follows (taken from the URDF website on [joints](http://wiki.ros.org/urdf/XML/joint)):

- revolute - a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
- continuous - a continuous hinge joint that rotates around the axis and has no upper and lower limits
- prismatic - a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
- fixed - This is not really a joint because it cannot move. All degrees of freedom are locked. This type of joint does not require - the axis, calibration, dynamics, limits or safety_controller.
- floating - This joint allows motion for all 6 degrees of freedom.
- planar - This joint allows motion in a plane perpendicular to the axis.

All these different types of joints can be represented in blender using the constraint definitions available for bones of Blender's armatures. Remember that every link is defined using one armature. Each of these armatures contains exactly one bone, which is oriented along the axis that defines the joint. Along or around this axis (depending on the type of joint), constraints can be set to limit the movement of the joint both in Blender (so that it is not possible to set a joint position in Blender that the real robot is not capable of) and are later imported by MARS. To save you the tedious work of setting all different constraints by hand, Phobos provides a button for this that simply let's you choose the type of joint and type in the desired limits. As all constraints are defined in the local coordinate frames of the individual bones, it is entirely possible to set all constraints of identical joints (if, for instance, a robot possesses multiple identically-built legs) at once. Furthermore, the dynamic properties maximum effort and velocity can also be set using the same operator.

### Masses and Inertia

It's important for simulation models to have a similar or preferably identical mass distribution as the original robot. Editing a model in Blender, this can at times become tricky, but Phobos helps you with that, too. The obvious and surely simplest method is to assign the masses to the visual objects of the robot as custom properties. In most setups, the visual objects of a link will represent the actual parts of a real robot and thus it is straightforward to weigh those parts set the masses in Blender accordingly.
Now URDF, in which we want to export, does not support masses for visual objects. It does not even support masses for collision objects which are, after all, a representation of physical objects for simulation purposes. What URDF does is demand the specification of one mass value per link, specified in a separate *inertial* element, so that masses of several physical objects belonging to that link have to be combined. URDF further demands that if there is an inertial object in the link, it also needs to define the resulting inertia tensor (as well as optionally a shifted center of mass location). Phobos does not leave you in the dark on your own, condemned to calculate all the masses and inertias of your links by yourself, but offers different solutions to these problems (beyond the obvious solution of not exporting mass and inertia information at all, which is of course possible).

As noted before, mass can be assigned, quite naturally, to visual objects in the form of a custom property. This can be done manually or using the "Edit custom property" operator. However we recommend using the explicit "Edit mass" operator, as it assigns a time stamp to the object, saving when the mass was changed. Why this may be handy is explained in the following.
Contrary to first intuition, there need not be equal numbers of collision and visual objects. For instance, to better represent the internal mass distribution of the main body of a robot, one might want to place an additional collision object with a large mass to simulate the battery inside the collision object which represents the outer casing. There may be other occasions when one wants to simulate a visual object with a mass, but no collision. Thus, to cover all possible cases, Phobos lets you assign masses not only to visual, but also to collision objects; yet not every such object may have a twin of the other type (let's call such corresponding visual and collision objects *pairs* or *partner* objects). This makes it impossible to calculate the mass of a link as a whole simply by summing up the masses of its child objects of one type alone. To make this work, Phobos let's you synchronise the masses you defined in your visual and collision objects, as long as it can detect pairs using their respective names. For instance, if there is a visual object called *visual_foot_front_left* and a collision object called *collision_foot_front_left*, they will be automatically paired for purposes of synchronisation. Note that this only works if the similarly-named objects are found in the same link. You will note that for collision objects generated by Phobos from visual objects, this naming convention is used by default.
The synchronisation can be done in three ways: *visual to collision*, *collision to visual* and - in case you really lost track - *latest to oldest*. For this last variety, we need the timestamps set by the "Edit mass" operator mentioned before. Note that synchronisation is only performed if both objects of a pair are selected.

So now we have assigned and synchronized all masses, leaving the problem of handling the inertia. The first option to deal with it is to not deal with it: if only mass but no inertia information is provided, the masses are still exported to SMURF in the form of simulation-relevant annotations to visual and collision objects. In this case, MARS lets ODE (the physics engine used) calculate inertia values from the geometry information provided for the objects, which is a reasonable approximation as long as real and simulated geometry are similar and the mass distribution of the real objects are rather homogenous. The drawback of this method is that different physics engines or simulation software might handle such cases differently and thus compatibility between simulations becomes an issue (or rather even more of an issue than it is already).
If one wishes to have more control, one can let Phobos calculate inertia from collision objects if they contain mass information and are of a primitive type; Phobos cannot calculate inertia for mesh geometries. Upon this calculation, inertial objects holding both mass and inertia information are created as children of the *link*. Note that the inertia tensor is calculated in local coordinate space of the object, not of the link. Inertial objects thus created follow a similar naming convention as visual and collision objects, i.e. an inertial derived from object *collision_foot_front_left* will be named *inertial_collsion_foot_front_left*. Explicitly creating these objects in Blender lets the user review the inertia tensors resulting from the collision geometry. Phobos is also able to combine inertia and mass information in all such inertial objects of a link to calculate the mass and inertia of the link as a whole, storing the information in a separate "link inertial object" (named "inertial_linkname"). This is the data that will be exported to URDF.
Finally, if an approximation based on collision geometry - assuming homogenous density - won't do, the user can specify inertia explicitly. For this reason, both visual/collision-level inertial objects or link-level inertial objects can be created manually and filled with inertia data that was precalculated, exported from CAD or measured on the real robot parts. This method can also be combined with the aforementioned one by creating inertial objects for every relevant visual and collision object, filling in the correct inertias and then letting Phobos calculate the combined inertia of the link.


## Editing a model

### Some Definitions

*Operators* are constructs in Blender that can be called from the GUI, thus whenever you click a button or menu item, you execute an operator. While there is more to operators than that, when we talk about an operator, unless explicitly specified differently, we refer to the button.


## Exporting your model

Model data can be exported with Phobos in a number of formats: YAML, URDF, SMURF.


### URDF

URDF, the "Unified Robot Description Format", is an XML-Format limited to providing information about a robot's kinematics. Other types of information, such as referring to motors, sensors etc., is mostly ignored or limited to specific ROS solutions. While closely affiliated with the ROS framework, URDF is not part of ROS (any more) and the accompanying tools for parsing etc. can be used independently. The complete definition of URDF can be found here: ![http://wiki.ros.org/urdf/XML]. Regarding its core functionality, i.e., defining kinematics, URDF is further limited as it does not allow to specify parallel linkages (joint loops).


## Phobos' Robot Representation

To facilitate simple im- and export of robot models in various formats, Phobos uses a file-format-independent Python dictionary representation to store robot data. This dictionary representation is loosely based on URDF/SDF, with some added MARS' and 'Blender's own naming conventions. It is layed out in the following in YAML-notation:

model
- {link}
	- filename: str
	- [pose]: (d, d, d, d, d, d, d)
	- {visual}:
		- {visual_1}
			- [pose]: (d, d, d, d, d, d, d) #x, y, z, w, x, y, z
			- {material}:
				- name: str
				- [diffuseColor]: (d, d, d, d)
				- [ambientColor]: (d, d, d, d)
				- [emissionColor]: (d, d, d, d)
				- [specularColor]: (d, d, d, d)
				- transparency: d
			- {geometry}:
				- type: str ("box" | "sphere" | "cylinder" | "plane" | "mesh")
				- radius: d #sphere
				- [size]: (d, d, d) #box
				- radius, height: d, d #cylinder
				- [size]: (d, d, d) #mesh
				- [size]: (d, d) #plane
                - filename: str #mesh
		- {visual_2}
			- ...
		- ...
	- {collision}
		- {collision_1}
			- bitmask: int
			- {geometry}:
				- type: str ("box" | "sphere" | "cylinder" | "plane" | "mesh")
				- radius: d #sphere
				- [size]: (d, d, d) #box
				- radius, height: d, d #cylinder
				- [size]: (d, d, d) #mesh
				- [size]: (d, d) #plane
				- filename #mesh
			- [pose]: (d, d, d, d, d, d, d)
			- max_contacts: int
		- {collision_2}
			- ...
		- ...
	- {inertial}:
		- mass: d
		- [inertia]: (d, d, d, d, d, d) #ixx, ixy, ixz, iyx, iyy, iyz
- {joint}
	- parent: str
	- child: str
	- jointType: str ("hinge", "continuous", "linear")
- ["sensor"]
	- link: str
	- sensorType: str
- ["motor"]
- ["controller"]
- {group}:
	- [group1]: (str, ...) #names of links
	- ...


int: integer
d: double
str: string

## Some Notes on Blender

### Blender Transformation Matrices

In Blender, every object possesses four different transformation matrices to save its current location, rotation and scale in space. These are:

- matrix_basis
- matrix_local
- matrix_world
- matrix_parent_inverse

These matrices make the parent-child relationships possible that objects can be structurally ordered with in Blender. Thus very different behavior can be observed for these matrices depending on whether they belong to a *global* or *child* object. Global objects, i.e. objects which do not have a parent, their location, rotation and scale are fully defined in relation to the world, thus their matrix_basis, matrix_local and matrix_world are all equal. As they possess no parent, the parent inverse matrix is the identity transform. In objects which are children of other objects, i.e., which have a parent, the three matrices serve different functions, as outlined below.

=== matrix_basis ===

matrix_basis is what is displayed in Blender's "Transform" settings in the sidebar. This is often a point of confusion because it does *not* necessarily refer to the object's actual position and orientation in space, the exception being global objects, where it always does.

=== matrix_local ===

The local matrix defines the transformation from the parent world transform to the child transform. This means that if the local matrix of a child is applied to its basis, it would reside at its parent's origin.

=== matrix_world ===

The absolute transform of the object in the world. This is the location, orientation and scale at which the object is displayed in blender.

=== matrix_parent_inverse ===

The parent inverse matrix is set at the time of parenting and *never changes afterwards*, no matter what transformations are applied to the parent or the child (in fact, it doesn't even change after a parent-child relationship is cancelled and is simply ignored by Blender until the object becomes a child again, in which case it is simply overwritten). It is the transformation which, if applied to the child, reverses the change of the origin of the child's coordinate system that resulted from establishing the parent-child relationship. It is thus equal to the inverse of the parent's *world* transform. This last point is important! As the parent could be a child of another object, the local and basis transforms would not reflect the change of origin of the parent, thus the world transform has to be used to derive the parent inverse.

=== Summary ===

- matrix_basis: the object's "own" transform in its own coordinate system
- matrix_local: the transform which brings an object from it's parent's origin to its position in the world
- matrix_world: the absolute transform of the object with respect to the world
- matrix_parent_inverse: the inverse of the parent's world transform at time of parenting


== Location & Transformation ==
