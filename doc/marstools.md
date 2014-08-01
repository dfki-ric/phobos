MARStools Blender ADD-ON {#marstools}
===================

## Overview

The MARStools are an Add-On for Blender which simplifies editing robot models and environments to be used in the MARS simulation.

## Installation

The MARStools provide a simple way of installation via the *install.sh* shell script.

### Linux

### OSX

### Windows


## Robot Model

### Fomats

MARS uses two different representation of robot models, the (S)upplementable, (M)ostly (U)niversal (R)obot (F)ormat (SMURF) and the classic "MARS scene". While MARS scene files as described in the MARS documentation are still valid and will continue to function in the future, the MARStools Add-On was designed to support mainly the editing of SMURF-models. For this reason, any future feature implementations aiming for MARS scene related functionality are most likely not going to happen.

SMURF uses at its heart the URDF (Unified Robot Description Format) for defining the kinematic model of a robot. This format was created in the ROS world and contains only sparse information beyond the "mechanical skeleton" of a robot. Thus, the SMURF format encompasses additional files adding all additional information that is needed by MARS and in fact any custom-defined pieces of data one wants to assign to arbitrary parts of the robot. That being said, the MARStools assist the user in the core functionality of SMURF, i.e. building a viable URDF robot model in Blender and annotation pieces of information needed to simulate the robot in MARS.

### Structure of robots models

URDF models consist of *links*, which define coordinate frames relative to which objects for visualisation and collision detection can be arranged. These links themselves are placed relative to one another as defined in the accompanying *joints*, one joint connecting two links at a time. Thus a tree is built from these links interconnected by joints which at the same time forms a hierarchical, branching kinematic chain (thus the name *link*). In Blender, we represent these link coordinate frames as *armature* objects. By establishing parent-child relationships between links and thus placing the child link relative to the parent link in its parent's local coordinate frame, we can implicitly represent URDF's joints as well. Every link in turn contains its own child objects, which are regular blender mesh objects representing the robot's shape both visually (*visual* objects) and for purposes of collision simulation (*collision* objects). It is further possible to place additional *inertial* objects as children of links, thus re-defining their simulated center of mass and containing information on the inertia of a link. This is necessary as a link can contain multiple visual and collision objects, such that the inertia of the link has to be calculated from a number of collision objects and the center of mass is most likely not at the same position as the link itself.
Apart from these purely physical properties, URDF joints further define properties and limitations of dynamic behavior, as explained below.

### Joints

In URDF, there are a number of different joint types, listed as follows (taken from the URDF website):

revolute - a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
continuous - a continuous hinge joint that rotates around the axis and has no upper and lower limits
prismatic - a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
fixed - This is not really a joint because it cannot move. All degrees of freedom are locked. This type of joint does not require the axis, calibration, dynamics, limits or safety_controller.
floating - This joint allows motion for all 6 degrees of freedom.
planar - This joint allows motion in a plane perpendicular to the axis.

All these different types of joints can be represented in blender using the constaint definitions available for bones of Blender's armatures. Remember that every link is defined using one armature. Each of these armatures contains exactly one bone, which is oriented along the axis that defined the joint. Along or around this axis (depending on the type of joint), constraints can be set to limit the movement of the joint both in Blender (so that it is not possible to set a joint position in Blender that the real robot is not capable of) and are later imported by MARS. To save you the tedious work of setting all different constraints by hand, MARStools provides a button for this that simply let's you choose the type of joint and type in the desired limits. As all constraints are defined in the local coordinate frames of the individual bones, it is entirely possible to set all constraints of identical joints (if, for instance, a robot possesses multiple identically-built legs) at once.


### Masses and Inertia

if there is no single 'inertial_*linkname*' inertial, the summed-up masses are written to the link itself, as in this case, we only have masses and no inertia and thus do not want to export anything to URDF - in this case, the information in the link serves 

It's important for simulation models to have a similar or preferably identical mass distribution as the original robot. Editing a model in Blender, this can at times become tricky, but MARStools helps you with that, too. The obvious and surely simplest method is to assign the masses to the visual objects of the robot. In most setups, the visual objects of a link will represent the actual parts of a real robot and thus it is straightforward to weigh the real robot parts and afterwards set the masses in Blender accordingly.
Now URDF, in which we want to export, does not support masses for visual objects. It does not even support masses for collision objects which are, after all, a representation of physical objects for simulation purposes. What URDF does is demand the specification of one mass per link (specified in a separate *inertial* object, remember?), so that masses of several physical objects belonging to that link have to be combined. URDF further demands, if there is an inertial object in the link, that it also defines the resulting inertia matrix and optionally a center of mass location different from the origin of the link. To accomodate all these requirements, MARStools uses an implementation of separate, positionable inertia objects in Blender, with attributes *mass* (float) and *inertia* (string containing 6 floats representing the upper half of the inertia matrix - don't worry, there is a button for this, too!). Inertial objects are in fact the only way to get inertial data in URDF.
However, MARStools does not stop there and leave you in the dark on your own, condemned to calculate all the masses and inertias of your links. Instead, MARstools let's you, quite naturally, assign masses to its visual objects. It also let's you assign masses to collision objects. This is important as, contrary to first intuition, there need not be equal numbers of collision and visual objects. For instance, to better represent the internal mass distribution of the main body of a robot, one might want to place an additional collision object with a large mass to simulate the battery inside the collision object which represents the outer casing. There may be occasions, additionally, when one might want to show a visual object with a mass, but which at the same time does not collide.
Thus, to cover all possible cases, any visual or collision object in a link might carry a mass, yet not every such object may have a twin of the other type. This makes it impossible to sum up masses just using objects of one of the types exclusively. To make this situation work, MARStools let's you *synchronise* the masses you defined in your visual and collision objects, as long as it can detect pairs using their respective names. For instance, if there is a visual object called *visual_foot_front_left* and a collision object called *collision_foot_front_left*, they will be automatically paired for purposes of synchronisation. Note that this only works if the similarly-named objects are found in the same link. You will have noted that collision objects generated by MARStools from visual objects, this naming convention is used.
The synchronisation can be done in three ways: *visual to collision*, *collision to visual* and - in case you really lost track - *latest to oldest*. Note that synchronisation is only performed if both objects of a pair are selected.
So now we have assigned and synchronized all masses, leaving the problem of handling the inertia. MARStools can calculate inertia from collision objects if they contain mass information and create inertial objects holding both mass and inertia information as children of the *link*. If they are thus created, they will follow the same name convention as visual and collision objects, thus an inertial derived from object *collision_foot_front_left* will be named *inertial_foot_front_left*. MARStools can digest inertia and mass information in all such inertial objects in a link to calculate the mass and inertia of the link as a whole. Note, however, that no such calculation will be undertaken if there is a link-level inertial object already containing the information for the entire link.


## Exporting your model

Model data can be exported with the MARStools in a number of formats: YAML, URDF, SMURF


### URDF

URDF, the "Unified Robot Description Format", is an XML-Format limited to providing information about a robot's kinematics. Other types of information, such as referring to motors, sensors etc., is mostly ignored or limited to specific ROS solutions. While closely affiliated with the ROS framework, URDF is not part of ROS (any more) and the accompanying tools for parsing etc. can be used independently. The complete definition of URDF can be found here: ![http://wiki.ros.org/urdf/XML]. Regarding its core functionality, i.e., defining kinematics, URDF is further limited as it does not allow to specify parallel linkages (joint loops).


## The MARStools Robot Representation

To facilitate simple im- and export of robot models in various formats, the MARStools use a file-format-independent Python dictionary representation to store robot data. This dictionary representation is loosely based on URDF/SDF as well as MARS' and 'Blender's own naming conventions. It is layed out in the following in YAML-notation:

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

## Blender Transformations

== Rotation Matrices ==

In Blender, every object possesses four different transformation matrices to save it current location, rotation and scale in space. These are:

- matrix_basis
- matrix_local
- matrix_world
- matrix_parent_inverse

These matrices make the parent-child relationships possible that objects can be structurally ordered with in Blender. Thus very different behavior can be observed for these matrices depending on whether they belong to a *global* or *child* object. Global objects, i.e. objects which do not have a parent, their location, rotation and scale are fully defined in relation to the world, thus their matrix_basis, matrix_local and matrix_world are all equal. As they possess no parent, the parent inverse matrix is the identity transform. In objects which are children of other objects, i.e., which have a parent, the three matrices serve different functions, as outlined below.

=== matrix_basis ===

matrix_basis is what is displayed in Blender's "Transform" settings in the sidebar. This is often a point of confusion because it does *not* necessarily refer to the object's actual position and orientation in space. For global objects it does.

=== matrix_local ===

The local matrix defines the transformation from the parent world transform to the child transform. This means that if the local matrix of a child is applied to its basis, it would reside at its parent's origin.

=== matrix_world ===

The absolute transform of the object in the world. This is the location, orientation and scale at which the object is displayed in blender.

=== matrix_parent_inverse ===

The parent inverse matrix is set at the time of parenting and *never changes afterwards*, no matter what transformations are applied to the parent or the child (in fact, it doesn't even change after a parent-child relationship is cancelled and is simply ignored by Blender until the object becomes a child again, in which case it is simply overwritten). It is the transformation which, if applied to the child, reverses the change of the origin of the child's coordinate system that resulted from establishing the parent-child relationship. It is thus equal to the inverse of the parent's *world* transform. This last point is important! As the parent could be a child of another object, the local and basis transforms would not reflect the change of origin of the parent, thus the world transform has to be use to derive the parent inverse.

=== Summary ===

- matrix_basis: the object's "own" transform in its own coordinate system
- matrix_local: the transform which brings an object from it's parent's origin to its position in the world
- matrix_world: the absolute transform of the object with respect to the world
- matrix_parent_inverse: the inverse of the parent's world transform at time of parenting


== Location & Transformation ==
