#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Contains the functions required to model a joint within Blender.
"""
import math

import bpy
import mathutils
import numpy as np
from bpy.app.handlers import persistent

from ..phoboslog import log
from ..utils import io as ioUtils
from ..utils import selection as sUtils
from ..utils.validation import validate

from ..reserved_keys import JOINT_KEYS

def getJointConstraints(joint):
    """Returns the constraints defined in the joint as tuple of two lists.

    Args:
      joint(bpy_types.Object): The joint you want to get the constraints from

    Returns:
      : tuple -- lists containing axis and limit data

    """
    jt, crot = deriveJointType(joint, adjust=False)
    axis = None
    limits = None
    if jt not in ['floating', 'fixed']:
        if jt in ['revolute', 'continuous'] and crot:
            c = getJointConstraint(joint, 'LIMIT_ROTATION')
            # TODO delete me?
            # we cannot use joint for both as the first is a Blender 'Object', the second an 'Armature'
            # axis = (joint.matrix_local * -bpy.data.armatures[joint.name].bones[0].vector).normalized()
            # joint.data accesses the armature, thus the armature's name is not important anymore
            # axis = (joint.matrix_local * -joint.data.bones[0].vector).normalized()
            axis = joint.data.bones[
                0
            ].vector.normalized()  # vector along axis of bone (Y axis of pose bone) in object space
            if crot[0]:
                limits = (c.min_x, c.max_x)
            elif crot[1]:
                limits = (c.min_y, c.max_y)
            elif crot[2]:
                limits = (c.min_z, c.max_z)
        else:
            c = getJointConstraint(joint, 'LIMIT_LOCATION')
            if not c:
                raise Exception(
                    "JointTypeError: under-defined constraints in joint (" + joint.name + ")."
                )
            freeloc = [
                c.use_min_x and c.use_max_x and c.min_x == c.max_x,
                c.use_min_y and c.use_max_y and c.min_y == c.max_y,
                c.use_min_z and c.use_max_z and c.min_z == c.max_z,
            ]
            if jt == 'prismatic':
                if sum(freeloc) == 2:
                    # TODO delete me?
                    # axis = mathutils.Vector([int(not i) for i in freeloc])
                    # vector along axis of bone (Y axis of pose bone) in obect space
                    axis = joint.data.bones[0].vector.normalized()
                    if not freeloc[0]:
                        limits = (c.min_x, c.max_x)
                    elif not freeloc[1]:
                        limits = (c.min_y, c.max_y)
                    elif not freeloc[2]:
                        limits = (c.min_z, c.max_z)
                else:
                    raise Exception(
                        "JointTypeError: under-defined constraints in joint (" + joint.name + ")."
                    )
            elif jt == 'planar':
                if sum(freeloc) == 1:
                    axis = mathutils.Vector([int(i) for i in freeloc])
                    if axis[0]:
                        limits = (c.min_y, c.max_y, c.min_z, c.max_z)
                    elif axis[1]:
                        limits = (c.min_x, c.max_x, c.min_z, c.max_z)
                    elif axis[2]:
                        limits = (c.min_x, c.max_x, c.min_y, c.max_y)
                else:
                    raise Exception(
                        "JointTypeError: under-defined constraints in joint (" + joint.name + ")."
                    )
    return axis, limits


def getJointConstraint(joint, ctype):
    """Returns the constraints of a given joint.

    Args:
      joint(bpy_types.Object): the joint in question
      ctype: constraint type to retrieve

    Returns:

    """
    con = None
    for c in joint.pose.bones[0].constraints:
        if c.type == ctype:
            con = c
    return con


# TODO are spring and damping really required as defaults?
def setJointConstraints(
    joint,
    jointtype,
    lower=0.0,
    upper=0.0,
    lower2=0.0,
    upper2=0.0,
    spring=None,
    damping=None,
    velocity=None,
    effort=None,
    velocity2=None,
    effort2=None,
    maxeffort_approximation=None,
    maxspeed_approximation=None,
    axis = None,
    axis2 = None,
    gearboxreferencebody=None,
    gearboxratio=None,
    screwthreadpitch=None,
    visualaxis=None,
):
    """Sets the constraints for a given joint and jointtype.
    
    If the joint type is not recognized, the constraints will match a floating joint.
    
    The approximation for maximum effort/speed requires a dictionary with two entries (*function*
    *coefficients*).
    
    Based on the joint type, the respective resource object is applied to the link.

    Args:
      joint(bpy_types.Object): link object containing the joint to be edited
      jointtype(str): joint type (revolute, continuous, prismatic, fixed, floating, planar)
      lower(float, optional): lower limit of the constraint (defaults to 0.)
      upper(float, optional): upper limit of the constraint (defaults to 0.)
      velocity(float, optional): velocity limit of the constraint (defaults to 0.)
      effort(float, optional): effort limit of the constraint (defaults to 0.)
      spring(float, optional): spring stiffness for the joint (Default value = 0.0)
      damping(float, optional): spring damping for the joint (Default value = 0.0)
      maxeffort_approximation(dict, optional): function and coefficients for maximum effort (Default value = None)
      maxspeed_approximation(dict, optional): function and coefficients for maximum speed (Default value = None)

    Returns:

    """

    bpy.ops.object.select_all(action='DESELECT')
    joint.select_set(True)
    if joint.phobostype != 'link':
        log("Cannot set joint constraints. Not a link: {}".format(joint), 'ERROR')
        return

    log("Setting joint constraints at link {}.".format(joint.name), 'DEBUG')
    bpy.ops.object.mode_set(mode='POSE')

    # remove existing constraints from bone
    for cons in joint.pose.bones[0].constraints:
        joint.pose.bones[0].constraints.remove(cons)

    # set axis
    for ax, parameter in [(axis, "joint/axis"), (axis2, "joint/axis2")]:
        if ax is not None:
            if mathutils.Vector(tuple(ax)).length == 0.:
                log('Axis of joint {0} is of zero length: '.format(joint.name), 'ERROR')
            ax = (np.array(ax) / np.linalg.norm(ax)).tolist()
            joint[parameter] = mathutils.Vector(tuple(ax))

    # rotate joint object
    visualaxis = visualaxis if visualaxis is not None else axis if axis is not None else [0, 0, 1]
    if np.linalg.norm(visualaxis) != 0:
        bpy.ops.object.mode_set(mode='EDIT')
        editbone = joint.data.edit_bones[0]
        length = max(editbone.length, 0.1)  # make sure we do not have zero length
        editbone.tail = editbone.head + mathutils.Vector(tuple(visualaxis)).normalized() * length
        bpy.ops.object.mode_set(mode='POSE')

    # add spring & damping
    if spring or damping:
        joint['joint/dynamics/spring_stiffness'] = spring
        joint['joint/dynamics/damping'] = damping
        applySpringDamping(joint)

    # set constraints accordingly
    joint['joint/limits/lower'] = lower
    joint['joint/limits/upper'] = upper
    joint['joint/limits/lower2'] = lower2
    joint['joint/limits/upper2'] = upper2
    joint.data.bones.active = joint.pose.bones[0].bone
    joint.data.bones.active.select = True
    remove_screwdrivers(joint)
    if jointtype == 'revolute':
        set_revolute(joint, lower, upper)
    elif jointtype == 'continuous':
        set_continuous(joint)
    elif jointtype == 'prismatic':
        set_prismatic(joint, lower, upper)
    elif jointtype == 'fixed':
        set_fixed(joint)
    elif jointtype == 'floating':
        # 6DOF
        pass
    elif jointtype == 'planar':
        set_planar(joint)
    elif jointtype == 'ball':
        set_ball(joint, upper)
    elif jointtype == 'universal':
        set_universal(joint, lower, upper, lower2, upper2)
    elif jointtype == 'screw':
        set_screw(joint, lower, upper, axis, screwthreadpitch)
    elif jointtype == 'gearbox':
        set_gearbox(joint, axis, axis2, gearboxratio)
    else:
        log("Unknown joint type for joint " + joint.name + ". Behaviour like floating.", 'WARNING')
    joint['joint/type'] = jointtype
    bpy.ops.object.mode_set(mode='OBJECT')

    # check for approximation functions of effort and speed
    joint['joint/limits/velocity'] = velocity
    joint['joint/limits/effort'] = effort
    joint['joint/limits/velocity2'] = velocity2
    joint['joint/limits/effort2'] = effort2
    if maxeffort_approximation:
        if all(elem in ['function', 'coefficients'] for elem in maxeffort_approximation):
            joint['joint/limits/maxeffort_approximation'] = maxeffort_approximation['function']
            joint['joint/limits/maxeffort_coefficients'] = maxeffort_approximation['coefficients']
        else:
            log(
                "Approximation for max effort ill-defined in joint object {}.".format(
                    joint.name
                ),
                'ERROR',
            )
    if maxspeed_approximation:
        if all(elem in ['function', 'coefficients'] for elem in maxspeed_approximation):
            joint['joint/limits/maxspeed_approximation'] = maxspeed_approximation['function']
            joint['joint/limits/maxspeed_coefficients'] = maxspeed_approximation['coefficients']
        else:
            log(
                "Approximation for max speed ill-defined in joint object {}.".format(
                    joint.name
                ),
                'ERROR',
            )

    # gearbox
    joint['joint/gearbox/reference_body'] = gearboxreferencebody
    joint['joint/gearbox/ratio'] = gearboxratio

    # screw
    joint['joint/screw/thread_pitch'] = screwthreadpitch

    # set link/joint visualization
    resource_obj = ioUtils.getResource(('joint', jointtype))
    if resource_obj:
        log("Assigned resource to {}.".format(joint.name), 'DEBUG')
        joint.pose.bones[0].custom_shape = resource_obj

    # delete unused properties
    for key in JOINT_KEYS:
        key = "joint/"+key
        if key in joint and joint[key] is None:
            del joint[key]


@persistent
def load_handler(dummy):
    bpy.app.handlers.depsgraph_update_post.append(sdUpdater)


bpy.app.handlers.load_post.append(load_handler)

sdUpdaterObj = None
sdUpdaterDamping = 0
sdUpdaterSpring = 0
springKey = 'joint/dynamics/spring_stiffness'
dampingKey = 'joint/dynamics/damping'

def sdUpdater(scene):
    """
    Updates spring and damping if the values are changed by the user
    """
    global sdUpdaterObj, sdUpdaterDamping, sdUpdaterSpring
    obj = bpy.context.object

    if obj is None:
        return

    if sdUpdaterObj == obj:
        changeDetected = False

        # See if damping changed
        if dampingKey in obj:
            damping = obj[dampingKey]
            if damping != sdUpdaterDamping:
                changeDetected = True

        # See if spring changed
        if springKey in obj:
            spring = obj[springKey]
            if spring != sdUpdaterSpring:
                changeDetected = True

        # Apply new constraint if either changed
        if changeDetected:
            applySpringDamping(obj)

    else:  # A new object was selected, remember spring and damping
        sdUpdaterObj = obj
        sdUpdaterDamping = obj[dampingKey] if dampingKey in obj else None
        sdUpdaterSpring = obj[springKey] if springKey in obj else None


def applySpringDamping(joint):
    """
    Adds a rigid body constraint to a joint
    """
    global sdUpdaterObj, sdUpdaterDamping, sdUpdaterSpring

    # Read spring and damping from joint
    spring = joint[springKey] if springKey in joint else 0
    damping = joint[dampingKey] if dampingKey in joint else 0

    # Remember values of selected object
    sdUpdaterSpring = spring
    sdUpdaterDamping = damping
    sdUpdaterObj = joint

    if spring == 0 and damping == 0:
        # Remove constraint
        bpy.ops.rigidbody.constraint_remove()
    else:
        # Add new constraint if it's not present
        constraintExists = joint.rigid_body_constraint is not None
        try:
            if not constraintExists:
                bpy.ops.rigidbody.constraint_add(type='GENERIC_SPRING')
            # Add spring and damping via rigid body constraint
            bpy.context.object.rigid_body_constraint.spring_stiffness_y = spring
            bpy.context.object.rigid_body_constraint.spring_damping_y = damping
            bpy.context.object.rigid_body_constraint.use_spring_y = True
        except RuntimeError:
            log("No Blender Rigid Body World present, only adding custom properties.", 'ERROR')



def getJointType(joint):
    """

    Args:
      joint: 

    Returns:

    """
    jtype = 'floating'
    cloc = None
    crot = None
    limrot = None
    # we pick the first bone in the armature as there is only one
    for c in joint.pose.bones[0].constraints:
        if c.type == 'LIMIT_LOCATION':
            cloc = [
                c.use_min_x and c.min_x == c.max_x,
                c.use_min_y and c.min_y == c.max_y,
                c.use_min_z and c.min_z == c.max_z,
            ]
        elif c.type == 'LIMIT_ROTATION':
            limrot = c
            crot = [
                c.use_limit_x and (c.min_x != 0 or c.max_x != 0),
                c.use_limit_y and (c.min_y != 0 or c.max_y != 0),
                c.use_limit_z and (c.min_z != 0 or c.max_z != 0),
            ]
    ncloc = sum(cloc) if cloc else None
    ncrot = sum((limrot.use_limit_x, limrot.use_limit_y, limrot.use_limit_z)) if limrot else None
    # all but floating joints have translational limits
    if cloc:
        # fixed, revolute or continuous
        if ncloc == 3:
            if ncrot == 3:
                if sum(crot) > 0:
                    jtype = 'revolute'
                else:
                    jtype = 'fixed'
            elif ncrot == 2:
                jtype = 'continuous'
        elif ncloc == 2:
            jtype = 'prismatic'
        elif ncloc == 1:
            jtype = 'planar'
    return jtype, crot


@validate('joint_type')
def deriveJointType(joint, logging=False, adjust=False, errors=None):
    """Derives the type of the joint defined by the armature object.
    
    If the constraints do not match the specified joint type, a warning is logged. By using the
    adjust parameter it is possible to overwrite the joint type according to the specified joint
    constraints.

    Args:
      joint(bpy_types.Object): link object to derive the joint type from
      adjust(bool, optional): whether or not the type of the joint is corrected for the object
    according to the constraints (overwriting the existing joint type) (Default value = False)
      logging: (Default value = False)
      errors: (Default value = None)

    Returns:
      : tuple(2) -- jtype, crot

    """
    joint_type, crot = getJointType(joint)

    return joint_type, crot


def deriveJointState(joint):
    """Calculates the state of a joint from the state of the link armature.
    Note that this is the current state and not the zero state.

    Args:
      joint(bpy_types.Object): The joint(armature) to derive its state from.

    Returns:
      : dict

    """
    state = {
        'matrix': [list(vector) for vector in list(joint.pose.bones[0].matrix_basis)],
        'translation': list(joint.pose.bones[0].matrix_basis.to_translation()),
        'rotation_euler': list(joint.pose.bones[0].matrix_basis.to_euler()),
        'rotation_quaternion': list(joint.pose.bones[0].matrix_basis.to_quaternion()),
    }
    # TODO: hard-coding this could prove problematic if we at some point build armatures from multiple bones
    return state


def set_revolute(joint, lower, upper):
    """

    Args:
      joint: 
      lower: 
      upper: 

    Returns:

    """
    # fix location
    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
    cloc = getJointConstraint(joint, 'LIMIT_LOCATION')
    cloc.use_min_x = True
    cloc.use_min_y = True
    cloc.use_min_z = True
    cloc.use_max_x = True
    cloc.use_max_y = True
    cloc.use_max_z = True
    cloc.owner_space = 'LOCAL'
    # fix rotation x, z and limit y
    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
    crot = getJointConstraint(joint, 'LIMIT_ROTATION')
    crot.use_limit_x = True
    crot.min_x = 0
    crot.max_x = 0
    crot.use_limit_y = True
    crot.min_y = lower
    crot.max_y = upper
    crot.use_limit_z = True
    crot.min_z = 0
    crot.max_z = 0
    crot.owner_space = 'LOCAL'


def set_continuous(joint):
    """

    Args:
      joint: 

    Returns:

    """
    # fix location
    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
    cloc = getJointConstraint(joint, 'LIMIT_LOCATION')
    cloc.use_min_x = True
    cloc.use_min_y = True
    cloc.use_min_z = True
    cloc.use_max_x = True
    cloc.use_max_y = True
    cloc.use_max_z = True
    cloc.owner_space = 'LOCAL'
    # fix rotation x, z
    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
    crot = getJointConstraint(joint, 'LIMIT_ROTATION')
    crot.use_limit_x = True
    crot.min_x = 0
    crot.max_x = 0
    crot.use_limit_z = True
    crot.min_z = 0
    crot.max_z = 0
    crot.owner_space = 'LOCAL'


def set_prismatic(joint, lower, upper):
    """

    Args:
      joint: 
      lower: 
      upper: 

    Returns:

    """
    # fix location except for y axis
    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
    cloc = getJointConstraint(joint, 'LIMIT_LOCATION')
    cloc.use_min_x = True
    cloc.use_min_y = True
    cloc.use_min_z = True
    cloc.use_max_x = True
    cloc.use_max_y = True
    cloc.use_max_z = True
    if lower == upper:
        cloc.use_min_y = False
        cloc.use_max_y = False
    else:
        cloc.min_y = lower
        cloc.max_y = upper
    cloc.owner_space = 'LOCAL'
    # fix rotation
    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
    crot = getJointConstraint(joint, 'LIMIT_ROTATION')
    crot.use_limit_x = True
    crot.min_x = 0
    crot.max_x = 0
    crot.use_limit_y = True
    crot.min_y = 0
    crot.max_y = 0
    crot.use_limit_z = True
    crot.min_z = 0
    crot.max_z = 0
    crot.owner_space = 'LOCAL'


def set_fixed(joint):
    """

    Args:
      joint: 

    Returns:

    """
    # fix location
    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
    cloc = getJointConstraint(joint, 'LIMIT_LOCATION')
    cloc.use_min_x = True
    cloc.use_min_y = True
    cloc.use_min_z = True
    cloc.use_max_x = True
    cloc.use_max_y = True
    cloc.use_max_z = True
    cloc.owner_space = 'LOCAL'
    # fix rotation
    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
    crot = getJointConstraint(joint, 'LIMIT_ROTATION')
    crot.use_limit_x = True
    crot.min_x = 0
    crot.max_x = 0
    crot.use_limit_y = True
    crot.min_y = 0
    crot.max_y = 0
    crot.use_limit_z = True
    crot.min_z = 0
    crot.max_z = 0
    crot.owner_space = 'LOCAL'


def set_planar(joint):
    """

    Args:
      joint: 

    Returns:

    """
    # fix location
    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
    cloc = getJointConstraint(joint, 'LIMIT_LOCATION')
    cloc.use_min_y = True
    cloc.use_max_y = True
    cloc.owner_space = 'LOCAL'
    # fix rotation
    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
    crot = getJointConstraint(joint, 'LIMIT_ROTATION')
    crot.use_limit_x = True
    crot.min_x = 0
    crot.max_x = 0
    crot.use_limit_y = True
    crot.min_y = 0
    crot.max_y = 0
    crot.use_limit_z = True
    crot.min_z = 0
    crot.max_z = 0
    crot.owner_space = 'LOCAL'


def set_ball(joint, limit):
    """

    Args:
      joint:
      limit:

    Returns:

    """
    # fix location
    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
    cloc = getJointConstraint(joint, 'LIMIT_LOCATION')
    cloc.use_min_x = True
    cloc.use_min_y = True
    cloc.use_min_z = True
    cloc.use_max_x = True
    cloc.use_max_y = True
    cloc.use_max_z = True
    cloc.owner_space = 'LOCAL'
    # fix rotation x, z
    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
    crot = getJointConstraint(joint, 'LIMIT_ROTATION')
    crot.use_limit_x = True
    crot.min_x = -limit
    crot.max_x = limit
    crot.use_limit_z = True
    crot.min_z = -limit
    crot.max_z = limit
    crot.owner_space = 'LOCAL'


def set_universal(joint, lower, upper, lower2, upper2):
    """

    Args:
      upper2:
      lower2:
      upper:
      lower:
      joint:

    Returns:

    """
    # fix location
    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
    cloc = getJointConstraint(joint, 'LIMIT_LOCATION')
    cloc.use_min_x = True
    cloc.use_min_y = True
    cloc.use_min_z = True
    cloc.use_max_x = True
    cloc.use_max_y = True
    cloc.use_max_z = True
    cloc.owner_space = 'LOCAL'
    # fix rotation x, z
    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
    crot = getJointConstraint(joint, 'LIMIT_ROTATION')
    crot.use_limit_x = True
    crot.min_x = lower
    crot.max_x = upper
    crot.use_limit_z = True
    crot.min_z = lower2
    crot.max_z = upper2
    crot.owner_space = 'LOCAL'


def remove_screwdrivers(joint):
    """
    Delete all drivers phobos created on a given joint
    Args:
        joint:

    Returns:

    """
    bone = joint.pose.bones[0]
    if joint.animation_data is not None and joint.animation_data.drivers is not None:
        for fcurve in joint.animation_data.drivers:
            if fcurve.data_path == f'pose.bones["{bone.name}"].rotation_euler':
                if fcurve.driver.variables[0].name.startswith("phobos"):
                    joint.animation_data.drivers.remove(fcurve)
                else:
                    # TODO This is not our driver, it could interfere with our drivers
                    pass


def set_screw(joint, lower, upper, axis, pitch):
    """

    Args:
      pitch: Meters traveled per revolution
      joint:
      lower:
      upper:
      axis:

    Returns:

    """

    # fix location except for y-axis
    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
    cloc = getJointConstraint(joint, 'LIMIT_LOCATION')
    cloc.use_min_x = True
    cloc.use_min_z = True
    cloc.use_max_x = True
    cloc.use_max_z = True
    if lower == upper:
        cloc.use_min_y = False
        cloc.use_max_y = False
    else:
        cloc.use_min_y = True
        cloc.use_max_y = True
        cloc.min_y = lower
        cloc.max_y = upper
    cloc.owner_space = 'LOCAL'
    # fix rotation
    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
    crot = getJointConstraint(joint, 'LIMIT_ROTATION')
    crot.use_limit_x = True
    crot.min_x = 0
    crot.max_x = 0
    crot.use_limit_y = False
    crot.min_y = 0
    crot.max_y = 0
    crot.use_limit_z = True
    crot.min_z = 0
    crot.max_z = 0
    crot.owner_space = 'LOCAL'

    bone = joint.pose.bones[0]
    bone.rotation_mode = 'XYZ'

    if axis:
        # add screwdriver
        axisName = ["x", "y", "z"]
        maxValue = 0
        maxIndex = 0
        for index, value in enumerate(axis):
            if value > maxValue:
                maxValue = value
                maxIndex = index
        rotationExpression = f"phobosvar * {2 * math.pi / (maxValue * pitch)}"
        fcurve = bone.driver_add("rotation_euler", 1)
        driver = fcurve.driver

        variable = driver.variables.new()
        variable.name = "phobosvar"
        variable.type = "TRANSFORMS"
        target = variable.targets[0]
        target.id = joint
        target.bone_target = "Bone"
        target.transform_type = f'LOC_{axisName[maxIndex].upper()}'

        driver.expression = rotationExpression


def set_gearbox(joint, axis, axis2, ratio):
    """

    Args:
      joint:

    Returns:

    """
    # fix location
    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
    cloc = getJointConstraint(joint, 'LIMIT_LOCATION')
    cloc.use_min_x = True
    cloc.use_min_y = True
    cloc.use_min_z = True
    cloc.use_max_x = True
    cloc.use_max_y = True
    cloc.use_max_z = True
    cloc.owner_space = 'LOCAL'
    # free rotation
    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
    crot = getJointConstraint(joint, 'LIMIT_ROTATION')
    crot.owner_space = 'LOCAL'

    bone = joint.pose.bones[0]
    bone.rotation_mode = 'XYZ'

    # TODO create gearbox drivers, the code below might help

    """
    
    if axis is not None:
        # add driver

        x, y, z = axis  # parent
        vx, vy, vz = "phobosX", "phobosY", "phobosZ"
        xreq, yreq, zreq = x != 0, y != 0, z != 0  # value required?
        xexp, yexp, zexp = f"{x}*{vx}", f"{y}*{vy}", f"{z}*{vz}"  # value expression

        exps = []  # added expressions
        if xreq:
            exps.append(xexp)
        if yreq:
            exps.append(yexp)
        if zreq:
            exps.append(zexp)
        exp = "+".join(exps)

        input = f"{ratio} * ({exp})"
        parent = sUtils.getEffectiveParent(joint)

        for index, value in enumerate(axis2):
            if value != 0:
                fcurve = bone.driver_add("rotation_euler", index)
                driver = fcurve.driver

                if xreq:
                    variable = driver.variables.new()
                    variable.name = "phobosX"
                    variable.type = "TRANSFORMS"
                    target = variable.targets[0]
                    target.id = parent
                    target.bone_target = "Bone"
                    target.transform_type = 'ROT_X'

                if yreq:
                    variable = driver.variables.new()
                    variable.name = "phobosY"
                    variable.type = "TRANSFORMS"
                    target = variable.targets[0]
                    target.id = parent
                    target.bone_target = "Bone"
                    target.transform_type = 'ROT_Y'

                if zreq:
                    variable = driver.variables.new()
                    variable.name = "phobosZ"
                    variable.type = "TRANSFORMS"
                    target = variable.targets[0]
                    target.id = parent
                    target.bone_target = "Bone"
                    target.transform_type = 'ROT_Z'

                driver.expression = f"{value} * {input}"
    """
