#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.exporter
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadowski, Simon Reichel

Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

This file is part of Phobos, a Blender Add-On to edit robot models.

Phobos is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License
as published by the Free Software Foundation, either version 3
of the License, or (at your option) any later version.

Phobos is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Phobos.  If not, see <http://www.gnu.org/licenses/>.

File joints.py

Created on 7 Jan 2014
"""

import bpy
import mathutils
import phobos.defs as defs
from phobos.phoboslog import log
import phobos.utils.naming as nUtils
import phobos.utils.selection as sUtils
import phobos.utils.blender as bUtils


def createJoint(joint, linkobj=None):
    # TODO add some docstring
    # add joint information
    if not linkobj:
        linkobj = bpy.data.objects[joint['child']]  # TODO: Make this generic?
    if joint['name'] != linkobj.name:
        linkobj['joint/name'] = joint['name']
    # get hold of object
    bUtils.toggleLayer(defs.layerTypes['link'], True)
    sUtils.selectObjects([linkobj], clear=True, active=0)

    # set axis
    if 'axis' in joint:
        bpy.ops.object.mode_set(mode='EDIT')
        editbone = linkobj.data.edit_bones[0]
        # TODO delete me?
        #oldaxis = editbone.vector
        length = editbone.length
        axis = mathutils.Vector(tuple(joint['axis']))
        # TODO delete me?
        #oldaxis.cross(axis) # rotation axis
        editbone.tail = editbone.head + axis.normalized() * length

    # add constraints
    for param in ['effort', 'velocity']:
        try:
            if 'limits' in joint:
                linkobj['joint/max'+param] = joint['limits'][param]
        except KeyError:
            # TODO more details
            log("Key Error in adding joint constraints for joint", joint['name'])
    try:
        lower = joint['limits']['lower']
        upper = joint['limits']['upper']
    except KeyError:
        lower = 0.0
        upper = 0.0
    setJointConstraints(linkobj, joint['type'], lower, upper)
    for prop in joint:
        if prop.startswith('$'):
            for tag in joint[prop]:
                linkobj['joint/'+prop[1:]+'/'+tag] = joint[prop][tag]


def deriveJointType(joint, adjust=False):
    """ Derives the type of the joint defined by the armature object 'joint'
    based on the constraints defined in the joint. If the constraints do not
    match the specified joint type, a warning is logged. By using the adjust
    parameter it is possible to overwrite the joint type according to the
    specified joint constraints.

    :param joint: The joint you want to derive its type from.
    :type joint: bpy_types.Object
    :param adjust: Decides whether or not the type of the joint is corrected
    according to the constraints (overwriting the existing joint type)
    :type adjust: bool.
    :return: tuple(2) -- jtype, crot
    """
    # 'universal' in MARS nomenclature
    jtype = 'floating'
    cloc = None
    crot = None
    limrot = None
    # we pick the first bone in the armature as there is only one
    for c in joint.pose.bones[0].constraints:
        if c.type == 'LIMIT_LOCATION':
            cloc = [c.use_min_x and c.min_x == c.max_x,
                    c.use_min_y and c.min_y == c.max_y,
                    c.use_min_z and c.min_z == c.max_z]
        elif c.type == 'LIMIT_ROTATION':
            limrot = c
            crot = [c.use_limit_x and (c.min_x != 0 or c.max_x != 0),
                    c.use_limit_y and (c.min_y != 0 or c.max_y != 0),
                    c.use_limit_z and (c.min_z != 0 or c.max_z != 0)]
    ncloc = sum(cloc) if cloc else None
    ncrot = sum((limrot.use_limit_x, limrot.use_limit_y,
                 limrot.use_limit_z,)) if limrot else None
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

    # warn user if the constraints do not match the specified joint type
    try:
        if jtype != joint['joint/type']:
            log(("The specified joint type '{0}' at link '{1}' does not match " +
                "the required constraints (set to '{2}' instead).").format(
                    joint['joint/type'], joint['link/name'], jtype), "WARNING")
    except KeyError:
        log("No joint type specified for joint " + joint.name, "WARNING")

    if adjust:
        joint['joint/type'] = jtype
        log("Set type of joint '" + nUtils.getObjectName(joint) + "'to '" +
            jtype + "'.", "INFO")
    return jtype, crot


def getJointConstraints(joint):
    """ Returns the constraints defined in the joint as a combination of two lists, 'axis' and 'limits'.

    :param joint: The joint you want to get the constraints from.
    :type joint: bpy_types.Object
    :return: tuple -- containing the axis and limits lists.
    """
    jt, crot = deriveJointType(joint)
    axis = None
    limits = None
    if jt not in ['floating', 'fixed']:
        if jt in ['revolute', 'continuous'] and crot:
            c = getJointConstraint(joint, 'LIMIT_ROTATION')
            # TODO delete me?
            #we cannot use joint for both as the first is a Blender 'Object', the second an 'Armature'
            #axis = (joint.matrix_local * -bpy.data.armatures[joint.name].bones[0].vector).normalized()
            #joint.data accesses the armature, thus the armature's name is not important anymore
            #axis = (joint.matrix_local * -joint.data.bones[0].vector).normalized()
            axis = joint.data.bones[0].vector.normalized() #vector along axis of bone (Y axis of pose bone) in object space
            if crot[0]:
                limits = (c.min_x, c.max_x)
            elif crot[1]:
                limits = (c.min_y, c.max_y)
            elif crot[2]:
                limits = (c.min_z, c.max_z)
        else:
            c = getJointConstraint(joint, 'LIMIT_LOCATION')
            if not c:
                raise Exception("JointTypeError: under-defined constraints in joint ("+joint.name+").")
            freeloc = [c.use_min_x and c.use_max_x and c.min_x == c.max_x,
                       c.use_min_y and c.use_max_y and c.min_y == c.max_y,
                       c.use_min_z and c.use_max_z and c.min_z == c.max_z]
            if jt == 'prismatic':
                if sum(freeloc) == 2:
                    # TODO delete me?
                    #axis = mathutils.Vector([int(not i) for i in freeloc])
                    #vector along axis of bone (Y axis of pose bone) in obect space
                    axis = joint.data.bones[0].vector.normalized()
                    if not freeloc[0]:
                        limits = (c.min_x, c.max_x)
                    elif not freeloc[1]:
                        limits = (c.min_y, c.max_y)
                    elif not freeloc[2]:
                        limits = (c.min_z, c.max_z)
                else:
                    raise Exception("JointTypeError: under-defined constraints in joint ("+joint.name+").")
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
                    raise Exception("JointTypeError: under-defined constraints in joint ("+joint.name+").")
    return axis, limits


def getJointConstraint(joint, ctype):
    """This function gets the constraints out of a given joint.

    :param joint: The joint you want to extract the constraints from.
    :type joint: bpy_types.Object
    :param ctype: Specifies the constraint type you want to extract.
    """
    con = None
    for c in joint.pose.bones[0].constraints:
        if c.type == ctype:
            con = c
    return con


def setJointConstraints(joint, jointtype, lower=0.0, upper=0.0, spring=0.0, damping=0.0,
                        maxeffort_approximation=None, maxspeed_approximation=None):
    """This function sets the constraints for a given joint and jointtype.

    :param joint: The joint you want to set the constraints for.
    :type joint: bpy_types.Object
    :param jointtype: The joints type. its one of the following:
        - revolute
        - continuous
        - prismatic
        - fixed
        - floating
    :type jointtype: str
    :param lower: The constraints lower limit.
    :type lower: float
    :param upper: The constraints upper limit.
    :type upper:float
    """
    log("Processing joint: " + joint.name, 'DEBUG', 'setJointConstraints')
    bpy.ops.object.mode_set(mode='POSE')
    for c in joint.pose.bones[0].constraints:
        joint.pose.bones[0].constraints.remove(c)
    if joint.phobostype == 'link':
        # add spring & damping
        if jointtype in ['revolute', 'prismatic'] and (spring or damping):
            try:
                bpy.ops.rigidbody.constraint_add(type='GENERIC_SPRING')
                bpy.context.object.rigid_body_constraint.spring_stiffness_y = spring
                bpy.context.object.rigid_body_constraint.spring_damping_y = damping
            except RuntimeError:
                log("No Blender Rigid Body World present, only adding custom properties.", "ERROR")
            # we should make sure that the rigid body constraints gets changed
            # if the values below are changed manually by the user
            joint['joint/dynamics/springStiffness'] = spring
            joint['joint/dynamics/springDamping'] = damping
            joint['joint/dynamics/spring_const_constraint_axis1'] = spring  # FIXME: this is a hack
            joint['joint/dynamics/damping_const_constraint_axis1'] = damping  # FIXME: this is a hack, too
        # add constraints
        if jointtype == 'revolute':
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
        elif jointtype == 'continuous':
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
        elif jointtype == 'prismatic':
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
        elif jointtype == 'fixed':
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
        elif jointtype == 'floating':
            # 6DOF
            pass
        elif jointtype == 'planar':
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
        else:
            log("Unknown joint type for joint " + joint.name, "WARNING", "setJointConstraints")
        joint['joint/type'] = jointtype
        bpy.ops.object.mode_set(mode='OBJECT')

        # approximation functions for effort and speed
        if jointtype in ['revolute', 'continuous', 'prismatic']:
            try:
                if maxeffort_approximation:
                    joint["joint/maxeffort_approximation"] = maxeffort_approximation["function"]
                    joint["joint/maxeffort_coefficients"] = maxeffort_approximation["coefficients"]
                if maxspeed_approximation:
                    joint["joint/maxspeed_approximation"] = maxspeed_approximation["function"]
                    joint["joint/maxspeed_coefficients"] = maxspeed_approximation["coefficients"]
            except KeyError:
                log("Approximation for max effort and/or speed ill-defined in joint object " + joint.name,
                    "ERROR", "setJointConstraints")
