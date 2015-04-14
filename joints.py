#!/usr/bin/python

"""
.. module:: phobos.exporter
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadowski

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
from bpy.types import Operator
from bpy.props import FloatProperty, EnumProperty, BoolProperty
import math
import mathutils
import warnings
from . import defs
from phobos.logging import *


def register():
    """
    This function registers this module.
    At the moment it does nothing.

    :return: Nothing

    """
    print("Registering joints...")


def unregister():
    """
    This function unregisters this module.
    At the moment it does nothing.

    :return: Nothing

    """
    print("Unregistering joints...")


def deriveJointType(joint, adjust=False):
    """ Derives the type of the joint defined by the armature object 'joint' based on the constraints defined in the joint.

    :param joint: The joint you want to derive its type from.
    :type joint: blender object.
    :param adjust: Decides whether or not the type of the joint is adjusted after detecting (without checking whether
    the property "type" was previously defined in the armature or not).
    :type adjust: bool.
    :return: tuple(2) -- jtype, crot

    """
    jtype = 'floating'  # 'universal' in MARS nomenclature
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
    ncrot = sum((limrot.use_limit_x, limrot.use_limit_y, limrot.use_limit_z,)) if limrot else None
    if cloc:  # = if there is any constraint at all, as all joints but floating ones have translation limits
        if ncloc == 3:  # fixed or revolute
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
    if 'joint/type' in joint and joint['joint/type'] != jtype:
        warnings.warn("Type of joint "+joint.name+" does not match constraints!", Warning) #TODO: this goes to sys.stderr, i.e. nirvana
        if(adjust):
            joint['joint/type'] = jtype
            print("Changed type of joint'" + joint.name, 'to', jtype + "'.")
    return jtype, crot


def getJointConstraints(joint):
    """ Returns the constraints defined in the joint as a combination of two lists, 'axis' and 'limits'.

    :param joint: The joint you want to get the constraints from.
    :type joint: blender object.
    :return: tuple -- containing the axis and limits lists.

    """
    jt, crot = deriveJointType(joint)
    axis = None
    limits = None
    if jt not in ['floating', 'fixed']:
        if jt in ['revolute', 'continuous'] and crot:
            c = getJointConstraint(joint, 'LIMIT_ROTATION')
            #axis = (joint.matrix_local * -bpy.data.armatures[joint.name].bones[0].vector).normalized() #we cannot use joint for both as the first is a Blender 'Object', the second an 'Armature'
            #axis = (joint.matrix_local * -joint.data.bones[0].vector).normalized() #joint.data accesses the armature, thus the armature's name is not important anymore
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
                    #axis = mathutils.Vector([int(not i) for i in freeloc])
                    axis = joint.data.bones[0].vector.normalized() #vector along axis of bone (Y axis of pose bone) in obect space
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
    :param ctype: Specifies the constraint type you want to extract.
    :return: blender constraints object.

    """
    con = None
    for c in joint.pose.bones[0].constraints:
        if c.type == ctype:
            con = c
    return con


def setJointConstraints(joint, jointtype, lower=0.0, upper=0.0, spring=0.0, damping=0.0):
    """This function sets the constraints for a given joint and jointtype.

    :param joint: The joint you want to set the constraints for.
    :type joint: blender object.
    :param jointtype: The joints type. its one of the following:
        - revolute
        - continuous
        - prismatic
        - fixed
        - floating
    :type jointtype: string.
    :param lower: The constraints lower limit.
    :type lower: float.
    :param upper: The constraints upper limit.
    :type upper:float.
    :return: Nothing.

    """
    bpy.ops.object.mode_set(mode='POSE')
    for c in joint.pose.bones[0].constraints:
        joint.pose.bones[0].constraints.remove(c)
    if joint.phobostype == 'link':
        # add spring & damping
        if jointtype in ['revolute', 'prismatic'] and (spring or damping):
            bpy.ops.rigidbody.constraint_add(type='GENERIC_SPRING')
            bpy.context.object.rigid_body_constraint.spring_stiffness_y = spring
            bpy.context.object.rigid_body_constraint.spring_damping_y = damping
            # we should make sure that the rigid body constraints gets changed
            # if the values below are changed manually by the user
            joint['joint/springStiffness'] = spring
            joint['joint/springDamping'] = damping
            joint['joint/spring_const_constraint_axis1'] = spring  # FIXME: this is a hack
            joint['joint/damping_const_constraint_axis1'] = damping  # FIXME: this is a hack, too
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
            log("Unknown joint type", "ERROR")
        joint['joint/type'] = jointtype
        bpy.ops.object.mode_set(mode='OBJECT')


class DefineJointConstraintsOperator(Operator):
    """DefineJointConstraintsOperator

    """
    bl_idname = "object.define_joint_constraints"
    bl_label = "Adds Bone Constraints to the joint (link)"
    bl_options = {'REGISTER', 'UNDO'}

    passive = BoolProperty(
        name='passive',
        default=False,
        description='makes the joint passive (no actuation)'
    )

    useRadian = BoolProperty(
        name='useRadian',
        default=True,
        description='use degrees or rad for joints'
    )

    joint_type = EnumProperty(
        name='joint_type',
        default='revolute',
        description="type of the joint",
        items=defs.jointtypes)

    lower = FloatProperty(
        name="lower",
        default=0.0,
        description="lower constraint of the joint")

    upper = FloatProperty(
        name="upper",
        default=0.0,
        description="upper constraint of the joint")

    maxeffort = FloatProperty(
        name="max effort (N or Nm)",
        default=0.0,
        description="maximum effort of the joint")

    maxvelocity = FloatProperty(
        name="max velocity (m/s or rad/s)",
        default=0.0,
        description="maximum velocity of the joint. If you uncheck radian, you can enter °/sec here")

    spring = FloatProperty(
        name="spring constant",
        default=0.0,
        description="spring constant of the joint")

    damping = FloatProperty(
        name="damping constant",
        default=0.0,
        description="damping constant of the joint")

    # TODO: invoke function to read all values in

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "joint_type", text="joint_type")
        layout.prop(self, "passive", text="makes the joint passive (no actuation)")
        layout.prop(self, "useRadian", text="use radian")
        if self.joint_type != 'fixed':
            layout.prop(self, "maxeffort", text="max effort [" + ('Nm]' if self.joint_type in ['revolute', 'continuous'] else 'N]'))
            if self.joint_type in ['revolute', 'continuous']:
                layout.prop(self, "maxvelocity", text="max velocity [" + ("rad/s]" if self.useRadian else "°/s]"))
            else:
                layout.prop(self, "maxvelocity", text="max velocity [m/s]")
        if self.joint_type in ('revolute', 'prismatic'):
            layout.prop(self, "lower", text="lower [rad]" if self.useRadian else "lower [°]")
            layout.prop(self, "upper", text="upper [rad]" if self.useRadian else "upper [°]")
            layout.prop(self, "spring", text="spring constant [N/m]")
            layout.prop(self, "damping", text="damping constant")


    def invoke(self, context, event):
        aObject = context.active_object
        if 'joint/type' not in aObject and 'motor/type' in aObject:
            self.maxvelocity = aObject['motor/maxSpeed']
            self.maxeffort = aObject['motor/maxEffort']
        return self.execute(context)

    def execute(self, context):
        """This function executes this operator and sets the constraints and joint type for all selected links.
        rad/s is the default unit. rpm will be transformed into rad/s

        :param context: The blender context this operator works with.
        :return: Blender result.

        """
        lower=0
        upper=0
        velocity=0
        if self.joint_type in ('revolute', 'prismatic'):
            if not self.useRadian:
                lower = math.radians(self.lower)
                upper = math.radians(self.upper)
            else:
                lower = self.lower
                upper = self.upper
        if not self.useRadian:
            velocity = self.maxvelocity * ((2*math.pi) / 360) # from °/s to rad/s
        else:
            velocity = self.maxvelocity
        for link in context.selected_objects:
            bpy.context.scene.objects.active = link
            setJointConstraints(link, self.joint_type, lower, upper, self.spring, self.damping)
            if self.joint_type != 'fixed':
                link['joint/maxeffort'] = self.maxeffort
                link['joint/maxvelocity'] = velocity
            else:
                if "joint/maxeffort" in link: del link["joint/maxeffort"]
                if "joint/maxvelocity" in link: del link["joint/maxvelocity"]
            if self.passive:
                link['joint/passive'] = "$true"
            else:
                pass  # FIXME: add default motor here or upon export?
                      # upon export might have the advantage of being able
                      # to check for missing motors in the model checking
        return{'FINISHED'}


class AttachMotorOperator(Operator):
    """AttachMotorOperator

    """
    bl_idname = "object.attach_motor"
    bl_label = "Attaches motor values to selected joints"
    bl_options = {'REGISTER', 'UNDO'}

    P = FloatProperty(
        name="P",
        default=1.0,
        description="P-value")

    I = FloatProperty(
        name="I",
        default=0.0,
        description="I-value")

    D = FloatProperty(
        name="D",
        default=0.0,
        description="D-value")

    vmax = FloatProperty(
        name="maximum velocity [m/s] or [rad/s]",
        default=1.0,
        description="maximum turning velocity of the motor")

    taumax = FloatProperty(
        name="maximum torque [Nm]",
        default=1.0,
        description="maximum torque a motor can apply")

    motortype = EnumProperty(
        name='motor_type',
        default='PID',
        description="type of the motor",
        items=defs.motortypes)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "motortype", text="motor_type")
        layout.prop(self, "taumax", text="maximum torque [Nm]")
        layout.prop(self, "vmax", text="maximum velocity [m/s] or [rad/s]")
        if self.motortype == 'PID':
            layout.prop(self, "P", text="P")
            layout.prop(self, "I", text="I")
            layout.prop(self, "D", text="D")

    def invoke(self, context, event):
        aObject = context.active_object
        if 'motor/type' not in aObject and 'joint/type' in aObject and aObject['joint/type'] != 'fixed':
            self.taumax = aObject['joint/maxeffort']
            self.vmax = aObject['joint/maxvelocity']
        return self.execute(context)

    def execute(self, context):
        """This function executes this operator and attaches a motor to all selected links.

        :param context: The blender context this operator works with.
        :return:Blender result.

        """
        for joint in bpy.context.selected_objects:
            if joint.phobostype == "link":
                #TODO: these keys have to be adapted
                if self.motortype == 'PID':
                    joint['motor/p'] = self.P
                    joint['motor/i'] = self.I
                    joint['motor/d'] = self.D
                joint['motor/maxSpeed'] = self.vmax
                joint['motor/maxEffort'] = self.taumax
                #joint['motor/type'] = 'PID' if self.motortype == 'PID' else 'DC'
                joint['motor/type'] = self.motortype
        return{'FINISHED'}
