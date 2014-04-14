'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtjoints.py

Created on 7 Jan 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
from bpy.types import Operator
from bpy.props import FloatProperty, EnumProperty
import math
import mathutils
import warnings
import marstools.mtmaterials as mtmaterials
import marstools.mtutility as mtutility
import marstools.mtdefs as mtdefs

def register():
    print("Registering mtjoints...")


def unregister():
    print("Unregistering mtjoints...")

def deriveJointType(joint, adjust = False):
    jtype = 'floating' # 'universal' in MARS nomenclature
    cloc, crot = None
    for c in joint.pose.bones[0].constraints:
        if c.type == 'LIMIT_LOCATION':
            cloc = [c.use_min_x, c.use_max_x,
                    c.use_min_y, c.Use_max_y,
                    c.use_min_z, c.Use_max_z]
        elif c.type == 'LIMIT_ROTATION':
            crot = [c.use_lim_x, c.use_lim_y, c.use_lim_z]
    if crot:
        if sum(crot) < 3: #continuous or revolute
            if ((c.use_lim_x and (c.min_x or c.max_x))
                or (c.use_lim_y and (c.min_y or c.max_y))
                or (c.use_lim_z and (c.min_z or c.max_z))):
                jtype = 'revolute'
            else:
                jtype = 'continuous'
        else: # prismatic, planar or fixed
            if sum(cloc) >= 4:
                if sum(cloc) == 6:
                    jtype = 'fixed'
                else:
                    jtype = 'prismatic'
            else:
                jtype = 'planar'
    if 'jointType' in joint and joint['jointType'] != jtype:
        warnings.warn("Type of joint "+joint.name+" does not match constraints!", Warning) #TODO: not sure if that is correct like that
        if(adjust):
            joint['jointType'] = jtype
            print("Changed type to '"+jtype+"'.")
    return jtype, crot

def getJointConstraints(joint):
    if "jointType" not in joint:
        warnings.warn("Type of joint"+joint.name+"undefined. Trying to auto-assign joint type.")
        jt, crot = deriveJointType(joint, adjust = True)
    else:
        jt = joint['jointType']
    if jt not in ['floating', 'fixed']:
        if jt in ['revolute', 'continuous'] and crot:
            c = getJointConstraint(joint, 'LIMIT_ROTATION')
            axis = mathutils.Vector([int(not i) for i in crot])
            if crot[0]:
                limits = (c.min_x, c.max_x)
            elif crot[1]:
                limits = (c.min_y, c.max_y)
            elif crot[2]:
                limits = (c.min_z, c.max_z)
        else:
            c = getJointConstraint(joint, 'LIMIT_LOCATION')
            axis, limits = None
            freeloc = [c.use_min_x and c.use_max_x and c.min_x == c.max_x,
                    c.use_min_y and c.Use_max_y and c.min_x == c.max_x,
                    c.use_min_z and c.Use_max_z and c.min_x == c.max_x]
            if jt == 'prismatic':
                if sum(freeloc) == 2:
                    axis = mathutils.Vector([int(not i) for i in freeloc])
                    if freeloc[0]:
                        limits = (c.min_x, c.max_x)
                    elif freeloc[1]:
                        limits = (c.min_x, c.max_x)
                    elif freeloc[2]:
                        limits = (c.min_x, c.max_x)
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
    else:
        return None, None

def getJointConstraint(joint, ctype):
    con = None
    for c in joint.pose.bones[0].constraints:
        if c.type == ctype:
            con = c
    return con

def createJoint(name, jtype, scale, location, rotation = (0, 0, 0)):
    r1 = 0.075
    r2 = 1.5
    d1 = 4.0
    d2 = 0.05

    if jtype == 'hinge':
        j1 = mtutility.createPrimitive(name+'_1', 'cylinder', (r1*scale, d1*scale), mtdefs.layerTypes["joints"], 'joint', location, rotation)
        j2 = mtutility.createPrimitive(name, 'cylinder', (r2*scale, d2*scale), mtdefs.layerTypes["joints"], 'joint', location, rotation)
        #arrows.select = True
        j1.select = True
        j2.select = True
        bpy.context.scene.objects.active = j1
        bpy.ops.object.join()
    elif jtype == 'linear':
        j1 = mtutility.createPrimitive(name+'_1', 'box', (d1*scale, d2*scale, d2*scale), mtdefs.layerTypes["joints"], 'joint', location, rotation)
    elif jtype == 'planar':
        j1 = mtutility.createPrimitive(name+'_1', 'box', (d1*scale, d1*scale, d2*scale), mtdefs.layerTypes["joints"], 'joint', location, rotation)
    j1['jointType'] = jtype
    j1.MARStype = 'joint'
    j1['anchor'] = 'node2'
    j1['node2'] = ''
    #now add joint orientation
    bpy.ops.object.empty_add(type='ARROWS', location = j1.location, rotation = j1.rotation_euler)
    arrows = bpy.context.object
    dx = d1*0.7
    bpy.ops.transform.resize(value = (dx*scale, dx*scale, dx*scale))
    arrows.name = "axes_" + j1.name
    j1.select = True
    arrows.select = True
    bpy.context.scene.objects.active = j1
    bpy.ops.object.parent_set()
    return j1

def createJointSphere(joint, psize):
    #create the joint sphere / joint ball
    ball = mtutility.createPrimitive('joint_sphere_' + joint.name,
                                     'sphere',
                                     [psize],
                                     mtdefs.layerTypes['jointSpheres'],
                                     'joint_sphere',
                                     joint.location,
                                     joint.rotation_euler)
    #TODO: Why does it work here, but not below? It might be because I gave an absolute location to the create_primitive
    # Blender function and I did not, as in the commented code below, leave it out and set the location later...?
    ball.parent = joint.parent
    #TODO: the following lines should be rendered unnecessary by the above
    #ball.select = True
    #bpy.context.scene.objects.active = joint.parent
    #bpy.ops.object.parent_set() #makes active object parent of selected object
    ball['coll_bitmask'] = 0
    ball.MARStype = "body"
    ball['mass'] = 0
    #adapt the child limb
    limb = bpy.context.scene.objects[joint['node2']] #TODO: this is an elegant trick that should be used everywhere, maybe even bpy.data.objects[name]
    #limb.parent = ball
    limb.select = True
    bpy.context.scene.objects.active = ball
    bpy.ops.object.parent_set()
    #adapt the joint
    joint['node2'] = ball.name
    #TODO: the following lines should not be necessary, either (see problem above)
    #ball.location = joint.location
    #ball.rotation_euler = joint.rotation_euler


class AddJointsOperator(Operator):
    """Select n bodies (lowest child to overall parent, parent = active object) to be connected via newly-created joints."""
    bl_idname = "object.add_joints"
    bl_label = "Adds Joints between all selected Nodes (or World if only one Node is selected)"
    bl_options = {'REGISTER', 'UNDO'}

    joint_scale = FloatProperty(
        name = "joint_scale",
        default = 0.1,
        description = "scale of the joint arbor")

    joint_type = EnumProperty(
        name = "joint_type",
        default = "hinge",
        description = "type of the joint",
        items = [('hinge', 'hinge', 'hinge'),
                 ('linear', 'linear', 'linear'),
                 ('planar', 'planar', 'planar')]) #TODO: move this to mtdefs?

    def execute(self, context):

        bpy.data.worlds[0].showJoints = True
        nodes = []
        for obj in bpy.context.selected_objects:
            if obj.MARStype == "link":
                nodes.append(obj)
                obj.select = False

        if nodes == []:
            bpy.ops.error.message('INVOKE_DEFAULT', type="AddJoints Error", message="Not enough bodies selected.")
            return{'CANCELLED'}
        if len(nodes) < 2:
            node = nodes[0]
            location = node.location
            createJoint('joint_' + node.name + '_world', self.joint_type, self.joint_scale, location)
            joint = bpy.context.object #TODO: check if this really refers to active object and not "bpy.context.scene.objects.active"
            joint.parent = node
            joint['node2'] = 'world'
        else:
            parent = False
            for node in nodes:
                if node.parent in nodes:
                    parent = True
                    node.select = True
                    node.parent.select = True
                    bpy.ops.view3d.snap_cursor_to_selected()
                    #calculate relative location
                    location = bpy.context.scene.cursor_location - node.parent.location
                    joint = createJoint('joint_' + node.parent.name + '_' + node.name, self.joint_type, self.joint_scale, location)
                    joint.parent = node.parent #TODO: this might be the reason that we have to compute a relative location above,
                    #the other solution using the set_parent() operator might have prevented that problem
                    joint['node2'] = node.name
                    for obj in bpy.context.selected_objects:
                        obj.select = False
                    createJointSphere(joint, self.joint_scale/5.0)
            if not parent:
                bpy.ops.error.message('INVOKE_DEFAULT', type="AddJoints Error", message="Missing parent-child connection.")
        return{'FINISHED'}


class DefineJointConstraintsOperator(Operator):
    """DefineJointConstraintsOperator"""
    bl_idname = "object.define_joint_constraints_spheres"
    bl_label = "Creates Joint Helper Objects for all Joints"
    bl_options = {'REGISTER', 'UNDO'}

    lower = FloatProperty(
        name = "lower",
        default = 0.0,
        description = "lower constraint of the joint")

    upper = FloatProperty(
        name = "upper",
        default = 0.0,
        description = "upper constraint of the joint")

    def execute(self, context):
        for joint in bpy.context.selected_objects:
            if joint.MARStype == "joint":
                if joint['jointType'] == 'hinge':
                    joint["lowerConstraint"] = (self.lower / 180) * math.pi
                    joint["upperConstraint"] = (self.upper / 180) * math.pi
                else:
                    joint["lowerConstraint"] = self.lower
                    joint["upperConstraint"] = self.upper
        return{'FINISHED'}


class DeriveJointSpheresOperator(Operator):
    """DeriveJointSpheresOperator"""
    bl_idname = "object.derive_joint_spheres"
    bl_label = "Creates Joint Helper Objects for all Joints"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):

        js_layers = 20*[False]
        js_layers[2] = True
        for joint in bpy.context.selected_objects:
            if joint.MARStype == "joint":
                createJointSphere(joint, self.joint_scale/5.0)
        return{'FINISHED'}
