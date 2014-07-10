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
    ''' Derives the type of the joint defined by the armature object 'joint' based on the constraints defined in the joint.
    The parameter 'adjust' decides whether or not the type of the joint is adjusted after detecting (without checking whether
    the property "type" was previously defined in the armature or not).'''
    jtype = 'floating' # 'universal' in MARS nomenclature
    cloc = None
    crot = None
    limrot = None
    # we pick the first bone in the armature as there is only one
    for c in joint.pose.bones[0].constraints:
        if c.type == 'LIMIT_LOCATION':
            cloc = [c.use_min_x, c.use_max_x,
                    c.use_min_y, c.use_max_y,
                    c.use_min_z, c.use_max_z]
        elif c.type == 'LIMIT_ROTATION':
            limrot = c
            crot = [c.use_limit_x and (c.min_x != 0 or c.max_x != 0),
                    c.use_limit_y and (c.min_y != 0 or c.max_y != 0),
                    c.use_limit_z and (c.min_z != 0 or c.max_z != 0)]
    ncloc = sum(cloc) if cloc else None
    ncrot = sum((limrot.use_limit_x, limrot.use_limit_y, limrot.use_limit_z,)) if limrot else None
    if cloc: # = if there is any constraint at all, as all joints but floating ones have translation limits
        if ncloc == 6: # fixed or revolute
            if ncrot == 3:
                if sum(crot) > 0:
                    jtype = 'revolute'
                else:
                    jtype = 'fixed'
            elif ncrot == 2:
                jtype = 'continuous'
        elif ncloc == 4:
            jtype = 'prismatic'
        elif ncloc == 2:
            jtype = 'planar'
    if 'jointType' in joint and joint['jointType'] != jtype:
        warnings.warn("Type of joint "+joint.name+" does not match constraints!", Warning) #TODO: not sure if that is correct like that
    if(adjust):
        joint['jointType'] = jtype
        print("Changed type of joint'" + joint.name, 'to', jtype + "'.")
    return jtype, crot

def getJointConstraints(joint):
    ''' Returns the constraints defined in the joint as a combination of two lists, 'axis' and 'limits'.'''
    jt, crot = deriveJointType(joint, adjust = True)
    if jt not in ['floating', 'fixed']:
        if jt in ['revolute', 'continuous'] and crot:
            c = getJointConstraint(joint, 'LIMIT_ROTATION')
            #axis = (joint.matrix_local * -bpy.data.armatures[joint.name].bones[0].vector).normalized() #we cannot use joint for both as the first is a Blender 'Object', the second an 'Armature'
            #axis = (joint.matrix_local * -joint.data.bones[0].vector).normalized() #joint.data accesses the armature, thus the armature's name is not important anymore
            axis = joint.data.bones[0].vector.normalized()
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
            axis = None
            limits = None
            freeloc = [c.use_min_x and c.use_max_x and c.min_x == c.max_x,
                    c.use_min_y and c.Use_max_y and c.min_x == c.max_x,
                    c.use_min_z and c.Use_max_z and c.min_x == c.max_x]
            if jt == 'prismatic':
                if sum(freeloc) == 2:
                    axis = mathutils.Vector([int(not i) for i in freeloc])
                    if freeloc[0]:
                        limits = (c.min_x, c.max_x)
                    elif freeloc[1]:
                        limits = (c.min_y, c.max_y)
                    elif freeloc[2]:
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
        print(axis, limits)
        return axis, limits
    else:
        return None, None

def getJointConstraint(joint, ctype):
    con = None
    for c in joint.pose.bones[0].constraints:
        if c.type == ctype:
            con = c
    return con

#DEPRECATED
def createJoint(name, jtype, scale, location, rotation = (0, 0, 0)):
    r1 = 0.075
    r2 = 1.5
    d1 = 4.0
    d2 = 0.05

    if jtype == 'hinge':
        j1 = mtutility.createPrimitive(name+'_1', 'cylinder', (r1*scale, d1*scale), mtdefs.layerTypes["joint"], 'joint', location, rotation)
        j2 = mtutility.createPrimitive(name, 'cylinder', (r2*scale, d2*scale), mtdefs.layerTypes["joint"], 'joint', location, rotation)
        #arrows.select = True
        j1.select = True
        j2.select = True
        bpy.context.scene.objects.active = j1
        bpy.ops.object.join()
    elif jtype == 'linear':
        j1 = mtutility.createPrimitive(name+'_1', 'box', (d1*scale, d2*scale, d2*scale), mtdefs.layerTypes["joint"], 'joint', location, rotation)
    elif jtype == 'planar':
        j1 = mtutility.createPrimitive(name+'_1', 'box', (d1*scale, d1*scale, d2*scale), mtdefs.layerTypes["joint"], 'joint', location, rotation)
    j1['jointType'] = jtype
    j1.MARStype = 'joint'
    j1['anchor'] = 'node2'
    j1['node2'] = ''
    #now add joint orientation
    joint = bpy.ops.object.armature_add(location = bpy.context.scene.cursor_location)
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

#DEPRECATED
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

#DEPRECATED
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
    bl_idname = "object.define_joint_constraints"
    bl_label = "Adds Bone Constraints to the joint (link)"
    bl_options = {'REGISTER', 'UNDO'}

    joint_type = EnumProperty(
        name = 'joint_type',
        default = 'revolute',
        description = "type of the joint",
        items = mtdefs.jointtypes)

    lower = FloatProperty(
        name = "lower",
        default = 0.0,
        description = "lower constraint of the joint")

    upper = FloatProperty(
        name = "upper",
        default = 0.0,
        description = "upper constraint of the joint")

    def execute(self, context):
        lower = math.radians(self.lower)
        upper = math.radians(self.upper)
        for link in bpy.context.selected_objects:
            bpy.context.scene.objects.active = link
            bpy.ops.object.mode_set(mode='POSE')
            for c in link.pose.bones[0].constraints:
                link.pose.bones[0].constraints.remove(c)
            if link.MARStype == 'link':
                if self.joint_type == 'revolute':
                    # fix location
                    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
                    cloc = getJointConstraint(link, 'LIMIT_LOCATION')
                    cloc.use_min_x = True
                    cloc.use_min_y = True
                    cloc.use_min_z = True
                    cloc.use_max_x = True
                    cloc.use_max_y = True
                    cloc.use_max_z = True
                    cloc.owner_space = 'LOCAL'
                    # fix rotation x, z and limit z
                    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
                    crot = getJointConstraint(link, 'LIMIT_ROTATION')
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
                elif self.joint_type == 'continuous':
                    # fix location
                    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
                    cloc = getJointConstraint(link, 'LIMIT_LOCATION')
                    cloc.use_min_x = True
                    cloc.use_min_y = True
                    cloc.use_min_z = True
                    cloc.use_max_x = True
                    cloc.use_max_y = True
                    cloc.use_max_z = True
                    cloc.owner_space = 'LOCAL'
                    # fix rotation x, y
                    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
                    crot = getJointConstraint(link, 'LIMIT_ROTATION')
                    crot.use_limit_x = True
                    crot.min_x = 0
                    crot.max_x = 0
                    crot.use_limit_y = True
                    crot.min_y = 0
                    crot.max_y = 0
                    crot.owner_space = 'LOCAL'
                elif self.joint_type == 'prismatic':
                    # fix location except for x axis
                    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
                    cloc = getJointConstraint(link, 'LIMIT_LOCATION')
                    cloc.use_min_x = True
                    cloc.use_min_y = True
                    cloc.use_min_z = True
                    cloc.use_max_x = True
                    cloc.use_max_y = True
                    cloc.use_max_z = True
                    cloc.min_x = self.lower
                    cloc.max_x = self.upper
                    cloc.owner_space = 'LOCAL'
                    # fix rotation
                    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
                    crot = getJointConstraint(link, 'LIMIT_ROTATION')
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
                elif self.joint_type == 'fixed':
                    # fix location
                    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
                    cloc = getJointConstraint(link, 'LIMIT_LOCATION')
                    cloc.use_min_x = True
                    cloc.use_min_y = True
                    cloc.use_min_z = True
                    cloc.use_max_x = True
                    cloc.use_max_y = True
                    cloc.use_max_z = True
                    cloc.owner_space = 'LOCAL'
                    # fix rotation
                    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
                    crot = getJointConstraint(link, 'LIMIT_ROTATION')
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
                elif self.joint_type == 'floating':
                    # 6DOF
                    pass
                elif self.joint_type == 'planar':
                    # fix location
                    bpy.ops.pose.constraint_add(type='LIMIT_LOCATION')
                    cloc = getJointConstraint(link, 'LIMIT_LOCATION')
                    cloc.use_min_z = True
                    cloc.use_max_z = True
                    cloc.owner_space = 'LOCAL'
                    # fix rotation
                    bpy.ops.pose.constraint_add(type='LIMIT_ROTATION')
                    crot = getJointConstraint(link, 'LIMIT_ROTATION')
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
                    print("Unknown joint type, aborting")
                link['jointType'] = self.joint_type
        return{'FINISHED'}

#DEPRECATED
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
