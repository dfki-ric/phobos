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
import marstools.mtmaterials as mtmaterials
import marstools.mtutility as mtutility
import marstools.mtdefs as mtdefs

def register():
    print("Registering mtjoints...")


def unregister():
    print("Unregistering mtjoints...")

def createJoint(name, jtype, scale, location, rotation = (0, 0, 0)):
    r1 = 0.075
    r2 = 1.5
    d1 = 4.0
    d2 = 0.05

#    bpy.ops.mesh.primitive_cone_add(type='ARROWS')
    if jtype == 'hinge':
        mtutility.createPrimitive(name+'_1', 'cylinder', (r1*scale, d1*scale), mtdefs.layerTypes["joints"], 'joint', location, rotation)
        j1 = bpy.context.object
        mtutility.createPrimitive(name, 'cylinder', (r2*scale, d2*scale), mtdefs.layerTypes["joints"], 'joint', location, rotation)
        j2 = bpy.context.object
        j1.select = True
        j2.select = True
        bpy.context.scene.objects.active = j1
        bpy.ops.object.join()
    elif jtype == 'linear':
        mtutility.createPrimitive(name+'_1', 'box', (d1*scale, d2*scale, d2*scale), mtdefs.layerTypes["joints"], 'joint', location, rotation)
        j1 = bpy.context.object
    elif jtype == 'planar':
        mtutility.createPrimitive(name+'_1', 'box', (d1*scale, d1*scale, d2*scale), mtdefs.layerTypes["joints"], 'joint', location, rotation)
        j1 = bpy.context.object
    #TODO: set correct default values
    #j1.name = joint['name']
    j1['jointType'] = jtype
    j1.MARStype = 'joint'
    j1['anchor'] = 'node2'
    j1['node2'] = ''
    #TODO: make arrows (=coord systems) work for joints to show at least the orientation
    #arrows = bpy.ops.object.empty_add(type='ARROWS')
    #bpy.ops.transform.resize(value = (d1*scale, d1*scale, d1*scale)
    #bpy.context.scene.objects.active = j1
    #bpy.ops.group.create(name = j1.name)

# def createHingeJoint(name):
#
#     revolute - a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
# continuous - a continuous hinge joint that rotates around the axis and has no upper and lower limits
#
# planar - This joint allows motion in a plane perpendicular to the axis.
# prismatic - a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
#
# fixed - This is not really a joint because it cannot move. All degrees of freedom are locked. This type of joint does not require the axis, calibration, dynamics, limits or safety_controller.
# floating - This joint allows motion for all 6 degrees of freedom.


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
                 ('planar', 'planar', 'planar')])

    def execute(self, context):

        bpy.data.worlds[0].showJoints = True
        nodes = []
        for obj in bpy.context.selected_objects:
            if obj.MARStype == "body":
                nodes.append(obj)
                obj.select = False

        if nodes == []:
            bpy.ops.error.message('INVOKE_DEFAULT', type="AddJoints Error", message="Not enough objects of MARStype 'body' selected.")
            return{'CANCELLED'}
        if len(nodes) < 2:
            node = nodes[0]
            location = node.location
            createJoint('joint_' + node.name + '_world', self.joint_type, self.joint_scale, location)
            joint = bpy.context.object
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
                    createJoint('joint_' + node.parent.name + '_' + node.name, self.joint_type, self.joint_scale, location)
                    joint = bpy.context.object #TODO: check if this really refers to active object and not "bpy.context.scene.objects.active"
                    joint.parent = node.parent
                    joint['node2'] = node.name
                    for obj in bpy.context.selected_objects:
                        obj.select = False
            if not parent:
                bpy.ops.error.message('INVOKE_DEFAULT', type="AddJoints Error", message="None of the selected objects are related as parent-child.")
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
                # create ball
                bpy.ops.mesh.primitive_uv_sphere_add(size = 0.01,
                                              layers = js_layers)
                #                              location = joint.location)
                #                              rotation = joint.rotation_euler)
                ball = bpy.context.object
                ball.name = 'joint_sphere_' + joint.name
                ball.data.materials.append(mtmaterials.materials['joint_sphere'])
                ball.parent = joint.parent
                ball.select = True
                bpy.context.scene.objects.active = joint.parent
                bpy.ops.object.parent_set() #makes active object parent of selected object
                ball.location = joint.location
                ball.rotation_euler = joint.rotation_euler

                #modify limb
                print(joint.name)
                print("    ", joint['node2'])
                print("")
                limb = bpy.context.scene.objects[joint['node2']]
                limb.select = True
                bpy.context.scene.objects.active = ball
                bpy.ops.object.parent_set()

                #modify joint
                joint['anchor'] = "node2"
                joint['node2'] = ball.name
                ball["coll_bitmask"] = 0
                ball["type"] = "body"
                ball["mass"] = 0
        return{'FINISHED'}
