#!/usr/bin/python

"""
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

File importer.py

Created on 28 Feb 2014

@author: Kai von Szadkowski
"""

import bpy
import mathutils
import os
import yaml
from collections import namedtuple
import xml.etree.ElementTree as ET
from phobos.utility import *
from . import defs
from . import materials

import phobos.bobj_import as bobj_import
from . import joints
from . import sensors
from . import controllers
from phobos.logging import *

#This is a really nice pythonic approach to creating a list of constants
Defaults = namedtuple('Defaults', ['mass', 'idtransform'])
defaults = Defaults(0.001, #mass
                    [0.0, 0.0, 0.0]  # idtransform
                    )

def register():
    print("Registering importer...")

def unregister():
    print("Unregistering importer...")

def cleanUpScene():
    # select all objects
    bpy.ops.object.select_all(action="SELECT")

    # and delete them
    bpy.ops.object.delete()

    # after that we have to clean up all loaded meshes (unfortunately
    # this is not done automatically)
    for mesh in bpy.data.meshes:
        bpy.data.meshes.remove(mesh)

    # and all materials
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)

    # and all lights (aka lamps)
    for lamp in bpy.data.lamps:
        bpy.data.lamps.remove(lamp)

def round_float(float_as_str, decimal=6):
    '''
    Cast 'float_as_str' to float and round to 'decimal' decimal places.
    '''
    return round(float(float_as_str), decimal)
    
def pos_rot_tree_to_lists(position, rotation):
    '''
    Convert the xml representations of a position and a rotation to
    lists. If either is 'None', return a list of zeroes instead.
    '''
    if position:
        px = round_float(position.find('x').text)
        py = round_float(position.find('y').text)
        pz = round_float(position.find('z').text)
    else:
        px, py, pz = (0, 0, 0)
    if rotation:
        rw = round_float(rotation.find('w').text)
        rx = round_float(rotation.find('x').text)
        ry = round_float(rotation.find('y').text)
        rz = round_float(rotation.find('z').text)
    else:
        rw, rx, ry, rz = (0, 0, 0, 0)
        
    return [px, py, pz], [rw, rx, ry, rz]
        
def calc_pose_formats(position, rotation, pivot=[0,0,0]):
    '''
    Create a dictionary containing various representations of the pose
    represented by 'position' and 'rotation':
        - translation == position
        - rotation_quaternion ==n rotation
        - rotation_euler: euler angles
        - matrix: position and rotation in 4x4 matrix form
    '''
    px, py, pz = position
    if len(rotation) == 3:
        rot = mathutils.Euler(rotation).to_quaternion()
        #print(rotation)
    else:
        rot = mathutils.Quaternion(rotation)

    #if angle_offset is not 0.0:
    #    axis_vec = mathutils.Vector(axis)

    #    offset_matrix = mathutils.Matrix.Rotation(angle_offset, 4, axis_vec) #get_axis_rotation_matrix(axis, angle_offset)
    #    rot_matrix = rot.to_matrix().to_4x4()
    #    applied_matrix = rot_matrix * offset_matrix
    #    rot = applied_matrix.to_quaternion()

    rw, rx, ry, rz = rot
    pose_dict = {}

    neg_pivot_translation = mathutils.Matrix.Translation((-pivot[0], -pivot[1], -pivot[2]))
    pivot_translation = mathutils.Matrix.Translation(pivot)
    rotation_matrix = mathutils.Quaternion(rot).to_matrix().to_4x4()
    translation = mathutils.Matrix.Translation(position)
    print()
    print("translation:", translation)
    print("neg_pivot_translation:", neg_pivot_translation)
    print("rotation_matrix:", rotation_matrix)
    print("pivot_translation", pivot_translation)
    print()
    transformation_matrix = translation * neg_pivot_translation * rotation_matrix * pivot_translation

    rm = transformation_matrix
    #TODO: this is not good
    matrix = [[rm[0][0], rm[0][1], rm[0][2], rm[0][3]],
              [rm[1][0], rm[1][1], rm[1][2], rm[1][3]],
              [rm[2][0], rm[2][1], rm[2][2], rm[2][3]],
              [rm[3][0], rm[3][1], rm[3][2], rm[3][3]]]
    pose_dict['matrix'] = matrix
    loc, rot, sca = transformation_matrix.decompose()
    pose_dict['translation'] = [loc.x, loc.y, loc.z]
    pose_dict['rotation_quaternion'] = [rot.w, rot.x, rot.y, rot.z]
    euler = rot.to_euler()
    pose_dict['rotation_euler'] = [euler.x, euler.y, euler.z]

    #translation = [px, py, pz]
    #quaternion = mathutils.Quaternion([rw, rx, ry, rz])
    #euler = quaternion.to_euler()
    ##print(euler)
    #pose_dict['translation'] = translation
    ##pose_dict['rotation_quaternion'] = [rw, rx, ry, rz]
    #pose_dict['rotation_euler'] = [euler.x, euler.y, euler.z]
    #rm = quaternion.to_matrix()
    #matrix = [[rm[0][0], rm[0][1], rm[0][2], px],
    #          [rm[1][0], rm[1][1], rm[1][2], py],
    #          [rm[2][0], rm[2][1], rm[2][2], pz],
    #          [0.0,      0.0,      0.0,      1.0]]
   #
   # pose_dict['matrix'] = matrix

    print()
    print('pose_dict:', pose_dict)
    print()

    return pose_dict
    
def add_quaternion(rot1, rot2):
    '''
    Add two rotations in quaternion format.
    '''
    quat1 = mathutils.Quaternion(rot1)
    quat2 = mathutils.Quaternion(rot2)
    quat_sum = quat1 * quat2
    return (quat_sum.w, quat_sum.x, quat_sum.y, quat_sum.z)
    
    
def handle_missing_geometry(no_visual_geo, no_collision_geo, link_dict):
    '''
    Handle missing visual and collision geometry.
    I hope it was meant like that ...
    '''
    if no_visual_geo or no_collision_geo:
        print("\n### WARNING: Missing geometry information in", link_dict['name'], ".")
    if no_visual_geo:
        print('Affected visual elements are:', no_visual_geo, '.\n\
                Trying to get missing information from collision elements.')
        for visual in no_visual_geo:
            try:
                link_dict['visual'][visual]['geometry'] = link_dict['collision']['collision' + no_visual_geo[len('visual'):]]['geometry']
            except:
                pass # TODO: print something?
    elif no_collision_geo:
        print('Affected collision elements are:', no_visual_geo, '.\n\
                Trying to get missing information from visual elements.')
        for collision in no_collision_geo:
            try:
                link_dict['collision'][collision]['geometry'] = link_dict['visual']['visual' + no_collision_geo[len('collision'):]]['geometry']
            except:
                pass # TODO: print something?


# def get_axis_rotation_matrix(axis, angle):
#     '''
#     unnecessary
#     '''
#
#     import math
#
#     u, v, w = axis
#     u_sqr = u**2
#     v_sqr = v**2
#     w_sqr = w**2
#     sin_theta = math.sin(angle)
#     cos_theta = math.cos(angle)
#
#     a11 = u_sqr + (v_sqr + w_sqr) * cos_theta
#     a12 = u * v * (1 - cos_theta) - w * sin_theta
#     a13 = u * w * (1 - cos_theta) + v * sin_theta
#     a14 = 0.0
#     a21 = u * v * (1 - cos_theta) + w * sin_theta
#     a22 = v_sqr + (u_sqr + w_sqr) * cos_theta
#     a23 = v * w * (1 - cos_theta) - u * sin_theta
#     a24 = 0.0
#     a31 = u * w * (1 - cos_theta) - v * sin_theta
#     a32 = v * w * (1 - cos_theta) + u * sin_theta
#     a33 = w_sqr + (u_sqr + v_sqr) * cos_theta
#     a34 = 0.0
#     a41 = 0.0
#     a42 = 0.0
#     a43 = 0.0
#     a44 = 1.0
#
#     rotation_matrix = [[a11, a12, a13, a14],
#                        [a21, a22, a23, a24],
#                        [a31, a32, a33, a34],
#                        [a41, a42, a43, a44]]
#
#     return mathutils.Matrix(rotation_matrix).to_4x4()

def import_bobj(filepath):
    """
    """
    bobj_import.load(filepath)


class RobotModelParser():
    """Base class for a robot model file parser of a specific type"""

    def __init__(self, filepath):
        self.filepath = filepath
        self.path, self.filename = os.path.split(self.filepath)
        self.robot = {'links': {},
                      'joints': {},
                      'sensors': {},
                      'motors': {},
                      'controllers': {},
                      'materials': {},
                      'groups': {},
                      'chains': {}
                      }

    def scaleLink(self, link, newlink):
        """Scales newly-created armatures depending on the link's largest collision object."""
        newscale = 0.3
        if len(link['collision']) > 0:
            sizes = []
            for collname in link['collision']:
                collobj = link['collision'][collname]
                colltype = collobj['geometry']['type']
                if colltype == 'sphere':
                    sizes.append(collobj['geometry']['radius'])
                elif colltype == 'cylinder':
                    sizes.append(max(collobj['geometry']['radius'], collobj['geometry']['length']))
                elif colltype == 'mesh':
                    sizes.append(max(collobj['geometry']['scale']))  # TODO: this alone is not very informative
                else:
                    sizes.append(max(collobj['geometry']['size']))
            newscale = max(sizes)
        newlink.scale = (newscale, newscale, newscale)

    def placeChildLinks(self, parent):
        bpy.context.scene.layers = defLayers(defs.layerTypes['link'])
        print(parent['name'] + ', ', end='')
        children = []
        for l in self.robot['links']:
            if 'parent' in self.robot['links'][l] and self.robot['links'][l]['parent'] == parent['name']:
                children.append(self.robot['links'][l])
        for child in children:
            # 1: set parent relationship (this makes the parent inverse the inverse of the parents world transform)
            parentLink = bpy.data.objects[parent['name']]
            childLink = bpy.data.objects[child['name']]
            selectObjects([childLink, parentLink], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            # 2: move to parents origin by setting the world matrix to the parents world matrix
            childLink.matrix_world = parentLink.matrix_world        # removing this line does not seem to make a difference

            # #bpy.context.scene.objects.active = childLink
            # if 'pivot' in child:
            #     pivot = child['pivot']
            #     cursor_location = bpy.context.scene.cursor_location
            #     bpy.context.scene.cursor_location = mathutils.Vector((-pivot[0]*0.3, -pivot[1]*0.3, -pivot[2]*0.3))
            #     bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
            #     bpy.context.scene.cursor_location = cursor_location

            # 3: apply local transform as saved in urdf (change matrix_local from identity to urdf)
            location = mathutils.Matrix.Translation(child['pose']['translation'])
            rotation = mathutils.Euler(tuple(child['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
            transform_matrix = location * rotation
            childLink.matrix_local = transform_matrix
            # 4: be happy, as world and basis are now the same and local is the transform to be exported to urdf
            # 5: take care of the rest of the tree
            self.placeChildLinks(child)

    def placeLinkSubelements(self, link):
        bpy.context.scene.layers = defLayers([defs.layerTypes[t] for t in defs.layerTypes])
        parentLink = bpy.data.objects[link['name']]
        if 'inertial' in link:
            if 'pose' in link['inertial']:
                urdf_geom_loc = mathutils.Matrix.Translation(link['inertial']['pose']['translation'])
                urdf_geom_rot = mathutils.Euler(tuple(link['inertial']['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
            else:
                urdf_geom_loc = mathutils.Matrix.Identity(4)
                urdf_geom_rot = mathutils.Matrix.Identity(4)
            print(link['name'], link['inertial'])
            print(link['inertial']['name'])
            geoname = link['inertial']['name'] #g
            try:
                geom = bpy.data.objects[geoname]
            except:
                print('ERROR: missing subelement')
                return
            inertialname = link['inertial']['name']
            inertialobj = bpy.data.objects[inertialname]
            selectObjects([inertialobj, parentLink], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            inertialobj.matrix_local = urdf_geom_loc * urdf_geom_rot
        for geomsrc in ['visual', 'collision']:
            if geomsrc in link:
                for g in link[geomsrc]:
                    geomelement = link[geomsrc][g]
                    if 'pose' in geomelement:
                        urdf_geom_loc = mathutils.Matrix.Translation(geomelement['pose']['translation'])
                        urdf_geom_rot = mathutils.Euler(tuple(geomelement['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
                    else:
                        urdf_geom_loc = mathutils.Matrix.Identity(4)
                        urdf_geom_rot = mathutils.Matrix.Identity(4)
                    geoname = geomelement['name']
                    geom = bpy.data.objects[geoname]
                    # FIXME: this does not do anything - how to set basis matrix to local?
                    #geom.matrix_world = parentLink.matrix_world
                    #selectObjects([geom], True, 0)
                    #bpy.ops.object.transform_apply(location=True, rotation=True)
                    selectObjects([geom, parentLink], True, 1)
                    bpy.ops.object.parent_set(type='BONE_RELATIVE')
                    geom.matrix_local = urdf_geom_loc * urdf_geom_rot
                    try:
                        geom.scale = geomelement['geometry']['scale']
                    except KeyError:
                        pass

                    selectObjects([geom, parentLink], True, 1)
                    #if 'pivot' in link:
                    #    pivot = link['pivot']
                    #    #object_location = geom.location
                    #    cursor_location = bpy.context.scene.cursor_location
                    #    bpy.context.scene.cursor_location = mathutils.Vector((pivot[0],
                    #                                                          pivot[1],
                    #                                                          pivot[2]))
                    #   # print('loc:', object_location)
                    #   # print('piv:', pivot)
                    #    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
                    #    bpy.context.scene.cursor_location = cursor_location


    def attachSensor(self, sensor):
        bpy.context.scene.layers = defLayers([defs.layerTypes[t] for t in defs.layerTypes])
        try:
            parentLink = bpy.data.objects[sensor['link']]
            if 'pose' in sensor:
                urdf_geom_loc = mathutils.Matrix.Translation(sensor['pose']['translation'])
                urdf_geom_rot = mathutils.Euler(tuple(sensor['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
            else:
                urdf_geom_loc = mathutils.Matrix.Identity(4)
                urdf_geom_rot = mathutils.Matrix.Identity(4)
            sensorobj = bpy.data.objects[sensor['name']]
            selectObjects([sensorobj, parentLink], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            sensorobj.matrix_local = urdf_geom_loc * urdf_geom_rot
        except KeyError:
            print('###ERROR: inconsisent data on sensor:', sensor['name'])


    def createGeometry(self, viscol, geomsrc):
        newgeom = None
        if viscol['geometry'] is not {}:
            dimensions = None
            bpy.ops.object.select_all(action='DESELECT')
            geom = viscol['geometry']
            geomtype = geom['type']
            # create the Blender object
            # tag all objects
            for obj in bpy.data.objects:
                obj.tag = True
            if geomtype == 'mesh':
                bpy.context.scene.layers = defLayers(defs.layerTypes[geomsrc])
                filetype = geom['filename'].split('.')[-1]
                if filetype == 'obj' or filetype == 'OBJ':
                    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, geom['filename']))
                elif filetype == 'stl' or filetype == 'STL':
                    bpy.ops.import_mesh.stl(filepath=os.path.join(self.path, geom['filename']))
                # hack for test:
                elif filetype == 'bobj' or filetype == 'BOBJ':
                    import_bobj(os.path.join(self.path, geom['filename']))
                    #filename = geom['filename'].split('.')[0] + '.obj'
                    #try:
                    #    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, filename))
                    #except:
                    #    print('ERROR: Missing object.')
                    #    return
                # find the newly imported obj
                for obj in bpy.data.objects:
                    if not obj.tag:
                        newgeom = obj
                        #with obj file import, blender only turns the object, not the vertices,
                        #leaving a rotation in the matrix_basis, which we here get rid of
                        if filetype == 'obj':
                            bpy.ops.object.select_all(action='DESELECT')
                            newgeom.select = True
                            bpy.ops.object.transform_apply(rotation=True)
                newgeom.name = viscol['name']
                newgeom['filename'] = geom['filename']
                #newgeom.select = True
                #if 'scale' in geom:
                #    newgeom.scale = geom['scale']
                #bpy.ops.object.transform_apply(scale=True)
            elif geomtype == 'box':
                dimensions = geom['size']
            elif geomtype == 'cylinder':
                dimensions = (geom['radius'], geom['length'])
            elif geomtype == 'sphere':
                dimensions = geom['radius']
            else:
                log("Could not determine geometry type of " + geomsrc + viscol['name'] + '. Placing empty coordinate system.', "ERROR")
            if dimensions:  # if a standard primitive type is found, create the object
                newgeom = createPrimitive(viscol['name'], geomtype, dimensions, player=geomsrc)
                newgeom.select = True
                bpy.ops.object.transform_apply(scale=True)
            if newgeom is not None:
                newgeom.phobostype = geomsrc
                newgeom['geometry/type'] = geomtype
                if geomsrc == 'visual':
                    try:
                        newgeom.data.materials.append(bpy.data.materials[viscol['material']['name']])
                    except KeyError:
                        log('No material for obj', viscol['name'])
            #FIXME: place empty coordinate system and return...what? Error handling of file import!
        for prop in viscol:
            if prop.startswith('$'):
                for tag in viscol[prop]:
                    newgeom[prop[1:]+'/'+tag] = viscol[prop][tag]
        return newgeom

    def createInertial(self, name, inertial):
        bpy.ops.object.select_all(action='DESELECT')
        inert = createPrimitive('inertial_'+name, 'sphere', 0.01, 0, 'None', (0, 0, 0))
        inert.select = True
        bpy.ops.object.transform_apply(scale=True)
        for prop in inertial:
            if prop not in ['pose'] and inertial[prop] is not None:
                if not prop.startswith('$'):
                    inert[prop] = inertial[prop]
                else:
                    for tag in inertial[prop]:
                        inert[prop[1:]+'/'+tag] = inertial[prop][tag]
        inert.phobostype = 'inertial'
        return inert

    def createLink(self, link):
        bpy.context.scene.layers = defLayers(defs.layerTypes['link'])
        #create base object ( =armature)
        bpy.ops.object.select_all(action='DESELECT')
        #bpy.ops.view3d.snap_cursor_to_center()
        bpy.ops.object.armature_add(layers=defLayers([0]))
        newlink = bpy.context.active_object #print(bpy.context.object) #print(bpy.context.scene.objects.active) #bpy.context.selected_objects[0]
        newlink.name = link['name']
        newlink.location = (0.0, 0.0, 0.0)
        newlink.scale = (0.3, 0.3, 0.3) #TODO: make this depend on the largest visual or collision object
        bpy.ops.object.transform_apply(scale=True)
        newlink.phobostype = 'link'
        if newlink.name != link['name']:
            log("Warning, name conflict!")
        # place inertial
        if 'inertial' in link:
            self.createInertial(link['name'], link['inertial'])
        # place visual
        if 'visual' in link:
            for v in link['visual']:
                visual = link['visual'][v]
                if 'geometry' in visual:
                    self.createGeometry(visual, 'visual')
        # place collision
        if 'collision' in link:
            for c in link['collision']:
                collision = link['collision'][c]
                if 'geometry' in collision:
                    self.createGeometry(collision, 'collision')
        for prop in link:
            if prop.startswith('$'):
                for tag in link[prop]:
                    newlink['link/'+prop[1:]+'/'+tag] = link[prop][tag]
        return newlink

    def _get_object(self, link_name):
        """
        """
        objects = bpy.context.scene.objects
        obj = objects[link_name]
        return obj

    def _apply_joint_angle_offsets(self):
        '''
        '''
        links = self.robot['links']
        for link_name in links:
            link = links[link_name]
            if 'axis1' in link:
                # move edit bone y axis to axis1
                bpy.ops.object.mode_set(mode='EDIT')

                axis_vec = mathutils.Vector(link['axis1'])
                bpy.data.objects[link_name].data.bones.active.tail = axis_vec

                print()
                print('link name:', link_name)
                print("axis:", axis_vec)

                bpy.ops.object.mode_set(mode='OBJECT')
                pass
            if 'angle_offset' in link:
                #angle = link['angle_offset']
                ##axis = link['axis']
                #obj = self._get_object(link_name)
                #object_matrix = obj.matrix_world
                #loc, rotation, scale = object_matrix.decompose()
                #z_axis = mathutils.Vector([0, 0, 1])
                #axis_vec = rotation.to_matrix() * z_axis

                #offset_matrix = mathutils.Matrix.Rotation(angle, 4, axis_vec)
                #obj.matrix_world = offset_matrix * object_matrix


                obj = self._get_object(link_name)
                angle = -(link['angle_offset'])
                bpy.ops.object.mode_set(mode='POSE')
                pose_bone = obj.pose.bones['Bone']
                bpy.ops.pose.armature_apply()
                bpy.ops.object.mode_set(mode='OBJECT')
                y_axis = mathutils.Vector([0, 1, 0])
                bone_matrix = pose_bone.matrix
                loc, rot, scale = bone_matrix.decompose()
                axis = rot * y_axis
                offset_matrix = mathutils.Matrix.Rotation(angle, 4, axis)
                pose_bone.matrix = offset_matrix * bone_matrix


                #angle = link['angle_offset']
                #obj = self._get_object(link_name)
                #obj_matrix = obj.matrix_local
                #loc, rot, scale = obj_matrix.decompose()
                #z_axis = mathutils.Vector([0, 0, 1])
                #offset_matrix = mathutils.Matrix.Rotation(angle, 4, z_axis)
                #combined_matrix = obj_matrix * offset_matrix
                #obj.matrix_local = combined_matrix
    def createJoint(self, joint):
        bpy.context.scene.layers = defLayers(defs.layerTypes['link'])
        link = bpy.data.objects[joint['child']]
        # add joint information
        # link['joint/type'] = joint['type']

        # set axis
        selectObjects([link], clear=True, active=0)
        bpy.ops.object.mode_set(mode='EDIT')
        editbone = link.data.edit_bones[0]
        #oldaxis = editbone.vector
        length = editbone.length
        if 'axis' in joint:
            axis = mathutils.Vector(tuple(joint['axis']))
            #oldaxis.cross(axis) # rotation axis
            editbone.tail = editbone.head + axis.normalized() * length

        # add constraints
        for param in ['effort', 'velocity']:
            try:
                link['joint/max'+param] = joint['limits'][param]
            except KeyError:
                log("Key Error in adding joint constraints") #Todo: more details
        try:
            lower = joint['limits']['lower']
            upper = joint['limits']['upper']
        except KeyError:
            lower = 0.0
            upper = 0.0
        joints.setJointConstraints(link, joint['type'], lower, upper)
        for prop in joint:
            if prop.startswith('$'):
                for tag in joint[prop]:
                    link['joint/'+prop[1:]+'/'+tag] = joint[prop][tag]

    def createMotor(self, motor):
        try:
            joint = bpy.data.objects[motor['joint']]
            for prop in motor:
                if prop != 'joint':
                    if not prop.startswith('$'):
                        joint['motor/'+prop] = motor[prop]
                    else:
                        for tag in motor[prop]:
                            joint['motor/'+prop[1:]+'/'+tag] = motor[prop][tag]
        except KeyError:
            pass #log("Joint " + motor['joint'] + " does not exist", "ERROR")

    def createSensor(self, sensor):
        #newsensor = sensors.createSensor(sensor)
        pass

    def createController(self, controller):
        pass

    def createGroup(self, group):
        pass

    def createChain(self, chain):
        pass

    def createBlenderModel(self): #TODO: solve problem with duplicated links (linklist...namespaced via robotname?)
        """Creates the blender object representation of the imported model."""
        print("\n\nCreating Blender model...")
        for l in self.robot['links']:
            #print(l + ', ', end='')
            link = self.robot['links'][l]
            #print(link['name'])
            self.createLink(link)

        print("\n\nCreating joints...")
        for j in self.robot['joints']:
            print(j + ', ', end='')
            joint = self.robot['joints'][j]
            self.createJoint(joint)

        print("\n\nCreating sensors...")
        for s in self.robot['sensors']:
            sensor = self.robot['sensors'][s]
            self.createSensor(sensor)

        #build tree recursively and correct translation & rotation on the fly
        for l in self.robot['links']:
            if not 'parent' in self.robot['links'][l]:
                root = self.robot['links'][l]
        print("\n\nPlacing links...")
        self.placeChildLinks(root)
        print("\n\nAssigning model name...")
        try:
            print('ROOT:', root)
            rootlink = getRoot(bpy.data.objects[root['name']])
            rootlink['modelname'] = self.robot['name']
        except KeyError:
            print('ROOT:', root)
            print("Could not assign model name to root link.", "ERROR")
            assert False
        for link in self.robot['links']:
            self.placeLinkSubelements(self.robot['links'][link])
        for sensorname in self.robot['sensors']:
            sensor = self.robot['sensors'][sensorname]
            self.attachSensor(sensor)

        print("\n\nCreating motors...")
        for m in self.robot['motors']:
            motor = self.robot['motors'][m]
            self.createMotor(motor)

        print("\n\nCreating controllers...")
        for c in self.robot['controllers']:
            controller = self.robot['controllers'][c]
            self.createController(controller)

        print("\n\nCreating groups...")
        for g in self.robot['groups']:
            group = self.robot['groups'][g]
            self.createGroup(group)

        print("\n\nCreating chains...")
        for ch in self.robot['chains']:
            chain = self.robot['chains'][ch]
            self.createChain(chain)

        self._apply_joint_angle_offsets()

        print('Done!')

        #for obj in bpy.data.objects:
        #    print('name:', obj.name)
        #    matrix = obj.matrix_local
        #    loc, rot, scale = matrix.decompose()
            #print('rotation:')
            #print('\tx:', rot.x, '\n\ty:', rot.y, '\n\tz:', rot.z, '\n\tw:', rot.w)
            #print('location:')
            #print('\tx:', loc.x, '\n\ty:', loc.y, '\n\tz:', loc.z)
            #print('-------------------')
            
        
    def _debug_output(self):
        '''
        Write the robot dictionary to a yaml file in the source file's directory
        '''
        with open(self.filepath + '_ref_debug.yml', 'w') as outputfile:
            outputfile.write(yaml.dump(self.robot)) #last parameter prevents inline formatting for lists and dictionaries


class MARSModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a MARS scene"""

    def __init__(self, filepath):
        '''
        Initialise a 'MARSModelParser' object and a number of containers
        used to keep track of the parsed information and to apply it in
        the right places.
        
        Explanation of some of the containers:
            - self.link_index_dict:
                {link index: link name}
            - self.link_indices:
                set of all indices of xml nodes that are links
            - self.material_indices:
                {material index: dictionary containing material information}
            - self.applied_rel_id_poses:
                {link id: dictionary containing pose with parent pose
                          applied}
            - self.name_counter_dict:
                {link/visual/collision/material name:
                 amount of how often the name already has been given to
                 something (plus index for distinction)}
        '''
        RobotModelParser.__init__(self, filepath)
        
        self.xml_tree = None
        self.link_index_dict = {}
        self.link_groups_group_order = {}
        self.link_groups_link_order = {}
        self.link_indices = set([])
        self.material_indices = {}
        self.vis_coll_groups = {}
        self.applied_rel_id_poses = {}
        self.missing_vis_geos = {}
        self.missing_coll_geos = {}
        self.name_counter_dict = {}
        self.link_group_dict = {}
        self.robot_states = {}
        self.joint_info = {}
        self.parent_joint_dict = {}

    def parseModel(self):
        '''
        '''
        print("\nParsing MARS scene from", self.filepath)
        self.tree = ET.parse(self.filepath)
        root = self.tree.getroot()
        
        nodes = root.find('nodelist')
        joints = root.find('jointlist')
        sensors = root.find('sensorlist')
        motors = root.find('motorlist')
        controllers = root.find('controllerlist')
        material_list = root.find('materiallist')


        self._parse_joints(joints)
        self._get_links(nodes, joints)
        self._apply_relative_ids(nodes)

        self._parse_materials(material_list)
        
        links = self._parse_links(nodes)
        #self._add_parent_links(links)
        self.robot['name'] = self.filepath.split('.')[0]
        self.robot['links'] = links
        self.robot['joints'] = self._create_joints_dict()
        self.robot['sensors'] = self._parse_sensors(sensors)
        self.robot['motors'] = self._parse_motors(motors)
        self.robot['controllers'] = self._parse_controllers(controllers)
        self.robot['groups'] = self.link_groups_group_order
        self._parse_additional_visuals_and_collisions(self.robot, nodes)
        
        for link in self.robot['links']:
            handle_missing_geometry(self.missing_vis_geos[link], self.missing_coll_geos[link], self.robot['links'][link])

        self.robot['states'] = self.robot_states

        self._debug_output()

        #print(self.link_groups_link_order)
        #print()
        #print(self.vis_coll_groups)
        #print()
        #print(self.link_index_dict)
        #print()
        #print(self.link_indices)
        
       # assert False
    
    def _get_links(self, nodes, joints):
        '''
        Collect the indices of all nodes that are links inside
        'self.link_indices'.
        '''
        for joint in joints:
            child = int(joint.find('nodeindex2').text)
            self.link_indices.update([child])
        for node in nodes:
            if node.find('relativeid') is None:
                self.link_indices.update([int(node.find('index').text)])

    def _create_joints_dict(self):
        '''
        '''
        joints_dict = {}
        for joint_name in self.joint_info:
            joint = self.joint_info[joint_name]
            joint_dict = {}
            for info in ['name', 'type', 'angle_offset', 'axis1']:
                joint_dict[info] = joint[info]
            joint_dict['parent'] = self.link_index_dict[joint['parent_index']]
            joint_dict['child'] = self.link_index_dict[joint['child_index']]
            joints_dict[joint_name] = joint_dict
        return joints_dict
        
    def _parse_materials(self, materials_tree):
        '''
        Parse the materials from the MARS scene.
        
        TODO: change 'materials' so that superfluous 'color' entry is not needed
        '''
        material_list = []
        for material in materials_tree:
            material_dict = {}
            mat_id = int(material.find('id').text)
            
            # check needs to be explicit for future versions
            if material.find('name') is not None:
                name = self._get_distinct_name(material.find('name').text)
            else:
                name = 'material_' + str(mat_id)
            material_dict['name'] = name

            #TODO: fix later

            #for xml_colour in defs.MARSrevlegdict:
            #    colour = material.find(xml_colour)
            #    if colour is not None:
            #        r = round_float(colour.find('r').text)
            #        g = round_float(colour.find('g').text)
            #        b = round_float(colour.find('b').text)
            #        a = round_float(colour.find('a').text)
            #        py_colour = defs.MARSrevlegdict[xml_colour]
            #        material_dict[py_colour] = [r, g, b, a]
            #        # for now:
            #        material_dict['color'] = [r, g, b, a]
            
            
            transparency = material.find('transparency')
            if transparency is not None:
                material_dict['transparency'] = round_float(transparency.text)
            else:
                material_dict['transparency'] = 0.0
                
            material_dict['shininess'] = round_float(material.find('shininess').text)
            self.material_indices[mat_id] = material_dict
            material_list.append(material_dict)
            
        #for m in material_list:
        #    materials.makeMaterial(m['name'], tuple(m['color'][0:3]), (1, 1, 1), m['color'][-1])
    
    def _parse_geometry(self, vis_coll_dict, node, mode):
        '''
        Parse the geometry of a visual or collision object.
        '''
        size = None
        if mode == 'visual':
            size = node.find('visualsize')
            mesh_file = node.find('filename')
            if mesh_file is not None:
                geometry_type = 'mesh'
            else:
                geometry_type = node.find('physicmode').text
        elif mode == 'collision':
            size = node.find('extend')
            geometry_type = node.find('physicmode').text
        
        geometry_dict = {}
        geometry_dict['type'] = geometry_type
        
        if geometry_type == 'box' or geometry_type == 'mesh':
            x = round_float(size.find('x').text)
            y = round_float(size.find('y').text)
            z = round_float(size.find('z').text)
            geometry_dict['size'] = [x, y, z]
            if geometry_type == 'mesh':
                filename = node.find('filename').text
                geometry_dict['filename'] = filename
        elif geometry_type == 'sphere' or geometry_type == 'cylinder' or geometry_type == 'capsule':
            radius = round_float(size.find('x').text) / 2.0
            geometry_dict['radius'] = radius
            if geometry_type == 'cylinder' or geometry_type == 'capsule':
                height = round_float(size.find('z').text)
                geometry_dict['length'] = height
                # geometry_dict['height'] = height
        elif geometry_type == 'plane':
            x = round_float(size.find('x').text)
            y = round_float(size.find('y').text)
            geometry_dict['size'] = [x, y]
        
        if geometry_dict == {}:
            return False
        else:
            vis_coll_dict['geometry'] = geometry_dict
            return True
    
    def _parse_visual(self, visuals_dict, node, missing_vis_geo):
        '''
        Parse a visual object.
        '''
        visual_dict = {}
        name = self._get_distinct_name('visual_' + node.get('name'))
        visual_dict['name'] = name
        index = int(node.find('index').text)

        base_pose = self.applied_vis_col_poses[index]
        base_pos = base_pose['translation']
        rot = base_pose['rotation_quaternion']
        pivot_xml = node.find('pivot')
        if pivot_xml is not None:
            pivot = [float(pivot_xml.find('x').text), float(pivot_xml.find('y').text), float(pivot_xml.find('z').text)]
        else:
            pivot = [0.0, 0.0, 0.0]

        pos = base_pos
        #visual_dict['pose'] = calc_pose_formats(pos, rot, pivot=pivot)
        visual_dict['pose'] = calc_pose_formats(pos, rot)
        mat_index = int(node.find('material_id').text)
        visual_dict['material'] = self.material_indices[mat_index]
        
        if not self._parse_geometry(visual_dict, node, 'visual'):
            missing_vis_geo.append(name)
        
        visuals_dict[name] = visual_dict
        
    def _parse_collision(self, collisions_dict, node, missing_coll_geo):
        '''
        Parse a collision object.
        '''
        collision_dict = {}
        name = self._get_distinct_name('collision_' + node.get('name'))
        collision_dict['name'] = name
        index = int(node.find('index').text)        
        
        #if name.startswith('collision_joint_sphere'):
        #    position, rotation = pos_rot_tree_to_lists(None, None)
        #    collision_dict['pose'] = calc_pose_formats(position, rotation)
        #else:
        collision_dict['pose'] = self.applied_vis_col_poses[index]
        
        bitmask = int(float(node.find('coll_bitmask').text))
        collision_dict['bitmask'] = bitmask
        
        if not self._parse_geometry(collision_dict, node, 'collision'):
            missing_coll_geo.append(name)
        
        max_contacts = node.find('cmax_num_contacts')
        if max_contacts is not None:
            collision_dict['max_contacts'] = int(max_contacts.text)
        
        collisions_dict[name] = collision_dict
        
    def _parse_inertial(self, link_dict, node):
        '''
        Parse an inertial.
        
        Note: Pose in inertia does not seem to be available.
        '''
        inertial_dict = {}
        
        mass = node.find('mass')
        if mass is not None:
            inertial_dict['mass'] = round_float(mass.text)
        
        inertia = node.find('inertia')
        if inertia is not None and bool(inertia.text):  # 'inertia' states whether the defined inertia values are to be used
            i00 = round_float(node.find('i00').text)
            i01 = round_float(node.find('i01').text)
            i02 = round_float(node.find('i02').text)
            i11 = round_float(node.find('i11').text)
            i12 = round_float(node.find('i12').text)
            i22 = round_float(node.find('i22').text)
            inertial_dict['inertia'] = [i00, i01, i02,
                                             i11, i12,
                                                  i22]
                                
        if inertial_dict is not {}:
            # inertial pose is always origin for now
            position, rotation = pos_rot_tree_to_lists(None, None)
            inertial_dict['pose'] = calc_pose_formats(position, rotation)
            inertial_dict['name'] = self._get_distinct_name('inertial_' + node.get('name'))
            link_dict['inertial'] = inertial_dict

        return inertial_dict
     
    def _get_distinct_name(self, name):
        '''
        Create a name that has not been used yet in this model.
        '''
        if name in self.name_counter_dict:
            name_counter = self.name_counter_dict[name]
            distinct_name = name + '_' + str(name_counter)
            self.name_counter_dict[name] = name_counter + 1
        else:
            distinct_name = name
            self.name_counter_dict[name] = 1
        return distinct_name
        
    def _parse_links(self, nodes):
        '''
        Parse the links of the MARS scene.
        '''
        links_dict = {}
        for node in nodes:
            missing_vis_geo = []
            missing_coll_geo = []
            index = int(node.find('index').text)
            name = self._get_distinct_name(node.get('name'))
                
            self.link_index_dict[index] = name
            
            group = int(node.find('groupid').text)
            node_group_dict = {'name': name,
                               'index': index}
            if group in self.link_groups_group_order:
                self.link_groups_group_order[group].append(node_group_dict)
            else:
                self.link_groups_group_order[group] = [node_group_dict]
            
            rel_id = node.find('relativeid')
            if index in self.link_indices:
                link_dict = {}
                
                link_dict['name'] = name
                
                pose = self.applied_rel_id_poses[index]
                link_dict['pose'] = pose

                visuals_dict = {}
                #self._parse_visual(visuals_dict, node, missing_vis_geo)
                link_dict['visual'] = visuals_dict
                
                collisions_dict = {}
                #self._parse_collision(collisions_dict, node, missing_coll_geo)
                link_dict['collision'] = collisions_dict
                
                inertial_dict = self._parse_inertial(link_dict, node)
                link_dict['inertial'] = inertial_dict

                pivot = node.find('pivot')
                if pivot is not None:
                    x = float(pivot.find('x').text)
                    y = float(pivot.find('y').text)
                    z = float(pivot.find('z').text)
                    link_dict['pivot'] = [x, y, z]

                if rel_id is not None:
                    link_dict['parent'] = int(rel_id.text)
                
                links_dict[name] = link_dict
                self.missing_vis_geos[name] = missing_vis_geo
                self.missing_coll_geos[name] = missing_coll_geo

                if index in self.parent_joint_dict:
                    joint = self.joint_info[self.parent_joint_dict[index]]
                    axis = joint['axis1']
                    angle = joint['angle_offset']
                    link_dict['axis1'] = axis
                    link_dict['angle_offset'] = angle

        for group_index in self.link_groups_group_order:
            group = self.link_groups_group_order[group_index]
            for node in group:
                if node['index'] in self.link_indices:
                    link_index = node['index']
                    self.link_groups_link_order[link_index] = group
                    break
                    
        for link_index in self.link_groups_link_order:
            group = self.link_groups_link_order[link_index]
            for node in group:
                self.link_group_dict[node['index']] = link_index
        
        for link_name in links_dict:
            link_dict = links_dict[link_name]
            if 'parent' in link_dict:
                link_dict['parent'] = self.link_index_dict[self.link_group_dict[link_dict['parent']]]
        
        return links_dict
        
    def _add_parent_links(self, links):
        '''
        Replace parent indices (not actually parent indices!) with parent names inside the link
        dictionaries.
        
        TODO: There is a better solution, surely.
        '''
        for link_name in links:
            link = links[link_name]
            if 'parent' in link:
                rel_id = link['parent']
                if rel_id in self.vis_coll_groups:
                    parent_id = self.vis_coll_groups[rel_id]
                else:
                    parent_id = rel_id
                link['parent'] = self.link_index_dict[parent_id]

        
    def _parse_additional_visuals_and_collisions(self, model, nodes):
        '''
        Parse nodes that are no links as additional visual and collision
        objects for the already parsed links.
        '''
        for link in self.link_groups_link_order:
            for node in self.link_groups_link_order[link]:
                self.vis_coll_groups[node['index']] = link
        
        for node in nodes:
            index = int(node.find('index').text)
            if index in self.vis_coll_groups:
                link_index = self.vis_coll_groups[index]
                group = self.link_groups_link_order[link_index]
                for group_node in group:
                    if group_node['index'] == link_index:
                        name = group_node['name']
                        visuals_dict = model['links'][name]['visual']
                        self._parse_visual(visuals_dict, node, self.missing_vis_geos[name])
                        model['links'][name]['visual'] = visuals_dict
                        
                        collisions_dict = model['links'][name]['collision']
                        self._parse_collision(collisions_dict, node, self.missing_coll_geos[name])
                        model['links'][name]['collision'] = collisions_dict
                        
                        break
        
    def _parse_joints(self, joints):
        '''
        Parse the joints of the MARS scene.
        '''
        state_dict = {}
        for joint in joints:
            joint_dict = {}
            name = joint.get('name')
            joint_dict['name'] = name
            joint_dict['type'] = joint.find('type').text
            
            parent_index = int(joint.find('nodeindex1').text)
            joint_dict['parent_index'] = parent_index
            #joint_dict['parent'] = self.link_index_dict[parent_index]
            child_index = int(joint.find('nodeindex2').text)
            joint_dict['child_index'] = child_index
            self.parent_joint_dict[child_index] = name
            #joint_dict['child'] = self.link_index_dict[child_index]


            xml_offset = joint.find('angle1_offset')
            if xml_offset is not None:
                dict_offset = float(xml_offset.text)
            else:
                dict_offset = 0.0

            state_dict[name] = dict_offset
            joint_dict['angle_offset'] = dict_offset

            xml_axis = joint.find('axis1')
            if xml_axis is not None:
                x = float(xml_axis.find('x').text)
                y = float(xml_axis.find('y').text)
                z = float(xml_axis.find('z').text)
                dict_axis = [x, y, z]
            else:
                dict_axis = [0.0, 0.0, 0.0]

            joint_dict['axis1'] = dict_axis

            self.joint_info[name] = joint_dict

        self.robot_states['start'] = state_dict


    def _apply_relative_ids(self, nodes):
        absolute_poses = {}
        absolute_vis_coll_poses = {}
        relative_poses = {}
        for node in nodes:
            index = int(node.find('index').text)
            xml_position = node.find('position')
            xml_rotation = node.find('rotation')
            position, rotation = pos_rot_tree_to_lists(xml_position, xml_rotation)
            #if index in self.parent_joint_dict:
            #    joint = self.joint_info[self.parent_joint_dict[index]]
            #    axis = joint['axis']
            #    angle = joint['angle_offset']
            #    pose = calc_pose_formats(position, rotation, angle, axis)
            #else:
            pose = calc_pose_formats(position, rotation)

            rel_id_xml = node.find('relativeid')
            #if index in self.link_indices:
            #    link_poses[index] = pose
            if rel_id_xml is None:
                absolute_poses[index] = pose
                absolute_vis_coll_poses[index] = pose
                root = index
            else:
                rel_id = int(rel_id_xml.text)
                relative_poses[index] = {'pose': pose, 'rel_id': rel_id}

            num_rel_poses = -1
        while relative_poses:
            to_delete = []
            # check for infinite loop (happens if referenced node does not exist):
            if len(relative_poses) == num_rel_poses:
                print('Error: non-existent relative id')
                break
            num_rel_poses = len(relative_poses)
            for index in relative_poses:
                #print('link_poses:', link_poses)
                relative_pose = relative_poses[index]
                rel_id = relative_pose['rel_id']
                if rel_id in absolute_poses:
                    #print('index:', index)
                    #print('rel_id:', rel_id)
                    if rel_id in self.link_indices and rel_id is not root:
                        absolute_poses[index] = relative_pose['pose']
                        absolute_vis_coll_poses[index] = relative_pose['pose']
                    else:
                        reference_pose = absolute_poses[rel_id]
                        rel_matrix = mathutils.Matrix(relative_pose['pose']['matrix']).to_4x4()
                        ref_matrix = mathutils.Matrix(reference_pose['matrix']).to_4x4()
                        applied_matrix = ref_matrix * rel_matrix # or the other way round?
                        #print('relative:\n', rel_matrix)
                        #print(rel_matrix.to_quaternion())
                        #print('reference:\n', ref_matrix)
                        #print(ref_matrix.to_quaternion())
                        #print('applied:\n', applied_matrix)
                        #print(applied_matrix.to_quaternion())
                        loc, rot, scale = applied_matrix.decompose()
                        applied_pos = [loc.x, loc.y, loc.z]
                        applied_rot = [rot.w, rot.x, rot.y, rot.z]
                        absolute_poses[index] = calc_pose_formats(applied_pos, applied_rot)
                        absolute_vis_coll_poses[index] = calc_pose_formats(applied_pos, applied_rot)
                    if index in self.link_indices:
                        pos, rot = pos_rot_tree_to_lists(None, None)
                        absolute_vis_coll_poses[index] = calc_pose_formats(pos, rot)
                    to_delete.append(index)
            for index in to_delete:
                del relative_poses[index]
        self.applied_rel_id_poses = absolute_poses
        self.applied_vis_col_poses = absolute_vis_coll_poses



    
    def _parse_sensors(self, sensors):
        '''
        'link' is missing in manual
        '''
        sensors_dict = {}
        for sensor in sensors:
            sensor_dict = {}
            name = sensor.get('name')
            sensor_dict['name'] = name
            xml_link = sensor.find('attached_node')
            if xml_link is not None:
                 sensor_dict['link'] = self.link_index_dict[int(xml_link.text)]
            sensor_dict['type'] = sensor.get('type')
            sensors_dict[name] = sensor_dict
        return sensors_dict
    
    def _parse_motors(self, motors):
        '''
        don't know yet what motors look like
        '''
        motors_dict = {}
        for motor in motors:
            motor_dict = {}
            name = motor.get('name')

            motors_dict[name] = motor_dict

        return motors_dict
    
    def _parse_controllers(self, controllers):
        '''
        don't know yet what controllers look like
        '''
        controllers_dict = {}
        if controllers:
            for controller in controllers:
                controller_dict = {}
                name = controller.get('name')

                controllers_dict[name] = controller_dict

        return controllers_dict


class URDFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a URDF model"""

    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)

    def parsePose(self, origin):
        pose = {}
        if origin is not None:
            pose['translation'] = parse_text(origin.attrib['xyz'])
            pose['rotation_euler'] = parse_text(origin.attrib['rpy'])
        else:
            pose['translation'] = defaults.idtransform
            pose['rotation_euler'] = defaults.idtransform
        return pose

    def parseModel(self):
        print("\nParsing URDF model from", self.filepath)
        self.tree = ET.parse(self.filepath)
        self.root = self.tree.getroot()#[0]
        self.robot["name"] = self.root.attrib["name"]
        if 'version' in self.root.attrib:
            self.robot["version"] = self.root.attrib['version'] #TODO: implement version functionality (time code)

        #write links to dictionary
        links = {}
        print("\n\nParsing links..")
        for link in self.root.iter('link'):
            newlink = self.parseLink(link)
            #write link to list
            links[newlink['name']] = newlink
        self.robot['links'] = links

        #write joints to dictionary
        joints = {}
        print("\n\nParsing joints..")
        for joint in self.root.iter('joint'):
            if joint.find('parent') is not None: #this is needed as there are "joint" tags e.g. in transmission
                newjoint, pose = self.parseJoint(joint)
                self.robot['links'][newjoint['child']]['pose'] = pose
                joints[newjoint['name']] = newjoint
        self.robot['joints'] = joints

        #find any links that still have no pose (most likely because they had no parent)
        for link in links:
            if not 'pose' in links[link]:
                links[link]['pose'] = self.parsePose(None)

        #write parent-child information to nodes
        print("\n\nWriting parent-child information to nodes..")
        for j in self.robot['joints']:
            joint = self.robot['joints'][j]
            self.robot['links'][joint['child']]['parent'] = joint['parent']
            #print(joint['parent'] + ', ', end='')
            
        self.parseMaterials()

        self._debug_output()

    def parseLink(self, link):
        #print(link.attrib['name'] + ', ', end='')
        newlink = {a: link.attrib[a] for a in link.attrib}

        self.parseInertial(newlink, link)
        #no_visual_geo = self.parseVisual(newlink, link)
        #no_collision_geo = self.parseCollision(newlink, link)
        #handle_missing_geometry(no_visual_geo, no_collision_geo, newlink)
        #parse visual and collision objects
        for objtype in ['visual', 'collision']:
            newlink[objtype] = {}
            i = 0
            for xmlelement in link.iter(objtype):
                try:
                    elementname = xmlelement.attrib['name']
                except KeyError:
                    elementname = objtype + '_' + str(i) + '_' + newlink['name']
                    i += 1
                newlink[objtype][elementname] = {a: xmlelement.attrib[a] for a in xmlelement.attrib}
                dictelement = newlink[objtype][elementname]
                dictelement['name'] = elementname
                dictelement['pose'] = self.parsePose(xmlelement.find('origin'))
                geometry = xmlelement.find('geometry')
                if geometry is not None:
                    dictelement['geometry'] = {a: parse_text(geometry[0].attrib[a]) for a in geometry[0].attrib}
                    dictelement['geometry']['type'] = geometry[0].tag
                    if geometry[0].tag == 'mesh':
                        dictelement['geometry']['filename'] = geometry[0].attrib['filename']
                        try:
                            dictelement['geometry']['scale'] = parse_text(geometry[0].attrib['scale'])
                        except KeyError:
                            dictelement['geometry']['scale'] = [1.0, 1.0, 1.0]
                material = xmlelement.find('material')
                if material is not None:
                    dictelement['material'] = {'name': material.attrib['name']}
                    # We don't need to do the following, as any material with color or texture
                    # will be parsed in the parsing of materials in parseModel
                    # This might be necessary if there are name conflicts etc.
                    #color = material.find('color')
                    #if color is not None:
                    #    dictelement['material']['color'] = parse_text(color.attrib['rgba'])
        if newlink == {}:
            print("\n### WARNING:", newlink['name'], "is empty.")
        return newlink
        
    def parseInertial(self, link_dict, link_xml):
        '''
        '''
        inertial = link_xml.find('inertial')
        if inertial is not None: # 'if Element' yields none if the Element contains no children, thus this notation
            link_dict['inertial'] = {}
            link_dict['inertial']['pose'] = self.parsePose(inertial.find('origin'))
            mass = inertial.find('mass')
            if mass is not None:
                link_dict['inertial']['mass'] = float(mass.attrib['value'])
            inertia = inertial.find('inertia')
            if inertia is not None:
                values = []
                link_dict['inertial']['inertia'] = values.append(inertia.attrib[a] for a in inertia.attrib)
            link_dict['inertial']['name'] = 'inertial_' + link_dict['name']
            
  
        

    def parseJoint(self, joint):
        #print(joint.attrib['name']+', ', end='')
        newjoint = {a: joint.attrib[a] for a in joint.attrib}
        pose = self.parsePose(joint.find('origin'))
        newjoint['parent'] = joint.find('parent').attrib['link']
        newjoint['child'] = joint.find('child').attrib['link']
        axis = joint.find('axis')
        if axis is not None:
            newjoint['axis'] = parse_text(axis.attrib['xyz'])
        limit = joint.find('limit')
        if limit is not None:
            newjoint['limits'] = {a: parse_text(limit.attrib[a]) for a in limit.attrib}
        #calibration
        #dynamics
        #limit
        #mimic
        #safety_controller
        return newjoint, pose
        
    def parseMaterials(self):
        '''
        '''
        material_list = [] #TODO: build dictionary entry for materials
        print("\n\nParsing materials..")
        for material in self.root.iter('material'):
            newmaterial = {a: material.attrib[a] for a in material.attrib}
            color = material.find('color')
            if color is not None:
                newmaterial['color'] = parse_text(color.attrib['rgba'])
                material_list.append(newmaterial)
        for m in material_list:
            #TODO: handle duplicate names? urdf_robotname_xxx?
            materials.makeMaterial(m['name'], tuple(m['color'][0:3]), (1, 1, 1), m['color'][-1]) 
    
class SRDFModelParser(RobotModelParser):
    """Class derived from RobotModelParser wich parses a SRDF extension file for URDF"""
    
    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)
        
    def parseModel(self, robot):
        collision_Exclusives = self.buildCollisionExclusives()
        collision_Dic = self.buildCollisionDictionary(collision_Exclusives, robot)
        collision_Groups = self.buildCollisionGroups(collision_Dic)
        robot = self.buildBitmasks(collision_Groups, robot)
        return robot
        
    def buildBitmasks(self, collision_Groups, robot):
        bits = len(collision_Groups)
        if bits > 20:
            print("The blender bitmask is not capable of more than 20 bit. The bitmask will be cutted!")
            bits = 20
        for link in robot['links']:
            for i in range(0, bits):
                if link in collision_Groups[i]:
                    for coll in robot['links'][link]['collision']:
                        try:
                            robot['links'][link]['collision'][coll]['bitmask'] += 2**i
                        except KeyError:
                            robot['links'][link]['collision'][coll]['bitmask'] = 2**i
        return robot
    
    def buildCollisionExclusives(self):
        print("\nParsing SRDF extensions from", self.filepath)
        self.tree = ET.parse(self.filepath)
        self.root = self.tree.getroot()
        
        collision_Exclusives = []
        for disabled_coll in self.root.iter('disable_collisions'):
            pair = (disabled_coll.attrib['link1'], disabled_coll.attrib['link2'])
            collision_Exclusives.append(pair)
            #print("Append ", pair, " to collision Exclusives")
        return collision_Exclusives
        
    
    def buildCollisionDictionary(self, collision_exclusives, robot):
        dic = {}
        for pair in collision_exclusives:
            if 'root' in pair or (pair[0] != robot['joints'][pair[1]]['parent'] and pair[1] != robot['joints'][pair[0]]['parent']):
                if pair[0] not in dic:
                    dic[pair[0]]=[]
                    dic[pair[0]].append(pair[1])
                else:
                    if pair[1] not in dic[pair[0]]:
                        dic[pair[0]].append(pair[1])
                if pair[1] not in dic:
                    dic[pair[1]]=[]
                    dic[pair[1]].append(pair[0])
                else:
                    if pair[0] not in dic[pair[1]]:
                        dic[pair[1]].append(pair[0])
            else:
                pass
                #print("Pair: ", pair, " not included")
        print("Collision Dictionary:\n", dic)
        return dic
                
    def checkGroup(self, group, colls):
        cut = []
        for elem in group:
            if elem in colls:
                cut.append(elem)
        if len(cut) == len(group):
            return cut
        else:
            return []

    def processGroup(self, group, link, colls):
        if link in group:
            for coll in colls:
                if coll in group:
                    colls.remove(coll)
        else:
            cut = self.checkGroup(group, colls)
            if len(cut) > 0:
                group.append(link)
                for l in cut:
                    colls.remove(l)
                
    def buildCollisionGroups(self, dic):
        groups=[]
        for link in dic:
            #rint("Current link: ", link)
            colls = dic[link]
            #print("Current colls: ", colls)
            for group in groups:
                #print("Current group: ", group)
                self.processGroup(group, link, colls)
            while len(colls) > 0:
                newgroup = [link, colls.pop()]
                groups.append(newgroup)
                self.processGroup(newgroup, link, colls)
        print ("Number of collision Groups: ", len(groups))
        #print ("Collision Groups:\n", groups)
        return groups
        

class SMURFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a SMURF model"""

    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)

    def parseModel(self):
        print("Parsing SMURF model...")
        #smurf = None
        with open(self.filepath, 'r') as smurffile:
            smurf = yaml.load(smurffile)
        if smurf is None:
            log('No valid SMURF file.', "ERROR")
            return None
        urdffile = None
        srdffile = None
        ymlfiles = [f for f in smurf['files'] if f.endswith('.yml') or f.endswith('.yaml')]
        for f in smurf['files']:
            if f.endswith('.urdf'):
                urdffile = f
            if f.endswith('.srdf'):
                srdffile = f
        # get URDF info
        if urdffile is None:
            log("Did not find URDF file associated with SMURF.", "ERROR")
            return None
        urdfparser = URDFModelParser(os.path.join(self.path, urdffile))
        urdfparser.parseModel()
        if srdffile is not None:
            srdfparser = SRDFModelParser(os.path.join(self.path, srdffile))
            self.robot = srdfparser.parseModel(urdfparser.robot)
        else:
            self.robot = urdfparser.robot
        # make sure all types exist
        typelist = ['links', 'joints', 'materials', 'sensors', 'motors', 'controllers', 'groups', 'chains']
        for key in typelist:
            if key not in self.robot:
                self.robot[key] = {}
        #add the smurf information
        custom_dicts = {}
        for yml in ymlfiles:
            with open(os.path.join(self.path, yml), 'r') as ymlfile:
                ymldict = yaml.load(ymlfile)
            for key in ymldict:
                print(key)
                if key in ['materials', 'sensors', 'motors', 'controllers']:
                    for element in ymldict[key]:
                        if element['name'] not in self.robot[key]:
                            self.robot[key][element['name']] = element
                        else:
                            for tag in element:
                                self.robot[key][element['name']][tag] = element[tag]
                elif key in 'state':
                    pass  # TODO: handle state
                else:
                    custom_dicts[key] = ymldict[key]

        for key in custom_dicts:
            print('assign custom properties:', key, custom_dicts[key])
            for element in custom_dicts[key]:  # iterate over list with custom annotations
                print(element)
                try:
                    objtype = element['type']
                except KeyError:
                    log("Could not find 'type' in custom annotation: " + str(element), "ERROR")
                try:
                    objname = element['name']
                except KeyError:
                    log("Could not find 'name' in custom annotation: " + str(element), "ERROR")
                try:
                    if objtype+'s' in typelist:  #FIXME: this is a total hack!
                        objtype += 's'
                    else:
                        raise TypeError(objtype)
                    if objname in self.robot[objtype]:
                        for tag in element:
                            if tag not in ['type', 'name']:
                                if not '$'+key in self.robot[objtype][objname]:
                                    self.robot[objtype][objname]['$'+key] = {tag: element[tag]}
                                else:
                                    self.robot[objtype][objname]['$'+key][tag] = element[tag]
                    else:
                        raise NameError(objname)
                except TypeError:
                    print("###ERROR: could not find 'type' or 'name' in custom annotation", objtype, objname)
                except NameError:
                    log("Element " + str(objname) + " of type " + str(objtype) + " does not exist in this model.", "ERROR")

        #now some debug output
        with open(self.filepath+'_SMURF_debug.yml', 'w') as outputfile:
            outputfile.write(yaml.dump(self.robot))#, default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries


class RobotModelImporter(bpy.types.Operator):
    """Importer for MARS-compatible model or scene files"""
    bl_idname = "obj.import_robot_model"
    bl_label = "Import robot model file from various formats"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'

    # creating property for storing the path to the .scn file
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    # set a filter to only consider .scn files (only used internally)
    #filter_glob = bpy.props.StringProperty(default="*.*",options={'HIDDEN'})

    @classmethod
    def poll(cls, context):
        return context is not None

    def execute(self, context):
        # get the chosen file path
        #directory, filename = os.path.split(self.filepath)
        modeltype = self.filepath.split('.')[-1]

        if modeltype == 'scene':
            importer = MARSModelParser(self.filepath)
        elif modeltype == 'urdf':
            importer = URDFModelParser(self.filepath)
        elif modeltype == 'smurf' or modeltype == 'yml' or modeltype == 'yaml':
            importer = SMURFModelParser(self.filepath)
        else:
            print("Unknown model format, aborting import...")

        cleanUpScene()
        importer.parseModel()
        importer.createBlenderModel()

        return {'FINISHED'}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}

# Register and add to the file selector
bpy.utils.register_class(RobotModelImporter)


#def apply_rel_ids_old(self, nodes):
#    '''
#    Collect the absolute poses for all nodes.

#   Does not work like that since it calculates poses relative to root,
#   but may become useful again.
#    '''
    #base_poses = {}
    #rel_poses = {}
    #for node in nodes:
    #    index = int(node.find('index').text)
    #    xml_position = node.find('position')
    #    xml_rotation = node.find('rotation')
    #    position, rotation = pos_rot_tree_to_lists(xml_position, xml_rotation)
    #    pose = calc_pose_formats(position, rotation)
    #
    #    if node.find('relativeid') is not None:
    #        rel_pose_dict = {'pose': pose,
    #                         'rel_id': int(node.find('relativeid').text)}
    #        rel_poses[index] = rel_pose_dict
    #    else:
    #        base_pose_dict = {'pose': pose}
    #        base_poses[index] = base_pose_dict
    #num_rel_poses = -1
    #while rel_poses:
    #    to_delete = []
    #    # check for infinite loop (happens if referenced node does not exist):
    #    if len(rel_poses) == num_rel_poses:
    #        print('Error: non-existant relative id')
    #        break
    #    num_rel_poses = len(rel_poses)
    #    for rel_index in rel_poses:
    #        rel_pose = rel_poses[rel_index]
    #        base = rel_pose['rel_id']
    #        if base in base_poses:
    #            base_pose = base_poses[base]['pose']
    #            rel = rel_pose['pose']
    #            base_matrix = mathutils.Matrix(base_pose['matrix']).to_4x4()
    #            relative_matrix = mathutils.Matrix(rel['matrix']).to_4x4()
    #            applied_matrix = base_matrix * relative_matrix
    #            loc, rot, scale = applied_matrix.decompose()
    #            applied_pos = [loc.x, loc.y, loc.z]
    #            applied_rot = [rot.w, rot.x, rot.y, rot.z]
    #            base_poses[rel_index] = {'pose': calc_pose_formats(applied_pos, applied_rot)}
    #            to_delete.append(rel_index)
    #    for index in to_delete:
    #        del rel_poses[index]
    #self.applied_rel_id_poses = base_poses


def main():
    # call the newly registered operator
    cleanUpScene()
    bpy.ops.import_robot_model('INVOKE_DEFAULT')

if __name__ == '__main__':
    main()
