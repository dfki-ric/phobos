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

File robotdictionary.py

Created on 28 Jul 2014

@author: Kai von Szadkowski, Stefan Rahms
"""

import os
import bpy
import mathutils
import copy
import sys
import yaml
from datetime import datetime
import warnings
import phobos.defs as defs
import phobos.joints as joints
import phobos.utils.naming as namingUtils
import phobos.utils.selection as selectionUtils
import phobos.utils.general as generalUtils
import phobos.utils.blender as blenderUtils
import phobos.logging as log


def register():
    """This function is called when this module is registered in blender.

    """
    print("Registering export...")


def collectMaterials(objectlist):
    """This function collects all materials from a list of objects and sorts them into a dictionary

    :param objectlist: The objectlist to grab the materials from.
    :type objectlist: list
    :return: dict

    """
    materials = {}
    for obj in objectlist:
        if obj.phobostype == 'visual' and obj.data.materials:
            mat = obj.data.materials[0]  # simply grab the first material
            if mat.name not in materials:
                materials[mat.name] = deriveMaterial(mat)
                materials[mat.name]['users'] = 1
            else:
                materials[mat.name]['users'] += 1
    return materials


def deriveMaterial(mat):
    """This function takes a blender material and creates a phobos representation from it

    :param mat: The blender material to derive a phobos material from
    :type mat: bpy.types.Material
    :return: dict

    """
    material = initObjectProperties(mat, 'material')
    material['name'] = mat.name
    material['diffuseColor'] = dict(zip(['r', 'g', 'b'], [mat.diffuse_intensity * num for num in list(mat.diffuse_color)]))
    material['ambientColor'] = dict(zip(['r', 'g', 'b'], [mat.ambient * mat.diffuse_intensity * num for num in list(mat.diffuse_color)]))
    material['specularColor'] = dict(zip(['r', 'g', 'b'], [mat.specular_intensity * num for num in list(mat.specular_color)]))
    if mat.emit > 0:
        material['emissionColor'] = dict(zip(['r', 'g', 'b'], [mat.emit * mat.specular_intensity * num for num in list(mat.specular_color)]))
    material['shininess'] = mat.specular_hardness/2
    if mat.use_transparency:
        material['transparency'] = 1.0-mat.alpha
    for tex in mat.texture_slots:  # there are always 18 slots, regardless of whether they are filled or not
        if tex is not None:
            try:
                if tex.use_map_color_diffuse:  # regular diffuse color texture
                    material['diffuseTexture'] = mat.texture_slots[0].texture.image.filepath.replace('//', '') # grab the first texture
                if tex.use_map_normal:  # normal map
                    material['normalTexture'] = mat.texture_slots[0].texture.image.filepath.replace('//', '') # grab the first texture
                if tex.use_map_displacement:  # displacement map
                    material['displacementTexture'] = mat.texture_slots[0].texture.image.filepath.replace('//', '') # grab the first texture
            except (KeyError, AttributeError):
                print('None or incomplete texture data for material ' + namingUtils.getObjectName(mat) + '.')
    return material


def deriveLink(obj):
    """This function derives a link from a blender object and creates its initial phobos data structure.

    :param obj: The blender object to derive the link from.
    :type obj: bpy_types.Object
    :return: dict

    """
    props = initObjectProperties(obj, phobostype='link', ignoretypes=['joint', 'motor'])
    props["pose"] = deriveObjectPose(obj)
    props["collision"] = {}
    props["visual"] = {}
    props["inertial"] = {}
    props['approxcollision'] = []
    return props


def deriveJoint(obj):
    """This function derives a joint from a blender object and creates its initial phobos data structure.

    :param obj: The blender object to derive the joint from.
    :return: dict

    """
    if not 'joint/type' in obj.keys():
        jt, crot = joints.deriveJointType(obj, adjust=True)
    props = initObjectProperties(obj, phobostype='joint', ignoretypes=['link', 'motor'])
    props['parent'] = namingUtils.getObjectName(obj.parent)
    props['child'] = namingUtils.getObjectName(obj)
    axis, minmax = joints.getJointConstraints(obj)
    if axis:
        props['axis'] = list(axis)
    limits = {}
    if minmax is not None:
        if len(minmax) == 2:  # prismatic or revolute joint, TODO: planar etc.
            limits['lower'] = minmax[0]
            limits['upper'] = minmax[1]
    if 'maxvelocity' in props:
        limits['velocity'] = props['maxvelocity']
        del props['maxvelocity']
    if 'maxeffort' in props:
        limits['effort'] = props['maxeffort']
        del props['maxeffort']
    if limits != {}:
        props['limits'] = limits
    props['state'] = deriveJointState(obj)
    #TODO:
    # - calibration
    # - dynamics
    # - mimic
    # - safety_controller
    return props


def deriveJointState(joint):
    """Calculates the state of a joint from the state of the link armature.
    Note that this is the current state and not the zero state.

    :param joint: The joint(armature) to derive its state from.
    :type joint: bpy_types.Object
    :return: dict
    """
    state = {}
    state['matrix'] = [list(vector) for vector in list(joint.pose.bones[0].matrix_basis)]
    state['translation'] = list(joint.pose.bones[0].matrix_basis.to_translation())
    state['rotation_euler'] = list(joint.pose.bones[0].matrix_basis.to_euler()) #[0:3]
    state['rotation_quaternion'] = list(joint.pose.bones[0].matrix_basis.to_quaternion()) #[0:4]
    # TODO: hard-coding this could prove problematic if we at some point build armatures from multiple bones
    return state


def deriveMotor(obj, joint):
    """This function derives a motor from a object and joint.

    :param obj: The blender object to derive the motor from.
    :type obj: bpy_types.Object
    :param joint: The phobos joint to derive the constraints from.
    :type joint: dict
    :return: dict

    """
    props = initObjectProperties(obj, phobostype='motor', ignoretypes=['link', 'joint'])
    if len(props) > 1:  # if there are any 'motor' tags and not only a name
        props['joint'] = obj['joint/name'] if 'joint/name' in obj else obj.name
        try:
            if props['type'] == 'PID':
                if 'limits' in joint:
                    props['minValue'] = joint['limits']['lower']
                    props['maxValue'] = joint['limits']['upper']
            elif props['type'] == 'DC':
                props['minValue'] = 0
                props['maxValue'] = props["maxSpeed"]
        except KeyError:
            print("###WARNING: no lower and/or upper limit defined for joint", props['joint'])
        return props
    else:
        return None  # return None if no motor is attached


def deriveKinematics(obj):
    """This function takes an object and derives a link, joint and motor from it, if possible.

    :param obj: The object to derive its kinematics from.
    :type obj: bpy_types.Object
    :return: tuple

    """
    link = deriveLink(obj)
    joint = None
    motor = None
    if obj.parent:
        # TODO: here we have to identify root joints and write their properties to SMURF!
        # --> namespacing parent = "blub::blublink1"
        # --> how to mark separate smurfs in phobos (simply modelname?)
        # -> cut models in pieces but adding modelnames
        # -> automatic namespacing
        joint = deriveJoint(obj)
        motor = deriveMotor(obj, joint)
    return link, joint, motor


def deriveGeometry(obj):
    """This function derives the geometry from an object.

    :param obj: The blender object to derive the geometry from.
    :type obj: bpy_types.Object
    :return: dict

    """
    if 'geometry/type' in obj:
        geometry = {'type': obj['geometry/type']}
        gt = obj['geometry/type']
        if gt == 'box':
            geometry['size'] = list(obj.dimensions)
        elif gt == 'cylinder':
            geometry['radius'] = obj.dimensions[0]/2
            geometry['length'] = obj.dimensions[2]
        elif gt == 'sphere':
            geometry['radius'] = obj.dimensions[0]/2
        elif gt == 'mesh':
            filename = obj.data.name
            if bpy.data.worlds[0].useObj:
                filename += ".obj"
            elif bpy.data.worlds[0].useBobj:
                filename += ".bobj"
            elif bpy.data.worlds[0].useStl:
                filename += ".stl"
            elif bpy.data.worlds[0].useDae:
                filename += ".dae"
            else:
                filename += ".obj"
            geometry['filename'] = os.path.join('meshes', filename)
            geometry['scale'] = list(obj.scale)
            geometry['size'] = list(obj.dimensions)  # this is needed to calculate an approximate inertia
        return geometry
    else:
        warnings.warn("No geometry/type found for object "+namingUtils.getObjectName(obj)+".")
        return None


def deriveInertial(obj):
    """This function derives the inertial from the given object.

    :param obj: The object to derive the inertial from.
    :type obj: bpy_types.Object
    :return: tuple
    """
    props = initObjectProperties(obj, phobostype='inertial')
    props['inertia'] = list(map(float, obj['inertial/inertia']))
    props['pose'] = deriveObjectPose(obj)
    return props, obj.parent


def deriveObjectPose(obj):
    """This function derives a pose of link, visual or collision object.

    :param obj: The blender object to dereive the pose from.
    :type obj: bpy_types.Object
    :return: dict

    """
    pose = {}
    pose['matrix'] = [list(vector) for vector in list(obj.matrix_local)]
    pose['translation'] = list(obj.matrix_local.to_translation())
    pose['rotation_euler'] = list(obj.matrix_local.to_euler())
    pose['rotation_quaternion'] = list(obj.matrix_local.to_quaternion())
    return pose


def deriveVisual(obj):
    """This function derives the visual information from an object.

    :param obj: The blender object to derive the visuals from.
    :type obj: bpy_types.Object
    :return: dict

    """
    visual = initObjectProperties(obj, phobostype='visual', ignoretypes='geometry')
    visual['geometry'] = deriveGeometry(obj)
    visual['pose'] = deriveObjectPose(obj)
    if obj.lod_levels:
        if 'lodmaxdistances' in obj:
            maxdlist = obj['lodmaxdistances']
        else:
            maxdlist=[obj.lod_levels[i+1].distance for i in range(len(obj.lod_levels)-1)]+[100.0]
        lodlist = []
        for i in range(len(obj.lod_levels)):
            filename = obj.lod_levels[i].object.data.name
            if bpy.data.worlds[0].useObj:
                filename += ".obj"
            elif bpy.data.worlds[0].useBobj:
                filename += ".bobj"
            elif bpy.data.worlds[0].useStl:
                filename += ".stl"
            elif bpy.data.worlds[0].useDae:
                filename += ".dae"
            else:
                filename += ".obj"
            lodlist.append({'start': obj.lod_levels[i].distance, 'end': maxdlist[i], 'filename': os.path.join('meshes', filename)})
        visual['lod'] = lodlist
    #if obj.data.materials:
    #    visual['material'] = deriveMaterial(obj.data.materials[0]) #this is now centralized!
    return visual, obj.parent


def deriveCollision(obj):
    """This function derives the collision information from an object.

    :param obj: The blender object to derive the collision information from.
    :type obj: bpy_types.Object
    :return: dict

    """
    collision = initObjectProperties(obj, phobostype='collision', ignoretypes='geometry')
    collision['geometry'] = deriveGeometry(obj)
    collision['pose'] = deriveObjectPose(obj)
    # the bitmask is cut to length = 16 and reverted for int parsing
    try:
        collision['bitmask'] = int(''.join(['1' if group else '0' for group in obj.rigid_body.collision_groups[:16]])[::-1], 2)
    except AttributeError:
        pass
    return collision, obj.parent


def deriveCapsule(obj):
    """This function derives a capsule from a given blender object

    :param obj: The blender object to derive the capsule from.
    :type obj: bpy_types.Object
    :return: tuple

    """
    viscol_dict = {}
    capsule_pose = deriveObjectPose(obj)
    rotation = capsule_pose['rotation_euler']
    capsule_radius = obj['radius']
    for part in ['sphere1', 'cylinder', 'sphere2']:
        viscol = initObjectProperties(obj, phobostype='collision', ignoretypes='geometry')
        viscol['name'] = namingUtils.getObjectName(obj).split(':')[-1] + '_' + part
        geometry = {}
        pose = {}
        geometry['radius'] = capsule_radius
        if part == 'cylinder':
            geometry['length'] = obj['length']
            geometry['type'] = 'cylinder'
            pose = capsule_pose
        else:
            geometry['type'] = 'sphere'
            if part == 'sphere1':
                location = obj['sph1_location']
            else:
                location = obj['sph2_location']
            pose['translation'] = location
            pose['rotation_euler'] = rotation
            loc_mu = mathutils.Matrix.Translation(location)
            rot_mu = mathutils.Euler(rotation).to_quaternion()
            pose['rotation_quaternion'] = list(rot_mu)
            matrix = loc_mu * rot_mu.to_matrix().to_4x4()
            #print(list(matrix))
            pose['matrix'] = [list(vector) for vector in list(matrix)]
        viscol['geometry'] = geometry
        viscol['pose'] = pose
        try:
            viscol['bitmask'] = int(''.join(['1' if group else '0' for group in obj.rigid_body.collision_groups]), 2)
        except AttributeError:
            pass
        viscol_dict[part] = viscol
    return viscol_dict, obj.parent



def deriveApproxsphere(obj):
    """This function derives a approxsphere from a given blender object

    :param obj: The blender object to derive the approxsphere from.
    :type obj: bpy_types.Object
    :return: tuple

    """

    sphere = initObjectProperties(obj)
    sphere['radius'] = obj.dimensions[0]/2
    sphere['center'] = list(obj.matrix_local.to_translation())
    return sphere, obj.parent


def deriveSensor(obj):
    """This function derives a sensor from a given blender object

    :param obj: The blender object to derive the sensor from.
    :type obj: bpy_types.Object
    :return: dict

    """
    props = initObjectProperties(obj, phobostype='sensor')
    #props['pose'] = deriveObjectPose(obj)
    props['link'] = namingUtils.getObjectName(obj.parent)
    return props


def deriveController(obj):
    """This function derives a controller from a given blender object

    :param obj: The blender object to derive the controller from.
    :type obj: bpy_types.Object
    :return: dict

    """
    props = initObjectProperties(obj, phobostype='controller')
    return props

def deriveLight(obj):
    """This function derives a light from a given blender object

    :param obj: The blender object to derive the light from.
    :type obj: bpy_types.Object
    :return: tuple

    """
    light = initObjectProperties(obj, phobostype='light')
    light_data = obj.data
    if light_data.use_diffuse:
        light['color_diffuse'] = list(light_data.color)
    if light_data.use_specular:
        light['color_specular'] = copy.copy(light['color_diffuse'])
    light['type'] = light_data.type.lower()
    if light['type'] == 'SPOT':
        light['size'] = light_data.size
    light['position'] =  list(obj.matrix_local.to_translation())
    light['rotation'] = list(obj.matrix_local.to_euler())
    try:
        light['attenuation_linear'] = float(light_data.linear_attenuation)
    except AttributeError:
        pass
    try:
        light['attenuation_quadratic'] = float(light_data.quadratic_attenuation)
    except AttributeError:
        pass
    if light_data.energy:
        light['attenuation_constant'] = float(light_data.energy)

    if obj.parent is not None:
        light['parent'] = namingUtils.getObjectName(obj.parent,phobostype="link")

    return light

def initObjectProperties(obj, phobostype=None, ignoretypes=[]):
    """This function initializes a phobos data structure with a given object
    and derives basic information from its custom properties.

    :param obj: The object to derive initial properties from.
    :type obj: bpy_types.Object
    :param phobostype: This parameter can specify the type of the given object to include more specific information.
    :type phobostype: str
    :param ignoretypes: This list contains properties that should be ignored while initializing the objects properties.
    :type ignoretypes: list
    :return: dict

    """
    props = {'name': namingUtils.getObjectName(obj).split(':')[-1]}  #allow duplicated names differentiated by types
    if not phobostype:
        for key, value in obj.items():
            props[key] = value
    else:
        for key, value in obj.items():
            if hasattr(value, 'to_list'):  # transform Blender id_arrays into lists
                value = list(value)
            if '/' in key:
                if phobostype+'/' in key:
                    specs = key.split('/')[1:]
                    if len(specs) == 1:
                        props[key.replace(phobostype+'/', '')] = value
                    elif len(specs) == 2:
                        category, specifier = specs
                        if '$'+category not in props:
                            props['$'+category] = {}
                        props['$'+category][specifier] = value
                elif key.count('/') == 1: #ignore two-level specifiers if phobostype is not present
                    category, specifier = key.split('/')
                    if category not in ignoretypes:
                        if '$'+category not in props:
                            props['$'+category] = {}
                        props['$'+category][specifier] = value
    return props


def deriveDictEntry(obj):
    """This function takes a blender object and derives the phobos structure from it
    depending on the objects phobostype.

    :param obj: The object to derive the dict entry (phobos data structure) from.
    :type obj: bpy_types.Object
    :return: tuple

    """
    print(namingUtils.getObjectName(obj), end=', ')
    props = None
    parent = None
    try:
        if obj.phobostype == 'inertial':
            props, parent = deriveInertial(obj)
        elif obj.phobostype == 'visual':
            props, parent = deriveVisual(obj)
        elif obj.phobostype == 'collision':
            if obj['geometry/type'] == 'capsule':
                props, parent = deriveCapsule(obj)
            else:
                props, parent = deriveCollision(obj)
        elif obj.phobostype == 'approxsphere':
            props, parent = deriveApproxsphere(obj)
        elif obj.phobostype == 'sensor':
            props = deriveSensor(obj)
        elif obj.phobostype == 'controller':
            props = deriveController(obj)
        elif obj.phobostype == 'light':
            props, parent = deriveLight(obj)
    except KeyError:
        print('phobos: A KeyError occurred, likely due to missing information in the model:\n    ', sys.exc_info()[0])
    if obj.phobostype in ['sensor', 'controller']:
        return props
    else:
        return props, parent


def deriveGroupEntry(group):
    """This function gets a blender group and creates a list of phobos link skeletons out of the groups objects.

    :param group: The blender group to extract the links from.
    :type group: bpy_types.Group
    :return: list

    """
    links = []
    for obj in group.objects:
        if obj.phobostype == 'link':
            links.append({'type': 'link', 'name': namingUtils.getObjectName(obj)})
        else:
            print("### Error: group " + namingUtils.getObjectName(group) + " contains " + obj.phobostype + ': ' + namingUtils.getObjectName(obj))
    return links


def deriveChainEntry(obj):
    """TODO: Please add pyDoc ASAP.

    """
    returnchains = []
    if 'endChain' in obj:
        chainlist = obj['endChain']
    for chainName in chainlist:
        chainclosed = False
        parent = obj
        chain = {'name': chainName, 'start': '', 'end': namingUtils.getObjectName(obj), 'elements': []}
        while not chainclosed:
            if parent.parent is None:
                print('### Error: Unclosed chain, aborting parsing chain', chainName)
                chain = None
                break
            chain['elements'].append(parent.name)
            parent = parent.parent
            if 'startChain' in parent:
                startChain = parent['startChain']
                if chainName in startChain:
                    chain['start'] = namingUtils.getObjectName(parent)
                    chain['elements'].append(namingUtils.getObjectName(parent))
                    chainclosed = True
        if chain is not None:
            returnchains.append(chain)
    return returnchains


def storePose(robot_name, pose_name):
    """
    Store the current pose of all of a robot's selected links.
    Existing poses of the same name will be overwritten.

    :param robot_name: The robot the pose belongs to.
    :type robot_name: str.
    :param pose_name: The name the pose will be stored under.
    :type pose_name: str.
    :return: Nothing.
    """
    file_name = 'robot_poses_' + robot_name
    load_file = blenderUtils.readTextFile(file_name)
    if load_file == '':
        poses = {}
    else:
        poses = yaml.load(load_file)
    new_pose = {}
    prev_mode = bpy.context.mode
    bpy.ops.object.mode_set(mode='POSE')
    for root in selectionUtils.getRoots():
        if root['modelname'] == robot_name:
            links = selectionUtils.getChildren(root)
            for link in links:
                if link.select and link.phobostype == 'link':
                    link.pose.bones['Bone'].rotation_mode = 'XYZ'
                    new_pose[namingUtils.getObjectName(link, 'joint')] = link.pose.bones['Bone'].rotation_euler.y
    bpy.ops.object.mode_set(mode=prev_mode)
    poses[pose_name] = new_pose
    blenderUtils.updateTextFile(file_name, yaml.dump(poses))


def loadPose(robot_name, pose_name):
    """
    Load and apply a robot's stored pose.

    :param robot_name: The robot's name.
    :type robot_name: str.
    :param pose_name: The name the pose is stored under.
    :type pose_name: str.
    :return Nothing.
    """
    load_file = blenderUtils.readTextFile('robot_poses_' + robot_name)
    if load_file == '':
        log('No poses stored.', 'ERROR')
        return
    poses = yaml.load(load_file)
    if pose_name in poses:
        prev_mode = bpy.context.mode
        bpy.ops.object.mode_set(mode='POSE')
        for obj in selectionUtils.returnObjectList('link'):
            if namingUtils.getObjectName(obj, 'joint') in poses[pose_name]:
                obj.pose.bones['Bone'].rotation_mode = 'XYZ'
                obj.pose.bones['Bone'].rotation_euler.y = poses[pose_name][namingUtils.getObjectName(obj, 'joint')]
        bpy.ops.object.mode_set(mode=prev_mode)
    #else:
    #    log('No pose with name ' + pose_name + ' stored for robot ' + robot_name + '.', 'ERROR')


def get_poses(robot_name):
    """
    Get the names of the poses that have been stored for a robot.

    :param robot_name: The robot's name.
    :return: A list containing the poses' names.
    """
    load_file = blenderUtils.readTextFile('robot_poses_' + robot_name)
    if load_file == '':
        return []
    poses = yaml.load(load_file)
    return poses.keys()


def deriveStoredPoses():
    """
    Collect the poses that have been stored for the scene's robots.

    :return: A dictionary containing the poses.
    """
    poses_dict = {}
    for text in bpy.data.texts:
        file_name = text.name
        if file_name.startswith('robot_poses_'):
            robot_name = file_name[len('robot_poses_'):]
            poses_file = blenderUtils.readTextFile(file_name)
            if poses_file == '':
                poses_dict[robot_name] = {}
                break
            poses = yaml.load(poses_file)
            pose_dict = {}
            for pose in poses:
                new_pose = {}
                new_pose['name'] = pose
                new_pose['joints'] = poses[pose]
                pose_dict[pose] = new_pose
            poses_dict[robot_name] = pose_dict
    return poses_dict


def buildRobotDictionary():
    """Builds a python dictionary representation of a Blender robot model for export and inspection.

    """
    objectlist = bpy.context.selected_objects
    #notifications, faulty_objects = robotupdate.updateModel(bpy.context.selected_objects)
    #print(notifications)
    robot = {'links': {},
            'joints': {},
            'sensors': {},
            'motors': {},
            'controllers': {},
            'materials': {},
            'groups': {},
            'chains': {},
            'lights': {}
            }
    #save timestamped version of model
    robot["date"] = datetime.now().strftime("%Y%m%d_%H:%M")
    root = selectionUtils.getRoot(bpy.context.selected_objects[0])
    if root.phobostype != 'link':
        raise Exception("Found no 'link' object as root of the robot model.")
    else:
        if 'modelname' in root:
            robot['modelname'] = root["modelname"]
        else:
            robot['modelname'] = 'unnamed_robot'

    # digest all the links to derive link and joint information
    print('\nParsing links, joints and motors...')
    for obj in bpy.context.selected_objects:
        if obj.phobostype == 'link':
            link, joint, motor = deriveKinematics(obj)
            robot['links'][namingUtils.getObjectName(obj, phobostype="link")] = link  # it's important that this is really the object's name
            if joint:  # joint can be None if link is a root
                robot['joints'][joint['name']] = joint
            if motor:
                robot['motors'][joint['name']] = motor
            obj.select = False

    # add inertial information to link
    print('\n\nParsing inertials...')
    for l in robot['links']:
        #link = bpy.data.objects[l] NEW NAMING!
        link = selectionUtils.getObjectByName(l)[0] if selectionUtils.getObjectByName(l) is not None else "ERROR!"
        inertials = selectionUtils.getImmediateChildren(link, ['inertial'])
        if len(inertials) == 1:
            props, parent = deriveDictEntry(inertials[0])
            if not (props is None or parent is None):  # this may be the case if there is inertia information missing
                robot['links'][namingUtils.getObjectName(parent)]['inertial'] = props
            inertials[0].select = False
        elif len(inertials) > 1:
            for i in inertials:
                if i.name == 'inertial_' + l:
                    props, parent = deriveDictEntry(i)
                    robot['links'][namingUtils.getObjectName(parent, phobostype="link")]['inertial'] = props
            # FIXME: this has to be re-implemented
            #if linkinertial == None:
            #    mass, com, inertia = inertia.fuseInertiaData(inertials)
            #    parent = inertials[0].parent
            #    matrix_local = mathutils.Matrix.Translation(mathutils.Vector(com))
            #    pose = {}
            #    pose['matrix'] = [list(vector) for vector in list(matrix_local)]
            #    pose['translation'] = list(matrix_local.to_translation())
            #    pose['rotation_euler'] = list(matrix_local.to_euler())
            #    pose['rotation_quaternion'] = list(matrix_local.to_quaternion())
            #    props = {'mass': mass, 'pose': pose, 'inertia': inertia}
            #    robot['links'][parent.name]['inertial'] = props
            for i in inertials:
                i.select = False

    # complete link information by parsing visuals and collision objects
    print('\n\nParsing visual and collision (approximation) objects...')
    capsules_list = []
    for obj in bpy.context.selected_objects:
        print("Parsing object " + namingUtils.getObjectName(obj))
        if obj.phobostype in ['visual', 'collision']:
            props, parent = deriveDictEntry(obj)
            if all([key in props for key in ['cylinder', 'sphere1', 'sphere2']]):     # this is the case with simulated capsules
                capsules_list.append({'link': parent.name,
                                      'name': props['cylinder']['name'][:-len('_cylinder')],
                                      'radius': props['cylinder']['geometry']['radius'],
                                      'length': props['cylinder']['geometry']['length'] + 2*props['cylinder']['geometry']['radius'],
                                      #'bitmask': props['cylinder']['bitmask']
                                    })
                for key in props:
                    robot['links'][namingUtils.getObjectName(parent, phobostype="link")][obj.phobostype][key] = props[key]
            else:
                robot['links'][namingUtils.getObjectName(parent, phobostype="link")][obj.phobostype][namingUtils.getObjectName(obj, phobostype=obj.phobostype)] = props
            obj.select = False
        elif obj.phobostype == 'approxsphere':
            props, parent = deriveDictEntry(obj)
            robot['links'][namingUtils.getObjectName(parent)]['approxcollision'].append(props)
            obj.select = False

    robot['capsules'] = capsules_list

    # combine collision information for links
    for linkname in robot['links']:
        link = robot['links'][linkname]
        bitmask = 0
        for collname in link['collision']:
            try:
                bitmask = bitmask | link['collision'][collname]['bitmask']
            except KeyError:
                pass
        link['collision_bitmask'] = bitmask

    # parse sensors and controllers
    print('\n\nParsing sensors and controllers...')
    for obj in bpy.context.selected_objects:
        if obj.phobostype in ['sensor', 'controller']:
            robot[obj.phobostype+'s'][namingUtils.getObjectName(obj)] = deriveDictEntry(obj)
            obj.select = False

    # parse materials
    print('\n\nParsing materials...')
    robot['materials'] = collectMaterials(objectlist)
    for obj in objectlist:
        if obj.phobostype == 'visual' and len(obj.data.materials) > 0:
            mat = obj.data.materials[0]
            if not namingUtils.getObjectName(mat) in robot['materials']:
                robot['materials'][namingUtils.getObjectName(mat)] = deriveMaterial(mat) #this should actually never happen
            robot['links'][namingUtils.getObjectName(obj.parent)]['visual'][namingUtils.getObjectName(obj, phobostype="visual")]['material'] = namingUtils.getObjectName(mat)

    # gather information on groups of objects
    print('\n\nParsing groups...')
    for group in bpy.data.groups:  # TODO: get rid of the "data" part
        if len(group.objects) > 0 and namingUtils.getObjectName(group) != "RigidBodyWorld":
            robot['groups'][namingUtils.getObjectName(group)] = deriveGroupEntry(group)

    # gather information on chains of objects
    print('\n\nParsing chains...')
    chains = []
    for obj in bpy.data.objects:
        if obj.phobostype == 'link' and 'endChain' in obj:
            chains.extend(deriveChainEntry(obj))
    for chain in chains:
        robot['chains'][chain['name']] = chain

    # gather information on global lights
    print('\n\nParsing lights...')
    for obj in bpy.context.selected_objects:
        if obj.phobostype == 'light':
            robot['lights'][namingUtils.getObjectName(obj)] = deriveLight(obj)

    robot['poses'] = deriveStoredPoses()

    #shorten numbers in dictionary to n decimalPlaces and return it
    print('\n\nRounding numbers...')
    epsilon = 10**(-bpy.data.worlds[0].decimalPlaces)  # TODO: implement this separately
    return generalUtils.epsilonToZero(robot, epsilon, bpy.data.worlds[0].decimalPlaces)

