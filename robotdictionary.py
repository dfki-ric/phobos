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
import sys
import datetime
import warnings
import phobos.defs as defs
import phobos.joints as joints
from phobos.utility import *


def register():
    print("Registering export...")


def collectMaterials(objectlist):
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
    try:
        material['texturename'] = mat.texture_slots[0].texture.image.name # grab the first texture
    except (KeyError, AttributeError):
        print('None or incomplete texture data for material ' + mat.name + '.')
    return material


def deriveLink(obj):
    props = initObjectProperties(obj, phobostype='link', ignoretypes=['joint', 'motor'])
    props["pose"] = deriveObjectPose(obj)
    props["collision"] = {}
    props["visual"] = {}
    props["inertial"] = {}
    props['approxcollision'] = []
    return props


def deriveJoint(obj):
    if not 'joint/type' in obj.keys():
        jt, crot = joints.deriveJointType(obj, adjust=True)
    props = initObjectProperties(obj, phobostype='joint', ignoretypes=['link', 'motor'])
    props['parent'] = obj.parent.name
    props['child'] = getObjectName(obj)
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
    Note that this is the current state and not the zero state."""
    state = {}
    state['matrix'] = [list(vector) for vector in list(joint.pose.bones[0].matrix_basis)]
    state['translation'] = list(joint.pose.bones[0].matrix_basis.to_translation())
    state['rotation_euler'] = list(joint.pose.bones[0].matrix_basis.to_euler()) #[0:3]
    state['rotation_quaternion'] = list(joint.pose.bones[0].matrix_basis.to_quaternion()) #[0:4]
    # TODO: hard-coding this could prove problematic if we at some point build armatures from multiple bones
    return state


def deriveMotor(obj, joint):
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
        elif gt in ['mesh', 'capsule']:
            sMProp = 'geometry/'+defs.reservedProperties['SHAREDMESH']
            if sMProp in obj:
                filename = obj[sMProp]
            elif 'geometry/filename' in obj:
                filename = obj['geometry/filename']
            elif 'filename' in obj:
                filename = obj['filename']
            else:
                filename = getObjectName(obj).replace('/', '_')
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
            geometry['filename'] = os.path.join(bpy.data.worlds[0].meshpath, filename)
            geometry['scale'] = list(obj.scale)
            geometry['size'] = list(obj.dimensions)  # this is needed to calculate an approximate inertia
        return geometry
    else:
        warnings.warn("No geometry/type found for object "+getObjectName(obj)+".")
        return None


def deriveInertial(obj):
    """Derives a dictionary entry of an inertial object."""
    props = initObjectProperties(obj, phobostype='inertial')
    props['inertia'] = list(map(float, obj['inertial/inertia']))
    props['pose'] = deriveObjectPose(obj)
    return props, obj.parent


def deriveObjectPose(obj):
    """Derive pose of link, visual or collision object."""
    pose = {}
    pose['matrix'] = [list(vector) for vector in list(obj.matrix_local)]
    pose['translation'] = list(obj.matrix_local.to_translation())
    pose['rotation_euler'] = list(obj.matrix_local.to_euler())
    pose['rotation_quaternion'] = list(obj.matrix_local.to_quaternion())
    return pose


def deriveVisual(obj):
    visual = initObjectProperties(obj, phobostype='visual', ignoretypes='geometry')
    visual['geometry'] = deriveGeometry(obj)
    visual['pose'] = deriveObjectPose(obj)
    #if obj.data.materials:
    #    visual['material'] = deriveMaterial(obj.data.materials[0]) #this is now centralized!
    return visual, obj.parent


def deriveCollision(obj):
    collision = initObjectProperties(obj, phobostype='collision', ignoretypes='geometry')
    collision['geometry'] = deriveGeometry(obj)
    collision['pose'] = deriveObjectPose(obj)
    try:
        collision['bitmask'] = int(''.join(['1' if group else '0' for group in obj.rigid_body.collision_groups]), 2)
    except AttributeError:
        pass
    return collision, obj.parent


def deriveApproxsphere(obj):
    sphere = initObjectProperties(obj)
    sphere['radius'] = obj.dimensions[0]/2
    sphere['center'] = list(obj.matrix_local.to_translation())
    return sphere, obj.parent


def deriveSensor(obj):
    props = initObjectProperties(obj, phobostype='sensor')
    #props['pose'] = deriveObjectPose(obj)
    props['link'] = obj.parent.name
    return props


def deriveController(obj):
    props = initObjectProperties(obj, phobostype='controller')
    return props

def deriveLight(obj):
    light = initObjectProperties(obj, phobostype='light')
    light_data = obj.data
    light['color'] = {}
    light['color']['diffuse'] = light_data.color
    if light_data.type == 'SPOT':
        light['type'] = 'omnilight'
        light['angle'] = light_data.spot_size
    elif light_data.type == 'POINT':
        light['type'] = 'spotlight'
    else:
        #TODO: error
        pass
    light['position'] = obj.location
    rotation = mathutils.Vector(obj.rotation_euler).to_matrix()
    direction = mathutils.Vector((1, 0, 0)) * rotation
    light['direction'] = direction

    light['attenuation'] = {}
    light['attenuation']['linear'] = light_data.linear_attenuation
    light['attenuation']['quadratic'] = light_data.quadratic_attenuation
    light['attenuation']['constant'] = light_data.energy

    return light, obj.parent

def initObjectProperties(obj, phobostype=None, ignoretypes=[]):
    props = {'name': getObjectName(obj).split(':')[-1]}  #allow duplicated names differentiated by types
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
    print(getObjectName(obj), end=', ')
    props = None
    parent = None
    try:
        if obj.phobostype == 'inertial':
            props, parent = deriveInertial(obj)
        elif obj.phobostype == 'visual':
            props, parent = deriveVisual(obj)
        elif obj.phobostype == 'collision':
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
    links = []
    for obj in group.objects:
        if obj.phobostype == 'link':
            links.append({'type': 'link', 'name': getObjectName(obj)})
        else:
            print("### Error: group " + group.name + " contains " + obj.phobostype + ': ' + getObjectName(obj))
    return links


def deriveChainEntry(obj):
    returnchains = []
    if 'endChain' in obj:
        chainlist = obj['endChain']
    for chainName in chainlist:
        chainclosed = False
        parent = obj
        chain = {'name': chainName, 'start': '', 'end': getObjectName(obj), 'elements': []}
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
                    chain['start'] = parent.name
                    chain['elements'].append(parent.name)
                    chainclosed = True
        if chain is not None:
            returnchains.append(chain)
    return returnchains


def deriveStoredPoses():
    """
    """
    poses_dict = {}
    if len(bpy.data.actions) == 0:
        return {}
    pose_lib_name = bpy.data.actions.keys()[0]
    some_obj = bpy.context.scene.objects.active
    bpy.ops.object.mode_set(mode='OBJECT')
    for pose_name, i in zip(bpy.data.actions[pose_lib_name].pose_markers.keys(), range(len(bpy.data.actions[pose_lib_name].pose_markers.keys()))):
        selectObjects([getRoot(some_obj)], clear=True, active=0)
        pose_dict = {}
        bpy.ops.object.mode_set(mode='POSE')
        bpy.ops.poselib.apply_pose(pose_index=i)
        bpy.ops.object.mode_set(mode='OBJECT')
        for obj in bpy.context.scene.objects:
            if obj.phobostype == 'link':
                selectObjects([obj], clear=True, active=0)
                bpy.ops.object.mode_set(mode='POSE')
                obj.pose.bones['Bone'].rotation_mode = 'XYZ'
                y_angle = obj.pose.bones['Bone'].rotation_euler.y
                bpy.ops.object.mode_set(mode='OBJECT')
                pose_dict[obj.name] = y_angle
        poses_dict[pose_name] = pose_dict

    return poses_dict


def buildRobotDictionary():
    """Builds a python dictionary representation of a Blender robot model for export and inspection."""
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
    root = getRoot(bpy.context.selected_objects[0])
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
            robot['links'][obj.name] = link  # it's important that this is really the object's name
            if joint:  # joint can be None if link is a root
                robot['joints'][joint['name']] = joint
            if motor:
                robot['motors'][joint['name']] = motor
            obj.select = False

    # add inertial information to link
    print('\n\nParsing inertials...')
    for l in robot['links']:
        link = bpy.data.objects[l]
        inertials = getImmediateChildren(link, ['inertial'])
        if len(inertials) == 1:
            props, parent = deriveDictEntry(inertials[0])
            if not (props is None or parent is None):  # this may be the case if there is inertia information missing
                robot['links'][parent.name]['inertial'] = props
            inertials[0].select = False
        elif len(inertials) > 1:
            for i in inertials:
                if i.name == 'inertial_' + l:
                    props, parent = deriveDictEntry(i)
                    robot['links'][parent.name]['inertial'] = props
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
    for obj in bpy.context.selected_objects:
        if obj.phobostype in ['visual', 'collision']:
            props, parent = deriveDictEntry(obj)
            robot['links'][parent.name][obj.phobostype][getObjectName(obj)] = props
            obj.select = False
        elif obj.phobostype == 'approxsphere':
            props, parent = deriveDictEntry(obj)
            robot['links'][parent.name]['approxcollision'].append(props)
            obj.select = False

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
            robot[obj.phobostype+'s'][getObjectName(obj)] = deriveDictEntry(obj)
            obj.select = False

    # parse materials
    print('\n\nParsing materials...')
    robot['materials'] = collectMaterials(objectlist)
    for obj in objectlist:
        if obj.phobostype == 'visual' and len(obj.data.materials) > 0:
            mat = obj.data.materials[0]
            if not mat.name in robot['materials']:
                robot['materials'][mat.name] = deriveMaterial(mat) #this should actually never happen
            robot['links'][obj.parent.name]['visual'][getObjectName(obj)]['material'] = mat.name

    # gather information on groups of objects
    print('\n\nParsing groups...')
    for group in bpy.data.groups:  # TODO: get rid of the "data" part
        if len(group.objects) > 0 and group.name != "RigidBodyWorld":
            robot['groups'][group.name] = deriveGroupEntry(group)

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
            robot['lights'][getObjectName(obj)] = deriveLight(obj)

    robot['poses'] = deriveStoredPoses()

    #shorten numbers in dictionary to n decimalPlaces and return it
    print('\n\nRounding numbers...')
    epsilon = 10**(-bpy.data.worlds[0].decimalPlaces)  # TODO: implement this separately
    return epsilonToZero(robot, epsilon, bpy.data.worlds[0].decimalPlaces)


def check_geometry(geometry, owner_type, owner_key, link_key):
    """
    """
    notifications = ''
    if not 'type' in geometry:
        note = "CheckModel: Error, geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'type'."
        notifications += note + "\n"
        print(note)
    else:
        if geometry['type'] == 'box' or geometry['type'] == 'plane':
            if not 'size' in geometry:
                note = "CheckModel: Error, box / plane type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'size'."
                notifications += note + "\n"
                print(note)
        elif geometry['type'] == 'sphere':
            if not 'radius' in geometry:
                note = "CheckModel: Error, sphere type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'radius'."
                notifications += note + "\n"
                print(note)
        elif geometry['type'] == 'cylinder':
            if not 'radius' in geometry:
                note = "CheckModel: Error, cylinder type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'radius'."
                notifications += note + "\n"
                print(note)
            if not 'length' in geometry:
                note = "CheckModel: Error, cylinder type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'length'."
                notifications += note + "\n"
                print(note)
        elif geometry['type'] == 'mesh':
            if not 'size' in geometry:
                note = "CheckModel: Error, mesh type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'size'."
                notifications += note + "\n"
                print(note)
            if not 'filename' in geometry:
                note = "CheckModel: Error, mesh type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'filename'."
                notifications += note + "\n"
                print(note)
        else:
            note = "CheckModel: Error, geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has invalid value for attribute 'type': '" + geometry['type'] + "'."
            notifications += note + "\n"
            print(note)
    return notifications


def check_visuals(visuals, link_key):
    """
    """
    notifications = ''
    for visual_key in visuals.keys():
        visual = visuals[visual_key]
        if not 'pose' in visual:
            note = "CheckModel: Error, visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'pose'."
            notifications += note + "\n"
            print(note)
        if not 'name' in visual:
            note = "CheckModel: Error, visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'name'."
            notifications += note + "\n"
            print(note)
        if not 'material' in visual:
            note = "CheckModel: Warning, visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'material'."
            notifications += note + "\n"
            print(note)
        else:
            material = visual['material']
            if not 'name' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'name'."
                notifications += note + "\n"
                print(note)
            if not 'diffuseColor' in material:
                note = "CheckModel: Error, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'diffuseColor'."
                notifications += note + "\n"
                print(note)
            if not 'ambientColor' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'ambientColor'."
                notifications += note + "\n"
                print(note)
            if not 'emissionColor' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'emissionColor'."
                notifications += note + "\n"
                print(note)
            if not 'specularColor' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'specularColor'."
                notifications += note + "\n"
                print(note)
            if not 'transparency' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'transparency'."
                notifications += note + "\n"
                print(note)
        if not 'geometry' in visual:
            note = "CheckModel: Error, visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'geometry'."
            notifications += note + "\n"
            print(note)
        else:
            notifications += check_geometry(visual['geometry'], 'visual', visual_key, link_key)
    return notifications


def check_collisions(collisions, link_key):
    """
    """
    notifications = ''
    for collision_key in collisions.keys():
        collision = collisions[collision_key]
        if not 'bitmask' in collision:
            note = "CheckModel: Warning, collision '" + collision_key + "' has no attribute 'bitmask'."
            notifications += note + "\n"
            print(note)
        if not 'name' in collision:
            note = "CheckModel: Warning, collision '" + collision_key + "' has no attribute 'name'."
            notifications += note + "\n"
            print(note)
        if not 'geometry' in collision:
            note = "CheckModel: Error, collision '" + collision_key + "' of link '" + link_key + "' has no attribute 'geometry'."
            notifications += note + "\n"
            print(note)
        else:
            notifications += check_geometry(collision['geometry'], 'collision', collision_key, link_key)
        if not 'pose' in collision:
            note = "CheckModel: Error, collision '" + collision_key + "' has no attribute 'pose'."
            notifications += note + "\n"
            print(note)
        if not 'max_contacts' in collision:
            note = "CheckModel: Warning, collision '" + collision_key + "' has no attribute 'max_contacts'."
            notifications += note + "\n"
            print(note)
        elif collision['max_contacts'] < 1:
            note = "CheckModel: Note, attribute 'max_contacts' in collision '" + collision_key + "' should not be zero or negative."
            notifications += note + "\n"
            print(note)
    return notifications


def check_links(links):
    """
    """
    notifications = ''
    for link_key in links.keys():
        link = links[link_key]
        '''if not 'filename' in link:
            note = "CheckModel: Error, link '" + link_key + "' has no attribute 'filename'."
            notifications += note + "\n"
            print(note)'''#not included in the model - date 13.10.2014
        if not 'pose' in link:
            note = "CheckModel: Error, link '" + link_key + "' has no attribute 'pose'."
            notifications += note + "\n"
            print(note)
        if not 'name' in link:
            note = "CheckModel: Error, link '" + link_key + "' has no attribute 'name'."
            notifications += note + "\n"
            print(note)
        if not 'parent' in link:
            note = "CheckModel: Error, link '" + link_key + "' has no attribute 'parent'."
            notifications += note + "\n"
            print(note)
        if not 'visual' in link:
            note = "CheckModel: Warning, link '" + link_key + "' has no attribute 'visual'."
            notifications += note + "\n"
            print(note)
        else:
            notifications += check_visuals(link['collision'], link_key)
        if not 'collision' in link:
            note = "CheckModel: Warning, link '" + link_key + "' has no attribute 'collision'."
            notifications += note + "\n"
            print(note)
        else:
            notifications += check_collisions(link['collision'], link_key)
        if not 'inertial' in link:
            note = "CheckModel: Warning, link '" + link_key + "' has no attribute 'inertial'."
            notifications += note + "\n"
            print(note)
        else:
            inertial = link['inertial']
            if not 'mass' in inertial:
                note = "CheckModel: Error, inertial in link '" + link_key + "' has no attribute 'mass'."
                notifications += note + "\n"
                print(note)
            if not 'inertia' in inertial:
                note = "CheckModel: Error, inertial in link '" + link_key + "' has no attribute 'inertia'."
                notifications += note + "\n"
                print(note)
    return notifications


def check_joints(joints):
    """
    """
    notifications = ''
    for joint_key in joints.keys():
        joint = joints[joint_key]
        if not'axis' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'axis'."
            notifications += note + "\n"
            print(note)
        if not'limits' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'limits'."
            notifications += note + "\n"
            print(note)
        if not'name' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'name'."
            notifications += note + "\n"
            print(note)
        if not 'parent' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'parent'."
            notifications += note + "\n"
            print(note)
        if not 'child' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'child'."
            notifications += note + "\n"
            print(note)
        if not 'type' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'type'."
            notifications += note + "\n"
            print(note)
        elif joint['type'] not in defs.jointtypes:
            note = "CheckModel: Error, joint '" + joint_key + "' has invalid value for attribute 'joint/type': '" + joint['type'] + "'."
            notifications += note + '\n'
            print(note)
    return notifications


def check_dict(model):
    """
    """
    notifications = ''
    notifications += check_links(model['links'])
    notifications += check_joints(model['joints'])
    return notifications
