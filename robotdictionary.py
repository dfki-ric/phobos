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

import bpy
import mathutils
import sys
import datetime
import warnings
import phobos.defs as defs
import phobos.utility as utility
import phobos.joints as joints
import phobos.inertia as inertia
from phobos.utility import *


def register():
    print("Registering export...")


def collectMaterials(objectlist):
    materials = {}
    for obj in objectlist:
        if obj.MARStype == 'visual' and obj.data.materials:
            mat = obj.data.materials[0]  # simply grab the first material
            if mat.name not in materials:
                materials[mat.name] = deriveMaterial(mat)
                materials[mat.name]['users'] = 1
            else:
                materials[mat.name]['users'] += 1
    return materials


def deriveMaterial(mat):
    material = {}
    material['name'] = mat.name
    material['diffuseFront'] = dict(zip(['r', 'g', 'b'], [mat.diffuse_intensity * num for num in list(mat.diffuse_color)]))
    material['ambientFront'] = dict(zip(['r', 'g', 'b'], [mat.ambient * mat.diffuse_intensity * num for num in list(mat.diffuse_color)]))
    material['specularFront'] = dict(zip(['r', 'g', 'b'], [mat.specular_intensity * num for num in list(mat.specular_color)]))
    if mat.emit > 0:
        material['emissionFront'] = dict(zip(['r', 'g', 'b'], [mat.emit * mat.specular_intensity * num for num in list(mat.specular_color)]))
    material['shininess'] = mat.specular_hardness/2
    if mat.use_transparency:
        material['transparency'] = 1.0-mat.alpha
    try:
        material['texturename'] = mat.texture_slots[0].texture.image.name # grab the first texture
    except (KeyError, AttributeError):
        print('None or incomplete texture data for material ' + mat.name + '.')
    return material


def deriveLink(obj, typetags=False):
    props = initObjectProperties(obj, 'link')
    props["pose"] = deriveObjectPose(obj)
    props["collision"] = {}
    props["visual"] = {}
    props["inertial"] = {}
    props['approxcollision'] = []
    return props


def deriveJoint(obj, typetags=False):
    props = initObjectProperties(obj, 'joint')
    if typetags:
        if not '/' in obj.name:
            props['name'] = obj.name + '_joint'
        else:
            props['name'] = obj.name.replace('/', '/joint')
    else:
        props['name'] = obj.name
    props['parent'] = obj.parent.name
    props['child'] = obj.name
    props['type'], crot = joints.deriveJointType(obj, True)
    axis, limit = joints.getJointConstraints(obj)
    if axis:
        props['axis'] = list(axis)
    if limit:
        props['limits'] = list(limit) # limit gets returned as None if there are no limits
    props["state"] = deriveJointState(obj)
    #TODO:
    # - calibration
    # - dynamics
    # - mimic
    # - safety_controller
    return props


def deriveJointState(joint):
    '''Calculates the state of a joint from the state of the link armature.
    Note that this is the current state and not the zero state.'''
    state = {}
    state['matrix'] = [list(vector) for vector in list(joint.pose.bones[0].matrix_basis)]
    state['translation'] = list(joint.pose.bones[0].matrix_basis.to_translation())
    state['rotation_euler'] = list(joint.pose.bones[0].matrix_basis.to_euler()) #[0:3]
    state['rotation_quaternion'] = list(joint.pose.bones[0].matrix_basis.to_quaternion()) #[0:4]
    # TODO: hard-coding this could prove problematic if we at some point build armatures from multiple bones
    return state


def deriveMotor(obj):
    props = initObjectProperties(obj, 'motor')
    props['name'] = obj.name
    props['joint'] = obj.name
    return props#, obj.parent


def deriveKinematics(obj, typetags=False):
    link = deriveLink(obj, typetags)
    joint = None
    motor = None
    if obj.parent:
        joint = deriveJoint(obj, typetags)
        motor = deriveMotor(obj)
    return link, joint, motor


def deriveGeometry(obj):
    if 'geometryType' in obj:
        geometry = {'geometryType': obj['geometryType']}
        gt = obj['geometryType']
        if gt == 'box':
            geometry['size'] = list(obj.dimensions)
        elif gt == 'cylinder':
            geometry["radius"] = obj.dimensions[0]/2
            geometry["height"] = obj.dimensions[2]
        elif gt == 'sphere':
            geometry['radius'] = obj.dimensions[0]/2
        elif gt == 'mesh':
            filename = obj['filename'] if 'filename' in obj else obj.name.replace('/', '_')
            if bpy.data.worlds[0].useObj:
                extension = ".obj"
            elif bpy.data.worlds[0].useBobj:
                extension = ".bobj"
            elif bpy.data.worlds[0].useStl:
                extension = ".stl"
            else:
                extension = ".obj"
            geometry['filename'] = filename + extension
            geometry['scale'] = list(obj.scale)
            geometry['size'] = list(obj.dimensions)  # this is needed to calculate an approximate inertia
        return geometry
    else:
        warnings.warn("No geometryType found for object "+obj.name+".")
        return None


def deriveInertial(obj):
    """Derives a dictionary entry of an inertial object."""
    props = initObjectProperties(obj)
    #inertia = props['inertia'].split()
    props['inertia'] = list(map(float, obj['inertia']))
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
    visual = initObjectProperties(obj)
    visual['name'] = obj.name
    visual['pose'] = deriveObjectPose(obj)
    #if obj.data.materials:
    #    visual['material'] = deriveMaterial(obj.data.materials[0]) #this is now centralized!
    visual['geometry'] = deriveGeometry(obj)
    return visual, obj.parent


def deriveCollision(obj):
    collision = initObjectProperties(obj)
    collision['name'] = obj.name
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
    props = initObjectProperties(obj)
    return props


def deriveController(obj):
    props = initObjectProperties(obj)
    return props


def initObjectProperties(obj, marstype=None):
    props = {}
    if not marstype:
        for key, value in obj.items():
            props[key] = value
    else:
        for key, value in obj.items():
            if key.find(marstype+'/') >= 0:
                props[key.replace(marstype+'/', '')] = value
    props['name'] = obj.name
    return props


def cleanObjectProperties(props):
    """Cleans a predefined list of Blender-specific properties from the dictionary."""
    getridof = ['MARStype', '_RNA_UI', 'cycles_visibility', 'startChain', 'endChain', 'masschanged']
    if props:
        for key in getridof:
            if key in props:
                del props[key]
    return props


def deriveDictEntry(obj):
    print(obj.name, end=', ')
    props = None
    parent = None
    try:
        if obj.MARStype == 'inertial':
            props, parent = deriveInertial(obj)
        elif obj.MARStype == 'visual':
            props, parent = deriveVisual(obj)
        elif obj.MARStype == 'collision':
            props, parent = deriveCollision(obj)
        elif obj.MARStype == 'approxsphere':
            props, parent = deriveApproxsphere(obj)
        elif obj.MARStype == 'sensor':
            props = deriveSensor(obj)
        #elif obj.MARStype == 'motor':
        #    props, parent = deriveMotor(obj)
        elif obj.MARStype == 'controller':
            props = deriveController(obj)
    except KeyError:
        print('phobos: A KeyError occurred, likely due to missing information in the model:\n    ', sys.exc_info()[0])
    if obj.MARStype in ['sensor', 'motor' 'controller']:
        return cleanObjectProperties(props)
    else:
        return cleanObjectProperties(props), parent


def deriveGroupEntry(group, typetags):
    links = []
    #joints = []
    for obj in group.objects:
        if obj.MARStype == 'link':
            links.append({'type': 'link', 'name': obj.name})
            #joint = deriveJoint(obj, typetags)
            #joints.append({'type': 'joint', 'name': joint['name']})
        else:
            print("### Error: group " + group.name + " contains " + obj.MARStype + ': ' + obj.name)
    return links #+ joints


def deriveChainEntry(obj):
    returnchains = []
    if 'endChain' in obj:
        chainlist = obj['endChain']
    for chainName in chainlist:
        chainclosed = False
        parent = obj
        chain = {'name': chainName, 'start': '', 'end': obj.name, 'elements': []}
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


def buildRobotDictionary(typetags=False):
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
            'simulation': {}
            }
    #save timestamped version of model
    robot["date"] = datetime.now().strftime("%Y%m%d_%H:%M")
    root = getRoot(bpy.context.selected_objects[0])
    if root.MARStype != 'link':
        raise Exception("Found no 'link' object as root of the robot model.")
    else:
        if 'modelname' in root:
            robot['modelname'] = root["modelname"]
        else:
            robot['modelname'] = 'unnamed_robot'

    # digest all the links to derive link and joint information
    print('\nParsing links, joints and motors...')
    for obj in bpy.context.selected_objects:
        if obj.MARStype == 'link':
            link, joint, motor = deriveKinematics(obj, typetags)
            robot['links'][obj.name] = link # it's important that this is really the object's name
            if joint: # joint can be None if link is a root
                robot['joints'][joint['name']] = joint
            if motor:
                robot['motors'][joint['name']] = motor
            obj.select = False

    # add inertial information to link
    print('\n\nParsing inertials...')
    for l in robot['links']:
        link = bpy.data.objects[l]
        inertials = getImmediateChildren(link, 'inertial')
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
        if obj.MARStype in ['visual', 'collision']:
            props, parent = deriveDictEntry(obj)
            robot['links'][parent.name][obj.MARStype][obj.name] = props
            obj.select = False
        elif obj.MARStype == 'approxsphere':
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
        if obj.MARStype in ['sensor', 'controller']:
            robot[obj.MARStype+'s'][obj.name] = deriveDictEntry(obj)
            obj.select = False

    # parse materials
    print('\n\nParsing materials...')
    robot['materials'] = collectMaterials(objectlist)
    for obj in objectlist:
        if obj.MARStype == 'visual' and len(obj.data.materials) > 0:
            mat = obj.data.materials[0]
            if not mat.name in robot['materials']:
                robot['materials'][mat.name] = deriveMaterial(mat) #this should actually never happen
            robot['links'][obj.parent.name]['visual'][obj.name]['material'] = mat.name

    # gather information on groups of objects
    print('\n\nParsing groups...')
    for group in bpy.data.groups:  # TODO: get rid of the "data" part
        if len(group.objects) > 0 and group.name != "RigidBodyWorld":
            robot['groups'][group.name] = deriveGroupEntry(group, typetags)

    # gather information on chains of objects
    print('\n\nParsing chains...')
    chains = []
    for obj in bpy.data.objects:
        if obj.MARStype == 'link' and 'endChain' in obj:
            chains.extend(deriveChainEntry(obj))
    for chain in chains:
        robot['chains'][chain['name']] = chain

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
    elif geometry['type'] == 'box' or geometry['type'] == 'plane':
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
        if not 'material' in visual:
            note = "CheckModel: Warning, visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'material'."
            notifications += note + "\n"
            print(note)
        else:
            material = visual[material_key]
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
        if not 'filename' in link:
            note = "CheckModel: Error, link '" + link_key + "' has no attribute 'filename'."
            notifications += note + "\n"
            print(note)
        if not 'pose' in link:
            note = "CheckModel: Error, link '" + link_key + "' has no attribute 'pose'."
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
