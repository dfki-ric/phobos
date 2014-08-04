'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtrobotdictionary.py

Created on 28 Jul 2014

@author: Kai von Szadkowski, Stefan Rahms

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
import mathutils
import sys
import datetime
import warnings
import marstools.mtdefs as mtdefs
import marstools.mtutility as mtutility
import marstools.mtjoints as mtjoints
import marstools.mtinertia as mtinertia
from marstools.mtutility import *


def register():
    print("Registering mtexport...")


def collectMaterials(objectlist):
    print('\n\nCollecting materials...')
    materials = {}
    for obj in objectlist:
        if obj.MARStype == 'visual' and obj.data.materials:
            mat = obj.data.materials[0] #simply grab the first material
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
    except KeyError:
        print('Material ' + mat.name + ' has no property texturename.')
    except AttributeError:
        print('Texture in material ' + mat.name + ' not specified with image data.')
    return material


def deriveLink(obj):
    props = initObjectProperties(obj)
    props["pose"] = deriveObjectPose(obj)
    props["collision"] = {}
    props["visual"] = {}
    props["inertial"] = {}
    return props


def deriveJoint(obj):
    props = {'name': obj.name}
    props['parent'] = obj.parent.name
    props['child'] = obj.name
    props['jointType'], crot = mtjoints.deriveJointType(obj, True)
    axis, limit = mtjoints.getJointConstraints(obj)
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

    #derive motor data
    motorprops = {'name': obj.name, 'joint': obj.name}
    for key,value in obj.items():
        if key.find('motor/') >= 0:
            motorprops[key.replace('motor/', '')] = value
    return props, motorprops


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


def deriveKinematics(obj):
    link = deriveLink(obj)
    joint = None
    motor = None
    if obj.parent:
        joint, motor = deriveJoint(obj)
    return link, joint, motor


#def deriveMotor(obj):
#    props = initObjectProperties(obj)
#    return props, obj.parent


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
            filename = obj['filename'] if 'filename' in obj else obj.name.replace('/','_')
            geometry['filename'] = filename + (".bobj" if bpy.context.scene.world.exportBobj else ".obj")
            geometry['scale'] = list(obj.scale)
            geometry['size'] = list(obj.dimensions) #this is needed to calculate an approximate inertia if collision is inertia
        return geometry
    else:
        warnings.warn("No geometryType found for object "+obj.name+".")
        return None


def deriveInertial(obj):
    '''Derives a dictionary entry of an inertial object.'''
    props = initObjectProperties(obj)
    inertia = props['inertia'].split()
    props['inertia'] = list(map(float, inertia))
    props['pose'] = deriveObjectPose(obj)
    return props, obj.parent


def deriveObjectPose(obj):
    '''Derive pose of visual or collision object.'''
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
    return collision, obj.parent


def deriveSensor(obj):
    props = initObjectProperties(obj)
    return props


def deriveController(obj):
    props = initObjectProperties(obj)
    return props


def initObjectProperties(obj):
    props = {}
    for key in obj.keys():
        props[key] = obj[key]
    props['name'] = obj.name
    return props


def cleanObjectProperties(props):
    '''Cleans a predefined list of Blender-specific properties from the dictionary.'''
    getridof = ['MARStype', '_RNA_UI', 'cycles_visibility', 'startChain', 'endChain', 'masschanged']
    if props:
        for key in getridof:
            if key in props:
                del props[key]
    return props


def deriveDictEntry(obj):
    print("MARStools: Deriving dictionary entry for", obj.MARStype + ': ' + obj.name, )
    props = None
    parent = None
    try:
        if obj.MARStype == 'inertial':
            props, parent = deriveInertial(obj)
        elif obj.MARStype == 'visual':
            props, parent = deriveVisual(obj)
        elif obj.MARStype == 'collision':
            props, parent = deriveCollision(obj)
        elif obj.MARStype == 'sensor':
            props = deriveSensor(obj)
        #elif obj.MARStype == 'motor':
        #    props, parent = deriveMotor(obj)
        elif obj.MARStype == 'controller':
            props = deriveController(obj)
    except KeyError:
        print('MARStools: A KeyError occurred, likely due to missing information in the model:\n    ', sys.exc_info()[0])
    if obj.MARStype in ['sensor', 'motor' 'controller']:
        return cleanObjectProperties(props)
    else:
        return cleanObjectProperties(props), parent


def deriveGroupEntry(group):
    return [obj.name for obj in group.objects]


def deriveChainEntry(obj):
    returnchains = []
    if 'endChain' in obj:
        endChain = obj['endChain'].split()
    for chainName in endChain:
        chainclosed = False
        parent = obj
        chain = {'name': chainName, 'start': '', 'end': obj.name, 'elements': []}
        while not chainclosed:
            if parent.parent == None:
                print('### Error: Unclosed chain, aborting parsing chain', chainName)
                chain = None
                break
            chain['elements'].append(parent.name)
            parent = parent.parent
            if 'startChain' in parent:
                startChain = parent['startChain'].split()
                if chainName in startChain:
                    chain['start'] = parent.name
                    chain['elements'].append(parent.name)
                    chainclosed = True
        if chain is not None:
            returnchains.append(chain)
    return returnchains


def buildRobotDictionary():
    '''Builds a python dictionary representation of a Blender robot model for export and inspection.'''
    objectlist = bpy.context.selected_objects
    #notifications, faulty_objects = mtupdate.updateModel(bpy.context.selected_objects)
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
    for obj in bpy.context.selected_objects:
        if obj.MARStype == 'link':
            print('Parsing', obj.MARStype, obj.name, '...')
            link, joint, motor = deriveKinematics(obj)
            robot['links'][obj.name] = link
            if joint: #joint can be None if link is a root
                robot['joints'][joint['name']] = joint
            if motor:
                robot['motors'][joint['name']] = motor
            obj.select = False

    # add inertial information to link
    for l in robot['links']:
        link = bpy.data.objects[l]
        inertials = getImmediateChildren(link, 'inertial')
        if len(inertials) == 1:
            props, parent = deriveDictEntry(inertials[0])
            robot['links'][parent.name]['inertial'] = props
            inertials[0].select = False
        elif len(inertials) > 1:
            parent = inertials[0].parent.name
            mass, com, inertia = mtinertia.compound_inertia_analysis_3x3(inertials)
            matrix_local = mathutils.Matrix.Translation(mathutils.Vector(com))
            pose = {}
            pose['matrix'] = [list(vector) for vector in list(matrix_local)]
            pose['translation'] = list(matrix_local.to_translation())
            pose['rotation_euler'] = list(matrix_local.to_euler())
            pose['rotation_quaternion'] = list(matrix_local.to_quaternion())
            props = {'mass': mass, 'pose': pose, 'inertia': inertia}
            robot['links'][parent.name]['inertial'] = props
            for i in inertials:
                i.select = False

    # complete link information by parsing visuals and collision objects
    for obj in bpy.context.selected_objects:
        if obj.MARStype in ['visual', 'collision']:
            props, parent = deriveDictEntry(obj)
            robot['links'][parent.name][obj.MARStype][obj.name] = props
            obj.select = False

    # parse sensors and controllers
    for obj in bpy.context.selected_objects:
        if obj.MARStype in ['sensor', 'controller']:
            print('Parsing', obj.MARStype, obj.name, '...')
            robot[obj.MARStype+'s'][obj.name] = deriveDictEntry(obj)
            obj.select = False

    # parse materials
    robot['materials'] = collectMaterials(objectlist)
    for obj in objectlist:
        if obj.MARStype == 'visual' and len(obj.data.materials) > 0:
            mat = obj.data.materials[0]
            if not mat.name in robot['materials']:
                robot['materials'][mat.name] = deriveMaterial(mat) #this should actually never happen
            robot['links'][obj.parent.name]['visual'][obj.name]['material'] = mat.name

    # gather information on groups of objects
    for group in bpy.data.groups: #TODO: get rid of the "data" part
        robot['groups'][group.name] = deriveGroupEntry(group)

    # gather information on chains of objects
    chains = []
    for obj in bpy.data.objects:
        if obj.MARStype == 'link' and 'endChain' in obj:
            chains.extend(deriveChainEntry(obj))
    for chain in chains:
        robot['chains'][chain['name']] = chain

    #shorten numbers in dictionary to n decimalPlaces and return it
    epsilon = 10**(-bpy.data.worlds[0].decimalPlaces) #TODO: implement this separately
    return epsilonToZero(robot, epsilon, bpy.data.worlds[0].decimalPlaces)


def check_geometry(geometry, owner_type, owner_key, link_key):
    '''
    '''
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
    '''
    '''
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
    '''
    '''
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
    '''
    '''
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
    '''
    '''
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
        if not 'jointType' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'type'."
            notifications += note + "\n"
            print(note)
        elif joint['jointType'] not in ['hinge', 'continuous', 'linear']:
            note = "CheckModel: Error, joint '" + joint_key + "' has invalid value for attribute 'jointType': '" + joint['jointType'] + "'."
            notifications +=  note + '\n'
            print(note)
    return notifications


def check_dict(model):
    '''
    '''
    notifications = ''
    notifications += check_links(model['links'])
    notifications += check_joints(model['joints'])
    return notifications
