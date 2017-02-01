#!/usr/bin/python
# coding=utf-8

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

File model.py

Created on 28 Jul 2014

@author: Kai von Szadkowski, Stefan Rahms
"""

# import from standard Python
import os
import copy
from datetime import datetime

# imports from additional modules
import yaml

# import from Blender
import bpy
import mathutils

# import from Phobos
import phobos.model.links as linkmodel
import phobos.model.inertia as inertiamodel
import phobos.model.joints as jointmodel
#import phobos.model.motors as motormodel
import phobos.model.controllers as controllermodel
import phobos.model.sensors as sensormodel
import phobos.model.lights as lightmodel
import phobos.model.poses as poses
import phobos.utils.naming as nUtils
import phobos.utils.selection as sUtils
import phobos.utils.blender as bUtils
import phobos.utils.blender as eUtils
from phobos.phoboslog import log
from phobos.utils.general import epsilonToZero
from phobos.model.poses import deriveObjectPose
from phobos.model.geometries import deriveGeometry


        #    if prop != 'joint':
        #        if not prop.startswith('$'):
        #            joint['motor/'+prop] = motor[prop]
        #        else:
        #            for tag in motor[prop]:
        #                joint['motor/'+prop[1:]+'/'+tag] = motor[prop][tag]
    #except KeyError:
        #print("Joint " + motor['joint'] + " does not exist", "ERROR")


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
    material['diffuseColor'] = dict(zip(['r', 'g', 'b'],
                                        [mat.diffuse_intensity * num for num in list(mat.diffuse_color)]))
    material['ambientColor'] = dict(zip(['r', 'g', 'b'],
                                        [mat.ambient * mat.diffuse_intensity * num for num in list(mat.diffuse_color)]))
    material['specularColor'] = dict(zip(['r', 'g', 'b'],
                                         [mat.specular_intensity * num for num in list(mat.specular_color)]))
    if mat.emit > 0:
        material['emissionColor'] = dict(zip(['r', 'g', 'b'],
                                             [mat.emit * mat.specular_intensity * num for num in list(mat.specular_color)]))
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
                log("None or incomplete texture data for material " + nUtils.getObjectName(mat, 'material'),
                    "WARNING", "deriveMaterial")
    return material


def deriveLink(obj):
    """This function derives a link from a blender object and creates its initial phobos data structure.

    :param obj: The blender object to derive the link from.
    :type obj: bpy_types.Object
    :return: dict

    """
    props = initObjectProperties(obj, phobostype='link', ignoretypes=['joint', 'motor', 'entity'])
    parent = sUtils.getEffectiveParent(obj)
    props['parent'] = parent.name if parent else None
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
    if 'joint/type' not in obj.keys():
        jt, crot = jointmodel.deriveJointType(obj, adjust=True)
    props = initObjectProperties(obj, phobostype='joint', ignoretypes=['link', 'motor', 'entity'])

    parent = sUtils.getEffectiveParent(obj)
    props['parent'] = nUtils.getObjectName(parent)
    props['child'] = nUtils.getObjectName(obj)
    axis, minmax = jointmodel.getJointConstraints(obj)
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
    # TODO:
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
    state = {'matrix': [list(vector) for vector in list(joint.pose.bones[0].matrix_basis)],
             'translation': list(joint.pose.bones[0].matrix_basis.to_translation()),
             'rotation_euler': list(joint.pose.bones[0].matrix_basis.to_euler()),
             'rotation_quaternion': list(joint.pose.bones[0].matrix_basis.to_quaternion())}
    # TODO: hard-coding this could prove problematic if we at some point build armatures from multiple bones
    return state


def deriveMotor(obj, joint):
    """This function derives a motor from an object and joint.

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
            log("Missing data in motor " + obj.name + '. No motor created.', "WARNING", "deriveMotor")
            return None
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
    # joints and motors of root elements are only relevant for scenes, not within models
    if sUtils.getEffectiveParent(obj):
        # TODO: here we have to identify root joints and write their properties to SMURF!
        # --> namespacing parent = "blub::blublink1"
        # --> how to mark separate smurfs in phobos (simply modelname?)
        # -> cut models in pieces but adding modelnames
        # -> automatic namespacing
        joint = deriveJoint(obj)
        motor = deriveMotor(obj, joint)
    return link, joint, motor


def deriveInertial(obj):
    """This function derives the inertial from the given object.

    :param obj: The object to derive the inertial from.
    :type obj: bpy_types.Object
    :return: dict
    """
    try:
        props = initObjectProperties(obj, phobostype='inertial')
        props['inertia'] = list(map(float, obj['inertial/inertia']))
        props['pose'] = deriveObjectPose(obj)
    except KeyError as e:
        log("Missing data in inertial object " + obj.name + str(e), "ERROR", "deriveInertial")
        return None
    return props


def deriveVisual(obj):
    """This function derives the visual information from an object.

    :param obj: The blender object to derive the visuals from.
    :type obj: bpy_types.Object
    :return: dict

    """
    try:
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
                if bpy.data.worlds[0].phobosexportsettings.useObj:
                    filename += ".obj"
                elif bpy.data.worlds[0].phobosexportsettings.useStl:
                    filename += ".stl"
                elif bpy.data.worlds[0].phobosexportsettings.useDae:
                    filename += ".dae"
                else:
                    filename += ".obj"
                lodlist.append({'start': obj.lod_levels[i].distance, 'end': maxdlist[i], 'filename': os.path.join('meshes', filename)})
            visual['lod'] = lodlist
    except KeyError:
        log("Missing data in visual object " + obj.name, "ERROR", "deriveVisual")
        return None
    return visual


def deriveCollision(obj):
    """This function derives the collision information from an object.

    :param obj: The blender object to derive the collision information from.
    :type obj: bpy_types.Object
    :return: dict

    """
    try:
        collision = initObjectProperties(obj, phobostype='collision', ignoretypes='geometry')
        collision['geometry'] = deriveGeometry(obj)
        collision['pose'] = deriveObjectPose(obj)
        # the bitmask is cut to length = 16 and reverted for int parsing
        try:
            collision['bitmask'] = int(''.join(['1' if group else '0' for group in obj.rigid_body.collision_groups[:16]])[::-1], 2)
        except AttributeError:
            pass
    except KeyError:
        log("Missing data in collision object " + obj.name, "ERROR", "deriveCollision")
        return None
    return collision


def deriveCapsule(obj):
    """This function derives a capsule from a given blender object

    :param obj: The blender object to derive the capsule from.
    :type obj: bpy_types.Object
    :return: tuple

    """
    viscol_dict = {}
    capsule_pose = poses.deriveObjectPose(obj)
    rotation = capsule_pose['rotation_euler']
    capsule_radius = obj['geometry']['radius']
    for part in ['sphere1', 'cylinder', 'sphere2']:
        viscol = initObjectProperties(obj, phobostype='collision', ignoretypes='geometry')
        viscol['name'] = nUtils.getObjectName(obj).split(':')[-1] + '_' + part
        geometry = {}
        pose = {}
        geometry['radius'] = capsule_radius
        if part == 'cylinder':
            geometry['length'] = obj['geometry']['length']
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
    """This function derives an SRDF approximation sphere from a given blender object

    :param obj: The blender object to derive the approxsphere from.
    :type obj: bpy_types.Object
    :return: tuple

    """
    try:
        sphere = initObjectProperties(obj)
        sphere['radius'] = obj.dimensions[0]/2
        pose = deriveObjectPose(obj)
        sphere['center'] = pose['translation']
    except KeyError:
        log("Missing data in collision approximation object " + obj.name, "ERROR", "deriveApproxSphere")
        return None
    return sphere


def deriveSensor(obj):
    """This function derives a sensor from a given blender object

    :param obj: The blender object to derive the sensor from.
    :type obj: bpy_types.Object
    :return: dict
    """
    try:
        props = initObjectProperties(obj, phobostype='sensor')
        props['link'] = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
    except KeyError:
        log("Missing data in sensor " + obj.name, "ERROR", "deriveSensor")
        return None
    return props


def deriveController(obj):
    """This function derives a controller from a given blender object

    :param obj: The blender object to derive the controller from.
    :type obj: bpy_types.Object
    :return: dict
    """
    try:
        props = initObjectProperties(obj, phobostype='controller')
    except KeyError:
        log("Missing data in controller  " + obj.name, "ERROR", "deriveController")
        return None
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
    pose = deriveObjectPose(obj)
    light['position'] = pose['translation']
    light['rotation'] = pose['rotation_euler']
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

    light['parent'] = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
    return light


def initObjectProperties(obj, phobostype=None, ignoretypes=()):
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
    props = {'name': nUtils.getObjectName(obj, phobostype)}  # allow duplicated names differentiated by types
    if not phobostype:  # if no phobostype is defined, everything is parsed
        for key, value in obj.items():
            props[key] = value
    else:  # if a phobostype is defined, we search for special custom properties
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
                elif key.count('/') == 1:  # ignore two-level specifiers if phobostype is not present
                    category, specifier = key.split('/')
                    if category not in ignoretypes:
                        if '$'+category not in props:
                            props['$'+category] = {}
                        props['$'+category][specifier] = value
    return props


def deriveDictEntry(obj):
    """Derives a phobos dictionary entry from the provided object.

    :param obj: The object to derive the dict entry (phobos data structure) from.
    :type obj: bpy_types.Object
    :return: tuple

    """
    try:
        if obj.phobostype == 'inertial':
            props = deriveInertial(obj)
        elif obj.phobostype == 'visual':
            props = deriveVisual(obj)
        elif obj.phobostype == 'collision':
            props = deriveCollision(obj)
        elif obj.phobostype == 'approxsphere':
            props = deriveApproxsphere(obj)
        elif obj.phobostype == 'sensor':
            props = deriveSensor(obj)
        elif obj.phobostype == 'controller':
            props = deriveController(obj)
        elif obj.phobostype == 'light':
            props = deriveLight(obj)
    except KeyError:
        log("A KeyError occurred due to unspecifiable missing model data.", "DEBUG", "deriveDictEntry")
        return None, None
    return props


def deriveGroupEntry(group):
    """Derives a list of phobos link skeletons for a provided group object.

    :param group: The blender group to extract the links from.
    :type group: bpy_types.Group
    :return: list

    """
    links = []
    for obj in group.objects:
        if obj.phobostype == 'link':
            links.append({'type': 'link', 'name': nUtils.getObjectName(obj)})
        else:
            log("Group " + group.name + " contains " + obj.phobostype
                + ': ' + nUtils.getObjectName(obj), "ERROR", "deriveGroupEntry")
    return links


def deriveChainEntry(obj):
    """Derives a phobos dict entry for a kinematic chain ending in the provided object.

    :param obj:
    :return:
    """
    returnchains = []
    if 'endChain' in obj:
        chainlist = obj['endChain']
    for chainName in chainlist:
        chainclosed = False
        parent = obj
        chain = {'name': chainName, 'start': '', 'end': nUtils.getObjectName(obj), 'elements': []}
        while not chainclosed:
            if parent.parent is None:  # FIXME: use effectiveParent
                log("Unclosed chain, aborting parsing chain " + chainName, "ERROR", "deriveChainEntry")
                chain = None
                break
            chain['elements'].append(parent.name)
            parent = parent.parent  # FIXME: use effectiveParent
            if 'startChain' in parent:
                startchain = parent['startChain']
                if chainName in startchain:
                    chain['start'] = nUtils.getObjectName(parent)
                    chain['elements'].append(nUtils.getObjectName(parent))
                    chainclosed = True
        if chain is not None:
            returnchains.append(chain)
    return returnchains


def storePose(modelname, posename):
    """
    Stores the current pose of all of a robot's selected joints.
    Existing poses of the same name will be overwritten.

    :param modelname: The robot the pose belongs to.
    :type modelname: str.
    :param posename: The name the pose will be stored under.
    :type posename: str.
    :return: Nothing.
    """
    rootlink = None
    for root in sUtils.getRoots():
        if root['modelname'] == modelname:
            rootlink = root
    if rootlink:
        filename = modelname + '::poses'
        posedict = yaml.load(bUtils.readTextFile(filename))
        if not posedict:
            posedict = {posename: {'name': posename, 'joints': {}}}
        else:
            posedict[posename] = {'name': posename, 'joints': {}}
        bpy.ops.object.mode_set(mode='POSE')
        links = sUtils.getChildren(rootlink, ('link',), True, False)
        for link in (link for link in links if 'joint/type' in link
                     and link['joint/type'] not in ['fixed', 'floating']):
            link.pose.bones['Bone'].rotation_mode = 'XYZ'
            posedict[posename]['joints'][nUtils.getObjectName(link, 'joint')] = link.pose.bones['Bone'].rotation_euler.y
        bUtils.updateTextFile(filename, yaml.dump(posedict, default_flow_style=False))
    else:
        log("No model root could be found to store the pose for", "ERROR", "storePose")


def loadPose(modelname, posename):
    """
    Load and apply a robot's stored pose.

    :param modelname: The model's name.
    :type modelname: str.
    :param posename: The name the pose is stored under.
    :type posename: str.
    :return Nothing.
    """
    load_file = bUtils.readTextFile(modelname + '::poses')
    if load_file == '':
        log('No poses stored.', 'ERROR', 'loadPose')
        return
    poses = yaml.load(load_file)
    try:
        pose = poses[posename]
        prev_mode = bpy.context.mode
        bpy.ops.object.mode_set(mode='POSE')
        for obj in sUtils.getObjectsByPhobostypes(['link']):
            if nUtils.getObjectName(obj, 'joint') in pose['joints']:
                obj.pose.bones['Bone'].rotation_mode = 'XYZ'
                obj.pose.bones['Bone'].rotation_euler.y = float(pose['joints'][nUtils.getObjectName(obj, 'joint')])
        bpy.ops.object.mode_set(mode=prev_mode)
    except KeyError:
        log('No pose with name ' + posename + ' stored for model ' + modelname, 'ERROR', "loadPose")


def getPoses(modelname):
    """
    Get the names of the poses that have been stored for a robot.

    :param modelname: The model's name.
    :return: A list containing the poses' names.
    """
    load_file = bUtils.readTextFile(modelname + '::poses')
    if load_file == '':
        return []
    poses = yaml.load(load_file)
    return poses.keys()


def deriveTextData(modelname):
    """
    Collect additional data stored for a specific model.

    :param modelname: Name of the model for which data should be derived.
    :return: A dictionary containing additional data.
    """
    datadict = {}
    datatextfiles = [text for text in bpy.data.texts if text.name.startswith(modelname+'::')]
    for text in datatextfiles:
        try:
            dataname = text.name.split('::')[-1]
        except IndexError:
            log("Possibly invalidly named model data text file: " + modelname, "WARNING", "deriveTextData")
        try:
            data = yaml.load(bUtils.readTextFile(text.name))
        except yaml.scanner.ScannerError:
            log("Invalid formatting of data file: " + dataname, "ERROR", "deriveTextData")
        if data:
            datadict[dataname] = data
    return datadict


def buildModelDictionary(root):
    """Builds a python dictionary representation of a Phobos model.

    :param root: bpy.types.objects
    :return: dict
    """
    #os.system('clear')

    model = {'links': {},
             'joints': {},
             'sensors': {},
             'motors': {},
             'controllers': {},
             'materials': {},
             'meshes': {},
             'lights': {},
             'groups': {},
             'chains': {}
             }
    # timestamp of model
    model["date"] = datetime.now().strftime("%Y%m%d_%H:%M")
    if root.phobostype != 'link':
        log("Found no 'link' object as root of the robot model.", "ERROR", "buildModelDictionary")
        raise Exception(root.name + " is  no valid root link.")
    else:
        if 'modelname' in root:
            model['name'] = root["modelname"]
        else:
            log("No name for the model defines, setting to 'unnamed_model'", "WARNING", "buildModelDictionary")
            model['name'] = 'unnamed_model'

    log("Creating dictionary for robot " + model['name'] + " from object "
        + root.name, "INFO", "buildModelDictionary")

    # create tuples of objects belonging to model
    objectlist = sUtils.getChildren(root, selected_only=True, include_hidden=False)
    linklist = [link for link in objectlist if link.phobostype == 'link']

    # digest all the links to derive link and joint information
    log("Parsing links, joints and motors...", "INFO", "buildModelDictionary")
    for link in linklist:
        # parse link and extract joint and motor information
        linkdict, jointdict, motordict = deriveKinematics(link)
        model['links'][linkdict['name']] = linkdict
        if jointdict:  # joint will be None if link is a root
            model['joints'][jointdict['name']] = jointdict
        if motordict:  # motor will be None if no motor is attached or link is a root
            model['motors'][motordict['name']] = motordict
        # add inertial information to link
        try:  # if this link-inertial object is no present, we ignore the inertia!
            inertial = bpy.context.scene.objects['inertial_' + linkdict['name']]
            props = deriveDictEntry(inertial)
            if props is not None:
                model['links'][linkdict['name']]['inertial'] = props
        except KeyError:
            log("No inertia for link " + linkdict['name'], "WARNING", "buildModelDictionary")

    # combine inertia if certain objects are left out, and overwrite it
    inertials = (i for i in objectlist if i.phobostype == 'inertial' and "inertial/inertia" in i)
    editlinks = {}
    for i in inertials:
        if i.parent not in linklist:
            realparent = sUtils.getEffectiveParent(i)
            if realparent:
                parentname = nUtils.getObjectName(realparent)
                if parentname in editlinks:
                    editlinks[parentname].append(i)
                else:
                    editlinks[parentname] = [i]
    for linkname in editlinks:
        inertials = editlinks[linkname]
        try:
            inertials.append(bpy.context.scene.objects['inertial_' + linkname])
        except KeyError:
            pass
        mv, cv, iv = inertiamodel.fuseInertiaData(inertials)
        iv = inertiamodel.inertiaMatrixToList(iv)
        if mv is not None and cv is not None and iv is not None:
            model['links'][linkname]['inertial'] = {'mass': mv, 'inertia': iv,
                                                    'pose': {'translation': list(cv), 'rotation_euler': [0, 0, 0]}
                                                    }

    # complete link information by parsing visuals and collision objects
    log("Parsing visual and collision (approximation) objects...", "INFO", "buildModelDictionary")
    for obj in objectlist:
        try:
            if obj.phobostype in ['visual', 'collision']:
                props = deriveDictEntry(obj)
                parentname = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
                model['links'][parentname][obj.phobostype][nUtils.getObjectName(obj)] = props
            elif obj.phobostype == 'approxsphere':
                props = deriveDictEntry(obj)
                parentname = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
                model['links'][parentname]['approxcollision'].append(props)
        except KeyError:
            try:
                log(parentname + " not found", "ERROR")
            except TypeError:
                log("No parent found for " + obj.name, "ERROR")

    # combine collision information for links
    for linkname in model['links']:
        link = model['links'][linkname]
        bitmask = 0
        for collname in link['collision']:
            try:
                bitmask = bitmask | link['collision'][collname]['bitmask']
            except KeyError:
                pass
        link['collision_bitmask'] = bitmask

    # parse sensors and controllers
    log("Parsing sensors and controllers...", "INFO", "buildModelDictionary")
    for obj in objectlist:
        if obj.phobostype in ['sensor', 'controller']:
            props = deriveDictEntry(obj)
            model[obj.phobostype+'s'][nUtils.getObjectName(obj)] = props

    # parse materials
    log("Parsing materials...", "INFO", "buildModelDictionary")
    model['materials'] = collectMaterials(objectlist)
    for obj in objectlist:
        if obj.phobostype == 'visual' and len(obj.data.materials) > 0:
            mat = obj.data.materials[0]
            matname = nUtils.getObjectName(mat, 'material')
            if matname not in model['materials']:
                model['materials'][matname] = deriveMaterial(mat)  # this should actually never happen
            linkname = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
            model['links'][linkname]['visual'][nUtils.getObjectName(obj)]['material'] = matname

    # identify unique meshes
    log("Parsing meshes...", "INFO", "buildModelDictionary")
    for obj in objectlist:
        try:
            if ((obj.phobostype == 'visual' or obj.phobostype == 'collision') and
                    (obj['geometry/type'] == 'mesh') and (obj.data.name not in model['meshes'])):
                model['meshes'][obj.data.name] = obj
                for lod in obj.lod_levels:
                    if lod.object.data.name not in model['meshes']:
                        model['meshes'][lod.object.data.name] = lod.object
        except KeyError:
            log("Undefined geometry type in object " + obj.name, "ERROR", "buildModelDictionary")

    # gather information on groups of objects
    log("Parsing groups...", "INFO", "buildModelDictionary")
    for group in bpy.data.groups:  # TODO: get rid of the "data" part and check for relation to robot
        if len(group.objects) > 0 and nUtils.getObjectName(group, 'group') != "RigidBodyWorld":
            model['groups'][nUtils.getObjectName(group, 'group')] = deriveGroupEntry(group)

    # gather information on chains of objects
    log("Parsing chains...", "INFO", "buildModelDictionary")
    chains = []
    for obj in objectlist:
        if obj.phobostype == 'link' and 'endChain' in obj:
            chains.extend(deriveChainEntry(obj))
    for chain in chains:
        model['chains'][chain['name']] = chain

    # gather information on lights
    log("Parsing lights...", "INFO", "buildModelDictionary")
    for obj in objectlist:
        if obj.phobostype == 'light':
            model['lights'][nUtils.getObjectName(obj)] = deriveLight(obj)

    # add additional data to model
    model.update(deriveTextData(model['name']))

    # shorten numbers in dictionary to n decimalPlaces and return it
    log("Rounding numbers...", "INFO", "buildModelDictionary")
    epsilon = 10**(-bpy.data.worlds[0].phobosexportsettings.decimalPlaces)  # TODO: implement this separately
    return epsilonToZero(model, epsilon, bpy.data.worlds[0].phobosexportsettings.decimalPlaces), objectlist


def buildModelFromDictionary(model):
    """Creates the Blender representation of the imported model, using a model dictionary.

    """
    log("Creating Blender model...", 'INFO', 'buildModelFromDictionary')

    log("Creating links...", 'INFO', 'buildModelFromDictionary')
    for l in model['links']:
        link = model['links'][l]
        linkmodel.createLink(link)

    log("Creating joints...", 'INFO', 'buildModelFromDictionary')
    for j in model['joints']:
        joint = model['joints'][j]
        jointmodel.createJoint(joint)

    # build tree recursively and correct translation & rotation on the fly
    log("Placing links...", 'INFO', 'buildModelFromDictionary')
    for l in model['links']:
        if 'parent' not in model['links'][l]:
            root = model['links'][l]
            linkmodel.placeChildLinks(model, root)
            log("Assigning model name...", 'INFO', 'buildModelFromDictionary')
            try:
                rootlink = sUtils.getRoot(bpy.data.objects[root['name']])
                rootlink['modelname'] = model['name']
                rootlink.location=(0, 0, 0)
            except KeyError:
                log("Could not assign model name to root link.", "ERROR")

    log("Creating visual and collision objects...", 'INFO', 'buildModelFromDictionary')
    for link in model['links']:
        linkmodel.placeLinkSubelements(model['links'][link])

    try:
        log("Creating sensors...", 'INFO', 'buildModelFromDictionary')
        for s in model['sensors']:
            sensormodel.createSensor(model['sensors'][s])
    except KeyError:
        log("No sensors in model " + model['name'], 'INFO', 'buildModelFromDictionary')

    try:
        log("Creating motors...", 'INFO', 'buildModelFromDictionary')
        for m in model['motors']:
            eUtils.addDictionaryToObj(model['motors'][m],
                                      model['joints'][model['motors'][m]['joint']],
                                      category='motor')
    except KeyError:
        log("No motors in model " + model['name'], 'INFO', 'buildModelFromDictionary')

    try:
        log("Creating controllers...", 'INFO', 'buildModelFromDictionary')
        for c in model['controllers']:
            controllermodel.createController(model['controllers'][c])
    except KeyError:
        log("No controllers in model " + model['name'], 'INFO', 'buildModelFromDictionary')

    try:
        log("Creating groups...", 'INFO', 'buildModelFromDictionary')
        for g in model['groups']:
            createGroup(model['groups'][g])
    except KeyError:
        log("No kinematic groups in model " + model['name'], 'INFO', 'buildModelFromDictionary')

    try:
        log("Creating chains...", 'INFO', 'buildModelFromDictionary')
        for ch in model['chains']:
            createChain(model['chains'][ch])
    except KeyError:
        log("No kinematic chains in model " + model['name'], 'INFO', 'buildModelFromDictionary')

    try:
        log("Creating lights...", 'INFO', 'buildModelFromDictionary')
        for l in model['lights']:
            lightmodel.createLight(model['lights'][l])
    except KeyError:
        log("No lights in model " + model['name'], 'INFO', 'buildModelFromDictionary')

    # FIXME: this is a trick to force Blender to apply matrix_local
    # AAAAAARGH: THIS DOES NOT WORK!
    for obj in bpy.data.objects:
        bUtils.setObjectLayersActive(obj)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.transform.translate(value=(0, 0, 0))


def createGroup(group):
    pass


def createChain(group):
    pass
