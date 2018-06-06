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

import os
import copy
from datetime import datetime
import yaml

import bpy
import mathutils

import phobos.defs as defs
import phobos.model.links as linkmodel
import phobos.model.inertia as inertiamodel
import phobos.model.joints as jointmodel
import phobos.model.sensors as sensormodel
import phobos.model.lights as lightmodel
import phobos.model.poses as poses
import phobos.utils.naming as nUtils
import phobos.utils.selection as sUtils
import phobos.utils.blender as bUtils
import phobos.utils.editing as eUtils
import phobos.utils.io as ioUtils
from phobos.phoboslog import log
from phobos.utils.general import epsilonToZero
from phobos.model.poses import deriveObjectPose
from phobos.model.geometries import deriveGeometry
from phobos.defs import linkobjignoretypes


def collectMaterials(objectlist):
    """This function collects all materials from a list of objects and sorts them into a dictionary

    Args:
      objectlist(list: list): The objectlist to grab the materials from.

    Returns:
      dict

    """
    materials = {}
    for obj in objectlist:
        if obj.phobostype == 'visual':
            try:
                mat = obj.active_material
                if mat.name not in materials:
                    materials[mat.name] = deriveMaterial(mat)
                    materials[mat.name]['users'] = 1
                else:
                    materials[mat.name]['users'] += 1
            except AttributeError:
                log("Could not parse material in object " + obj.name, "ERROR")
    return materials


def deriveMaterial(mat):
    """This function takes a blender material and creates a phobos representation from it

    Args:
      mat(bpy.types.Material): The blender material to derive a phobos material from

    Returns:
      dict

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
    material['shininess'] = mat.specular_hardness / 2
    if mat.use_transparency:
        material['transparency'] = 1.0 - mat.alpha
    # there are always 18 slots, regardless of whether they are filled or not
    for tex in mat.texture_slots:
        if tex is not None:
            try:
                # regular diffuse color texture
                if tex.use_map_color_diffuse:
                    # grab the first texture
                    material['diffuseTexture'] = mat.texture_slots[
                        0].texture.image.filepath.replace('//', '')
                # normal map
                if tex.use_map_normal:
                    # grab the first texture
                    material['normalTexture'] = mat.texture_slots[
                        0].texture.image.filepath.replace('//', '')
                # displacement map
                if tex.use_map_displacement:
                    # grab the first texture
                    material['displacementTexture'] = mat.texture_slots[
                        0].texture.image.filepath.replace('//', '')
            except (KeyError, AttributeError):
                log("None or incomplete texture data for material "
                    + nUtils.getObjectName(mat, 'material'), "WARNING")
    return material


def deriveLink(obj):
    """This function derives a link from a blender object and creates its initial phobos data structure.

    Args:
      obj(bpy_types.Object): The blender object to derive the link from.

    Returns:
      dict

    """
    log("Deriving link from obj " + obj.name, "DEBUG")
    props = initObjectProperties(obj, phobostype='link', ignoretypes=linkobjignoretypes-{'link'})
    parent = sUtils.getEffectiveParent(obj)
    props['parent'] = nUtils.getObjectName(parent) if parent else None
    props['children'] = [child.name for child in obj.children if child.phobostype == 'link']
    props['object'] = obj
    props["pose"] = deriveObjectPose(obj)
    props["collision"] = {}
    props["visual"] = {}
    props["inertial"] = {}
    props['approxcollision'] = []

    # add inertial information to link
    try:
        inertia = deriveDictEntry(bpy.context.scene.objects['inertial_' + props['name']])
        if inertia is not None:
            props['inertial'] = inertia
    except KeyError:
        log("No inertia for link " + props['name'], "WARNING")
    return props


def deriveFullLinkInformation(obj):
    """This function derives the full link information (including joint and
    motor data) from a blender object and creates its initial phobos data
    structure.

    Args:
      obj(bpy_types.Object): The blender object to derive the link from.

    Returns:
      dict

    """
    props = initObjectProperties(obj, phobostype='link', ignoretypes=[
                                 'joint', 'motor', 'entity'])
    parent = sUtils.getEffectiveParent(obj)
    props['parent'] = parent.name if parent else None
    props["pose"] = deriveObjectPose(obj)
    props["joint"] = deriveJoint(obj, adjust=False)
    del props["joint"]["parent"]
    if any(item.startswith('motor') for item in props.keys()):
        props["motor"] = deriveMotor(obj, props['joint'])
    collisionObjects = sUtils.getImmediateChildren(
        obj, phobostypes=('collision'), include_hidden=True)
    collisionDict = {}
    for colobj in collisionObjects:
        collisionDict[colobj.name] = colobj
    props["collision"] = collisionDict
    visualObjects = sUtils.getImmediateChildren(
        obj, phobostypes=('visual'), include_hidden=True)
    visualDict = {}
    for visualobj in visualObjects:
        visualDict[visualobj.name] = visualobj
    props["visual"] = visualDict
    inertialObjects = sUtils.getImmediateChildren(
        obj, phobostypes=('inertial'), include_hidden=True)
    inertialDict = {}
    for inertialobj in inertialObjects:
        inertialDict[inertialobj.name] = inertialobj
    props["inertial"] = inertialDict
    props['approxcollision'] = []
    return props


def deriveJoint(obj, adjust=True):
    """This function derives a joint from a blender object and creates its initial phobos data structure.

    Args:
      obj: The blender object to derive the joint from.
      adjust:  (Default value = True)

    Returns:
      dict

    """
    if 'joint/type' not in obj.keys():
        jt, crot = jointmodel.deriveJointType(obj, adjust=adjust)
    props = initObjectProperties(obj, phobostype='joint', ignoretypes=linkobjignoretypes-{'joint'})

    parent = sUtils.getEffectiveParent(obj)
    props['parent'] = nUtils.getObjectName(parent)
    props['child'] = nUtils.getObjectName(obj)
    axis, minmax = jointmodel.getJointConstraints(obj)
    if axis:
        props['axis'] = list(axis)
    limits = {}
    if minmax is not None:
        # prismatic or revolute joint, TODO: planar etc.
        if len(minmax) == 2:
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
    # TODO: what about these?
    # - calibration
    # - dynamics
    # - mimic
    # - safety_controller
    return props


def deriveJointState(joint):
    """Calculates the state of a joint from the state of the link armature.
    Note that this is the current state and not the zero state.

    Args:
      joint(bpy_types.Object): The joint(armature) to derive its state from.

    Returns:
      dict

    """
    state = {'matrix': [list(vector) for vector in list(joint.pose.bones[0].matrix_basis)],
             'translation': list(joint.pose.bones[0].matrix_basis.to_translation()),
             'rotation_euler': list(joint.pose.bones[0].matrix_basis.to_euler()),
             'rotation_quaternion': list(joint.pose.bones[0].matrix_basis.to_quaternion())}
    # TODO: hard-coding this could prove problematic if we at some point build armatures from multiple bones
    return state


def deriveMotor(obj, joint):
    """This function derives a motor from an object and joint.

    Args:
      obj(bpy_types.Object): The blender object to derive the motor from.
      joint(dict): The phobos joint to derive the constraints from.

    Returns:
      dict

    """
    props = initObjectProperties(obj, phobostype='motor', ignoretypes=linkobjignoretypes-{'motor'})
    # if there are any 'motor' tags and not only a name
    if len(props) > 1:
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
            log("Missing data in motor " + obj.name + '. No motor created.', "WARNING")
            return None
        return props
    else:
        # return None if no motor is attached
        return None


def deriveKinematics(obj):
    """Returns dictionaries for link, joint and motor derived from passed object.

    Args:
      obj(bpy_types.Object): The object to derive its kinematics from.

    Returns:
      tuple

    """
    link = deriveLink(obj)
    joint = None
    motor = None
    # joints and motors of root elements are only relevant for scenes, not within models
    if sUtils.getEffectiveParent(obj):
        joint = deriveJoint(obj)
        motor = deriveMotor(obj, joint)
    return link, joint, motor


def deriveInertial(obj):
    """This function derives the inertial from the given object.

    Args:
      obj(bpy_types.Object): The object to derive the inertial from.

    Returns:
      dict

    """
    try:
        props = initObjectProperties(obj, phobostype='inertial')
        props['inertia'] = list(map(float, obj['inertial/inertia']))
        props['pose'] = deriveObjectPose(obj)
    except KeyError as e:
        log("Missing data in inertial object " + obj.name + str(e), "ERROR")
        return None
    return props


def deriveVisual(obj):
    """This function derives the visual information from an object.

    Args:
      obj(bpy_types.Object): The blender object to derive the visuals from.

    Returns:
      dict

    """
    try:
        visual = initObjectProperties(
            obj, phobostype='visual', ignoretypes='geometry')
        visual['geometry'] = deriveGeometry(obj)
        visual['pose'] = deriveObjectPose(obj)
        if obj.lod_levels:
            if 'lodmaxdistances' in obj:
                maxdlist = obj['lodmaxdistances']
            else:
                maxdlist = [obj.lod_levels[
                    i + 1].distance for i in range(len(obj.lod_levels) - 1)] + [100.0]
            lodlist = []
            for i in range(len(obj.lod_levels)):
                filename = obj.lod_levels[
                    i].object.data.name + ioUtils.getOutputMeshtype()
                lodlist.append({'start': obj.lod_levels[i].distance, 'end': maxdlist[
                               i], 'filename': os.path.join('meshes', filename)})
            visual['lod'] = lodlist
    except KeyError:
        log("Missing data in visual object " + obj.name, "ERROR")
        return None
    return visual


def deriveCollision(obj):
    """This function derives the collision information from an object.

    Args:
      obj(bpy_types.Object): The blender object to derive the collision information from.

    Returns:
      dict

    """
    try:
        collision = initObjectProperties(
            obj, phobostype='collision', ignoretypes='geometry')
        collision['geometry'] = deriveGeometry(obj)
        collision['pose'] = deriveObjectPose(obj)
        # the bitmask is cut to length = 16 and reverted for int parsing
        try:
            collision['bitmask'] = int(''.join(
                ['1' if group else '0' for group in obj.rigid_body.collision_groups[:16]])[::-1], 2)
            for group in obj.rigid_body.collision_groups[16:]:
                if group:
                    log(('Object {0} is on a collision layer higher than ' +
                        '16. These layers are ignored when exporting.').format(
                        obj.name), "WARNING")
                    break
        except AttributeError:
            pass
    except KeyError:
        log("Missing data in collision object " + obj.name, "ERROR")
        return None
    return collision


def deriveCapsule(obj):
    """This function derives a capsule from a given blender object

    Args:
      obj(bpy_types.Object): The blender object to derive the capsule from.

    Returns:
      tuple

    """
    viscol_dict = {}
    capsule_pose = poses.deriveObjectPose(obj)
    rotation = capsule_pose['rotation_euler']
    capsule_radius = obj['geometry']['radius']
    for part in ['sphere1', 'cylinder', 'sphere2']:
        viscol = initObjectProperties(
            obj, phobostype='collision', ignoretypes='geometry')
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
            # TODO delete me?
            # print(list(matrix))
            pose['matrix'] = [list(vector) for vector in list(matrix)]
        viscol['geometry'] = geometry
        viscol['pose'] = pose
        try:
            viscol['bitmask'] = int(''.join(
                ['1' if group else '0' for group in obj.rigid_body.collision_groups]), 2)
        except AttributeError:
            pass
        viscol_dict[part] = viscol
    return viscol_dict, obj.parent


def deriveApproxsphere(obj):
    """This function derives an SRDF approximation sphere from a given blender object

    Args:
      obj(bpy_types.Object): The blender object to derive the approxsphere from.

    Returns:
      tuple

    """
    try:
        sphere = initObjectProperties(obj)
        sphere['radius'] = obj.dimensions[0] / 2
        pose = deriveObjectPose(obj)
        sphere['center'] = pose['translation']
    except KeyError:
        log("Missing data in collision approximation object " + obj.name, "ERROR")
        return None
    return sphere


def deriveSensor(obj):
    """This function derives a sensor from a given blender object

    Args:
      obj(bpy_types.Object): The blender object to derive the sensor from.

    Returns:
      dict

    """
    try:
        props = initObjectProperties(obj, phobostype='sensor')
        props['link'] = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
    except KeyError:
        log("Missing data in sensor " + obj.name, "ERROR")
        return None
    return props


def deriveController(obj):
    """This function derives a controller from a given blender object

    Args:
      obj(bpy_types.Object): The blender object to derive the controller from.

    Returns:
      dict

    """
    try:
        props = initObjectProperties(obj, phobostype='controller')
    except KeyError:
        log("Missing data in controller  " + obj.name, "ERROR",)
        return None
    return props


def deriveLight(obj):
    """This function derives a light from a given blender object

    Args:
      obj(bpy_types.Object): The blender object to derive the light from.

    Returns:
      tuple

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
        # TODO handle this somehow
        pass
    try:
        light['attenuation_quadratic'] = float(
            light_data.quadratic_attenuation)
    except AttributeError:
        pass
    if light_data.energy:
        light['attenuation_constant'] = float(light_data.energy)

    light['parent'] = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
    return light


def initObjectProperties(obj, phobostype=None, ignoretypes=()):
    """This function initializes a phobos data structure with a given object
    and derives basic information from its custom properties.

    Args:
      obj(bpy_types.Object): The object to derive initial properties from.
      phobostype(str, optional): optional phobostype of the object (Default value = None)
      ignoretypes(list, optional): phobostypes to ignore during parsing (Default value = ()

    Returns:
      dict

    """
    # allow duplicated names differentiated by types
    props = {'name': nUtils.getObjectName(obj, phobostype)}
    # if no phobostype is defined, everything is parsed
    if not phobostype:
        for key, value in obj.items():
            props[key] = value
    # if a phobostype is defined, we search for special custom properties
    else:
        for key, value in obj.items():
            # transform Blender id_arrays into lists
            if hasattr(value, 'to_list'):
                value = list(value)
            if '/' in key:
                if phobostype + '/' in key:
                    specs = key.split('/')[1:]
                    if len(specs) == 1:
                        props[key.replace(phobostype + '/', '')] = value
                    elif len(specs) == 2:
                        category, specifier = specs
                        if '$' + category not in props:
                            props['$' + category] = {}
                        props['$' + category][specifier] = value
                # ignore two-level specifiers if phobostype is not present
                elif key.count('/') == 1:
                    category, specifier = key.split('/')
                    if category not in ignoretypes:
                        if '$' + category not in props:
                            props['$' + category] = {}
                        props['$' + category][specifier] = value
    return props


def deriveDictEntry(obj):
    """Derives a phobos dictionary entry from the provided object.

    Args:
      obj(bpy_types.Object): The object to derive the dict entry (phobos data structure) from.

    Returns:
      tuple

    """
    props = {}
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
        log("A KeyError occurred due to missing data in object" + obj.name, "DEBUG")
        return None, None
    return props


def deriveGroupEntry(group):
    """Derives a list of phobos link skeletons for a provided group object.

    Args:
      group(bpy_types.Group): The blender group to extract the links from.

    Returns:
      list

    """
    links = []
    for obj in group.objects:
        if obj.phobostype == 'link':
            links.append({'type': 'link', 'name': nUtils.getObjectName(obj)})
        else:
            log("Group " + group.name + " contains " + obj.phobostype +
                ': ' + nUtils.getObjectName(obj), "ERROR")
    return links


def deriveChainEntry(obj):
    """Derives a phobos dict entry for a kinematic chain ending in the provided object.

    Args:
      obj: return:

    Returns:

    """
    returnchains = []
    if 'endChain' in obj:
        chainlist = obj['endChain']
    for chainName in chainlist:
        chainclosed = False
        parent = obj
        chain = {'name': chainName, 'start': '',
                 'end': nUtils.getObjectName(obj), 'elements': []}
        while not chainclosed:
            # FIXME: use effectiveParent
            if parent.parent is None:
                log("Unclosed chain, aborting parsing chain " + chainName, "ERROR")
                chain = None
                break
            chain['elements'].append(parent.name)
            # FIXME: use effectiveParent
            parent = parent.parent
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
    """Stores the current pose of all of a robot's selected joints.
    Existing poses of the same name will be overwritten.

    Args:
      modelname(str): The robot the pose belongs to.
      posename(str): The name the pose will be stored under.

    Returns:
      Nothing.

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
        for link in (link for link in links if 'joint/type' in link and
                     link['joint/type'] not in ['fixed', 'floating']):
            link.pose.bones['Bone'].rotation_mode = 'XYZ'
            posedict[posename]['joints'][nUtils.getObjectName(link, 'joint')] = link.pose.bones[
                'Bone'].rotation_euler.y
        bUtils.updateTextFile(filename, yaml.dump(
            posedict, default_flow_style=False))
    else:
        log("No model root could be found to store the pose for", "ERROR")


def loadPose(modelname, posename):
    """Load and apply a robot's stored pose.

    Args:
      modelname(str): The model's name.
      posename(str.
:return Nothing.): The name the pose is stored under.

    Returns:

    """
    load_file = bUtils.readTextFile(modelname + '::poses')
    if load_file == '':
        log('No poses stored.', 'ERROR')
        return
    poses = yaml.load(load_file)
    try:
        pose = poses[posename]
        prev_mode = bpy.context.mode
        bpy.ops.object.mode_set(mode='POSE')
        for obj in sUtils.getObjectsByPhobostypes(['link']):
            if nUtils.getObjectName(obj, 'joint') in pose['joints']:
                obj.pose.bones['Bone'].rotation_mode = 'XYZ'
                obj.pose.bones['Bone'].rotation_euler.y = float(
                    pose['joints'][nUtils.getObjectName(obj, 'joint')])
        bpy.ops.object.mode_set(mode=prev_mode)
    except KeyError:
        log('No pose with name ' + posename +
            ' stored for model ' + modelname, 'ERROR')


def getPoses(modelname):
    """Get the names of the poses that have been stored for a robot.

    Args:
      modelname: The model's name.

    Returns:
      A list containing the poses' names.

    """
    load_file = bUtils.readTextFile(modelname + '::poses')
    if load_file == '':
        return []
    poses = yaml.load(load_file)
    return poses.keys()


def deriveTextData(modelname):
    """Collect additional data stored for a specific model.

    Args:
      modelname: Name of the model for which data should be derived.

    Returns:
      A dictionary containing additional data.

    """
    datadict = {}
    datatextfiles = [
        text for text in bpy.data.texts if text.name.startswith(modelname + '::')]
    for text in datatextfiles:
        try:
            dataname = text.name.split('::')[-1]
        except IndexError:
            log("Possibly invalidly named model data text file: " + modelname, "WARNING")
        try:
            data = yaml.load(bUtils.readTextFile(text.name))
        except yaml.scanner.ScannerError:
            log("Invalid formatting of data file: " + dataname, "ERROR")
        if data:
            datadict[dataname] = data
    return datadict


def deriveModelDictionaryFromSubmodel(modelname):
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

    # collect general model properties
    model['date'] = datetime.now().strftime("%Y%m%d_%H:%M")
    model['name'] = modelname

    # collect all submodels
    submodels = [a for a in bpy.data.objects if a.phobostype == 'submodel']

    # namespace links, joints, motors etc for each submodel
    for subm in submodels:
        print('-----------------------', subm.name, subm['submodelname'], '\n')
        rootlink = [r for r in bpy.data.objects if sUtils.isRoot(r)
                    and r['modelname'] == subm['submodelname']][0]
        adict = buildModelDictionary(rootlink)
        for l in adict['links']:
            model['links'][namespaced(l, subm.name)] = namespaceLink(
                adict['links'][l], subm.name)
        for j in adict['joints']:
            model['joints'][namespaced(j, subm.name)] = namespaceJoint(
                adict['joints'][j], subm.name)
        for m in adict['motors']:
            model['motors'][namespaced(m, subm.name)] = namespaceMotor(
                adict['motors'][m], subm.name)
        for mat in adict['materials']:
            if mat not in model['materials']:
                model['materials'][mat] = adict['materials'][mat]
        for mesh in adict['meshes']:
            model['meshes'][namespaced(mesh, subm.name)] = adict['meshes'][mesh]
        print('\n\n')

    for subm in submodels:
        rootlink = [r for r in bpy.data.objects if sUtils.isRoot(r)
                    and r['modelname'] == subm['submodelname']][0]
        if subm.parent:
            # get interfaces and parents
            parentsubmodelname = subm.parent.parent.parent['submodelname']
            parentinterfacename = subm.parent.parent['interface/name']
            parentsubmodel = [r for r in bpy.data.objects if sUtils.isRoot(r)
                              and r['modelname'] == parentsubmodelname][0]
            parentinterface = [i for i in sUtils.getChildren(parentsubmodel,
                                                             ('interface',))
                               if i['interface/name'] == parentinterfacename][0]
            parentlinkname = parentinterface.parent.name

            # derive link pose for root link
            matrix = eUtils.getCombinedTransform(subm, subm.parent.parent.parent)
            pose = {'rawmatrix': matrix,
                    'matrix': [list(vector) for vector in list(matrix)],
                    'translation': list(matrix.to_translation()),
                    'rotation_euler': list(matrix.to_euler()),
                    'rotation_quaternion': list(matrix.to_quaternion())
                    }
            model['links'][namespaced(rootlink.name, subm.name)]['pose'] = pose

            # derive additional joint
            model['joints'][a.name] = deriveJoint(rootlink)
            #print(yaml.dump(model['joints'][a.name]))
            model['joints'][a.name]['name'] = namespaced(rootlink.name, a.name)
            model['joints'][a.name]['parent'] = namespaced(parentlinkname, a.parent.parent.parent.name)
            model['joints'][a.name]['child'] = namespaced(rootlink.name, a.name)
            #print(yaml.dump(model['joints'][a.name]))
        #print('######################')
        #for j in model['joints']:
        #    print(model['joints'][j]['name'], model['joints'][j]['child'], model['joints'][j]['child'])
        #print('######################')
    return model


def namespaceMotor(motor, namespace):
    motor['name'] = namespaced(motor['name'], namespace)
    motor['joint'] = namespaced(motor['joint'], namespace)
    return motor


def namespaceLink(link, namespace):
    link['name'] = namespaced(link['name'], namespace)
    for element in link['collision']:
        link['collision'][element]['name'] = namespaced(link['collision'][element]['name'], namespace)
    for element in link['visual']:
        link['visual'][element]['name'] = namespaced(link['visual'][element]['name'], namespace)
    return link


def namespaceJoint(joint, namespace):
    joint['name'] = namespaced(joint['name'], namespace)
    joint['child'] = namespaced(joint['child'], namespace)
    joint['parent'] = namespaced(joint['parent'], namespace)
    return joint

def namespaced(name, namespace):
    return namespace+'_'+name


def buildModelDictionary(root, name=''):
    """Builds a python dictionary representation of a Phobos model.

    Args:
      root: bpy.types.Object
      name:  (Default value = '')

    Returns:
      dict

    """
    if root.phobostype not in ['link', 'submodel']:
        log(root.name + " is no valid 'link' or 'submodel' object.", "ERROR")
        return None

    if name:
        modelname = name
    elif 'modelname' in root:
        modelname = root['modelname']
    else:
        modelname = 'unnamed'

    model = {'links': {},
             'joints': {},
             'sensors': {},
             'motors': {},
             'controllers': {},
             'materials': {},
             'meshes': {},
             'lights': {},
             'groups': {},
             'chains': {},
             'date': datetime.now().strftime("%Y%m%d_%H:%M"),
             'name': modelname
             }

    log("Creating dictionary for model " + modelname + " with root " + root.name, "INFO")

    # create tuples of objects belonging to model
    objectlist = sUtils.getChildren(root,
                                    selected_only=ioUtils.getExpSettings().selectedOnly,
                                    include_hidden=False)
    linklist = [link for link in objectlist if link.phobostype == 'link']

    # digest all the links to derive link and joint information
    log("Parsing links, joints and motors..." + (str(len(linklist))), "INFO")
    for link in linklist:
        # parse link and extract joint and motor information
        linkdict, jointdict, motordict = deriveKinematics(link)
        model['links'][linkdict['name']] = linkdict
        # joint may be None if link is a root
        if jointdict:
            model['joints'][jointdict['name']] = jointdict
        # motor may be None if no motor is attached or link is a root
        if motordict:
            model['motors'][motordict['name']] = motordict

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
            model['links'][linkname]['inertial'] = {
                'mass': mv, 'inertia': iv,
                'pose': {'translation': list(cv),
                         'rotation_euler': [0, 0, 0]}
            }

    # complete link information by parsing visuals and collision objects
    log("Parsing visual and collision (approximation) objects...", "INFO")
    for obj in objectlist:
        # try:
        if obj.phobostype in ['visual', 'collision']:
            props = deriveDictEntry(obj)
            parentname = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
            model['links'][parentname][obj.phobostype][
                nUtils.getObjectName(obj)] = props
        elif obj.phobostype == 'approxsphere':
            props = deriveDictEntry(obj)
            parentname = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
            model['links'][parentname]['approxcollision'].append(props)

        # TODO delete me?
        # except KeyError:
        #    try:
        #        log(parentname + " not found", "ERROR")
        #    except TypeError:
        #        log("No parent found for " + obj.name, "ERROR")

    # combine collision information for links
    for linkname in model['links']:
        link = model['links'][linkname]
        bitmask = 0
        for collname in link['collision']:
            try:
                # bitwise OR to add all collision layers
                bitmask = bitmask | link['collision'][collname]['bitmask']
            except KeyError:
                pass
        link['collision_bitmask'] = bitmask

    # parse sensors and controllers
    log("Parsing sensors and controllers...", "INFO")
    for obj in objectlist:
        if obj.phobostype in ['sensor', 'controller']:
            props = deriveDictEntry(obj)
            model[obj.phobostype + 's'][nUtils.getObjectName(obj)] = props

    # parse materials
    log("Parsing materials...", "INFO")
    model['materials'] = collectMaterials(objectlist)
    for obj in objectlist:
        if obj.phobostype == 'visual':
            mat = obj.active_material
            try:
                if mat.name not in model['materials']:
                    # this should actually never happen
                    model['materials'][mat.name] = deriveMaterial(
                        mat)
                linkname = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
                model['links'][linkname]['visual'][nUtils.getObjectName(obj)][
                    'material'] = mat.name
            except AttributeError:
                log("Could not parse material for object " + obj.name, "ERROR")

    # identify unique meshes
    log("Parsing meshes...", "INFO")
    for obj in objectlist:
        try:
            if ((obj.phobostype == 'visual' or
                 obj.phobostype == 'collision') and
                    (obj['geometry/type'] == 'mesh') and
                    (obj.data.name not in model['meshes'])):
                model['meshes'][obj.data.name] = obj
                for lod in obj.lod_levels:
                    if lod.object.data.name not in model['meshes']:
                        model['meshes'][lod.object.data.name] = lod.object
        except KeyError:
            log("Undefined geometry type in object " + obj.name, "ERROR")

    # gather information on groups of objects
    log("Parsing groups...", "INFO")
    # TODO: get rid of the "data" part and check for relation to robot
    for group in bpy.data.groups:
        # skip empty groups
        if not group.objects:
            continue

        # handle submodel groups separately from other groups
        if 'submodeltype' in group.keys():
            continue
            # TODO create code to derive Submodels
            # model['submodels'] = deriveSubmodel(group)
        elif nUtils.getObjectName(group, 'group') != "RigidBodyWorld":
            model['groups'][nUtils.getObjectName(
                group, 'group')] = deriveGroupEntry(group)

    # gather information on chains of objects
    log("Parsing chains...", "INFO")
    chains = []
    for obj in objectlist:
        if obj.phobostype == 'link' and 'endChain' in obj:
            chains.extend(deriveChainEntry(obj))
    for chain in chains:
        model['chains'][chain['name']] = chain

    # gather information on lights
    log("Parsing lights...", "INFO")
    for obj in objectlist:
        if obj.phobostype == 'light':
            model['lights'][nUtils.getObjectName(obj)] = deriveLight(obj)

    # gather submechanism information from links
    log("Parsing submechanisms...", "INFO")
    submechanisms = []
    for link in linklist:
        if 'submechanism/name' in link.keys():
            #for key in [key for key in link.keys() if key.startswith('submechanism/')]:
            #    submechanisms.append({key.replace('submechanism/', ''): value
            #                        for key, value in link.items()})
            submech = {'type': link['submechanism/type'],
                       'contextual_name': link['submechanism/name'],
                       'jointnames_independent': [j.name for j in link['submechanism/independent']],
                       'jointnames_spanningtree': [j.name for j in link['submechanism/spanningtree']],
                       'jointnames_active': [j.name for j in link['submechanism/active']]
                       }
            submechanisms.append(submech)
    model['submechanisms'] = submechanisms

    # add additional data to model
    model.update(deriveTextData(model['name']))

    # shorten numbers in dictionary to n decimalPlaces and return it
    log("Rounding numbers...", "INFO")
    # TODO: implement this separately
    epsilon = 10**(-ioUtils.getExpSettings().decimalPlaces)
    return epsilonToZero(model, epsilon, ioUtils.getExpSettings().decimalPlaces)


def buildModelFromDictionary(model):
    """Creates the Blender representation of the imported model, using a model dictionary.

    Args:
      model:

    Returns:

    """
    # DOCU add some more docstring
    log("Creating Blender model...", 'INFO')

    log("Creating links...", 'INFO')
    for l in model['links']:
        link = model['links'][l]
        model['links'][l]['object'] = linkmodel.createLink(link)

    log("Setting parent-child relationships", 'INFO')
    bUtils.toggleLayer(defs.layerTypes['link'], True)
    for l in model['links']:
        parent = model['links'][l]
        for c in parent['children']:
            child = model['links'][c]
            child['object'].matrix_world = parent['object'].matrix_world
            sUtils.selectObjects([child['object'], parent['object']], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')

    log("Creating joints...", 'INFO')
    for j in model['joints']:
        joint = model['joints'][j]
        jointmodel.createJoint(joint)
    log('...finished.', 'INFO')

    # set transformations
    log("Placing links...", 'INFO')
    for l in model['links']:
        if 'parent' not in model['links'][l]:
            root = model['links'][l]
            break
    linkmodel.placeChildLinks(model, root)

    log("Assigning model name...", 'INFO')
    try:
        rootlink = sUtils.getRoot(bpy.data.objects[root['name']])
        rootlink['modelname'] = model['name']
        rootlink.location = (0, 0, 0)
    except (KeyError, NameError):
        log("Could not assign model name to root link.", "ERROR")

    try:
        log("Creating sensors...", 'INFO')
        for s in model['sensors']:
            sensormodel.createSensor(model['sensors'][s])
    except KeyError:
        log("No sensors in model " + model['name'], 'INFO')

    try:
        log("Creating motors...", 'INFO')
        for m in model['motors']:
            eUtils.addDictionaryToObj(model['motors'][m],
                                      model['joints'][
                                          model['motors'][m]['joint']],
                                      category='motor')
    except KeyError:
        log("No motors in model " + model['name'], 'INFO')

    try:
        log("Creating controllers...", 'INFO')
        for c in model['controllers']:
            controllermodel.createController(model['controllers'][c])
    except KeyError:
        log("No controllers in model " + model['name'], 'INFO')

    try:
        log("Creating groups...", 'INFO')
        for g in model['groups']:
            createGroup(model['groups'][g])
    except KeyError:
        log("No kinematic groups in model " + model['name'], 'INFO')

    try:
        log("Creating chains...", 'INFO')
        for ch in model['chains']:
            createChain(model['chains'][ch])
    except KeyError:
        log("No kinematic chains in model " + model['name'], 'INFO')

    try:
        log("Creating lights...", 'INFO')
        for l in model['lights']:
            lightmodel.createLight(model['lights'][l])
    except KeyError:
        log("No lights in model " + model['name'], 'INFO')

    # display all objects after import
    for obj in bpy.data.objects:
        bUtils.setObjectLayersActive(obj)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.view3d.view_selected()
    # update transformations
    bUtils.update()


def createGroup(group):
    # TODO lots of code missing here... make it a dev branch
    pass


def createChain(group):
    # TODO lots of code missing here... make it a dev branch
    pass
