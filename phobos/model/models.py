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
from phobos.utils.validation import validate
from phobos.phoboslog import log
from phobos.utils.general import roundFloatsInDict, sortListsInDict
from phobos.model.poses import deriveObjectPose
from phobos.model.geometries import deriveGeometry
from phobos.defs import linkobjignoretypes


def collectMaterials(objectlist):
    """Returns a dictionary of materials contained in a list of objects.

    Only visual objects are considered and the dict keys represent the material names.

    If a material is used by multiple objects, the *user* count is increased by one.

    Args:
        objectlist(list): list of objects to derive dictionary from

    Returns:
        dict -- dictionary of materials
    """
    materials = {}
    for obj in objectlist:
        if obj.phobostype == 'visual':
            mat = obj.active_material
            if mat:
                if mat.name not in materials:
                    materials[mat.name] = deriveMaterial(mat)
                    materials[mat.name]['users'] = 1
                else:
                    materials[mat.name]['users'] += 1
    return materials


@validate('material')
def deriveMaterial(mat, logging=False, errors=None):
    """Returns a Phobos representation of a Blender material.

    Colors are returned as a dictionary with three keys ('r', 'g', 'b').

    It contains always:
        *name*: name of the material
        *diffuseColor*: the diffuse color of the material
        *ambientColor*: the ambient color of the material
        *specularColor*: the specular color of the material

    Depending on the material and texture configuration it might also contain:
        *emissionColor*: the emission color of the material
        *transparency*: the transparency of the material
        *diffuseTexture*: the diffuse texture file path of the material
        *normalTexture*: the normal texture file path of the material
        *displacementTexture*: the displacement texture file path of the material

    Args:
      mat (bpy.types.Material): Blender material to derive a Phobos description from

    Returns:
        dict -- Phobos representation of the material
    """
    # TODO: annotations to materials could be used to fuse annotation objects' materials for different
    #       graphics engines of simulations etc., currently works by adding custom properties
    if "No material defined." in errors:
        return {}

    material = initObjectProperties(mat, 'material', includeannotations=False)

    material['name'] = mat.name

    # create color dictionaries
    material['diffuseColor'] = dict(
        zip(['r', 'g', 'b'], [mat.diffuse_intensity * num for num in list(mat.diffuse_color)]))
    material['ambientColor'] = dict(
        zip(['r', 'g', 'b'], [mat.ambient * mat.diffuse_intensity * num for num in list(
            mat.diffuse_color)]))
    material['specularColor'] = dict(
        zip(['r', 'g', 'b'], [mat.specular_intensity * num for num in list(mat.specular_color)]))
    if mat.emit > 0:
        material['emissionColor'] = dict(
            zip(['r', 'g', 'b'], [mat.emit * mat.specular_intensity * num for num in list(
                mat.specular_color)]))
    material['shininess'] = mat.specular_hardness / 2
    if mat.use_transparency:
        material['transparency'] = 1.0 - mat.alpha

    # return material without texture information if there are validation errors
    if errors:
        return material

    # there are always 18 slots, regardless of whether they are filled or not
    for tex in mat.texture_slots:
        if tex is not None:
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
    return material


@validate('link')
def deriveLink(linkobj, objectlist=[], logging=False, errors=None):
    """Derives a dictionary for the link represented by the provided obj.

    If objectlist is provided, only objects contained in the list are taken into account
    for creating the resulting dictionary.

    The dictionary contains (besides any generic object properties) this information:
        *parent*: name of parent object or None
        *children*: list of names of the links child links
        *object*: bpy.types.Object which represents the link
        *pose*: deriveObjectPose of the linkobj
        *collision*: empty dictionary
        *visual*: empty dictionary
        *inertial*: derive_inertia of all child inertials of the link
        *approxcollision*: empty dictionary

    Args:
        linkobj(bpy.types.Object): blender object to derive the link from
        objectlist: list of bpy.types.Object

    .. seealso deriveObjectPose
    .. seealso deriveInertial
    """
    # use scene objects if no objects are defined
    if not objectlist:
        objectlist = list(bpy.context.scene.objects)

    log("Deriving link from object " + linkobj.name + ".", 'DEBUG')
    props = initObjectProperties(linkobj, phobostype='link',
                                 ignoretypes=linkobjignoretypes - {'link'})
    parent = sUtils.getEffectiveParent(linkobj, objectlist)
    props['parent'] = nUtils.getObjectName(parent) if parent else None
    props['parentobj'] = parent
    props['children'] = [child.name for child in linkobj.children if child.phobostype == 'link']
    props['object'] = linkobj
    props['pose'] = deriveObjectPose(linkobj)
    props['collision'] = {}
    props['visual'] = {}
    props['inertial'] = {}
    props['approxcollision'] = []

    # gather all visual/collision objects for the link from the objectlist
    for obj in [item for item in objectlist if item.phobostype in ['visual', 'collision',
                                                                   'approxsphere']]:
        effectiveparent = sUtils.getEffectiveParent(obj, ignore_selection=bool(objectlist))
        if effectiveparent == linkobj:
            log("  Adding " + obj.phobostype + " '" + nUtils.getObjectName(obj) + "' to link.",
                'DEBUG')
            if obj.phobostype == 'approxsphere':
                props['approxcollision'].append(deriveDictEntry(obj))
            else:
                props[obj.phobostype][nUtils.getObjectName(obj)] = deriveDictEntry(obj)

    # gather the inertials for fusing the link inertia
    inertials = inertiamodel.gatherInertialChilds(linkobj, objectlist)

    # get inertia data
    mass, com, inertia = inertiamodel.fuse_inertia_data(inertials)

    if not any([mass, com, inertia]):
        log("No inertia information for link object " + linkobj.name + ".", 'DEBUG')
    else:
        # add inertia to link
        inertia = inertiamodel.inertiaMatrixToList(inertia)
        props['inertial'] = {
            'mass': mass,
            'inertia': list(inertia),
            'pose': {'translation': list(com), 'rotation_euler': [0, 0, 0]}}

    bitmask = 0
    for collname in props['collision']:
        try:
            # bitwise OR to add all collision layers
            bitmask = bitmask | props['collision'][collname]['bitmask']
        except KeyError:
            pass
    props['collision_bitmask'] = bitmask

    return props


def get_link_information(linkobj):
    """Returns the full link information including joint and motor data from a blender object.

    :param linkobj: blender object to derive the link from
    :type linkobj: bpy.types.Object

    :return: representation of the link including motor and joint data
    :rtype: dict

    .. seealso:: derive_link
    """
    assert linkobj.phobostype == 'link', ("Wrong phobostype: " + linkobj.phobostype +
                                          " instead of link.")

    props = initObjectProperties(linkobj, phobostype='link',
                                 ignoretypes=['joint', 'motor', 'entity'])

    parent = sUtils.getEffectiveParent(linkobj)
    props['parent'] = parent.name if parent else None
    props['pose'] = deriveObjectPose(linkobj)
    props['joint'] = deriveJoint(linkobj, logging=False, adjust=False)
    del props['joint']['parent']

    # derive Motor
    if any(item.startswith('motor') for item in props):
        props['motor'] = deriveMotor(linkobj, props['joint'])

    # collect collision objs for link
    collisionobjs = sUtils.getImmediateChildren(
        linkobj, phobostypes=('collision'), include_hidden=True)
    collisiondict = {}
    for colobj in collisionobjs:
        collisiondict[colobj.name] = colobj
    props['collision'] = collisiondict

    # collect visual objs for link
    visualobjects = sUtils.getImmediateChildren(
        linkobj, phobostypes=('visual'), include_hidden=True)
    visualdict = {}
    for visualobj in visualobjects:
        visualdict[visualobj.name] = visualobj
    props["visual"] = visualdict

    # collect inertial objects
    inertialdict = {nUtils.getObjectName(obj): obj for obj in linkobj.children
                    if obj.phobostype == 'inertial'}
    props["inertial"] = inertialdict

    # collect sensor objects
    sensorobjects = sUtils.getImmediateChildren(
        linkobj, phobostypes=('sensor'), include_hidden=True)
    sensordict = {}
    for sensorobj in sensorobjects:
        sensordict[sensorobj.name] = sensorobj
    props["sensor"] = sensordict

    props['approxcollision'] = []
    return props


@validate('joint')
def deriveJoint(obj, logging=False, adjust=False, errors=None):
    """Derives a joint from a blender object and creates its initial phobos data structure.

    Args:
      obj (bpy.types.Object): object to derive the joint from
      adjust (bool): TODO

    Returns:
      dict
    """
    joint_type, crot = jointmodel.deriveJointType(obj, adjust=adjust, logging=logging)
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


def deriveInertial(obj):
    """Returns a dictionary describing the inertial information represented by the provided object.

    Contains these keys:
        *mass*: float
        *inertia*: list
        *pose*: inertiapose containing:
            *translation*: center of mass of the objects
            *rotation*: [0., 0., 0.]

    Args:
        obj(bpy.types.Object): object of phobostype 'inertial'
    """
    if obj.phobostype != 'inertial':
        log("Object '{0}' is not of phobostype 'inertial'.".format(obj.name), 'ERROR')
        return None

    props = initObjectProperties(obj, phobostype='inertial')
    props['pose'] = deriveObjectPose(obj)
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

        # check for material of the visual
        material = deriveMaterial(obj.active_material, logging=True)
        if material:
            visual['material'] = material['name']

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


def deriveSensor(obj, names=False, objectlist=[]):
    """This function derives a sensor from a given blender object

    Args:
        obj(bpy_types.Object): The blender object to derive the sensor from.
        names(bool): return the link object name instead of an object link.
        objectlist (list(bpy.types.Object)): objectlist to which possible parents are restricted

    Returns:
      dict
    """
    log("Deriving sensor from object " + nUtils.getObjectName(obj, phobostype='sensor') + ".",
        'DEBUG')
    try:
        props = initObjectProperties(obj, phobostype='sensor')
        if names:
            props['link'] = nUtils.getObjectName(
                sUtils.getEffectiveParent(obj, objectlist=objectlist), phobostype='sensor')
        else:
            props['link'] = sUtils.getEffectiveParent(obj, objectlist=objectlist)
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


def initObjectProperties(obj, phobostype=None, ignoretypes=(), includeannotations=True,
                         ignorename=False):
    """Initializes phobos dictionary of *obj*, including information stored in custom properties.

    Args:
      obj(bpy_types.Object): object to derive initial properties from.
      phobostype(str, optional): limit parsing of data fields to this phobostype
      ignoretypes(list, optional): list of properties ignored while initializing the objects properties.
      ignorename(bool, optional): whether or not to add the object's name

    Returns:
      dict

    """
    # allow duplicated names differentiated by types
    props = {} if ignorename else {'name': nUtils.getObjectName(obj, phobostype)}

    # if no phobostype is defined, everything is parsed
    if not phobostype:
        for key, value in obj.items():
            props[key] = value

    # search for type-specific properties if phobostype is defined
    else:
        for key, value in obj.items():
            # transform Blender id_arrays into lists
            if hasattr(value, 'to_list'):
                value = list(value)

            # remove phobostype namespaces for the object
            if key.startswith(phobostype + '/'):
                if key.count('/') == 1:
                    props[key.replace(phobostype + '/', '')] = value
                # TODO why do we need the $?
                elif key.count('/') == 2:
                    category, specifier = key.split('/')[1:]
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

    # collect phobostype specific annotations from child objects
    if includeannotations:
        annotationobjs = sUtils.getImmediateChildren(obj, ('annotation',), selected_only=True)
        for annot in annotationobjs:
            log("  Adding annotations from {}.".format(nUtils.getObjectName(
                annot, phobostype='annotation')), 'DEBUG')
            props.update(initObjectProperties(annot, phobostype, ignoretypes, includeannotations,
                                              ignorename=True))
    return props


def deriveDictEntry(obj, names=False, objectlist=[]):
    """Derives a phobos dictionary entry from the provided object.

    Args:
      obj(bpy_types.Object): The object to derive the dict entry (phobos data structure) from.
      names(bool): use object names as dict entries instead of object links.

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
            props = deriveSensor(obj, names=names, objectlist=objectlist)
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


# TODO remove unused function? (there are also undefined variables in it)
# def deriveModelDictionaryFromSubmodel(modelname):
#     """Derive a dictionary from a submodel.

#     :param modelname: name of the submodel to derive from
#     :type modelname: str

#     :return: representation of the submodel
#     :rtype: dict
#     """
#     assert isinstance(modelname, str), "Modelname is not a string: " + type(modelname)
#     model = {
#         'links': {},
#         'joints': {},
#         'sensors': {},
#         'motors': {},
#         'controllers': {},
#         'materials': {},
#         'meshes': {},
#         'lights': {},
#         'groups': {},
#         'chains': {}
#     }

#     # collect general model properties
#     model['date'] = datetime.now().strftime("%Y%m%d_%H:%M")
#     model['name'] = modelname

#     # collect all submodels
#     submodels = [a for a in bpy.data.objects if a.phobostype == 'submodel']

#     # namespace links, joints, motors etc for each submodel
#     for subm in submodels:
#         print('-----------------------', subm.name, subm['submodelname'], '\n')
#         rootlink = [r for r in bpy.data.objects if sUtils.isRoot(r)
#                     and r['modelname'] == subm['submodelname']][0]
#         adict = deriveModelDictionary(rootlink)
#         for l in adict['links']:
#             model['links'][namespaced(l, subm.name)] = namespaceLink(
#                 adict['links'][l], subm.name)
#         for j in adict['joints']:
#             model['joints'][namespaced(j, subm.name)] = namespaceJoint(
#                 adict['joints'][j], subm.name)
#         for m in adict['motors']:
#             model['motors'][namespaced(m, subm.name)] = namespaceMotor(
#                 adict['motors'][m], subm.name)
#         for mat in adict['materials']:
#             if mat not in model['materials']:
#                 model['materials'][mat] = adict['materials'][mat]
#         for mesh in adict['meshes']:
#             model['meshes'][namespaced(mesh, subm.name)] = adict['meshes'][mesh]
#         print('\n\n')

#     for subm in submodels:
#         rootlink = [r for r in bpy.data.objects if sUtils.isRoot(r)
#                     and r['modelname'] == subm['submodelname']][0]
#         if subm.parent:
#             # get interfaces and parents
#             parentsubmodelname = subm.parent.parent.parent['submodelname']
#             parentinterfacename = subm.parent.parent['interface/name']
#             parentsubmodel = [r for r in bpy.data.objects if sUtils.isRoot(r)
#                               and r['modelname'] == parentsubmodelname][0]
#             parentinterface = [i for i in sUtils.getChildren(parentsubmodel,
#                                                              ('interface',))
#                                if i['interface/name'] == parentinterfacename][0]
#             parentlinkname = parentinterface.parent.name

#             # derive link pose for root link
#             matrix = eUtils.getCombinedTransform(subm, subm.parent.parent.parent)
#             pose = {'rawmatrix': matrix,
#                     'matrix': [list(vector) for vector in list(matrix)],
#                     'translation': list(matrix.to_translation()),
#                     'rotation_euler': list(matrix.to_euler()),
#                     'rotation_quaternion': list(matrix.to_quaternion())
#                     }
#             model['links'][namespaced(rootlink.name, subm.name)]['pose'] = pose

#             # derive additional joint
#             model['joints'][a.name] = deriveJoint(rootlink, adjust=True)
#             # print(yaml.dump(model['joints'][a.name]))
#             model['joints'][a.name]['name'] = namespaced(rootlink.name, a.name)
#             model['joints'][a.name]['parent'] = namespaced(parentlinkname, a.parent.parent.parent.name)
#             model['joints'][a.name]['child'] = namespaced(rootlink.name, a.name)
#             # print(yaml.dump(model['joints'][a.name]))
#     return model


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


def deriveModelDictionary(root, name='', objectlist=[]):
    """Returns a dictionary representation of a Phobos model.

    If name is not specified, it overrides the modelname in the root. If the modelname is not
    defined at all, 'unnamed' will be used instead.

    Args:
        root(bpy_types.Object): root object of the model
        name(str): name for the derived model
        objectlist(list: bpy_types.Object): objects to derive the model from
    """
    if root.phobostype not in ['link', 'submodel']:
        log(root.name + " is no valid 'link' or 'submodel' object.", "ERROR")
        return None

    # define model name
    if name:
        modelname = name
    elif 'modelname' in root:
        modelname = root['modelname']
    else:
        modelname = 'unnamed'

    model = {
        'links': {},
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

    log("Creating dictionary for model '" + modelname + "' with root '" + root.name + "'.", 'INFO',
        prefix="\n")

    # create tuples of objects belonging to model
    if not objectlist:
        objectlist = sUtils.getChildren(root,
                                        selected_only=ioUtils.getExpSettings().selectedOnly,
                                        include_hidden=False)
    linklist = [link for link in objectlist if link.phobostype == 'link']

    # digest all the links to derive link and joint information
    log("Parsing links, joints and motors... " + (str(len(linklist))) + " total.", "INFO")
    for link in linklist:
        # parse link information (including inertia)
        model['links'][nUtils.getObjectName(link, 'link')] = deriveLink(link, logging=True,
                                                                        objectlist=objectlist)

        # parse joint and motor information
        if sUtils.getEffectiveParent(link):
            # joint may be None if link is a root
            jointdict = deriveJoint(link, logging=True, adjust=True)
            model['joints'][jointdict['name']] = jointdict

            # TODO check this
            motordict = deriveMotor(link, jointdict)
            # motor may be None if no motor is attached
            if motordict:
                model['motors'][motordict['name']] = motordict

    # parse sensors and controllers
    sencons = [obj for obj in objectlist if obj.phobostype in ['sensor', 'controller']]
    log("Parsing sensors and controllers... {} total.".format(len(sencons)), 'INFO')
    for obj in sencons:
        props = deriveDictEntry(obj, names=True, objectlist=objectlist)
        model[obj.phobostype + 's'][nUtils.getObjectName(obj)] = props

    # parse materials
    log("Parsing materials...", 'INFO')
    model['materials'] = collectMaterials(objectlist)
    for obj in objectlist:
        if obj.phobostype == 'visual':
            mat = obj.active_material
            if mat:
                if mat.name not in model['materials']:
                    model['materials'][mat.name] = deriveMaterial(mat)
                    linkname = nUtils.getObjectName(sUtils.getEffectiveParent(obj,
                        ignore_selection=bool(objectlist)))
                    model['links'][linkname]['visual'][nUtils.getObjectName(obj)]['material'] = mat.name

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
    log("Parsing groups...", 'INFO')
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

    def getSubmechanisms(link):
        if 'submechanism/name' in link.keys():
            submech = {'type': link['submechanism/type'],
                       'contextual_name': link['submechanism/name'],
                       'name': link['submechanism/subtype'] if 'submechanism/subtype' in link else link['submechanism/type'],
                       'jointnames_independent': [nUtils.getObjectName(j, 'joint') for j in
                                                  link['submechanism/independent']],
                       'jointnames_spanningtree': [nUtils.getObjectName(j, 'joint') for j in
                                                   link['submechanism/spanningtree']],
                       'jointnames_active': [nUtils.getObjectName(j, 'joint') for j in
                                             link['submechanism/active']],
                       # TODO: this should work in almost all cases, still a bit of a hack:
                       'file_path': '../submechanisms/urdf/' + link['submechanism/name'] + '.urdf'
                       }
            log('    ' + submech['contextual_name'], 'DEBUG')
        else:
            submech = None
        mechanisms = [submech] if submech else []
        for c in link.children:
            if c.phobostype in ['link', 'interface'] and c in objectlist:
                mechanisms.extend(getSubmechanisms(c))
        return mechanisms

    model['submechanisms'] = getSubmechanisms(root)

    # add additional data to model
    model.update(deriveTextData(model['name']))

    # shorten numbers in dictionary to n decimalPlaces and return it
    log("Rounding numbers to {} digits.".format(ioUtils.getExpSettings().decimalPlaces), 'INFO')
    model = roundFloatsInDict(model, ioUtils.getExpSettings().decimalPlaces)
    log("Sorting objects.", 'DEBUG')
    model = sortListsInDict(model)

    return model


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
    linkmodel.setLinkTransformations(model, root)

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
