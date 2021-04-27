#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import os
import copy
from datetime import datetime
import json

import bpy
import mathutils

import phobos.defs as defs
import phobos.model.links as linkmodel
import phobos.model.inertia as inertiamodel
import phobos.model.joints as jointmodel
import phobos.model.sensors as sensormodel
import phobos.model.lights as lightmodel
import phobos.model.motors as motormodel
import phobos.model.controllers as controllermodel
import phobos.model.materials as matmodel
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
      : dict -- dictionary of materials

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
      mat(bpy.types.Material): Blender material to derive a Phobos description from
      logging: (Default value = False)
      errors: (Default value = None)

    Returns:
      : dict -- Phobos representation of the material

    """
    # TODO: annotations to materials could be used to fuse annotation objects' materials for different
    #       graphics engines of simulations etc., currently works by adding custom properties
    if "No material defined." in errors:
        return {}

    material = initObjectProperties(mat, 'material', includeannotations=False)

    material['name'] = mat.name

    # create color dictionaries
    material['diffuseColor'] = dict(
        zip(['r', 'g', 'b'], [1.0 * num for num in list(mat.diffuse_color[:3])])
    )
    material['ambientColor'] = dict(
        zip(
            ['r', 'g', 'b'],
            [1.0 * num for num in list(mat.diffuse_color)],
        )
    )
    material['specularColor'] = dict(
        zip(['r', 'g', 'b'], [mat.specular_intensity * num for num in list(mat.specular_color)])
    )
    # todo: if mat.emit > 0:
    #     material['emissionColor'] = dict(
    #         zip(
    #             ['r', 'g', 'b'],
    #             [mat.emit * mat.specular_intensity * num for num in list(mat.specular_color)],
    #         )
    #     )
    # todo: material['shininess'] = mat.specular_hardness / 2
    material['shininess'] = mat.roughness * 100
    #todo: if mat.use_transparency:
    if mat.diffuse_color[3] != 1.0:
        material['transparency'] = 1.0 - mat.diffuse_color[3]

    # return material without texture information if there are validation errors
    if errors:
        return material

    if mat.node_tree:
        textures = [x for x in mat.node_tree.nodes if x.type=='TEX_IMAGE']
        print(textures[0])
    # there are always 18 slots, regardless of whether they are filled or not
    for tex in mat.texture_slots:
        if tex is not None:
            # regular diffuse color texture
            if tex.use_map_color_diffuse:
                # grab the first texture
                material['diffuseTexture'] = mat.texture_slots[0].texture.image.filepath.replace(
                    '//', ''
                )
            # normal map
            if tex.use_map_normal:
                # grab the first texture
                material['normalTexture'] = mat.texture_slots[0].texture.image.filepath.replace(
                    '//', ''
                )
            # displacement map
            if tex.use_map_displacement:
                # grab the first texture
                material['displacementTexture'] = mat.texture_slots[
                    0
                ].texture.image.filepath.replace('//', '')
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
    .. seealso deriveInertial (Default value = [])
      logging: (Default value = False)
      errors: (Default value = None)

    Returns:

    """
    # use scene objects if no objects are defined
    if not objectlist:
        objectlist = list(bpy.context.scene.objects)

    if logging:
        log("Deriving link from object " + linkobj.name + ".", 'DEBUG')
    props = initObjectProperties(
        linkobj, phobostype='link', ignoretypes=linkobjignoretypes - {'link'}
    )
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
    for obj in [
        item for item in objectlist if item.phobostype in ['visual', 'collision', 'approxsphere']
    ]:
        effectiveparent = sUtils.getEffectiveParent(obj)
        if effectiveparent == linkobj:
            if logging:
                log(
                    "  Adding " + obj.phobostype + " '" + nUtils.getObjectName(obj) + "' to link.",
                    'DEBUG',
                )
            if obj.phobostype == 'approxsphere':
                props['approxcollision'].append(deriveDictEntry(obj))
            else:
                props[obj.phobostype][nUtils.getObjectName(obj)] = deriveDictEntry(obj)

    # gather the inertials for fusing the link inertia
    inertials = inertiamodel.gatherInertialChilds(linkobj, objectlist)

    # get inertia data
    mass, com, inertia = inertiamodel.fuse_inertia_data(inertials)

    if not any([mass, com, inertia]):
        if logging:
            log("No inertia information for link object " + linkobj.name + ".", 'DEBUG')
    else:
        # add inertia to link
        inertia = inertiamodel.inertiaMatrixToList(inertia)
        props['inertial'] = {
            'mass': mass,
            'inertia': list(inertia),
            'pose': {'translation': list(com), 'rotation_euler': [0, 0, 0]},
        }

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
    
    The link information is derived according to :func:`derive_link`.

    Args:
      linkobj(bpy.types.Object): blender object to derive the link from

    Returns:
      dict: link representation of the object

    """
    props = initObjectProperties(
        linkobj, phobostype='link', ignoretypes=['joint', 'motor', 'entity']
    )

    parent = sUtils.getEffectiveParent(linkobj)
    props['parent'] = parent.name if parent else None
    props['pose'] = deriveObjectPose(linkobj)
    props['joint'] = deriveJoint(linkobj, logging=False, adjust=False)
    del props['joint']['parent']

    # collect collision objs for link
    collisionobjs = sUtils.getImmediateChildren(
        linkobj, phobostypes=('collision'), include_hidden=True
    )
    collisiondict = {}
    for colobj in collisionobjs:
        collisiondict[colobj.name] = colobj
    props['collision'] = collisiondict

    # collect visual objs for link
    visualobjects = sUtils.getImmediateChildren(
        linkobj, phobostypes=('visual'), include_hidden=True
    )
    visualdict = {}
    for visualobj in visualobjects:
        visualdict[visualobj.name] = visualobj
    props["visual"] = visualdict

    # collect inertial objects
    inertialdict = {
        nUtils.getObjectName(obj): obj for obj in linkobj.children if obj.phobostype == 'inertial'
    }
    props["inertial"] = inertialdict

    # collect sensor objects
    sensorobjects = sUtils.getImmediateChildren(
        linkobj, phobostypes=('sensor'), include_hidden=True
    )
    sensordict = {}
    for sensorobj in sensorobjects:
        sensordict[sensorobj.name] = sensorobj
    if sensordict:
        props["sensor"] = sensordict

    props['approxcollision'] = []
    return props


@validate('joint')
def deriveJoint(obj, logging=False, adjust=False, errors=None):
    """Derives a joint from a blender object and creates its initial phobos data structure.

    Args:
      obj(bpy.types.Object): object to derive the joint from
      adjust(bool, optional): TODO (Default value = False)
      logging: (Default value = False)
      errors: (Default value = None)

    Returns:
      : dict

    """
    joint_type, crot = jointmodel.deriveJointType(obj, adjust=adjust, logging=logging)
    props = initObjectProperties(
        obj, phobostype='joint', ignoretypes=linkobjignoretypes - {'joint'}
    )

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
    if 'maxSpeed' in props:
        limits['velocity'] = props['maxSpeed']
        del props['maxSpeed']
    if 'maxspeed' in props:
        limits['velocity'] = props['maxspeed']
        del props['maxspeed']
    if 'maxVelocity' in props:
        limits['velocity'] = props['maxVelocity']
        del props['maxVelocity']
    if 'maxvelocity' in props:
        limits['velocity'] = props['maxvelocity']
        del props['maxvelocity']
    if 'maxEffort' in props:
        limits['effort'] = props['maxEffort']
        del props['maxEffort']
    if 'maxeffort' in props:
        limits['effort'] = props['maxeffort']
        del props['maxeffort']
    if limits != {}:
        props['limits'] = limits
    props['pose'] = deriveObjectPose(obj)
    if "$motor" in props:
        # transfer motor limits
        props["$motor"]["maxEffort"] = limits['effort']
        props["$motor"]["maxSpeed"] = limits['velocity']
    # TODO: what about these?
    # - calibration
    # - dynamics
    # - mimic
    # - safety_controller
    return props


@validate('inertia_data')
def deriveInertial(obj, logging=True, **kwargs):
    """Returns a dictionary describing the inertial information represented by the provided object.
    
    Contains these keys:
        *mass*: float
        *inertia*: list
        *pose*: inertiapose containing:
            *translation*: center of mass of the objects
            *rotation*: [0., 0., 0.]

    Args:
      obj(bpy.types.Object): object of phobostype 'inertial'
      logging(bool, optional): whether to log information or not (Default value = True)
      **kwargs: 

    Returns:

    """
    if obj.phobostype != 'inertial':
        if logging:
            log("Object '{0}' is not of phobostype 'inertial'.".format(obj.name), 'ERROR')
        return None

    props = initObjectProperties(obj, phobostype='inertial')
    props['pose'] = deriveObjectPose(obj, logging=logging, **kwargs)
    return props


@validate('visual')
def deriveVisual(obj, logging=True, **kwargs):
    """This function derives the visual information from an object.
    
    Contains these keys:
        *name*: name of the visual
        *geometry*: derived according to `deriveGeometry`
        *pose*: derived according to `deriveObjectPose`
        *lod*: (opt.) level of detail levels

    Args:
      obj(bpy.types.Object): object to derive the visual representation from
      logging: (Default value = True)
      **kwargs: 

    Returns:
      : dict -- model representation of the visual object

    """
    visual = initObjectProperties(obj, phobostype='visual', ignoretypes='geometry')
    visual['geometry'] = deriveGeometry(obj, logging=logging)
    visual['pose'] = deriveObjectPose(obj, logging=logging)

    # check for material of the visual
    material = deriveMaterial(obj.active_material, logging=logging)
    if material:
        visual['material'] = material['name']

    #todo2.9: if obj.lod_levels:
    #     if 'lodmaxdistances' in obj:
    #         maxdlist = obj['lodmaxdistances']
    #     else:
    #         maxdlist = [obj.lod_levels[i + 1].distance for i in range(len(obj.lod_levels) - 1)] + [
    #             100.0
    #         ]
    #     lodlist = []
    #     for i in range(len(obj.lod_levels)):
    #         filename = obj.lod_levels[i].object.data.name + ioUtils.getOutputMeshtype()
    #         lodlist.append(
    #             {
    #                 'start': obj.lod_levels[i].distance,
    #                 'end': maxdlist[i],
    #                 'filename': os.path.join('meshes', filename),
    #             }
    #         )
    #     visual['lod'] = lodlist
    return visual


def deriveCollision(obj):
    """Returns the collision information from the specified object.

    Args:
      obj(bpy.types.Object): object to derive the collision information from

    Returns:
      : dict -- phobos representation of the collision object

    """
    collision = initObjectProperties(obj, phobostype='collision', ignoretypes='geometry')
    collision['geometry'] = deriveGeometry(obj)
    collision['pose'] = deriveObjectPose(obj)

    # the bitmask is cut to length = 16 and reverted for int parsing
    if 'collision_groups' in dir(obj.rigid_body):
        collision['bitmask'] = int(
            ''.join(['1' if group else '0' for group in obj.rigid_body.collision_groups[:16]])[
                ::-1
            ],
            2,
        )
        for group in obj.rigid_body.collision_groups[16:]:
            if group:
                log(
                    (
                        "Object {0} is on a collision layer higher than 16. These layers are "
                        + "ignored when exporting."
                    ).format(obj.name),
                    'WARNING',
                )
                break
    return collision


def deriveApproxsphere(obj):
    """This function derives an SRDF approximation sphere from a given blender object

    Args:
      obj(bpy_types.Object): The blender object to derive the approxsphere from.

    Returns:
      : tuple

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


def deriveLight(obj):
    """This function derives a light from a given blender object

    Args:
      obj(bpy_types.Object): The blender object to derive the light from.

    Returns:
      : tuple

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
        light['attenuation_quadratic'] = float(light_data.quadratic_attenuation)
    except AttributeError:
        pass
    if light_data.energy:
        light['attenuation_constant'] = float(light_data.energy)

    light['parent'] = nUtils.getObjectName(sUtils.getEffectiveParent(obj))
    return light


def recursive_dictionary_cleanup(dictionary):
    """Recursively enrich the dictionary and replace object links with names etc.
    
    These patterns are replaced:
        [phobostype, bpyobj] -> {'object': bpyobj, 'name': getObjectName(bpyobj, phobostype)}

    Args:
      dictionary(dict): dictionary to enrich

    Returns:
      : dict -- dictionary with replace/enriched patterns

    """
    for key, value in dictionary.items():
        # handle everything as list, so we can loop over it
        unlist = False
        if not isinstance(value, list):
            value = [value]
            unlist = True

        itemlist = []
        for item in value:
            if isinstance(item, list) and item:
                # (phobostype, bpyobj) -> {'object': bpyobj, 'name': getObjectName(bpyobj)}
                if (
                    len(item) == 2
                    and isinstance(item[0], str)
                    and (item[0] in ['joint'] + [enum[0] for enum in defs.phobostypes])
                    and isinstance(item[1], bpy.types.Object)
                ):
                    itemlist.append(
                        {
                            'object': item[1],
                            'name': nUtils.getObjectName(item[1], phobostype=item[0]),
                        }
                    )

            # recursion on subdictionaries
            elif isinstance(item, dict):
                itemlist.append(recursive_dictionary_cleanup(item))
            else:
                itemlist.append(item)

        # extract single items back out of the list
        dictionary[key] = itemlist if not unlist else itemlist[0]
    return dictionary


def initObjectProperties(
    obj, phobostype=None, ignoretypes=(), includeannotations=True, ignorename=False
):
    """Initializes phobos dictionary of *obj*, including information stored in custom properties.

    Args:
      obj(bpy_types.Object): object to derive initial properties from.
      phobostype(str, optional): limit parsing of data fields to this phobostype (Default value = None)
      ignoretypes(list, optional): list of properties ignored while initializing the objects properties. (Default value = ())
      ignorename(bool, optional): whether or not to add the object's name (Default value = False)
      includeannotations: (Default value = True)

    Returns:
      : dict -- phobos properties of the object

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
                # TODO make this work for all levels of hierarchy
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
            log(
                "  Adding annotations from {}.".format(
                    nUtils.getObjectName(annot, phobostype='annotation')
                ),
                'DEBUG',
            )
            props.update(
                initObjectProperties(
                    annot, phobostype, ignoretypes, includeannotations, ignorename=True
                )
            )

    # recursively enrich the property dictionary
    props = recursive_dictionary_cleanup(props)

    return props


def deriveDictEntry(obj, names=False, objectlist=[], logging=True, adjust=True):
    """Derives a phobos dictionary entry from the provided object.

    Args:
      obj(bpy_types.Object): The object to derive the dict entry (phobos data structure) from.
      names(bool, optional): use object names as dict entries instead of object links. (Default value = False)
      logging(bool, optional): whether to log messages or not (Default value = True)
      objectlist: (Default value = [])
      adjust: (Default value = True)

    Returns:
      : dict -- phobos representation of the object

    """
    props = {}
    try:
        if obj.phobostype == 'inertial':
            props = deriveInertial(obj, adjust=adjust, logging=logging)
        elif obj.phobostype == 'visual':
            props = deriveVisual(obj)
        elif obj.phobostype == 'collision':
            props = deriveCollision(obj)
        elif obj.phobostype == 'approxsphere':
            props = deriveApproxsphere(obj)
        elif obj.phobostype == 'sensor':
            props = sensormodel.deriveSensor(
                obj, names=names, objectlist=objectlist, logging=logging
            )
        elif obj.phobostype == 'controller':
            props = controllermodel.deriveController(obj)
        elif obj.phobostype == 'light':
            props = deriveLight(obj)
        elif obj.phobostype == 'motor':
            props = motormodel.deriveMotor(obj)
        elif obj.phobostype == 'annotation':
            props = deriveAnnotation(obj)
    except KeyError:
        log("A KeyError occurred due to missing data in object" + obj.name, "DEBUG")
        return None, None
    return props


def deriveAnnotation(obj):
    """Derives the annotation info of an object.

    Args:
      obj: TODO

    Returns:

    """
    props = {}

    props = initObjectProperties(obj, includeannotations=False)
    return props


def deriveGroupEntry(group):
    """Derives a list of phobos link skeletons for a provided group object.

    Args:
      group(bpy_types.Group): The blender group to extract the links from.

    Returns:
      : list

    """
    links = []
    for obj in group.objects:
        if obj.phobostype == 'link':
            links.append({'type': 'link', 'name': nUtils.getObjectName(obj)})
        else:
            log(
                "Group "
                + group.name
                + " contains "
                + obj.phobostype
                + ': '
                + nUtils.getObjectName(obj),
                "ERROR",
            )
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
        chain = {'name': chainName, 'start': '', 'end': nUtils.getObjectName(obj), 'elements': []}
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
      : A dictionary containing additional data.

    """
    datadict = {}
    datatextfiles = [text for text in bpy.data.texts if text.name.startswith(modelname + '::')]
    for text in datatextfiles:
        try:
            dataname = text.name.split('::')[-1]
        except IndexError:
            log("Possibly invalidly named model data text file: " + modelname, "WARNING")
        try:
            data = json.loads(bUtils.readTextFile(text.name))
        except:
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
#         print('-----------------------', subm.name, subm['submodel/name'], '\n')
#         rootlink = [r for r in bpy.data.objects if sUtils.isRoot(r)
#                     and r['model/name'] == subm['submodel/name']][0]
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
#                     and r['model/name'] == subm['submodel/name']][0]
#         if subm.parent:
#             # get interfaces and parents
#             parentsubmodelname = subm.parent.parent.parent['submodel/name']
#             parentinterfacename = subm.parent.parent['interface/name']
#             parentsubmodel = [r for r in bpy.data.objects if sUtils.isRoot(r)
#                               and r['model/name'] == parentsubmodelname][0]
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
#             # print(json.dumps(model['joints'][a.name]))
#             model['joints'][a.name]['name'] = namespaced(rootlink.name, a.name)
#             model['joints'][a.name]['parent'] = namespaced(parentlinkname, a.parent.parent.parent.name)
#             model['joints'][a.name]['child'] = namespaced(rootlink.name, a.name)
#             # print(json.dumps(model['joints'][a.name]))
#     return model


def namespaceMotor(motor, namespace):
    """

    Args:
      motor: 
      namespace: 

    Returns:

    """
    motor['name'] = namespaced(motor['name'], namespace)
    motor['joint'] = namespaced(motor['joint'], namespace)
    return motor


def namespaceLink(link, namespace):
    """

    Args:
      link: 
      namespace: 

    Returns:

    """
    link['name'] = namespaced(link['name'], namespace)
    for element in link['collision']:
        link['collision'][element]['name'] = namespaced(
            link['collision'][element]['name'], namespace
        )
    for element in link['visual']:
        link['visual'][element]['name'] = namespaced(link['visual'][element]['name'], namespace)
    return link


def namespaceJoint(joint, namespace):
    """

    Args:
      joint: 
      namespace: 

    Returns:

    """
    joint['name'] = namespaced(joint['name'], namespace)
    joint['child'] = namespaced(joint['child'], namespace)
    joint['parent'] = namespaced(joint['parent'], namespace)
    return joint


def namespaced(name, namespace):
    """

    Args:
      name: 
      namespace: 

    Returns:

    """
    return namespace + '_' + name


def deriveModelDictionary(root, name='', objectlist=[]):
    """Returns a dictionary representation of a Phobos model.
    
    If name is not specified, it overrides the modelname in the root. If the modelname is not
    defined at all, 'unnamed' will be used instead.

    Args:
      root(bpy_types.Object): root object of the model
      name(str, optional): name for the derived model (Default value = '')
      objectlist(list: bpy_types.Object): objects to derive the model from
      objectlist: (Default value = [])

    Returns:

    """
    if root.phobostype not in ['link', 'submodel']:
        log(root.name + " is no valid 'link' or 'submodel' object.", "ERROR")
        return None

    # define model name
    if name:
        modelname = name
    elif 'model/name' in root:
        modelname = root['model/name']
    else:
        modelname = 'unnamed'

    # define model version
    if 'model/version' in root:
        modelversion = root['model/version']
    else:
        modelversion = 'undefined'

    modeldescription = bUtils.readTextFile('README.md')

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
        'name': modelname,
        'version': modelversion,
        'description': modeldescription,
    }

    log(
        "Creating dictionary for model '" + modelname + "' with root '" + root.name + "'.",
        'INFO',
        prefix="\n",
    )

    # create tuples of objects belonging to model
    if not objectlist:
        objectlist = sUtils.getChildren(
            root, selected_only=ioUtils.getExpSettings().selectedOnly, include_hidden=False
        )
    linklist = [link for link in objectlist if link.phobostype == 'link']

    # digest all the links to derive link and joint information
    log("Parsing links, joints and motors... " + (str(len(linklist))) + " total.", "INFO")
    for link in linklist:
        # parse link information (including inertia)
        model['links'][nUtils.getObjectName(link, 'link')] = deriveLink(
            link, logging=True, objectlist=objectlist
        )

        # parse joint and motor information
        if sUtils.getEffectiveParent(link):
            # joint may be None if link is a root
            # to prevent confusion links are always defining also joints
            jointdict = deriveJoint(link, logging=True, adjust=True)
            log("  Setting joint type '{}' for link.".format(jointdict['type']), 'DEBUG')
            # first check if we have motor information in the joint properties
            # if so they can be extended/overwritten by motor objects later on
            if '$motor' in jointdict:
                motordict = jointdict['$motor']
                # at least we need a type property
                if 'type' in motordict:
                    # if no name is given derive it from the joint
                    if not 'name' in motordict:
                        motordict["name"] = jointdict['name']
                    model['motors'][motordict['name']] = motordict
                    # link the joint by name:
                    motordict['joint'] = jointdict['name']
                del jointdict['$motor']

            model['joints'][jointdict['name']] = jointdict

            for mot in [child for child in link.children if child.phobostype == 'motor']:
                motordict = motormodel.deriveMotor(mot, jointdict)
                # motor may be None if no motor is attached
                if motordict:
                    log("  Added motor {} to link.".format(motordict['name']), 'DEBUG')
                    if motordict['name'] in model["motors"]:
                        model['motors'][motordict['name']].update(motordict)
                    else:
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
                    linkname = nUtils.getObjectName(
                        sUtils.getEffectiveParent(obj, ignore_selection=bool(objectlist))
                    )
                    model['links'][linkname]['visual'][nUtils.getObjectName(obj)][
                        'material'
                    ] = mat.name

    # identify unique meshes
    log("Parsing meshes...", "INFO")
    for obj in objectlist:
        try:
            if (
                (obj.phobostype == 'visual' or obj.phobostype == 'collision')
                and (obj['geometry/type'] == 'mesh')
                and (obj.data.name not in model['meshes'])
            ):
                model['meshes'][obj.data.name] = obj
                #todo2.9: for lod in obj.lod_levels:
                #     if lod.object.data.name not in model['meshes']:
                #         model['meshes'][lod.object.data.name] = lod.object
        except KeyError:
            log("Undefined geometry type in object " + obj.name, "ERROR")

    # gather information on groups of objects
    log("Parsing groups...", 'INFO')
    #todo2.9: TODO: get rid of the "data" part and check for relation to robot
    # for group in bpy.data.groups:
    #     # skip empty groups
    #     if not group.objects:
    #         continue

    #     # handle submodel groups separately from other groups
    #     if 'submodeltype' in group.keys():
    #         continue
    #         # TODO create code to derive Submodels
    #         # model['submodels'] = deriveSubmodel(group)
    #     elif nUtils.getObjectName(group, 'group') != "RigidBodyWorld":
    #         model['groups'][nUtils.getObjectName(group, 'group')] = deriveGroupEntry(group)

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
        """

        Args:
          link: 

        Returns:

        """
        if 'submechanism/name' in link.keys():
            submech = {
                'type': link['submechanism/type'],
                'contextual_name': link['submechanism/name'],
                'name': link['submechanism/subtype']
                if 'submechanism/subtype' in link
                else link['submechanism/type'],
                'jointnames_independent': [
                    nUtils.getObjectName(j, 'joint') for j in link['submechanism/independent']
                ],
                'jointnames_spanningtree': [
                    nUtils.getObjectName(j, 'joint') for j in link['submechanism/spanningtree']
                ],
                'jointnames_active': [
                    nUtils.getObjectName(j, 'joint') for j in link['submechanism/active']
                ],
                # TODO: this should work in almost all cases, still a bit of a hack:
                'file_path': '../submechanisms/urdf/' + link['submechanism/name'] + '.urdf',
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
      model(dict): model representation of the imported model

    Returns:

    """
    log("Creating Blender model...", 'INFO', prefix='\n' + '-' * 25 + '\n')

    log("  Initializing materials... ({} total)".format(len(model['materials'])), 'INFO')
    for mat in model['materials']:
        matmodel.createMaterial(model['materials'][mat], logging=True, adjust=True)
        # ['name'], tuple(mat['color'][0:3]), (1, 1, 1), mat['color'][-1])

    newobjects = []
    log("  Creating links... ({} total)".format(len(model['links'])), 'INFO')
    for lnk in model['links']:
        link = model['links'][lnk]
        model['links'][lnk]['object'] = linkmodel.createLink(link)
        newobjects.append(model['links'][lnk]['object'])
        newobjects.extend(model['links'][lnk]['object'].children)

    log("Setting parent-child relationships", 'INFO', prefix='\n')
    bUtils.toggleLayer('link', True)
    for lnk in model['links']:
        parent = model['links'][lnk]

        log("Children for link " + parent['name'] + ":\n" + '\n'.join(parent['children']), 'DEBUG')
        for chi in parent['children']:
            child = model['links'][chi]
            child['object'].matrix_world = parent['object'].matrix_world
            eUtils.parentObjectsTo(child['object'], parent['object'])

    # set transformations
    log("Transforming links...  ({} total)".format(len(model['links'])), 'INFO', prefix='\n')
    for lnk in model['links']:
        if 'parent' not in model['links'][lnk]:
            root = model['links'][lnk]
            break
    linkmodel.setLinkTransformations(model, root)

    log("Creating joints... ({} total)".format(len(model['joints'])), 'INFO', prefix='\n')
    for j in model['joints']:
        joint = model['joints'][j]
        jointmodel.createJoint(joint, links=model['links'])

    log("Assigning model name: {}".format(model['name']), 'INFO')
    rootlink = sUtils.getRoot(bpy.data.objects[root['object'].name])
    if 'name' not in model:
        log("Model name not specified in URDF. Make sure to define it thereafter.", 'WARNING')
    else:
        rootlink['model/name'] = model['name']
    rootlink.location = (0, 0, 0)

    # TODO make sure this works
    log("Creating sensors...", 'INFO')
    if 'sensors' in model and model['sensors']:
        for sen in model['sensors']:
            sensormodel.createSensor(model['sensors'][sen], model['sensors'][sen]['parent'])
    else:
        log("  No sensors in model.", 'INFO')

    # TODO make sure this works
    log("Creating motors...", 'INFO')
    if 'motors' in model and model['motors']:
        for motor in model['motors']:
            eUtils.setProperties(
                model['joints'][model['motors'][motor]['joint']],
                model['motors'][motor],
                category='motor',
            )
    else:
        log("  No motors in model.", 'INFO')

    # TODO make sure this works
    log("Creating groups...", 'INFO')
    if 'groups' in model and model['groups']:
        for group in model['groups']:
            createGroup(model['groups'][group])
    else:
        log("  No kinematic groups in model.", 'INFO')

    # TODO make sure this works
    log("Creating chains...", 'INFO')
    if 'chains' in model and model['chains']:
        for ch in model['chains']:
            createChain(model['chains'][ch])
    else:
        log("  No kinematic chains in model.", 'INFO')

    # TODO make sure this works
    log("Creating lights...", 'INFO')
    if 'lights' in model and model['lights']:
        for light in model['lights']:
            lightmodel.createLight(model['lights'][light])
    else:
        log("  No lights in model.", 'INFO')

    # display new objects after import
    sUtils.selectObjects(newobjects, clear=True, active=0)
    eUtils.sortObjectsToLayers(newobjects)
    for obj in newobjects:
        bUtils.setObjectLayersActive(obj, extendlayers=True)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.view3d.view_selected()

    # update the scene
    bUtils.update()


def gatherAnnotations(model):
    """Gathers custom properties annotating elements of the robot
    across the model. These annotations were created in the model.py
    module and are marked with a leading '$'.

    Args:
      model(dict): The robot model dictionary.
      ignore_keys(list): Ignored annotation categories (Default value = [])

    Returns:
      : dict -- A dictionary of the gathered annotations.

    """
    # TODO check this stuff
    annotations = {}
    elementlist = []
    types = ('links', 'joints', 'sensors', 'motors', 'controllers', 'materials')
    # gather information from directly accessible types
    for objtype in types:
        for elementname in model[objtype]:
            tmpdict = model[objtype][elementname]
            tmpdict['temp_type'] = objtype[:-1]
            elementlist.append(tmpdict)

    # add information from types hidden in links
    for linkname in model['links']:
        for objtype in ('collision', 'visual'):
            if objtype in model['links'][linkname]:
                for elementname in model['links'][linkname][objtype]:
                    tmpdict = model['links'][linkname][objtype][elementname]
                    tmpdict['temp_type'] = objtype
                    elementlist.append(tmpdict)
        if 'inertial' in model['links'][linkname]:
            tmpdict = model['links'][linkname]['inertial']
            tmpdict['temp_type'] = 'inertial'
            elementlist.append(tmpdict)

    # loop through the list of annotated elements and categorize the data
    for element in elementlist:
        delkeys = []
        for key in element.keys():
            if key.startswith('$'):
                category = key[1:]
                # ignore motor properties for link and joint types
                if category == "motor":
                    if element['temp_type'] == "link" or element['temp_type'] == "joint":
                        continue
                if category not in annotations:
                    annotations[category] = {}
                if element['temp_type'] not in annotations[category]:
                    annotations[category][element['temp_type']] = []
                tmpdict = {k: element[key][k] for k in element[key]}
                tmpdict['name'] = element['name']
                annotations[category][element['temp_type']].append(tmpdict)
                delkeys.append(key)
        delkeys.append('temp_type')
        for key in delkeys:
            if key in element:
                del element[key]

    return annotations


def replace_object_links(dictionary):
    """Replaces object links in a dictionary with object names.
    
    This is required for generic parsed object definitions, as object links are represented by a
    simple dictionary with *name* and *object*.
    
    For most exports, this can be run prior export parsing, to create the respective name linking
    within the model.

    Args:
      dictionary(dict): model dictionary (or similar) to replace object links in

    Returns:
      : dict -- model dictionary with name linking instead of object links

    """
    newdict = {}
    if isinstance(dictionary, list):
        newlist = []
        for item in dictionary:
            newlist.append(replace_object_links(item))
        return newlist

    for key, value in dictionary.items():
        if isinstance(value, list):
            if all([isinstance(item, dict) for item in value]) and all(
                [('name' in item and 'object' in item) for item in value]
            ):
                newdict[key] = sorted([item['name'] for item in value])

        elif isinstance(value, dict):
            newdict[key] = replace_object_links(value)
        else:
            newdict[key] = value

    return newdict


def createGroup(group):
    """

    Args:
      group: 

    Returns:

    """
    # TODO lots of code missing here... make it a dev branch
    pass


def createChain(group):
    """

    Args:
      group: 

    Returns:

    """
    # TODO lots of code missing here... make it a dev branch
    pass
