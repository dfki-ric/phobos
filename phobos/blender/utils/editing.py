#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Contains the utility functions for editing objects and Phobos models.
"""

import math

import bpy
import mathutils

from .. import defs
from ..phoboslog import log
from ..utils import blender as bUtils
from ..utils import io as ioUtils
from ..utils import naming as nUtils
from ..utils import selection as sUtils


def dissolveLink(obj, delete_other=False):
    """Remove the selected link and reparent all links, inertia, visual and collisions to its effective Parent.

    Args:
      obj(bpy.types.Object): the link to dissolve
      delete_other: (Default value = False)

    Returns:

    """

    # Store original layers and show all layers
    originallayers = {}
    for name, coll in bpy.context.window.view_layer.layer_collection.children.items():
        originallayers[name] = coll.exclude
        coll.exclude = False

    if not obj.phobostype == 'link':
        log('Selected object {} is not a link!'.format(obj.name), 'ERROR')
        return

    else:
        delete = [obj]
        # Get all children
        children = sUtils.getRecursiveChildren(
            obj, phobostypes=('link', 'inertial', 'visual', 'collision'), include_hidden=True
        )

        if delete_other:
            other_children = sUtils.getRecursiveChildren(
                obj,
                recursion_depth=2,
                phobostypes=('sensor', 'submodel'),
                include_hidden=True,
            )
            delete += [child for child in other_children if child not in children]

        # Get the parent
        parent = obj.parent
        # If parent is not None ( Root )
        if obj.parent:
            # Reparent
            parentObjectsTo(children, parent, clear=True)
            # Delete the objects
            sUtils.selectObjects(delete, clear=True, active=-1)
            bpy.ops.object.delete()
    # Restore original layers
    for key, value in originallayers.items():
        bpy.context.window.view_layer.layer_collection.children[key].exclude = value


def getCombinedTransform(obj, effectiveparent):
    """Get the combined transform of the object relative to the effective parent.
    
    This combines all transformations in the parenting hierarchy up to the specified effective
    parent.
    
    Note, that the scale transformation of the effective parent is used anyway, as it scales the
    local matrix of the child object.

    Args:
      obj(bpy.types.Object): the child object
      effectiveparent(bpy.types.Object): the effective parent of the child object

    Returns:
      : bpy.types.Matrix -- the combined transformations of the child object

    """
    parent = obj.parent
    matrix = obj.matrix_local

    # use the parents absolute scale to scale the relative matrix
    if parent:
        scale_mat = mathutils.Matrix.Identity(4)
        scale_mat[0][0], scale_mat[1][1], scale_mat[2][2] = parent.matrix_world.to_scale()
        matrix = scale_mat @ matrix

    # combine transformations up to effective parent
    while parent is not None:
        if parent == effectiveparent:
            break

        # use relative rotation
        matrix = parent.matrix_local @ matrix
        parent = parent.parent

    return matrix


def restructureKinematicTree(link, root=None):
    """Restructures a tree such that the ``link`` provided becomes the root of the tree.
    
    If no root object is provided, :func:`phobos.utils.selection.getRoot` will be used.
    
    For instance, the following tree::
    
           A
          / \\
         B   C
        / \   \\
       D   E   F
    
    would, using the call restructureKinematicsTree(C), become::
    
            C
           / \\
          A   F
         /
        B
       / \\
      D   E
    
    Currently, this function ignores all options such as unselected or hidden objects.

    Args:
      link(bpy.types.Object): the link which will become the new root object
      root(bpy.types.Object, optional): the current root object (Default value = None)

    Returns:
      None: None

    """
    if not root:
        root = sUtils.getRoot(link)
    links = [link]
    obj = link

    # stop right now when the link is already root
    if not obj.parent:
        log('No restructure necessary. Link is already root.', 'INFO')
        return

    # gather chain of links ascending the tree
    while obj.parent.name != root.name:
        obj = obj.parent
        if obj.phobostype == 'link':
            links.append(obj)
    links.append(root)

    log("Unparenting objects for restructure: " + str([link.name for link in links]) + ".", 'DEBUG')
    # unparent all links
    sUtils.selectObjects(links, True)
    bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')

    log("Restructuring objects for new hierarchy.", 'DEBUG')
    for i in range(len(links) - 1):
        parent = links[i]
        child = links[i + 1]
        parentObjectsTo(child, parent)

    log("Copying model information from old root.", 'DEBUG')
    # copy properties
    if 'model/name' in root:
        link['model/name'] = root['model/name']
        del root['model/name']
    if 'model/version' in root:
        link['model/version'] = root['model/version']
        del root['model/version']
    log("Restructured kinematic tree to new root: {}.".format(link.name), 'INFO')


def parentObjectsTo(objects, parent, clear=False):
    """Parents the specified objects to the parent object.
    
    Depending on their phobostype the objects are parented either *bone relative* or *object*.
    
    If *clear* is set, the parenting of the objects will be cleared (keeping the transform), before
    parenting.

    Args:
      objects(list(bpy.types.Object): objects to set parent of
      parent(bpy.types.Object): parent object
      clear(bool, optional): if True, the parenting of the objects will be cleared (Default value = False)

    Returns:

    """
    if isinstance(objects, tuple):
        objects = list(objects)
    elif not isinstance(objects, list):
        objects = [objects]

    # Store original layers
    #originallayers = list(bpy.context.scene.layers)
    # Select all layers
    #bpy.context.scene.layers = [True for i in range(20)]
    # Restore original layers
    #bpy.context.scene.layers = originallayers

    if clear:
        sUtils.selectObjects(objects, active=0, clear=True)
        bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')

    sUtils.selectObjects([parent] + objects, active=0, clear=True)

    if parent.phobostype == 'link':
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
    else:
        bpy.ops.object.parent_set(type='OBJECT')


def getNearestCommonParent(objs):
    """Returns hierarchically lowest common parent of the provided objects

    Args:
      objs: list of objects (bpy_types.Object)

    Returns:

    """
    anchor = objs[0]  # pick one link as the anchor link
    rest = objs[1:]  # get other links to iterate over
    in_all = False  # this will be true if all 'rest' branches have parent as a common parent
    parent = anchor  # the candidate for common parent
    inter_objects = set()
    while not in_all and parent.parent:
        in_all = True
        parent = parent.parent  # go up the anchor branch
        inter_objects.add(parent)
        for obj in rest:  # start at base of each rest branch
            o = obj
            while (
                o.parent and o.parent != parent
            ):  # as long as there is a parent that is not the candidate parent
                o = o.parent
                inter_objects.add(o)
            if (
                o.parent != parent
            ):  # check which break condition happened, break if not arrived at parent
                in_all = False
                break
    if not in_all:  # this is only true if none of the branches set it to False and broke afterwards
        return None, []
    else:
        inter_objects.remove(parent)
        return parent, list(inter_objects)


def instantiateSubmodel(submodelname, instancename, size=1.0):
    """Creates an instance of the submodel specified by the submodelname.
    
    The instance receives the definitions of the group as it is generated.

    Args:
      submodelname: name of the submodel (Blender group) to create an
    instance of
      instancename: name the instance object will receive
      size: (Default value = 1.0)

    Returns:

    """
    submodel = None
    interfaces = None

    # find the existing group for submodel and interface
    for group in bpy.data.groups:
        # search for namespaced groups with the exact name
        if ':' in group.name and submodelname == group.name:
            submodel = group
        if group.name.startswith('interfaces:') and submodelname.split(':')[1] in group.name:
            interfaces = group

    if not submodel:
        log('Selected submodel is not defined.', 'ERROR')
    if not interfaces:
        log('No interfaces defined for this submodel.', 'INFO')

    # add the submodel and write in data
    bpy.ops.object.group_instance_add(group=submodel.name)
    submodelobj = bpy.context.active_object
    submodelobj.phobostype = 'submodel'
    submodelobj['submodeltype'] = submodel.name.split(':')[0]
    # TODO currently this works only by name binding, we should add links to
    # the group here
    submodelobj['submodel/name'] = submodelname
    submodelobj['submodelname'] = submodelname
    # copy custom props from group to instance
    for key in submodel.keys():
        submodelobj[key] = submodel[key]
    submodelobj.name = instancename
    submodelobj.empty_draw_size = size

    # add the interfaces if available
    if interfaces:
        # create group and make real
        bpy.ops.object.group_instance_add(group=interfaces.name)
        bpy.ops.object.duplicates_make_real()

        # write interface parameters and change namespace
        for obj in bpy.context.selected_objects:
            nUtils.addNamespace(obj, instancename)
            obj.name = obj.name.rsplit('.')[0]
            obj['submodeltype'] = 'interface'
            bUtils.toggleTransformLock(obj, True)

        # parent interfaces to submodel empty
        parentObjectsTo(bpy.context.selected_objects, submodelobj)

        # delete empty parent object of interfaces
        sUtils.selectObjects(
            objects=[
                a
                for a in bpy.context.selected_objects
                if a.type == 'EMPTY' and 'submodeltype' in a and a['submodeltype'] == 'interface'
            ],
            clear=True,
            active=0,
        )
        bpy.ops.object.delete(use_global=False)
    return submodelobj


def defineSubmodel(submodelname, submodeltype, version='', objects=None):
    """Defines a new submodule group with the specified name and type.
    
    The group will be named like so:
        'submodeltype:submodelname/version'
    
    Objects with the phobostype 'interface' (if present) are handled separately
    and put into a respective submodel group (which features the 'interface'
    submodeltype).
    
    If the version is omitted, the respective part of the name is dropped, too.
    If no object list is provided the objects are derived from selection.
    The submodeltype is also added as dict entry to the group in Blender.
    
    The selected objects are moved to the respective layer for submodels or
    interfaces.

    Args:
      submodelname: descriptive name of the submodel
      submodeltype: type of the submodel (e.g. 'fmu', 'mechanics')
      version: a version string (e.g. '1.0', 'dangerous') (Default value = '')
      objects: the objects which belong to the submodel (None will derive
    objects from the selection) (Default value = None)

    Returns:
      : a tuple of the submodelgroup and interfacegroup/None

    """
    if not objects:
        objects = bpy.context.selected_objects

    # split interface from physical objects
    interfaces = [i for i in objects if i.phobostype == 'interface']
    physical_objects = [p for p in objects if p.phobostype != 'interface']

    # make the physical group
    sUtils.selectObjects(physical_objects, True, 0)
    submodelgroupname = submodeltype + ':' + submodelname
    if version != '':
        submodelgroupname += '/' + version
    if submodelgroupname in bpy.data.groups.keys():
        log('submodelgroupname ' + 'already exists', 'WARNING')
    bpy.ops.group.create(name=submodelgroupname)
    submodelgroup = bpy.data.groups[submodelgroupname]
    submodelgroup['submodeltype'] = submodeltype
    submodelgroup['version'] = version

    modeldefs = defs.definitions['submodeltypes'][submodeltype]

    # copy the definition parameters to the group properties
    for key in modeldefs['definitions']:
        submodelgroup[key] = modeldefs['definitions'][key]

    # move objects to submodel layer
    for obj in physical_objects:
        if not 'submodel' in bpy.context.scene.collection.children.keys():
            newcollection = bpy.data.collections.new('submodel')
            bpy.context.scene.collection.children.link(newcollection)
        for name, collection in bpy.context.scene.collection.children.items():
            if name == 'submodel':
                collection.objects.link(obj)
            elif obj.name in collection.objects:
                collection.objects.unlink(obj)
    log('Created submodel group ' + submodelname + ' of type "' + submodeltype + '".', 'DEBUG')

    interfacegroup = None
    # make the interface group
    if interfaces:
        sUtils.selectObjects(interfaces, True, 0)
        interfacegroupname = 'interfaces:' + submodelname
        if version != '':
            interfacegroupname += '/' + version
        # TODO what about overwriting groups with same names?
        bpy.ops.group.create(name=interfacegroupname)
        interfacegroup = bpy.data.groups[interfacegroupname]
        interfacegroup['submodeltype'] = 'interfaces'

        # copy interface definitions from submodel definitions
        for key in modeldefs['interfaces']:
            interfacegroup[key] = modeldefs['interfaces'][key]

        # move objects to interface layer
        for obj in interfaces:
            bUtils.sortObjectToCollection(obj, cname='interface')
        log('Created interface group for submodel ' + submodelname + '.', 'DEBUG')
    else:
        log('No interfaces for this submodel.', 'DEBUG')

    for i in interfaces:
        i.show_name = True
    return (submodelgroup, interfacegroup)


def removeSubmodel(submodelname, submodeltype, version='', interfaces=True):
    """Removes a submodel definition from the Blender project.
    Returns True or False depending on whether groups have been removed or not.

    Args:
      submodelname: the name of the submodel
      submodeltype: the submodeltype of the submodel
      version: optional version of the submodel (Default value = '')
      interfaces: True if interface should also be deleted, else False. (Default value = True)

    Returns:
      : True if groups have been removed, else False.

    """
    # build the group name to look for
    submodelgroupname = submodeltype + ':' + submodelname
    if version != '':
        submodelgroupname += '/' + version

    # remove the submodelgroup
    if submodelgroupname in bpy.data.groups:
        bpy.data.groups.remove(bpy.data.groups[submodelgroupname])
        if not interfaces:
            return True

    if interfaces:
        interfacegroupname = 'interfaces:' + submodelname
        if version != '':
            interfacegroupname += '/' + version

        if interfacegroupname in bpy.data.groups:
            bpy.data.groups.remove(bpy.data.groups[interfacegroupname])
            return True
    return False


def toggleInterfaces(interfaces=None, modename='toggle'):
    """

    Args:
      interfaces: (Default value = None)
      modename: (Default value = 'toggle')

    Returns:

    """
    modedict = {'toggle': 0, 'activate': 1, 'deactivate': 2}
    mode = modedict[modename]
    if not interfaces:
        interfaces = [i for i in bpy.context.selected_objects if i.phobostype == 'interface']
    for i in interfaces:
        if mode == 0:
            i.show_name = not i.show_name
        elif mode == 1:
            i.show_name = True
        elif mode == 2:
            i.show_name = False


def connectInterfaces(parentinterface, childinterface, transform=None):
    """

    Args:
      parentinterface:
      childinterface:
      transform: (Default value = None)

    Returns:

    """
    # first check if the interface is child of the root object and if not, restructure the tree
    root = sUtils.getRoot(childinterface)
    parent = childinterface.parent
    if root != parent:
        restructureKinematicTree(parent)
    childsubmodel = childinterface.parent

    # connect the interfaces
    sUtils.selectObjects(objects=[parentinterface], clear=True, active=0)
    bpy.ops.object.make_single_user(object=True, obdata=True)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    sUtils.selectObjects(objects=[childinterface], clear=True, active=0)
    bpy.ops.object.make_single_user(object=True, obdata=True)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    # parent interfaces
    bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
    parentObjectsTo(childsubmodel, childinterface, clear=True)
    parentObjectsTo(childinterface, parentinterface)

    loc, rot, sca = parentinterface.matrix_world.decompose()
    # apply additional transform (ignoring the scale of the parent interface)
    if not transform:
        transform = (
            mathutils.Euler((math.radians(180.0), 0.0, math.radians(180.0)), 'XYZ')
            .to_matrix()
            .to_4x4()
        )

    childinterface.matrix_world = (
        mathutils.Matrix.Translation(loc) @ rot.to_matrix().to_4x4() @ transform
    )

    # TODO clean this up
    # try:
    #    del childsubmodel['modelname']
    # except KeyError:
    #    pass
    # TODO: re-implement this for MECHANICS models
    # try:
    #     # parent visual and collision objects to new parent
    #     children = sUtils.getImmediateChildren(parent, ['visual', 'collision', 'interface'])
    #     print(children)
    #     sUtils.selectObjects(children, True, 0)
    #     bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
    #     print()
    #     parentObjectsTo(children, sUtils.getEffectiveParent(parent, ignore_selection=True))
    #     bpy.ops.object.parent_set(type='BONE_RELATIVE')
    # except (IndexError, AttributeError):
    #     pass  # no objects to re-parent
    parentinterface.show_name = False
    childinterface.show_name = False


def disconnectInterfaces(parentinterface, childinterface, transform=None):
    """

    Args:
      parentinterface:
      childinterface:
      transform: (Default value = None)

    Returns:

    """
    # unparent the child
    sUtils.selectObjects(objects=[childinterface], clear=True, active=0)
    bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')

    # select the former parent of the interface as new root
    if childinterface.children and len(childinterface.children) > 0:
        # prefer submodel instances
        for child in childinterface.children:
            if child.phobostype == 'submodel':
                root = child
                break
        # otherwise just use the first child
        else:
            root = childinterface.children[0]

    # restructure the kinematic tree to make the interface child of the submodel again
    sUtils.selectObjects(objects=[root], clear=True, active=0)
    bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
    parentObjectsTo(childinterface, root)

    # apply additional transform
    if transform:
        childinterface.matrix_world = root.matrix_world @ transform

    # make the interfaces active again
    parentinterface.show_name = True
    childinterface.show_name = True


def setProperties(obj, diction, category=None):
    """Adds the specified dictionary as custom properties to the object.
    
    If a category is provided, the keys of the dictionary are prepended with the category:
        `category/key`

    Args:
      obj(bpy.types.Object): object to add the information to
      diction(dict): information to add to the object
      category(str, optional): category for the dictionary entries (Default value = None)

    Returns:

    """
    for key, value in diction.items():
        obj[(category + '/' + key) if category else key] = value


def getProperties(obj, category=None):
    """Returns a dictionary of custom property information of the object.
    
    If a category is provided, only the custom properties of the specified category are returned.
    Otherwise, the phobostype of the object will be used as category.
    
    The dictionary contains the custom property keys with the category removed (e.g. 'name' for
    'link/name').

    Args:
      obj(bpy.types.Object): object to get properties of
      category(str, optional): property category to look for (Default value = None)

    Returns:
      : dict -- custom property information of the phobostype/category for the object

    """
    if not category:
        category = obj.phobostype
    try:
        diction = {
            key.replace(category + '/', ''): value
            for key, value in obj.items()
            if key.startswith(category + '/')
        }
    except KeyError:
        log("Failed filtering properties for category " + category, "ERROR")
    return diction


def removeProperties(obj, props, recursive=False):
    """Removes a list of custom properties from the specified object.
    
    The specified property list can contain names with wildcards at the end (e.g. sensor*).
    
    If recursive is set, the properties will be removed recursively from all children, too.

    Args:
      obj(bpy.types.Object): object to remove the properties from
      props(list(str): list of property names, which will be removed from the object
      recursive(bool, optional): if True, the properties will be removed recursively from the children, too (Default value = False)

    Returns:

    """
    for prop in props:
        if len(prop) == 0:
            continue
        if prop in obj:
            del obj[prop]
        elif prop[-1] == '*':
            for objprop in list(obj.keys()):
                if objprop.startswith(prop[:-1]):
                    del obj[objprop]

    if recursive:
        for child in obj.children:
            removeProperties(child, props, recursive=recursive)


def mergeLinks(links, targetlink, movetotarget=False):
    """

    Args:
      links: 
      targetlink: 
      movetotarget: (Default value = False)

    Returns:

    """
    for link in links:
        if movetotarget:
            link.matrix_world = targetlink.matrix_world
        sUtils.selectObjects([link], clear=True, active=0)
        bpy.ops.object.select_grouped(type='CHILDREN')
        bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
        try:
            parentObjectsTo(bpy.context.selected_objects, targetlink)
        except RuntimeError as e:
            log("Cannot resolve new parent hierarchy: " + str(e), 'ERROR')
        del link




# def addAnnotationObject(obj, annotation, name=None, size=0.1, namespace=None, parent=None):
#     """Add a new annotation object with the specified annotations to the object.
#
#     The annotation object will receive 'annotation_object' as its default name, unless a name is
#     provided. Naming is done using :func:`phobos.utils.naming.safelyName`.
#
#     The annotation object will be scaled according to the **size** parameter.
#
#     If ``namespace`` is provided, the annotations will be saved with this string prepended.
#     This is done using :func:`addAnnotation`.
#
#     Args:
#       obj(bpy.types.Object): object to add annotation object to
#       annotation(dict): annotations that will be added
#       name(str, optional): name for the new annotation object (Default value = None)
#       size(int/float, optional): size of the new annotation object (Default value = 0.1)
#       namespace(str, optional): namespace that will be prepended to the annotations (Default value = None)
#
#     Returns:
#       : bpy.types.Object - the new annotation object
#
#     """
#     loc = obj.matrix_world.to_translation()
#     if not name:
#         name = obj.name + '_annotation_object'
#
#     annot_obj = bUtils.createPrimitive(
#         name,
#         'box',
#         [1, 1, 1],
#         defs.layerTypes['annotation'],
#         plocation=loc,
#         phobostype='annotation',
#     )
#     annot_obj.scale = (size,) * 3
#
#     resource = ioUtils.getResource(['annotation', namespace.split('/')[-1]])
#     if resource:
#         annot_obj.data = resource.data
#     else:
#         annot_obj.data = ioUtils.getResource(['annotation', 'default']).data
#
#     # make sure all layers are enabled for parenting
#     originallayers = {}
#     for name, coll in bpy.context.window.view_layer.layer_collection.children.items():
#         originallayers[name] = coll.exclude
#         coll.exclude = False
#
#     # parent annotation object
#     parentObjectsTo(annot_obj, obj)
#
#     # Restore original layers
#     for key, value in originallayers.items():
#         bpy.context.window.view_layer.layer_collection.children[key].exclude = value
#
#     addAnnotation(annot_obj, annotation, namespace=namespace)
#     return annot_obj


def addAnnotation(obj, annotation, namespace=None, ignore=[]):
    """Adds the specified annotations to the object.
    
    If provided, the namespace will be prepended to the annotation keys and separated with a /.

    Args:
      obj(bpy.types.Object): object to add the annotations to
      annotation(dict): annotations to add to the object
      namespace(str, optional): namespace which will be prepended to the annotations (Default value = None)
      ignore(list(str, optional): skip these keys when adding annotations (Default value = [])

    Returns:

    """
    def flatten_dict(input_dict):
        flat = {}
        for k, v in input_dict.items():
            if type(v) == dict:
                flat_v = flatten_dict(v)
                for k2, v2 in flat_v:
                    flat[k+"/"+k2] = v2
            else:
                flat[k] = v
        return flat

    flat_annotations = flatten_dict(annotation)

    for key, value in flat_annotations.items():
        obj[key] = value


def sortObjectsToLayers(objs):
    """Sorts the specified objects to the layers which match their phobostype.
    
    The layer for each phobostype is defined according to `phobos.defs.layerTypes`.

    Args:
      objs(list(bpy.types.Object): objects to move to their respective layer

    Returns:

    """
    for obj in objs:
        if obj.phobostype != 'undefined':
            # first check if we have a collcection for the type
            bUtils.sortObjectToCollection(obj, cname=obj.phobostype)
        else:
            log("The phobostype of object {} is undefined.".format(obj.name), 'ERROR')


def smoothen_surface(obj):
    """Applies various steps to make the specified object look clean and smooth.

    Args:
      obj(bpy.types.Object): object to make look clean

    Returns:

    """
    bpy.context.view_layer.objects.active = obj

    # recalculate surface normals
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent()
    bpy.ops.mesh.mark_sharp(clear=True)
    bpy.ops.object.mode_set(mode='OBJECT')

    # add smooth shading
    bpy.ops.object.shade_smooth()

    # use edge split modifier to improve the look of CAD-models
    for mod in obj.modifiers:
        if mod.type == 'EDGE_SPLIT':
            log("Edge split modifier already added to object {}.".format(obj.name), 'DEBUG')
            break
    else:
        bpy.ops.object.modifier_add(type='EDGE_SPLIT')
