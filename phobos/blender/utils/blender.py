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
Contains all utility functions that are connected to Blender functionality first.
"""

import os

import bpy
import mathutils

from . import naming as nUtils
from . import selection as sUtils
from .. import reserved_keys
from ..model import materials
from ..phoboslog import log


def update():
    """Forces Blender to update scene contents (i.e. transformations).
    
    Sometimes when e.g. manipulating internal matrices of Blender objects such as
    matrix_world or matrix_local, Blender will not recalculate all related transforms,
    especially not the visual transform. The bpy.context.scene.update() function often
    named as the solution to this will lead to the matrices being updated, but not the
    visual transforms. This Function runs code (may be updated with new Blender verions)
    that forces Blender to update the visual transforms.

    Args:

    Returns:

    """
    bpy.context.scene.frame_set(1)
    bpy.context.scene.frame_set(0)


def compileEnumPropertyList(iterable):
    """

    Args:
      iterable: 

    Returns:

    """
    return ((a,) * 3 for a in iterable)


def getBlenderVersion():
    """TODO Missing documentation"""
    # DOCU add some docstring
    return bpy.app.version[0] * 100 + bpy.app.version[1]


def getPhobosPreferences():
    """TODO Missing documentation"""
    return bpy.context.preferences.addons["phobos"].preferences


def printMatrices(obj, info=None):
    """This function prints the matrices of an object to the screen.

    Args:
      obj(bpy.types.Object): The object to print the matrices from.
      info(bool, optional): If True the objects name will be included into the printed info. (Default value = None)

    Returns:

    """
    if not info:
        info = obj.name
    print(
        "\n----------------",
        info,
        "---------------------\n",
        "local:\n",
        obj.matrix_local,
        "\n\nworld:\n",
        obj.matrix_world,
        "\n\nparent_inverse:\n",
        obj.matrix_parent_inverse,
        "\n\nbasis:\n",
        obj.matrix_basis,
    )


def createPrimitive(
    pname,
    ptype,
    psize,
    player=0,
    pmaterial=None,
    plocation=(0, 0, 0),
    protation=(0, 0, 0),
    phobostype=None,
):
    """Generates the primitive specified by the input parameters

    Args:
      pname(str): The primitives new name.
      ptype(str): The new primitives type. Can be one of *box, sphere, cylinder, cone, disc*
      psize(float or list): The new primitives size. Depending on the ptype it can be either a single float or a tuple.
      player: The layer bitmask for the new blender object. (Default value = 0)
      pmaterial: The new primitives material. (Default value = None)
      plocation(tuple, optional): The new primitives location. (Default value = (0)
      protation(tuple, optional): The new primitives rotation. (Default value = (0)
      phobostype(str, optional): phobostype of object to be created (Default value = None)
      0: 
      0): 

    Returns:
      : bpy.types.Object - the new blender object.

    """
    # TODO: allow placing on currently active layer?
    #try:
    #    n_layer = int(player)
    #except ValueError:
    #    n_layer = defs.layerTypes[player]
    #players = defLayers([n_layer])
    # the layer has to be active to prevent problems with object placement
    #bpy.context.scene.layers[n_layer] = True
    if ptype == "box":
        bpy.ops.mesh.primitive_cube_add(location=plocation, rotation=protation)
        obj = bpy.context.object
        obj.dimensions = psize
    elif ptype == "sphere":
        bpy.ops.mesh.primitive_uv_sphere_add(
            radius=psize, location=plocation, rotation=protation
        )
    elif ptype == "cylinder":
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=32,
            radius=psize[0],
            depth=psize[1],
            location=plocation,
            rotation=protation,
            end_fill_type='TRIFAN',
        )
    elif ptype == "cone":
        bpy.ops.mesh.primitive_cone_add(
            vertices=32,
            radius=psize[0],
            depth=psize[1],
            cap_end=True,
            location=plocation,
            rotation=protation,
            end_fill_type='TRIFAN',
        )
    elif ptype == 'disc':
        bpy.ops.mesh.primitive_circle_add(
            vertices=psize[1],
            radius=psize[0],
            fill_type='TRIFAN',
            location=plocation,
            rotation=protation,
        )
    elif ptype == 'ico':
        bpy.ops.mesh.primitive_ico_sphere_add(
            radius=psize, location=plocation, rotation=protation
        )
    else:
        log("Primitive type not found: " + ptype + ". Adding default cube instead.", 'WARNING')
        bpy.ops.mesh.primitive_cube_add(location=plocation, rotation=protation)
        obj = bpy.context.object
        obj.dimensions = psize

    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj = bpy.context.object
    if phobostype:
        obj.phobostype = phobostype
        sortObjectToCollection(obj, cname=phobostype)
    nUtils.safelyName(obj, pname, phobostype)
    if pmaterial:
        materials.assignMaterial(obj, pmaterial)
    return obj


def setObjectLayersActive(obj, extendlayers=False):
    """Sets all layers, the specified object is on to active.
    
    If extendlayers is set, the layers of the object are added to the already active layers instead
    of replacing them.

    Args:
      obj(bpy.types.Object): object of which the layers shall be activated
      extendlayers(bool, optional): activate the object layers in addition to the already active layers (Default value = False)

    Returns:

    """
    for cname, coll in bpy.context.scene.collection.children.items():
        if obj.name in coll.objects:
            bpy.context.window.view_layer.layer_collection.children[cname].exclude = False
        elif not extendlayers:
            bpy.context.window.view_layer.layer_collection.children[cname].exclude = True


def toggleLayer(cname, value=None):
    """This function toggles a specific collection or sets it to a desired value.

    Args:
      cname(string): The collection name you want to change.
      value(bool, optional): True if visible, None for toggle. (Default value = None)

    Returns:

    """
    craeteCollectionIfNotExists(cname)
    coll = bpy.context.window.view_layer.layer_collection.children[cname]
    if value:
        coll.exclude = not value
    else:
        coll.exclude = not coll.exclude


def updateTextFile(textfilename, newContent):
    """This function updates a blender textfile or creates a new one if it is not existent.

    Args:
      textfilename(str): The blender textfiles file name.
      newContent(str): The textfiles new content.

    Returns:

    """
    try:
        bpy.data.texts.remove(bpy.data.texts[textfilename])
    except KeyError:
        # TODO handle this error somehow
        # Not important. Just create.
        pass
    createNewTextfile(textfilename, newContent)


def readTextFile(textfilename):
    """Returns the content of a specified text file in Blender's data.
    
    If the file is not found, an empty string is returned.

    Args:
      textfilename(str): The blender textfiles name.

    Returns:
      : str -- textfile content or empty string

    """
    if textfilename not in bpy.data.texts:
        log("No text file " + textfilename + " found.", "WARNING")
        return ""
    return "\n".join([l.body for l in bpy.data.texts[textfilename].lines])


def createNewTextfile(textfilename, contents):
    """This function creates a new blender text file with the given content.

    Args:
      textfilename(str): The new blender texts name.
      contents(str): The new textfiles content.

    Returns:

    """
    for text in bpy.data.texts:
        text.tag = True
    bpy.ops.text.new()
    newtext = None
    for text in bpy.data.texts:
        if not text.tag:
            newtext = text
    for text in bpy.data.texts:
        text.tag = False
    newtext.name = textfilename
    bpy.data.texts[textfilename].write(contents)


def openScriptInEditor(scriptname):
    """This function opens a script/textfile in an open blender text window. Nothing happens if there is no
    available text window.

    Args:
      scriptname(str): The scripts name.

    Returns:

    """
    if scriptname in bpy.data.texts:
        for area in bpy.context.screen.areas:
            if area.type == 'TEXT_EDITOR':
                area.spaces.active.text = bpy.data.texts[scriptname]
    else:
        log("There is no script named " + scriptname + "!", "ERROR")


def cleanObjectProperties(props, phobostype=None):
    """Cleans a predefined list of Blender-specific or other properties from the dictionary.

    Args:
      props: 

    Returns:

    """
    getridof = [
        'phobostype',
        '_RNA_UI',
        'cycles_visibility',
        'startChain',
        'endChain',
        'masschanged',
    ]
    if phobostype is not None and hasattr(reserved_keys, phobostype.upper()+"_KEYS"):
        getridof += getattr(reserved_keys, phobostype.upper()+"_KEYS")
    if props:
        for key in getridof:
            if key in props:
                del props[key]
    return props


def cleanScene():
    """Clean up blend file (remove all objects, meshes, materials and lights)"""
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


def createPreview(objects, export_path, modelname, render_resolution=256, opengl=False):
    """Creates a thumbnail of the given objects.

    Args:
      objects(list of bpy.types.Object): list of objects for the thumbnail
      export_path(str): folder to export image to
      modelname(str): name of model (used as file name)
      render_resolution(int, optional): side length of resulting image in pixels (Default value = 256)
      opengl(bool, optional): whether to use opengl rendering or not (Default value = False)

    Returns:

    """
    log("Creating thumbnail of model: " + modelname, "INFO")

    # render presets
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    bpy.context.scene.render.resolution_x = render_resolution
    bpy.context.scene.render.resolution_y = render_resolution
    bpy.context.scene.render.resolution_percentage = 100

    # hide everything that is not supposed to show
    for ob in bpy.data.objects:
        if not (ob in objects):
            ob.hide_render = True
            ob.hide_viewport = True

    # render the preview
    if opengl:  # use the viewport representation to create preview
        bpy.ops.view3d.view_selected()
        bpy.ops.render.opengl(view_context=True)
    else:  # use real rendering
        # create camera
        bpy.ops.object.camera_add(align='VIEW')
        cam = bpy.context.active_object
        bpy.data.cameras[cam.data.name].type = 'ORTHO'
        bpy.data.scenes[0].camera = cam  # set camera as active camera

        sUtils.selectObjects(objects, True, 0)
        bpy.ops.view3d.camera_to_view_selected()
        # create light
        bpy.ops.object.light_add(type='SUN', radius=1)
        light = bpy.context.active_object
        light.matrix_world = cam.matrix_world
        # set background
        oldcolor = bpy.data.worlds['World'].node_tree.nodes['Background'].inputs[0].default_value[:3]
        bpy.data.worlds['World'].node_tree.nodes['Background'].inputs[0].default_value[:3] = (1,1,1)
        bpy.ops.render.render()
        bpy.data.worlds['World'].node_tree.nodes['Background'].inputs[0].default_value[:3] = oldcolor
        sUtils.selectObjects([cam, light], True, 0)
        bpy.ops.object.delete()

    # safe render and reset the scene
    log("Saving model preview to: " + os.path.join(export_path, modelname + '.png'), "INFO")
    bpy.data.images['Render Result'].save_render(os.path.join(export_path, modelname + '.png'))

    # make all objects visible again
    for ob in bpy.data.objects:
        ob.hide_render = False
        ob.hide_viewport = False


def toggleTransformLock(obj, setting=None):
    """Toogle transform lock for the referred object

    Args:
      obj: 
      setting: (Default value = None)

    Returns:

    """
    obj.lock_location[0] = setting if setting is not None else not obj.lock_location[0]
    obj.lock_location[1] = setting if setting is not None else not obj.lock_location[1]
    obj.lock_location[2] = setting if setting is not None else not obj.lock_location[2]
    obj.lock_rotation[0] = setting if setting is not None else not obj.lock_rotation[0]
    obj.lock_rotation[1] = setting if setting is not None else not obj.lock_rotation[1]
    obj.lock_rotation[2] = setting if setting is not None else not obj.lock_rotation[2]
    obj.lock_scale[0] = setting if setting is not None else not obj.lock_scale[0]
    obj.lock_scale[1] = setting if setting is not None else not obj.lock_scale[1]
    obj.lock_scale[2] = setting if setting is not None else not obj.lock_scale[2]


def switchToScene(scenename):
    """Switch to Blender scene of the given name

    Args:
      scenename: 

    Returns:

    """
    if scenename not in bpy.data.scenes.keys():
        bpy.data.scenes.new(scenename)
    bpy.context.window.scene = bpy.data.scenes[scenename]
    return bpy.data.scenes[scenename]


def getCombinedDimensions(objects):
    """Returns the dimension of the space the objects passed occupy.

    Args:
      objects(list): list of

    Returns:
      : list of floats (x, y, z) of dimensions

    Raises:
      ValueError: If empty object list is passed.

    """
    bbpoints = []
    for o in objects:
        for p in o.bound_box:
            bbpoints.append(o.matrix_world @ mathutils.Vector(p))
    mindims = [min([bbpoint[i] for bbpoint in bbpoints]) for i in (0, 1, 2)]
    maxdims = [max([bbpoint[i] for bbpoint in bbpoints]) for i in (0, 1, 2)]
    return [abs(maxdims[i] - mindims[i]) for i in (0, 1, 2)]


def sortObjectToCollection(obj, cname="Collection"):
    set_active = False
    if bpy.context.active_object == obj:
        set_active = True
    craeteCollectionIfNotExists(cname)
    for name, collection in bpy.context.scene.collection.children.items():
        if name == cname:
            if not obj.name in collection.objects:
                collection.objects.link(obj)
        elif obj.name in collection.objects:
            collection.objects.unlink(obj)
    # unlink from general scene collection
    if obj.name in bpy.context.scene.collection.objects:
        bpy.context.scene.collection.objects.unlink(obj)
    if set_active:
        bpy.context.view_layer.objects.active = obj


def craeteCollectionIfNotExists(cname="Collection"):
    if not cname in bpy.context.scene.collection.children.keys():
        newcollection = bpy.data.collections.new(cname)
        bpy.context.scene.collection.children.link(newcollection)


def activateObjectCollection(obj):
    for cname,coll in bpy.context.scene.collection.children.items():
        if obj.name in coll.objects:
            toggleLayer(cname, True)
