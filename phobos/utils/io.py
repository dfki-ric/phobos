#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import shutil
import sys
import os
import bpy

from phobos import defs
from phobos import display
from phobos.phoboslog import log

from phobos.io.entities import entity_types
from phobos.io.meshes import mesh_types
from phobos.io.scenes import scene_types

from phobos.utils import selection as sUtils
from phobos.utils import naming as nUtils
from phobos.utils import blender as bUtils


indent = '  '
xmlHeader = '<?xml version="1.0"?>\n<!-- created with Phobos ' + defs.version + ' -->\n'


def xmlline(ind, tag, names, values):
    """Generates an xml line with specified values.
    To use this function you need to know the indentation level you need for this line.
    Make sure the names and values list have the correct order.

    Args:
      ind(int >= 0): Indentation level
      tag(String): xml element tag
      names(list (same order as for values): Names of xml element's attributes
      values(list (same order as for names): Values of xml element's attributes

    Returns:
      : String -- Generated xml line.

    """
    line = [indent * max(0, ind) + '<' + tag]
    for i in range(len(names)):
        line.append(' ' + names[i] + '="' + str(values[i]) + '"')
    line.append('/>\n')
    return ''.join(line)


def l2str(items, start=0, end=None):
    """Generates string from (part of) a list.

    Args:
      items(list): List from which the string is derived (elements need to implement str())
      start(int, optional): Inclusive start index for iteration (Default value = 0)
      end(int, optional): Exclusive end index for iteration (Default value = None)

    Returns:
      : str - Generated string.

    """
    start = max(start, 0)
    end = end if end else len(items)
    return ' '.join([str(i) for i in items[start:end]])


def securepath(path):
    """Checks whether directories of a path exist and generates them if necessary.

    Args:
      path(str): The path to be secured (directories only)

    Returns:
      : String -- secured path as absolute path, None on error

    """
    path = os.path.abspath(path)
    if not os.path.exists(path):
        try:
            os.makedirs(path)
        except NotADirectoryError:
            log(path + " is not a valid directory", "ERROR")
            return None
    return path


def getExpSettings():
    """Returns Phobos' export settings as displayed in the GUI"""
    return bpy.context.scene.phobosexportsettings


def getExportModels():
    """Returns a list of objects representing a model (root) in the Blender scene"""
    if getExpSettings().selectedOnly:
        roots = [root for root in sUtils.getRoots() if root.select_get()]
    else:
        roots = sUtils.getRoots()
    return list(roots)


def getEntityRoots():
    """Returns all objects for which entity properties are specified.

    Args:

    Returns:
      list: roots of well-defined entities in the scene

    """
    roots = [
        obj
        for obj in bpy.context.scene.objects
        if sUtils.isEntity(obj) and (not getExpSettings().selectedOnly or obj.select_get())
    ]
    return roots


def getModelListForEnumProp(self, context):
    """Returns list of all exportable models in the scene formatted for an EnumProperty

    Args:
      context: 

    Returns:

    """
    rootnames = set([nUtils.getModelName(r) for r in getExportModels()])
    return sorted([(r,) * 3 for r in rootnames])


def getDictFromYamlDefs(phobostype, defname, name):
    """Returns a phobos representation of the object specified by the definition parameters.

    Args:
      phobostype(str): phobostype of the definition
      defname(str): name of the individual definition
      name(str): name for the new object

    Returns:
      : dict -- phobos representation from the definition

    """
    if 'material' in defs.def_settings[phobostype + 's'][defname]:
        material = defs.def_settings[phobostype + 's'][defname]['material']
    else:
        material = None

    # separate properties and annotations from each other
    props = {
        key: value
        for key, value in defs.definitions[phobostype + 's'][defname].items()
        if not isinstance(value, dict)
    }
    annots = {
        key: value
        for key, value in defs.definitions[phobostype + 's'][defname].items()
        if isinstance(value, dict)
    }

    # create a dictionary holding the properties
    phobos_dict = {
        'name': name,
        'defname': defname,
        'category': defs.def_settings[phobostype + 's'][defname]['categories'],
        'material': material,
        'type': defs.def_settings[phobostype + 's'][defname]['type'],
        'props': props,
        'annotations': annots,
    }

    # add the general settings for this object
    general_settings = defs.def_settings[phobostype + 's'][defname]
    for gensetting in general_settings.keys():
        phobos_dict[gensetting] = general_settings[gensetting]

    return phobos_dict


def getOutputMeshtype():
    """Returns the mesh type to be used in exported files as specified in the GUI"""
    return str(getExpSettings().outputMeshtype)

def getOutputPathtype():
    """Returns the path type to be used in exported files as specified in the GUI"""
    return str(getExpSettings().outputPathtype)

def getRosPackageName():
    """Returns the ros package name used for ros path definitions"""
    return str(getExpSettings().rosPackageName)

def getOutputMeshpath(path, meshtype=None, pathType="relative"):
    """Returns the folder path for mesh file export as specified in the GUI.
    
    Phobos by default creates a directory 'meshes' in the export path and subsequently creates
    sub-directories of "export/path/meshes" for every format, e.g. resulting in "export/path/mesh/obj"
    for .obj file export.

    Args:
      path(str): export path root (set in the GUI)
      meshtype(str, optional): a valid mesh type, otherwise the type set in the GUI is used (Default value = None)

    Returns:
      string: output path for meshes

    """
    # pathType relative is default
    rpath = os.path.join(path, 'meshes', meshtype if meshtype else getOutputMeshtype()) + "/"

    if not pathType:
        pathType = getOutputPathtype()
    if pathType == "ros_package":
        rpath = "package://"+getRosPackageName()+"/"+os.path.join('meshes', meshtype if meshtype else getOutputMeshtype()) + "/"

    return rpath


def getEntityTypesForExport():
    """Returns list of entity types available for export"""
    return [
        typename
        for typename in sorted(entity_types)
        if 'export' in entity_types[typename] and 'extensions' in entity_types[typename]
    ]


def getEntityTypesForImport():
    """Returns list of entity types available for import"""
    return [
        typename
        for typename in sorted(entity_types)
        if 'import' in entity_types[typename] and 'extensions' in entity_types[typename]
    ]


def getSceneTypesForExport():
    """Returns list of scene types available for export"""
    return [
        typename
        for typename in sorted(scene_types)
        if 'export' in scene_types[typename] and 'extensions' in scene_types[typename]
    ]


def getSceneTypesForImport():
    """Returns list of scene types available for import"""
    return [
        typename
        for typename in sorted(scene_types)
        if 'import' in scene_types[typename] and 'extensions' in scene_types[typename]
    ]


def getMeshTypesForExport():
    """Returns list of mesh types available for export"""
    return [
        typename
        for typename in sorted(mesh_types)
        if 'export' in mesh_types[typename] and 'extensions' in mesh_types[typename]
    ]


def getMeshTypesForImport():
    """Returns list of mesh types available for import"""
    return [
        typename
        for typename in sorted(mesh_types)
        if 'import' in mesh_types[typename] and 'extensions' in mesh_types[typename]
    ]


def getExportPath():
    """Returns the root path of the export.
    
    Phobos by default creates directories in this path for every format that is used for export,
    i.e. a folder "export/path/urdf" will be created for URDF files.

    Args:

    Returns:

    """
    if os.path.isabs(getExpSettings().path):
        return getExpSettings().path
    else:
        return os.path.normpath(os.path.join(bpy.path.abspath('//'), getExpSettings().path))


def getAbsolutePath(path):
    """Returns an absolute path derived from the .blend file

    Args:
      path: 

    Returns:

    """
    if os.path.isabs(path):
        return path
    else:
        return os.path.join(bpy.path.abspath('//'), path)


def importBlenderModel(filepath, namespace='', prefix=False):
    """Imports an existing Blender model into the current .blend scene

    Args:
      filepath(str): Path of the .blend file
      namespace: (Default value = '')
      prefix: (Default value = False)

    Returns:

    """
    if os.path.exists(filepath) and os.path.isfile(filepath) and filepath.endswith('.blend'):
        log("Importing Blender model" + filepath, "INFO")
        objects = []
        with bpy.data.libraries.load(filepath) as (data_from, data_to):
            for objname in data_from.objects:
                objects.append({'name': objname})
        bpy.ops.wm.append(directory=filepath + "/Object/", files=objects)
        imported_objects = bpy.context.selected_objects
        resources = [obj for obj in imported_objects if obj.name.startswith('resource::')]
        new_objects = [obj for obj in imported_objects if not obj.name.startswith('resource::')]
        if resources:
            if 'resources' not in bpy.data.scenes.keys():
                bpy.data.scenes.new('resources')
            sUtils.selectObjects(resources)
            bpy.ops.object.make_links_scene(scene='resources')
            bpy.ops.object.delete(use_global=False)
            sUtils.selectObjects(new_objects)
        bpy.ops.view3d.view_selected(use_all_regions=False)
        # allow the use of both prefixes and namespaces, thus truly merging
        # models or keeping them separate for export
        if namespace != '':
            if prefix:
                for obj in bpy.context.selected_objects:
                    # set prefix instead of namespace
                    obj.name = namespace + '__' + obj.name
                    # make sure no internal name-properties remain
                    for key in obj.keys():
                        try:
                            if obj[key].endswidth("/name"):
                                del obj[key]
                        except AttributeError:  # prevent exceptions from non-string properties
                            pass
            else:
                for obj in bpy.context.selected_objects:
                    nUtils.addNamespace(obj, namespace)
        submechanism_roots = [
            obj
            for obj in bpy.data.objects
            if obj.phobostype == 'link' and 'submechanism/spanningtree' in obj
        ]
        for root in submechanism_roots:
            partlist = [root] + root['submechanism/spanningtree']
            if 'submechanism/freeloader' in root:
                partlist += root['submechanism/freeloader']
            sUtils.selectObjects(partlist, active=0)
            bpy.ops.group.create(name='submechanism:' + root['submechanism/name'])
        return True
    else:
        return False


def importResources(restuple, filepath=None):
    """Accepts an iterable of iterables describing resource objects to import. For instance,
    reslist=(('joint', 'continuous'), ('interface', 'default', 'bidirectional'))
    would import the resource objects named 'joint_continuous' and
    'interface_default_bidirectional'.

    Args:
      restuple: iterable of iterables containing import objects
      filepath: path to file from which to load resource (Default value = None)
    Returns(tuple of bpy.types.Object): imported objects

    Returns:

    """
    currentscene = bpy.context.scene.name
    bUtils.switchToScene('resources')
    # avoid importing the same objects multiple times
    imported_objects = [
        nUtils.stripNamespaceFromName(obj.name) for obj in bpy.data.scenes['resources'].objects
    ]
    requested_objects = ['_'.join(resource) for resource in restuple]
    new_objects = [obj for obj in requested_objects if obj not in imported_objects]

    # if no filepath is provided, use the path from the preferences
    if not filepath:
        filepath = os.path.join(bUtils.getPhobosConfigPath(), 'resources', 'resources.blend')
        print(filepath)

    # import new objects from resources.blend
    if new_objects:
        with bpy.data.libraries.load(filepath) as (data_from, data_to):
            objects = [{'name': name} for name in new_objects if name in data_from.objects]
            if objects:
                bpy.ops.wm.append(directory=filepath + "/Object/", files=objects)
            else:
                log('Resource objects could not be imported.', 'ERROR')
                bUtils.switchToScene(currentscene)
                return None
    objects = bpy.context.selected_objects
    for obj in objects:
        nUtils.addNamespace(obj, 'resource')
    bUtils.switchToScene(currentscene)
    return objects


def getResource(specifiers):
    """Returns a resource object defined by an iterable of strings.

    Args:
      specifiers(iterable): strings specifying the resource

    Returns:
      : bpy.types.Object -- resource object (or None if it could not be imported)

    """
    log("Searching for resource object " + '_'.join(specifiers) + ".", 'DEBUG')

    resobjname = nUtils.addNamespaceToName('_'.join(specifiers), 'resource')

    if 'resources' not in bpy.data.scenes or resobjname not in bpy.data.scenes['resources'].objects:
        newobjects = importResources((specifiers,))
        if not newobjects:
            return None
        return newobjects[0]
    return bpy.data.scenes['resources'].objects[resobjname]


def copy_model(model):
    """Returns a recursive deep copy of a model dictionary.
    
    The deep copy recreates dictionaries and lists, while keeping Blender objects and everything
    else untouched.
    
    This function is required, as we can not use copy.deepcopy() due to the Blender objects in our
    Phobos representation.

    Args:
      model(dict): model dictionary to copy

    Returns:
      : dict -- deep copy of the model dictionary

    """
    if isinstance(model, dict):
        newmodel = {}
        for key, value in model.items():
            if isinstance(value, dict) or isinstance(value, list):
                newmodel[key] = copy_model(value)
            else:
                newmodel[key] = value
        return newmodel
    elif isinstance(model, list):
        newlist = []
        for value in model:
            if isinstance(value, bpy.types.Object):
                newlist.append(value)
            elif isinstance(value, dict) or isinstance(value, list):
                newlist.append(copy_model(value))
            else:
                newlist.append(value)
        return newlist
    raise TypeError(
        "Deep copy failed. Unsuspected element in the dictionary: {}".format(type(model))
    )


def exportModel(model, exportpath='.', entitytypes=None):
    """Exports model to a given path in the provided formats.

    Args:
      model(dict): dictionary of model to export
      exportpath(str, optional): path to export root (Default value = '.')
      entitytypes(list of str, optional): export types - model will be exported to all (Default value = None)

    Returns:

    """
    if not exportpath:
        exportpath = getExportPath()
    if not entitytypes:
        entitytypes = getEntityTypesForExport()

    # TODO: Move texture export to individual formats? This is practically SMURF
    # TODO: Also, this does not properly take care of textures embedded in a .blend file
    # export textures
    if getExpSettings().exportTextures:
        path = os.path
        for materialname in model['materials']:
            mat = model['materials'][materialname]
            for texturetype in ['diffuseTexture', 'normalTexture', 'displacementTexture']:
                # skip materials without texture
                if texturetype not in mat:
                    continue

                sourcepath = path.join(path.expanduser(bpy.path.abspath('//')), mat[texturetype])
                if path.isfile(sourcepath):
                    texture_path = securepath(path.join(exportpath, 'textures'))
                    log(
                        "Exporting texture {} of material {} to {}.".format(
                            texturetype, mat[texturetype], texture_path
                        ),
                        'INFO',
                    )
                    try:
                        shutil.copy(
                            sourcepath, path.join(texture_path, path.basename(mat[texturetype]))
                        )
                    except shutil.SameFileError:
                        log(
                            "{} of material {} already in place.".format(texturetype, materialname),
                            'WARNING',
                        )

                    # update the texture path in the model
                    mat[texturetype] = 'textures/' + path.basename(mat[texturetype])

    # export model in selected formats
    for entitytype in entitytypes:
        typename = "export_entity_" + entitytype
        # check if format exists and should be exported
        if not getattr(bpy.context.scene, typename, False):
            continue
        # format exists and is exported:
        model_path = os.path.join(exportpath, entitytype)
        securepath(model_path)

        # export model using entity export function
        log("Export model '" + model['name'] + "' as " + entitytype + " to " + model_path, "DEBUG")

        # pass a model copy to the entity export, as these might alter the dictionary
        newmodel = copy_model(model)
        entity_types[entitytype]['export'](newmodel, model_path)

    # export meshes in selected formats
    i = 1
    mt = len([m for m in mesh_types if getattr(bpy.context.scene, "export_mesh_" + m, False)])
    mc = len(model['meshes'])
    n = mt * mc
    for meshtype in mesh_types:
        mesh_path = getOutputMeshpath(exportpath, meshtype, "relative")
        try:
            if getattr(bpy.context.scene, "export_mesh_" + meshtype, False):
                securepath(mesh_path)
                for meshname in model['meshes']:
                    mesh_types[meshtype]['export'](model['meshes'][meshname], mesh_path)
                    display.setProgress(i / n, 'Exporting ' + meshname + '.' + meshtype + '...')
                    i += 1
        except KeyError as e:
            log("Error exporting mesh {0} as {1}: {2}".format(meshname, meshtype, str(e)), "ERROR")
    display.endProgress()


def exportScene(
    scenedict, exportpath='.', scenetypes=None, export_entity_models=False, entitytypes=None
):
    """Exports provided scene to provided path

    Args:
      scenedict(dict): dictionary of scene
      exportpath(str, optional): path to scene export folder (Default value = '.')
      scenetypes(list of str, optional): export types for scene - scene will be exported to all (Default value = None)
      export_entity_models(bool, optional): whether to export entities additionally (Default value = False)
      entitytypes(list of str, optional): types to export entities in in case they are exported (Default value = None)

    Returns:

    """
    if not exportpath:
        exportpath = getExportPath()
    if not scenetypes:
        scenetypes = getSceneTypesForExport()
    if export_entity_models:
        for entity in scenedict['entities']:
            exportModel(entity, exportpath, entitytypes)
    for scenetype in scenetypes:
        gui_typename = "export_scene_" + scenetype
        # check if format exists and should be exported
        if getattr(bpy.context.scene, gui_typename):
            scene_types[scenetype]['export'](
                scenedict['entities'], os.path.join(exportpath, scenedict['name'])
            )
