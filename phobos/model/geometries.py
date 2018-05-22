
# TODO add shebang and document introduction
import os
import bpy
import phobos.defs as defs
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.io.meshes.meshes as meshes
from phobos.model.materials import assignMaterial
from phobos.phoboslog import log


def getLargestDimension(geometry):
    # DOCU add some docstring
    if geometry['type'] == 'box':
        return max(geometry['size'])
    if geometry['type'] == 'cylinder' or geometry['type'] == 'capsule':
        return max((geometry['radius'], geometry['length']))
    if geometry['type'] == 'sphere':
        return geometry['radius']
    if geometry['type'] == 'mesh':
        # scale would make no sense here for an absolute measure
        return max(geometry['size']) if 'size' in geometry else 0.2


def deriveGeometry(obj):
    """This function derives the geometry from an object.

    :param obj: The blender object to derive the geometry from.
    :type obj: bpy_types.Object
    :return: dict
    """
    try:
        geometry = {'type': obj['geometry/type']}
        gt = obj['geometry/type']
        if gt == 'box':
            geometry['size'] = list(obj.dimensions)
        elif gt == 'cylinder' or gt == 'capsule':
            geometry['radius'] = obj.dimensions[0]/2
            geometry['length'] = obj.dimensions[2]
        elif gt == 'capsule':
            geometry['radius'] = obj.dimensions[0]/2
            geometry['length'] = obj.dimensions[2] - obj.dimensions[0]
        elif gt == 'sphere':
            geometry['radius'] = obj.dimensions[0]/2
        elif gt == 'mesh':
            geometry['filename'] = obj.data.name
            geometry['scale'] = list(obj.scale)
            # FIXME: is this needed to calculate an approximate inertia
            geometry['size'] = list(obj.dimensions)
        # any other geometry type, i.e. 'plane'
        else:
            geometry['size'] = list(obj.dimensions)
        return geometry
    except KeyError as err:
        log("Undefined geometry for object " + nUtils.getObjectName(obj) +
            " " + str(err), "ERROR")
        return None


def createGeometry(viscol, geomsrc):
    """Creates Blender object for visual or collision objects.
    Returns reference to new object or None if creation failed.

    :param viscol: visual/collision dictionary element
    :type viscol: dict
    :param geomsrc: new object's phobostype
    :type geomsrc: str
    :return: bpy.types.Object or None
    """
    if 'geometry' not in viscol or viscol['geometry'] is {}:
        return None
    bpy.ops.object.select_all(action='DESELECT')
    geom = viscol['geometry']
    geomtype = geom['type']
    # create the Blender object
    if geomtype == 'mesh':
        bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes[geomsrc])
        meshname = "".join(os.path.basename(geom["filename"]).split(".")[:-1])
        if not os.path.isfile(geom['filename']):
            log(geom['filename'] + " is no file. Object " + viscol['name'] +
                " will have empty mesh!", "ERROR")
            bpy.data.meshes.new(meshname)
        if meshname in bpy.data.meshes:
            log('Assigning copy of existing mesh ' + meshname + ' to ' + viscol['name'], 'INFO')
            bpy.ops.object.add(type='MESH')
            newgeom = bpy.context.object
            newgeom.data = bpy.data.meshes[meshname]
        else:
            log('Importing mesh for link element ' + viscol['name'], 'INFO')
            filetype = geom['filename'].split('.')[-1].lower()
            newgeom = meshes.importMesh(geom['filename'], filetype)
            newgeom.data.name = meshname
            if not newgeom:
                log('Failed to import mesh file ' + geom['filename'], 'ERROR')
                return
            # scale imported object
            if 'scale' in geom:
                sUtils.selectObjects((newgeom,), clear=True)
                newgeom.scale = geom['scale']
    else:
        if geomtype == 'box':
            dimensions = geom['size']
        elif geomtype == 'cylinder':
            dimensions = (geom['radius'], geom['length'])
        elif geomtype == 'sphere':
            dimensions = geom['radius']
        else:
            log("Unknown geometry type of " + geomsrc + viscol['name']
                + '. Placing empty coordinate system.', "ERROR")
            bpy.ops.object.empty_add(type='PLAIN_AXES', radius=0.2)
            obj = bpy.context.object
            obj.phobostype = geomsrc
            nUtils.safelyName(bpy.context.active_object, viscol['name'], phobostype=geomsrc)
            return None
        log('Creating primtive for {0}: {1}'.format(geomsrc, viscol['name']), 'INFO')
        newgeom = bUtils.createPrimitive(viscol['name'], geomtype, dimensions, phobostype=geomsrc)
        newgeom.select = True
        bpy.ops.object.transform_apply(scale=True)

    # from here it's the same for both meshes and primitives
    newgeom['geometry/type'] = geomtype
    if geomsrc == 'visual':
        try:
            if 'name' in viscol['material']:
                assignMaterial(newgeom, viscol['material']['name'])
            else:
                assignMaterial(newgeom, viscol['material'])
        except KeyError:
            log('No material for obj ' + viscol['name'], 'DEBUG')
    # FIXME: place empty coordinate system and return...what? Error handling of file import!
    for prop in viscol:
        if prop.startswith('$'):
            for tag in viscol[prop]:
                newgeom[prop[1:]+'/'+tag] = viscol[prop][tag]
    newgeom.name = viscol['name']
    newgeom[geomsrc+"/name"] = viscol['name']
    return newgeom

    # TODO still needed? If yes, make it an dev branch or issue
    ##code for capsules:
    #
    # #buildModelDictionary:
    #     capsules_list = []
    #     # in for-loop:
    #     if all([key in props for key in ['cylinder', 'sphere1', 'sphere2']]):     # this is the case with simulated capsules
    #         capsules_list.append({'link': parent.name,
    #                               'name': props['cylinder']['name'][:-len('_cylinder')],
    #                               'radius': props['cylinder']['geometry']['radius'],
    #                               'length': props['cylinder']['geometry']['length'] + 2*props['cylinder']['geometry']['radius'],
    #                               #'bitmask': props['cylinder']['bitmask']
    #                             })
    #         for key in props:
    #             robot['links'][nUtils.getObjectName(parent, phobostype="link")][obj.phobostype][key] = props[key]
    #     robot['capsules'] = capsules_list
    #
    #
    # # deriveDictEntry
    # if obj['geometry/type'] == 'capsule':
    #                 props, parent = deriveCapsule(obj)
    #             else:
# exportModelToSmurf
#     print('capsules:', model['capsules'])
#     capsules = model['capsules']
#     for capsule in capsules:
#         if capsule['name'] in bitmasks:
#             bitmask = bitmasks[capsule['name']]['bitmask']
#             capsule['bitmask'] = bitmask
#
#                     if model['capsules']:
#             op.write(yaml.dump({'capsules': model['capsules']}, default_flow_style=False))
