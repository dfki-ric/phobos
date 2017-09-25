
# TODO add shebang and document introduction
import os
import re
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
            " " + str(err), "ERROR", "deriveGeometry")
        return None


def createGeometry(viscol, geomsrc):
    """Creates geometrical Blender object for visual or collision objects.

    :param viscol: The visual/collision dictionary element you want to create the geometry for.
    :type viscol: dict
    :param geomsrc: The new viscols phobostype.
    :type geomsrc: str

    """
    if 'geometry' not in viscol or viscol['geometry'] is {}:
        return None
    bpy.ops.object.select_all(action='DESELECT')
    geom = viscol['geometry']
    geomtype = geom['type']
    # create the Blender object
    if geomtype == 'mesh':
        # TODO delete me?
        # if hasattr(self, 'zipped') and self.zipped:
        #     if not os.path.isdir(os.path.join(self.tmp_path, tmp_dir_name)):
        #         os.mkdir(os.path.join(self.tmp_path, tmp_dir_name))
        #     archive = zipfile.ZipFile(self.filepath)
        #     archive.extract(geom['filename'], path=os.path.join(self.tmp_path, tmp_dir_name))
        #     geom_path = os.path.join(os.path.abspath(os.path.join(self.tmp_path, tmp_dir_name)), geom['filename'])
        # else:
        if 'sourcefilepath' in geom:
            geom_path = os.path.normpath(os.path.join(os.path.dirname(geom['sourcefilepath']), geom['filename']))
            log('sourcefilepath: ' + geom_path, 'DEBUG', 'createGeometry')
        else:
            geom_path = geom['filename']
        # TODO still up to date?
        # Remove 'urdf/package://{package_name}' to workaround the lack
        # of rospack here. This supposes that the urdf file is in the
        # urdf folder and that the meshes are in the meshes folder at
        # the same level as the urdf folder.
        if 'package://' in geom_path:
            geom_path = re.sub(r'(.*)urdf/package://([^/]+)/(.*)', '\\1\\3', geom_path)

        bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes[geomsrc])
        meshname = "".join(os.path.basename(geom["filename"]).split(".")[:-1])
        if not os.path.isfile(geom_path):
            log(geom_path + " is no file. Object " + viscol['name'] +
                " will have empty mesh!", "ERROR", "createGeometry")
            bpy.data.meshes.new(meshname)
        if meshname in bpy.data.meshes:
            log('Assigning copy of existing mesh ' + meshname + ' to ' + viscol['name'], 'INFO', 'createGeometry')
            bpy.ops.object.add(type='MESH')
            newgeom = bpy.context.object
            newgeom.data = bpy.data.meshes[meshname]
        else:
            log('Importing mesh for link element ' + viscol['name'], 'INFO', 'createGeometry')
            filetype = geom['filename'].split('.')[-1].lower()
            newgeom = meshes.importMesh(geom_path, filetype)
            newgeom.data.name = meshname
            if not newgeom:
                log('Failed to import mesh file ' + geom['filename'], 'ERROR', 'createGeometry')
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
            log("Could not determine geometry type of " + geomsrc + viscol['name'] + '. Placing empty coordinate system.', "ERROR")
            bpy.ops.object.empty_add(type='PLAIN_AXES', radius=0.2)
            bpy.context.active_object.name = viscol['name']
            return None
        log('Creating primtive for obj ' + viscol['name'], 'INFO', 'createGeometry')
        newgeom = bUtils.createPrimitive(viscol['name'], geomtype, dimensions, player=geomsrc)
        newgeom.select = True
        bpy.ops.object.transform_apply(scale=True)

    # from here it's the same for both meshes and primitives
    newgeom.phobostype = geomsrc
    newgeom['geometry/type'] = geomtype
    if geomsrc == 'visual':
        try:
            if 'name' in viscol['material']:
                assignMaterial(newgeom, viscol['material']['name'])
            else:
                assignMaterial(newgeom, viscol['material'])
        except KeyError:
            log('No material for obj ' + viscol['name'], 'DEBUG', 'createGeometry')
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
