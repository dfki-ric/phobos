
# TODO add shebang and document introduction
import os
import bpy
import mathutils
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

    Args:
      obj(bpy_types.Object): The blender object to derive the geometry from.

    Returns:
      dict

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


def createGeometry(viscol, geomsrc, linkobj=None):
    """Creates Blender object for visual or collision objects.
    Returns reference to new object or None if creation failed.

    Args:
      viscol(dict): visual/collision dictionary element
      geomsrc(str): new object's phobostype
      linkobj(bpy.types.Object): link object

    Returns:
      bpy.types.Object or None

    """
    if 'geometry' not in viscol or viscol['geometry'] is {}:
        return None
    bpy.ops.object.select_all(action='DESELECT')
    geom = viscol['geometry']
    # create the Blender object
    if geom['type'] == 'mesh':
        bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes[geomsrc])
        meshname = "".join(os.path.basename(geom["filename"]).split(".")[:-1])
        if not os.path.isfile(geom['filename']):
            log(geom['filename'] + " is no file. Object " + viscol['name'] + " will have empty mesh!", "ERROR")
            bpy.data.meshes.new(meshname)
        else:
            if meshname in bpy.data.meshes:
                log('Assigning copy of existing mesh ' + meshname + ' to ' + viscol['name'], 'INFO')
                bpy.ops.object.add(type='MESH')
                newgeom = bpy.context.object
                newgeom.data = bpy.data.meshes[meshname]
            else:
                log("Importing mesh for {0} element: '{1}".format(geomsrc, viscol['name']), 'INFO')
                filetype = geom['filename'].split('.')[-1].lower()
                newgeom = meshes.importMesh(geom['filename'], filetype)
                newgeom.data.name = meshname
                if not newgeom:
                    log('Failed to import mesh file ' + geom['filename'], 'ERROR')
                    return
            # scale imported object
            if 'scale' in geom:
                newgeom.scale = geom['scale']
    else:
        if geom['type'] == 'box':
            dimensions = geom['size']
        elif geom['type'] == 'cylinder':
            dimensions = (geom['radius'], geom['length'])
        elif geom['type'] == 'sphere':
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
        newgeom = bUtils.createPrimitive(viscol['name'], geom['type'], dimensions, phobostype=geomsrc)
        newgeom.select = True
        bpy.ops.object.transform_apply(scale=True)

    # from here it's the same for both meshes and primitives
    newgeom['geometry/type'] = geom['type']
    if geomsrc == 'visual':
        try:
            assignMaterial(newgeom, viscol['material'])
        except KeyError:
            log('No material for visual ' + viscol['name'], 'DEBUG')
    for prop in viscol:
        if prop.startswith('$'):
            for tag in viscol[prop]:
                newgeom[prop[1:]+'/'+tag] = viscol[prop][tag]
    nUtils.safelyName(newgeom, viscol['name'])
    newgeom[geomsrc+"/name"] = viscol['name']
    newgeom.phobostype = geomsrc

    # place geometric object relative to its parent link
    if linkobj:
        if 'pose' in viscol:
            log('Setting transformation of element: ' + viscol['name'], 'DEBUG')
            location = mathutils.Matrix.Translation(viscol['pose']['translation'])
            rotation = mathutils.Euler(tuple(viscol['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
        else:
            log('No pose in element: ' + viscol['name'], 'DEBUG')
            location = mathutils.Matrix.Identity(4)
            rotation = mathutils.Matrix.Identity(4)
        sUtils.selectObjects([newgeom, linkobj], True, 1)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
        newgeom.matrix_local = location * rotation
        if 'scale' in viscol['geometry']:
            newgeom.scale = mathutils.Vector(viscol['geometry']['scale'])
    return newgeom
