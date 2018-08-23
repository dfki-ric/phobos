
# TODO add shebang and document introduction
import os
import bpy
import mathutils
import phobos.defs as defs
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.utils.editing as eUtils
import phobos.io.meshes.meshes as meshes
from phobos.model.materials import assignMaterial
from phobos.phoboslog import log
from phobos.utils.validation import validate


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


@validate('geometry_type')
def deriveGeometry(obj, adjust=False, **kwargs):
    """This function derives the geometry from an object.

    The returned dictionary contains this information (depending on the geometry type):
        *type*: geometry type of the object
        *size*: dimensions of the object (only for box and mesh)
        *radius*: radius of the object (only for cylinder, capsule and sphere)
        *lenght*: length of the object (only for cylinder and capsule)
        *scale*: scale of the object (only for mesh)

    Args:
        obj (bpy_types.Object): object to derive the geometry from

    Returns:
        dict -- dictionary representation of the geometry
    """
    geometry = {'type': obj['geometry/type']}
    gtype = obj['geometry/type']

    # enrich the dictionary with sizes, lengths, etc depending on geometry type
    if gtype == 'box':
        geometry['size'] = list(obj.dimensions)

    elif gtype == 'cylinder':
        geometry['radius'] = obj.dimensions[0]/2
        geometry['length'] = obj.dimensions[2]

    elif gtype == 'capsule':
        geometry['radius'] = obj.dimensions[0]/2
        geometry['length'] = obj.dimensions[2] - obj.dimensions[0]

    elif gtype == 'sphere':
        geometry['radius'] = obj.dimensions[0]/2

    elif gtype == 'mesh':
        geometry['filename'] = obj.data.name
        geometry['scale'] = deriveScale(obj)
        # FIXME: is this needed to calculate an approximate inertia
        geometry['size'] = list(obj.dimensions)

    return geometry


def deriveScale(obj):
    """Returns the scale of the specified object.

    Object scale is gathered from the matrix_world, as the link scales in Blender might change the
    mesh scale, too.

    Args:
        obj (bpy.types.Object): object to derive the scale of

    Returns:
        list(float) -- three scale floats (x, y, z) combined from all parents and the object itself
    """
    return list(obj.matrix_world.to_scale())


def createGeometry(viscol, geomsrc, linkobj=None):
    """Creates Blender object for visual or collision objects.

    If the creation fails, nothing is returned.

    These entries in the dictionary are mandatory:
        *geometry*:
            *type*: type of geometry (mesh, box, cylinder, sphere)

    Depending on the geometry type other values are required: *size*, *radius*, *length*

    These entries are optional:
        *geometry*:
            *scale*: scale for the new geometry
        *material*: material name to assign to the visual
        *pose*: specifies the placement of the new object relative to the optional linkobj
            *translation*: position vector for the new object
            *rotation_euler*: rotation for the new object

    Furthermore any generic properties, prepended by a `$` will be added as custom properties to the
    visual/collision object. E.g. $test/etc would be put to visual/test/etc for a visual object.
    However, these properties are extracted only in the first layer of hierarchy.

    Args:
        viscol (dict): visual/collision model dictionary representation
        geomsrc (str): phobostype of the new object
        linkobj (bpy.types.Object): link object to attach the visual/collision object to

    Returns:
      bpy.types.Object or None
    """
    if 'geometry' not in viscol or viscol['geometry'] is {}:
        log("Could not create {}. Geometry information not defined!".format(geomsrc), 'ERROR')
        return None

    bpy.ops.object.select_all(action='DESELECT')
    geom = viscol['geometry']

    # create the Blender object
    if geom['type'] == 'mesh':
        bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes[geomsrc])
        meshname = "".join(os.path.basename(geom["filename"]).split(".")[:-1])
        if not os.path.isfile(geom['filename']):
            log("This path " + geom['filename'] + " is no file. Object " + viscol['name'] +
                " will have empty mesh!", 'ERROR')
            bpy.ops.object.add(type='MESH')
            newgeom = bpy.context.active_object
            nUtils.safelyName(newgeom, viscol['name'], phobostype=geomsrc)
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
        log("Creating primtive for {0}: {1}".format(geomsrc, viscol['name']), 'INFO')
        newgeom = bUtils.createPrimitive(viscol['name'], geom['type'], dimensions, phobostype=geomsrc)
        newgeom.select = True
        bpy.ops.object.transform_apply(scale=True)

    # from here it's the same for both meshes and primitives
    newgeom['geometry/type'] = geom['type']
    if geomsrc == 'visual':
        if 'material' in viscol:
            assignMaterial(newgeom, viscol['material'])
        else:
            log('No material for visual {}.'.format(viscol['name']), 'WARNING')

    # write generic custom properties
    for prop in viscol:
        if prop.startswith('$'):
            for tag in viscol[prop]:
                newgeom[prop[1:]+'/'+tag] = viscol[prop][tag]

    nUtils.safelyName(newgeom, viscol['name'])
    newgeom[geomsrc + '/name'] = viscol['name']
    newgeom.phobostype = geomsrc

    # place geometric object relative to its parent link
    if linkobj:
        if 'pose' in viscol:
            log("Setting transformation of element: " + viscol['name'], 'DEBUG')
            location = mathutils.Matrix.Translation(viscol['pose']['translation'])
            rotation = mathutils.Euler(tuple(viscol['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
        else:
            log("No pose in element: " + viscol['name'], 'DEBUG')
            location = mathutils.Matrix.Identity(4)
            rotation = mathutils.Matrix.Identity(4)
        sUtils.selectObjects([newgeom, linkobj], True, 1)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
        newgeom.matrix_local = location * rotation

    # scale imported object
    if 'scale' in geom:
        newgeom.scale = geom['scale']

    # make object smooth
    eUtils.smoothen_surface(newgeom)

    return newgeom
