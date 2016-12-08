__author__ = 'kavonszadkowski'

import os
import zipfile
import bpy
import mathutils
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
import phobos.model.poses as poses
from phobos.phoboslog import log


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
            filename = obj.data.name
            expsets = bpy.data.worlds[0].phobosexportsettings
            if expsets.useObj:
                filename += ".obj"
            elif expsets.useStl:
                filename += ".stl"
            elif expsets.useDae:
                filename += ".dae"
            else:
                filename += ".obj"
            geometry['filename'] = os.path.join('meshes', filename)
            geometry['scale'] = list(obj.scale)
            geometry['size'] = list(obj.dimensions)  # FIXME: is this needed to calculate an approximate inertia
        else:  # any other geometry type, i.e. 'plane'
            geometry['size'] = list(obj.dimensions)
        return geometry
    except KeyError as err:
        log("Undefined geometry for object " + nUtils.getObjectName(obj)
            + " " + str(err), "ERROR", "deriveGeometry")
        return None


def createGeometry(self, viscol, geomsrc):
    """This function takes a visual or collision object and creates the geometry for it.

    :param viscol: The visual/collision object you want to create the geometry for.
    :type viscol: dict
    :param geomsrc: The new viscols phobostype.
    :type geomsrc: str

    """
    #TODO: Write doc
    newgeom = None
    if viscol['geometry'] is not {}:
        dimensions = None
        bpy.ops.object.select_all(action='DESELECT')
        geom = viscol['geometry']
        geomtype = geom['type']
        # create the Blender object
        # tag all objects
        for obj in bpy.data.objects:
            obj['phobosTag'] = True
        if geomtype == 'mesh':
            if hasattr(self, 'zipped') and self.zipped:
                if not os.path.isdir(os.path.join(self.tmp_path, tmp_dir_name)):
                    os.mkdir(os.path.join(self.tmp_path, tmp_dir_name))
                archive = zipfile.ZipFile(self.filepath)
                archive.extract(geom['filename'], path=os.path.join(self.tmp_path, tmp_dir_name))
                geom_path = os.path.join(os.path.abspath(os.path.join(self.tmp_path, tmp_dir_name)), geom['filename'])
            else:
                geom_path = os.path.join(self.path, geom['filename'])
            # Remove 'urdf/package://{package_name}' to workaround the lack
            # of rospack here. This supposes that the urdf file is in the
            # urdf folder and that the meshes are in the meshes folder at
            # the same level as the urdf folder.
            if 'package://' in geom_path:
               geom_path = re.sub(r'(.*)urdf/package://([^/]+)/(.*)',
                                  '\\1\\3',
                                  geom_path)

            bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes[geomsrc])
            meshname = "".join(os.path.basename(geom["filename"]).split(".")[:-1])
            if not os.path.isfile(geom_path):
                log(geom_path + " is no file. Object " + self.praefixNames(viscol['name'], geomsrc) + " will have empty mesh!", "ERROR", "importer:createGeometry")
                bpy.data.meshes.new(meshname)
            if meshname in bpy.data.meshes:
                bpy.ops.object.add(type='MESH')
                newgeom = bpy.context.object
                newgeom.data = bpy.data.meshes[meshname]
            else:
                filetype = geom['filename'].split('.')[-1]
                if filetype == 'obj' or filetype == 'OBJ':
                    bpy.ops.import_scene.obj(filepath=geom_path)
                elif filetype == 'stl' or filetype == 'STL':
                    bpy.ops.import_mesh.stl(filepath=geom_path)
                # find the newly imported obj
                for obj in bpy.data.objects:
                    if 'phobosTag' not in obj:
                        newgeom = obj
                        #with obj file import, blender only turns the object, not the vertices,
                        #leaving a rotation in the matrix_basis, which we here get rid of
                        if filetype == 'obj':
                            bpy.ops.object.select_all(action='DESELECT')
                            newgeom.select = True
                            bpy.ops.object.transform_apply(rotation=True)
            #print(viscol)
                        newgeom['filename'] = geom['filename']
            #newgeom.select = True
            #if 'scale' in geom:
            #    newgeom.scale = geom['scale']
            #bpy.ops.object.transform_apply(scale=True)
                for obj in bpy.data.objects:
                    if 'phobosTag' in obj:
                        del obj['phobosTag']
        elif geomtype == 'box':
            dimensions = geom['size']
        elif geomtype == 'cylinder':
            dimensions = (geom['radius'], geom['length'])
        elif geomtype == 'sphere':
            dimensions = geom['radius']
        else:
            log("Could not determine geometry type of " + geomsrc + viscol['name'] + '. Placing empty coordinate system.', "ERROR")
        if dimensions:  # if a standard primitive type is found, create the object
            newgeom = bUtils.createPrimitive(viscol['name'], geomtype, dimensions, player=geomsrc)
            newgeom.select = True
            bpy.ops.object.transform_apply(scale=True)
        if newgeom is not None:
            newgeom.phobostype = geomsrc
            newgeom['geometry/type'] = geomtype
            if geomsrc == 'visual':
                try:
                    if 'name' in viscol['material']:
                        newgeom.data.materials.append(bpy.data.materials[viscol['material']['name']])
                    else:
                        newgeom.data.materials.append(bpy.data.materials[viscol['material']])
                except KeyError:
                    log('No material for obj', viscol['name'])
        #FIXME: place empty coordinate system and return...what? Error handling of file import!
    for prop in viscol:
        if prop.startswith('$'):
            for tag in viscol[prop]:
                newgeom[prop[1:]+'/'+tag] = viscol[prop][tag]
    newgeom.name = self.praefixNames(viscol['name'], geomsrc)
    newgeom[geomsrc+"/name"] = viscol['name']
    return newgeom

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


def deriveCapsule(obj):
    """This function derives a capsule from a given blender object

    :param obj: The blender object to derive the capsule from.
    :type obj: bpy_types.Object
    :return: tuple

    """
    viscol_dict = {}
    capsule_pose = poses.deriveObjectPose(obj)
    rotation = capsule_pose['rotation_euler']
    capsule_radius = obj['geometry']['radius']
    for part in ['sphere1', 'cylinder', 'sphere2']:
        viscol = initObjectProperties(obj, phobostype='collision', ignoretypes='geometry')
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
            #print(list(matrix))
            pose['matrix'] = [list(vector) for vector in list(matrix)]
        viscol['geometry'] = geometry
        viscol['pose'] = pose
        try:
            viscol['bitmask'] = int(''.join(['1' if group else '0' for group in obj.rigid_body.collision_groups]), 2)
        except AttributeError:
            pass
        viscol_dict[part] = viscol
    return viscol_dict, obj.parent

