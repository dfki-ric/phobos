#!/usr/bin/python

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

File export.py

Created on 13 Feb 2014

@author: Kai von Szadkowski
"""

import bpy
import mathutils
import os
from datetime import datetime
import yaml
import struct
import itertools
from bpy.types import Operator
from bpy.props import BoolProperty
from phobos.utility import *
from . import marssceneexport as mse
from . import robotdictionary


def register():
    print("Registering export...")


def unregister():
    print("Unregistering export...")

indent = '  '
xmlHeader = '<?xml version="1.0"?>\n'
xmlFooter = indent+'</robot>\n'


def exportBobj(path, obj):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select = True#
    bpy.context.scene.objects.active = obj
    #TODO: make this exception-handled
    totverts = totuvco = totno = 1

    globalNormals = {}

    # ignore dupli children
    if obj.parent and obj.parent.dupli_type in {'VERTS', 'FACES'}:
        print(obj.name, 'is a dupli child - ignoring')
        return

    mesh = obj.to_mesh(bpy.context.scene, True, 'PREVIEW')
    #mesh.transform(obj.matrix_world)

    faceuv = len(mesh.uv_textures)
    if faceuv:
        uv_layer = mesh.uv_textures.active.data[:]

    if bpy.app.version[0] * 100 + bpy.app.version[1] >= 265:
        face_index_pairs = [(face, index) for index, face in enumerate(mesh.tessfaces)]
    else:
        face_index_pairs = [(face, index) for index, face in enumerate(mesh.faces)]

    mesh.calc_normals()

    me_verts = mesh.vertices[:]

    out = open(os.path.join(path, obj.name) + '.bobj', "wb")

    for v in mesh.vertices:
        out.write(struct.pack('ifff', 1, v.co[0], v.co[1], v.co[2]))

    if faceuv:
        uv = uvkey = uv_dict = f_index = uv_index = None

        uv_face_mapping = [[0, 0, 0, 0]] * len(face_index_pairs)  # a bit of a waste for tri's :/

        uv_dict = {}  # could use a set() here
        if bpy.app.version[1] >= 65:
            uv_layer = mesh.tessface_uv_textures.active.data[:]
        else:
            uv_layer = mesh.uv_textures.active.data
        for f, f_index in face_index_pairs:
            for uv_index, uv in enumerate(uv_layer[f_index].uv):
                uvkey = round(uv[0], 6), round(uv[1], 6)
                try:
                    uv_face_mapping[f_index][uv_index] = uv_dict[uvkey]
                except:  # TODO: what can really go wrong here?
                    uv_face_mapping[f_index][uv_index] = uv_dict[uvkey] = len(uv_dict)
                    out.write(struct.pack('iff', 2, uv[0], uv[1]))

        del uv, uvkey, uv_dict, f_index, uv_index

    for f, f_index in face_index_pairs:
        if f.use_smooth:
            for v_idx in f.vertices:
                v = me_verts[v_idx]
                noKey = roundVector(v.normal, 6)
                if noKey not in globalNormals:
                    globalNormals[noKey] = totno
                    totno += 1
                    out.write(struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2]))
        else:
            # Hard, 1 normal from the face.
            noKey = roundVector(f.normal, 6)
            if noKey not in globalNormals:
                globalNormals[noKey] = totno
                totno += 1
                out.write(struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2]))

    for f, f_index in face_index_pairs:
        f_smooth = f.use_smooth
        # write smooth info for face?

        f_v_orig = [(vi, me_verts[v_idx]) for vi, v_idx in enumerate(f.vertices)]

        if len(f_v_orig) == 3:
            f_v_iter = (f_v_orig, )
        else:
            f_v_iter = (f_v_orig[0], f_v_orig[1], f_v_orig[2]), (f_v_orig[0], f_v_orig[2], f_v_orig[3])

        for f_v in f_v_iter:
            da = struct.pack('i', 4)
            out.write(da)

            if faceuv:
                if f_smooth:  # Smoothed, use vertex normals
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], globalNormals[roundVector(v.normal, 6)]))
                else:  # No smoothing, face normals
                    no = globalNormals[roundVector(f.normal, 6)]
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], no))
            else:  # No UV's
                if f_smooth:  # Smoothed, use vertex normals
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, 0, globalNormals[roundVector(v.normal, 6)]))
                else:  # No smoothing, face normals
                    no = globalNormals[roundVector(f.normal, 6)]
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, 0, no))
    out.close()


def exportObj(path, obj):
    objname = obj.name
    obj.name = 'tmp_export_666'  # surely no one will ever name an object like so
    tmpobject = createPrimitive(objname, 'box', (2.0, 2.0, 2.0))
    tmpobject.data = obj.data  # copy the mesh here
    outpath = os.path.join(path, objname) + '.obj'
    bpy.ops.export_scene.obj(filepath=outpath, use_selection=True, use_normals=True, use_materials=False)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = objname

    #This is the old implementation which did not work properly (08.08.2014)
    #bpy.ops.object.select_all(action='DESELECT')
    #obj.select = True
    #outpath = os.path.join(path, obj.name) + '.obj'
    #world_matrix = obj.matrix_world.copy()
    ##inverse_local_rotation = obj.matrix_local.to_euler().to_matrix().inverted()
    ##world_scale = world_matrix.to_scale() TODO: implement scale
    ## we move the object to the world origin and revert its local rotation
    ##print(inverse_local_rotation, mathutils.Matrix.Translation((0, 0, 0)))
    ##obj.matrix_world = inverse_local_rotation.to_4x4() * mathutils.Matrix.Identity(4)
    #obj.matrix_world = mathutils.Matrix.Identity(4)
    #bpy.ops.export_scene.obj(filepath=outpath, axis_forward='-Z',
    #                         axis_up='Y', use_selection=True, use_normals=True)
    #obj.matrix_world = world_matrix


def exportStl(path, obj):
    objname = obj.name
    obj.name = 'tmp_export_666'  # surely no one will ever name an object like so
    tmpobject = createPrimitive(objname, 'box', (2.0, 2.0, 2.0))
    tmpobject.data = obj.data  # copy the mesh here
    outpath = os.path.join(path, objname) + '.stl'
    bpy.ops.export_mesh.stl(filepath=outpath)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = objname


def exportModelToYAML(model, filepath):
    print("phobos YAML export: Writing model data to", filepath )
    with open(filepath, 'w') as outputfile:
        outputfile.write('#YAML dump of robot model "'+model['modelname']+'", '+datetime.now().strftime("%Y%m%d_%H:%M")+"\n\n")
        outputfile.write(yaml.dump(model)) # default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries


def xmlline(ind, tag, names, values):
    line = []
    line.append(indent*ind+'<'+tag)
    for i in range(len(names)):
        line.append(' '+names[i]+'="'+str(values[i])+'"')
    line.append('/>\n')
    return ''.join(line)


def l2str(items, start=-1, end=-1):
    line = []
    i = start if start >= 0 else 0
    maxi = end if end >= 0 else len(items)
    while i < maxi:
        line.append(str(items[i])+' ')
        i += 1
    return ''.join(line)[0:-1]


def gatherAnnotations(model):
    annotations = {}
    types = ('links', 'joints', 'sensors', 'motors', 'controllers', 'materials')
    #fixme: collision / visual??
    for type in types:
        for elementname in model[type]:
            element = model[type][elementname]
            for key in element:
                if key.startswith('$'):
                    category = key[1:]
                    print(elementname, category)
                    if category not in annotations:
                        annotations[category] = []
                    tmpdict = {k: element[key][k] for k in element[key]}
                    tmpdict['type'] = type[:-1]
                    tmpdict['name'] = elementname
                    annotations[category].append(tmpdict)
    return annotations


def writeURDFGeometry(output, element):
    output.append(indent*4+'<geometry>\n')
    if element['type'] == 'box':
        output.append(xmlline(5, 'box', ['size'], [l2str(element['size'])]))
    elif element['type'] == "cylinder":
        output.append(xmlline(5, 'cylinder', ['radius', 'length'], [element['radius'], element['height']]))
    elif element['type'] == "sphere":
        output.append(xmlline(5, 'sphere', ['radius'], [element['radius']]))
    elif element['type'] in ['capsule', 'mesh']: # capsules are not supported in URDF and are emulated using meshes
        output.append(xmlline(5, 'mesh', ['filename', 'scale'], [element['filename'], element['scale']])) # TODO: check if this is working properly
    output.append(indent*4+'</geometry>\n')


def exportModelToURDF(model, filepath):
    output = []
    output.append(xmlHeader)
    output.append(indent+'<robot name="'+model['modelname']+'">\n\n')
    #export link information
    for l in model['links'].keys():
        link = model['links'][l]
        output.append(indent*2+'<link name="'+l+'">\n')
        if 'mass' in link['inertial'] and 'inertia' in link['inertial']:
            output.append(indent*3+'<inertial>\n')
            if 'pose' in link['inertial']:
                output.append(xmlline(4, 'origin', ['xyz', 'rpy'], [l2str(link['inertial']['pose']['translation']), l2str(link['inertial']['pose']['rotation_euler'])]))
            output.append(xmlline(4, 'mass', ['value'], [str(link['inertial']['mass'])]))
            output.append(xmlline(4, 'inertia', ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'], [str(i) for i in link['inertial']['inertia']]))
            output.append(indent*3+'</inertial>\n')
        #visual object
        if link['visual']:
            for v in link['visual']:
                vis = link['visual'][v]
                output.append(indent*3+'<visual name="' + vis['name'] + '">\n')
                output.append(xmlline(4, 'origin', ['xyz', 'rpy'], [l2str(vis['pose']['translation']), l2str(vis['pose']['rotation_euler'])]))
                writeURDFGeometry(output, vis['geometry'])
                if 'material' in vis:
                    if model['materials'][vis['material']]['users'] == 0: #FIXME: change back to 1 when implemented in urdfloader
                        mat = model['materials'][vis['material']]
                        output.append(indent*4+'<material name="' + mat['name'] + '">\n')
                        color = mat['diffuseFront']
                        output.append(indent*5+'<color rgba="'+l2str([color[num] for num in ['r', 'g', 'b']]) + ' ' + str(mat["transparency"]) + '"/>\n')
                        if 'texturename' in mat:
                            output.append(indent*5+'<texture filename="'+mat['texturename']+'"/>\n')
                        output.append(indent*4+'</material>\n')
                    else:
                        output.append(indent*4+'<material name="' + vis["material"] + '"/>\n')
                output.append(indent*3+'</visual>\n')
        #collision object
        if link['collision']:
            for c in link['collision']:
                col = link['collision'][c]
                output.append(indent*3+'<collision name="' + col['name'] + '">\n')
                output.append(xmlline(4, 'origin', ['xyz', 'rpy'], [l2str(col['pose']['translation']), l2str(col['pose']['rotation_euler'])]))
                writeURDFGeometry(output, col['geometry'])
                output.append(indent*3+'</collision>\n')
        output.append(indent*2+'</link>\n\n')
    #export joint information
    for j in model['joints']:
        joint = model['joints'][j]
        output.append(indent*2+'<joint name="'+joint['name']+'" type="'+joint["type"]+'">\n')
        child = model['links'][joint["child"]]
        output.append(xmlline(3, 'origin', ['xyz', 'rpy'], [l2str(child['pose']['translation']), l2str(child['pose']['rotation_euler'])]))
        output.append(indent*3+'<parent link="'+joint["parent"]+'"/>\n')
        output.append(indent*3+'<child link="'+joint["child"]+'"/>\n')
        if 'axis' in joint:
            output.append(indent*3+'<axis xyz="'+l2str(joint['axis'])+'"/>\n')
        if 'limits' in joint:
            output.append(xmlline(3, 'limit', [p for p in joint['limits']], [joint['limits'][p] for p in joint['limits']]))
        output.append(indent*2+'</joint>\n\n')
    #export material information
    for m in model['materials']:
        if model['materials'][m]['users'] > 0:  # FIXME: change back to 1 when implemented in urdfloader
            output.append(indent*2+'<material name="' + m + '">\n')
            color = model['materials'][m]['diffuseFront']
            transparency = model['materials'][m]['transparency'] if 'transparency' in model['materials'][m] else 0.0
            output.append(indent*3+'<color rgba="'+l2str([color[num] for num in ['r', 'g', 'b']]) + ' ' + str(1.0-transparency) + '"/>\n')
            if 'texturename' in model['materials'][m]:
                            output.append(indent*3+'<texture filename="'+model['materials'][m]['texturename']+'"/>\n')
            output.append(indent*2+'</material>\n\n')
    #finish the export
    output.append(xmlFooter)
    with open(filepath, 'w') as outputfile:
        outputfile.write(''.join(output))
    # problem of different joint transformations needed for fixed joints
    print("phobos URDF export: Writing model data to", filepath )


def exportModelToSRDF(model, path):
    """
    This function exports the SRDF-relevant data from the dictionary to a specified path. Further detail on different
    elements of SRDF:

    <group>
    Groups in SRDF can contain *links*, *joints*, *chains* and other *groups* (the latter two of which have to be specified
    upstream. As nested groups is just a shortcut for adding links and joints to a group, it is not supported and the
    user will have to add all links and joints explicitly to each group.
    Originally both links and their associated parent joints were added. SRDF however implicitly assumes this, so the
    current implementation only adds the links.

    <chain>
    Chains are elements to simplify defining groups and are supported. The dictionary also contains a list of all
    elements belonging to that chain, which is discarded and not written to SRDF, however. It might be written to SMURF
    in the future.

    <link_sphere_approximatio>
    SRDF defines the convention that if no sphere is defined, one large sphere is
    assumed for that link. If one wants to have no sphere at all, it is necessary to define a sphere of radius 0.
    As one large sphere can be explicitly added by the user and should be if that is what he intends (WYSIWYG),
    we add a sphere of radius 0 by default if no sphere is specified.

    Currently not supported:
    - <group_state>
    - <virtual_joint>

    :param model: a robot model dictionary
    :param path: the outpath for the file
    :return: None
    """
    output = []
    output.append(xmlHeader)
    output.append(indent+'<robot name="'+model['modelname']+'">\n\n')
    for groupname in model['groups']:
        output.append(indent*2 + '<group name="'+groupname+'">\n')
        for member in model['groups'][groupname]:
            output.append(indent*3+'<'+member['type']+' name="'+member['name']+'" />\n')
        output.append(indent*2 + '</group>\n\n')
    for chainname in model['chains']:
        output.append(indent*2 + '<group name="'+chainname+'">\n')
        chain = model['chains'][chainname]
        output.append(indent*3 + '<chain base_link="'+chain['start']+'" tip_link="'+chain['end']+'" />\n')
        output.append(indent*2 + '</group>\n\n')
    #for joint in model['state']['joints']:
    #    pass
    #passive joints
    for joint in model['joints']:
        try:
            if model['joints'][joint]['passive']:
                output.append(indent*2+'<passive_joint name="'+model['links'][joint]['name']+'">\n\n')
        except KeyError:
            pass
    for link in model['links']:
        if len(model['links'][link]['approxcollision']) > 0:
            output.append(indent*2+'<link_sphere_approximation link="'+model['links'][link]['name']+'">\n')
            for sphere in model['links'][link]['approxcollision']:
                output.append(xmlline(3, 'sphere', ('center', 'radius'), (l2str(sphere['center']), sphere['radius'])))
            output.append(indent*2+'</link_sphere_approximation>\n\n')
        else:
            output.append(indent*2+'<link_sphere_approximation link="'+model['links'][link]['name']+'">\n')
            output.append(xmlline(3, 'sphere', ('center', 'radius'), ('0.0 0.0 0.0', '0')))
            output.append(indent*2+'</link_sphere_approximation>\n\n')
    #calculate collision-exclusive links
    for combination in itertools.combinations(model['links'], 2):
        link1 = model['links'][combination[0]]
        link2 = model['links'][combination[1]]
        # TODO: we might want to automatically add parent/child link combinations
        try:
            if link1['collision_bitmask'] & link2['collision_bitmask'] == 0:
                output.append(xmlline(2, 'disable_collisions', ('link1', 'link2'), (link1['name'], link2['name'])))
        except KeyError:
            pass
    output.append('\n')
    #finish the export
    output.append(xmlFooter)
    with open(path, 'w') as outputfile:
        outputfile.write(''.join(output))
    # FIXME: problem of different joint transformations needed for fixed joints
    print("phobos SRDF export: Writing model data to", path)


def exportModelToSMURF(model, path):
    export = {#'semantics': model['groups'] != {} or model['chains'] != {},,
              'state': False,  #model['state'] != {}, #TODO: handle state
              'materials': model['materials'] != {},
              'sensors': model['sensors'] != {},
              'motors': model['motors'] != {},
              'controllers': model['controllers'] != {},
              'simulation': model['simulation'] != {}
              }
    #create all filenames
    smurf_filename = model['modelname'] + ".smurf"
    urdf_filename =  model['modelname'] + ".urdf"
    filenames = {#'semantics': model['modelname'] + "_semantics.yml",
                 'state': model['modelname'] + "_state.yml",
                 'materials': model['modelname'] + "_materials.yml",
                 'sensors': model['modelname'] + "_sensors.yml",
                 'motors': model['modelname'] + "_motors.yml",
                 'controllers': model['modelname'] + "_controllers.yml",
                 'simulation': model['modelname'] + "_simulation.yml"
                 }

    annotationdict = gatherAnnotations(model)
    print(annotationdict)
    for category in annotationdict:
        filenames[category] = model['modelname'] + '_'+category+'.yml'
        export[category] = True

    infostring = ' definition SMURF file for "'+model['modelname']+'", '+model["date"]+"\n\n"

    #write model information
    print('Writing SMURF information to', smurf_filename)
    modeldata = {}
    modeldata["date"] = model["date"]
    modeldata["files"] = [urdf_filename] + [filenames[f] for f in filenames if export[f]]
    with open(path + smurf_filename, 'w') as op:
        op.write('#main SMURF file of model "'+model['modelname']+'"\n\n')
        op.write("modelname: "+model['modelname']+"\n")
        op.write(yaml.dump(modeldata, default_flow_style=False))

    #write urdf
    exportModelToURDF(model, path + urdf_filename)

    # #write semantics (SRDF information in YML format)
    # if export['semantics']:
    #     with open(path + filenames['semantics'], 'w') as op:
    #         op.write('#semantics'+infostring)
    #         op.write("modelname: "+model['modelname']+'\n')
    #         semantics = {}
    #         if model['groups'] != {}:
    #             semantics['groups'] = model['groups']
    #         if model['chains'] != {}:
    #             semantics['chains'] = model['chains']
    #         op.write(yaml.dump(semantics, default_flow_style=False))

    #write state (state information of all joints, sensor & motor activity etc.) #TODO: implement everything but joints
    if export['state']:
        states = []
        #gather all states
        for jointname in model['joints']:
            joint = model['joints'][jointname]
            if 'state' in joint:  # this should always be the case, but testing doesn't hurt
                tmpstate = joint['state'].copy()
                tmpstate['name'] = jointname
                states.append(joint['state'])
        with open(path + filenames['state'], 'w') as op:
            op.write('#state'+infostring)
            op.write("modelname: "+model['modelname']+'\n')
            op.write(yaml.dump(states))#, default_flow_style=False))

    #write materials, sensors, motors & controllers
    for data in ['materials', 'sensors', 'motors', 'controllers']:
        if export[data]:
            with open(path + filenames[data], 'w') as op:
                op.write('#' + data + infostring)
                op.write(yaml.dump({data: list(model[data].values())}, default_flow_style=False))

#write additional information
    for data in annotationdict.keys():
        if export[data]:
            with open(path + filenames[data], 'w') as op:
                op.write('#' + data + infostring)
                op.write(yaml.dump({data: annotationdict[data]}, default_flow_style=False))


def exportSceneToSMURF(path):
    """Exports all robots in a scene to separate SMURF folders."""
    pass


def exportModelToMARS(model, path):
    """Exports selected robot as a MARS scene"""
    mse.exportModelToMARS(model, path)


def securepath(path): #TODO: this is totally not error-handled!
    if not os.path.exists(path):
        os.makedirs(path)
    return os.path.expanduser(path)


class ExportModelOperator(Operator):
    """ExportModelOperator"""
    bl_idname = "object.phobos_export_robot"
    bl_label = "Export the selected model(s)"
    bl_options = {'REGISTER', 'UNDO'}

    typetags = BoolProperty(
                name = 'typetags',
                default = False,
                description = 'add link/joint typetags'
                )

    def execute(self, context):
        export(self.typetags)
        return {'FINISHED'}


def export(typetags=False):
    #TODO: check if all selected objects are on visible layers (option bpy.ops.object.select_all()?)
    if bpy.data.worlds[0].relativePath:
        outpath = securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"), bpy.data.worlds[0].path)))
    else:
        outpath = securepath(os.path.expanduser(bpy.data.worlds[0].path))
    yaml = bpy.data.worlds[0].exportYAML
    urdf = bpy.data.worlds[0].exportURDF
    srdf = bpy.data.worlds[0].exportSRDF
    smurf = bpy.data.worlds[0].exportSMURF
    mars = bpy.data.worlds[0].exportMARSscene
    meshexp = bpy.data.worlds[0].exportMesh
    objexp = bpy.data.worlds[0].useObj
    bobjexp = bpy.data.worlds[0].useBobj
    stlexp = bpy.data.worlds[0].useStl
    objectlist = bpy.context.selected_objects

    if yaml or urdf or smurf or mars:
        robot = robotdictionary.buildRobotDictionary(typetags)
        if yaml:
            exportModelToYAML(robot, outpath + robot["modelname"] + "_dict.yml")
        if mars:
            exportModelToMARS(robot, outpath + robot["modelname"] + "_mars.scene")
        if srdf:
            exportModelToSRDF(robot, outpath + robot["modelname"] + ".srdf")
        if smurf:
            exportModelToSMURF(robot, outpath)
        elif urdf:
            exportModelToURDF(robot, outpath + robot["modelname"] + ".urdf")
    selectObjects(objectlist, True)
    if meshexp:
        show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269
        if show_progress:
            wm = bpy.context.window_manager
            total = float(len(objectlist))
            wm.progress_begin(0, total)
            i = 1
        for obj in bpy.context.selected_objects:
            if ((obj.phobostype == 'visual' or obj.phobostype == 'collision')
                    and obj['geometry/type'] == 'mesh' and 'filename' not in obj):
                if objexp:
                    exportObj(outpath, obj)
                if bobjexp:
                    exportBobj(outpath, obj)
                if stlexp:
                    exportStl(outpath, obj)
            if show_progress:
                wm.progress_update(i)
                i += 1
        if show_progress:
            wm.progress_end()
