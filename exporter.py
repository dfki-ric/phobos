#!/usr/bin/python

"""
.. module:: phobos.exporter
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadowski

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
from . import defs


def register():
    """
    This function registers this module.
    At the moment it does nothing.

    :return: Nothing

    """
    print("Registering export...")


def unregister():
    """
    This function unregisters this module.
    At the moment it does nothing.

    :return: Nothing

    """
    print("Unregistering export...")


indent = '  '
# xmlHeader = '<!-- created with Phobos ' + defs.version + ' -->\n<?xml version="1.0"?>\n'
xmlHeader = '<?xml version="1.0"?>\n<!-- created with Phobos ' + defs.version + ' -->\n'
xmlFooter = indent + '</robot>\n'

def veckey3d(v):
    return round(v.x, 6), round(v.y, 6), round(v.z, 6)

def exportBobj(path, obj):
    """This function exports an object to the specified path as a .bobj

    :param path: The path to export the object to. *without filename!*
    :type path: String
    :param obj: The blender object you want to export.
    :type: bpy.types.Object
    :return: Nothing.

    """
    bpy.ops.object.select_all(action='DESELECT')
    obj.select = True
    bpy.context.scene.objects.active = obj
    #TODO: make this exception-handled
    totverts = totuvco = totno = 1

    face_vert_index = 1 #Jan Paul: New, transferred from MARS code
    
    globalNormals = {}

    # ignore dupli children
    if obj.parent and obj.parent.dupli_type in {'VERTS', 'FACES'}:
        print(getObjectName(obj), 'is a dupli child - ignoring')
        return

    mesh = obj.to_mesh(bpy.context.scene, True, 'PREVIEW') #Jan Paul: ", True)": calculate tesselation faces added as test
    #mesh.transform(obj.matrix_world)

    write_uv = False #Jan Paul: New, transferred from MARS code
    faceuv = len(mesh.uv_textures)
    if faceuv:
        uv_layer = mesh.uv_textures.active.data[:]
        write_uv = True #Jan Paul: New, transferred from MARS code

    if bpy.app.version[0] * 100 + bpy.app.version[1] >= 265:
        face_index_pairs = [(face, index) for index, face in enumerate(mesh.tessfaces)]
    else:
        face_index_pairs = [(face, index) for index, face in enumerate(mesh.faces)]

    mesh.calc_normals()

    me_verts = mesh.vertices[:]

    out = open(determineMeshOutpath(obj, getObjectName(obj), 'bobj', path), "wb")

    for v in mesh.vertices:
        out.write(struct.pack('ifff', 1, v.co[0], v.co[1], v.co[2]))

    if faceuv:
        #print ("faceuv")
        uv = uvkey = uv_dict = f_index = uv_index = None

        #uv_face_mapping = [[0, 0, 0, 0]] * len(face_index_pairs)  # a bit of a waste for tri's :/
        uv_face_mapping = [[0, 0, 0, 0] for i in range(len(face_index_pairs))]  # a bit of a waste for tri's :/  #Jan Paul: New, transferred from MARS code
            
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
                    #print ("try OK")
                except:  # TODO: what can really go wrong here?
                    #print ("try Exception")
                    uv_face_mapping[f_index][uv_index] = uv_dict[uvkey] = len(uv_dict)
                    out.write(struct.pack('iff', 2, uv[0], uv[1]))

        uv_unique_count = len(uv_dict) #Jan Paul: New, transferred from MARS code
        
        del uv, uvkey, uv_dict, f_index, uv_index

    for f, f_index in face_index_pairs:
        if f.use_smooth:
            for v_idx in f.vertices:
                v = me_verts[v_idx]
                #noKey = roundVector(v.normal, 6)
                noKey = veckey3d(v.normal) #Jan Paul: New, transferred from MARS code
                if noKey not in globalNormals:
                    globalNormals[noKey] = totno
                    totno += 1
                    #out.write(struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2]))
                    da = struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2]) #Jan Paul: New, transferred from MARS code
                    out.write(da) #Jan Paul: New, transferred from MARS code
        else:
            # Hard, 1 normal from the face.
            #noKey = roundVector(f.normal, 6)
            noKey = veckey3d(f.normal) #Jan Paul: New, transferred from MARS code
            if noKey not in globalNormals:
                globalNormals[noKey] = totno
                totno += 1
                #out.write(struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2]))
                da = struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2]) #Jan Paul: New, transferred from MARS code
                out.write(da)

    for f, f_index in face_index_pairs:
        f_smooth = f.use_smooth
        if faceuv: #Jan Paul: New, transferred from MARS code
            tface = uv_layer[f_index] #Jan Paul: New, transferred from MARS code
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
                #print ("faceuv")
                if f_smooth:  # Smoothed, use vertex normals
                    #print ("f_smooth")
                    for vi, v in f_v:
                        #out.write(struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi],
                        #                      globalNormals[roundVector(v.normal, 6)]))
                        da = struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], globalNormals[veckey3d(v.normal)]) #Jan Paul: New, transferred from MARS code
                        out.write(da)  # vert, uv, normal #Jan Paul: New, transferred from MARS code
                else:  # No smoothing, face normals
                    #print ("hard")
                    no = globalNormals[roundVector(f.normal, 6)]
                    for vi, v in f_v:
                        #out.write(struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], no))
                        da = struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], no) #Jan Paul: New, transferred from MARS code
                        out.write(da)  # vert, uv, normal #Jan Paul: New, transferred from MARS code
            else:  # No UV's
                #print ("no UVs")
                if f_smooth:  # Smoothed, use vertex normals
                    for vi, v in f_v:
                        #out.write(struct.pack('iii', v.index + totverts, 0, globalNormals[roundVector(v.normal, 6)]))
                        da = struct.pack('iii', v.index + totverts, 0, globalNormals[veckey3d(v.normal)]) #Jan Paul: New, transferred from MARS code
                        out.write(da)  # vert, uv, normal #Jan Paul: New, transferred from MARS code
                else:  # No smoothing, face normals
                    #no = globalNormals[roundVector(f.normal, 6)]
                    no = globalNormals[veckey3d(f.normal)] #Jan Paul: New, transferred from MARS code
                    for vi, v in f_v:
                        #out.write(struct.pack('iii', v.index + totverts, 0, no))
                        da = struct.pack('iii', v.index + totverts, 0, no) #Jan Paul: New, transferred from MARS code
                        out.write(da)  # vert, uv, normal #Jan Paul: New, transferred from MARS code
    out.close()


def exportObj(path, obj):
    """This function exports a specific object to a chosen path as an .obj

    :param path: The path you want the object export to. *without the filename!*
    :type path: String
    :param obj: The blender object you want to export.
    :type obj: bpy.types.Object
    :return: Nothing.

    """
    objname = getObjectName(obj)
    oldBlenderObjName = obj.name
    obj.name = 'tmp_export_666'  # surely no one will ever name an object like so
    tmpobject = createPrimitive(objname, 'box', (2.0, 2.0, 2.0))
    tmpobject.data = obj.data  # copy the mesh here
    outpath = determineMeshOutpath(obj, objname, 'obj', path)
    bpy.ops.export_scene.obj(filepath=outpath, use_selection=True, use_normals=True, use_materials=False)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = oldBlenderObjName

    #This is the old implementation which did not work properly (08.08.2014)
    #bpy.ops.object.select_all(action='DESELECT')
    #obj.select = True
    #outpath = os.path.join(path, getObjectName(obj)) + '.obj'
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
    """This function exports a specific object to a chosen path as a .stl

    :param path: The path you want the object exported to. *without filename!*
    :type path: String
    :param obj: The blender object you want to export.
    :type obj: bpy.types.Object
    :return: Nothing.

    """
    objname = getObjectName(obj)
    oldBlenderObjectName = obj.name
    print("OBJNAME: " + objname)
    obj.name = 'tmp_export_666'  # surely no one will ever name an object like so
    tmpobject = createPrimitive(objname, 'box', (1.0, 1.0, 1.0))
    tmpobject.data = obj.data  # copy the mesh here
    outpath = determineMeshOutpath(obj, objname, 'stl', path)
    bpy.ops.export_mesh.stl(filepath=outpath)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = oldBlenderObjectName


def exportDae(path, obj):
    """This function exports a specific object to a chosen path as a .dae

    :param path: The path you want the object exported to. *without filename!*
    :type path: String
    :param obj: The blender object you want to export.
    :type obj: bpy.types.Object
    :return: Nothing.

    """
    objname = getObjectName(obj)
    oldBlenderObjectName = obj.name
    print("OBJNAME: " + objname)
    obj.name = 'tmp_export_666'  # surely no one will ever name an object like so
    tmpobject = createPrimitive(objname, 'box', (1.0, 1.0, 1.0))
    tmpobject.data = obj.data  # copy the mesh here
    outpath = determineMeshOutpath(obj, objname, 'dae', path)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.wm.collada_export(filepath=outpath, selected=True)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = oldBlenderObjectName


def exportModelToYAML(model, filepath):
    """This function exports a given robot model to a specified filepath as YAML.

    :param model: The robot model to export
    :type model: dict -- the generated robot model dictionary.
    :param filepath:  The filepath to export the robot to. *WITH filename!*
    :type filepath: String
    :return: Nothing.

    """
    print("phobos YAML export: Writing model data to", filepath)
    with open(filepath, 'w') as outputfile:
        outputfile.write('# YAML dump of robot model "' + model['modelname'] + '", ' + datetime.now().strftime(
            "%Y%m%d_%H:%M") + "\n")
        outputfile.write("# created with Phobos" + defs.version + " - https://github.com/rock-simulation/phobos\n\n")
        outputfile.write(yaml.dump(
            model))  # default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries


def xmlline(ind, tag, names, values):
    """This function generates a xml line with specified values.
    To use this function you need to know the indentation level you need for this line.
    Make sure the names and values list have the correct order.

    :param ind: The level of indentation
    :type ind: int, has to be positive!
    :param tag: This is the xml lines tag
    :type tag: String
    :param names: This are the names of the xml lines attributions.
    :type names: list, check for analogue order to values.
    :param values: This are the values of the xml lines attributions.
    :type values: list, check  for analogue order to names.
    :return: String -- the generated xml line.

    """
    #TODO: Make the ind secure against negative numbers with abs?
    line = [indent * ind + '<' + tag]
    for i in range(len(names)):
        line.append(' ' + names[i] + '="' + str(values[i]) + '"')
    line.append('/>\n')
    return ''.join(line)


def l2str(items, start=-1, end=-1):
    """This function takes a list and generates a String with its element.

    :param items: The list of elements you want to generate a String from. *Make sure the elements can be cast to
    Strings with str().*
    :type items: list
    :param start:
    :param end:
    :return:

    """
    line = []
    i = start if start >= 0 else 0
    maxi = end if end >= 0 else len(items)
    while i < maxi:
        line.append(str(items[i]) + ' ')
        i += 1
    return ''.join(line)[0:-1]


def gatherAnnotations(model):
    """This function gathers custom properties annotating elements of the robot
    across the model. These annotations were created in the robotdictionary.py
    module and are marked with a leading '$'.

    :param model: The robot model dictionary.
    :return: dict -- A dictionary of the gathered annotations.

    """
    annotations = {}
    elementlist = []
    types = ('links', 'joints', 'sensors', 'motors', 'controllers', 'materials')
    # gather information from directly accessible types
    for objtype in types:
        for elementname in model[objtype]:
            #tmpdict = model[objtype][elementname].copy()
            tmpdict = model[objtype][elementname]
            tmpdict['temp_type'] = objtype[:-1]
            elementlist.append(tmpdict)
    # add information from types hidden in links
    for linkname in model['links']:
        for objtype in ('collision', 'visual'):
            if objtype in model['links'][linkname]:
                for elementname in model['links'][linkname][objtype]:
                    #tmpdict = model['links'][linkname][objtype][elementname].copy()
                    tmpdict = model['links'][linkname][objtype][elementname]
                    #tmpdict['link'] = linkname
                    tmpdict['temp_type'] = objtype
                    elementlist.append(tmpdict)
        if 'inertial' in model['links'][linkname]:
            #tmpdict = model['links'][linkname]['inertial'].copy()
            tmpdict = model['links'][linkname]['inertial']
            #tmpdict['link'] = linkname
            tmpdict['temp_type'] = 'inertial'
            elementlist.append(tmpdict)
    # loop through the list of annotated elements and categorize the data
    for element in elementlist:
        delkeys = []
        for key in element.keys():
            if key.startswith('$'):
                category = key[1:]
                if category not in annotations:
                    annotations[category] = {}
                if element['temp_type'] not in annotations[category]:
                    annotations[category][element['temp_type']] = []
                tmpdict = {k: element[key][k] for k in element[key]}
                tmpdict['name'] = element['name']
                annotations[category][element['temp_type']].append(tmpdict)
                delkeys.append(key)
        delkeys.append('temp_type')
        print('element:', element)
        for key in delkeys:
            print(key)
            del element[key]
    #print('annotations:', annotations)
    #for category in annotations:
    #    for element in annotations[category]:
    #        del element['type']
    return annotations


def gatherCollisionBitmasks(model):
    """This function collects all collision bitmasks in a given model.

    :param model: The robot model to search in.
    :return: dict -- a dictionary containing all bitmasks with corresponding element name (key).

    """
    bitmasks = {}
    for linkname in model['links']:
        for elementname in model['links'][linkname]['collision']:
            element = model['links'][linkname]['collision'][elementname]
            if 'bitmask' in element:
                bitmask = {'name': elementname, 'link': linkname, 'bitmask': element['bitmask']}
                bitmasks[elementname] = bitmask
    return bitmasks


def writeURDFGeometry(output, element):
    """This functions writes the URDF geometry for a given element at the end of a given String.

    :param output: The String to append the URDF output string on.
    :type outpute: String.
    :param element: A certain element to parse into URDF.
    :type element: dict.
    :return: String -- The extended String

    """
    output.append(indent * 4 + '<geometry>\n')
    if element['type'] == 'box':
        output.append(xmlline(5, 'box', ['size'], [l2str(element['size'])]))
    elif element['type'] == "cylinder":
        output.append(xmlline(5, 'cylinder', ['radius', 'length'], [element['radius'], element['length']]))
    elif element['type'] == "sphere":
        output.append(xmlline(5, 'sphere', ['radius'], [element['radius']]))
    elif element['type'] in ['capsule', 'mesh']:  # capsules are not supported in URDF and are emulated using meshes
        output.append(xmlline(5, 'mesh', ['filename', 'scale'], [element['filename'], l2str(element['scale'])]))
    output.append(indent * 4 + '</geometry>\n')


def exportModelToURDF(model, filepath):
    """This functions writes the URDF of a given model into a file at the given filepath.
    All of the files content will be overwritten in this process.

    :param model: The model you want to convert into URDF.
    :type model: dict.
    :param filepath: The filepath you want to export the URDF to.
    :type filepath: String.
    :return: Nothing.

    """
    output = [xmlHeader, indent + '<robot name="' + model['modelname'] + '">\n\n']
    #export link information
    for l in model['links'].keys():
        link = model['links'][l]
        output.append(indent * 2 + '<link name="' + l + '">\n')
        if 'mass' in link['inertial'] and 'inertia' in link['inertial']:
            output.append(indent * 3 + '<inertial>\n')
            if 'pose' in link['inertial']:
                output.append(xmlline(4, 'origin', ['xyz', 'rpy'], [l2str(link['inertial']['pose']['translation']),
                                                                    l2str(link['inertial']['pose']['rotation_euler'])]))
            output.append(xmlline(4, 'mass', ['value'], [str(link['inertial']['mass'])]))
            output.append(xmlline(4, 'inertia', ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'],
                                  [str(i) for i in link['inertial']['inertia']]))
            output.append(indent * 3 + '</inertial>\n')
        #visual object
        if link['visual']:
            for v in link['visual']:
                vis = link['visual'][v]
                output.append(indent * 3 + '<visual name="' + vis['name'] + '">\n')
                output.append(xmlline(4, 'origin', ['xyz', 'rpy'],
                                      [l2str(vis['pose']['translation']), l2str(vis['pose']['rotation_euler'])]))
                writeURDFGeometry(output, vis['geometry'])
                if 'material' in vis:
                    if model['materials'][vis['material']][
                        'users'] == 0:  #FIXME: change back to 1 when implemented in urdfloader
                        mat = model['materials'][vis['material']]
                        output.append(indent * 4 + '<material name="' + mat['name'] + '">\n')
                        color = mat['diffuseColor']
                        output.append(
                            indent * 5 + '<color rgba="' + l2str([color[num] for num in ['r', 'g', 'b']]) + ' ' + str(
                                mat["transparency"]) + '"/>\n')
                        if 'texturename' in mat:
                            output.append(indent * 5 + '<texture filename="' + mat['texturename'] + '"/>\n')
                        output.append(indent * 4 + '</material>\n')
                    else:
                        output.append(indent * 4 + '<material name="' + vis["material"] + '"/>\n')
                output.append(indent * 3 + '</visual>\n')
        #collision object
        if link['collision']:
            for c in link['collision']:
                col = link['collision'][c]
                output.append(indent * 3 + '<collision name="' + col['name'] + '">\n')
                output.append(xmlline(4, 'origin', ['xyz', 'rpy'],
                                      [l2str(col['pose']['translation']), l2str(col['pose']['rotation_euler'])]))
                writeURDFGeometry(output, col['geometry'])
                output.append(indent * 3 + '</collision>\n')
        output.append(indent * 2 + '</link>\n\n')
    #export joint information
    missing_maxeffort = False
    for j in model['joints']:
        joint = model['joints'][j]
        output.append(indent * 2 + '<joint name="' + joint['name'] + '" type="' + joint["type"] + '">\n')
        child = model['links'][joint["child"]]
        output.append(xmlline(3, 'origin', ['xyz', 'rpy'],
                              [l2str(child['pose']['translation']), l2str(child['pose']['rotation_euler'])]))
        output.append(indent * 3 + '<parent link="' + joint["parent"] + '"/>\n')
        output.append(indent * 3 + '<child link="' + joint["child"] + '"/>\n')
        if 'axis' in joint:
            output.append(indent * 3 + '<axis xyz="' + l2str(joint['axis']) + '"/>\n')
        if 'limits' in joint:
            if 'effort' not in joint['limits']:
                print("\n###WARNING: joint '" + joint['name'] + "' does not specify a maximum effort!###")
                missing_maxeffort = True
            output.append(
                xmlline(3, 'limit', [p for p in joint['limits']], [joint['limits'][p] for p in joint['limits']]))
        output.append(indent * 2 + '</joint>\n\n')
    #export material information
    print("\n###WARNING: Created URDF is invalid due to missing values!###")
    for m in model['materials']:
        if model['materials'][m]['users'] > 0:  # FIXME: change back to 1 when implemented in urdfloader
            output.append(indent * 2 + '<material name="' + m + '">\n')
            color = model['materials'][m]['diffuseColor']
            transparency = model['materials'][m]['transparency'] if 'transparency' in model['materials'][m] else 0.0
            output.append(indent * 3 + '<color rgba="' + l2str([color[num] for num in ['r', 'g', 'b']]) + ' ' + str(
                1.0 - transparency) + '"/>\n')
            if 'texturename' in model['materials'][m]:
                output.append(indent * 3 + '<texture filename="' + model['materials'][m]['texturename'] + '"/>\n')
            output.append(indent * 2 + '</material>\n\n')
    #finish the export
    output.append(xmlFooter)
    with open(filepath, 'w') as outputfile:
        outputfile.write(''.join(output))
    # problem of different joint transformations needed for fixed joints
    print("phobos URDF export: Writing model data to", filepath)


def exportModelToSRDF(model, path):
    """This function exports the SRDF-relevant data from the dictionary to a specified path.
    Further detail on different elements of SRDF:

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

    <passive_joint>
    Marks a joint as passive.

    <disable_collisions>
    Disables collisions between pairs of links to simplify collision checking and avoid collisions
    of parents and children at their joints.


    Currently not supported:
    - <group_state>
    - <virtual_joint>

    :param model: a robot model dictionary.
    :param path: the outpath for the file.
    :return: Nothing.

    """
    output = []
    output.append(xmlHeader)
    output.append(indent + '<robot name="' + model['modelname'] + '">\n\n')
    for groupname in model['groups']:
        output.append(indent * 2 + '<group name="' + groupname + '">\n')
        for member in model['groups'][groupname]:
            output.append(indent * 3 + '<' + member['type'] + ' name="' + member['name'] + '" />\n')
        output.append(indent * 2 + '</group>\n\n')
    for chainname in model['chains']:
        output.append(indent * 2 + '<group name="' + chainname + '">\n')
        chain = model['chains'][chainname]
        output.append(indent * 3 + '<chain base_link="' + chain['start'] + '" tip_link="' + chain['end'] + '" />\n')
        output.append(indent * 2 + '</group>\n\n')
    #for joint in model['state']['joints']:
    #    pass
    #passive joints
    for joint in model['joints']:
        try:
            if model['joints'][joint]['passive']:
                output.append(indent * 2 + '<passive_joint name="' + model['links'][joint]['name'] + '"/>\n\n')
        except KeyError:
            pass
    for link in model['links']:
        if len(model['links'][link]['approxcollision']) > 0:
            output.append(indent * 2 + '<link_sphere_approximation link="' + model['links'][link]['name'] + '">\n')
            for sphere in model['links'][link]['approxcollision']:
                output.append(xmlline(3, 'sphere', ('center', 'radius'), (l2str(sphere['center']), sphere['radius'])))
            output.append(indent * 2 + '</link_sphere_approximation>\n\n')
        else:
            output.append(indent * 2 + '<link_sphere_approximation link="' + model['links'][link]['name'] + '">\n')
            output.append(xmlline(3, 'sphere', ('center', 'radius'), ('0.0 0.0 0.0', '0')))
            output.append(indent * 2 + '</link_sphere_approximation>\n\n')
    #calculate collision-exclusive links
    collisionExclusives = []
    for combination in itertools.combinations(model['links'], 2):
        link1 = model['links'][combination[0]]
        link2 = model['links'][combination[1]]
        # TODO: we might want to automatically add parent/child link combinations
        try:
            if link1['collision_bitmask'] & link2['collision_bitmask'] == 0:
                #output.append(xmlline(2, 'disable_collisions', ('link1', 'link2'), (link1['name'], link2['name'])))
                collisionExclusives.append((link1['name'], link2['name']))
        except KeyError:
            pass

    def addPCCombinations(parent):
        """Function to add parent/child link combinations for all parents an children that are not already set via collision bitmask.

        :param parent: This is the parent object.
        :type parent: dict.

        """
        children = getImmediateChildren(parent, 'link')
        if len(children) > 0:
            for child in children:
                #output.append(xmlline(2, 'disable_collisions', ('link1', 'link2'), (mother.name, child.name)))
                if ((parent, child) not in collisionExclusives) or ((child, parent) not in collisionExclusives):
                    collisionExclusives.append((parent.name, child.name))
                addPCCombinations(child)

    # FIXME: Do we need this?
    roots = getRoots()
    for root in roots:
        if root.name == 'root':
            addPCCombinations(root)

    for pair in collisionExclusives:
        output.append(xmlline(2, 'disable_collisions', ('link1', 'link2'), (pair[0], pair[1])))

    output.append('\n')
    #finish the export
    output.append(xmlFooter)
    with open(path, 'w') as outputfile:
        outputfile.write(''.join(output))
    # FIXME: problem of different joint transformations needed for fixed joints
    print("phobos SRDF export: Writing model data to", path)


def exportModelToSMURF(model, path):
    """This function exports a given model to a specific path as a smurf representation.

    :param model: The model you want to export.
    :type model: dict.
    :param path: The path you want to save the smurf file *without file name!*
    :type param: String.
    :return: Nothing.

    """

    bitmasks = gatherCollisionBitmasks(model)

    export = {'state': False,  # model['state'] != {}, # TODO: handle state
              'materials': model['materials'] != {},
              'sensors': model['sensors'] != {},
              'motors': model['motors'] != {},
              'controllers': model['controllers'] != {},
              'collision': bitmasks != {}
              }
    #create all filenames
    smurf_filename = model['modelname'] + ".smurf"
    urdf_filename = model['modelname'] + ".urdf"
    filenames = {'state': model['modelname'] + "_state.yml",
                 'materials': model['modelname'] + "_materials.yml",
                 'sensors': model['modelname'] + "_sensors.yml",
                 'motors': model['modelname'] + "_motors.yml",
                 'controllers': model['modelname'] + "_controllers.yml",
                 'collision': model['modelname'] + "_collision.yml",
                 }
    fileorder = ['collision', 'materials', 'motors', 'sensors', 'controllers', 'state']

    annotationdict = gatherAnnotations(model)
    for category in annotationdict:
        filenames[category] = model['modelname'] + '_' + category + '.yml'
        fileorder.append(category)
        export[category] = True

    infostring = ' definition SMURF file for "' + model['modelname'] + '", ' + model["date"] + "\n\n"

    #write model information
    print('Writing SMURF information to', smurf_filename)
    modeldata = {"date": model["date"], "files": [urdf_filename] + [filenames[f] for f in fileorder if export[f]]}
    with open(path + smurf_filename, 'w') as op:
        op.write('# main SMURF file of model "' + model['modelname'] + '"\n')
        op.write('# created with Phobos ' + defs.version + ' - https://github.com/rock-simulation/phobos\n\n')
        op.write("SMURF version: " + defs.version + "\n")
        op.write("modelname: " + model['modelname'] + "\n")
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
            op.write('#state' + infostring)
            op.write("modelname: " + model['modelname'] + '\n')
            op.write(yaml.dump(states))  #, default_flow_style=False))

    #write materials, sensors, motors & controllers
    for data in ['materials', 'sensors', 'motors', 'controllers']:
        if export[data]:
            with open(path + filenames[data], 'w') as op:
                op.write('#' + data + infostring)
                op.write(yaml.dump({data: list(model[data].values())}, default_flow_style=False))

    #write collision bitmask information
    if export['collision']:
        with open(path + filenames['collision'], 'w') as op:
            op.write('#collision data' + infostring)
            op.write(yaml.dump({'collision': list(bitmasks.values())}, default_flow_style=False))

    #write additional information
    for category in annotationdict.keys():
        if export[category]:
            outstring = '#' + category + infostring
            for elementtype in annotationdict[category]:
                outstring += elementtype + ':\n'
                outstring += yaml.dump(annotationdict[category][elementtype],
                                       default_flow_style=False) + "\n"
            with open(path + filenames[category], 'w') as op:
                op.write(outstring)


class ExportSceneOperator(Operator):
    """This Blender operator exports the selected robot models in the current
     Blender scene as a SMURF scene (*.smurfs).
    """
    bl_idname = "object.phobos_export_scene"
    bl_label = "Export the selected model(s) in a scene."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        exportSMURFsScene()
        return {'FINISHED'}


def exportSMURFsScene(selected_only=True, subfolders=True):
    """Exports all robots in a scene in *.smurfs format.
    :param selected_only: Decides if only models with selected root links are exported.
    :param subfolders: If True, the export is structured with subfolders for each model.
    """
    objects = {}
    models = {}  # models to be exported by name
    for root in getRoots():
        if ('modelname' in root) and (not (selected_only and not root.select)):
            objects[root['modelname']] = getChildren(root)
            if not root['modelname'] in models:
                models[root['modelname']] = [root]
            else:
                models[root['modelname']].append(root)

    entities = []
    for modelname in models:
        entitylist = models[modelname]
        unnamed_entities = 0
        for entity in entitylist:
            if 'entityname' in entity:
                entityname = entity['entityname']
            else:
                entityname = entity['modelname']+'_'+str(unnamed_entities)
                unnamed_entities += 1
            entitypose = robotdictionary.deriveObjectPose(entitylist[0])
            uri = os.path.join(modelname, modelname+'.smurf') if subfolders else modelname+'.smurf'
            scenedict = {'name': entityname,
                         'type': 'smurf',
                         'file': uri,
                         'anchor': root['anchor'] if 'anchor' in root else 'none',
                         'position': entitypose['translation'],
                         'rotation': entitypose['rotation_quaternion'],
                         'pose': 'default'}  # TODO: implement multiple poses
            entities.append(scenedict)

    if bpy.data.worlds[0].relativePath:
        outpath = securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"), bpy.data.worlds[0].path)))
    else:
        outpath = securepath(os.path.expanduser(bpy.data.worlds[0].path))

    with open(os.path.join(outpath, bpy.data.worlds['World'].sceneName + '.smurfs'),
              'w') as outputfile:
        outputfile.write("# SMURF scene: '" + bpy.data.worlds['World'].sceneName
                         + "'; created " + datetime.now().strftime("%Y%m%d_%H:%M") + "\n")
        outputfile.write("# created with Phobos " + defs.version
                         + " - https://github.com/rock-simulation/phobos\n\n")
        outputfile.write(yaml.dump({'entities': entities}))

    for modelname in models:
        smurf_outpath = securepath(os.path.join(outpath, modelname) if subfolders else outpath)
        selectObjects(objects[modelname], True)
        export(smurf_outpath)


def exportModelToMARS(model, path):
    """Exports selected robot as a MARS scene

    :param model: The robot model you want to export.
    :type model: dict.
    :param path: The path you want the MARS file to be located
    :type path: String.
    :return: Nothing.

    """

    mse.exportModelToMARS(model, path)

def determineMeshOutpath(obj, alternative: str, exporttype: str, path: str) -> str:
    """Determines the meshes filename for a specific object

    :param obj: The object you want to export the mesh from
    :param alternative: The alternativ name if the object has no geometry/sharedMesh tag
    :param exporttype: The filetype you want export to (without leading .)
    :param path: The path you want the mesh export to
    :return: str - Returns the filepath to export the mesh to
    """
    if "geometry/filename" in obj:
        return os.path.join(path, obj["geometry/filename"] + '.' + exporttype)
    else:
        return os.path.join(path, alternative) + '.' + exporttype



def securepath(path):  #TODO: this is totally not error-handled!
    """This function checks whether a path exists or not.
    If it doesn't the functions creates the path.

    :param path: The path you want to check for existence *DIRECTIONS ONLY*
    :type path: String.
    :return: String -- the path given as parameter, but secured by expanding ~ constructs.

    """

    if not os.path.exists(path):
        os.makedirs(path)
    return os.path.expanduser(path)


class ExportModelOperator(Operator):
    """This blender operator exports the robot model to chosen formats.
    You can choose one or more of the following file formats:
    - SMURF
    - SRDF
    - YAML
    - MARS

    """
    bl_idname = "object.phobos_export_robot"
    bl_label = "Export the selected model(s)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        export()
        return {'FINISHED'}


def export(path=''):
    """This function does the actual exporting of the robot model.

    :return: Nothing.
    """
    #TODO: check if all selected objects are on visible layers (option bpy.ops.object.select_all()?)
    if path == '':
        if bpy.data.worlds[0].relativePath:
            outpath = securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"), bpy.data.worlds[0].path)))
        else:
            outpath = securepath(os.path.expanduser(bpy.data.worlds[0].path))
    else:
        outpath = path
    if not outpath.endswith(os.path.sep):
        outpath += os.path.sep
    yaml = bpy.data.worlds[0].exportYAML
    urdf = bpy.data.worlds[0].exportURDF
    srdf = bpy.data.worlds[0].exportSRDF
    smurf = bpy.data.worlds[0].exportSMURF
    mars = bpy.data.worlds[0].exportMARSscene
    meshexp = bpy.data.worlds[0].exportMesh
    objexp = bpy.data.worlds[0].useObj
    bobjexp = bpy.data.worlds[0].useBobj
    stlexp = bpy.data.worlds[0].useStl
    daeexp = bpy.data.worlds[0].useDae
    objectlist = bpy.context.selected_objects

    if yaml or urdf or smurf or mars:
        robot = robotdictionary.buildRobotDictionary()
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
    selectObjects(objectlist, True)  # FIXME: does this make sense, as it is already the list of selected objects?
    if meshexp:
        show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269
        if show_progress:
            wm = bpy.context.window_manager
            total = float(len(objectlist))
            wm.progress_begin(0, total)
            i = 1
        for obj in bpy.context.selected_objects:
            if ((obj.phobostype == 'visual' or obj.phobostype == 'collision')
                and obj['geometry/type'] == 'mesh' and 'filename' not in obj and 'geometry/'+defs.reservedProperties['SHAREDMESH'] not in obj):
                if objexp:
                    exportObj(outpath, obj)
                if bobjexp:
                    exportBobj(outpath, obj)
                if stlexp:
                    exportStl(outpath, obj)
                if daeexp:
                    exportDae(outpath, obj)
            if show_progress:
                wm.progress_update(i)
                i += 1
        if show_progress:
            wm.progress_end()
