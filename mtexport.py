'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtexport.py

Created on 13 Feb 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.

NOTE: If you edit this script, please make sure not to use any imports
not supported by Blender's standard Python distribution. This is a script
intended to be usable on its own and thus should not use external dependencies,
especially none of the other modules of the MARStools package.
'''

import bpy
import mathutils
import os
from datetime import datetime
import yaml
import struct
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, IntProperty
from marstools.mtutility import *
import marstools.mtdefs as mtdefs
import marstools.mtmarssceneexport as mtmse
import marstools.mtinertia as mtinertia
import marstools.mtrobotdictionary as mtrobotdictionary

def register():
    print("Registering mtexport...")

def unregister():
    print("Unregistering mtexport...")

indent = '  '
urdfHeader = '<?xml version="1.0"?>\n'
urdfFooter = indent+'</robot>\n'

def exportBobj(path, obj):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select = True#
    bpy.context.scene.objects.active = obj
    #TODO: make this exception-handled
    totverts = totuvco = totno = 1

    globalNormals = {}

    # ignore dupli children
    if obj.parent and obj.parent.dupli_type in {'VERTS', 'FACES'}:
        # XXX
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
                except:
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
                        out.write(struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], globalNormals[veckey3d(v.normal)]))
                else:  # No smoothing, face normals
                    no = globalNormals[roundVector(f.normal, 6)]
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], no))
            else:  # No UV's
                if f_smooth:  # Smoothed, use vertex normals
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, 0, globalNormals[roundVector(f.normal, 6)]))
                else:  # No smoothing, face normals
                    no = globalNormals[roundVector(f.normal, 6)]
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, 0, no))
    out.close()

def exportObj(path, obj):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select = True
    outpath = os.path.join(path, obj.name) + '.obj'
    world_matrix = obj.matrix_world.copy()
    obj.matrix_world = mathutils.Matrix.Identity(4)
    bpy.ops.export_scene.obj(filepath=outpath, axis_forward='-Z',
                             axis_up='Y', use_selection=True, use_normals=True)
    obj.matrix_world = world_matrix

def exportModelToYAML(model, filepath):
    print("MARStools YAML export: Writing model data to", filepath )
    with open(filepath, 'w') as outputfile:
        outputfile.write('#YAML dump of robot model "'+model['modelname']+'", '+datetime.now().strftime("%Y%m%d_%H:%M")+"\n\n")
        outputfile.write(yaml.dump(model))#, default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries

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

def writeURDFGeometry(output, element):
    output.append(indent*4+'<geometry>\n')
    if element['geometryType'] == 'box':
        output.append(xmlline(5, 'box', ['size'], [l2str(element['size'])]))
    elif element['geometryType'] == "cylinder":
        output.append(xmlline(5, 'cylinder', ['radius', 'length'], [element['radius'], element['height']]))
    elif element['geometryType'] == "sphere":
        output.append(xmlline(5, 'sphere', ['radius'], [element['radius']]))
    elif element['geometryType'] == "mesh":
        output.append(xmlline(5, 'mesh', ['filename', 'scale'], [element['filename'], '1.0 1.0 1.0']))#TODO correct this after implementing scale properly
    output.append(indent*4+'</geometry>\n')

def exportModelToURDF(model, filepath):
    output = []
    output.append(urdfHeader)
    output.append(indent+'<robot name="'+model['modelname']+'">\n\n')
    #export link information
    for l in model['links'].keys():
        link = model['links'][l]
        output.append(indent*2+'<link name="'+l+'">\n')
        if link['inertial'] != {} and 'mass' in link['inertial'] and 'inertia' in link['inertial']:
            output.append(indent*3+'<inertial>\n')
            if 'pose' in link['inertial']:
                output.append(xmlline(4, 'origin', ['xyz', 'rpy'], [l2str(link['inertial']['pose']['translation']), l2str(link['inertial']['pose']['rotation_euler'])]))
            output.append(xmlline(4, 'mass', ['value'], [str(link['inertial']['mass'])]))
            output.append(xmlline(4, 'inertia', ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'], link['inertial']['inertia']))
            output.append(indent*3+'</inertial>\n')
        #visual object
        if link['visual']:
            for v in link['visual']:
                vis = link['visual'][v]
                output.append(indent*3+'<visual name="' + vis['name'] + '">\n')
                output.append(xmlline(4, 'origin', ['xyz', 'rpy'], [l2str(vis['pose']['translation']), l2str(vis['pose']['rotation_euler'])]))
                writeURDFGeometry(output, vis['geometry'])
                if 'material' in vis:
                    if model['materials'][vis['material']]['users'] == 1:
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
        output.append(indent*2+'<joint name="'+joint['name']+'" type="'+joint["jointType"]+'">\n')
        child = model['links'][joint["child"]]
        output.append(xmlline(3, 'origin', ['xyz', 'rpy'], [l2str(child['pose']['translation']), l2str(child['pose']['rotation_euler'])]))
        output.append(indent*3+'<parent link="'+joint["parent"]+'"/>\n')
        output.append(indent*3+'<child link="'+joint["child"]+'"/>\n')
        if 'axis' in joint:
            output.append(indent*3+'<axis xyz="'+l2str(joint['axis'])+'"/>\n')
        if 'limits' in joint:
            output.append(xmlline(3, 'limit', ['lower', 'upper', 'velocity', 'effort'], [str(joint['limits'][0]), str(joint['limits'][1]), 1.0, 1.0])) #TODO: effort and velocity
        output.append(indent*2+'</joint>\n\n')
    #export material information
    for m in model['materials']:
        if model['materials'][m]['users'] > 1:
            output.append(indent*2+'<material name="' + m + '">\n')
            color = model['materials'][m]['diffuseFront']
            transparency = model['materials'][m]['transparency'] if 'transparency' in model['materials'][m] else 0.0
            output.append(indent*3+'<color rgba="'+l2str([color[num] for num in ['r', 'g', 'b']]) + ' ' + str(transparency) + '"/>\n')
            if 'texturename' in model['materials'][m]:
                            output.append(indent*3+'<texture filename="'+model['materials'][m]['texturename']+'"/>\n')
            output.append(indent*2+'</material>\n\n')
    #finish the export
    output.append(urdfFooter)
    with open(filepath, 'w') as outputfile:
        outputfile.write(''.join(output))
    # problem of different joint transformations needed for fixed joints
    print("MARStools URDF export: Writing model data to", filepath )

def exportModelToSMURF(model, path):
    export = {'semantics': model['groups'] != {} or model['chains'] != {},
              'state': False,#model['state'] != {}, #TODO: handle state
              'materials': model['materials'] != {},
              'sensors': model['sensors'] != {},
              'motors': model['motors'] != {},
              'controllers': model['controllers'] != {},
              'simulation': model['simulation'] != {}
              }


    #create all filenames
    smurf_filename = model['modelname'] + ".smurf"
    urdf_filename =  model['modelname'] + ".urdf"
    filenames = {'semantics': model['modelname'] + "_semantics.yml",
                 'state': model['modelname'] + "_state.yml",
                 'materials': model['modelname'] + "_materials.yml",
                 'sensors': model['modelname'] + "_sensors.yml",
                 'motors': model['modelname'] + "_motors.yml",
                 'controllers': model['modelname'] + "_controllers.yml",
                 'simulation': model['modelname'] + "_simulation.yml"
                 }

    infostring = ' definition SMURF file for "'+model['modelname']+'", '+model["date"]+"\n\n"

    #write model information
    print('Writing SMURF information to...\n'+smurf_filename)
    modeldata = {}
    modeldata["date"] = model["date"]
    modeldata["files"] = [urdf_filename] + [filenames[f] for f in filenames if export[f]]
    with open(path + smurf_filename, 'w') as op:
        op.write('#main SMURF file of model "'+model['modelname']+'"\n\n')
        op.write("modelname: "+model['modelname']+"\n")
        op.write(yaml.dump(modeldata, default_flow_style=False))

    #write urdf
    exportModelToURDF(model, path + urdf_filename)

    #write semantics (SRDF information in YML format)
    if export['semantics']:
        with open(path + filenames['semantics'], 'w') as op:
            op.write('#semantics'+infostring)
            op.write("modelname: "+model['modelname']+'\n')
            semantics = {}
            if model['groups'] != {}:
                semantics['groups'] = model['groups']
            if model['chains'] != {}:
                semantics['chains'] = model['chains']
            op.write(yaml.dump(semantics, default_flow_style=False))

    #write state (state information of all joints, sensor & motor activity etc.) #TODO: implement everything but joints
    if export['state']:
        states = []
        #gather all states
        for jointname in model['joints']:
            joint = model['joints'][jointname]
            if 'state' in joint: #this should always be the case, but testing doesn't hurt
                tmpstate = joint['state'].copy()
                tmpstate['name'] = jointname
                states.append(joint['state'])
        with open(path + filenames['state'], 'w') as op:
            op.write('#state'+infostring)
            op.write("modelname: "+model['modelname']+'\n')
            op.write(yaml.dump(states))#, default_flow_style=False))

    #write materials
    if export['materials']:
        with open(path + filenames['materials'], 'w') as op:
            op.write('#materials'+infostring)
            op.write(yaml.dump({'materials': list(model['materials'].values())}, default_flow_style=False))

    #write sensors
    if export['sensors']:
        with open(path + filenames['sensors'], 'w') as op:
            op.write('#sensors'+infostring)
            op.write("modelname: "+model['modelname']+'\n')
            op.write(yaml.dump(list(model['sensors'].values()), default_flow_style=False))

    #write motors
    if export['motors']:
        with open(path + filenames['motors'], 'w') as op:
            op.write('#motors'+infostring)
            op.write("modelname: "+model['modelname']+'\n')
            #op.write("motors:\n")
            op.write(yaml.dump({'motors': list(model['motors'].values())}, default_flow_style=False))

    #write controllers
    if export['controllers']:
        with open(path + filenames['controllers'], 'w') as op:
            op.write('#controllers'+infostring)
            op.write("modelname: "+model['modelname']+'\n')
            op.write(yaml.dump(list(model['controllers'].values()), default_flow_style=False))

    #write simulation
    if export['simulation']:
        nodes = {}
        for link in model['links']:
            for objtype in ['visual', 'collision']:
                for objname in model['links'][link][objtype]:
                    obj = model['links'][link]['visual'][objname]
                    if 'mass' in obj:
                        nodes[obj['name']] = {'mass': obj['mass']}
        with open(path + filenames['simulation'], 'w') as op:
            op.write('#simulation'+infostring)
            op.write("modelname: "+model['modelname']+'\n')
            #TODO: handle simulation-specific data
            op.write(yaml.dump(nodes, default_flow_style=False))
            op.write(yaml.dump(list(model['simulation'].values()), default_flow_style=False))

def exportSceneToSMURF(path):
    """Exports all robots in a scene to separate SMURF folders."""
    pass

def exportModelToMARS(model, path):
    """Exports selected robot as a MARS scene"""
    mtmse.exportModelToMARS(model, path)

def securepath(path): #TODO: this is totally not error-handled!
    if not os.path.exists(path):
        os.makedirs(path)
    return os.path.expanduser(path)

class ExportModelOperator(Operator):
    """ExportModelOperator"""
    bl_idname = "object.mt_export_robot"
    bl_label = "Export the selected model(s)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        #TODO: check if all selected objects are on visible layers (option bpy.ops.object.select_all()?)
        if bpy.data.worlds[0].relativePath:
           path = securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"), bpy.context.scene.world.path)))
        else:
            path = securepath(os.path.expanduser(bpy.context.scene.world.path))
        main(outpath = path,
                yaml = bpy.data.worlds[0].exportYAML,
                urdf = bpy.data.worlds[0].exportURDF,
                smurf = bpy.data.worlds[0].exportSMURF,
                mars = bpy.data.worlds[0].exportMARSscene,
                objexp = bpy.data.worlds[0].exportObj,
                bobjexp = bpy.data.worlds[0].exportBobj)
        return {'FINISHED'}

def main(outpath = '', yaml=True, urdf=True, smurf=True, mars=False, objexp=False, bobjexp=False):
    objectlist = bpy.context.selected_objects
    if yaml or urdf or smurf or mars:
        robot = mtrobotdictionary.buildRobotDictionary()
        if yaml:
            exportModelToYAML(robot, outpath + robot["modelname"] + "_dict.yml")
        if mars:
            exportModelToMARS(robot, outpath + robot["modelname"] + "_mars.scene")
        if smurf:
            exportModelToSMURF(robot, outpath)
        elif urdf:
            exportModelToURDF(robot, outpath + robot["modelname"] + ".urdf")
    selectObjects(objectlist, True)
    if objexp or bobjexp:
        show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269;
        if show_progress:
            wm = bpy.context.window_manager
            total = float(len(objectlist))
            wm.progress_begin(0, total)
            i = 1
        for obj in bpy.context.selected_objects:
            if ((obj.MARStype == 'visual' or
                obj.MARStype == 'collision') and obj['geometryType'] == 'mesh'):
                if objexp:
                    exportObj(outpath, obj)
                if bobjexp:
                    exportBobj(outpath, obj)
            if show_progress:
                wm.progress_update(i)
                i += 1
        if show_progress:
            wm.progress_end()

#allow manual execution of script in blender
if __name__ == '__main__':
    main()
