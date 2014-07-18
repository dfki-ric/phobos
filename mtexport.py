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
import sys
import mathutils
import os
from datetime import datetime
import yaml
import warnings
import struct
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, IntProperty
import marstools.mtdefs as mtdefs
from marstools.mtutility import *
import marstools.mtmarssceneexport as mtmse
import marstools.mtjoints as mtjoints
import marstools.mtmisctools as mtmisctools
import marstools.mtinertia as mtinertia

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

def collectMaterials(objectlist):
    print('\n\nCollecting materials...')
    materials = {}
    for obj in objectlist:
        if obj.MARStype == 'visual' and obj.data.materials:
            mat = obj.data.materials[0] #simply grab the first material
            if mat.name not in materials:
                materials[mat.name] = deriveMaterial(mat)
                materials[mat.name]['users'] = 1
            else:
                materials[mat.name]['users'] += 1
    return materials

def deriveMaterial(mat):
    material = {}
    material["name"] = mat.name
    material["diffuseFront"] = dict(zip(['r', 'g', 'b'], [mat.diffuse_intensity * num for num in list(mat.diffuse_color)]))
    material["specularFront"] = dict(zip(['r', 'g', 'b'], [mat.specular_intensity * num for num in list(mat.specular_color)]))
    material['shininess'] = mat.specular_hardness
    material["transparency"] = 1.0-mat.alpha
    if mat.texture_slots[0] is not None: # grab the first texture
        material["texturename"] = mat.texture_slots[0].texture.image.name
    return material

def deriveLink(obj):
    props = initObjectProperties(obj)
    props["pose"] = deriveObjectPose(obj)
    props["collision"] = {}
    props["visual"] = {}
    props["inertial"] = {}
    return props

def deriveJoint(obj):
    props = {'name': 'joint_' + obj.parent.name + '_' + obj.name}
    props['parent'] = obj.parent.name
    props['child'] = obj.name
    props['jointType'], crot = mtjoints.deriveJointType(obj, True)
    axis, limit = mtjoints.getJointConstraints(obj)
    if axis:
        props['axis'] = list(axis)
    if limit:
        props['limits'] = list(limit) # limit gets returned as None if there are no limits
    props["state"] = deriveJointState(obj)
    #TODO:
    # - calibration
    # - dynamics
    # - mimic
    # - safety_controller

    #derive motor data
    motorprops = {'name': 'motor_' + obj.name, 'joint': 'joint_' + obj.name}
    for key,value in obj.items():
        if key.find('motor/') >= 0:
            motorprops[key.replace('motor/', '')] = value
    return props, motorprops

def deriveJointState(joint):
    '''Calculates the state of a joint from the state of the link armature.
    Note that this is the current state and not the zero state.'''
    state = {}
    state['matrix'] = [list(vector) for vector in list(joint.pose.bones[0].matrix_basis)]
    state['translation'] = list(joint.pose.bones[0].matrix_basis.to_translation())
    state['rotation_euler'] = list(joint.pose.bones[0].matrix_basis.to_euler()) #[0:3]
    state['rotation_quaternion'] = list(joint.pose.bones[0].matrix_basis.to_quaternion()) #[0:4]
    # TODO: hard-coding this could prove problematic if we at some point build armatures from multiple bones
    return state

def deriveKinematics(obj):
    link = deriveLink(obj)
    joint = None
    motor = None
    if obj.parent:
        joint, motor = deriveJoint(obj)
    return link, joint, motor

#def deriveMotor(obj):
#    props = initObjectProperties(obj)
#    return props, obj.parent

def deriveGeometry(obj):
    if 'geometryType' in obj:
        geometry = {'geometryType': obj['geometryType']}
        gt = obj['geometryType']
        if gt == 'box':
            geometry['size'] = list(obj.dimensions)
        elif gt == 'cylinder':
            geometry["radius"] = obj.dimensions[0]/2
            geometry["height"] = obj.dimensions[2]
        elif gt == 'sphere':
            geometry['radius'] = obj.dimensions[0]/2
        elif gt == 'mesh':
            filename = obj['filename'] if 'filename' in obj else obj.name
            geometry['filename'] = filename + (".bobj" if bpy.context.scene.world.exportBobj else ".obj")
            geometry['scale'] = list(obj.scale)
            geometry['size'] = list(obj.dimensions) #this is needed to calculate an approximate inertia if collision is inertia
        return geometry
    else:
        warnings.warn("No geometryType found for object "+obj.name+".")
        return None

def deriveInertial(obj):
    '''Derives a dictionary entry of an inertial object. These settings override any 'mass' or 'inertia'
    settings placed in the object itself.'''
    props = initObjectProperties(obj)
    inertia = props['inertia'].split()
    props['inertia'] = list(map(float, inertia))
    props['pose'] = deriveObjectPose(obj)
    return props, obj.parent

def deriveObjectPose(obj):
    '''Derive pose of visual or collision object. This is at the moment the same
    implementation as the deriveLinkPose function and may be merged in the future.'''
    pose = {}
    pose['matrix'] = [list(vector) for vector in list(obj.matrix_local)]
    pose['translation'] = list(obj.matrix_local.to_translation())
    pose['rotation_euler'] = list(obj.matrix_local.to_euler())
    pose['rotation_quaternion'] = list(obj.matrix_local.to_quaternion())
    return pose

def deriveVisual(obj):
    visual = initObjectProperties(obj)
    visual['name'] = obj.name
    visual['pose'] = deriveObjectPose(obj)
    #if obj.data.materials:
    #    visual['material'] = deriveMaterial(obj.data.materials[0]) #this is now centralized!
    visual['geometry'] = deriveGeometry(obj)
    return visual, obj.parent

def deriveCollision(obj):
    collision = initObjectProperties(obj)
    collision['name'] = obj.name
    collision['geometry'] = deriveGeometry(obj)
    collision['pose'] = deriveObjectPose(obj)
    return collision, obj.parent

def deriveSensor(obj):
    props = initObjectProperties(obj)
    return props

def deriveController(obj):
    props = initObjectProperties(obj)

    return props
def initObjectProperties(obj):
    props = {}
    for key in obj.keys():
        props[key] = obj[key]
    props['name'] = obj.name
    return props

def cleanObjectProperties(props):
    '''Cleans a predefined list of Blender-specific properties from the dictionary.'''
    getridof = ['MARStype', '_RNA_UI', 'cycles_visibility', 'startChain', 'endChain', 'masschanged']
    if props:
        for key in getridof:
            if key in props:
                del props[key]
    return props

def deriveDictEntry(obj):
    print("MARStools: Deriving dictionary entry for", obj.MARStype + ': ' + obj.name, )
    props = None
    parent = None
    try:
        if obj.MARStype == 'inertial':
            props, parent = deriveInertial(obj)
        elif obj.MARStype == 'visual':
            props, parent = deriveVisual(obj)
        elif obj.MARStype == 'collision':
            props, parent = deriveCollision(obj)
        elif obj.MARStype == 'sensor':
            props = deriveSensor(obj)
        #elif obj.MARStype == 'motor':
        #    props, parent = deriveMotor(obj)
        elif obj.MARStype == 'controller':
            props = deriveController(obj)
    except KeyError:
        print('MARStools: A KeyError occurred, likely because there is missing information in the model:\n    ', sys.exc_info()[0])
    if obj.MARStype in ['sensor', 'motor' 'controller']:
        return cleanObjectProperties(props)
    else:
        return cleanObjectProperties(props), parent

def deriveGroupEntry(group):
    return [obj.name for obj in group.objects]

def deriveChainEntry(obj):
    returnchains = []
    if 'endChain' in obj:
        endChain = obj['endChain'].split()
    for chainName in endChain:
        chainclosed = False
        parent = obj
        chain = {'name': chainName, 'start': '', 'end': obj.name, 'elements': []}
        while not chainclosed:
            if parent.parent == None:
                print('### Error: Unclosed chain, aborting parsing chain', chainName)
                chain = None
                break
            chain['elements'].append(parent.name)
            parent = parent.parent
            if 'startChain' in parent:
                startChain = parent['startChain'].split()
                if chainName in startChain:
                    chain['start'] = parent.name
                    chain['elements'].append(parent.name)
                    chainclosed = True
        if chain is not None:
            returnchains.append(chain)
    return returnchains

def buildRobotDictionary():
    objectlist = bpy.context.selected_objects
    #notifications, faulty_objects = mtmisctools.checkModel(bpy.context.selected_objects)
    #print(notifications)
    robot = {'links': {},
            'joints': {},
            'sensors': {},
            'motors': {},
            'controllers': {},
            'materials': {},
            'groups': {},
            'chains': {},
            'simulation': {}
            }
    #save timestamped version of model
    robot["date"] = datetime.now().strftime("%Y%m%d_%H:%M")
    root = getRoot(bpy.context.selected_objects[0])
    if root.MARStype != 'link':
        raise Exception("Found no 'link' object as root of the robot model.")
    else:
        if 'modelname' in root:
            robot['modelname'] = root["modelname"]
        else:
            robot['modelname'] = 'unnamed_robot'
    # now parse the scene
    # digest all the links to derive link and joint information
    for obj in bpy.context.selected_objects:
        if obj.MARStype == 'link':
            print('Parsing', obj.MARStype, obj.name, '...')
            link, joint, motor = deriveKinematics(obj)
            robot['links'][obj.name] = link
            if joint: #joint can be None if link is a root
                robot['joints'][joint['name']] = joint
            if motor:
                robot['motors'][joint['name']] = motor
            obj.select = False
    # complete link information by parsing visuals and collision objects
    #try:
    for obj in bpy.context.selected_objects:
        if obj.MARStype == 'inertial':
            props, parent = deriveDictEntry(obj)
            robot['links'][parent.name][obj.MARStype] = props
            obj.select = False
        if obj.MARStype in ['visual', 'collision']:
            props, parent = deriveDictEntry(obj)
            robot['links'][parent.name][obj.MARStype][obj.name] = props
            obj.select = False
    # TODO: change this to not solely depend on the last collision object
    # recalculate inertial from collision geometry if necessary
    print('\n\n[MARStools] Calculating inertia from collision and visual objects...')
    for linkname in robot['links']:
        link = robot['links'][linkname]
        print('Checking inertial for link', link['name'])
        if 'mass' in link['inertial'] and 'inertia' not in link['inertial']:
                print('Found mass, but no inertia...trying to derive from collision...')
                for c in link['collision']:
                    col = link['collision'][c]
                    if 'geometry' in col:
                        link['inertial']['inertia'] = mtinertia.calculateInertia(link['inertial']['mass'], col['geometry'])
                        link['inertial']['pose'] = col['pose']
                #TODO: check if this really makes sense or if we should not export anything special to let ODE handle the job!!!
                print('...failed. Inserting dummy inertia...')
                link['inertial']['inertia'] = mtinertia.getDummyInertia()
        else:
            print("###WARNING: link", link['name'], "has no mass.")
    # parse motors, sensors and controllers
    for obj in bpy.context.selected_objects:
        if obj.MARStype in ['sensor', 'controller']:
            print('Parsing', obj.MARStype, obj.name, '...')
            robot[obj.MARStype+'s'][obj.name] = deriveDictEntry(obj)
            obj.select = False
    #except (KeyError, TypeError):
    #    print("An error occurred when inserting the entry for object", obj.name, sys.exc_info()[0], sys.exc_info()[1], sys.exc_info()[2])
    #    print(robot)
    # parse materials
    robot['materials'] = collectMaterials(objectlist)
    for obj in objectlist:
        if obj.MARStype == 'visual' and len(obj.data.materials) > 0:
            mat = obj.data.materials[0]
            if not mat.name in robot['materials']:
                robot['materials'][mat.name] = deriveMaterial(mat) #this should actually never happen
            robot['links'][obj.parent.name]['visual'][obj.name]['material'] = mat.name
    # gather information on groups of objects
    for group in bpy.data.groups: #TODO: get rid of the "data" part
        robot['groups'][group.name] = deriveGroupEntry(group)
    # gather information on chains of objects
    chains = []
    for obj in bpy.data.objects:
        if obj.MARStype == 'link' and 'endChain' in obj:
            chains.extend(deriveChainEntry(obj))
    for chain in chains:
        robot['chains'][chain['name']] = chain
    epsilon = 10**(-bpy.data.worlds[0].decimalPlaces) #TODO: implement this separately
    return epsilonToZero(robot, epsilon, bpy.data.worlds[0].decimalPlaces)

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
        output.append(indent*3+'<inertial>\n')
        if 'pose' in link['inertial']:
            output.append(xmlline(4, 'origin', ['xyz', 'rpy'], [l2str(link['inertial']['pose']['translation']), l2str(link['inertial']['pose']['rotation_euler'])]))
        output.append(xmlline(4, 'mass', ['value'], [str(link['inertial']['mass'])]))
        if 'inertia' in link['inertial']:
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
            output.append(indent*3+'<color rgba="'+l2str([color[num] for num in ['r', 'g', 'b']]) + ' ' + str(model['materials'][m]["transparency"]) + '"/>\n')
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
        with open(path + filenames['simulation'], 'w') as op:
            op.write('#simulation'+infostring)
            op.write("modelname: "+model['modelname']+'\n')
            #TODO: handle simulation-specific data
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
        robot = buildRobotDictionary()
        if yaml:
            exportModelToYAML(robot, outpath + robot["modelname"] + "_dict.yml")
        if mars:
            exportModelToMARS(robot, outpath + robot["modelname"] + "_mars.scn")
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
