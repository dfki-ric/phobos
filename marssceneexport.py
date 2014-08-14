'''
Phobos - a Blender Add-On to work with MARS robot models

File marssceneexporter.py

Created on 7 Jan 2014

@author: Malte Langosz, Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
'''


# ******** TODO:
#               - add jointOffset calculation by using rotation difference
#                 on joint axis of node2

import bpy
import os
import mathutils

class IdGenerator(object):
    def __init__(self, initValue=0):
        self._nextId = initValue
    def __call__(self):
        ret, self._nextId = self._nextId, self._nextId + 1
        return ret

def getID(name):
    for obj in bpy.data.objects:
        if obj.name == name:
            return obj["id"] if "id" in obj else 0
    return 0

def createNodes(model):
    nodes = []
    groupid = 0
    for l in model['links']:
        link = model['links'][l]
        groupid += 1
        relativeid = link['id']
        node = {'type': 'link',
                'obj': link,
                'id': link['id'],
                'groupid': str(groupid),
                'noPhysical': 'true',
                'name': l,
                'origname': 'box',
                'physicmode': 'box',
                'filename': 'PRIMITIVE'}
        if bpy.data.objects[l].parent:
            node['relativeid'] = model['links'][bpy.data.objects[l].parent.name]['id']
        link = model['links'][l]
        node['mass'] = '0.001' #FUXME: this needs to be implemented via inertial
        nodes.append(node)
        for v in link['visual']:
            vis = link['visual'][v]
            node = {'type': 'visual',
                    'obj': vis,
                    'id': vis['id'],
                    'groupid': str(groupid),
                    'relativeid': relativeid,
                    'noPhysical': 'true',
                    'physicmode': 'box',
                    'name': vis['name'],
                    'origname': vis['name'],
                    'collision_bitmask': str(0)}
            if vis['geometry']['geometryType'] == 'mesh':
                node['filename'] = vis['geometry']['filename']
            node['material_id'] = model['materials'][vis['material']]['id']
            node['mass'] = '0.001' #FUXME: this needs to be implemented via inertial
            nodes.append(node)
        #for c in link['collision']:
        #    col = link['collision'][c]
        #    id += 1
        #    node = {'id': str(id),
        #            'groupid': str(groupid),
        #            'relativeid': relativeid,
        #            'name': vis['name'],
        #            'physicMode': col['geometry']['geometryType'],
        #            'collision_bitmask': col['collision_bitmask'] if "coll_bitmask" in col else '65535'}
        #    if 'radius' in col['geometry']:
        #        node['radius': str(col['geometry']['radius'])]
        #    if 'height' in col['geometry']:
        #        node['radius': str(col['geometry']['height'])]
        #    node['matID'] = model['materials'][vis['material']['name']]['id']
        #    nodes.append(node)
    return nodes

def writeNode(node):
    out = []
    out.append('    <node name="' + node['name'] + '">')
    proplist = ['origname', 'filename', 'index', 'groupid', 'relativeid', 'physicmode',
                'noPhysical', 'material_id', 'collision_bitmask', 'mass']
    #TODO: movable
    for p in proplist:
        if p in node:
            out.append('      <'+p+'>' + str(node[p]) + '</'+p+'>')

    obj = node['obj']
    #if 'geometry' in node['obj']:
    #    geom = node['obj']['geometry']
    #    ext = [geom['size'][i] * geom['scale'][i] for i in range(3)]
    #    radius = geom['radius'] if 'radius' in geom else 0
    #    height = geom['height'] if 'height' in geom else 0
    #    ext[0] = radius if radius > 0. else geom['scale'][0]
    #    ext[1] = height if height > 0. else geom['scale'][1]
    #    pivot = utility.calcBoundingBoxCenter(obj.bound_box)
    #    parent = obj.parent
    #    parentPos = parent.matrix_world * utility.calcBoundingBoxCenter(parent.bound_box)
    #    childPos = obj.matrix_world * pivot
    #    childPos = childPos - parentPos
    #   center = parent.matrix_world.to_quaternion().inverted() * childPos
    #   childRot = obj.matrix_local.to_quaternion()
    #else:
    #    center = obj['pose']['translation']
    #    childRot = obj['pose']['rotation_quaternion']

    outputVector(out, "position", obj['pose']['translation'], 6)
    outputQuaternion(out, "rotation", obj['pose']['rotation_quaternion'], 6)
    if node['type'] in ['visual', 'collision']:
        if 'size' in obj['geometry']:
            outputVector(out, "extend", obj['geometry']['size'], 6)
        elif 'radius' in obj['geometry']:
            outputVector(out, "extend", [obj['geometry']['radius']], 6)
    else:
        outputVector(out, "extend", [0.1, 0.1, 0.1], 6)
    out.append('    </node>')
    return out


def outputVector(out, name, vec, indentLevel):
    indent = " " * indentLevel
    out.append(indent + '<'+name+'>')
    out.append(indent + '  <x>'+str(vec[0])+'</x>')
    out.append(indent + '  <y>'+str(vec[1])+'</y>')
    out.append(indent + '  <z>'+str(vec[2])+'</z>')
    out.append(indent + '</'+name+'>')

def outputQuaternion(out, name, q, indentLevel):
    indent = " " * indentLevel
    out.append(indent + '<'+name+'>')
    out.append(indent + '  <x>'+str(q[1])+'</x>')
    out.append(indent + '  <y>'+str(q[2])+'</y>')
    out.append(indent + '  <z>'+str(q[3])+'</z>')
    out.append(indent + '  <w>'+str(q[0])+'</w>')
    out.append(indent + '</'+name+'>')

# def createJoints(model):
#     joints = []
#     id = 0
#     for j in model['joints']:
#         id += 1
#         jo = model['joints'][j]
#         node = {'obj': jo,
#                 'id': str(id),
#                 'type': jo['jointType'],
#                 'name': j,
#                 }
#
# def writeJoint(joint):
#
#     #jointType = {"hinge": 1, "fixed": 6, "slider": 3}[joint["jointType"]]
#     jointType = joint["jointType"]
#     anchorPos = {"custom": 4, "node1": 1, "node2": 2, "center": 3}[joint["anchor"]]
#
#     out = []
#     out.append('    <joint name="'+joint['name']+'">')
#     out.append('      <index>'+joint['id']+'</index>')
#     out.append('      <type>'+joint['type']+'</type>')
#     out.append('      <nodeindex1>'+str(joint.parent["id"])+'</nodeindex1>')
#     out.append('      <nodeindex2>'+str(node2ID)+'</nodeindex2>')
#     out.append('      <anchorpos>'+str(anchorPos)+'</anchorpos>')
#     outputVector(out, "anchor", center, 6)
#     outputVector(out, "axis1", (invert*axis[0],invert*axis[1],invert*axis[2]), 6)
#     if "axis2x" in joint:
#         outputVector(out, "axis2", (joint["axis2x"], joint["axis2y"], joint["axis2z"]), 6)
#     if "lowStop" in joint:
#         out.append('      <lowStopAxis1>'+str(joint["lowStop"])+'</lowStopAxis1>')
#     if "highStop" in joint:
#         out.append('      <highStopAxis1>'+str(joint["highStop"])+'</highStopAxis1>')
#     if "springConst" in joint:
#         out.append('      <spring_const_constraint_axis1>'+str(joint["springConst"])+'</spring_const_constraint_axis1>')
#     if "dampingConst" in joint:
#         out.append('      <damping_const_constraint_axis1>'+str(joint["dampingConst"])+'</damping_const_constraint_axis1>')
#     out.append('      <angle1_offset>'+str(invert*jointOffset)+'</angle1_offset>')
#     out.append('    </joint>')
#     return out #jointOffset*invert

# axis = joint.matrix_world.to_quaternion() * mathutils.Vector((0.0, 0.0, 1.0))
#     center = joint.matrix_world.to_translation()
#     if joint["node2"] in bpy.data.objects:
#         node2 = bpy.data.objects[joint["node2"]]
#         node2ID = node2["id"]
#     else:
#         node2 = None
#         node2ID = 0
#
#     if "invertAxis" in joint and joint["invertAxis"] == 1:
#         invert = -1
#     else:
#         invert = 1
#
#     if "jointOffset" in joint:
#         jointOffset = joint["jointOffset"]
#     elif node2:
#         joint.rotation_mode = 'QUATERNION'
#         v1 = joint.rotation_quaternion * mathutils.Vector((1.0, 0.0, 0.0))
#         v2 = node2.rotation_quaternion * mathutils.Vector((1.0, 0.0, 0.0))
#         jointOffset = v1.angle(v2, 0.0)
#         q = joint.rotation_quaternion.copy().inverted()
#         axis_ = q * v1.cross(v2)
#         if axis_[2] > 0:
#             jointOffset *= -1
#     else:
#         jointOffset = 0.0


# def writeMotor(joint, motorValue):
#     motor_type = joint["motor_type"] if "motor_type" in joint else 1
#     motor_axis = joint["motorAxis"] if "motorAxis" in joint else 1
#     motor_p = joint["p"] if "p" in joint else defValues["defP"]
#     motor_i = joint["i"] if "i" in joint else defValues["defI"]
#     motor_d = joint["d"] if "d" in joint else defValues["defD"]
#     low_stop = joint["lowStop"] if "lowStop" in joint else -6.28
#     high_stop = joint["highStop"] if "highStop" in joint else 6.28
#     max_speed = joint["maxSpeed"] if "maxSpeed" in joint else defValues["defMaxMotorSpeed"]
#     max_force = joint["maxForce"] if "maxForce" in joint else defValues["defMaxMotorForce"]
#
#     out = []
#     out.append('    <motor name="'+joint.name+'">')
#     out.append('      <index>'+str(joint["id"])+'</index>')
#     out.append('      <jointIndex>'+str(joint["id"])+'</jointIndex>')
#     out.append('      <axis>'+str(motor_axis)+'</axis>')
#     out.append('      <maximumVelocity>'+str(max_speed)+'</maximumVelocity>')
#     out.append('      <motorMaxForce>'+str(max_force)+'</motorMaxForce>')
#     out.append('      <type>'+str(motor_type)+'</type>')
#     out.append('      <p>'+str(motor_p)+'</p>')
#     out.append('      <i>'+str(motor_i)+'</i>')
#     out.append('      <d>'+str(motor_d)+'</d>')
#     out.append('      <min_val>'+str(low_stop)+'</min_val>')
#     out.append('      <max_val>'+str(high_stop)+'</max_val>')
#     out.append('      <value>'+str(motorValue)+'</value>')
#     out.append('    </motor>')
#     return out
#
# def writeSensor(sensor, out):
#     sensorType = sensor["sensorType"] if "sensorType" in sensor else "unknown"
#     rate = sensor["rate"] if "rate" in sensor else 10.0
#     idList = {}
#
#     if "listMotors" in sensor:
#         idList = myMotorList
#     elif "listNodes" in sensor:
#         for i, obj in enumerate(objList):
#             idList[i] = obj["id"]
#     for key, value in sensor.items():
#         if key[:5] == "index":
#             index = getID(value)
#             if index != 0:
#                 idList[int(key[5:])] = index
#     out.append('    <sensor name="'+sensor.name+'" type="'+str(sensorType)+'">')
#     out.append('      <index>'+str(sensor["id"])+'</index>')
#     out.append('      <rate>'+str(rate)+'</rate>')
#     for value in idList.values():
#         out.append('      <id>'+str(value)+'</id>')
#     if sensorType == "Joint6DOF":
#         nodeID = getID(sensor["nodeID"])
#         jointID = getID(sensor["jointID"])
#         out.append('      <nodeID>'+str(nodeID)+'</nodeID>')
#         out.append('      <jointID>'+str(jointID)+'</jointID>')
#     elif sensorType == "RaySensor":
#         nodeID = getID(sensor["attached_node"])
#         out.append('      <attached_node>'+str(nodeID)+'</attached_node>')
#         out.append('      <width>'+str(sensor["width"])+'</width>')
#         out.append('      <opening_width>'+str(sensor["opening_width"])+
#                   '</opening_width>')
#         out.append('      <max_distance>'+str(sensor["max_distance"])+
#                   '</max_distance>')
#         if "draw_rays" in sensor:
#             out.append('      <draw_rays>'+str(sensor["draw_rays"])+'</draw_rays>')
#     elif sensorType == "CameraSensor":
#         nodeID = getID(sensor["attached_node"])
#         out.append('      <attached_node>'+str(nodeID)+'</attached_node>')
#         out.append('      <depth_image>'+str(sensor["depth_image"])+'</depth_image>')
#         out.append('      <show_cam hud_idx="'+str(sensor["hud_idx"])+'">'+str(sensor["show_cam"])+'</show_cam>')
#         out.append('      <position_offset x="'+str(sensor["position_offset_x"])+
#                 '" y="'+str(sensor["position_offset_y"])+
#                 '" z="'+str(sensor["position_offset_z"])+'"/>')
#         out.append('      <orientation_offset yaw="'+str(sensor["orientation_offset_yaw"])+
#                 '" pitch="'+str(sensor["orientation_offset_pitch"])+
#                 '" roll="'+str(sensor["orientation_offset_roll"])+'"/>')
#     out.append('    </sensor>')


def writeMaterial(material):
    out = []
    out.append('    <material>')
    out.append('      <id>'+material['id']+'</id>')
    out.append('      <diffuseFront>');
    out.append('        <a>'+str(1-material['transparency'])+'</a>');
    out.append('        <r>'+str(material['diffuseFront']['r'])+'</r>');
    out.append('        <g>'+str(material['diffuseFront']['g'])+'</g>');
    out.append('        <b>'+str(material['diffuseFront']['b'])+'</b>');
    out.append('      </diffuseFront>');
    out.append('      <ambientFront>');
    out.append('        <a>'+str(1-material['transparency'])+'</a>');
    out.append('        <r>'+str(material['ambientFront']['r'])+'</r>');
    out.append('        <g>'+str(material['ambientFront']['g'])+'</g>');
    out.append('        <b>'+str(material['ambientFront']['b'])+'</b>');
    out.append('      </ambientFront>');
    out.append('      <specularFront>');
    out.append('        <a>'+str(1-material['transparency'])+'</a>');
    out.append('        <r>'+str(material['specularFront']['r'])+'</r>');
    out.append('        <g>'+str(material['specularFront']['g'])+'</g>');
    out.append('        <b>'+str(material['specularFront']['b'])+'</b>');
    out.append('      </specularFront>');
    if (material['emissionFront']['r'] > 0
        or material['emissionFront']['g'] > 0
        or material['emissionFront']['b'] > 0):
        out.append('      <emissionFront>');
        out.append('        <a>'+str(1-material['transparency'])+'</a>');
        out.append('        <r>'+str(material['emissionFront']['r'])+'</r>');
        out.append('        <g>'+str(material['emissionFront']['g'])+'</g>');
        out.append('        <b>'+str(material['emissionFront']['b'])+'</b>');
        out.append('      </emissionFront>');
    out.append('      <shininess>'+str(material['shininess'])+'</shininess>');
    out.append('      <transparency>'+str(material['transparency'])+'</transparency>');
    if "texturename" in material:
        out.append('      <texturename>'+str(material["texturename"])+'</texturename>');
    out.append('    </material>')
    return out


def exportModelToMARS(model, filepath):
    print(model)
    out = ['<?xml version="1.0"?>',
           "<!DOCTYPE dfkiMarsSceneFile PUBLIC '-//DFKI/RIC/MARS SceneFile 1.0//EN' ''>",
           '<SceneFile>',
           '  <version>0.2</version>']

    #add ids to materials
    i = 1
    for mat in model['materials']:
        model['materials'][mat]['id'] = str(i)
        i += 1

    #add ids to links, visual and collision objects
    i = 1
    for l in model['links']:
        link = model['links'][l]
        link['id'] = i
        i += 1
        for vis in link['visual']:
            link['visual'][vis]['id'] = str(i)
            i += 1
        for col in link['collision']:
            link['collision'][col]['id'] = str(i)
            i += 1

    out.append('  <nodelist>')
    nodes = createNodes(model)
    for node in nodes:
        out.extend(writeNode(node))
    out.append('  </nodelist>')

    #out.append('  <jointlist>')
    #joints = createJoints(model)
    #for joint in joints:
    #    out.append(writeJoint(joint))
    #out.append('  </jointlist>')
#     motorValue = []
#     for joint in jointList:
#         motorOffset = writeJoint(joint)
#         if not "springConst" in joint:
#             if joint["jointType"] == "hinge":
#                 motorValue.append(motorOffset)
#             if joint["jointType"] == "hinge2":
#                 motorValue.append(motorOffset)
#             if joint["jointType"] == "slider":
#                 motorValue.append(motorOffset)


#     out.append('  <motorlist>')
#     i = 0
#     for joint in jointList:
#         if not "springConst" in joint:
#             if joint["jointType"] == "hinge":
#                 out.extend(writeMotor(joint, motorValue[i]))
#                 i += 1
#             elif joint["jointType"] == "hinge2":
#                 out.extend(writeMotor(joint, motorValue[i]))
#                 i += 1
#             elif joint["jointType"] == "slider":
#                 out.extend(writeMotor(joint, motorValue[i]))
#                 i += 1
#     out.append('  </motorlist>')
#
#     haveController = 0
#     for joint in jointList:
#         if "controllerIndex" in joint and joint["controllerIndex"] > 0:
#             haveController = 1
#             myMotorList[joint["controllerIndex"]] = str(joint["id"])
#
#     mySensorList = {}
#     for sensor in sensorList:
#         if "controllerIndex" in sensor and sensor["controllerIndex"] > 0:
#             haveController = 1
#             mySensorList[sensor["id"]] = str(sensor["id"])
#
#
#     out.append('  <sensorlist>')
#     for sensor in sensorList:
#         out.extend(writeSensor(sensor))
#     out.append('  </sensorlist>')
#
#     if haveController == 1:
#         out.append('  <controllerlist>')
#         out.append('    <controller>')
#         out.append('      <rate>40</rate>')
#         for value in mySensorList.values():
#             out.append('      <sensorid>'+value+'</sensorid>')
#         for value in myMotorList.values():
#             out.append('      <motorid>'+value+'</motorid>')
#         out.append('    </controller>')
#         out.append('  </controllerlist>')

    # write materials
    out.append('  <materiallist>')
    for material in model['materials']:
        out.extend(writeMaterial(model['materials'][material]))
    out.append('  </materiallist>')

    out.append('  <graphicOptions>')
    out.append('    <clearColor>')
    out.append('      <r>0.550000</r>')
    out.append('      <g>0.670000</g>')
    out.append('      <b>0.880000</b>')
    out.append('      <a>1.000000</a>')
    out.append('    </clearColor>')
    out.append('    <fogEnabled>false</fogEnabled>')
    out.append('  </graphicOptions>')
    out.append('</SceneFile>')

    print('\n\n\n')
    print(out)


    with open(filepath, 'w') as outputfile:
        outputfile.write('\n'.join(out))


# it would be nice to also set the pivot to the object center
#Blender.Redraw()
