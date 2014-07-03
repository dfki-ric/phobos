'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtupdate.py

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
import marstools.mtutility as mtutility

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

def outputVector(outStream, name, vec, indentLevel):
    indent = " " * indentLevel
    outStream.write(indent + '<'+name+'>\n')
    outStream.write(indent + '  <x>'+str(vec[0])+'</x>\n')
    outStream.write(indent + '  <y>'+str(vec[1])+'</y>\n')
    outStream.write(indent + '  <z>'+str(vec[2])+'</z>\n')
    outStream.write(indent + '</'+name+'>\n')

def outputQuaternion(outStream, name, q, indentLevel):
    indent = " " * indentLevel
    outStream.write(indent + '<'+name+'>\n')
    outStream.write(indent + '  <x>'+str(q[1])+'</x>\n')
    outStream.write(indent + '  <y>'+str(q[2])+'</y>\n')
    outStream.write(indent + '  <z>'+str(q[3])+'</z>\n')
    outStream.write(indent + '  <w>'+str(q[0])+'</w>\n')
    outStream.write(indent + '</'+name+'>\n')

def calcCenter(bound_box):
    """returns a mathutils.Vector for the bounding box's center point"""
    c = sum((mathutils.Vector(b) for b in bound_box), mathutils.Vector())
    return c / 8

def writeNode(link):
    noPhysical = 'collision' in link
    if 'collision' in link:
        geom = link['collision']['geometry']
        physicMode = geom['geometryType']
        radius = geom["radius"] if "radius" in geom else 0.0
        height = geom["height"] if "height" in geom else 0.0

        ext = [geom['size'][i] * geom['scale'][i] for i in range(3)]
        ext[0] = radius if radius > 0. else geom['scale'][0]
        ext[1] = height if height > 0. else geom['scale'][1]

        coll_bitmask = link['collision']['coll_bitmask'] if "coll_bitmask" in link['collision'] else '65535'


    filename = link['name'] + '.obj' if bpy.worlds[0].exportObj else link['name'] + '.bobj'

    parentID = 0
    pivot = calcCenter(obj.bound_box)
    if obj.parent:
        parentID = obj.parent["id"]
        parent = obj.parent
        parentPos = parent.matrix_world * calcCenter(parent.bound_box)
        childPos = obj.matrix_world * pivot
        childPos = childPos - parentPos
        center = parent.matrix_world.to_quaternion().inverted() * childPos
        childRot = obj.matrix_local.to_quaternion()
    else:
        center = link['pose']['translation']
        childRot = link['pose']['rotation_quaternion']

    out.append('    <node name="' + link['name'] + '">\n')
    out.append('      <origname>' + link['name'] + '</origname>\n')
    out.append('      <filename>'+filename+'</filename>\n')
    out.append('      <index>'+str(obj["id"])+'</index>\n')
    out.append('      <groupid>'+str(obj["group"])+'</groupid>\n')
    out.append('      <physicmode>'+physicMode+'</physicmode>\n')
    if(noPhysical):
        out.append('      <noPhysical>'+str(noPhysical)+'</noPhysical>\n')
    if parentID:
        out.append('      <relativeid>'+str(parentID)+'</relativeid>\n')
    outputVector(out, "position", center, 6)
    outputQuaternion(out, "rotation", childRot, 6)
    outputVector(out, "extend", ext, 6)
    outputVector(out, "pivot", pivot, 6)
    outputVector(out, "visualsize", obj.dimensions, 6)
    if "movable" in obj:
        out.append('      <movable>'+str(obj["movable"])+'</movable>\n')
    else:
        out.append('      <movable>true</movable>\n')
    if "mass" in obj:
        out.append('      <mass>'+str(obj["mass"])+'</mass>\n')
    out.append('      <material_id>'+str(matID)+'</material_id>\n')
    out.append('      <coll_bitmask>'+str(coll_bitmask)+'</coll_bitmask>\n')
    out.append('    </node>\n')



def writeJoint(joint):
    axis = joint.matrix_world.to_quaternion() * mathutils.Vector((0.0, 0.0, 1.0))
    center = joint.matrix_world.to_translation()
    if joint["node2"] in bpy.data.objects:
        node2 = bpy.data.objects[joint["node2"]]
        node2ID = node2["id"]
    else:
        node2 = None
        node2ID = 0

    if "invertAxis" in joint and joint["invertAxis"] == 1:
        invert = -1
    else:
        invert = 1

    if "jointOffset" in joint:
        jointOffset = joint["jointOffset"]
    elif node2:
        joint.rotation_mode = 'QUATERNION'
        v1 = joint.rotation_quaternion * mathutils.Vector((1.0, 0.0, 0.0))
        v2 = node2.rotation_quaternion * mathutils.Vector((1.0, 0.0, 0.0))
        jointOffset = v1.angle(v2, 0.0)
        q = joint.rotation_quaternion.copy().inverted()
        axis_ = q * v1.cross(v2)
        if axis_[2] > 0:
            jointOffset *= -1
    else:
        jointOffset = 0.0

    #jointType = {"hinge": 1, "fixed": 6, "slider": 3}[joint["jointType"]]
    jointType = joint["jointType"]
    anchorPos = {"custom": 4, "node1": 1, "node2": 2, "center": 3}[joint["anchor"]]

    out.append('    <joint name="'+joint.name+'">\n')
    out.append('      <index>'+str(joint["id"])+'</index>\n')
    out.append('      <type>'+str(jointType)+'</type>\n')
    out.append('      <nodeindex1>'+str(joint.parent["id"])+'</nodeindex1>\n')
    out.append('      <nodeindex2>'+str(node2ID)+'</nodeindex2>\n')
    out.append('      <anchorpos>'+str(anchorPos)+'</anchorpos>\n')
    outputVector(out, "anchor", center, 6)
    outputVector(out, "axis1", (invert*axis[0],invert*axis[1],invert*axis[2]), 6)
    if "axis2x" in joint:
        outputVector(out, "axis2", (joint["axis2x"], joint["axis2y"], joint["axis2z"]), 6)
    if "lowStop" in joint:
        out.append('      <lowStopAxis1>'+str(joint["lowStop"])+'</lowStopAxis1>\n')
    if "highStop" in joint:
        out.append('      <highStopAxis1>'+str(joint["highStop"])+'</highStopAxis1>\n')
    if "springConst" in joint:
        out.append('      <spring_const_constraint_axis1>'+str(joint["springConst"])+'</spring_const_constraint_axis1>\n')
    if "dampingConst" in joint:
        out.append('      <damping_const_constraint_axis1>'+str(joint["dampingConst"])+'</damping_const_constraint_axis1>\n')
    out.append('      <angle1_offset>'+str(invert*jointOffset)+'</angle1_offset>\n')
    out.append('    </joint>\n')
    return jointOffset*invert


def writeMotor(joint, motorValue):
    motor_type = joint["motor_type"] if "motor_type" in joint else 1
    motor_axis = joint["motorAxis"] if "motorAxis" in joint else 1
    motor_p = joint["p"] if "p" in joint else defValues["defP"]
    motor_i = joint["i"] if "i" in joint else defValues["defI"]
    motor_d = joint["d"] if "d" in joint else defValues["defD"]
    low_stop = joint["lowStop"] if "lowStop" in joint else -6.28
    high_stop = joint["highStop"] if "highStop" in joint else 6.28
    max_speed = joint["maxSpeed"] if "maxSpeed" in joint else defValues["defMaxMotorSpeed"]
    max_force = joint["maxForce"] if "maxForce" in joint else defValues["defMaxMotorForce"]

    out.append('    <motor name="'+joint.name+'">\n')
    out.append('      <index>'+str(joint["id"])+'</index>\n')
    out.append('      <jointIndex>'+str(joint["id"])+'</jointIndex>\n')
    out.append('      <axis>'+str(motor_axis)+'</axis>\n')
    out.append('      <maximumVelocity>'+str(max_speed)+'</maximumVelocity>\n')
    out.append('      <motorMaxForce>'+str(max_force)+'</motorMaxForce>\n')
    out.append('      <type>'+str(motor_type)+'</type>\n')
    out.append('      <p>'+str(motor_p)+'</p>\n')
    out.append('      <i>'+str(motor_i)+'</i>\n')
    out.append('      <d>'+str(motor_d)+'</d>\n')
    out.append('      <min_val>'+str(low_stop)+'</min_val>\n')
    out.append('      <max_val>'+str(high_stop)+'</max_val>\n')
    out.append('      <value>'+str(motorValue)+'</value>\n')
    out.append('    </motor>\n')


def writeSensor(sensor):
    sensorType = sensor["sensorType"] if "sensorType" in sensor else "unknown"
    rate = sensor["rate"] if "rate" in sensor else 10.0
    idList = {}

    if "listMotors" in sensor:
        idList = myMotorList
    elif "listNodes" in sensor:
        for i, obj in enumerate(objList):
            idList[i] = obj["id"]
    for key, value in sensor.items():
        if key[:5] == "index":
            index = getID(value)
            if index != 0:
                idList[int(key[5:])] = index
    out.append('    <sensor name="'+sensor.name+'" type="'+str(sensorType)+'">\n')
    out.append('      <index>'+str(sensor["id"])+'</index>\n')
    out.append('      <rate>'+str(rate)+'</rate>\n')
    for value in idList.values():
        out.append('      <id>'+str(value)+'</id>\n')
    if sensorType == "Joint6DOF":
        nodeID = getID(sensor["nodeID"])
        jointID = getID(sensor["jointID"])
        out.append('      <nodeID>'+str(nodeID)+'</nodeID>\n')
        out.append('      <jointID>'+str(jointID)+'</jointID>\n')
    elif sensorType == "RaySensor":
        nodeID = getID(sensor["attached_node"])
        out.append('      <attached_node>'+str(nodeID)+'</attached_node>\n')
        out.append('      <width>'+str(sensor["width"])+'</width>\n')
        out.append('      <opening_width>'+str(sensor["opening_width"])+
                  '</opening_width>\n')
        out.append('      <max_distance>'+str(sensor["max_distance"])+
                  '</max_distance>\n')
        if "draw_rays" in sensor:
            out.append('      <draw_rays>'+str(sensor["draw_rays"])+'</draw_rays>\n')
    elif sensorType == "CameraSensor":
        nodeID = getID(sensor["attached_node"])
        out.append('      <attached_node>'+str(nodeID)+'</attached_node>\n')
        out.append('      <depth_image>'+str(sensor["depth_image"])+'</depth_image>\n')
        out.append('      <show_cam hud_idx="'+str(sensor["hud_idx"])+'">'+str(sensor["show_cam"])+'</show_cam>\n')
        out.append('      <position_offset x="'+str(sensor["position_offset_x"])+
                '" y="'+str(sensor["position_offset_y"])+
                '" z="'+str(sensor["position_offset_z"])+'"/>\n')
        out.append('      <orientation_offset yaw="'+str(sensor["orientation_offset_yaw"])+
                '" pitch="'+str(sensor["orientation_offset_pitch"])+
                '" roll="'+str(sensor["orientation_offset_roll"])+'"/>\n')
    out.append('    </sensor>\n')


def writeMaterial(material):
    out.append('    <material>\n')
    out.append('      <id>'+str(material["marsID"])+'</id>\n')
    out.append('      <diffuseFront>\n');
    out.append('        <a>1.0</a>\n');
    out.append('        <r>'+str(material.diffuse_color[0])+'</r>\n');
    out.append('        <g>'+str(material.diffuse_color[1])+'</g>\n');
    out.append('        <b>'+str(material.diffuse_color[2])+'</b>\n');
    out.append('      </diffuseFront>\n');
    out.append('      <specularFront>\n');
    out.append('        <a>1.0</a>\n');
    out.append('        <r>'+str(material.specular_color[0])+'</r>\n');
    out.append('        <g>'+str(material.specular_color[1])+'</g>\n');
    out.append('        <b>'+str(material.specular_color[2])+'</b>\n');
    out.append('      </specularFront>\n');
    out.append('      <shininess>'+str(material.specular_hardness/2)+'</shininess>\n');
    if "cullMask" in material:
        out.append('      <cullMask>'+str(material["cullMask"])+'</cullMask>\n');
    if "texturename" in material:
        out.append('      <texturename>'+str(material["texturename"])+'</texturename>\n');
    out.append('    </material>\n')

def exportModelToMARS(model, filepath):
    out = ['<?xml version="1.0"?>\n',
           "<!DOCTYPE dfkiMarsSceneFile PUBLIC '-//DFKI/RIC/MARS SceneFile 1.0//EN' ''>\n",
           '<SceneFile>\n',
           '  <version>0.2</version>\n']

    #add ids to materials
    i = 1
    for mat in model['materials']:
        model['materials'][mat]['id'] = i
        i += 1

    out.append('  <nodelist>\n')
    for l in model['links']:
        out.append(writeNode(model['links'][l]))
    out.append('  </nodelist>\n')

    out.append('  <jointlist>\n')
    motorValue = []
    for joint in jointList:
        motorOffset = writeJoint(joint)
        if not "springConst" in joint:
            if joint["jointType"] == "hinge":
                motorValue.append(motorOffset)
            if joint["jointType"] == "hinge2":
                motorValue.append(motorOffset)
            if joint["jointType"] == "slider":
                motorValue.append(motorOffset)
    out.append('  </jointlist>\n')

    out.append('  <motorlist>\n')
    i = 0
    for joint in jointList:
        if not "springConst" in joint:
            if joint["jointType"] == "hinge":
                writeMotor(joint, motorValue[i])
                i += 1
            elif joint["jointType"] == "hinge2":
                writeMotor(joint, motorValue[i])
                i += 1
            elif joint["jointType"] == "slider":
                writeMotor(joint, motorValue[i])
                i += 1
    out.append('  </motorlist>\n')

    haveController = 0
    for joint in jointList:
        if "controllerIndex" in joint and joint["controllerIndex"] > 0:
            haveController = 1
            myMotorList[joint["controllerIndex"]] = str(joint["id"])

    mySensorList = {}
    for sensor in sensorList:
        if "controllerIndex" in sensor and sensor["controllerIndex"] > 0:
            haveController = 1
            mySensorList[sensor["id"]] = str(sensor["id"])


    out.append('  <sensorlist>\n')
    for sensor in sensorList:
        writeSensor(sensor)
    out.append('  </sensorlist>\n')

    if haveController == 1:
        out.append('  <controllerlist>\n')
        out.append('    <controller>\n')
        out.append('      <rate>40</rate>\n')
        for value in mySensorList.values():
            out.append('      <sensorid>'+value+'</sensorid>\n')
        for value in myMotorList.values():
            out.append('      <motorid>'+value+'</motorid>\n')
        out.append('    </controller>\n')
        out.append('  </controllerlist>\n')

    out.append('  <materiallist>\n')

    for material in bpy.data.materials:
        if "marsID" in material and material["marsID"] != 0:
            writeMaterial(material)

    out.append('  </materiallist>\n')

    out.append('  <graphicOptions>\n')
    out.append('    <clearColor>\n')
    out.append('      <r>0.550000</r>\n')
    out.append('      <g>0.670000</g>\n')
    out.append('      <b>0.880000</b>\n')
    out.append('      <a>1.000000</a>\n')
    out.append('    </clearColor>\n')
    out.append('    <fogEnabled>false</fogEnabled>\n')
    out.append('  </graphicOptions>\n')
    out.append('</SceneFile>\n')

    out.close()

    os.chdir(defValues["path"])
    os.system("rm "+defValues["filename"]+".scn")
    os.system("zip "+defValues["filename"]+".scn "+defValues["filename"]+".scene")

    show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269;
    if show_progress:
        wm = bpy.context.window_manager
        total = float(len(objList))
        wm.progress_begin(0, total)
    i = 1
    for obj in objList:
        if "use" in obj:
            continue
        obj_filename = obj.name + (".bobj" if defValues["exportBobj"] else ".obj")
        obj.select = True
        bpy.context.scene.objects.active = obj
        bpy.ops.object.modifier_apply(modifier='EdgeSplit')
        #bpy.ops.object.transform_apply(rotation=True)
        location = obj.location.copy()
        rotation = obj.rotation_quaternion.copy()
        parent = obj.parent
        obj.location = [0.0, 0.0, 0.0]
        obj.rotation_quaternion = [1.0, 0.0, 0.0, 0.0]
        obj.parent = None
        if defValues["exportMesh"]:
            out_name = defValues["path"]+"/" + obj_filename
            if defValues["exportBobj"]:
                exportBobj(out_name, obj)
            else:
                bpy.ops.export_scene.obj(filepath=out_name, axis_forward='-Z',
                                         axis_up='Y', use_selection=True,
                                         use_normals=True)
        os.system("zip "+defValues["filename"]+".scn "+obj_filename)
        #os.system("zip "+defValues["filename"]+".scn "+obj.name+".mtl")
        obj.location = location
        obj.rotation_quaternion = rotation
        obj.parent = parent
        obj.select = False
        if show_progress:
            wm.progress_update(i)
        i += 1
    if show_progress:
        wm.progress_end()

    with open(filepath, 'w') as outputfile:
        outputfile.write(''.join(out))


# it would be nice to also set the pivot to the object center
#Blender.Redraw()

if __name__ == '__main__':
    main()
