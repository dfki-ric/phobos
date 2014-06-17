'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtcreateprops.py

Created on 7 Jan 2014

@author: Malte Langosz, Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
'''


# ******** TODO:
#               - add jointOffset calculation by using rotation difference
#                 on joint axis of node2

import bpy
import os, glob
import mathutils
import struct


objList = []
jointList = []
sensorList = []
haveController = 0
myMotorList = {}
out = None

#This is a really bad hack, there has to be a better way of doing this.
def initGlobalVariables():
    global objList
    global jointList
    global sensorList
    global haveController
    global myMotorList
    global out
    objList = []
    jointList = []
    sensorList = []
    haveController = 0
    myMotorList = {}
    out = None

class IdGenerator(object):
    def __init__(self, initValue=0):
        self._nextId = initValue
    def __call__(self):
        ret, self._nextId = self._nextId, self._nextId + 1
        return ret

nextMaterialId = IdGenerator(1)

# configuration
defValues = { "filename": "example",
              "path": ".",
              "exportMesh": True,
              "exportBobj": False,
              "defCollBitmask": 65535,
              "defP": 13,
              "defI": 0.015,
              "defD": 0,
              "defMaxMotorForce": 18,
              "defMaxMotorSpeed": 12,
              }


def parseDefaultValues():
    scn = bpy.context.scene
    global defValues
    for key in defValues:
        if key in scn.world:
            defValues[key] = scn.world[key]
    defValues["path"] = os.path.expanduser(defValues["path"])


def veckey3d(v):
    return round(v.x, 6), round(v.y, 6), round(v.z, 6)

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


def exportBobj(outname, obj):
    totverts = totuvco = totno = 1

    face_vert_index = 1

    globalNormals = {}

    if obj.select:

        # ignore dupli children
        if obj.parent and obj.parent.dupli_type in {'VERTS', 'FACES'}:
            # XXX
            print(obj.name, 'is a dupli child - ignoring')
            return

        #obj.select = False
        mesh = obj.to_mesh(bpy.context.scene, True, 'PREVIEW')
        #mesh.transform(obj.matrix_world)

        write_uv = False
        faceuv = len(mesh.uv_textures)
        if faceuv:
            uv_layer = mesh.uv_textures.active.data[:]
            write_uv = True

        if bpy.app.version[1] >= 65:
            face_index_pairs = [(face, index) for index, face in enumerate(mesh.tessfaces)]
        else:
            face_index_pairs = [(face, index) for index, face in enumerate(mesh.faces)]

        mesh.calc_normals()

        me_verts = mesh.vertices[:]

        out = open(outname, "wb")

        for v in mesh.vertices:
            out.write(struct.pack('ifff', 1, v.co[0], v.co[1], v.co[2]))

        if faceuv:
            uv = uvkey = uv_dict = f_index = uv_index = None

            uv_face_mapping = [[0, 0, 0, 0] for i in range(len(face_index_pairs))]  # a bit of a waste for tri's :/

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

            uv_unique_count = len(uv_dict)

            del uv, uvkey, uv_dict, f_index, uv_index

        for f, f_index in face_index_pairs:
            if f.use_smooth:
                for v_idx in f.vertices:
                    v = me_verts[v_idx]
                    noKey = veckey3d(v.normal)
                    if noKey not in globalNormals:
                        globalNormals[noKey] = totno
                        totno += 1
                        da = struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2])
                        out.write(da)
            else:
                # Hard, 1 normal from the face.
                noKey = veckey3d(f.normal)
                if noKey not in globalNormals:
                    globalNormals[noKey] = totno
                    totno += 1
                    da = struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2])
                    out.write(da)

        for f, f_index in face_index_pairs:
            f_smooth = f.use_smooth
            if faceuv:
                tface = uv_layer[f_index]
            # wrtie smooth info for face?

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
                            da = struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], globalNormals[veckey3d(v.normal)])
                            out.write(da)  # vert, uv, normal
                    else:  # No smoothing, face normals
                        no = globalNormals[veckey3d(f.normal)]
                        for vi, v in f_v:
                            da = struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], no)
                            out.write(da)  # vert, uv, normal
                else:  # No UV's
                    if f_smooth:  # Smoothed, use vertex normals
                        for vi, v in f_v:
                            da = struct.pack('iii', v.index + totverts, 0, globalNormals[veckey3d(v.normal)])
                            out.write(da)  # vert, uv, normal
                    else:  # No smoothing, face normals
                        no = globalNormals[veckey3d(f.normal)]
                        for vi, v in f_v:
                            da = struct.pack('iii', v.index + totverts, 0, no)
                            out.write(da)  # vert, uv, normal
        out.close()


def getChildren(parent):
    children = []
    for obj in bpy.data.objects:
        if obj.select and obj.parent == parent:
            children.append(obj)
    return children

def fillList(obj):
    if "MARStype" in obj:
        if obj.MARStype == "body":
            objList.append(obj)
        elif obj.MARStype == "joint":
            jointList.append(obj)
        elif obj.MARStype == "sensor":
            sensorList.append(obj)
    obj.select = False

    children = getChildren(obj)
    for child in children:
        fillList(child)

def writeSceneHeader():
    out.write('<?xml version="1.0"?>\n'
              "<!DOCTYPE dfkiMarsSceneFile PUBLIC '-//DFKI/RIC/MARS SceneFile 1.0//EN' ''>\n"
              '<SceneFile>\n'
              '  <version>0.2</version>\n')

def calcCenter(boundingbox):
    c = [0,0,0]
    for v in boundingbox:
        for i in range(3):
            c[i] += v[i]
    for i in range(3):
        c[i] /= 8.
    return c

def writeNode(obj):
    #TODO: move to updateproperties
    if obj.active_material is None:
        print("WARNING: Object %s has no material! Creating default" % str(obj))
        obj.active_material = bpy.data.materials.new("default")
    if "marsID" in obj.active_material and obj.active_material["marsID"] != 0:
        matID = obj.active_material["marsID"]
    else:
        matID = nextMaterialId()
        obj.active_material["marsID"] = matID

    obj_name = obj["use"] if "use" in obj else obj.name
    filename = obj_name + (".bobj" if defValues["exportBobj"] else ".obj") #!

    # get bounding box:
    bBox = obj.bound_box
    center = calcCenter(bBox)
    size = [0.0, 0.0, 0.0]
    size[0] = abs(2.0*(bBox[0][0] - center[0]))
    size[1] = abs(2.0*(bBox[0][1] - center[1]))
    size[2] = abs(2.0*(bBox[0][2] - center[2]))

    sizeScaleX = obj["sizeScaleX"] if "sizeScaleX" in obj else 1.0
    sizeScaleY = obj["sizeScaleY"] if "sizeScaleY" in obj else 1.0
    sizeScaleZ = obj["sizeScaleZ"] if "sizeScaleZ" in obj else 1.0

    physicMode = obj["physicMode"] if "physicMode" in obj else "box"
    radius = float(obj["radius"]) if "radius" in obj else 0.0
    height = float(obj["height"]) if "height" in obj else 0.0

    ext = [0, 0, 0]
    ext[0] = radius if radius > 0. else sizeScaleX*size[0]
    ext[1] = height if height > 0. else sizeScaleY*size[1]
    ext[2] = sizeScaleZ*size[2]

    pivot = center
    center = obj.location.copy()
    center += obj.matrix_world.to_quaternion() * mathutils.Vector((pivot[0], pivot[1], pivot[2]))

    noPhysical = False
    if "noPhysical" in obj:
        noPhysical = obj["noPhysical"]

    if "mass" in obj:
        density = 0
    elif "density" in obj:
        density = obj["density"]
    else:
        density = 500
    coll_bitmask = obj["coll_bitmask"] if "coll_bitmask" in obj else defValues["defCollBitmask"]

    parentID = 0

    obj.rotation_mode = 'QUATERNION'
    q = obj.rotation_quaternion

    if obj.parent:
        parentID = obj.parent["id"]
        parent = obj.parent
        parentIQ = parent.matrix_world.to_quaternion().inverted()
        bBox = parent.bound_box
        pivot2 = calcCenter(bBox)
        v = mathutils.Vector((pivot2[0], pivot2[1], pivot2[2]))
        #v = parent.matrix_world.to_quaternion() * v
        parentPos = parent.matrix_world * v
        childPos = obj.matrix_world * mathutils.Vector((pivot[0], pivot[1], pivot[2]))
        childPos = childPos - parentPos
        center = parentIQ * childPos
        parentRot = parent.matrix_world.to_quaternion()
        childRot = obj.matrix_world.to_quaternion()
        childRot = parentRot.rotation_difference(childRot)
        q = childRot

    out.write('    <node name="'+obj.name+'">\n')
    out.write('      <origname>'+obj_name+'</origname>\n')
    out.write('      <filename>'+filename+'</filename>\n')
    out.write('      <index>'+str(obj["id"])+'</index>\n')
    out.write('      <groupid>'+str(obj["group"])+'</groupid>\n')
    out.write('      <physicmode>'+physicMode+'</physicmode>\n')
    if(noPhysical):
        out.write('      <noPhysical>'+str(noPhysical)+'</noPhysical>\n')
    if parentID:
        out.write('      <relativeid>'+str(parentID)+'</relativeid>\n')
    outputVector(out, "position", center, 6)
    outputQuaternion(out, "rotation", q, 6)
    outputVector(out, "extend", ext, 6)
    outputVector(out, "pivot", pivot, 6)
    outputVector(out, "visualsize", size, 6)
    if "movable" in obj:
        out.write('      <movable>'+str(obj["movable"])+'</movable>\n')
    else:
        out.write('      <movable>true</movable>\n')
    if "mass" in obj:
        out.write('      <mass>'+str(obj["mass"])+'</mass>\n')
    out.write('      <density>'+str(density)+'</density>\n')
    out.write('      <material_id>'+str(matID)+'</material_id>\n')
    out.write('      <coll_bitmask>'+str(coll_bitmask)+'</coll_bitmask>\n')
    out.write('    </node>\n')



def writeJoint(joint):
    #bBox = joint.bound_box
    pos = mathutils.Vector((0.0, 0.0, 1.0))
    axis = joint.matrix_world.to_quaternion() * pos
    center = joint.matrix_world * mathutils.Vector((0.0, 0.0, 0.0))
    node2ID = 0
    node2 = None
    if joint["node2"] in bpy.data.objects:
        node2 = bpy.data.objects[joint["node2"]]
        node2ID = node2["id"]

    invert = 1
    if "invertAxis" in joint and joint["invertAxis"] == 1:
        invert = -1

    jointOffset = 0.0
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

    #jointType = {"hinge": 1, "fixed": 6, "slider": 3}[joint["jointType"]]
    jointType = joint["jointType"]
    anchorPos = {"custom": 4, "node1": 1, "node2": 2, "center": 3}[joint["anchor"]]

    #calcCenter(bBox)
    #center = sum(bBox) / 8.
    out.write('    <joint name="'+joint.name+'">\n')
    out.write('      <index>'+str(joint["id"])+'</index>\n')
    out.write('      <type>'+str(jointType)+'</type>\n')
    out.write('      <nodeindex1>'+str(joint.parent["id"])+'</nodeindex1>\n')
    out.write('      <nodeindex2>'+str(node2ID)+'</nodeindex2>\n')
    out.write('      <anchorpos>'+str(anchorPos)+'</anchorpos>\n')
    outputVector(out, "anchor", center, 6)
    outputVector(out, "axis1", (invert*axis[0],invert*axis[1],invert*axis[2]), 6)
    if "axis2x" in joint:
        outputVector(out, "axis2", (joint["axis2x"], joint["axis2y"], joint["axis2z"]), 6)
    if "lowStop" in joint:
        out.write('      <lowStopAxis1>'+str(joint["lowStop"])+'</lowStopAxis1>\n')
    if "highStop" in joint:
        out.write('      <highStopAxis1>'+str(joint["highStop"])+'</highStopAxis1>\n')
    if "springConst" in joint:
        out.write('      <spring_const_constraint_axis1>'+str(joint["springConst"])+'</spring_const_constraint_axis1>\n')
    if "dampingConst" in joint:
        out.write('      <damping_const_constraint_axis1>'+str(joint["dampingConst"])+'</damping_const_constraint_axis1>\n')
    out.write('      <angle1_offset>'+str(invert*jointOffset)+'</angle1_offset>\n')
    out.write('    </joint>\n')
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

    out.write('    <motor name="'+joint.name+'">\n')
    out.write('      <index>'+str(joint["id"])+'</index>\n')
    out.write('      <jointIndex>'+str(joint["id"])+'</jointIndex>\n')
    out.write('      <axis>'+str(motor_axis)+'</axis>\n')
    out.write('      <maximumVelocity>'+str(max_speed)+'</maximumVelocity>\n')
    out.write('      <motorMaxForce>'+str(max_force)+'</motorMaxForce>\n')
    out.write('      <type>'+str(motor_type)+'</type>\n')
    out.write('      <p>'+str(motor_p)+'</p>\n')
    out.write('      <i>'+str(motor_i)+'</i>\n')
    out.write('      <d>'+str(motor_d)+'</d>\n')
    out.write('      <min_val>'+str(low_stop)+'</min_val>\n')
    out.write('      <max_val>'+str(high_stop)+'</max_val>\n')
    out.write('      <value>'+str(motorValue)+'</value>\n')
    out.write('    </motor>\n')


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
    out.write('    <sensor name="'+sensor.name+'" type="'+str(sensorType)+'">\n')
    out.write('      <index>'+str(sensor["id"])+'</index>\n')
    out.write('      <rate>'+str(rate)+'</rate>\n')
    for value in idList.values():
        out.write('      <id>'+str(value)+'</id>\n')
    if sensorType == "Joint6DOF":
        nodeID = getID(sensor["nodeID"])
        jointID = getID(sensor["jointID"])
        out.write('      <nodeID>'+str(nodeID)+'</nodeID>\n')
        out.write('      <jointID>'+str(jointID)+'</jointID>\n')
    elif sensorType == "RaySensor":
        nodeID = getID(sensor["attached_node"])
        out.write('      <attached_node>'+str(nodeID)+'</attached_node>\n')
        out.write('      <width>'+str(sensor["width"])+'</width>\n')
        out.write('      <opening_width>'+str(sensor["opening_width"])+
                  '</opening_width>\n')
        out.write('      <max_distance>'+str(sensor["max_distance"])+
                  '</max_distance>\n')
        if "draw_rays" in sensor:
            out.write('      <draw_rays>'+str(sensor["draw_rays"])+'</draw_rays>\n')
    elif sensorType == "CameraSensor":
        nodeID = getID(sensor["attached_node"])
        out.write('      <attached_node>'+str(nodeID)+'</attached_node>\n')
        out.write('      <depth_image>'+str(sensor["depth_image"])+'</depth_image>\n')
        out.write('      <show_cam hud_idx="'+str(sensor["hud_idx"])+'">'+str(sensor["show_cam"])+'</show_cam>\n')
        out.write('      <position_offset x="'+str(sensor["position_offset_x"])+
                '" y="'+str(sensor["position_offset_y"])+
                '" z="'+str(sensor["position_offset_z"])+'"/>\n')
        out.write('      <orientation_offset yaw="'+str(sensor["orientation_offset_yaw"])+
                '" pitch="'+str(sensor["orientation_offset_pitch"])+
                '" roll="'+str(sensor["orientation_offset_roll"])+'"/>\n')
    out.write('    </sensor>\n')


def writeMaterial(material):
    out.write('    <material>\n')
    out.write('      <id>'+str(material["marsID"])+'</id>\n')
    out.write('      <diffuseFront>\n');
    out.write('        <a>1.0</a>\n');
    out.write('        <r>'+str(material.diffuse_color[0])+'</r>\n');
    out.write('        <g>'+str(material.diffuse_color[1])+'</g>\n');
    out.write('        <b>'+str(material.diffuse_color[2])+'</b>\n');
    out.write('      </diffuseFront>\n');
    out.write('      <specularFront>\n');
    out.write('        <a>1.0</a>\n');
    out.write('        <r>'+str(material.specular_color[0])+'</r>\n');
    out.write('        <g>'+str(material.specular_color[1])+'</g>\n');
    out.write('        <b>'+str(material.specular_color[2])+'</b>\n');
    out.write('      </specularFront>\n');
    out.write('      <shininess>'+str(material.specular_hardness/2)+'</shininess>\n');
    if "cullMask" in material:
        out.write('      <cullMask>'+str(material["cullMask"])+'</cullMask>\n');
    if "texturename" in material:
        out.write('      <texturename>'+str(material["texturename"])+'</texturename>\n');
    out.write('    </material>\n')



def findRoot():
    root = None
    for obj in bpy.data.objects:
        if obj.select:
            if not obj.parent:
                root = obj
                break
    return root


def main():
    initGlobalVariables()
    global out

    parseDefaultValues()
    out = open(defValues["path"]+"/"+defValues["filename"]+".scene", "w")

    writeSceneHeader()
    root = findRoot()
    if root:
        fillList(root)
    for material in bpy.data.materials:
        if "marsID" in material:
            material["marsID"] = 0

    out.write('  <nodelist>\n')
    for node in objList:
        writeNode(node)
    out.write('  </nodelist>\n')

    out.write('  <jointlist>\n')
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
    out.write('  </jointlist>\n')

    out.write('  <motorlist>\n')
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
    out.write('  </motorlist>\n')

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


    out.write('  <sensorlist>\n')
    for sensor in sensorList:
        writeSensor(sensor)
    out.write('  </sensorlist>\n')

    if haveController == 1:
        out.write('  <controllerlist>\n')
        out.write('    <controller>\n')
        out.write('      <rate>40</rate>\n')
        for value in mySensorList.values():
            out.write('      <sensorid>'+value+'</sensorid>\n')
        for value in myMotorList.values():
            out.write('      <motorid>'+value+'</motorid>\n')
        out.write('    </controller>\n')
        out.write('  </controllerlist>\n')

    out.write('  <materiallist>\n')

    for material in bpy.data.materials:
        if "marsID" in material and material["marsID"] != 0:
            writeMaterial(material)

    out.write('  </materiallist>\n')

    out.write('  <graphicOptions>\n')
    out.write('    <clearColor>\n')
    out.write('      <r>0.550000</r>\n')
    out.write('      <g>0.670000</g>\n')
    out.write('      <b>0.880000</b>\n')
    out.write('      <a>1.000000</a>\n')
    out.write('    </clearColor>\n')
    out.write('    <fogEnabled>false</fogEnabled>\n')
    out.write('  </graphicOptions>\n')
    out.write('</SceneFile>\n')

    out.close()

    os.chdir(defValues["path"])
    os.system("rm "+defValues["filename"]+".scn")
    os.system("zip "+defValues["filename"]+".scn "+defValues["filename"]+".scene")


    #wm = bpy.context.window_manager
    total = float(len(objList))
    #wm.progress_begin(0, total)
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
        #wm.progress_update(i)
        i += 1

    #wm.progress_end()


# it would be nice to also set the pivot to the object center
#Blender.Redraw()

if __name__ == '__main__':
    main()
