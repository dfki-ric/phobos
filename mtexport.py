'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File export.py

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
import datetime
import yaml
import warnings
from bpy.types import Operator
import marstools.mtdefs as mtdefs
from marstools.mtutility import *
import marstools.mtjoints as mtjoints
import marstools.mtmisctools as mtmisctools

def register():
    print("Registering mtexport...")

indent = '  '
urdfHeader = '<?xml version="1.0"?>\n'
urdfFooter = indent+'</robot>\n'

#bpy.data.armatures["Armature.001"].bones["Bone"].x_axis
#bpy.data.objects["Armature.001"].pose.bones["Bone"].x_axis

def calcPose(obj, objtype):
    #pre-calculations
    bBox = obj.bound_box
    center = calcBoundingBoxCenter(bBox)
    size = [0.0]*3
    size[0] = abs(2.0*(bBox[0][0] - center[0]))
    size[1] = abs(2.0*(bBox[0][1] - center[1]))
    size[2] = abs(2.0*(bBox[0][2] - center[2]))

    pose = []
    if objtype == "link" or objtype == "visual" or objtype == "collision":
        pivot = calcBoundingBoxCenter(obj.bound_box)
        if obj.parent:
            parent = obj.parent
            parentPos = parent.matrix_world * calcBoundingBoxCenter(parent.bound_box)
            childPos = obj.matrix_world * pivot
            childPos = childPos - parentPos
            center = parent.matrix_world.to_quaternion().inverted() * childPos
            childRot = obj.matrix_local.to_quaternion()
        else:
            center = obj.location
            childRot = obj.rotation_quaternion
        pose = list(center)
        pose.extend(childRot)
#    elif objtype == "visual":
#        pass
#    elif objtype == "collision":
#        pass
    elif objtype == "joint": #TODO: we need this for the offset information of the joint with respect to the link
        pos = mathutils.Vector((0.0, 0.0, 1.0))
        axis = obj.matrix_world.to_quaternion() * pos
        center = obj.matrix_world * mathutils.Vector((0.0, 0.0, 0.0))
        obj.rotation_mode = 'QUATERNION'
        v1 = obj.rotation_quaternion * mathutils.Vector((1.0, 0.0, 0.0))
        if obj["node2"] != "world":
            node2 = getObjByName(obj["node2"])
            v2 = node2.rotation_quaternion * mathutils.Vector((1.0, 0.0, 0.0)) #TODO: link to other node in node2
        q = obj.rotation_quaternion.copy().inverted()
        pose = list(center)
        pose.extend(q)
    return pose

def deriveLink(obj):
    props = initObjectProperties(obj)
    props["filename"] = obj.name + (".bobj" if bpy.context.scene.world.exportBobj else ".obj") #TODO: this is only valid if this function is only called upon export
    props["pose"] = calcPose(obj, "link")
    props["collision"] = {}
    props["visual"] = {}
    props["inertial"] = {}
    if "mass" in props:
        props["inertial"]["mass"] = props["mass"]
        del props["mass"]
    if "inertia" in props:
        props["inertial"]["inertia"] = props["inertia"]
        del props["inertia"]
    return props

def deriveJoint(obj):
    props = initObjectProperties(obj)
    #motorprops = {key:props[key] for key in [key if props[key].find('motor/') else None for key in props]}
    props['parent'] = obj.parent.name
    children = getImmediateChildren(obj, 'link')
    props['child'] = children[0].name if children else '' #list contains only 1 element
    props['jointType'], crot = mtjoints.deriveJointType(obj)
    axis, limit = mtjoints.getJointConstraints(obj)
    props['axis'] = obj.rotation_quaternion * axis if axis else '' #calcPose(obj, 0, "joint") #TODO: the 0 is an ugly hack
    props['limit'] = limit if limit else ''
    props['state'] = obj.pose.bones[0].rotation_euler[0:3] #TODO: hard-coding this could prove problematic if we at some point build armatures from multiple bones
    #TODO:
    # - calibration
    # - dynamics
    # - mimic
    # - safety_controller
    return props#, motorprops

def deriveMotor(obj):
    props = initObjectProperties(obj)
    return props, obj.parent

def deriveGeometry(obj):
    if 'geometryType' in obj:
        geometry = {'geometryType': obj['geometryType']}
        gt = obj['geometryType']
        if gt == 'box':
            geometry['size'] = obj.dimensions
        elif gt == 'cylinder':
            geometry["radius"] = obj.dimensions[0]
            geometry["height"] = obj.dimensions[1]
        elif gt == 'sphere':
            geometry['radius'] = obj.dimensions[0]
        elif gt == 'mesh':
            geometry['size'] = obj.dimensions #TODO: this has to be converted to scale when URDF is exported
        return geometry
    else:
        warnings.warn("No geometryType found for object "+obj.name+".")
        return None

def deriveInertial(obj):
    props = initObjectProperties(obj)
    props['inertia'] = props['inertia'].split()
    return props, obj.parent

def deriveVisual(obj):
    visual = initObjectProperties(obj)
    visual["pose"] = calcPose(obj, "visual")
    if obj.data.materials:
        material = {}
        material["name"] = obj.data.materials[0].name #simply grab the first material
        material["diffuseColor"] = list(obj.data.materials[0].diffuse_color) #TODO: get rid of this and directly retrieve information from blenders material list
        visual["material"] = material
    visual["geometry"] = deriveGeometry(obj)
    return visual, obj.parent

def deriveCollision(obj):
    collision = initObjectProperties(obj)
    collision["geometry"] = deriveGeometry(obj)
    collision["pose"] = calcPose(obj, "collision") #TODO: technically, this creates twice the computation for naught
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
    #clean dictionary entries
    getridof = ['MARStype', '_RNA_UI', 'cycles_visibility']
    if props:
        for key in getridof:
            if key in props:
                del props[key]
    return props

def deriveDictEntry(obj):
    print("MARStools: Deriving dictionary entry for", obj.name)
    props = None
    parent = None
    try:
        if obj.MARStype == 'link':
            props = deriveLink(obj)
        elif obj.MARStype == 'joint':
            props = deriveJoint(obj)
        elif obj.MARStype == 'inertial':
            props, parent = deriveInertial(obj)
        elif obj.MARStype == 'visual':
            props, parent = deriveVisual(obj)
        elif obj.MARStype == 'collision':
            props, parent = deriveCollision(obj)
        elif obj.MARStype == 'sensor':
            props = deriveSensor(obj)
        elif obj.MARStype == 'motor':
            props, parent = deriveMotor(obj)
        elif obj.MARStype == 'controller':
            props = deriveController(obj)
    except KeyError:
        print('MARStools: A KeyError occurred, likely because there is missing information in the model:\n    ', sys.exc_info()[0])
    if obj.MARStype in ['link', 'joint', 'sensor', 'motor' 'controller']:
        return cleanObjectProperties(props)
    else:
        return cleanObjectProperties(props), parent

def deriveGroupEntry(group):
    return [obj.name for obj in group.objects]

def buildRobotDictionary():
    notifications, faulty_objects = mtmisctools.checkModel(bpy.context.selected_objects)
    print(notifications)
    robot = {"link": {},
            "joint": {},
            "sensor": {},
            "motor": {},
            "controller": {},
            "group": {}}
    #save timestamped version of model
    robot["date"] = datetime.datetime.now().strftime("%Y%m%d_%H:%M")
    root = getRoot(bpy.context.selected_objects[0])
    if root.MARStype == 'link':
        if 'modelname' in root:
            robot['modelname'] = root["modelname"]
        else:
            robot['modelname'] = 'unnamed_robot'
    else:
        print("ERROR: Found no 'link' object as root of the robot model.")
    # now parse the scene
    # first digest all the links and joints as well as sensors, motors and controllers
    for obj in bpy.context.selected_objects:
        if obj.MARStype in ['link', 'joint', 'sensor', 'motor', 'controller']:
            robot[obj.MARStype][obj.name] = deriveDictEntry(obj)
            obj.select = False
    # second complete link information by parsing visuals and collision objects
    try:
        for obj in bpy.context.selected_objects:
            if obj.MARStype in ['inertial', 'visual', 'collision']:
                props, parent = deriveDictEntry(obj)
                robot[parent.parent.MARStype][parent.parent.name][obj.MARStype] = props
    except (KeyError, TypeError):
        print("An error occurred when inserting the entry for object", obj.name, sys.exc_info()[0], sys.exc_info()[1], sys.exc_info()[2])
        print(robot)
    for group in bpy.data.groups: #TODO: get rid of the "data" part
        robot["group"] = deriveGroupEntry(group)
    return robot

def exportModelToYAML(model, filepath):
    print("MARStools YAML export: Writing model data to", filepath )
    with open(filepath, 'w') as outputfile:
        outputfile.write('#YAML dump of robot model "'+model["modelname"]+'", '+datetime.datetime.now().strftime("%Y%m%d_%H:%M"))
        outputfile.write(yaml.dump(model, default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries

def xmlline(ind, tag, names, values):
    line = []
    line.append(indent*ind+'<'+tag)
    for i in range(len(names)):
        line.append(' '+names[i]+'="'+values[i]+'"')
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

def exportModelToURDF(model, filepath):
    output = []
    output.append(urdfHeader)
    output.append(indent+'<robot name="'+model["modelname"]+'">\n\n')
    #export link information
    for l in model["link"].keys():
        link = model["link"][l]
        output.append(indent*2+'<link name="'+l+'">\n')
        #output.append(xmlline(2, 'link', ['name'], [l]))
        output.append(indent*3+'<inertial>\n')
        #output.append(xmlline(4, 'origin', ['xyz', 'rpy'], [l2str(link["pose"][0:3]), l2str(link["pose"][3:])])) #this is the original code, now moved to joints
        #output.append(xmlline(4, 'origin', ['xyz', 'rpy'], ["0 0 0", "0 0 0"])) #TODO: add this if it's needed (pivot)
        output.append(xmlline(4, 'mass', ['value'], [str(link['inertial']['mass'])]))
        if "inertia" in link:
            output.append(xmlline(4, 'inertia', ['ixx', 'ixy', 'ixz', 'iyx', 'iyy', 'iyz'], map(float, link['inertial']['inertia'])))
        output.append(indent*3+'</inertial>\n')
        output.append(indent*3+'<visual>\n')
        #origin #TODO: offset of visual to real representation
        output.append(indent*4+'<geometry>\n')
        output.append(xmlline(5, 'mesh', ['filename', 'scale'], [link["filename"], '1.0']))
        output.append(indent*4+'</geometry>\n')
        if 'material' in link['visual']:
            output.append(indent*4+'<material name="' + link["visual"]["material"]["name"] + '">\n')
            output.append(indent*5+'<color rgba="'+l2str(link["visual"]["material"]["diffuseColor"]) + '1.0"/>\n')
            output.append(indent*4+'</material>\n')
        output.append(indent*3+'</visual>\n')
        output.append(indent*3+'<collision>\n')
        output.append(indent*4+'<geometry>\n')
        if link['collision']['geometry']['geometryType'] == "box":
            output.append(xmlline(5, 'box', ['size'], l2str(link["collision"]["geometry"]["size"])))
        elif link['collision']['geometry']['geometryType'] == "cylinder":
            output.append(xmlline(5, 'cylinder', ['radius', 'length'], [link["collision"]["geometry"]["radius"], link["collision"]["geometry"]["height"]]))
        elif link['collision']['geometry']['geometryType'] == "sphere":
            output.append(xmlline(5, 'cylinder', ['radius'], [link["collision"]["geometry"]["radius"]]))
        elif link['collision']['geometry']['geometryType'] == "mesh":
            output.append(xmlline(5, 'mesh', ['filename', 'scale'], [link["name"], '1.0']))#TODO correct this after implementing filename and scale properly
        output.append(indent*4+'</geometry>\n')
        output.append(indent*3+'</collision>\n')
        output.append(indent*2+'</link>\n\n')
    #export joint information
    for j in model["joint"]:
        joint = model["joint"][j]
        output.append(indent*2+'<joint name="'+j+'" type="'+joint["jointType"]+'">\n')#TODO: currently no floating joints are supported
        child = model["link"][joint["child"]]
        output.append(xmlline(3, 'origin', ['xyz', 'rpy'], [l2str(child["pose"][0:3]), l2str(child["pose"][3:])]))
        output.append(indent*3+'<parent link="'+joint["parent"]+'"/>\n')
        output.append(indent*3+'<child link="'+joint["child"]+'"/>\n')
        if "lowerConstraint" in joint:
            output.append(xmlline(3+'limit', ['lower', 'upper'], [joint["lowerConstraint"], joint["upperConstraint"]]))
        output.append(indent*2+'</joint>\n\n')
        #if "pose" in joint:
        #    output.append(indent*2+'<origin xyz="'+str(joint["pose"][0:3])+' rpy="'+str(joint["pose"][3:-1]+'/>\n')) #todo: correct lists and relative poses!!!
    #finish the export
    output.append(urdfFooter)
    with open(filepath, 'w') as outputfile:
        outputfile.write(''.join(output))
    #print(model["link"].keys())
    # problem of different joint transformations needed for fixed joints
    print("MARStools URDF export: Writing model data to", filepath )

def exportModelToSMURF(model, path): # Syntactically Malleable Universal Robot Format / Supplementable, Mostly URF / Supplement-Managed URF
    #create all filenames
    model_filename = os.path.expanduser(path + model["modelname"] + ".yml")
    urdf_filename = os.path.expanduser(path + model["modelname"] + ".urdf")
    materials_filename = os.path.expanduser(path + model["modelname"] + "_materials.yml")
    sensors_filename = os.path.expanduser(path + model["modelname"] + "_sensors.yml")
    motors_filename = os.path.expanduser(path + model["modelname"] + "_motors.yml")
    controllers_filename = os.path.expanduser(path + model["modelname"] + "_controllers.yml")
    simulation_filename = os.path.expanduser(path + model["modelname"] + "_simulation.yml")

    infostring = ' definition SMURF file for "'+model["modelname"]+', '+model["date"]+"\n"

    #write model information
    print('Writing SMURF information to...\n'+model_filename)
    modeldata = {}
    #modeldata["modelname"] = model["modelname"]
    modeldata["date"] = model["date"]
    modeldata["files"] = [urdf_filename, materials_filename,
                          sensors_filename, motors_filename,
                          controllers_filename, simulation_filename]
    with open(model_filename, 'w') as op:
        op.write('#main SMURF file of the model "'+model["modelname"]+"\n")
        op.write("modelname: "+model["modelname"]+"\n")
        op.write(yaml.dump(modeldata, default_flow_style=False))

    #write urdf
    exportModelToURDF(model, urdf_filename)

    #write materials
    with open(materials_filename, 'w') as op:
        op.write('#materials'+infostring)
        op.write("modelname: "+model["modelname"]+"\n")
        materialdata = {}
        for key in bpy.data.materials.keys():
            print("MARStools: processing material", key)
            mat = bpy.data.materials[key]
            materialdata[key] = {}
            materialdata[key]["color_diffuse"] = list(mat.diffuse_color)
            materialdata[key]["color_specular"] = list(mat.specular_color)
            materialdata[key]["alpha"] = mat.alpha
        op.write(yaml.dump(materialdata, default_flow_style=False))

    #write sensors
    with open(sensors_filename, 'w') as op:
        op.write('#sensors'+infostring)
        op.write("modelname: "+model["modelname"]+"\n")
        op.write(yaml.dump(model["sensor"], default_flow_style=False))

    #write motors
    with open(motors_filename, 'w') as op:
        op.write('#motors'+infostring)
        op.write("modelname: "+model["modelname"]+"\n")
        op.write(yaml.dump(model["motor"], default_flow_style=False))

    #write controllers
    with open(controllers_filename, 'w') as op:
        op.write('#controllers'+infostring)
        op.write("modelname: "+model["modelname"]+"\n")
        op.write(yaml.dump(model["controller"], default_flow_style=False))

    #write simulation
    with open(simulation_filename, 'w') as op:
        op.write('#simulation'+infostring)
        op.write("modelname: "+model["modelname"]+"\n")
        simulationdata = {}
        #TODO: handle simulationd-specific data
        op.write(yaml.dump(simulationdata, default_flow_style=False))

def exportSceneToSMURF(path):
    pass

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
        #TODO: add selection of all layers bpy.ops.object.select_all()
        if bpy.data.worlds[0].exportSMURF or bpy.data.worlds[0].exportURDF or bpy.data.worlds[0].exportYAML:
            main(yaml = bpy.data.worlds[0].exportYAML,
                              urdf = bpy.data.worlds[0].exportURDF,
                              smurf = bpy.data.worlds[0].exportSMURF)
        else:
            main()
        return {'FINISHED'}

def main(yaml=True, urdf=True, smurf=True):
    if yaml or urdf or smurf:
        robot = buildRobotDictionary()
        if yaml:
            outpath = securepath(os.path.expanduser(bpy.context.scene.world.path))
            exportModelToYAML(robot, outpath + robot["modelname"] + "_dict.yml")
        if smurf:
            outpath = securepath(os.path.expanduser(bpy.context.scene.world.path))
            exportModelToSMURF(robot, outpath)
        elif urdf:
            outpath = securepath(os.path.expanduser(bpy.context.scene.world.path))
            exportModelToURDF(robot, outpath + robot["modelname"] + ".urdf")


#allow manual execution of script in blender
if __name__ == '__main__':
    main()