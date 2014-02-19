'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtcreateprops.py

Created on 13 Feb 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.

NOTE: If you edit this script, please make sure not to use any imports
not supported by Blender's standard python distribution. This is a script
intended to be usable on its own and thus should not use external dependencies,
especially none on the other modules of the MARStools package.
'''

import bpy
import mathutils
import os
import yaml
import marstools.mtdefs as mtdefs
import marstools.mtutility as mtutility

xmlindent = '  '
urdfHeader = '<xml version="1.0">\n'
urdfFooter = '</xml>'

def calcPose(obj, center, type):
    pose = []
    if type == "link" or type == "visual" or type == "collision":
        pivot = center
        location = obj.location.copy()
        location += obj.matrix_world.to_quaternion() * mathutils.Vector((pivot[0], pivot[1], pivot[2]))

        obj.rotation_mode = 'QUATERNION'
        q = obj.rotation_quaternion

        if obj.parent:
            parent = obj.parent
            parentIQ = parent.matrix_world.to_quaternion().inverted()
            pivot2 = mtutility.calcBoundingBoxCenter(parent.bound_box)
            v = mathutils.Vector((pivot2[0], pivot2[1], pivot2[2]))
            #v = parent.matrix_world.to_quaternion() * v
            parentPos = parent.matrix_world * v
            childPos = obj.matrix_world * mathutils.Vector((pivot[0], pivot[1], pivot[2]))
            childPos = childPos - parentPos
            location = parentIQ * childPos
            parentRot = parent.matrix_world.to_quaternion()
            childRot = obj.matrix_world.to_quaternion()
            childRot = parentRot.rotation_difference(childRot)
            q = childRot
        pose = list(location)
        pose.extend(q)
#    elif type == "visual":
#        pass
#    elif type == "collision":
#        pass
    elif type == "joint":
        pos = mathutils.Vector((0.0, 0.0, 1.0))
        axis = obj.matrix_world.to_quaternion() * pos
        center = obj.matrix_world * mathutils.Vector((0.0, 0.0, 0.0))
        obj.rotation_mode = 'QUATERNION'
        v1 = obj.rotation_quaternion * mathutils.Vector((1.0, 0.0, 0.0))
        if obj["node2"] != "world":
            node2 = mtutility.getObjByName(obj["node2"])
            v2 = node2.rotation_quaternion * mathutils.Vector((1.0, 0.0, 0.0)) #TODO: link to other node in node2
        q = obj.rotation_quaternion.copy().inverted()
        pose = list(center)
        pose.extend(q)
    return pose

def deriveDictEntry(obj):
    props = {}
    #first get all custom properties (this is effectively a super-set of MARS-defined properties)
    print("MARStools: Deriving dictionary entry for", obj.name)
    for key in obj.keys():
        props[key] = obj[key]

    #pre-calculations
    bBox = obj.bound_box
    center = mtutility.calcBoundingBoxCenter(obj.bound_box)
    size = [0.0, 0.0, 0.0]
    size[0] = abs(2.0*(bBox[0][0] - center[0]))
    size[1] = abs(2.0*(bBox[0][1] - center[1]))
    size[2] = abs(2.0*(bBox[0][2] - center[2]))
    #now manage individual properties
    if obj.MARStype == "body":
        props["filename"] = obj.name + (".bobj" if bpy.context.scene.world.exportBobj else ".obj")
        props["pose"] = calcPose(obj, center, "link")
        #inertial #TODO: implement inertia calculation

        #collision object
        collision = {}
        if "collisionBitmask" in props:
            collision["bitmask"] = props["collisionBitmask"]
            del props["collisionBitmask"]
        else:
            collision["bitmask"] = mtdefs.type_properties["body_default"][mtdefs.type_properties["body"].index("collisionBitmask")] #TODO: this is just plain ugly
        collGeom = {}
        if "collisionPrimitive" in props:
            collGeom["collisionPrimitive"] = str(props["collisionPrimitive"])
            del props["collisionPrimitive"] #TODO ugly
        else:
            collGeom["collisionPrimitive"] = "box"
        if collGeom["collisionPrimitive"] == "sphere":
            collGeom["radius"] = max(size)/2
        elif collGeom["collisionPrimitive"] == "box":
            collGeom["size"] = size
        elif collGeom["collisionPrimitive"] == "cylinder":
            collGeom["radius"] = size[0]
            collGeom["height"] = size[1]
        elif collGeom["collisionPrimitive"] == "mesh":
            collGeom["size"] = size
        elif collGeom["collisionPrimitive"] == "plane":
            collGeom["size"] = [size[0], size[1]]
        collision["geometry"] = collGeom
        collision["pose"] = calcPose(obj, center, "collision")
        if "maxContacts" in props:
            collision["max_contacts"] = str(obj["maxContacts"])
            del props["maxContacts"] #TODO ugly
        else:
            collision["max_contacts"] = -1
        props["collision"] = collision

        #visual object
        visual = {}
        visual["pose"] = calcPose(obj, center, "visual")
        visual["material"] = {}
        visual["geometry"] = collision["geometry"]["collisionPrimitive"]
    elif obj.MARStype == "joint":
        props["parent"] = obj.parent.name
        props["child"] = props["node2"]
        del props["node2"]
        props["pose"] = calcPose(obj, 0, "joint") #TODO: the 0 is an ugly hack
    #clean dictionary entries
    getridof = ["MARStype", "_RNA_UI"]
    for key in getridof:
        if key in props:
            del props[key]
    return props


def buildRobotDictionary():
    robot = {"body": {},
            "joint": {},
            "sensor": {},
            "motor": {},
            "controller": {}}
    for obj in bpy.context.selected_objects:#bpy.data.objects:
        robot[obj.MARStype][obj.name] = deriveDictEntry(obj)
        #check if we need a fixed joint
        if obj.MARStype == "body" and obj.parent and obj.parent.MARStype == "body":
            fixedjoint = {}
            fixedjoint["name"] = obj.parent.name+"_fixedto_"+obj.name
            fixedjoint["parent"] = obj.parent.name
            fixedjoint["child"] = obj.name
            #fixedjoint["pose"] = (0,0,0,0,0,0) #calcPose(obj, 0, "body") #TODO; not sure what the difference is in this case
            fixedjoint["jointType"] = "fixed"
            robot["joint"][fixedjoint["name"]] = fixedjoint
        #check if we have a root object with a modelname
        if "modelname" in obj:
            robot["modelname"] = obj["modelname"]
    if not "modelname" in robot:
        robot["modelname"] = "robot"
    return robot

def exportModelToYAML(model, path):
    filename = os.path.expanduser(path + model["modelname"] + ".yaml")
    print("MARStools YAML export: Writing model data to", filename )
    with open(filename, 'w') as outputfile:
        outputfile.write(yaml.dump(model))

def xmlline(indent, tag, names, values):
    line = []
    line.append(xmlindent*indent+'<'+tag)
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

def exportModelToURDF(model, path):
    filename = os.path.expanduser(path + model["modelname"] + ".urdf")
    output = []
    output.append(urdfHeader)
    output.append(xmlindent+'<robot name="'+model["modelname"]+'">\n')
    for l in model["body"].keys():
        link = model["body"][l]
        output.append(xmlline(2, 'link', ['name'], [l]))
        output.append(xmlindent*3+'<inertial>\n')
        output.append(xmlline(4, 'origin', ['xyz', 'rpy'], [l2str(link["pose"][0:3]), l2str(link["pose"][3:-1])]))
        output.append(xmlline(4, 'mass', ['value'], [str(link["mass"])]))
        #inertia
        output.append(xmlindent*3+'</inertial>\n')
        output.append(xmlindent*3+'<visual>\n')
        #origin
        #geometry
        #material
        output.append(xmlindent*3+'</visual>\n')
        output.append(xmlindent*3+'<collision>\n')
        #geometry
        output.append(xmlindent*3+'</collision>\n')
        output.append(xmlindent*2+'</link>\n\n')
    for j in model["joint"]:
        joint = model["joint"][j]
        output.append(xmlindent*2+'<joint name="'+j+'" type="'+joint["jointType"]+'/>\n') #TODO: correct type
        output.append(xmlindent*3+'<parent link="'+joint["parent"]+'"/>\n')
        output.append(xmlindent*3+'<child link="'+joint["child"]+'"/>\n')
        output.append(xmlindent*2+'</joint>\n\n')
        #if "pose" in joint:
        #    output.append(xmlindent*2+'<origin xyz="'+str(joint["pose"][0:3])+' rpy="'+str(joint["pose"][3:-1]+'/>\n')) #todo: correct lists and relative poses!!!
    output.append(urdfFooter)
    with open(filename, 'w') as outputfile:
        outputfile.write(''.join(output))
    #print(model["body"].keys())
    # problem of different joint transformations needed for fixed joints
    print("MARStools URDF export: Writing model data to", filename )

def exportToSMURF(model, path): # Syntactically Malleable Universal Robot Format / Supplementable, Mostly URF / Supplement-Managed URF
    exportModelToURDF(model, path)
    urdf_filename = os.path.expanduser(path + model["modelname"] + ".urdf")
    materials_filename = os.path.expanduser(path + "materials_" + model["modelname"] + ".yaml")
    sensors_filename = os.path.expanduser(path + "sensors_" + model["modelname"] + ".yaml")
    motors_filename = os.path.expanduser(path + "motors_" + model["modelname"] + ".yaml")
    simulation_filename = os.path.expanduser(path + "simulation_" + model["modelname"] + ".yaml")

def main():
    robot = buildRobotDictionary()
    exportModelToYAML(robot, bpy.context.scene.world.path)
    exportModelToURDF(robot, bpy.context.scene.world.path)
    #exportModelToSMURF(robot, bpy.context.scene.world.path)


#allow manual execution of script in blender
if __name__ == '__main__':
    main()