import os
import sys
import math
import shutil
import zipfile
import textwrap

import xml.dom.minidom

import bpy
import mathutils

EPSILON = sys.float_info.epsilon

# enum mapping of indices onto descriptive strings
anchorPositions = {1: "node1",
                   2: "node2",
                   3: "center",
                   4 : "custom"
                  }

# enum of allowed light types
lightTypes = {1 : "POINT",
              2 : "SPOT"
             }

# global list of nodes, joints and materials
nodeList = []
jointList = []
sensorList = []
materialList = []
controllerList = []
lightList = []

# global dictionary for mapping of names of imported .obj file
# unto the first node who imported it (used also for just having
# one cylindrical mesh for the joint helper objects)
objFileMap = {}

# global list of imported but not used objects
unusedNodeList = []

# map of keyword differences between MARS .scn and Blender for nodes
nodeKeyMap = {"groupid" : "group",
              "index" : "id"
             }

# map of keyword differences between MARS .scn and Blender for joints
# (change the identifier "anchor" and "anchorpos" for the export script)
jointKeyMap = {"index" : "id",
               "type" : "jointType",
               "anchor" : "anchorpos",
               "anchorpos" : "anchor"
              }

# map of keyword differences between MARS .scn and Blender for sensors
sensorKeyMap = {"type" : "sensorType"
               }


# map of keyword differences between MARS .scn and Blender for materials
materialKeyMap = {"id" : "marsID"
                 }

# map of keyword differences between MARS .scn and Blender for motors
motorKeyMap = {"name" : "motor_name",
               "index" : "motor_index",
               "type" : "motor_type",
               "axis" : "motorAxis",
               "maximumVelocity" : "maxSpeed",
               "motorMaxForce" : "maxForce",
               "min_val" : "lowStop",
               "max_val" : "highStop"
              }

# delete all objects, meshes and material from Blender
def cleanUpScene():
    # select all objects
    bpy.ops.object.select_all(action="SELECT")

    # and delete them
    bpy.ops.object.delete()

    # after that we have to clean up all loaded meshes (unfortunately
    # this is not done automatically)
    for mesh in bpy.data.meshes:
        bpy.data.meshes.remove(mesh)

    # and all materials
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)

    # and all lights (aka lamps)
    for lamp in bpy.data.lamps:
        bpy.data.lamps.remove(lamp)

    # clear global parameters (needed when importing this file and trying
    # to load multiple .scn files)
    nodeList.clear()
    jointList.clear()
    sensorList.clear()
    materialList.clear()
    controllerList.clear()
    lightList.clear()
    objFileMap.clear()
    unusedNodeList.clear()


# clean up all the "empty" whitespace nodes
def removeWhitespaceNodes(parent, unlink=True):
    remove_list = []

    for child in parent.childNodes:
        if child.nodeType == xml.dom.minidom.Node.TEXT_NODE and \
           not child.data.strip():
            remove_list.append(child)
        elif child.hasChildNodes():
            removeWhitespaceNodes(child, unlink)

    for node in remove_list:
        node.parentNode.removeChild(node)
        if unlink:
            node.unlink()


# convert the given xml structure into a python dict
def getGenericConfig(parent):
    child = parent.firstChild
    if not child:
        return None
    # if it is a text node, we just return the contained value
    elif (child.nodeType == xml.dom.minidom.Node.TEXT_NODE):
        # check if it is an integer?
        try:
            return int(child.nodeValue)
        except ValueError:
            pass

        # or a float?
        try:
            return float(child.nodeValue)
        except ValueError:
            pass

        # if it is a string, remove leading and trailing
        # white space characters
        string = child.nodeValue.strip()

        # see if it is the boolean 'True'?
        if string in ["true", "True"]:
            string = True

        # or the boolean 'False'?
        if string in ["false", "False"]:
            string = False

        return string

    config = {}

    # check for attributes ...
    if parent.hasAttributes():
        for key, value in parent.attributes.items():
#            print("attrib [%s : %s]" % (key, value))
            if key not in config:
                config[key] = value
            else:
                print("Warning! Key '%s' already exists!" % key)

    # and check the appending child nodes
    while child is not None:
        if (child.nodeType == xml.dom.minidom.Node.ELEMENT_NODE):
            key   = child.tagName
            value = getGenericConfig(child)
#            print("element [%s : %s]" % (key, value))
            if key not in config:
                config[key] = value
            else:
                # while parsing there could be multiple entries
                # for "id" (sensor), "sensorid" or "motorid" (both
                # controller)
                if key == "id" or \
                   key == "sensorid" or \
                   key == "motorid":
                    if isinstance(config[key],int):
                        config[key] = [config[key]]
                    config[key].append(value)
                else:
                    print("Warning! Key '%s' already exists!" % child.tagName)

        child = child.nextSibling

    # check if it is a vector
    if isinstance(config,dict) and sorted(config.keys()) == ["x","y","z"]:
        # and transform it into a Blender Vector
        config = mathutils.Vector((config["x"],
                                   config["y"],
                                   config["z"]))

    # check if it is a quaternion
    if isinstance(config,dict) and sorted(config.keys()) == ["w","x","y","z"]:
      # and transform it into a Blender Quaternion
      config = mathutils.Quaternion((config["w"],
                                     config["x"],
                                     config["y"],
                                     config["z"]))

    # check if it is a color
    if isinstance(config,dict) and sorted(config.keys()) == ["a","b","g","r"]:
        # and transform it into a Blender Color
        config = mathutils.Color((config["r"],
                                  config["g"],
                                  config["b"]))

    return config


def checkConfigParameter(config, key):
    if key not in config.keys():
        print("WARNING! Config does not contain parameter \'%s\'!" % key)
        return False
    return True


def setParentChild(parent, child):
    if parent not in nodeList:
        print("WARNING! Unable to set parent-child relationship! Parent non-existing! Parent must be a node!")
        return False

    if child not in nodeList and child not in jointList and child not in sensorList:
        print("WARNING! Unable to set parent-child relationship! Child non-existing! Child must be either a node,a joint or a sensor!")
        return False

    # de-select all objects
    bpy.ops.object.select_all(action="DESELECT")

    # select the child
    child.select = True

    # select the parent
    parent.select = True

    # set the parent to be the currently active object
    bpy.context.scene.objects.active = parent

    # set the parent-child relationship
    bpy.ops.object.parent_set(type="OBJECT")

    return True


def duplicateObject(original, linked=True):
    # de-select all currently selected nodes
    bpy.ops.object.select_all(action="DESELECT")

    # select the original object
    original.select = True

    # and set it as the currently active object
    bpy.context.scene.objects.active = original

    # and link duplicate it (which makes a copy of the given
    # object using the same meshes and materials but have an
    # own transformation)
    bpy.ops.object.duplicate(linked=True)

    # store the pointer to the copied object
    copy = bpy.context.selected_objects[0]

    # clear the parent of the copied object
    bpy.ops.object.parent_clear()

    # clear all custom properties (just to be sure to don't
    # have some properties belonging to the original but were
    # not present in the copy)
    for key in copy.keys():
        del copy[key]

    return copy


def centerNodeOrigin(node):
    if node.name not in bpy.data.objects:
        print("WARNING! Unable to center the origin of node <%s>! Node does not exist!" % node.name)
        return False

    # de-select all objects
    bpy.ops.object.select_all(action="DESELECT")

    # select the node/object
    node.select = True

    # set the parent to be the currently active object
    bpy.context.scene.objects.active = node

    # set the origin of the mesh to the center of its
    # bounding box
    bpy.ops.object.origin_set(type="ORIGIN_GEOMETRY",
                              center="BOUNDS")

    return True

def centerAllNodeOrigin():

    # select all objects
    bpy.ops.object.select_all(action="SELECT")

    # set the origin of the mesh to the center of its
    # bounding box
    bpy.ops.object.origin_set(type="ORIGIN_GEOMETRY",
                              center="BOUNDS")
    # de-select all objects
    bpy.ops.object.select_all(action="DESELECT")

    return True

def calculateCenter(boundingBox):
    c = mathutils.Vector()
    for v in boundingBox:
        for i in range(3):
            c[i] += v[i]
    return c / 8.0


def parseMaterial(domElement):
    # read the config from the xml file
    config = getGenericConfig(domElement)

    # check for the material index
    if not checkConfigParameter(config,"id"):
        return False
    marsID = config["id"]

    # create a name out of the material index (not needed in MARS
    # but here in Blender)
    name = "mars_material_%u" % marsID

#    print("# Creating material <%s>" % name)

    # TODO: Is this really necessary? Right now before we start we
    # remove all objects, meshes and materials!?
    # check if a material with the same name already does exist
    if name in bpy.data.materials:
        # when it exists remove it (just to wipe the plate clear)
        tmp = bpy.data.materials[name]
        bpy.data.materials.remove(tmp)

    # create a new material
    material = bpy.data.materials.new(name)

    # add the material to the global list
    materialList.append(material)

    # add each item of 'config' as a custom property to the material
    for (key, value) in config.items():
        if key in materialKeyMap:
            material[materialKeyMap[key]] = value
        else:
            material[key] = value

    # set the diffuse color
    if checkConfigParameter(config,"diffuseFront"):
        material.diffuse_color = config["diffuseFront"]

    # set the specular color
    if checkConfigParameter(config,"specularFront"):
        material.specular_color = config["specularFront"]

    # set the shininess
    if checkConfigParameter(config,"shininess"):
        material.specular_hardness = 2 * config["shininess"]

    # TODO: what about transparency?!?

    return True


def parseNode(domElement, tmpDir):
    # read the config from the xml file
    config = getGenericConfig(domElement)

    # handle node name
    if not checkConfigParameter(config,"name"):
        return False
    name = config["name"]

#    print("# Creating node <%s>" % name)

#    print("%s : %s" % (config["name"], config))

    # handle node physic mode
    if checkConfigParameter(config,"physicmode"):
        typeName = config["physicmode"]
        if typeName in nodeTypes:
            physicMode = nodeTypes.index(typeName)
        else:
            print("ERROR! Could not get type for node: %s" % name)

    if checkConfigParameter(config,"origname"):
        origName = config["origname"]

    if checkConfigParameter(config,"filename"):
        filename = config["filename"]

    if filename == "PRIMITIVE":
        if not origName:
            origName = nodeTypes[physicMode]
        elif origName != nodeTypes[physicMode]:
            tmp = nodeTypes[physicMode]
            print("WARNING! Origname set to \"%s\" for primitive in node \"%s\" with physicMode \"%s\"" % (origName, name, tmp))

    if checkConfigParameter(config,"index"):
        index = int(config["index"])

    if checkConfigParameter(config,"position"):
        position = config["position"]
    else:
        position = mathutils.Vector()

    if checkConfigParameter(config,"rotation"):
        rotation = config["rotation"]
    else:
        rotation = mathtutils.Quaternion()
        rotation.identity()

    if checkConfigParameter(config,"extend"):
        extend = config["extend"]
    else:
        extend = mathutils.Vector()

    if checkConfigParameter(config,"pivot"):
        pivot = config["pivot"]
    else:
        pivot = mathutils.Vector()

    if checkConfigParameter(config, "visualposition"):
        visual_position = config["visualposition"]
    else:
        visual_position = mathutils.Vector()

    if checkConfigParameter(config, "visualrotation"):
        visual_rotation = config["visualrotation"]
    else:
        visual_rotation = mathutils.Quaternion()
        visual_rotation.identity()

    if checkConfigParameter(config,"visualsize"):
        visual_size = config["visualsize"]
    else:
        visual_size = extend

    ######## LOAD THE NODE IN BLENDER ########

    # "pointer" to the newly created object
    node = None

    if filename == "PRIMITIVE":

        if typeName == "box":
            # create a new box as representation of the node
            bpy.ops.mesh.primitive_cube_add()
            # get the "pointer" to the new node
            node = bpy.context.selected_objects[0]
            # set the size of the cube
            node.dimensions = extend

        elif typeName == "sphere":
            # create a new sphere as representation of the node
            bpy.ops.mesh.primitive_uv_sphere_add(size = extend.x)
            # get the "pointer" to the new node
            node = bpy.context.selected_objects[0]

        elif typeName == "reference":
            # TODO: is that really needed?
            print("Warning! Unhandled node type \'reference\'.")

        elif typeName == "mesh":
            # TODO: is that really needed?
            print("Warning! Unhandled node type \'mesh\'.")

        elif typeName == "cylinder":
            # create a new cylinder as representation of the node
            bpy.ops.mesh.primitive_cylinder_add(radius = extend.x, depth = extend.y)
            # get the "pointer" to the new node
            node = bpy.context.selected_objects[0]

        elif typeName == "capsule":
            print("Warning! Node type \'capsule\' yet supported, using \'cylinder\' instead.")
            # create a new cylinder as representation of the node
            bpy.ops.mesh.primitive_cylinder_add(radius = extend.x, depth = extend.y)
            # get the "pointer" to the new node
            node = bpy.context.selected_objects[0]

        elif typeName == "plane":
            # create a new plane as representation of the node
            bpy.ops.mesh.primitive_plane_add()
            # get the "pointer" to the new node
            node = bpy.context.selected_objects[0]
            # set the size of the cube
            node.dimensions = extend

        else:
            print("Cannot find primitive type: %s" % origName)

    elif physicMode == "terrain":
        # TODO: Creating terrain in Blender ...
        print("Warning! \'Terrain\' nodes are not supported right now! Using a small box as a placeholder!")
        # create a new box as placeholder for the "terrain" node
        bpy.ops.mesh.primitive_cube_add()
        # get the "pointer" to the new node
        node = bpy.context.selected_objects[0]
        # set the size of the cube
        node.dimensions = mathutils.Vector((0.1,0.1,0.1))

    # we have to load the node from an import file
    else:
        # check whether the object was previously already imported
        # and is not used
        for nodeName in unusedNodeList:
            if filename in nodeName or name in nodeName:
                node = bpy.data.objects[nodeName]
                # remove the current object from the unused list
                unusedNodeList.remove(nodeName)

        # see if the mesh was already imported for another node
        if not node:
            # if the mesh was already imported
            if filename in objFileMap:
                # get the other object which already imported the same
                # .obj file
                tmp = objFileMap[filename]

                # duplicate the given object
                node = duplicateObject(tmp)

        # if not, import the respective .obj file
        if not node:

            # import the .obj file (after importing a .obj file the newly
            # added objects are selected by Blender)
            bpy.ops.import_scene.obj(filepath=tmpDir+os.sep+filename)

            # apply the rotation to the imported object (is needed in
            # order to get the right orientation of the imported object)
            bpy.ops.object.transform_apply(rotation=True)

            # if there were added multiple meshes from one .obj file
            if len(bpy.context.selected_objects) > 1:
                # store the list of newly added meshes
                new_object_list = bpy.context.selected_objects

                # put the newly imported objects into the "unused" list
                for tmp in new_object_list:
                    # add the name of the imported object to the list of
                    # unused nodes
                    unusedNodeList.append(tmp.name)

                    # get the currently added object
                    if filename in tmp.name or name in tmp.name:
                        node = bpy.data.objects[tmp.name]
                        # remove the current object from the unused list
                        unusedNodeList.remove(tmp.name)

            # if there was added just one mesh
            else:
                # get the newly added node as the only selected object
                node = bpy.context.selected_objects[0]

                # store the mapping between the loaded mesh file and a pointer
                # to the first node which imported it
                objFileMap[filename] = node

        else:
            print("WARNING! Mesh \'%s\' already imported! Skipping second import!" % name)

        # whenever we decrease the size of a mesh (dimensions > visual_size),
        # we can set the new size just now
        # TODO: Find out why!?!
        if visual_size.x < node.dimensions.x or \
           visual_size.y < node.dimensions.y or \
           visual_size.z < node.dimensions.z:
            # set the size of the object
            node.dimensions = visual_size

        # apply the scaling to the node (needed for the export script to work;
        # can only be applied to single user meshes, states an error when using
        # with a mesh with used by multiple objects)
        if node.data.users == 1:
            # de-select all objects
            bpy.ops.object.select_all(action="DESELECT")
            # select the node/object
            node.select = True
            # set it as the currently active object
            bpy.context.scene.objects.active = node
            # apply the scaling (will set all scaling factors to 1.0)
            bpy.ops.object.transform_apply(scale=True)

        # set the scale of the object (should be always 1.0!?)
        node["sizeScaleX"] = node.scale[0]
        node["sizeScaleY"] = node.scale[1]
        node["sizeScaleZ"] = node.scale[2]

    # Set the parameter, orientation and position of the new node
    if node:

        # add the node to the global node list
        nodeList.append(node)

        # set the name of the object
        node.name = name

        # set the object type to be a node
        node["type"] = "body"

        # if there is no "groupid" in the config set the default value of zero
        if not checkConfigParameter(config,"groupid"):
            node["group"] = 0

        # add each item of 'config' as a custom property to the node
        for (key, value) in config.items():
            if key in nodeKeyMap:
                node[nodeKeyMap[key]] = value
            else:
                node[key] = value

        # if the node should be positioned "relative" to some other node
        if checkConfigParameter(config,"relativeid"):
            # get the ID of this "relative" node
            relativeID = config["relativeid"]

            # find the "relative" node
            relative = None
            for tmp in nodeList:
                if tmp["id"] == relativeID:
                    relative = tmp

            # if the "relative" node was found
            if relative:
                # calculate the absolute position based on the "relative" node's
                # position, the center of its bounding box (could be different
                # than its orign) and the given position
                position = relative.location + \
                           relative.rotation_quaternion * \
                           (calculateCenter(relative.bound_box) + position)

                # calculate the absolute orientation based on the "relative" node's
                # orientation and the given orientation
                rotation = relative.rotation_quaternion * rotation
            else:
                print("WARNING! Could not find relative node (id: %u)!" % relativeID)

        # set the position of the object
        node.location = position + rotation * (visual_position - pivot)

        # set the rotation of the object
        node.rotation_mode = "QUATERNION"
        node.rotation_quaternion = rotation * visual_rotation

        # whenever we increase the size of a mesh (dimensions < visual_size),
        # we have to reset the origin first before we can set the new size
        # TODO: Find out why!?!
        if visual_size.x > node.dimensions.x or \
           visual_size.y > node.dimensions.y or \
           visual_size.z > node.dimensions.z:
            # center the origin of the loaded node to the center
            # of its bounding box
            centerNodeOrigin(node)

            # set the size of the object. This have to be done AFTER
            # centerNodeOrigin()!!
            node.dimensions = visual_size

        # if a node is linked to a material
        if checkConfigParameter(config,"material_id"):
            # get the node ID
            materialID = config["material_id"]

            # find the corresponding material
            for material in materialList:
                # and also test if it is already set (happens when we duplicate an object)
                if material["marsID"] == materialID and material != node.active_material:
                    # store the pointer to the old material (if there is
                    # one)
                    tmp = None
                    if node.active_material != None:
                        tmp = node.active_material

                    # add the new material to the node
                    node.active_material = material

                    # clean up the "old" material (default material added
                    # by importing the .obj file without .mtl file; if you
                    # have multiple objects using the same material, e.g.
                    # loading multiple meshes from the same .obj file, the
                    # last one deletes the material)
                    if tmp and tmp.users == 0:
                        bpy.data.materials.remove(tmp)

    else:
        print("ERROR! Something went wrong while creating node \'%s\'" % name)
        return False

    return True


def parseJoint(domElement):
    # read the config from the xml file
    config = getGenericConfig(domElement)

    # handle joint name
    if not checkConfigParameter(config,"name"):
        return False
    name = config["name"]

#    print("# Creating joint <%s>" % name)

    # handle joint type
    if checkConfigParameter(config,"type"):
        typeName = config["type"]
        if typeName in jointTypes:
            type = jointTypes.index(typeName)
            if type == 0:
                print("JointData: type given is undefined for joint \'%s\'" % name)
        else:
            print("JointData: no type given for joint \'%s\'" % name)

    if checkConfigParameter(config,"index"):
        index = config["index"]

    if checkConfigParameter(config,"nodeindex1"):
        nodeIndex1 = config["nodeindex1"]
        if not nodeIndex1:
            print("JointData: no first node attached to joint \'%s\'" % name);

    if checkConfigParameter(config,"nodeindex2"):
        nodeIndex2 = config["nodeindex2"]

    # handle axis 1
    if checkConfigParameter(config,"axis1"):
        axis1 = config["axis1"]

    if checkConfigParameter(config,"anchorpos"):
        anchorPos = config["anchorpos"]
        # check if it is an integer
        if isinstance(anchorPos, int):
            # check if it is a defined integer value
            if anchorPos in anchorPositions.keys():
                # if so, we have to convert it to the respective string
                anchorPos = anchorPositions[anchorPos]
                # write it back to the config for later use
                config["anchorpos"] = anchorPos

    if checkConfigParameter(config,"anchor"):
        anchor = config["anchor"]


    ######## LOAD THE JOINT IN BLENDER ########

    if type != jointTypes.index("fixed"):
        # check if a joint helper object was already created
        if "joint_helper" not in objFileMap:
            # if not, create a new cylinder as representation of the joint
            bpy.ops.mesh.primitive_cylinder_add(radius=0.01, depth=0.2)

            # get the "pointer" to the new joint
            joint = bpy.context.selected_objects[0]

            # register the joint helper object with the global .obj file
            # map (this is not a hundred percent save; but importing an
            # .obj file named "joint_helper" should never happen)
            objFileMap["joint_helper"] = joint
        else:
            # else get the original joint helper object
            tmp = objFileMap["joint_helper"]

            # and duplicate the object
            joint = duplicateObject(tmp)

        # add the node to the global node list
        jointList.append(joint)

        # set the name of the object
        joint.name = name

        # set the object type to be a joint
        joint["type"] = "joint"

        # add each item of 'config' as a custom property to the joint
        for (key, value) in config.items():
            if key in jointKeyMap:
                joint[jointKeyMap[key]] = value
            else:
                joint[key] = value

        # set the color of the joint helper object to green
        if "joint_material" not in bpy.data.materials:
            # create new "green" material
            mat = bpy.data.materials.new("joint_material")
            mat.diffuse_color = mathutils.Color((0.0,
                                                 1.0,
                                                 0.0))
            mat.diffuse_shader = "LAMBERT"
            mat.diffuse_intensity = 0.6
            mat.specular_color = mathutils.Color((0.208,
                                                  0.208,
                                                  0.208))
            mat.specular_shader = "COOKTORR"
            mat.specular_intensity = 0.5
            mat.alpha = 1.0
            mat.ambient = 1.0
        else:
            mat = bpy.data.materials["joint_material"]

        joint.active_material = mat

        # check whether 'axis1' is valid and the type is not 'fixed'
        if axis1.length_squared < EPSILON and type != 6:
            print("ERROR! Cannot create joint \'%s\' without axis1" % name)
            #TODO: remove created cylinder
            return False

        # find the corresponding two nodes
        node1 = None
        node2 = None

        for tmp in nodeList:
            # check for thr right "ids"
            if tmp["id"] == nodeIndex1:
                node1 = tmp
            if tmp["id"] == nodeIndex2:
                node2 = tmp

        # determine the anchor position of the joint
        if anchorPos == "node1": # (1) "node1"
            joint.location = node1.location
        elif anchorPos == "node2": # (2) "node2"
            joint.location = node2.location
        elif anchorPos == "center": # (3) "center"
            joint.location = (node1.location + node2.location) / 2.0
        elif anchorPos == "custom": # (4) "custom"
            joint.location = anchor
        else:
            #TODO: What position should be set in this case?
            print("WARNING! Wrong anchor position for joint \'%s\'" % name)

        # set the orientation of the joint
        z_axis = mathutils.Vector((0.0,0.0,1.0))
        joint.rotation_mode = "QUATERNION"
        joint.rotation_quaternion = z_axis.rotation_difference(axis1)

        # setting up the node hierarchy (between parent and child node)
        if node1 and node2:
            # set the parent-child relationship
            setParentChild(node1,node2)

        # setting up the node hierarchy (between parent node and joint helper)
        if node1:
            # set the parent-child relationship
            setParentChild(node1,joint)

        # store the pointer to the second joint node as custom property
        if node2:
            joint["node2"] = node2.name

    # if it is a 'fixed' joint, we don't create a helper object, we just create
    # the parent-child-relationship and set the groupID of both objects accordingly
    else:
        # find the corresponding two nodes
        node1 = None
        node2 = None

        for tmp in nodeList:
            # check for thr right "ids"
            if tmp["id"] == nodeIndex1:
                node1 = tmp
            if tmp["id"] == nodeIndex2:
                node2 = tmp

        # setting up the node hierarchy (between parent and child node)
        if node1 and node2:
            # set the parent-child relationship
            setParentChild(node1,node2)

            # check whether the groupID of both nodes differ (highly probable,
            # if used in combination with a joint!)
            if node1["group"] != node2["group"]:
                # some helper variables for the groupIDs of node1 and node2
                groupID1 = node1["group"]
                groupID2 = node2["group"]
                # see if there are other nodes with the same groupID as the child
                for tmp in nodeList:
                    if tmp["group"] == groupID2:
                        tmp["group"] = groupID1

    return True


def checkGroupIDs():
#    print("# Checking group IDs")

    # check all nodes ...
    for node1 in nodeList:
        # objects with group ID zero are ignored because they are handled
        # seperately by MARS (not as one object consisting of multiple nodes)
        if node1["group"] == 0:
            continue

        # put all nodes with the same group ID together in a list
        group = []
        for node2 in nodeList:
            # check for matching group IDs
            if node2["group"] == node1["group"]:
                group.append(node2)

        # if there are other nodes with the same group ID
        if len(group) > 1:
            # see if the nodes with the same group ID are either parents
            # or children of a joint
            parents  = []
            children = []
            for node2 in group:
                for joint in jointList:
                    if joint["node2"] == node2.name and node2 not in children:
                        children.append(node2)
                    if joint.parent == node2 and node2 not in parents:
                        parents.append(node2)

            # if there is only one child, we use this as parent for the whole
            # group (would make sense)
            if len(children) == 1:
                # set the parent-child relationship
                node2 = children[0]
                setParentChild(node2,node1)
            # if there are no children, we use the "first" parent as "group parent"
            elif len(children) == 0 and len(parents) > 0:
                # set the parent-child relationship
                node2 = parents[0]
                setParentChild(node2,node1)
            else:
                print("WARNING! Unable to set parent-child relationship for node <%s> while checking group IDs!" % node1.name)


def parseMotor(domElement):
    # read the config from the xml file
    config = getGenericConfig(domElement)

    # check for the joint index (because we just store all the motor
    # information within the joint helper object)
    if not checkConfigParameter(config,"jointIndex"):
        return False
    jointIndex = config["jointIndex"]

#    print("# Creating motor <%s>" % config["name"])

    # find the joint in question
    for tmp in jointList:
        if tmp["id"] == jointIndex:
            joint = tmp

    # when the joint was found
    if joint:
        # remove the additional joint index from the config
        del config["jointIndex"]

        # add each item of 'config' as a custom property to the joint
        for (key, value) in config.items():
            if key in motorKeyMap:
                joint[motorKeyMap[key]] = value
            else:
                joint[key] = value

    return True


def parseSensor(domElement):
    # read the config from the xml file
    config = getGenericConfig(domElement)

    # handle sensor name
    if not checkConfigParameter(config,"name"):
        return False

    name = config["name"]

    # check if right sensor type
    if checkConfigParameter(config,"type"):
        if config["type"] not in sensorTypes:
            print("WARNING! Unhandled sensor type <%s> encountered! Skipping sensor!" % config["type"])
            return False

#    print("# Adding sensor <%s>" % name)

    # check if a sensor helper object was already created
    if "sensor_helper" not in objFileMap:
        # if not, create a new sphere as representation of the sensor
        bpy.ops.mesh.primitive_uv_sphere_add(size=0.02)

        # get the "pointer" to the new sensor
        sensor = bpy.context.selected_objects[0]

        # register the sensor helper object with the global .obj file
        # map (this is not a hundred percent save; but importing an
        # .obj file named "sensor_helper" should never happen)
        objFileMap["sensor_helper"] = sensor
    else:
        # else get the original sensor helper object
        tmp = objFileMap["sensor_helper"]

        # and duplicate the object
        sensor = duplicateObject(tmp)

    # when the joint was found
    if sensor:
        # add the node to the global sensor list
        sensorList.append(sensor)

        # set the name of the object to be the same as the senor
        sensor.name = name

        # set the correct object type to "sensor"
        sensor["type"] = "sensor"

        # add each item of 'config' as a custom property to the joint
        for (key, value) in config.items():
            if key in sensorKeyMap:
                sensor[sensorKeyMap[key]] = value
            else:
                sensor[key] = value

        # set the color of the joint helper object to yellow
        if "sensor_material" not in bpy.data.materials:
            # create new "yellow" material
            mat = bpy.data.materials.new("sensor_material")
            mat.diffuse_color = mathutils.Color((1.0,
                                                 1.0,
                                                 0.0))
            mat.diffuse_shader = "LAMBERT"
            mat.diffuse_intensity = 0.6
            mat.specular_color = mathutils.Color((0.208,
                                                  0.208,
                                                  0.208))
            mat.specular_shader = "COOKTORR"
            mat.specular_intensity = 0.5
            mat.alpha = 1.0
            mat.ambient = 1.0
        else:
            mat = bpy.data.materials["sensor_material"]

        sensor.active_material = mat

        if config["type"] == "NodeContactForce":
            # get the node ID of the belonging object
            nodeID = config["id"]
            node = None
            for tmp in nodeList:
                if tmp["id"] == nodeID:
                    node = tmp
                    break

            # if the node exists
            if node:
                # put the sensor helper object 10 cm under the object
                # TODO: is that a good position
                sensor.location = node.location + mathutils.Vector((0.0,0.0,-0.1))

                # if the position is already taken by another sensor
                # move the new sensor a little bit down
                for tmp in sensorList:
                    distance = sensor.location - tmp.location
                    if tmp != sensor and distance.length < 0.03:
                        sensor.location = sensor.location + mathutils.Vector((0.0,0.0,-0.05))

                # set the sensor helper object as child for the given object
                setParentChild(node,sensor)

            else:
                print("WARNING! For sensor <%s> node (%d) does not exist!" %(name, nodeID))

        elif config["type"] == "Joint6DOF":
            # get the node ID of the belonging object
            nodeID = config["nodeID"]
            node = None
            for tmp in nodeList:
                if tmp["id"] == nodeID:
                    node = tmp
                    break

            # if the node exists
            if node:
                # put the sensor helper object 10 cm under the object
                # TODO: is that a good position
                sensor.location = node.location + mathutils.Vector((0.0,0.0,0.1))

                # if the position is already taken by another sensor
                # move the new sensor a little bit up
                for tmp in sensorList:
                    distance = sensor.location - tmp.location
                    if tmp != sensor and distance.length < 0.03:
                        sensor.location = sensor.location + mathutils.Vector((0.0,0.0,0.05))

                # set the sensor helper object as child for the given object
                setParentChild(node,sensor)

            else:
                print("WARNING! For sensor <%s> node (%d) does not exist!" %(name, nodeID))

        elif config["type"] == "NodePosition" or \
             config["type"] == "NodeRotation" or \
             config["type"] == "NodeAngularVelocity" or \
             config["type"] == "NodeVelocity":
            # get the node ID of the belonging object
            nodeID = config["id"]
            node = None
            for tmp in nodeList:
                if tmp["id"] == nodeID:
                    node = tmp
                    break

            # if the node exists
            if node:
                # put the sensor helper object 10 cm under the object
                # TODO: is that a good position
                sensor.location = node.location + mathutils.Vector((0.0,0.0,0.1))

                # if the position is already taken by another sensor
                # move the new sensor a little bit up
                for tmp in sensorList:
                    distance = sensor.location - tmp.location
                    if tmp != sensor and distance.length < 0.03:
                        sensor.location = sensor.location + mathutils.Vector((0.0,0.0,0.05))

                # set the sensor helper object as child for the given object
                setParentChild(node,sensor)

            else:
                print("WARNING! For sensor <%s> node (%d) does not exist!" %(name, nodeID))

        elif config["type"] == "JointPosition" or \
             config["type"] == "JointVelocity" or \
             config["type"] == "MotorCurrent":
            # get the node ID of the belonging object
            nodeID = config["id"]
            #if it is a list, just pick the first item
            if isinstance(nodeID,list):
                nodeID = nodeID[0]
            node = None
            for tmp in nodeList:
                if tmp["id"] == nodeID:
                    node = tmp
                    break

            # if the node exists
            if node:
                # put the sensor helper object 10 cm under the object
                # TODO: is that a good position
                sensor.location = node.location + mathutils.Vector((0.0,0.0,0.1))

                # if the position is already taken by another sensor
                # move the new sensor a little bit up
                for tmp in sensorList:
                    distance = sensor.location - tmp.location
                    if tmp != sensor and distance.length < 0.03:
                        sensor.location = sensor.location + mathutils.Vector((0.0,0.0,0.05))

                # set the sensor helper object as child for the given object
                setParentChild(node,sensor)

            else:
                print("WARNING! For sensor <%s> node (%d) does not exist!" %(name, nodeID))
        else:
            print("WARNING! Unhandled sensor type <%s>!" % config["type"])

    return True


def parseController(domElement):
    # read the config from the xml file
    config = getGenericConfig(domElement)

    if not checkConfigParameter(config,"index"):
        # get the number of available controllers as the controller ID
        controllerID = len(controllerList)
    else:
        controllerID = config["index"]

#    print("# Adding controller <%d>" % controllerID)

    # due to the fact that we don't have an actual helper object,
    # we just store the whole config in the global list
    controllerList.append(config)

    # see if the controller contains sensors
    if checkConfigParameter(config,"sensorid"):
        sensorIDs = config["sensorid"]
    else:
        sensorIDs = []

    # see if the controller contains motors
    if checkConfigParameter(config,"motorid"):
        motorIDs = config["motorid"]
    else:
        motorIDs = []

    # add the controller ID to the contained sensors
    for sensor in sensorList:
        if sensor["index"] in sensorIDs:
            sensor["controllerIndex"] = controllerID

    # add the controller ID to the contained motors
    for joint in jointList:
        if "motor_index" in joint and joint["motor_index"] in motorIDs:
            joint["controllerIndex"] = controllerID

    return True

def parseLight(domElement):
    # read the config from the xml file
    config = getGenericConfig(domElement)

    # handle light name
    if not checkConfigParameter(config,"name"):
        return False
    name = config["name"]

    # Get the type of the light
    if checkConfigParameter(config,"type"):
        lightType = config["type"]
        # check if it is an integer
        if isinstance(lightType, int):
            # and one of the allowed integers
            if lightType in lightTypes.keys():
                # map it to the corresponding string
                lightType = lightTypes[lightType]

    # check if the light types is one of the allowed ones
    if lightType not in lightTypes.values():
        print("WARNING! Unrecognized light type \"%s\"! Using \"POINT\" type!" % lightType)
        # otherwise set it to be a point light
        lightType = "POINT"

    # position the lamp to a specified location
    if checkConfigParameter(config,"position"):
        position = config["position"]
    else:
        position = mathutils.Vector()

    # create the new light as "lightType" and at the given "position"
    bpy.ops.object.lamp_add(type=lightType, location=position)

    # get the pointer to the new light
    light = bpy.context.selected_objects[0]

    # set the right name for the lamp
    light.name = name

    # calculate the direction (orientation) of the light
    if checkConfigParameter(config,"lookat"):
        # get the value from the config
        lookat = config["lookat"]

        # calculate the quaternion given the vector
        minus_z_axis = mathutils.Vector((0.0,0.0,-1.0))

        # set the orientation of the light
        light.rotation_mode = "QUATERNION"
        light.rotation_quaternion = minus_z_axis.rotation_difference(lookat - position)

    # set the color of the light
    # (Here we use the "diffuse" light; the "ambient" light is more or less
    # like a global light; while "specular" light is the color of light after
    # reflection under a certain angle on a surface)
    if checkConfigParameter(config,"diffuse"):
        light.data.color = config["diffuse"]

    # set the falloff type to use linear and quadratic attenuation
    light.data.falloff_type = "LINEAR_QUADRATIC_WEIGHTED"

    # set the value for linear attenuation
    if checkConfigParameter(config,"linearAttenuation"):
        light.data.linear_attenuation = config["linearAttenuation"]

    # check for the quadratic attenuation
    if checkConfigParameter(config,"quadraticAttenuation"):
        light.data.quadratic_attenuation = config["quadraticAttenuation"]

    # if it is a "spot" light
    if light.data.type == "SPOT":
        # we have to set the angle of the spot
        if checkConfigParameter(config,"angle"):
            light.data.spot_size = math.pi * config["angle"] / 180.0

        # and the softness of the spot edges
        if checkConfigParameter(config,"exponent"):
            light.data.spot_blend = config["exponent"]

    # add each item of 'config' as a custom property to the lamp
    for (key, value) in config.items():
        light[key] = value

    # append it to the global light list
    lightList.append(light)

    return True


def createWorldProperties():
    # get the first world
    world = bpy.data.worlds[0]

    # set the default values for the world properties
    world["path"] = "."
    world["filename"] = "example"
    world["exportBobj"] = False
    world["exportMesh"] = True


def main(fileDir, filename):
    tmpDir = os.path.join(fileDir, "tmp")

    # change the current directory
    if os.path.isdir(fileDir) :
        os.chdir(fileDir)
    else:
        print("ERROR! File path (%s) does not exist!" % fileDir)
        return False

    # if there is already a "tmp" directory delete it
    shutil.rmtree(tmpDir, ignore_errors=True)

    # extract the .scn file to the "tmp" directory
    zipfile.ZipFile(os.path.join(fileDir, filename), "r").extractall(tmpDir)

    # path to the .scene file
    scenepath = os.path.join(tmpDir, filename.replace('.scn', '.scene'))

    # if the .scene file exists, read all of its content
    if os.path.isfile(scenepath) :
        dom = xml.dom.minidom.parse(scenepath)
    else:
        print("ERROR! Couldn't find .scene file (%s)!" % scenepath)
        return False

    # --- DO THE PARSING HERE!!! ---

    # before we start, wipe the plate clean
    cleanUpScene()

    # clean up all the unnecessary white spaces
    removeWhitespaceNodes(dom)

    # parsing all materials
    materials = dom.getElementsByTagName("material")
    for material in materials :
        if not parseMaterial(material):
            print("Error while parsing material!")
            return False

    # parsing all nodes
    nodes = dom.getElementsByTagName("node")
    for node in nodes :
        if not parseNode(node, tmpDir):
            print("Error while parsing node!")
            return False

    # set the origin of all nodes to the center of the bounding box
    centerAllNodeOrigin()

    # clean up if some unused meshes were loaded
    if len(unusedNodeList) > 0:
        print("WARNING! Not all imported meshes are in use!")
        #TODO: remove unneeded objects/meshes
        pass

    # parsing all joints
    joints = dom.getElementsByTagName("joint")
    for joint in joints :
        if not parseJoint(joint):
            print("Error while parsing joint!")
            return False

    # check for nodes with the same group ID
    checkGroupIDs()

    # parsing all motors
    motors = dom.getElementsByTagName("motor")
    for motor in motors :
        if not parseMotor(motor):
            print("Error while parsing motor!")
            return False

    # parsing all sensors
    sensors = dom.getElementsByTagName("sensor")
    for sensor in sensors :
        if not parseSensor(sensor):
            print("Error while parsing sensor!")
            return False

    # parsing all controllers
    controllers = dom.getElementsByTagName("controller")
    for controller in controllers :
        if not parseController(controller):
            print("Error while parsing controller!")
            return False

    # parsing all lights
    lights = dom.getElementsByTagName("light")
    for light in lights :
        if not parseLight(light):
            print("Error while parsing light!")
            return False

    # creating the global world properties
    createWorldProperties()

    # de-select all objects
    bpy.ops.object.select_all(action="DESELECT")

    #cleaning up afterwards
    shutil.rmtree(tmpDir)

    return True


class marsScnFileImporter(bpy.types.Operator):
    """Importer for MARS .scn files"""
    bl_idname = "import_scene.scn"
    bl_label = "Import MARS .scn file"

    # creating property for storing the path to the .scn file
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    # set a filter to only consider .scn files (only used internally)
    filter_glob = bpy.props.StringProperty(default="*.scn",options={'HIDDEN'})

    @classmethod
    def poll(cls, context):
        return context is not None

    def execute(self, context):
        # get the chosen file path
        directory, filename = os.path.split(self.filepath)

        # import the mars .scn file
        main(directory, filename)

        return {'FINISHED'}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}


if __name__ == '__main__' :

    # Register and add to the file selector
    bpy.utils.register_class(marsScnFileImporter)

    # call the newly registered operator
    bpy.ops.import_scene.scn('INVOKE_DEFAULT')

