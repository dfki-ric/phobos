'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtcreateprops.py

Created on 7 Jan 2014

@author: Malte Langosz, Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.

NOTE: If you edit this script, please make sure not to use any imports
not supported by Blender's standard python distribution. This is a script
intended to be usable on its own and thus should not use external dependencies,
especially none on the other modules of the MARStools package.
'''

import bpy
import os, glob
import mathutils


#editmode = Blender.Window.EditMode()
#if editmode: Blender.Window.EditMode(0)

class IDGenerator(object):

    def __init__(self, startID=0):
        self.nextID = startID
        self.startID = startID

    def __call__(self):
        ret, self.nextID = self.nextID, self.nextID + 1
        return ret

    def reset(self):
        self.nextID = self.startID


def setDefault(obj, key, value):
    if key not in obj:
        obj[key] = value
    return obj[key]

def setDefaultType(obj, value):
    if obj.MARStype == "undefined":
        obj.MARStype = value
    return obj.MARStype

def getChildren(parent):
    children = []
    for obj in bpy.data.objects:
        if obj.parent == parent:
            children.append(obj)
    return children

def updateType(obj):
    if "type" in obj:
        obj.MARStype = str(obj["type"])
        del obj["type"]

class MARSPropsGenerator():

    def __init__(self):
        self.getNextBodyID = IDGenerator(1)
        self.getNextJointID = IDGenerator(1)
        self.getNextGroupID = IDGenerator(1)
        self.createWorldProperties()

    def reset(self):
        self.getNextBodyID.reset()
        self.getNextJointID.reset()
        self.getNextGroupID.reset()

    def createWorldProperties(self):
        world = bpy.data.worlds[0]
        if not hasattr(world, "path"):
            world.path = "."
        #if not hasattr(world, "filename"):
        #    world.filename = "filename"
        if not hasattr(world, "exportBobj"):
            world.exportBobj = False
        if not hasattr(world, "exportMesh"):
            world.exportMesh = True

    def createBodyProperties(self, obj):
        obj.MARStype = "body"
        obj["id"] = self.getNextBodyID()
        if obj.parent:
            children = getChildren(obj.parent)
            setGroup = False
            for child in children:
                if child.MARStype == "joint":
                    if "node2" in child and child["node2"] == obj.name:
                        obj["group"] = self.getNextGroupID()
                        setGroup = True
                        break
            if not setGroup and obj.parent:
                obj["group"] = obj.parent["group"]
        else:
            obj["group"] = self.getNextGroupID()
        if "mass" not in obj and "density" not in obj:
            setDefault(obj, "mass", 0.001)
        #TODO_ make collision bitmask a string in binary format
        #if "collision_bitmask" in obj and obj["collision_bitmask"]

    def createJointProperties(self, obj):
        obj.MARStype = "joint"
        obj["id"] = self.getNextJointID()
        setDefault(obj, "node2", "air")
        setDefault(obj, "jointType", "hinge")
        setDefault(obj, "anchor", "node2")
        setDefault(obj, "controllerIndex", 0)
        children = getChildren(obj.parent)
        if len(children) == 2:
            if children[0] != obj:
                obj["node2"] = children[0].name
            else:
                obj["node2"] = children[1].name
        else:
            print(obj.name+": num children: "+str(len(children)))

    def handleProps(self, obj):
        print("Creating properties for object:", obj.name)
    #    obj.select = False
        obj.data.name = obj.name
        updateType(obj)
        defaultType = "body"
        if obj.name.find("joint") > -1:
            defaultType = "joint"
        objType = setDefaultType(obj, defaultType)
        if objType == "body":
            self.createBodyProperties(obj)
        elif objType == "joint":
            self.createJointProperties(obj)
        else:
            print("Unable to determine type for", obj.name)
        children = getChildren(obj)
        for obj in children:
            self.handleProps(obj)

def getRoots():
    """Finds any selected root objects"""
    roots = []
    for obj in bpy.context.selected_objects:
        if obj.parent == None:
            roots.append(object)
    return roots

def main(roots = None):
    if roots == None:
        roots = getRoots()
#TODO: the following code has to be tested for correct re-initialization
    for root in roots:
        print("MARStools: Updating properties for model", root.name)
        if not "modelname" in root:
            root["modelname"] = root.name
            print("MARStools: new root detected, setting modelname to", root.name)
        propscreator = MARSPropsGenerator()
        propscreator.handleProps(root)


if __name__ == '__main__':
    main()
