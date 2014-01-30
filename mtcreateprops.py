'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtcreateprops.py

Created on 7 Jan 2014

@author: Malte Langosz, Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
'''

import bpy
import os, glob
import mathutils
import marstools.mtmaterials as mtmaterials


#editmode = Blender.Window.EditMode()
#if editmode: Blender.Window.EditMode(0)

class IDGenerator(object):

    startID = 0

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

def getChildren(parent):
    children = []
    for obj in bpy.data.objects:
        if obj.parent == parent:
            children.append(obj)
    return children

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
        if not hasattr(world, "filename"):
            world.filename = "filename"
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
                if "MARStype" in child and child.MARStype == "joint":
                    if "node2" in child and child["node2"] == obj.name:
                        obj["group"] = self.getNextGroupID()
                        setGroup = True
                        break
            if not setGroup and obj.parent:
                obj["group"] = obj.parent["group"]
        else:
            obj["group"] = self.getNextGroupID()
        if "mass" not in obj and "density" not in obj:
            setDefault(obj, "mass", 0.1)

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
        print("handle: "+obj.name)
    #    obj.select = False
        obj.data.name = obj.name
        defaultType = "body"
        if obj.name.find("joint") > -1:
            defaultType = "joint"

        objType = setDefault(obj, "MARStype", defaultType)
        if objType == "body":
            self.createBodyProperties(obj)
        elif objType == "joint":
            self.createJointProperties(obj)

        children = getChildren(obj)
        for obj in children:
            self.handleProps(obj)


def getRoot():
    for obj in bpy.data.objects:
        if (obj.select) and (obj.parent == None):
            return obj

def main():
    propscreator = MARSPropsGenerator()

    mtmaterials.createMARSMaterials()
    root = getRoot()

    if root:
        print(root.name)
        propscreator.reset()
        propscreator.handleProps(root)


if __name__ == '__main__':
    main()
