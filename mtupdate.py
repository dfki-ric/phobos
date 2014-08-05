'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtupdate.py

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
import marstools.mtutility as mtutility
import marstools.mtinertia as mtinertia
from datetime import datetime as dt

#editmode = Blender.Window.EditMode()
#if editmode: Blender.Window.EditMode(0)

#TODO: this has to be massively extended
def convertOldModel():
    for obj in bpy.data.objects:
        if "type" in obj:
            obj.MARStype = str(obj["type"])
            del obj["type"]

def setDefault(obj, key, value):
    if key not in obj:
        obj[key] = value
    return obj[key]

def createLinkProperties(self, obj):
    if "mass" not in obj and "density" not in obj:
        setDefault(obj, "mass", 0.001)
    #TODO_ make collision bitmask a string in binary format
    #if "collision_bitmask" in obj and obj["collision_bitmask"]

def updateObject(obj, fix = False):
    notifications = []
    faulty_objects = []
    if obj.MARStype == 'link':
        inertials = mtutility.getImmediateChildren(obj, ['inertial'])
        if len(inertials) == 0:
            notifications.append("Warning, link '" + obj.name + "' has no inertial object.")
            #if fix:
            #    mtinertia.createInertial(obj)
        elif len(inertials) > 1:
            pass
        elif len(inertials) == 1:
            pass
        # checking whether masses add up if we have an inertial by now
        mass = mtutility.calculateSum(mtutility.getImmediateChildren(obj), 'mass')
        if not mass > 0:
            notifications.append("Warning, link '" + obj.name + "' has no mass.")
        for inertial in inertials:
            print('Checking inertial: ' + inertial.name)
            if not 'inertia' in inertial:
                notifications.append("Error, inertial of link '" + obj.name + "' has no inertia.")
            if not 'mass' in inertial or not inertial['mass'] > 0:
                notifications.append("Error, inertial of link '" + obj.name + "' has no attribute 'mass' or zero mass.")
                faulty_objects.append(obj)
                #if fix:
                #    inertial['mass'] = mass
                #    inertial['masschanged'] = dt.now().isoformat()
    elif obj.MARStype == 'inertial':
        if fix:
            if not obj.name.startswith('inertial_'):
                obj.name = 'inertial_' + obj.name
    elif obj.MARStype == 'visual':
        if fix:
            if not "geometryType" in obj:
                obj["geometryType"] = "mesh"
            if not obj.name.startswith('visual_'):
                obj.name = 'visual_' + obj.name
            if not 'masschanged' in obj:
                obj['masschanged'] = dt.now().isoformat()
    elif obj.MARStype == 'collision':
        if fix:
            if not obj.name.startswith('collision_'):
                obj.name = 'collision_' + obj.name
            if not 'masschanged' in obj:
                obj['masschanged'] = dt.now().isoformat()
    elif obj.MARStype == 'sensor':
        pass
    return notifications, faulty_objects

def updateModel(root, fix = False):
    notifications = []
    faulty_objects = []
    children = mtutility.getChildren(root)
    for obj in children:
        n, f = updateObject(obj, fix)
        notifications.extend(n)
        faulty_objects.extend(f)
    return notifications, faulty_objects

def updateModels(roots = None, fix = False):
    notifications = []
    faulty_objects = []
    if roots == None:
        roots = mtutility.getRoots()
    for root in roots:
        print("MARStools: Updating properties for model", root.name)
        if not "modelname" in root:
            root["modelname"] = root.name
            print("MARStools: new root detected, setting modelname to", root.name)
        n, f = updateModel(root, fix)
        notifications.extend(n)
        faulty_objects.extend(f)
    #Deselect all objects and select those with errors
    #bpy.ops.object.select_all() # alternatively:
    for obj in bpy.data.objects:
        obj.select = False
    for obj in faulty_objects:
        obj.select = True
    #bpy.ops.error.message('INVOKE_DEFAULT', type="Errors", message='\n'.join(notifications))
    print('\n'.join(notifications))


if __name__ == '__main__':
    updateModels()
