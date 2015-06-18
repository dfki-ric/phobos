#!/usr/bin/python

"""
Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

This file is part of Phobos, a Blender Add-On to edit robot models.

Phobos is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License
as published by the Free Software Foundation, either version 3
of the License, or (at your option) any later version.

Phobos is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Phobos.  If not, see <http://www.gnu.org/licenses/>.

File robotupdate.py

Created on 7 Jan 2014

@author: Malte Langosz, Kai von Szadkowski
"""

import bpy
from datetime import datetime as dt
import phobos.utils.naming as namingUtils
import phobos.utils.selection as selectionUtils
import phobos.utils.general as generalUtils


#editmode = Blender.Window.EditMode()
#if editmode: Blender.Window.EditMode(0)

#TODO: this has to be massively extended
def convertOldModel():
    for obj in bpy.data.objects:
        if "type" in obj:
            obj.phobostype = str(obj["type"])
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
    if obj.phobostype == 'link':
        inertials = selectionUtils.getImmediateChildren(obj, ['inertial'])
        if len(inertials) == 0:
            notifications.append("Warning, link '" + namingUtils.getObjectName(obj) + "' has no inertial object.")
            #if fix:
            #    inertia.createInertial(obj)
        elif len(inertials) > 1:
            pass
        elif len(inertials) == 1:
            pass
        # checking whether masses add up if we have an inertial by now
        mass = generalUtils.calculateSum(selectionUtils.getImmediateChildren(obj), 'mass')
        #FIXME: inertia.calculateMassOfLink???
        if not mass > 0:
            notifications.append("Warning, link '" + namingUtils.getObjectName(obj) + "' has no mass.")
        for inertial in inertials:
            print('Checking inertial: ' + inertial.name)
            if not 'inertia' in inertial:
                notifications.append("Error, inertial of link '" + namingUtils.getObjectName(obj) + "' has no inertia.")
            if not 'mass' in inertial or not inertial['mass'] > 0:
                notifications.append("Error, inertial of link '" + namingUtils.getObjectName(obj) + "' has no attribute 'mass' or zero mass.")
                faulty_objects.append(obj)
                #if fix:
                #    inertial['mass'] = mass
                #    inertial['masschanged'] = dt.now().isoformat()
    elif obj.phobostype == 'inertial':
        if fix:
            if not obj.name.startswith('inertial_'):
                obj.name = 'inertial_' + obj.name
    elif obj.phobostype == 'visual':
        if fix:
            if not "geometry/type" in obj:
                notifications.append("Warning, visual '" + namingUtils.getObjectName(obj) + "' has no geometry/type.")
                obj["geometry/type"] = "mesh"
            if not obj.name.startswith('visual_'):
                obj.name = 'visual_' + obj.name
            if not 'masschanged' in obj:
                obj['masschanged'] = dt.now().isoformat()
    elif obj.phobostype == 'collision':
        if fix:
            if not obj.name.startswith('collision_'):
                obj.name = 'collision_' + obj.name
            if not 'masschanged' in obj:
                obj['masschanged'] = dt.now().isoformat()
    elif obj.phobostype == 'sensor':
        pass
    return notifications, faulty_objects

def updateModel(root, fix = False):
    notifications = []
    faulty_objects = []
    children = selectionUtils.getChildren(root)
    for obj in children:
        n, f = updateObject(obj, fix)
        notifications.extend(n)
        faulty_objects.extend(f)
    return notifications, faulty_objects

def updateModels(roots = None, fix = False):
    notifications = []
    faulty_objects = []
    if roots == None:
        roots = selectionUtils.getRoots()
    for root in roots:
        print("Phobos: Updating properties for model", root.name)
        if not "modelname" in root:
            root["modelname"] = root.name
            print("Phobos: new root detected, setting modelname to", root.name)
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
