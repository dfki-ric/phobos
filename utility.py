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

File utility.py

Created on 9 Jan 2014

@author: Kai von Szadkowski
"""

import re
import bpy
import mathutils
from datetime import datetime
from . import defs
from . import materials
from phobos.logging import *


def register():
    print("Registering phobos.utility...")


def unregister():
    print("Unregistering phobos.utility...")


def is_float(s):
    """Tests if an input variable (string) is a float number."""
    try:
        float(s)
        return True
    except (ValueError, TypeError):
        return False


def is_int(s):
    """Tests is an input variable (string) is an int number."""
    try:
        int(s)
        return True
    except ValueError:
        pl.logger.log("Value is no Integer", "ERROR")
        return False


def parse_number(s):
    """Takes an input variable (string) and determines whether it represents
    a float number, int or string"""
    if is_int(s):
        return int(s)
    elif is_float(s):
        return float(s)
    else:
        return s


def only_contains_int(stringlist):
    """Checks if a list of strings contains int numbers exclusively."""
    for num in stringlist:
        if not is_int(num):
            return False
    return True


def only_contains_float(stringlist):
    """Checks if a list of strings contains float numbers exclusively."""
    for num in stringlist:
        if not is_float(num):
            return False
    return True


def find_in_list(alist, prop, value):
    """Returns the index of the first object in a list which has a field
    named *prop* with value *value*. If no such object is found, returns -1."""
    n = -1
    for i in range(len(alist)):
        try:
            if alist[i][prop] == value:
                n = i
                break
        except KeyError:
            pl.logger.log("The object at index " + str(i) + " has no property " + str(prop))
    return n


def retrieve_from_list(alist, prop, value):
    """Returns the first object in a list which has a field named
    *prop* with value *value*. If no such object is found, returns "None"."""
    n = -1
    for i in range(len(alist)):
        try:
            if alist[i][prop] == value:
                n = i
                break
        except KeyError:
            pl.logger.log("The object at index " + str(i) + " has no property " + str(prop))
    if n >= 0:
        return alist[n][prop]
    else:
        return "None"


def parse_text(s):
    """Parses a text by splitting up elements separated by whitespace. The elements are then
    try to be parsed as lists of floats, ints or strings or, if only one element is found,
    are tried to be parsed using the function parse_number()."""
    numstrings = s.split()
    if numstrings == []:
        return None
    if len(numstrings) > 1:
        if only_contains_int(numstrings):
            nums = [int(num) for num in numstrings]
            return nums
        elif only_contains_float(numstrings):
            nums = [float(num) for num in numstrings]
            return nums
        else:
            return numstrings  # s
    else:
        return parse_number(s)


def printMatrices(obj, info=None):
    if not info:
        info = obj.name
    print("\n----------------", info, "---------------------\n",
          "local:\n", obj.matrix_local,
          "\n\nworld:\n", obj.matrix_world,
          "\n\nparent_inverse:\n", obj.matrix_parent_inverse,
          "\n\nbasis:\n", obj.matrix_basis)


def assignMaterial(obj, materialname):
    if materialname not in bpy.data.materials:
        if materialname in defs.defaultmaterials:
            materials.createPhobosMaterials()
        else:
            #print("###ERROR: material to be assigned does not exist.")
            pl.logger.log("Material to be assigned does not exist.", "ERROR")
            return None
    obj.data.materials.append(bpy.data.materials[materialname])
    if bpy.data.materials[materialname].use_transparency:
        obj.show_transparent = True


def createPrimitive(pname, ptype, psize, player=0, pmaterial="None", plocation=(0, 0, 0), protation=(0, 0, 0),
                    verbose=False):
    """Generates the primitive specified by the input parameters"""
    if verbose:
        print(ptype, psize)
    try:
        # n_layer = bpy.context.scene.active_layer
        n_layer = int(player)
    except ValueError:
        n_layer = defs.layerTypes[player]
    players = defLayers([n_layer])
    bpy.context.scene.layers[n_layer] = True  # the layer has to be active to prevent problems with object placement
    if ptype == "box":
        bpy.ops.mesh.primitive_cube_add(layers=players, location=plocation, rotation=protation)
        obj = bpy.context.object
        obj.dimensions = psize
    if ptype == "sphere":
        bpy.ops.mesh.primitive_uv_sphere_add(size=psize, layers=players, location=plocation, rotation=protation)
    elif ptype == "cylinder":
        bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=psize[0], depth=psize[1], layers=players,
                                            location=plocation, rotation=protation)
    elif ptype == "cone":
        bpy.ops.mesh.primitive_cone_add(vertices=32, radius=psize[0], depth=psize[1], cap_end=True, layers=players,
                                        location=plocation, rotation=protation)
    elif ptype == 'disc':
        bpy.ops.mesh.primitive_circle_add(vertices=psize[1], radius=psize[0], fill_type='TRIFAN', location=plocation, rotation=protation, layers=players)
    obj = bpy.context.object
    obj.name = pname
    if pmaterial != 'None':
        assignMaterial(obj, pmaterial)
    return obj


def toggleLayer(index, value=None):
    if value:
        bpy.context.scene.layers[index] = value
    else:
        bpy.context.scene.layers[index] = not bpy.context.scene.layers[index]


def defLayers(layerlist):
    """Returns a list of 20 elements encoding the visible layers according to layerlist"""
    if type(layerlist) is not list:
        layerlist = [layerlist]
    layers = 20 * [False]
    for layer in layerlist:
        layers[layer] = True
    return layers


def returnObjectList(marstype):
    """Returns list of all objects in the current scene matching marstype"""
    objlist = []
    for obj in bpy.context.scene.objects:
        if obj.phobostype == marstype:
            objlist.append(obj)
    return objlist


def getChildren(root):
    """Finds all children for a given root"""
    children = []
    for obj in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
        if getRoot(obj) == root:
            children.append(obj)
    return children


def getImmediateChildren(obj, phobostypes=None):
    """Finds all immediate children for a given object"""
    children = []
    for child in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
        if child.parent == obj:
            if phobostypes is not None:
                if child.phobostype in phobostypes:
                    children.append(child)
            else:
                children.append(child)
    return children


def getRoot(obj=None):
    """Finds the root object of a model given one of the model elements is selected or provided"""
    if obj == None:
        for anobj in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
            if (anobj.select):
                obj = anobj
    child = obj
    if child == None:
        pl.logger.log("No root object found! Check your object selection", "ERROR")
        return None
    while child.parent != None:
        child = child.parent
    return child


def getRoots():
    """Returns a list of all roots (=objects without parent) present in the scene"""
    roots = []
    for obj in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
        if not obj.parent and obj.phobostype == "link":
            roots.append(obj)

    if roots == []:
        print("Phobos: No root objects found.")
    else:
        print("Phobos: Found", len(roots), "root object(s)", [root.name + "; " for root in roots])
    return roots  # TODO: Should we change this and all other list return values in a tuple or generator expression?


def calcBoundingBoxCenter(boundingbox):
    """Calculates the center of a bounding box"""
    c = sum((mathutils.Vector(b) for b in boundingbox), mathutils.Vector())
    return c / 8


def selectObjects(objects, clear=True, active=-1):
    '''Selects all objects provided in list, clears current selection if clear is True
    and sets one of the objects the active objects if a valid index is provided.'''
    if clear:
        bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select = True
    if active >= 0:
        bpy.context.scene.objects.active = objects[active]


def replaceNameElement(prop, old, new):
    """For all selected elements in Blender, replace an *old* part of a string *prop*erty with *new*."""
    for obj in bpy.context.selected_objects:
        if prop in obj and obj[prop].find(old) > -1:
            obj[prop] = obj[prop].replace(old, new)


def roundVector(v, n):
    """Returns a mathutils.Vector with its components rounded to n digits."""
    return round(v.x, n), round(v.y, n), round(v.z, n)


def epsilonToZero(data, epsilon, decimals):
    """Recursively loops through a dictionary and sets all floating values
     < epsilon equal to zero."""
    if is_float(data):
        return 0 if abs(data) < epsilon else round(data, decimals)
    elif type(data) is list:
        return [epsilonToZero(a, epsilon, decimals) for a in data]
    elif type(data) is dict:
        return {key: epsilonToZero(value, epsilon, decimals) for key, value in data.items()}
    else:  # any other type, such as string
        return data


def calculateSum(objects, numeric_prop):
    """Returns sum of *numeric_prop* in *objects*."""
    numsum = 0
    for obj in objects:
        try:
            numsum += obj[numeric_prop]
        except KeyError:
            log("The object '" + obj.name + "' has not property '" + numeric_prop + "'")
    return numsum


def datetimeFromIso(iso):
    """Accepts a date-time string in iso format and returns a datetime object."""
    return datetime(*[int(a) for a in re.split(":|-|T|\.", iso)])


def distance(objects):
    v = objects[0].matrix_world.to_translation() - objects[1].matrix_world.to_translation()
    return v.length, v


def outerProduct(v, u):
    """Returns a mathutils.Matrix representing the outer product of vectors v and u."""
    lines = []
    for vi in v:
        lines.append([vi * ui for ui in u])
    return mathutils.Matrix(lines)


def createNewTextfile(textfilename, contents):
    for text in bpy.data.texts:
        text.tag = True
    bpy.ops.text.new()
    newtext = None
    for text in bpy.data.texts:
        if not text.tag:
            newtext = text
    for text in bpy.data.texts:
        text.tag = False
    newtext.name = textfilename
    bpy.data.texts[textfilename].write(contents)


def cleanObjectProperties(props):
    """Cleans a predefined list of Blender-specific or other properties from the dictionary."""
    getridof = ['phobostype', '_RNA_UI', 'cycles_visibility', 'startChain', 'endChain', 'masschanged']
    if props:
        for key in getridof:
            if key in props:
                del props[key]
    return props

# def useLegacyNames(data):
#    if type(data) is str:
#        print(data, end=': ')
#        if data in defs.MARSlegacydict:
#            print(defs.MARSlegacydict[data])
#            return defs.MARSlegacydict[data]
#        else:
#            print(data)
#            return data
#    elif type(data) is dict:
#        tmpdict = {useLegacyNames(key): value for key, value in data.items()}
#        return {key: useLegacyNames(value) for key, value in tmpdict.items()}


