#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Contains the utility functions for editing objects and Phobos models.
"""
import bpy
import mathutils
import phobos.blender.utils.io as ioUtils
import phobos.blender.utils.blender as bUtils
import phobos.blender.utils.naming as nUtils
import phobos.blender.utils.editing as eUtils
import phobos.blender.utils.selection as sUtils
import phobos.blender.defs as defs


def createInterface(ifdict, parent, origin=mathutils.Matrix()):
    """Create an interface object and optionally parent to existing object.

    ifdict is expected as:

    | **type**: str
    | **direction**: str
    | **model**: str
    | **name**: str
    | **parent**: bpy.types.Object (optional)
    | **scale**: float (optional)

    Args:
      ifdict(dict): interface data
      parent(bpy.types.Object, optional): designated parent object (Default value = None)

    Returns:
      bpy.data.Object: newly created interface object

    """
    bUtils.toggleLayer('interface', value=True)

    if not parent:
        try:
            parent = ifdict['parent']
            assert isinstance(parent, bpy.types.Object)
        except (AttributeError, AssertionError, KeyError):
            parent = None

    model = ifdict['model'] if 'model' in ifdict else 'default'
    templateobj = ioUtils.getResource(('interface', model, ifdict['direction']))
    scale = ifdict['scale'] if 'scale' in ifdict else 1.0
    ifobj = bUtils.createPrimitive(
        ifdict['name'],
        'box',
        (1.0, 1.0, 1.0),
        defs.layerTypes['interface'],
        plocation=origin.to_translation(),
        protation=origin.to_euler(),
        phobostype='interface',
    )
    nUtils.safelyName(ifobj, ifdict['name'], 'interface')
    ifobj.data = templateobj.data
    ifobj.phobostype = "interface"
    ifobj.scale = (scale,) * 3
    ifobj['interface/type'] = ifdict['type']
    ifobj['interface/direction'] = ifdict['direction']
    if parent is not None:
        eUtils.parentObjectsTo(ifobj, parent)
        ifobj.matrix_local = origin

    # write the custom properties to the sensor
    if "props" in ifdict:
        eUtils.addAnnotation(ifobj, ifdict['props'], namespace='sensor')

    # select the new interface
    sUtils.selectObjects([ifobj], clear=True, active=0)
    return ifobj