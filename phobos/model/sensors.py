#!/usr/bin/python
# coding=utf-8

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

File sensors.py

Created on 6 Jan 2014

@author: Kai von Szadkowski
"""


import bpy
import mathutils
from phobos import defs
from phobos.phoboslog import log
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.utils.naming as nUtils
import phobos.utils.editing as eUtils
import phobos.utils.io as ioUtils


def deriveSensor(obj, names=False, objectlist=[]):
    """This function derives a sensor from a given blender object

    Args:
        obj(bpy_types.Object): The blender object to derive the sensor from.
        names(bool): return the link object name instead of an object link.
        objectlist (list(bpy.types.Object)): objectlist to which possible parents are restricted

    Returns:
      dict
    """
    from phobos.model.models import initObjectProperties
    log("Deriving sensor from object " + nUtils.getObjectName(obj, phobostype='sensor') + ".",
        'DEBUG')
    try:
        props = initObjectProperties(obj, phobostype='sensor')
        if names:
            props['link'] = nUtils.getObjectName(
                sUtils.getEffectiveParent(obj, objectlist=objectlist), phobostype='link')
        else:
            props['link'] = nUtils.getObjectName(
                sUtils.getEffectiveParent(obj, objectlist=objectlist), phobostype='link')
    except KeyError:
        log("Missing data in sensor " + obj.name, "ERROR")
        return None
    return props


def attachSensor(self, sensor):
    """This function attaches a given sensor to its parent link.

    Args:
      sensor(dict): The sensor you want to attach to its parent link.

    Returns:

    """
    bpy.context.scene.layers = bUtils.defLayers([defs.layerTypes[t] for t in defs.layerTypes])
    # TODO delete me?
    #try:
    if 'pose' in sensor:
        urdf_geom_loc = mathutils.Matrix.Translation(sensor['pose']['translation'])
        urdf_geom_rot = mathutils.Euler(tuple(sensor['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
    else:
        urdf_geom_loc = mathutils.Matrix.Identity(4)
        urdf_geom_rot = mathutils.Matrix.Identity(4)
    sensorobj = bpy.data.objects[sensor['name']]
    if 'link' in sensor:
        parentLink = sUtils.getObjectByNameAndType(sensor['link'], 'link')
        eUtils.parentObjectsTo(sensorobj, parentLink)
    else:
        #TODO: what? handle it...
        pass
    sensorobj.matrix_local = urdf_geom_loc * urdf_geom_rot
    # TODO delete me?
    #except KeyError:
    #    log("inconsistent data on sensor: "+ sensor['name'], "ERROR")


def cameraRotLock(object):
    """DOCU: PLEASE ADD PYDOC. What should this do exactly?

    Args:
      object(bpy_types.Object): The object to lock the rotation for

    Returns:

    """
    sUtils.selectObjects([object], active=0)
    bpy.ops.transform.rotate(value=-1.5708, axis=(-1, 0, 0), constraint_axis=(False, False, True), constraint_orientation='LOCAL', mirror=False, proportional='DISABLED', proportional_edit_falloff='SMOOTH', proportional_size=1)
    bpy.ops.transform.rotate(value=1.5708, axis=(0, -1, 0), constraint_axis=(True, False, False), constraint_orientation='LOCAL', mirror=False, proportional='DISABLED', proportional_edit_falloff='SMOOTH', proportional_size=1)
    bpy.ops.object.constraint_add(type='LIMIT_ROTATION')
    object.constraints["Limit Rotation"].use_limit_x = True
    object.constraints["Limit Rotation"].use_limit_y = True
    object.constraints["Limit Rotation"].use_limit_z = True
    object.constraints["Limit Rotation"].min_x = object.rotation_euler[0]
    object.constraints["Limit Rotation"].max_x = object.rotation_euler[0]
    object.constraints["Limit Rotation"].min_y = object.rotation_euler[1]
    object.constraints["Limit Rotation"].max_y = object.rotation_euler[1]
    object.constraints["Limit Rotation"].min_z = object.rotation_euler[2]
    object.constraints["Limit Rotation"].max_z = object.rotation_euler[2]

# FIXME there are two functions for creating sensors!
# def createSensor(self, sensor):
#     if 'link' in sensor:
#         reference = [sensor['link']]
#     elif 'joint' in sensor:
#         reference = [sensor['joint']]
#         pass
#     elif 'links' in sensor:
#         reference = sensor['links']
#     elif 'joints' in sensor:
#         reference = sensor['joints']
#     elif 'motors' in sensor:
#         reference = sensor['motors']
#     else:
#         reference = None

#     # FIXME: This needs to be resolved in a generic way!
#     createSensor(sensor, reference)
#     attachSensor(sensor)

def createSensor(sensor, reference, origin=mathutils.Matrix()):
    """This function creates a new sensor specified by its parameters.

    Args:
      sensor(dict): The phobos representation of the new sensor.
      reference(bpy_types.Object): This is an object to add a parent relationship to.
      origin(mathutils.Matrix, optional): The new sensors origin. (Default value = mathutils.Matrix()

    Returns:
      The newly created sensor object

    """
    layers = defs.layerTypes['sensor']
    bUtils.toggleLayer(layers, value=True)

    # create sensor object
    if sensor['shape'].startswith('resource'):
        newsensor = bUtils.createPrimitive(
            sensor['name'], 'box', [1, 1, 1], layers,
            plocation=origin.to_translation(), protation=origin.to_euler(),
            pmaterial=sensor['material'])
        # use resource name provided as: "resource:whatever_name"
        resource_obj = ioUtils.getResource(['sensor'] + sensor['shape'].split('://')[1].split('_'))
        if resource_obj:
            log("Assigned resource mesh and materials to new sensor object.", 'DEBUG')
            newsensor.data = resource_obj.data
        else:
            log("Could not use resource mesh for sensor. Default cube used instead.", 'WARNING')
    else:
        newsensor = bUtils.createPrimitive(
            sensor['name'], sensor['shape'], sensor['size'], layers,
            plocation=origin.to_translation(), protation=origin.to_euler(),
            pmaterial=sensor['material'])

    # assign the parent if available
    if reference is not None:
        eUtils.parentObjectsTo(newsensor, reference)

    # TODO we need to deal with other types of parameters for sensors

    # TODO cameraRotLock() use or dispose?
    # contact, force and torque sensors (or unknown sensors)
    #else:
    #    newsensor = bUtils.createPrimitive(
    #        sensor['name'], 'ico', 0.05, layers, 'phobos_sensor',
    #        origin.to_translation(), protation=origin.to_euler())
    #    if sensor['name'] == 'Joint6DOF':
    #        # TODO delete me? handle this
    #        #newsensor['sensor/nodes'] = nUtils.getObjectName(reference)
    #        pass
    #    elif 'Node' in sensor['type']:
    #        newsensor['sensor/nodes'] = sorted([nUtils.getObjectName(ref) for ref in reference])
    #    elif 'Joint' in sensor['type'] or 'Motor' in sensor['type']:
    #        newsensor['sensor/joints'] = sorted([nUtils.getObjectName(ref) for ref in reference])
#         elif sensor['type'] in ['Joint6DOF']:
#             for obj in context.selected_objects:
#                 if obj.phobostype == 'link':
#                     sensor['name'] = "sensor_joint6dof_" + nUtils.getObjectName(obj, phobostype="joint")
#                     sensors.createSensor(sensor, obj, obj.matrix_world)
#         elif 'Node' in sensor['type']:
#             sensors.createSensor(sensor, [obj for obj in context.selected_objects if obj.phobostype == 'collision'],
#                          mathutils.Matrix.Translation(context.scene.cursor_location))
#         elif 'Motor' in sensor['type'] or 'Joint' in sensor['type']:
#             sensors.createSensor(sensor, [obj for obj in context.selected_objects if obj.phobostype == 'link'],
#                          mathutils.Matrix.Translation(context.scene.cursor_location))

    # set sensor properties
    newsensor.phobostype = 'sensor'
    newsensor.name = sensor['name']
    newsensor['sensor/type'] = sensor['type']

    # write the custom properties to the sensor
    eUtils.addAnnotation(newsensor, sensor['props'], namespace='sensor')

    # throw warning if type is not known
    # TODO we need to link this error to the sensor type specifications
    if sensor['type'] not in [key.lower() for key in defs.def_settings['sensors']]:
        log("Sensor " + sensor['name'] + " is of unknown/custom type: " + sensor['type'] + ".",
            'WARNING')

    # select the new sensor
    sUtils.selectObjects([newsensor], clear=True, active=0)
    return newsensor

# TODO this class should reside at operators... give it a dev branch
# class AddLegacySensorOperator(Operator):
#     """AddSensorOperator"""
#     bl_idname = "phobos.add_legacy_sensor"
#     bl_label = "Add/Update a sensor"
#     bl_options = {'REGISTER', 'UNDO'}
#
#     sensor_type = StringProperty(
#         name = "sensor_type",
#         default = "",
#         description = "type of the sensor to be created")
#
#     sensor_scale = FloatProperty(
#         name = "sensor_scale",
#         default = 0.05,
#         description = "scale of the sensor visualization")
#
#     def execute(self, context):
#         location = bpy.context.scene.cursor_location
#         if self.sensor_type in defs.sensortypes:
#             if "Node" in self.sensor_type or "Joint" in self.sensor_type or "Motor" in self.sensor_type:
#                 objects = []
#                 sensors = []
#                 for obj in bpy.context.selected_objects:
#                     if obj.phobostype == "sensor":
#                         sensors.append(obj)
#                         self.sensor_type = obj["sensor/type"]
#                     else:
#                         objects.append(obj)
#                 if len(sensors) <= 0:
#                     utility.createPrimitive(self.sensor_type, "sphere", self.sensor_scale, defs.layerTypes["sensor"], "sensor", location)
#                     sense = bpy.context.scene.objects.active
#                     sense.phobostype = "sensor"
#                     sense.name = self.sensor_type
#                     sense["sensor/type"] = self.sensor_type
#                     sensors.append(sense)
#                 for sensor in sensors:
#                     for key in sensor.keys():
#                         if key.find("index") >= 0:
#                             del sensor[key]
#                             print("Deleting " + key + " in " + sensor.name)
#                     i = 1
#                     if "Node" in sensor["sensor/type"]:
#                         for obj in objects:
#                             if obj.phobostype == "collision":
#                                 sensor["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
#                                 i += 1
#                         print("Added nodes to new " + self.sensor_type)
#                     elif "Joint" in sensor["sensor/type"] or "Motor" in sensor["sensor/type"]:
#                         for obj in objects:
#                             if obj.phobostype == "link":
#                                 sensor["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
#                                 i += 1
#                         print("Added nodes to new " + self.sensor_type)
#                     if len(objects) > 0:
#                         sensor.parent = utility.getRoot(objects[0])
#             else: #visual sensor
#                 if self.sensor_type == "RaySensor":
#                     print("Added nodes to new " + self.sensor_type)
#                 elif self.sensor_type == "CameraSensor":
#                     bpy.ops.object.add(type='CAMERA', location=bpy.context.scene.cursor_location)
#                     sensor = bpy.context.active_object
#                     print("Added nodes to new " + self.sensor_type)
#                 elif self.sensor_type == "ScanningSonar":
#                     print("Added nodes to new " + self.sensor_type)
#             # add the pre-defined sensor properties
#             for prop in defs.definitions['sensora'][self.sensor_type]:
#                 sensor[prop] = defs.sensorProperties[self.sensor_type][prop]
#         else:
#             print("Sensor could not be created: unknown sensor type.")
#         return {'FINISHED'}
