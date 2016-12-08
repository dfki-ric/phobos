#!/usr/bin/python
# coding=utf-8

"""
Copyright 2014-2016, University of Bremen & DFKI GmbH Robotics Innovation Center

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

File phobosgui.py

Created on 3 Nov 2016

@author: Kai von Szadkowski
"""

import phobos.model.models as models
import phobos.utils.naming as nUtils
from phobos.phoboslog import log


# class RobotModelParser():
#     """Base class for a robot model file parser of a specific type
#
#     """
#
#     def __init__(self, filepath):
#         """This init saves the filepath splitted into path and filename and creates an initial empty robot dictionary.
#
#         :param filepath: The filepath you want to export the robot to *WITH FILENAME*
#         :type filepath: str
#
#         """
#         self.filepath = filepath
#         self.path, self.filename = os.path.split(self.filepath)
#         self.robot = {'links': {},
#                       'joints': {},
#                       'sensors': {},
#                       'motors': {},
#                       'controllers': {},
#                       'materials': {},
#                       'groups': {},
#                       'chains': {}
#                       }
#         if os.access(self.path, os.W_OK):
#             self.tmp_path = self.path
#         elif os.access(os.path.expanduser('.'), os.W_OK):
#             self.tmp_path = os.path.expanduser('.')
#         else:
#             raise Exception('WTF? No write permission for home dir.')
#
#     def praefixNames(self, name, praefix):
#         """This function takes a name and a praefix and praefixes the name with it if its not already praefixed with it.
#
#         :param name: The name to praefix.
#         :type name: str
#         :param praefix:  The praefix to use.
#         :type praefix: str
#         :return: str - The praefixed name
#
#         """
#         prae = praefix + "_"
#         if (name.startswith(prae)):
#             return name
#         else:
#             return prae+name
#
#     def scaleLink(self, link, newlink):
#         """Scales newly-created armatures depending on the link's largest collision object.
#         The function is very simple and could be improved to scale the links even more appropriate.
#
#         :param link: the link containing the collision objects
#         :type link: dict
#         :param newlink: the link you want to scale.
#         :type newlink: dict
#
#         """
#         newscale = 0.3
#         if len(link['collision']) > 0:
#             sizes = []
#             for collname in link['collision']:
#                 collobj = link['collision'][collname]
#                 colltype = collobj['geometry']['type']
#                 if colltype == 'sphere':
#                     sizes.append(collobj['geometry']['radius'])
#                 elif colltype == 'cylinder':
#                     sizes.append(max(collobj['geometry']['radius'], collobj['geometry']['length']))
#                 elif colltype == 'mesh':
#                     sizes.append(max(collobj['geometry']['scale']))  # TODO: this alone is not very informative
#                 else:
#                     sizes.append(max(collobj['geometry']['size']))
#             newscale = max(sizes)
#         newlink.scale = (newscale, newscale, newscale)
#
#     def placeChildLinks(self, parent):
#         """This function creates the parent-child-relationship for a given parent and all existing children in blender.
#
#         :param parent: This is the parent link you want to set the children for.
#         :type: dict
#
#         """
#         bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes['link'])
#         print(parent['name'] + ', ', end='')
#         children = []
#         for l in self.robot['links']:
#             if 'parent' in self.robot['links'][l] and self.robot['links'][l]['parent'] == parent['name']:
#                 children.append(self.robot['links'][l])
#         for child in children:
#             # 1: set parent relationship (this makes the parent inverse the inverse of the parents world transform)
#             parentLink = bpy.data.objects[self.praefixNames(parent['name'], "link")]
#             childLink = bpy.data.objects[self.praefixNames(child['name'], "link")]
#             sUtils.selectObjects([childLink, parentLink], True, 1)
#             bpy.ops.object.parent_set(type='BONE_RELATIVE')
#             # 2: move to parents origin by setting the world matrix to the parents world matrix
#             childLink.matrix_world = parentLink.matrix_world        # removing this line does not seem to make a difference
#
#             # #bpy.context.scene.objects.active = childLink
#             # if 'pivot' in child:
#             #     pivot = child['pivot']
#             #     cursor_location = bpy.context.scene.cursor_location
#             #     bpy.context.scene.cursor_location = mathutils.Vector((-pivot[0]*0.3, -pivot[1]*0.3, -pivot[2]*0.3))
#             #     bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
#             #     bpy.context.scene.cursor_location = cursor_location
#
#             # 3: apply local transform as saved in urdf (change matrix_local from identity to urdf)
#             location = mathutils.Matrix.Translation(child['pose']['translation'])
#             rotation = mathutils.Euler(tuple(child['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
#             transform_matrix = location * rotation
#             childLink.matrix_local = transform_matrix
#             # 4: be happy, as world and basis are now the same and local is the transform to be exported to urdf
#             # 5: take care of the rest of the tree
#             self.placeChildLinks(child)
#
#     def placeLinkSubelements(self, link):
#         """This function finds all subelements for a given link and sets the appropriate relations.
#         In this case subelements are interials, visuals and collisions.
#
#         :param link: The parent link you want to set the subelements for
#         :type link: dict
#
#         """
#         bpy.context.scene.layers = bUtils.defLayers([defs.layerTypes[t] for t in defs.layerTypes])
#         parentLink = bpy.data.objects[self.praefixNames(link['name'], "link")]
#         if 'inertial' in link:
#             if 'pose' in link['inertial']:
#                 urdf_geom_loc = mathutils.Matrix.Translation(link['inertial']['pose']['translation'])
#                 urdf_geom_rot = mathutils.Euler(tuple(link['inertial']['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
#             else:
#                 urdf_geom_loc = mathutils.Matrix.Identity(4)
#                 urdf_geom_rot = mathutils.Matrix.Identity(4)
#             geoname = link['inertial']['name'] #g
#             try:
#                 geom = bpy.data.objects[geoname]
#             except:
#                 print('ERROR: missing subelement')
#                 return
#             inertialname = link['inertial']['name']
#             inertialobj = bpy.data.objects[inertialname]
#             sUtils.selectObjects([inertialobj, parentLink], True, 1)
#             bpy.ops.object.parent_set(type='BONE_RELATIVE')
#             inertialobj.matrix_local = urdf_geom_loc * urdf_geom_rot
#         for geomsrc in ['visual', 'collision']:
#             if geomsrc in link:
#                 for g in link[geomsrc]:
#                     geomelement = link[geomsrc][g]
#                     if 'pose' in geomelement:
#                         urdf_geom_loc = mathutils.Matrix.Translation(geomelement['pose']['translation'])
#                         urdf_geom_rot = mathutils.Euler(tuple(geomelement['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
#                     else:
#                         urdf_geom_loc = mathutils.Matrix.Identity(4)
#                         urdf_geom_rot = mathutils.Matrix.Identity(4)
#                     geoname = self.praefixNames(geomelement['name'], geomsrc)
#                     geom = bpy.data.objects[geoname]
#                     # FIXME: this does not do anything - how to set basis matrix to local?
#                     #geom.matrix_world = parentLink.matrix_world
#                     #selectObjects([geom], True, 0)
#                     #bpy.ops.object.transform_apply(location=True, rotation=True)
#                     sUtils.selectObjects([geom, parentLink], True, 1)
#                     bpy.ops.object.parent_set(type='BONE_RELATIVE')
#                     geom.matrix_local = urdf_geom_loc * urdf_geom_rot
#                     try:
#                         geom.scale = geomelement['geometry']['scale']
#                     except KeyError:
#                         pass
#
#                     sUtils.selectObjects([geom, parentLink], True, 1)
#
#     def _get_object(self, link_name):
#         """
#         """
#         objects = bpy.context.scene.objects
#         obj = objects[link_name]
#         return obj
#
#     def _apply_joint_angle_offsets(self):
#         """This function applies joint angle offsets in blender to all links in self.robot.
#
#         """
#         links = self.robot['links']
#         for link_name in links:
#             link = links[link_name]
#             if 'axis1' in link:
#                 # move edit bone y axis to axis1
#                 bpy.ops.object.mode_set(mode='EDIT')
#
#                 axis_vec = mathutils.Vector(link['axis1'])
#                 bpy.data.objects[link_name].data.bones.active.tail = axis_vec
#
#                 #print()
#                 #print('link name:', link_name)
#                 #print("axis:", axis_vec)
#
#                 bpy.ops.object.mode_set(mode='OBJECT')
#                 pass
#             if 'angle_offset' in link:
#                 obj = self._get_object(link_name)
#                 angle = -(link['angle_offset'])
#                 bpy.ops.object.mode_set(mode='POSE')
#                 pose_bone = obj.pose.bones['Bone']
#                 bpy.ops.pose.armature_apply()
#                 bpy.ops.object.mode_set(mode='OBJECT')
#                 y_axis = mathutils.Vector([0, 1, 0])
#                 bone_matrix = pose_bone.matrix
#                 loc, rot, scale = bone_matrix.decompose()
#                 axis = rot * y_axis
#                 offset_matrix = mathutils.Matrix.Rotation(angle, 4, axis)
#                 pose_bone.matrix = offset_matrix * bone_matrix
#
#
#     def createController(self, controller):
#         """This function creates a specified controller.
#
#         :param controller: The controller to create.
#         :type controller: dict
#
#         """
#         controllers.addController(controller)
#
#     def createLight(self, light):
#         """This function creates a specified light.
#
#         :param light: The light to create.
#         :type light: dict
#
#         """
#         lights.addLight(light)
#
#     def createGroup(self, group):
#         """
#
#         :param group:
#
#         """
#         pass
#
#     def createChain(self, chain):
#         """
#
#         :param chain:
#
#         """
#         pass
#
#     def createBlenderModel(self): #TODO: solve problem with duplicated links (linklist...namespaced via robotname?)
#         """Creates the blender object representation of the imported model.
#         For that purpose it uses the former specified robot model dictionary.
#
#         """
#         print("\n\nCreating Blender model...")
#         print("Creating links...")
#         for l in self.robot['links']:
#             #print(l + ', ', end='')
#             link = self.robot['links'][l]
#             #print(link['name'])
#             self.createLink(link)
#
#         print("\n\nCreating joints...")
#         for j in self.robot['joints']:
#             print(j + ', ', end='')
#             joint = self.robot['joints'][j]
#             self.createJoint(joint)
#
#         #build tree recursively and correct translation & rotation on the fly
#         print("\n\nPlacing links...")
#         for l in self.robot['links']:
#             if not 'parent' in self.robot['links'][l]:
#                 root = self.robot['links'][l]
#                 self.placeChildLinks(root)
#                 print("\n\nAssigning model name...")
#                 try:
#                     rootlink = sUtils.getRoot(bpy.data.objects[self.praefixNames(root['name'], "link")])
#                     rootlink['modelname'] = self.robot['name']
#                 except KeyError:
#                     log("Could not assign model name to root link.", "ERROR")
#         for link in self.robot['links']:
#             self.placeLinkSubelements(self.robot['links'][link])
#
#
#         print("\n\nCreating sensors...")
#         for s in self.robot['sensors']:
#             sensor = self.robot['sensors'][s]
#             self.createSensor(sensor)
#
#         for sensorname in self.robot['sensors']:
#             sensor = self.robot['sensors'][sensorname]
#             self.attachSensor(sensor)
#
#         print("\n\nCreating motors...")
#         for m in self.robot['motors']:
#             motor = self.robot['motors'][m]
#             self.createMotor(motor)
#
#         print("\n\nCreating controllers...")
#         for c in self.robot['controllers']:
#             controller = self.robot['controllers'][c]
#             self.createController(controller)
#
#         print("\n\nCreating groups...")
#         for g in self.robot['groups']:
#             group = self.robot['groups'][g]
#             self.createGroup(group)
#
#         print("\n\nCreating chains...")
#         for ch in self.robot['chains']:
#             chain = self.robot['chains'][ch]
#             self.createChain(chain)
#
#         print("\n\nCreating lights...")
#         for light in self.robot['lights']:
#             self.createLight(self.robot['lights'][light])
#
#         #self._apply_joint_angle_offsets()
#
#         # remove tmp dir containing extracted object files
#         if os.path.isdir(os.path.join(self.tmp_path, tmp_dir_name)):
#             shutil.rmtree(os.path.join(self.tmp_path, tmp_dir_name))
#
#         print('Done!')
#
#
#     def _debug_output(self):
#         """Writes the robot dictionary to a yaml file in the source file's directory
#
#         """
#         with open(self.filepath + '_ref_debug.yml', 'w') as outputfile:
#             outputfile.write(yaml.dump(self.robot)) #last parameter prevents inline formatting for lists and dictionaries

def deriveGenericEntity(entityobj, outpath=None):
    """This function handles an entity of unknown type by simply exporting its custom properties.

    :param entityobj: The object representing the entity.
    :type entityobj: bpy.types.Object
    :param outpath: If True data will be exported into subfolders.
    :type outpath: str
    :return: dict - An entry for the scenes entitiesList

    """
    log("Exporting " + nUtils.getObjectName(entityobj, 'entity') + " as entity of type 'generic", "INFO")
    entity = models.initObjectProperties(entityobj, 'entity', ['geometry'])
    return entity
