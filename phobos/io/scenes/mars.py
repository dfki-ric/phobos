#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

# def round_float(float_as_str, decimal=6):
#     """Casts 'float_as_str' to float and round to 'decimal' decimal places. The possible exception
#     **ValueError** is not handled in the function itself!

#     Args:
#       float_as_str(str: str): The string you want to cast into a float.
#       decimal(int, optional): The number of decimal places you want to round to. Its default is 6.

#     Returns:
#       float

#     """
#     return round(float(float_as_str), decimal)


# def pos_rot_tree_to_lists(position, rotation):
#     """Convert the xml representations of a position and a rotation to lists.
#     If either is 'None', return a list of zeroes instead.

#     Args:
#       position(str): The xml representation of a position.
#       rotation(str): The xml representation of a rotation.

#     Returns:
#       tuple of two lists

#     """
#     if position:
#         px = round_float(position.find('x').text)
#         py = round_float(position.find('y').text)
#         pz = round_float(position.find('z').text)
#     else:
#         px, py, pz = (0, 0, 0)
#     if rotation:
#         rw = round_float(rotation.find('w').text)
#         rx = round_float(rotation.find('x').text)
#         ry = round_float(rotation.find('y').text)
#         rz = round_float(rotation.find('z').text)
#     else:
#         rw, rx, ry, rz = (0, 0, 0, 0)

#     return [px, py, pz], [rw, rx, ry, rz]

# class MARSModelParser(RobotModelParser):
#     """
#     Class derived from RobotModelParser which parses a MARS scene.
#
#     Parsing MARS scenes is still experimental, so that the user may have to
#     adapt the scene in Blender after import.
#     """
#
#     def __init__(self, filepath, zipped=False):
#         """
#         Initialise a 'MARSModelParser' object and a number of containers
#         used to keep track of the parsed information and to apply it in
#         the right places.
#
#         Explanation of some of the containers:
#             - self.link_index_dict:
#                 {link index: link name}
#             - self.link_indices:
#                 set of all indices of xml nodes that are links
#             - self.material_indices:
#                 {material index: dictionary containing material information}
#             - self.applied_rel_id_poses:
#                 {link id: dictionary containing pose with parent pose
#                           applied}
#             - self.name_counter_dict:
#                 {link/visual/collision/material name:
#                  amount of how often the name already has been given to
#                  something (plus index for distinction)}
#
#         :param zipped: Indicate whether or not the file to parse from is zipped.
#         :type zipped: bool.
#         :return: Nothing.
#         """
#
#         RobotModelParser.__init__(self, filepath)
#
#         self.xml_tree = None
#
#         self.link_index_dict = {}
#         self.joint_index_dict = {}
#         self.motor_index_dict = {}
#         self.sensor_index_dict = {}
#         self.link_groups_group_order = {}
#         self.link_groups_link_order = {}
#         self.link_indices = set([])
#         self.material_indices = {}
#         self.vis_coll_groups = {}
#         self.applied_rel_id_poses = {}
#         self.missing_vis_geos = {}
#         self.missing_coll_geos = {}
#         self.name_counter_dict = {}
#         self.link_group_dict = {}
#         self.robot_states = {}
#         self.joint_info = {}
#         self.parent_joint_dict = {}
#
#         self.controller_counter = 1
#
#         self.zipped = zipped
#
#     def parseModel(self):
#         """
#         Parse a MARS scene from a '.scene' file.
#
#         Zipped '.scn' files will be handled accordingly.
#
#         :return: Nothing.
#         """
#         print("\nParsing MARS scene from", self.filepath)
#
#         if self.zipped:
#             filename = os.path.basename(self.filepath).split('.')[0] + '.scene'
#             archive = zipfile.ZipFile(self.filepath)
#             zipfiles = archive.namelist()
#             if filename not in zipfiles:
#                 for zf in zipfiles:
#                     if zf.endswith('.scene'):
#                         filename = zf
#                         break
#                 else:
#                     print('Failed to read from archive', self.filename + "."
#                           + "No scene file found.")
#             self.xml_tree = ET.parse(archive.open(filename))
#         else:
#             self.xml_tree = ET.parse(self.filepath)
#
#         root = self.xml_tree.getroot()
#
#         nodes = root.find('nodelist')
#         joints = root.find('jointlist')
#         sensors = root.find('sensorlist')
#         motors = root.find('motorlist')
#         controllers = root.find('controllerlist')
#         lights = root.find('lightlist')
#         material_list = root.find('materiallist')
#
#         self._parse_joints(joints)
#         self._get_links(nodes, joints)
#         self._apply_relative_ids(nodes)
#
#         self.robot['materials'] = self._parse_materials(material_list)
#
#         links = self._parse_links(nodes)
#         #self._add_parent_links(links)
#         self.robot['name'] = os.path.basename(self.filepath).split('.')[0]
#         self.robot['links'] = links
#         self.robot['joints'] = self._create_joints_dict()
#         #print(self.robot)
#         self.robot['motors'] = self._parse_motors(motors)
#         self.robot['sensors'] = self._parse_sensors(sensors)
#         self.robot['controllers'] = self._parse_controllers(controllers)
#         self.robot['groups'] = self.link_groups_group_order
#         self._parse_additional_visuals_and_collisions(self.robot, nodes)
#         self.robot['lights'] = self._parse_lights(lights, links)
#
#         for link in self.robot['links']:
#             handle_missing_geometry(self.missing_vis_geos[link], self.missing_coll_geos[link], self.robot['links'][link])
#
#         self.robot['states'] = self.robot_states
#
#         self._debug_output()
#
#         #print(self.joint_index_dict)
#
#     def _find_name(self, struct, prefix='', emerg='link'):
#         """
#         Find the name of an XML structure. If neither an attribute nor a tag named 'name'
#         is present, construct a new name using the 'emerg' parameter and the structure's
#         'index' tag, if present. A prefix is optional. If necessary, the name will be made
#         distinct by adding an index.
#
#         :param struct: The XML structure to find a name in.
#         :type struct: xml.etree.ElementTree.Element.
#         :param prefix: An optional prefix to add to the name.
#         :type prefix: str.
#         :param emerg: A string used to construct a name if none is found.
#         :type emerg: str.
#         :return: A name for the XML structure.
#         """
#         name = struct.get('name')
#         if name is None:
#             name_xml = struct.find('name')
#             if name_xml is None:
#                 index = struct.find('index')
#                 if index is None:
#                     name = emerg
#                 else:
#                     name = emerg + '_' + index.text
#             else:
#                 name = name_xml.text
#         if prefix is not '':
#             prefix += '_'
#         return self._get_unique_name(prefix + name)
#
#     def _get_links(self, nodes, joints):
#         """
#         Collect the indices of all nodes that are links inside
#         'self.link_indices'.
#
#         :param nodes: The 'nodes' element of the currently parsed scene.
#         :type nodes: xml.etree.ElementTree.Element.
#         :param joints: The 'joints' element of the currently parsed scene.
#         :type joints: xml.etree.ElementTree.Element
#         :return: Nothing.
#         """
#         if joints is not None:
#             for joint in joints:
#                 child = int(joint.find('nodeindex2').text)
#                 self.link_indices.update([child])
#         for node in nodes:
#             if node.find('relativeid') is None:
#                 self.link_indices.update([int(node.find('index').text)])
#
#     def _create_joints_dict(self):
#         """
#         Create a dictionary containing the necessary information for the robot
#         dictionary. Use the previously parsed information in 'self.joint_info'
#         and 'self.link_index_dict' for this.
#
#         :return: The created dictionary.
#         """
#         joints_dict = {}
#         for joint_name in self.joint_info:
#             joint = self.joint_info[joint_name]
#             joint_dict = {}
#             for info in ['name', 'type', 'angle_offset', 'axis1', 'limits']:
#                 if info in joint:
#                     joint_dict[info] = joint[info]
#             joint_dict['parent'] = self.link_index_dict[joint['parent_index']]
#             joint_dict['child'] = self.link_index_dict[joint['child_index']]
#             joints_dict[joint_name] = joint_dict
#         return joints_dict
#
#     def _parse_materials(self, materials_tree):
#         """
#         Parse the materials from the MARS scene.
#
#         :param materials_tree: The 'materiallist' element from the currently
#         parsed scene.
#         :type materials_tree: xml.etree.ElementTree.Element.
#         :return: A dictionary containing the parsed information.
#         """
#         materials_dict = {}
#         for material in materials_tree:
#             material_dict = {}
#             mat_id = int(material.find('id').text)
#
#             # check needs to be explicit for future versions
#             name = self._find_name(material, emerg='material')
#             material_dict['name'] = name
#
#             #print('DEFS:', dir(defs))
#             for xml_colour in MARScolordict:
#                 colour = material.find(xml_colour)
#                 if colour is not None:
#                     r = round_float(colour.find('r').text)
#                     g = round_float(colour.find('g').text)
#                     b = round_float(colour.find('b').text)
#                     a = round_float(colour.find('a').text)
#                     py_colour = MARScolordict[xml_colour]
#                     material_dict[py_colour] = [r, g, b, a]
#                     # for now:
#             #        material_dict['color'] = [r, g, b, a]
#
#             transparency = material.find('transparency')
#             if transparency is not None:
#                 material_dict['transparency'] = round_float(transparency.text)
#             else:
#                 material_dict['transparency'] = 0.0
#
#             shininess = material.find('shininess')
#             if shininess is not None:
#                 material_dict['shininess'] = round_float(shininess.text)
#             self.material_indices[mat_id] = name
#             materials_dict[name] = material_dict
#
#             if 'diffuseColor' in material_dict:
#                 materials.makeMaterial(name, tuple(material_dict['diffuseColor'][0:3]), (1, 1, 1), material_dict['diffuseColor'][-1])
#
#         return materials_dict
#
#     def _parse_geometry(self, vis_coll_dict, node, mode):
#         """
#         Parse the geometry of a visual or collision object and add it to a the
#         respective existing dictionary.
#
#         :param vis_coll_dict: The visual or collision dictionary the parsed
#         information should be added to.
#         :type vis_coll_dict: dict.
#         :param node: The 'node' element the information should be parsed from.
#         :type node: xml.etree.ElementTree.Element.
#         :param mode: Indicate whether the geometry dictionary is supposed to
#         a visual or a collision dictionary; options are 'visual' and 'collision'.
#         :return: True if geometry information has been found in 'node', False
#         else.
#         """
#         size = None
#         if mode == 'visual':
#             size = node.find('visualsize')
#             if size is None:
#                 size = node.find('extend')
#             mesh_file = node.find('filename')
#             if mesh_file is not None:
#                 geometry_type = 'mesh'
#             else:
#                 geometry_type = node.find('physicmode').text
#         elif mode == 'collision':
#             size = node.find('extend')
#             geometry_type = node.find('physicmode').text
#
#         geometry_dict = {}
#         geometry_dict['type'] = geometry_type
#         if geometry_type == 'box' or geometry_type == 'mesh':
#             x = round_float(size.find('x').text)
#             y = round_float(size.find('y').text)
#             z = round_float(size.find('z').text)
#             geometry_dict['size'] = [x, y, z]
#             if geometry_type == 'mesh':
#                 filename = node.find('filename').text
#                 geometry_dict['filename'] = filename
#         elif geometry_type == 'sphere' or geometry_type == 'cylinder' or geometry_type == 'capsule':
#             radius = round_float(size.find('x').text) / 2.0
#             geometry_dict['radius'] = radius
#             if geometry_type == 'cylinder' or geometry_type == 'capsule':
#                 height = round_float(size.find('z').text)
#                 geometry_dict['length'] = height
#                 # geometry_dict['height'] = height
#         elif geometry_type == 'plane':
#             x = round_float(size.find('x').text)
#             y = round_float(size.find('y').text)
#             geometry_dict['size'] = [x, y]
#
#         if geometry_dict == {}:
#             return False
#         else:
#             vis_coll_dict['geometry'] = geometry_dict
#             return True
#
#     #def _parse_visual(self, visuals_dict, node, missing_vis_geo, root_child):
#     def _parse_visual(self, visuals_dict, node, missing_vis_geo):
#         """
#         Parse information for a visual object from a 'node' element.
#         If no information for the visual's geometry can be parsed, indicate this by
#         adding the visual's name to 'missing_vis_geo'.
#
#         :param visuals_dict: The dictionary the parsed information should be added to.
#         :type visuals_dict: dict.
#         :param node: The 'node' element the information should be parsed from.
#         :type node: xml.etree.ElementTree.Element.
#         :param missing_vis_geo: A list of the names of visuals that do not have geometry
#         information.
#         :type missing_vis_geo: list.
#         :return: Nothing.
#         """
#         visual_dict = {}
#         name = self._find_name(node, prefix='visual')
#         visual_dict['name'] = name
#         index = int(node.find('index').text)
#
#         base_pose = self.applied_vis_col_poses[index]
#         base_pos = base_pose['translation']
#         rot = base_pose['rotation_quaternion']
#         pivot_xml = node.find('pivot')
#         if pivot_xml is not None:   # and not root_child:
#             pivot = [float(pivot_xml.find('x').text), float(pivot_xml.find('y').text), float(pivot_xml.find('z').text)]
#         else:
#             pivot = [0.0, 0.0, 0.0]
#
#         pos = base_pos
#         visual_dict['pose'] = calc_pose_formats(pos, rot, pivot=pivot)
#         #visual_dict['pose'] = calc_pose_formats(pos, rot)
#         mat_index = int(node.find('material_id').text)
#         visual_dict['material'] = self.material_indices[mat_index]
#
#         if not self._parse_geometry(visual_dict, node, 'visual'):
#             missing_vis_geo.append(name)
#
#         visuals_dict[name] = visual_dict
#
#     def _parse_collision(self, collisions_dict, node, missing_coll_geo):
#         """
#         Parse information for a collision object from a 'node' element.
#         If no information for the collision's geometry can be parsed, indicate this by
#         adding the collision's name to 'missing_coll_geo'.
#
#         :param collisions_dict: The dictionary the parsed information should be added to.
#         :type collisions_dict: dict.
#         :param node: The 'node' element the information should be parsed from.
#         :type node: xml.etree.ElementTree.Element.
#         :param missing_coll_geo: A list of the names of collisions that do not have geometry
#         information.
#         :type missing_coll_geo: list.
#         :return: Nothing.
#         """
#         collision_dict = {}
#         name = self._find_name(node, prefix='collision')
#         collision_dict['name'] = name
#         index = int(node.find('index').text)
#
#         #if name.startswith('collision_joint_sphere'):
#         #    position, rotation = pos_rot_tree_to_lists(None, None)
#         #    collision_dict['pose'] = calc_pose_formats(position, rotation)
#         #else:
#         collision_dict['pose'] = self.applied_vis_col_poses[index]
#
#         bitmask = node.find('coll_bitmask')
#         if bitmask is not None:
#             collision_dict['bitmask'] = int(float(bitmask.text))
#
#         if not self._parse_geometry(collision_dict, node, 'collision'):
#             missing_coll_geo.append(name)
#
#         max_contacts = node.find('cmax_num_contacts')
#         if max_contacts is not None:
#             collision_dict['max_contacts'] = int(max_contacts.text)
#
#         collisions_dict[name] = collision_dict
#
#     def _parse_inertial(self, link_dict, node):
#         """
#         Parse the inertial information of a node and write it into the
#         respective link's dictionary.
#
#         :param link_dict: The dictionary of for the link the inertial
#         information should be written to.
#         :type link_dict: dict.
#         :param node: The 'node' element the information should be parsed from.
#         :type node: xml.etree.ElementTree.Element.
#         :return: Nothing.
#         """
#         inertial_dict = {}
#
#         mass = node.find('mass')
#         if mass is not None:
#             inertial_dict['mass'] = round_float(mass.text)
#
#         inertia = node.find('inertia')
#         if inertia is not None and bool(inertia.text):  # 'inertia' states whether the defined inertia values are to be used
#             i00 = round_float(node.find('i00').text)
#             i01 = round_float(node.find('i01').text)
#             i02 = round_float(node.find('i02').text)
#             i11 = round_float(node.find('i11').text)
#             i12 = round_float(node.find('i12').text)
#             i22 = round_float(node.find('i22').text)
#             inertial_dict['inertia'] = [i00, i01, i02,
#                                              i11, i12,
#                                                   i22]
#
#         if inertial_dict is not {}:
#             # inertial pose is always origin for now
#             position, rotation = pos_rot_tree_to_lists(None, None)
#             inertial_dict['pose'] = calc_pose_formats(position, rotation)
#             inertial_dict['name'] = self._find_name(node, prefix='inertial')
#             link_dict['inertial'] = inertial_dict
#
#     def _get_unique_name(self, name):
#         """
#         Create a name that has not been used yet in the currently parsed scene
#         by adding an index to the original name if necessary.
#
#         :param name: The original name.
#         :type name: str.
#         :return: A unique version of the original name.
#         """
#         if name in self.name_counter_dict:
#             name_counter = self.name_counter_dict[name]
#             distinct_name = name + '_' + str(name_counter)
#             self.name_counter_dict[name] = name_counter + 1
#         else:
#             distinct_name = name
#             self.name_counter_dict[name] = 1
#         return distinct_name
#
#     def _parse_links(self, nodes):
#         """
#         Parse information for the links from the currently parsed scene.
#
#         :param nodes: The 'nodelist' element of the MARS scene.
#         :type nodes: xml.etree.ElementTree.Element.
#         :return: A dictionary containing the parsed information.
#         """
#         links_dict = {}
#         for node in nodes:
#             missing_vis_geo = []
#             missing_coll_geo = []
#             index = int(node.find('index').text)
#             name = self._find_name(node)
#
#             self.link_index_dict[index] = name
#
#             group_id = node.find('groupid')
#             if group_id is not None:
#                 group = int(group_id.text)
#             else:
#                 group = 0
#             node_group_dict = {'name': name,
#                                'index': index}
#             if group in self.link_groups_group_order:
#                 self.link_groups_group_order[group].append(node_group_dict)
#             else:
#                 self.link_groups_group_order[group] = [node_group_dict]
#
#             rel_id = node.find('relativeid')
#             if index in self.link_indices:
#                 link_dict = {}
#
#                 link_dict['name'] = name
#
#                 pose = self.applied_rel_id_poses[index]
#                 link_dict['pose'] = pose
#
#                 visuals_dict = {}
#                 #self._parse_visual(visuals_dict, node, missing_vis_geo)
#                 link_dict['visual'] = visuals_dict
#
#                 collisions_dict = {}
#                 #self._parse_collision(collisions_dict, node, missing_coll_geo)
#                 link_dict['collision'] = collisions_dict
#
#                 self._parse_inertial(link_dict, node)
#
#                 pivot = node.find('pivot')
#                 if pivot is not None:
#                     x = float(pivot.find('x').text)
#                     y = float(pivot.find('y').text)
#                     z = float(pivot.find('z').text)
#                     link_dict['pivot'] = [x, y, z]
#
#                 if rel_id is not None:
#                     link_dict['parent'] = int(rel_id.text)
#
#                 links_dict[name] = link_dict
#                 self.missing_vis_geos[name] = missing_vis_geo
#                 self.missing_coll_geos[name] = missing_coll_geo
#
#                 if index in self.parent_joint_dict:
#                     joint = self.joint_info[self.parent_joint_dict[index]]
#                     axis = joint['axis1']
#                     angle = joint['angle_offset']
#                     link_dict['axis1'] = axis
#                     link_dict['angle_offset'] = angle
#
#         for group_index in self.link_groups_group_order:
#             group = self.link_groups_group_order[group_index]
#             for node in group:
#                 if node['index'] in self.link_indices:
#                     link_index = node['index']
#                     self.link_groups_link_order[link_index] = group
#                     break
#
#         for link_index in self.link_groups_link_order:
#             group = self.link_groups_link_order[link_index]
#             for node in group:
#                 self.link_group_dict[node['index']] = link_index
#
#         for link_name in links_dict:
#             link_dict = links_dict[link_name]
#             if 'parent' in link_dict:
#                 link_dict['parent'] = self.link_index_dict[self.link_group_dict[link_dict['parent']]]
#
#         return links_dict
#
#     def _add_parent_links(self, links):
#         """
#         Replace the 'parent' place holders inside the parsed links with the
#         actual parent names.
#
#         :param links: A dictionary containing the parsed link information.
#         :type links: dict.
#         :return: Nothing.
#         """
#         for link_name in links:
#             link = links[link_name]
#             if 'parent' in link:
#                 rel_id = link['parent']
#                 if rel_id in self.vis_coll_groups:
#                     parent_id = self.vis_coll_groups[rel_id]
#                 else:
#                     parent_id = rel_id
#                 link['parent'] = self.link_index_dict[parent_id]
#
#     def _parse_additional_visuals_and_collisions(self, model, nodes):
#         """
#         Parse nodes that are no links as additional visual and collision
#         objects for the already parsed links.
#
#         :param model: The current scene's already parsed information.
#         :type model: dict.
#         :param nodes: The 'nodelist' element of the currently parse scene.
#         :type nodes: xml.etree.ElementTree.Element.
#         :return: Nothing.
#         """
#         for link in self.link_groups_link_order:
#             for node in self.link_groups_link_order[link]:
#                 self.vis_coll_groups[node['index']] = link
#
#         for node in nodes:
#             index = int(node.find('index').text)
#             #rel_id = node.find('relativeid')
#             #if rel_id is None:           # check if node is root
#             #    root_child = True
#             #else:
#             #    root_child = False
#             if index in self.vis_coll_groups:
#                 link_index = self.vis_coll_groups[index]
#                 group = self.link_groups_link_order[link_index]
#                 for group_node in group:
#                     if group_node['index'] == link_index:
#                         name = group_node['name']
#                         visuals_dict = model['links'][name]['visual']
#                         #self._parse_visual(visuals_dict, node, self.missing_vis_geos[name], root_child)
#                         self._parse_visual(visuals_dict, node, self.missing_vis_geos[name])
#                         model['links'][name]['visual'] = visuals_dict
#
#                         collisions_dict = model['links'][name]['collision']
#                         self._parse_collision(collisions_dict, node, self.missing_coll_geos[name])
#                         model['links'][name]['collision'] = collisions_dict
#
#                         break
#
#     def _parse_joints(self, joints):
#         """
#         Parse the joint information from the currently parsed scene.
#
#         :param joints: The 'jointlist' element of the currently parsed scene.
#         :type joints: xml.etree.ElementTree.Element.
#         :return: Nothing.
#         """
#         state_dict = {}
#         if joints is None:
#             return
#         for joint in joints:
#             joint_dict = {}
#             name = self._find_name(joint, emerg='joint')
#             joint_dict['name'] = name
#             mars_type = joint.find('type').text
#
#             lower_limit = None
#             xml_lower_limit = joint.find('lowStopAxis1')
#             if xml_lower_limit is not None:
#                 lower_limit = float(xml_lower_limit.text)
#             upper_limit = None
#             xml_upper_limit = joint.find('highStopAxis1')
#             if xml_upper_limit is not None:
#                 upper_limit = float(xml_lower_limit.text)
#
#             if upper_limit or lower_limit:
#                 if 'limits' not in joint_dict:
#                     joint_dict['limits'] = {}
#                 has_limits = True
#                 if upper_limit:
#                     joint_dict['limits']['upper'] = upper_limit
#                 if lower_limit:
#                     joint_dict['limits']['lower'] = lower_limit
#             else:
#                 has_limits = False
#
#             joint_dict['type'] = get_phobos_joint_name(mars_type, has_limits)
#
#             parent_index = int(joint.find('nodeindex1').text)
#             joint_dict['parent_index'] = parent_index
#             #joint_dict['parent'] = self.link_index_dict[parent_index]
#             child_index = int(joint.find('nodeindex2').text)
#             joint_dict['child_index'] = child_index
#             self.parent_joint_dict[child_index] = name
#             #joint_dict['child'] = self.link_index_dict[child_index]
#
#             xml_offset = joint.find('angle1_offset')
#             if xml_offset is not None:
#                 dict_offset = float(xml_offset.text)
#             else:
#                 dict_offset = 0.0
#
#             state_dict[name] = dict_offset
#             joint_dict['angle_offset'] = dict_offset
#
#             xml_axis = joint.find('axis1')
#             if xml_axis is not None:
#                 x = float(xml_axis.find('x').text)
#                 y = float(xml_axis.find('y').text)
#                 z = float(xml_axis.find('z').text)
#                 dict_axis = [x, y, z]
#             else:
#                 dict_axis = [0.0, 0.0, 0.0]
#
#             joint_dict['axis1'] = dict_axis
#
#             self.joint_info[name] = joint_dict
#             self.joint_index_dict[int(joint.find('index').text)] = name
#
#         self.robot_states['start'] = state_dict
#
#     def _apply_relative_ids(self, nodes):
#         """
#         Calculate absolute poses from the relative ones given in the scene by
#         using the nodes' 'relativeid' tags. Store these poses into
#         'self.applied_vis_col_poses'.
#
#         :param nodes: The 'nodelist' element of the currently parse scene.
#         :type nodes: xml.etree.ElementTree.Element.
#         :return: Nothing.
#         """
#         absolute_poses = {}
#         absolute_vis_coll_poses = {}
#         relative_poses = {}
#         for node in nodes:
#             index = int(node.find('index').text)
#             xml_position = node.find('position')
#             xml_rotation = node.find('rotation')
#             position, rotation = pos_rot_tree_to_lists(xml_position, xml_rotation)
#             #if index in self.parent_joint_dict:
#             #    joint = self.joint_info[self.parent_joint_dict[index]]
#             #    axis = joint['axis']
#             #    angle = joint['angle_offset']
#             #    pose = calc_pose_formats(position, rotation, angle, axis)
#             #else:
#             pose = calc_pose_formats(position, rotation)
#
#             rel_id_xml = node.find('relativeid')
#             #if index in self.link_indices:
#             #    link_poses[index] = pose
#             roots = []
#             if rel_id_xml is None:
#                 absolute_poses[index] = pose
#                 #absolute_vis_coll_poses[index] = pose
#                 roots.append(index)
#                 relative_poses[index] = {'pose': pose, 'rel_id': -1}
#             else:
#                 rel_id = int(rel_id_xml.text)
#                 relative_poses[index] = {'pose': pose, 'rel_id': rel_id}
#
#             num_rel_poses = -1
#         while relative_poses:
#             to_delete = []
#             # check for infinite loop (happens if referenced node does not exist):
#             if len(relative_poses) == num_rel_poses:
#                 print('Error: non-existent relative id')
#                 break
#             num_rel_poses = len(relative_poses)
#             for index in relative_poses:
#                 #print('link_poses:', link_poses)
#                 relative_pose = relative_poses[index]
#                 rel_id = relative_pose['rel_id']
#                 if rel_id in absolute_poses or rel_id == -1:
#                     #print('index:', index)
#                     #print('rel_id:', rel_id)
#                     if rel_id in self.link_indices or rel_id == -1: # and rel_id not in roots:
#                         absolute_poses[index] = relative_pose['pose']
#                         absolute_vis_coll_poses[index] = relative_pose['pose']
#                     else:
#                         reference_pose = absolute_poses[rel_id]
#                         rel_matrix = mathutils.Matrix(relative_pose['pose']['matrix']).to_4x4()
#                         ref_matrix = mathutils.Matrix(reference_pose['matrix']).to_4x4()
#                         applied_matrix = ref_matrix * rel_matrix # or the other way round?
#                         #print('relative:\n', rel_matrix)
#                         #print(rel_matrix.to_quaternion())
#                         #print('reference:\n', ref_matrix)
#                         #print(ref_matrix.to_quaternion())
#                         #print('applied:\n', applied_matrix)
#                         #print(applied_matrix.to_quaternion())
#                         loc, rot, scale = applied_matrix.decompose()
#                         applied_pos = [loc.x, loc.y, loc.z]
#                         applied_rot = [rot.w, rot.x, rot.y, rot.z]
#                         absolute_poses[index] = calc_pose_formats(applied_pos, applied_rot)
#                         absolute_vis_coll_poses[index] = calc_pose_formats(applied_pos, applied_rot)
#                     if index in self.link_indices:
#                         pos, rot = pos_rot_tree_to_lists(None, None)
#                         absolute_vis_coll_poses[index] = calc_pose_formats(pos, rot)
#                     to_delete.append(index)
#             for index in to_delete:
#                 del relative_poses[index]
#         self.applied_rel_id_poses = absolute_poses
#         self.applied_vis_col_poses = absolute_vis_coll_poses
#
#     def _parse_sensors(self, sensors):
#         """
#         Parse the sensor information from the currently parsed scene.
#
#         :param sensors: The scene's 'sensorlist' element.
#         :type sensors: xml.etree.ElementTree.Element.
#         :return: A dictionary containing the parsed information.
#         """
#         sensors_dict = {}
#         for sensor in sensors:
#             sensor_dict = {}
#             name = self._find_name(sensor, emerg='sensor')
#             sensor_dict['name'] = name
#             index = sensor.find('index')
#             if index is not None:
#                 self.sensor_index_dict[int(index.text)] = name
#
#             xml_rate = sensor.find('rate')
#             if xml_rate is not None:
#                 sensor_dict['rate'] = float(xml_rate.text)
#
#             xml_link = sensor.find('nodeID')
#             if xml_link is not None:
#                 sensor_dict['link'] = self.link_index_dict[int(xml_link.text)]
#
#             xml_joint = sensor.find('jointID')
#             if xml_joint is not None:
#                 sensor_dict['joint'] = self.joint_index_dict[int(xml_joint.text)]
#
#             type = sensor.get('type')
#             sensor_dict['type'] = type
#
#             if type in ['JointArray', 'JointAVGTorque', 'JointLoad', 'JointPosition', 'JointTorque', 'JointVelocity']:
#                 sensor_dict['joints'] = []
#                 for xml_id in sensor.findall('id'):
#                     sensor_dict['joints'].append(self.joint_index_dict[int(xml_id.text)])
#             elif type == 'MotorCurrent':
#                 sensor_dict['motors'] = []
#                 for xml_id in sensor.findall('id'):
#                     sensor_dict['motors'].append(self.motor_index_dict[int(xml_id.text)])
#             elif type in ['NodeAngularVelocity', 'NodeCOM', 'NodeContactForce', 'NodePosition', 'NodeRotation', 'NodeVelocity']:
#                 sensor_dict['links'] = []
#                 for xml_id in sensor.findall('id'):
#                     sensor_dict['links'].append(self.link_index_dict[int(xml_id.text)])
#
#             sensors_dict[name] = sensor_dict
#         return sensors_dict
#
#     def _parse_motors(self, motors):
#         """
#         Parse the motor information from the currently parsed scene.
#
#         :param motors: The scene's 'motorlist' element.
#         :type motors: xml.etree.ElementTree.Element.
#         :return: A dictionary containing the parsed information.
#         """
#         motors_dict = {}
#         if motors is None:
#             return motors_dict
#         for motor in motors:
#             motor_dict = {}
#             name = self._find_name(motor, emerg='motor')
#             motor_dict['name'] = name
#
#             joint_index = int(motor.find('jointIndex').text)
#             joint_name = self.joint_index_dict[joint_index]
#             motor_dict['joint'] = joint_name
#             joint = self.robot['joints'][joint_name]
#             if not 'limits' in joint:
#                 joint['limits'] = {}
#             velocity = float(motor.find('maximumVelocity').text)
#             effort = float(motor.find('motorMaxForce').text)
#             joint['limits']['velocity'] = velocity
#             joint['limits']['effort'] = effort
#             motor_dict['velocity'] = velocity
#             motor_dict['effort'] = effort
#             #motor_dict['velocity'] = float(motor.find('maximumVelocity').text)
#             #motor_dict['effort'] = float(motor.find('motorMaxForce').text)
#
#             type_num = int(motor.find('type').text)
#             if type_num == 1:
#                 motor_dict['type'] = 'PID'
#                 for value in ['p', 'i', 'd']:
#                     motor_dict[value] = float(motor.find(value).text)
#             elif type_num == 2:
#                 motor_dict['type'] = 'DC'
#
#             self.motor_index_dict[int(motor.find('index').text)] = name
#
#             motors_dict[name] = motor_dict
#
#         return motors_dict
#
#     def _parse_controllers(self, controllers):
#         """
#         Parse the controller information from the currently parsed scene.
#
#         :param controllers: The scene's 'controllerlist' element.
#         :type controllers: xml.etree.ElementTree.Element.
#         :return: A dictionary containing the parsed information.
#         """
#         controllers_dict = {}
#         if controllers:
#             for controller in controllers:
#                 controller_dict = {}
#                 name = self._find_name(controller, emerg='controller')
#                 self.controller_counter += 1
#                 controller_dict['name'] = name
#                 controller_dict['rate'] = float(controller.find('rate').text)
#                 sensor_list = []
#                 for id in controller.findall('sensorid'):
#                     sensor_list.append(self.sensor_index_dict[int(id.text)])
#                 if sensor_list != []:
#                     controller_dict['sensors'] = sensor_list
#                 motor_list = []
#                 for id in controller.findall('motorid'):
#                     motor_list.append(self.motor_index_dict[int(id.text)])
#                 if motor_list != []:
#                     controller_dict['motors'] = motor_list
#
#                 controllers_dict[name] = controller_dict
#
#         return controllers_dict
#
#     def _parse_lights(self, lights, links):
#         """
#         Parse the light information from the currently parsed scene.
#
#         :param lights: The scene's 'lightlist' element.
#         :type lights: xml.etree.ElementTree.Element.
#         :return: A dictionary containing the parsed information.
#         """
#         lights_dict = {}
#         if lights:
#             for light in lights:
#                 light_dict = {}
#                 name = self._find_name(light, emerg='light')
#                 light_dict['name'] = name
#
#                 dims = ['x', 'y', 'z']
#                 position_xml = light.find('position')
#                 position = [float(position_xml.find(dim).text) for dim in dims]
#
#                 look_at = light.find('lookat')
#                 direction = [float(look_at.find(dim).text) - position[i] for (i, dim) in zip(range(len(position)), dims)]
#                 rotation = mathutils.Vector((1, 0, 0)).rotation_difference(direction).to_euler()
#                 rel_pose = calc_pose_formats(position, rotation)
#
#                 node_index_xml = light.find('nodeIndex')
#                 if node_index_xml is not None:
#                     node_index = int(node_index_xml.text)
#                     if node_index in self.link_index_dict:
#                         parent = self.link_index_dict[node_index]
#                         light_dict['parent'] = parent
#                         base_matrix = mathutils.Matrix(links[parent]['pose']['matrix']).to_4x4()
#                         abs_matrix = base_matrix * mathutils.Matrix(rel_pose['matrix']).to_4x4()
#                         loc, rot, scale = abs_matrix.decompose()
#                         applied_pos = [loc.x, loc.y, loc.z]
#                         applied_rot = [rot.w, rot.x, rot.y, rot.z]
#                         abs_pose = calc_pose_formats(applied_pos, applied_rot)
#                     else:
#                         log("Node with index", node_index, "is not a link. Light '" + light + "' was not attached to a node.\n" \
#                             + "The light's pose will differ from the specification.")
#                         abs_pose = rel_pose
#                 else:
#                     abs_pose = rel_pose
#
#                 light_dict['pose'] = abs_pose
#
#                 colours_dict = {}
#                 for colour_str in ['ambient', 'diffuse', 'specular']:
#                     colour = light.find(colour_str)
#                     if colour is not None:
#                         colour_dict = {}
#                         for value in ['a', 'r', 'g', 'b']:
#                             colour_dict[value] = float(colour.find(value).text)
#                         colours_dict[colour_str] = colour_dict
#                 if colours_dict is not {}:
#                     light_dict['color'] = colours_dict
#
#                 attenuation_dict = {}
#                 for att_str in ['constant', 'linear', 'quadratic']:
#                     att = light.find(att_str + 'Attenuation')
#                     if att is not None:
#                         attenuation_dict[att_str] = float(att.text)
#                 if attenuation_dict is not {}:
#                     light_dict['attenuation'] = attenuation_dict
#
#                 #for value_str in ['angle', 'exponent']:
#                     #value = light.find(value_str)
#                     #if value is not None:
#                     #    light_dict[value_str] = float(value.text)
#
#                 angle = light.find('angle')
#                 if angle is not None:
#                     light_dict['angle'] = math.radians(float(angle.text))
#                 exponent = light.find('exponent')
#                 if exponent is not None:
#                     light_dict['exponent'] = float(exponent.text)
#
#                 light_type = light.find('type')
#                 if light_type is not None:
#                     type_num = int(light_type.text)
#                     if type_num == 1:
#                         light_dict['type'] = 'omnilight'
#                     elif type_num == 2:
#                         light_dict['type'] = 'spotlight'
#
#                 directional = light.find('directional')
#                 if directional is not None:
#                     light_dict['directional'] = bool(directional.text)
#
#                 lights_dict[name] = light_dict
#
#         return lights_dict
