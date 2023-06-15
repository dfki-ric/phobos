# #!/usr/bin/python3
# # coding=utf-8
#
# # -------------------------------------------------------------------------------
# # This file is part of Phobos, a Blender Add-On to edit robot models.
# # Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
# #
# # You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# # If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# # -------------------------------------------------------------------------------
#
# """
# Contains the functions for the srdf entity.
# """
#
# import itertools
# import os
# import xml.etree.ElementTree as ET
# from phobos.blender.utils.io import l2str, xmlline, indent, xmlHeader
#
#
# def exportSRDF(model, path, mesh_format=''):
#     """This function exports the SRDF-relevant data from the dictionary to a specified path.
#     Further detail on different elements of SRDF:
#
#     <group>
#     Groups in SRDF can contain *links*, *joints*, *chains* and other *groups* (the latter two of which have to be specified
#     upstream. As nested groups is just a shortcut for adding links and joints to a group, it is not supported and the
#     user will have to add all links and joints explicitly to each group.
#     Originally both links and their associated parent joints were added. SRDF however implicitly assumes this, so the
#     current implementation only adds the links.
#
#     <chain>
#     Chains are elements to simplify defining groups and are supported. The dictionary also contains a list of all
#     elements belonging to that chain, which is discarded and not written to SRDF, however. It might be written to SMURF
#     in the future.
#
#     <link_sphere_approximatio>
#     SRDF defines the convention that if no sphere is defined, one large sphere is
#     assumed for that link. If one wants to have no sphere at all, it is necessary to define a sphere of radius 0.
#     As one large sphere can be explicitly added by the user and should be if that is what he intends (WYSIWYG),
#     we add a sphere of radius 0 by default if no sphere is specified.
#
#     <passive_joint>
#     Marks a joint as passive.
#
#     <disable_collisions>
#     Disables collisions between pairs of links to simplify collision checking and avoid collisions
#     of parents and children at their joints.
#
#
#     Currently not supported:
#     - <group_state>
#     - <virtual_joint>
#
#     Args:
#       model(dict): a robot model dictionary.
#       path(str): the outpath for the file.
#       mesh_format: (Default value = '')
#
#     Returns:
#
#     """
#     output = [xmlHeader, indent + '<robot name="' + model['name'] + '">\n\n']
#     sorted_group_keys = sorted(model['groups'])
#     for groupname in sorted_group_keys:
#         output.append(indent * 2 + '<group name="' + groupname + '">\n')
#         # TODO: once groups are implemented, this should be sorted aswell:
#         for member in model['groups'][groupname]:
#             output.append(indent * 3 + '<' + member['type'] + ' name="' + member['name'] + '" />\n')
#         output.append(indent * 2 + '</group>\n\n')
#     sorted_chain_keys = sorted(model['chains'].keys())
#     for chainname in sorted_chain_keys:
#         output.append(indent * 2 + '<group name="' + chainname + '">\n')
#         chain = model['chains'][chainname]
#         output.append(
#             indent * 3
#             + '<chain base_link="'
#             + chain['start']
#             + '" tip_link="'
#             + chain['end']
#             + '" />\n'
#         )
#         output.append(indent * 2 + '</group>\n\n')
#     # TODO delete me?
#     # for joint in model['state']['joints']:
#     #    pass
#     # passive joints
#     sorted_joint_keys = sorted(model['joints'].keys())
#     for joint in sorted_joint_keys:
#         try:
#             if model['joints'][joint]['passive']:
#                 output.append(
#                     indent * 2 + '<passive_joint name="' + model['links'][joint]['name'] + '"/>\n\n'
#                 )
#         except KeyError:
#             pass
#     sorted_link_keys = sorted(model['links'].keys())
#     for link in sorted_link_keys:
#         if len(model['links'][link]['approxcollision']) > 0:
#             output.append(
#                 indent * 2
#                 + '<link_sphere_approximation link="'
#                 + model['links'][link]['name']
#                 + '">\n'
#             )
#             # TODO: there does not seem to be a way to sort the spheres if there are multiple
#             for sphere in model['links'][link]['approxcollision']:
#                 output.append(
#                     xmlline(
#                         3,
#                         'sphere',
#                         ('center', 'radius'),
#                         (l2str(sphere['center']), sphere['radius']),
#                     )
#                 )
#             output.append(indent * 2 + '</link_sphere_approximation>\n\n')
#         else:
#             output.append(
#                 indent * 2
#                 + '<link_sphere_approximation link="'
#                 + model['links'][link]['name']
#                 + '">\n'
#             )
#             output.append(xmlline(3, 'sphere', ('center', 'radius'), ('0.0 0.0 0.0', '0')))
#             output.append(indent * 2 + '</link_sphere_approximation>\n\n')
#     # calculate collision-exclusive links
#     collisionExclusives = []
#     for combination in itertools.combinations(model['links'], 2):
#         link1 = model['links'][combination[0]]
#         link2 = model['links'][combination[1]]
#         # TODO: we might want to automatically add parent/child link combinations
#         try:
#             if link1['collision_bitmask'] & link2['collision_bitmask'] == 0:
#                 # TODO delete me?
#                 # output.append(xmlline(2, 'disable_collisions', ('link1', 'link2'), (link1['name'], link2['name'])))
#                 collisionExclusives.append((link1['name'], link2['name']))
#         except KeyError:
#             pass
#     with open(os.path.join(path, model['name'] + '.srdf'), 'w') as outputfile:
#         outputfile.write(''.join(output))
#
#
# def parseSRDFModel(self, robot):
#     """
#
#     Args:
#       robot:
#
#     Returns:
#
#     """
#     collision_Exclusives = self.buildCollisionExclusives()
#     collision_Dic = self.buildCollisionDictionary(collision_Exclusives, robot)
#     collision_Groups = self.buildCollisionGroups(collision_Dic)
#     robot = self.buildBitmasks(collision_Groups, robot)
#     return robot
#
#
# def buildBitmasks(self, collision_Groups, robot):
#     """
#
#     Args:
#       collision_Groups:
#       robot:
#
#     Returns:
#
#     """
#     bits = len(collision_Groups)
#     if bits > 20:
#         # CHECK this was moved to logging. Is it printed twice?
#         print("The Blender bitmask is not capable of more than 20 bit. The bitmask will be cutted!")
#         bits = 20
#     for link in robot['links']:
#         for i in range(0, bits):
#             if link in collision_Groups[i]:
#                 for coll in robot['links'][link]['collision']:
#                     try:
#                         robot['links'][link]['collision'][coll]['bitmask'] += 2 ** i
#                     except KeyError:
#                         robot['links'][link]['collision'][coll]['bitmask'] = 2 ** i
#     return robot
#
#
# def buildCollisionExclusives(self):
#     """TODO Missing documentation"""
#     print("\nParsing SRDF extensions from", self.filepath)
#     self.tree = ET.parse(self.filepath)
#     self.root = self.tree.getroot()
#
#     collision_Exclusives = []
#     for disabled_coll in self.root.iter('disable_collisions'):
#         pair = (disabled_coll.attrib['link1'], disabled_coll.attrib['link2'])
#         collision_Exclusives.append(pair)
#         # TODO delete me?
#         # print("Append ", pair, " to collision Exclusives")
#     return collision_Exclusives
#
#
# def buildCollisionDictionary(self, collision_exclusives, robot):
#     """
#
#     Args:
#       collision_exclusives:
#       robot:
#
#     Returns:
#
#     """
#     dic = {}
#     for pair in collision_exclusives:
#         if 'root' in pair or (
#             pair[0] != robot['joints'][pair[1]]['parent']
#             and pair[1] != robot['joints'][pair[0]]['parent']
#         ):
#             if pair[0] not in dic:
#                 dic[pair[0]] = []
#                 dic[pair[0]].append(pair[1])
#             else:
#                 if pair[1] not in dic[pair[0]]:
#                     dic[pair[0]].append(pair[1])
#             if pair[1] not in dic:
#                 dic[pair[1]] = []
#                 dic[pair[1]].append(pair[0])
#             else:
#                 if pair[0] not in dic[pair[1]]:
#                     dic[pair[1]].append(pair[0])
#         else:
#             pass
#             # TODO handle this somehow...
#             # print("Pair: ", pair, " not included")
#     print("Collision Dictionary:\n", dic)
#     return dic
#
#
# def checkGroup(self, group, colls):
#     """
#
#     Args:
#       group:
#       colls:
#
#     Returns:
#
#     """
#     cut = []
#     for elem in group:
#         if elem in colls:
#             cut.append(elem)
#     if len(cut) == len(group):
#         return cut
#     else:
#         return []
#
#
# def processGroup(self, group, link, colls):
#     """
#
#     Args:
#       group:
#       link:
#       colls:
#
#     Returns:
#
#     """
#     if link in group:
#         for coll in colls:
#             if coll in group:
#                 colls.remove(coll)
#     else:
#         cut = self.checkGroup(group, colls)
#         if len(cut) > 0:
#             group.append(link)
#             for l in cut:
#                 colls.remove(l)
#
#
# def buildCollisionGroups(self, dic):
#     """
#
#     Args:
#       dic:
#
#     Returns:
#
#     """
#     groups = []
#     for link in dic:
#         # TODO remove me?
#         # print("Current link: ", link)
#         colls = dic[link]
#         # TODO remove me?
#         # print("Current colls: ", colls)
#         for group in groups:
#             # TODO remove me?
#             # print("Current group: ", group)
#             self.processGroup(group, link, colls)
#         while len(colls) > 0:
#             newgroup = [link, colls.pop()]
#             groups.append(newgroup)
#             self.processGroup(newgroup, link, colls)
#     print("Number of collision Groups: ", len(groups))
#     # print ("Collision Groups:\n", groups)
#     return groups
#
#
# # registering export functions of types with Phobos
# entity_type_dict = {'srdf': {'export': exportSRDF, 'extensions': ('srdf', 'xml')}}
