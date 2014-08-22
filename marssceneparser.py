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

File marssceneparser.py

Created ???

@author: Stefan Rahms
"""

import xml.etree.ElementTree as ET


colour_dict = {'diffuseFront': 'diffuseColor',
              'ambientFront': 'ambientColor',
              'emissionFront': 'emissionColor',
              'specularFront': 'specularColor'}

def round_float(float_as_str, decimal=6):
    '''
    does not seem to work right for some reason
    '''
    return round(float(float_as_str), decimal)

class MARS_Scene_Parser(object):
    '''
    '''
    def __init__(self):
        '''
        '''
        self.xml_tree = None
        self.link_index_dict = {}
        self.link_groups = {}
        self.material_indices = {}
        self.link_indices = set([])
        self.vis_coll_groups = {}
        self.base_poses = {}

    def _read_scene(self, filename):
        '''
        '''
        tree = ET.parse(filename)
        self.xml_tree = tree
        return tree

    def _get_pose(self, node):
        '''
        '''
        position = node.find('position')
        px = round_float(position.find('x').text)
        py = round_float(position.find('y').text)
        pz = round_float(position.find('z').text)
        rotation = node.find('rotation')
        rw = round_float(rotation.find('w').text)
        rx = round_float(rotation.find('x').text)
        ry = round_float(rotation.find('y').text)
        rz = round_float(rotation.find('z').text)
        return [px, py, pz, rw, rx, ry, rz]

    def _get_links(self, nodes, joints):
        '''
        '''
        for joint in joints:
            parent = int(joint.find('nodeindex1').text)
            self.link_indices.update([parent])
            child = int(joint.find('nodeindex2').text)
            self.link_indices.update([child])

    def _parse_materials(self, materials):
        '''
        done
        '''
        for material in materials:
            material_dict = {}
            mat_id = int(material.find('id').text)

            # check needs to be explicit for future versions
            if material.find('name') is not None:
                name = material.find('name').text
            else:
                name = 'material_' + str(mat_id)
            material_dict['name'] = name

            for xml_colour in colour_dict:
                colour = material.find(xml_colour)
                if colour is not None:
                    r = round_float(colour.find('r').text)
                    g = round_float(colour.find('g').text)
                    b = round_float(colour.find('b').text)
                    a = round_float(colour.find('a').text)
                    py_colour = colour_dict[xml_colour]
                    material_dict[py_colour] = [r, g, b, a]

            transparency = material.find('transparency')
            if transparency is not None:
                material_dict['transparency'] = round_float(transparency.text)
            else:
                material_dict['transparency'] = 0.0

            material_dict['shininess'] = round_float(material.find('shininess').text)

            self.material_indices[mat_id] = material_dict

    def _parse_geometry(self, node, mode):
        '''
        mesh incomplete
        '''
        size = None
        if mode == 'visual':
            size = node.find('visualsize')
        elif mode == 'collision':
            size = node.find('extend')

        geometry_dict = {}
        geometry_type = node.find('physicmode').text
        geometry_dict['type'] = geometry_type

        if geometry_type == 'box' or geometry_type == 'mesh':
            x = round_float(size.find('x').text)
            y = round_float(size.find('y').text)
            z = round_float(size.find('z').text)
            geometry_dict['size'] = [x, y, z]
            if geometry_type == 'mesh':
                filename = node.find('origname').text
                geometry_dict['filename'] = filename
        elif geometry_type == 'sphere' or geometry_type == 'cylinder' or geometry_type == 'capsule':
            radius = round_float(size.find('x').text) / 2.0
            geometry_dict['radius'] = radius
            if geometry_type == 'cylinder' or geometry_type == 'capsule':
                height = round_float(size.find('z').text)
                geometry_dict['height'] = height
        elif geometry_type == 'plane':
            x = round_float(size.find('x').text)
            y = round_float(size.find('y').text)
            geometry_dict['size'] = [x, y]
        return geometry_dict

    def _parse_visual(self, visuals_dict, node):
        '''
        '''
        visual_dict = {}
        name = node.get('name')
        index = int(node.find('index').text)

        pose = self.base_poses[index]
        visual_pose = [0.0]*len(pose)
        visual_position = node.find('visualposition')
        if visual_position is not None:
            visual_pose[0] = round_float(visual_position.find('x').text)
            visual_pose[1] = round_float(visual_position.find('y').text)
            visual_pose[2] = round_float(visual_position.find('z').text)
        visual_rotation = node.find('visualrotation')
        if visual_rotation is not None:
            visual_pose[3] = round_float(visual_rotation.find('w').text)
            visual_pose[4] = round_float(visual_rotation.find('x').text)
            visual_pose[5] = round_float(visual_rotation.find('y').text)
            visual_pose[6] = round_float(visual_rotation.find('z').text)
        abs_pose = []
        for p, vp in zip(pose, visual_pose):
            abs_pose.append(p+vp)
        visual_dict['pose'] = abs_pose

        mat_index = int(node.find('material_id').text)
        visual_dict['material'] = self.material_indices[mat_index]

        geometry_dict = self._parse_geometry(node, 'visual')
        visual_dict['geometry'] = geometry_dict

        visuals_dict[name] = visual_dict

    def _parse_collision(self, collisions_dict, node):
        '''
        max_contacts not in example. can it occur?
        '''
        collision_dict = {}
        name = node.get('name')
        index = int(node.find('index').text)
        collision_dict['pose'] = self.base_poses[index]

        bitmask = int(node.find('coll_bitmask').text)
        collision_dict['bitmask'] = bitmask

        geometry_dict = self._parse_geometry(node, 'collision')
        collision_dict['geometry'] = geometry_dict

        max_contacts = node.find('cmax_num_contacts')
        if max_contacts is not None:
            collision_dict['max_contacts'] = int(max_contacts.text)

        collisions_dict[name] = collision_dict


    def _parse_links(self, nodes):
        '''
        '''
        links_dict = {}
        for node in nodes:
            index = int(node.find('index').text)
            name = node.get('name')
            self.link_index_dict[index] = name

            group = int(node.find('groupid').text)
            node_group_dict = {'name': name,
                               'index': index}
            if group in self.link_groups:
                self.link_groups[group].append(node_group_dict)
            else:
                self.link_groups[group] = [node_group_dict]

            if index in self.link_indices:
                link_dict = {}

                link_dict['filename'] = node.find('filename').text

                pose = self.base_poses[index]
                link_dict['pose'] = pose

                visuals_dict = {}
                self._parse_visual(visuals_dict, node)
                link_dict['visuals'] = visuals_dict

                collisions_dict = {}
                self._parse_collision(collisions_dict, node)
                link_dict['collisions'] = collisions_dict

                inertial_dict = {}
                mass = round_float(node.find('mass').text)
                inertial_dict['mass'] = mass
                inertia = node.find('inertia')
                if inertia is None or not bool(inertia.text):
                    inertial_dict['inertia'] = [1.0, 0.0, 0.0,
                                                     1.0, 0.0,
                                                          1.0]
                else:
                    i00 = round_float(node.find('i00').text)
                    i01 = round_float(node.find('i01').text)
                    i02 = round_float(node.find('i02').text)
                    i11 = round_float(node.find('i11').text)
                    i12 = round_float(node.find('i12').text)
                    i22 = round_float(node.find('i22').text)
                    inertial_dict['inertia'] = [i00, i01, i02,
                                                     i11, i12,
                                                          i22]
                links_dict[name] = link_dict

            else:
                self.vis_coll_groups[index] = group

        return links_dict

    def _parse_additional_visuals_and_collisions(self, model, nodes):
        '''
        not well-tested yet
        '''
        for node in nodes:
            index = int(node.find('index').text)
            if index in self.vis_coll_groups:
                group = self.link_groups[self.vis_coll_groups[index]]
                for group_node in group:
                    if group_node['index'] in self.link_indices:
                        visuals_dict = model['links'][group_node['name']]['visuals']
                        self._parse_visual(visuals_dict, node)
                        model['links'][group_node['name']]['visuals'] = visuals_dict

                        collisions_dict = model['links'][group_node['name']]['collisions']
                        self._parse_collision(collisions_dict, node)
                        model['links'][group_node['name']]['collisions'] = collisions_dict

                        break

    def _parse_joints(self, joints):
        '''
        '''
        joints_dict = {}
        for joint in joints:
            joint_dict = {}
            name = joint.get('name')
            joint_dict['jointType'] = joint.find('type').text

            parent_index = int(joint.find('nodeindex1').text)
            joint_dict['parent'] = self.link_index_dict[parent_index]
            child_index = int(joint.find('nodeindex2').text)
            joint_dict['child'] = self.link_index_dict[child_index]

            joints_dict[name] = joint_dict
        return joints_dict

    def _apply_relative_ids(self, nodes):
        '''
        seems to work but not well-tested yet
        '''
        base_nodes = {}
        rel_nodes = {}
        for node in nodes:
            index = int(node.find('index').text)
            pose = self._get_pose(node)
            if node.find('relativeid') is not None:
                node_dict = {'pose': pose,
                             'rel_id': int(node.find('relativeid').text)}
                rel_nodes[index] = node_dict
            else:
                base_nodes[index] = pose
        num_rel_nodes = -1
        while rel_nodes:
            to_delete = []
            if len(rel_nodes) == num_rel_nodes:             # check for infinite loop (happens if referenced node does not exist)
                print('Error: non-existant relative id')
                break
            num_rel_nodes = len(rel_nodes)
            for rel_index in rel_nodes:
                rel_node = rel_nodes[rel_index]
                base = rel_node['rel_id']
                if base in base_nodes:
                    base_pose = base_nodes[base]
                    rel_pose = rel_node['pose']
                    applied_pose = []
                    for bp, rp in zip(base_pose, rel_pose):
                        applied_pose.append(bp+rp)
                    base_nodes[rel_index] = applied_pose
                    to_delete.append(rel_index)
            for index in to_delete:
                del rel_nodes[index]
        self.base_poses = base_nodes

    def _parse_sensors(self, sensors):
        '''
        'link' is missing in manual
        '''
        sensors_dict = {}
        for sensor in sensors:
            sensor_dict = {}
            name = sensor.get('name')
            sensor_dict['link'] = None  # where to get this?
            sensor_dict['sensorType'] = sensor.get('type')
            sensors_dict[name] = sensor_dict

    def _parse_motors(self, motors):
        '''
        don't know yet what motors look like
        '''
        pass

    def _parse_controllers(self, controllers):
        '''
        don't know yet what controllers look like
        '''
        pass

    def parse_scene(self, filename):
        '''
        '''
        self.link_index_dict = {}
        self.link_groups = {}
        self.material_indices = {}

        model = {}
        tree = self._read_scene(filename)
        root = tree.getroot()
        nodes = root.find('nodelist')
        joints = root.find('jointlist')
        sensors = root.find('sensorlist')
        motors = root.find('motorlist')
        controllers = root.find('controllerlist')
        materials = root.find('materiallist')

        self._apply_relative_ids(nodes)
        self._get_links(nodes, joints)
        self._parse_materials(materials)

        model['links'] = self._parse_links(nodes)
        model['joints'] = self._parse_joints(joints)
        model['sensors'] = self._parse_sensors(sensors)
        #model['motors'] = self._parse_motors(motors)
        #model['controllers'] = self._parse_controllers(controllers)
        model['groups'] = self.link_groups
        self._parse_additional_visuals_and_collisions(model, nodes)
        return model

# ---

def test_read_scene():
    '''
    '''
    tree = msp._read_scene('kai_car_4.scene')
    root = tree.getroot()
    print(root.tag)

def test_xml_tree_handling():
    '''
    '''
    tree = msp._read_scene('kai_car_4.scene')
    root = tree.getroot()
    for child in root:
        print(child.tag)

def test_round_float():
    '''
    '''
    print(round_float('0.000000000001'))
    print(round_float('0.23948324321'))
    print(round_float('3.892737823'))

def test_parse_scene(msp):
    '''
    '''
    #model = msp.parse_scene('kai_car_4.scene')
    model = msp.parse_scene('2014_01_13_mantis_V3.scene')
    for key in model:
        if key == 'links':
            print('links:')
            links = model[key]
            for link_key in links:
                print('\t' + link_key + ': ' + str(links[link_key]))
                print()
        else:
            print(key + ': ' + str(model[key]))
    print()
    print('link_index_dict:', msp.link_index_dict)
    print('link_groups:', msp.link_groups)
    print('material_indices:', msp.material_indices)
    print('link_indices:', msp.link_indices)
    print('vis_coll_groups:', msp.vis_coll_groups)

def test():
    '''
    '''
    msp = MARS_Scene_Parser()
    #test_read_scene(msp)
    #test_xml_tree_handling(msp)
    test_parse_scene(msp)
    #test_round_float()

if __name__ == '__main__':
    test()
