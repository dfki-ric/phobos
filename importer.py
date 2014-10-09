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

File importer.py

Created on 28 Feb 2014

@author: Kai von Szadkowski
"""

import bpy
import mathutils
import os
import yaml
from collections import namedtuple
import xml.etree.ElementTree as ET
from phobos.utility import *
from . import defs
from . import materials

#This is a really nice pythonic approach to creating a list of constants
Defaults = namedtuple('Defaults', ['mass', 'idtransform'])
defaults = Defaults(0.001, #mass
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #idtransform
                    )

def register():
    print("Registering importer...")

def unregister():
    print("Unregistering importer...")

def cleanUpScene():
    # select all objects
    bpy.ops.object.select_all(action="SELECT")

    # and delete them
    bpy.ops.object.delete()

    # after that we have to clean up all loaded meshes (unfortunately
    # this is not done automatically)
    for mesh in bpy.data.meshes:
        bpy.data.meshes.remove(mesh)

    # and all materials
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)

    # and all lights (aka lamps)
    for lamp in bpy.data.lamps:
        bpy.data.lamps.remove(lamp)

def round_float(float_as_str, decimal=6):
    '''
    Cast 'float_as_str' to float and round to 'decimal' decimal places.
    '''
    return round(float(float_as_str), decimal)
    
def pos_rot_tree_to_lists(position, rotation):
    '''
    Convert the xml representations of a position and a rotation to
    lists. If either is 'None', return a list of zeroes instead.
    '''
    if position:
        px = round_float(position.find('x').text)
        py = round_float(position.find('y').text)
        pz = round_float(position.find('z').text)
    else:
        px, py, pz = (0, 0, 0)
    if rotation:
        rw = round_float(rotation.find('w').text)
        rx = round_float(rotation.find('x').text)
        ry = round_float(rotation.find('y').text)
        rz = round_float(rotation.find('z').text)
    else:
        rw, rx, ry, rz = (0, 0, 0, 0)
        
    return [px, py, pz], [rw, rx, ry, rz]
        
def calc_pose_formats(position, rotation):
    '''
    Create a dictionary containing various representations of the pose
    represented by 'position' and 'rotation':
        - translation == position
        - rotation_quaternion == rotation
        - rotation_euler: euler angles
        - matrix: position and rotation in 4x4 matrix form
    '''
    px, py, pz = position
    if len(rotation) == 3:
        rot = mathutils.Euler(rotation).to_quaternion()
        print(rotation)
    else:
        rot = rotation
    rw, rx, ry, rz = rot
    pose_dict = {}
    
    translation = [px, py, pz]
    quaternion = mathutils.Quaternion([rw, rx, ry, rz])
    euler = quaternion.to_euler()
    print(euler)
    pose_dict['translation'] = translation
    pose_dict['rotation_quaternion'] = [rw, rx, ry, rz]
    pose_dict['rotation_euler'] = [euler.x, euler.y, euler.z]
    rm = quaternion.to_matrix()
    matrix = [[rm[0][0], rm[0][1], rm[0][2], px],
              [rm[1][0], rm[1][1], rm[1][2], py],
              [rm[2][0], rm[2][1], rm[2][2], pz],
              [0.0,      0.0,      0.0,      1.0]]
    
    pose_dict['matrix'] = matrix
    
    return pose_dict
    
def add_quaternion(rot1, rot2):
    '''
    Add two rotations in quaternion format.
    '''
    quat1 = mathutils.Quaternion(rot1)
    quat2 = mathutils.Quaternion(rot2)
    quat_sum = quat1 * quat2
    return (quat_sum.w, quat_sum.x, quat_sum.y, quat_sum.z)
    
    
def handle_missing_geometry(no_visual_geo, no_collision_geo, link_dict):
    '''
    Handle missing visual and collision geometry.
    I hope it was meant like that ...
    '''
    if no_visual_geo or no_collision_geo:
        print("\n### WARNING: Missing geometry information in", newlink['name'], ".")
    if no_visual_geo:
        print('Affected visual elements are:', no_visual_geo, '.\n\
                Trying to get missing information from collision elements.')
        for visual in no_visual_geo:
            try:
                link_dict['visual'][visual]['geometry'] = link_dict['collision']['collision' + geo[len('visual'):]]['geometry']
            except:
                pass # TODO: print something?
    elif no_collision_geo:
        print('Affected collision elements are:', no_visual_geo, '.\n\
                Trying to get missing information from visual elements.')
        for collision in no_collision_geo:
            try:
                link_dict['collision'][collision]['geometry'] = link_dict['visual']['visual' + geo[len('collision'):]]['geometry']
            except:
                pass # TODO: print something?
    

class RobotModelParser():
    """Base class for a robot model file parser of a specific type"""

    def __init__(self, filepath):
        self.filepath = filepath
        self.path, self.filename = os.path.split(self.filepath)
        self.robot = {}

    def placeChildLinks(self, parent):
        print(parent['name']+ ', ', end='')
        children = []
        for l in self.robot['links']:
            if 'parent' in self.robot['links'][l] and self.robot['links'][l]['parent'] == parent['name']:
                children.append(self.robot['links'][l])
        for child in children:
            # 1: set parent relationship (this makes the parent inverse the inverse of the parents world transform)
            parentLink = bpy.data.objects[parent['name']]
            childLink = bpy.data.objects[child['name']]
            bpy.ops.object.select_all(action="DESELECT") #bpy.context.selected_objects = []
            childLink.select = True
            parentLink.select = True
            bpy.context.scene.objects.active = parentLink
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            # 2: move to parents origin by setting the world matrix to the parents world matrix
            childLink.matrix_world = parentLink.matrix_world        # removing this line does not seem to make a difference
            # 3: apply local transform as saved in urdf (change matrix_local from identity to urdf)
            location = mathutils.Matrix.Translation(child['pose']['translation'])
            rotation = mathutils.Euler(tuple(child['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
            transform_matrix = location * rotation
            childLink.matrix_local = transform_matrix
            # 4: be happy, as world and basis are now the same and local is the transform to be exported to urdf
            # 5: take care of the rest of the tree
            self.placeChildLinks(child)

    def placeLinkSubelements(self, link):
        #urdf_sca = #TODO: solve problem with scale
        # 3.2: make sure to take into account visual information #TODO: also take into account inertial and joint axis (for joint sphere) and collision (bounding box)
        #* urdf_visual_loc * urdf_visual_rot #*urdf_sca
        parentLink = bpy.data.objects[link['name']]
        if 'inertial' in link:
            if 'pose' in link['inertial']:
                urdf_geom_loc = mathutils.Matrix.Translation(link['inertial']['pose']['translation'])
                urdf_geom_rot = mathutils.Euler(tuple(link['inertial']['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
            else:
                urdf_geom_loc = mathutils.Matrix.Identity(4)
                urdf_geom_rot = mathutils.Matrix.Identity(4)
            print(link['name'], link['inertial'])
            print(link['inertial']['name'])
            geoname = link['inertial']['name'] #g
            geom = bpy.data.objects[geoname]
            bpy.ops.object.select_all(action="DESELECT")
            geom.select = True
            parentLink.select = True
            bpy.context.scene.objects.active = parentLink
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            #geom.matrix_world = parentLink.matrix_world #FIXME: this applies the scale of the parent, making boxes BIIIG
            geom.matrix_local = urdf_geom_loc * urdf_geom_rot
        for geomsrc in ['visual', 'collision']:
            if geomsrc in link:
                for g in link[geomsrc]:
                    geom = link[geomsrc][g]
                    print([key for key in geom])
                    if 'pose' in geom:
                        urdf_geom_loc = mathutils.Matrix.Translation(geom['pose']['translation'])
                        urdf_geom_rot = mathutils.Euler(tuple(geom['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
                    else:
                        urdf_geom_loc = mathutils.Matrix.Identity(4)
                        urdf_geom_rot = mathutils.Matrix.Identity(4)
                    geoname = geom['name'] #g
                    geom = bpy.data.objects[geoname]
                    bpy.ops.object.select_all(action="DESELECT")
                    geom.select = True
                    parentLink.select = True
                    bpy.context.scene.objects.active = parentLink
                    bpy.ops.object.parent_set(type='BONE_RELATIVE')
                    geom.matrix_world = parentLink.matrix_world
                    geom.matrix_local = urdf_geom_loc * urdf_geom_rot

    def createGeometry(self, viscol, geomsrc):
        newgeom = None
        if viscol['geometry'] is not {}:
            bpy.ops.object.select_all(action='DESELECT')
            geom = viscol['geometry']
            geomtype = geom['geometryType']
            # create the Blender object
            # tag all objects
            for obj in bpy.data.objects:
                obj.tag = True
            if geomtype == 'mesh':
                filetype = geom['filename'].split('.')[-1]
                if filetype == 'obj' or filetype == 'OBJ':
                    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, geom['filename']))
                elif filetype == 'stl' or filetype == 'STL':
                    bpy.ops.import_mesh.stl(filepath=os.path.join(self.path, geom['filename']))
                # hack for test:
                elif filetype == 'bobj' or filetype == 'BOBJ':
                    filename = 'visual_' + geom['filename'].split('.')[0] + '.obj'
                    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, filename))
                else:
                    print('ERROR: Could not import object.')
                # find the newly imported obj
                for obj in bpy.data.objects:
                    if not obj.tag:
                        newgeom = obj
                        #with obj file import, blender only turns the object, not the vertices,
                        #leaving a rotation in the matrix_basis, which we here get rid of
                        if filetype == 'obj':
                            bpy.ops.object.select_all(action='DESELECT')
                            newgeom.select = True
                            bpy.ops.object.transform_apply(rotation=True)
                newgeom.name = viscol['name']
                #newgeom.layers = defLayers([defs.layerTypes[geomsrc]])
            elif geomtype == 'box':
                newgeom = createPrimitive(viscol['name'],
                                          geomtype,
                                          geom['size'],
                                          defs.layerTypes[geomsrc]
                                          )
            elif geomtype == 'cylinder':
                newgeom = createPrimitive(viscol['name'],
                                          geomtype,
                                          (geom['radius'], geom['length']),
                                          defs.layerTypes[geomsrc]
                                          )
            elif geomtype == 'sphere':
                newgeom = createPrimitive(viscol['name'],
                                          geomtype,
                                          geom['radius'], #tuple would cause problem here
                                          defs.layerTypes[geomsrc]
                                          )
            else:
                print("### ERROR: Could not determine geometry type of " + geomsrc + viscol['name'] + '. Placing empty coordinate system.')
            if newgeom is not None:
                newgeom.MARStype = geomsrc
                newgeom.select = True
                if 'scale' in geom:
                    newgeom.scale = geom['scale']
                bpy.ops.object.transform_apply(scale=True)
                newgeom['geometryType'] = geomtype
                #TODO: which other properties remain?
            #FIXME: place empty coordinate system and return...what?
        return newgeom

    def createInertial(self, name, inertial):
        bpy.ops.object.select_all(action='DESELECT')
        inert = createPrimitive('inertial_'+name, 'sphere', 0.01, 0, 'None', (0, 0, 0))
        inert.select = True
        bpy.ops.object.transform_apply(scale=True)
        #obj = bpy.context.object
        #obj.name = name
        for prop in inertial:
            if prop not in ['pose'] and inertial[prop] is not None:
                inert[prop] = inertial[prop]
        inert.MARStype = 'inertial'
        return inert

    def createLink(self, link):
        print("Creating link", link['name'])
        #create base object ( =armature)
        bpy.ops.object.select_all(action='DESELECT')
        #bpy.ops.view3d.snap_cursor_to_center()
        bpy.ops.object.armature_add(layers=defLayers([0]))
        newlink = bpy.context.active_object #print(bpy.context.object) #print(bpy.context.scene.objects.active) #bpy.context.selected_objects[0]
        newlink.name = link['name']
        newlink.location = (0.0, 0.0, 0.0)
        newlink.scale = (0.3, 0.3, 0.3) #TODO: make this depend on the largest visual or collision object
        bpy.ops.object.transform_apply(scale=True)
        newlink.MARStype = 'link'
        if newlink.name != link['name']:
            print("Warning, name conflict!")
        # place inertial
        if 'inertial' in link:
            self.createInertial(link['name'], link['inertial'])
        # place visual
        if 'visual' in link:
            for v in link['visual']:
                visual = link['visual'][v]
                if 'geometry' in visual:
                    self.createGeometry(visual, 'visual')
        # place collision
        if 'collision' in link:
            for c in link['collision']:
                collision = link['collision'][c]
                if 'geometry' in collision:
                    self.createGeometry(collision, 'collision')
        return newlink

    def createBlenderModel(self): #TODO: solve problem with duplicated links (linklist...namespaced via robotname?)
        """Creates the blender object representation of the imported model."""
        print("\n\nCreating Blender model...")
        for l in self.robot['links']:
            #print(l + ', ', end='')
            link = self.robot['links'][l]
            #print(link['name'])
            self.createLink(link)

        #build tree recursively and correct translation & rotation on the fly
        for l in self.robot['links']:
            if not 'parent' in self.robot['links'][l]:
                root = self.robot['links'][l]
        print("\n\nPlacing links...")
        self.placeChildLinks(root)
        for link in self.robot['links']:
            self.placeLinkSubelements(self.robot['links'][link])
        print('Done!')

        for obj in bpy.data.objects:
            print('name:', obj.name)
            matrix = obj.matrix_local
            loc, rot, scale = matrix.decompose()
            print('rotation:')
            print('\tx:', rot.x, '\n\ty:', rot.y, '\n\tz:', rot.z, '\n\tw:', rot.w)
            print('location:')
            print('\tx:', loc.x, '\n\ty:', loc.y, '\n\tz:', loc.z)
            print('-------------------')
            
        
    def _debug_output(self):
        '''
        Write the robot dictionary to a yaml file in the source file's directory
        '''
        with open(self.filepath + '_ref_debug.yml', 'w') as outputfile:
            outputfile.write(yaml.dump(self.robot)) #last parameter prevents inline formatting for lists and dictionaries


class MARSModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a MARS scene"""

    def __init__(self, filepath):
        '''
        Initialise a 'MARSModelParser' object and a number of containers
        used to keep track of the parsed information and to apply it in
        the right places.
        
        Explanation of some of the containers:
            - self.link_index_dict:
                {link index: link name}
            - self.link_indices:
                set of all indices of xml nodes that are links
            - self.material_indices:
                {material index: dictionary containing material information}
            - self.applied_rel_id_poses:
                {link id: dictionary containing pose with parent pose
                          applied}
            - self.name_counter_dict:
                {link/visual/collision/material name:
                 amount of how often the name already has been given to
                 something (plus index for distinction)}
        '''
        RobotModelParser.__init__(self, filepath)
        
        self.xml_tree = None
        self.link_index_dict = {}
        self.link_groups_group_order = {}
        self.link_groups_link_order = {}
        self.link_indices = set([])
        self.material_indices = {}
        self.vis_coll_groups = {}
        self.applied_rel_id_poses = {}
        self.missing_vis_geos = {}
        self.missing_coll_geos = {}
        self.name_counter_dict = {}
        self.link_group_dict = {}
        self.robot_states = {}

    def parseModel(self):
        '''
        '''
        print("\nParsing MARS scene from", self.filepath)
        self.tree = ET.parse(self.filepath)
        root = self.tree.getroot()
        
        nodes = root.find('nodelist')
        joints = root.find('jointlist')
        sensors = root.find('sensorlist')
        motors = root.find('motorlist')
        controllers = root.find('controllerlist')
        materials = root.find('materiallist')
        
        self._get_links(nodes, joints)
        self._apply_relative_ids(nodes)

        self._parse_materials(materials)
        
        links = self._parse_links(nodes)
        #self._add_parent_links(links)
        self.robot['links'] = links
        self.robot['joints'] = self._parse_joints(joints)
        self.robot['sensors'] = self._parse_sensors(sensors)
        self.robot['motors'] = self._parse_motors(motors)
        self.robot['controllers'] = self._parse_controllers(controllers)
        self.robot['groups'] = self.link_groups_group_order
        self._parse_additional_visuals_and_collisions(self.robot, nodes)
        
        for link in self.robot['links']:
            handle_missing_geometry(self.missing_vis_geos[link], self.missing_coll_geos[link], self.robot['links'][link])

        self.robot['states'] = self.robot_states

        self._debug_output()
        
        #print(self.link_groups_link_order)
        #print()
        #print(self.vis_coll_groups)
        #print()
        #print(self.link_index_dict)
        #print()
        #print(self.link_indices)
        
       # assert False
    
    def _get_links(self, nodes, joints):
        '''
        Collect the indices of all nodes that are links inside
        'self.link_indices'.
        '''
        for joint in joints:
            child = int(joint.find('nodeindex2').text)
            self.link_indices.update([child])
        for node in nodes:
            if node.find('relativeid') is None:
                self.link_indices.update([int(node.find('index').text)])
            
        
    def _parse_materials(self, materials_tree):
        '''
        Parse the materials from the MARS scene.
        
        TODO: change 'materials' so that superfluous 'color' entry is not needed
        '''
        material_list = []
        for material in materials_tree:
            material_dict = {}
            mat_id = int(material.find('id').text)
            
            # check needs to be explicit for future versions
            if material.find('name') is not None:
                name = self._get_distinct_name(material.find('name').text)
            else:
                name = 'material_' + str(mat_id)
            material_dict['name'] = name
            
            for xml_colour in defs.MARSrevlegdict:
                colour = material.find(xml_colour)
                if colour is not None:
                    r = round_float(colour.find('r').text)
                    g = round_float(colour.find('g').text)
                    b = round_float(colour.find('b').text)
                    a = round_float(colour.find('a').text)
                    py_colour = defs.MARSrevlegdict[xml_colour]
                    material_dict[py_colour] = [r, g, b, a]
                    # for now:
                    material_dict['color'] = [r, g, b, a]
            
            
            transparency = material.find('transparency')
            if transparency is not None:
                material_dict['transparency'] = round_float(transparency.text)
            else:
                material_dict['transparency'] = 0.0
                
            material_dict['shininess'] = round_float(material.find('shininess').text)
            self.material_indices[mat_id] = material_dict
            material_list.append(material_dict)
            
        for m in material_list:
            materials.makeMaterial(m['name'], tuple(m['color'][0:3]), (1, 1, 1), m['color'][-1])
    
    def _parse_geometry(self, vis_coll_dict, node, mode):
        '''
        Parse the geometry of a visual or collision object.
        '''
        size = None
        if mode == 'visual':
            size = node.find('visualsize')
            mesh_file = node.find('filename')
            if mesh_file is not None:
                geometry_type = 'mesh'
            else:
                geometry_type = node.find('physicmode').text
        elif mode == 'collision':
            size = node.find('extend')
            geometry_type = node.find('physicmode').text
        
        geometry_dict = {}
        geometry_dict['geometryType'] = geometry_type
        
        if geometry_type == 'box' or geometry_type == 'mesh':
            x = round_float(size.find('x').text)
            y = round_float(size.find('y').text)
            z = round_float(size.find('z').text)
            geometry_dict['size'] = [x, y, z]
            if geometry_type == 'mesh':
                filename = node.find('filename').text
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
        
        if geometry_dict == {}:
            return False
        else:
            vis_coll_dict['geometry'] = geometry_dict
            return True
    
    def _parse_visual(self, visuals_dict, node, missing_vis_geo):
        '''
        Parse a visual object.
        '''
        visual_dict = {}
        name = self._get_distinct_name('visual_' + node.get('name'))
        visual_dict['name'] = name
        index = int(node.find('index').text)
        
        visual_dict['pose'] = self.applied_rel_id_poses[index]
        
        mat_index = int(node.find('material_id').text)
        visual_dict['material'] = self.material_indices[mat_index]
        
        if not self._parse_geometry(visual_dict, node, 'visual'):
            missing_vis_geo.append(name)
        
        visuals_dict[name] = visual_dict
        
    def _parse_collision(self, collisions_dict, node, missing_coll_geo):
        '''
        Parse a collision object.
        '''
        collision_dict = {}
        name = self._get_distinct_name('collision_' + node.get('name'))
        collision_dict['name'] = name
        index = int(node.find('index').text)        
        
        collision_dict['pose'] = self.applied_rel_id_poses[index]
        
        bitmask = int(float(node.find('coll_bitmask').text))
        collision_dict['bitmask'] = bitmask
        
        if not self._parse_geometry(collision_dict, node, 'collision'):
            missing_coll_geo.append(name)
        
        max_contacts = node.find('cmax_num_contacts')
        if max_contacts is not None:
            collision_dict['max_contacts'] = int(max_contacts.text)
        
        collisions_dict[name] = collision_dict
        
    def _parse_inertial(self, link_dict, node):
        '''
        Parse an inertial.
        
        Note: Pose in inertia does not seem to be available.
        '''
        inertial_dict = {}
        
        mass = node.find('mass')
        if mass is not None:
            inertial_dict['mass'] = round_float(mass.text)
        
        inertia = node.find('inertia')
        if inertia is not None and bool(inertia.text):  # 'inertia' states whether the defined inertia values are to be used
            i00 = round_float(node.find('i00').text)
            i01 = round_float(node.find('i01').text)
            i02 = round_float(node.find('i02').text)
            i11 = round_float(node.find('i11').text)
            i12 = round_float(node.find('i12').text)
            i22 = round_float(node.find('i22').text)
            inertial_dict['inertia'] = [i00, i01, i02,
                                             i11, i12,
                                                  i22]
                                
        if inertial_dict is not {}:
            # inertial pose is always origin for now
            position, rotation = pos_rot_tree_to_lists(None, None)
            inertial_dict['pose'] = calc_pose_formats(position, rotation)
            inertial_dict['name'] = self._get_distinct_name('inertial_' + node.get('name'))
            link_dict['inertial'] = inertial_dict
     
    def _get_distinct_name(self, name):
        '''
        Create a name that has not been used yet in this model.
        '''
        if name in self.name_counter_dict:
            name_counter = self.name_counter_dict[name]
            distinct_name = name + '_' + str(name_counter)
            self.name_counter_dict[name] = name_counter + 1
        else:
            distinct_name = name
            self.name_counter_dict[name] = 1
        return distinct_name
        
    def _parse_links(self, nodes):
        '''
        Parse the links of the MARS scene.
        '''
        links_dict = {}
        for node in nodes:
            missing_vis_geo = []
            missing_coll_geo = []
            index = int(node.find('index').text)
            name = self._get_distinct_name(node.get('name'))
                
            self.link_index_dict[index] = name
            
            group = int(node.find('groupid').text)
            node_group_dict = {'name': name,
                               'index': index}
            if group in self.link_groups_group_order:
                self.link_groups_group_order[group].append(node_group_dict)
            else:
                self.link_groups_group_order[group] = [node_group_dict]
            
            rel_id = node.find('relativeid')
            if index in self.link_indices:
                link_dict = {}
                
                link_dict['name'] = name
                
                pose = self.applied_rel_id_poses[index]
                link_dict['pose'] = pose
                
                visuals_dict = {}
                #self._parse_visual(visuals_dict, node, missing_vis_geo)
                link_dict['visual'] = visuals_dict
                
                collisions_dict = {}
                #self._parse_collision(collisions_dict, node, missing_coll_geo)
                link_dict['collision'] = collisions_dict
                
                inertial_dict = self._parse_inertial(link_dict, node)
                
                if rel_id is not None:
                    link_dict['parent'] = int(rel_id.text)
                
                links_dict[name] = link_dict
                self.missing_vis_geos[name] = missing_vis_geo
                self.missing_coll_geos[name] = missing_coll_geo
        
        for group_index in self.link_groups_group_order:
            group = self.link_groups_group_order[group_index]
            for node in group:
                if node['index'] in self.link_indices:
                    link_index = node['index']
                    self.link_groups_link_order[link_index] = group
                    break
                    
        for link_index in self.link_groups_link_order:
            group = self.link_groups_link_order[link_index]
            for node in group:
                self.link_group_dict[node['index']] = link_index
        
        for link_name in links_dict:
            link_dict = links_dict[link_name]
            if 'parent' in link_dict:
                link_dict['parent'] = self.link_index_dict[self.link_group_dict[link_dict['parent']]]
        
        return links_dict
        
    def _add_parent_links(self, links):
        '''
        Replace parent indices (not actually parent indices!) with parent names inside the link
        dictionaries.
        
        TODO: There is a better solution, surely.
        '''
        for link_name in links:
            link = links[link_name]
            if 'parent' in link:
                rel_id = link['parent']
                if rel_id in self.vis_coll_groups:
                    parent_id = self.vis_coll_groups[rel_id]
                else:
                    parent_id = rel_id
                link['parent'] = self.link_index_dict[parent_id]
                
                #else:
                #    print('--------------')
                #    print('| WTF-ERROR! |')
                #    print('--------------')
        
    def _parse_additional_visuals_and_collisions(self, model, nodes):
        '''
        Parse nodes that are no links as additional visual and collision
        objects for the already parsed links.
        '''
        for link in self.link_groups_link_order:
            for node in self.link_groups_link_order[link]:
                self.vis_coll_groups[node['index']] = link
        
        for node in nodes:
            index = int(node.find('index').text)
            if index in self.vis_coll_groups:
                link_index = self.vis_coll_groups[index]
                group = self.link_groups_link_order[link_index]
                for group_node in group:
                    if group_node['index'] == link_index:
                        visuals_dict = model['links'][group_node['name']]['visual']
                        self._parse_visual(visuals_dict, node, self.missing_vis_geos[group_node['name']])
                        model['links'][group_node['name']]['visual'] = visuals_dict
                        
                        collisions_dict = model['links'][group_node['name']]['collision']
                        self._parse_collision(collisions_dict, node, self.missing_coll_geos[group_node['name']])
                        model['links'][group_node['name']]['collision'] = collisions_dict
                        
                        break
        
    def _parse_joints(self, joints):
        '''
        Parse the joints of the MARS scene.
        '''
        joints_dict = {}
        state_dict = {}
        for joint in joints:
            joint_dict = {}
            name = joint.get('name')
            joint_dict['name'] = name
            joint_dict['type'] = joint.find('type').text
            
            parent_index = int(joint.find('nodeindex1').text)
            joint_dict['parent'] = self.link_index_dict[parent_index]
            child_index = int(joint.find('nodeindex2').text)
            joint_dict['child'] = self.link_index_dict[child_index]
            
            joints_dict[name] = joint_dict

            xml_offset = joint.find('angle1_offset')
            if xml_offset is not None:
                state_dict[name] = float(xml_offset.text)
            else:
                state_dict[name] = 0.0

        self.robot_states['start'] = state_dict
        return joints_dict


    def _apply_relative_ids(self, nodes):
        absolute_poses = {}
        link_poses = {}
        relative_poses = {}
        for node in nodes:
            index = int(node.find('index').text)
            xml_position = node.find('position')
            xml_rotation = node.find('rotation')
            position, rotation = pos_rot_tree_to_lists(xml_position, xml_rotation)
            pose = calc_pose_formats(position, rotation)

            rel_id_xml = node.find('relativeid')
            #if index in self.link_indices:
            #    link_poses[index] = pose
            if rel_id_xml is None:
                absolute_poses[index] = pose
                root = index
            else:
                rel_id = int(rel_id_xml.text)
                relative_poses[index] = {'pose': pose, 'rel_id': rel_id}

            num_rel_poses = -1
        while relative_poses:
            to_delete = []
            # check for infinite loop (happens if referenced node does not exist):
            if len(relative_poses) == num_rel_poses:
                print('Error: non-existant relative id')
                break
            num_rel_poses = len(relative_poses)
            for index in relative_poses:
                #print('link_poses:', link_poses)
                relative_pose = relative_poses[index]
                rel_id = relative_pose['rel_id']
                if rel_id in link_poses or rel_id in absolute_poses:
                    #print('index:', index)
                    #print('rel_id:', rel_id)
                    if rel_id in self.link_indices and rel_id is not root:
                        absolute_poses[index] = relative_pose['pose']
                    else:
                        reference_pose = absolute_poses[rel_id]
                        rel_matrix = mathutils.Matrix(relative_pose['pose']['matrix']).to_4x4()
                        ref_matrix = mathutils.Matrix(reference_pose['matrix']).to_4x4()
                        applied_matrix = ref_matrix * rel_matrix # or the other way round?
                        #print('relative:\n', rel_matrix)
                        #print(rel_matrix.to_quaternion())
                        #print('reference:\n', ref_matrix)
                        #print(ref_matrix.to_quaternion())
                        #print('applied:\n', applied_matrix)
                        #print(applied_matrix.to_quaternion())
                        loc, rot, scale = applied_matrix.decompose()
                        applied_pos = [loc.x, loc.y, loc.z]
                        applied_rot = [rot.w, rot.x, rot.y, rot.z]
                        absolute_poses[index] = calc_pose_formats(applied_pos, applied_rot)
                    to_delete.append(index)
            for index in to_delete:
                del relative_poses[index]
        self.applied_rel_id_poses = absolute_poses



    
    def _parse_sensors(self, sensors):
        '''
        'link' is missing in manual
        '''
        sensors_dict = {}
        for sensor in sensors:
            sensor_dict = {}
            name = sensor.get('name')
            sensor_dict['name'] = name
            xml_link = sensor.find('attached_node')
            if xml_link is not None:
                 sensor_dict['link'] = self.link_index_dict[int(xml_link.text)]
            sensor_dict['type'] = sensor.get('type')
            sensors_dict[name] = sensor_dict
        return sensors_dict
    
    def _parse_motors(self, motors):
        '''
        don't know yet what motors look like
        '''
        motors_dict = {}
        for motor in motors:
            motor_dict = {}
            name = motor.get('name')

            motors_dict[name] = motor_dict

        return motors_dict
    
    def _parse_controllers(self, controllers):
        '''
        don't know yet what controllers look like
        '''
        controllers_dict = {}
        for controller in controllers:
            controller_dict = {}
            name = controller.get('name')

            controllers_dict[name] = controller_dict

        return controllers_dict


class URDFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a URDF model"""

    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)

    def parseModel(self):
        print("\nParsing URDF model from", self.filepath)
        self.tree = ET.parse(self.filepath)
        self.root = self.tree.getroot()#[0]
        self.robot["name"] = self.root.attrib["name"]
        if 'version' in self.root.attrib:
            self.robot["version"] = self.root.attrib['version'] #TODO: implement version functionality (time code)

        #write links to dictionary
        links = {}
        print("\n\nParsing links..")
        for link in self.root.iter('link'):
            newlink = self.parseLink(link)
            #write link to list
            links[newlink['name']] = newlink
            #print(newlink)
        self.robot['links'] = links

        #write joints to dictionary
        joints = {}
        print("\n\nParsing joints..")
        for joint in self.root.iter('joint'):
            if joint.find('parent') is not None: #this is needed as there are "joint" tags e.g. in transmission
                newjoint, pose = self.parseJoint(joint)
                self.robot['links'][newjoint['child']]['pose'] = pose
                joints[newjoint['name']] = newjoint
                #print(newjoint)
        self.robot['joints'] = joints

        #find any links that still have no pose (most likely because they had no parent)
        for link in links:
            if not 'pose' in links[link]:
                position, rotation = pos_rot_tree_to_lists(None, None)
                links[link]['pose'] = calc_pose_formats(position, rotation)
            #print(link, links[link]['pose'])

        #write parent-child information to nodes
        print("\n\nWriting parent-child information to nodes..")
        for j in self.robot['joints']:
            joint = self.robot['joints'][j]
            self.robot['links'][joint['child']]['parent'] = joint['parent']
            #print(joint['parent'] + ', ', end='')
            
        self.parseMaterials()

        self._debug_output()

    def parseLink(self, link):
        #print(link.attrib['name'] + ', ', end='')
        newlink = {a: link.attrib[a] for a in link.attrib}

        self.parseInertial(newlink, link)
        no_visual_geo = self.parseVisual(newlink, link)
        no_collision_geo = self.parseCollision(newlink, link)
        handle_missing_geometry(no_visual_geo, no_collision_geo, newlink)
        
        if newlink == {}:
            print("\n### WARNING:", newlink['name'], "is empty.")
        return newlink
        
    def parseInertial(self, link_dict, link_xml):
        '''
        '''
        inertial = link_xml.find('inertial')
        if inertial is not None: # !!! 'if Element' yields none if the Element contains no children, thus this notation !!!
            inertial_dict = {}
            origin = inertial.find('origin')
            if origin is not None:
                raw_pose = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
                inertial_dict['pose'] = calc_pose_formats(raw_pose[:3], raw_pose[3:])
            else:
                position, rotation = pos_rot_tree_to_lists(None, None)
                inertial_dict['pose'] = calc_pose_formats(position, rotation)
            mass = inertial.find('mass')
            if mass is not None:
                inertial_dict['mass'] = float(mass.attrib['value'])
            inertia = inertial.find('inertia')
            if inertia is not None:
                values = []
                inertial_dict['inertia'] = values.append(inertia.attrib[a] for a in inertia.attrib)
            inertial_dict['name'] = 'inertial_' + link_dict['name']
            
            link_dict['inertial'] = inertial_dict
            
    def parseVisual(self, link_dict, link_xml):
        '''
        '''
        visual_dict = {}
        no_vis_geo = []
        i=0
        for visual_xml in link_xml.iter('visual'):
            try:
                visname = visual_xml.attrib['name']
            except KeyError:
                visname = 'visual_' + str(i) + '_' + link_dict['name']
                i += 1
            visual_dict[visname] = {a: visual_xml.attrib[a] for a in visual_xml.attrib}
            vis_dict = visual_dict[visname]
            vis_dict['name'] = visname
            origin = visual_xml.find('origin')
            if origin is not None:
                 raw_pose = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
                 vis_dict['pose'] = calc_pose_formats(raw_pose[:3], raw_pose[3:])
            else:
                vis_dict['pose'] = calc_pose_formats(None, None)
                
            if not self.parseGeometry(vis_dict, visual_xml):
                no_vis_geo.append(visname)
            
            material = visual_xml.find('material')
            if material is not None:
                vis_dict['material'] = {'name': material.attrib['name']}
                color = material.find('color')
                if color is not None:
                    vis_dict['material']['color'] = parse_text(color.attrib['rgba'])
            else:
                vis_dict['material'] = {'name':'None'} #TODO: this is a hack!
                print("\n### Warning: No material provided for link", link_dict['name'])
                
        if visual_dict == {}: #'else' cannot be used as we don't use a break
            print("\n### WARNING: No visual information provided for link", link_dict['name'])
        else:
            link_dict['visual'] = visual_dict
        return no_vis_geo
        
    def parseCollision(self, link_dict, link_xml):
        '''
        '''
        collision_dict = {}
        no_coll_geo = []
        i=0
        for collision_xml in link_xml.iter('collision'):
            try:
                colname = collision_xml.attrib['name']
            except KeyError:
                colname = 'collision_' + str(i) + '_' + link_dict['name']
                i += 1
            collision_dict[colname] = {a: collision_xml.attrib[a] for a in collision_xml.attrib}
            col_dict = collision_dict[colname]
            col_dict['name'] = colname
            origin = collision_xml.find('origin')
            if origin is not None:
                raw_pose = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
                col_dict['pose'] = calc_pose_formats(raw_pose[:3], raw_pose[3:])
            else:
                col_dict['pose'] = calc_pose_formats(None, None)

            if not self.parseGeometry(col_dict, collision_xml):
                no_coll_geo.append(colname)
            
            if collision_dict == {}:
               print("\n### WARNING: No collision information provided for link", link_dict['name'])
            else:
                link_dict['collision'] = collision_dict
        return no_coll_geo
        
    def parseGeometry(self, vis_coll_dict, vis_coll_xml):
        '''
        '''
        geometry_xml = vis_coll_xml.find('geometry')
        if geometry_xml is not None:
            got_geometry = True
            geometry_dict = {a: parse_text(geometry_xml[0].attrib[a]) for a in geometry_xml[0].attrib}
            geometry_dict['geometryType'] = geometry_xml[0].tag
            if geometry_xml[0].tag == 'mesh':
                geometry_dict['filename'] = geometry_xml[0].attrib['filename'] #TODO: remove this, also from export, as it is double
            vis_coll_dict['geometry'] = geometry_dict
        else:
            no_geometry = False
        return got_geometry
        

    def parseJoint(self, joint):
        #print(joint.attrib['name']+', ', end='')
        newjoint = {a: joint.attrib[a] for a in joint.attrib}
        try:
            origin = joint.find('origin')
            origindict = {'xyz': origin.attrib['xyz'].split(), 'rpy': origin.attrib['rpy'].split()}
        except AttributeError:
            origindict = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
        newjoint['parent'] = joint.find('parent').attrib['link']
        newjoint['child'] = joint.find('child').attrib['link']
        raw_pose = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
        pose = calc_pose_formats(raw_pose[:3], raw_pose[3:])
        #axis
        #calibration
        #dynamics
        #limit
        #mimic
        #safety_controller
        return newjoint, pose
        
    def parseMaterials(self):
        '''
        '''
        material_list = [] #TODO: build dictionary entry for materials
        print("\n\nParsing materials..")
        for material in self.root.iter('material'):
            newmaterial = {a: material.attrib[a] for a in material.attrib}
            color = material.find('color')
            if color is not None:
                newmaterial['color'] = parse_text(color.attrib['rgba'])
                material_list.append(newmaterial)
        for m in material_list:
            #TODO: handle duplicate names? urdf_robotname_xxx?
            materials.makeMaterial(m['name'], tuple(m['color'][0:3]), (1, 1, 1), m['color'][-1]) 

class SMURFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a SMURF model"""

    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)

    def parseModel(self):
        print("Parsing SMURF model...")
        stream = open(self.filepath, 'r')
        smurf_spec = yaml.load(stream)
        smurf_files = smurf_spec['files']
        for smurf_file in smurf_files:
            if smurf_file.split('.')[-1] == 'urdf':
                urdf_parser = URDFModelParser(os.path.dirname(self.filepath) + '/' + smurf_file)
                urdf_parser.parseModel()
                self.robot = urdf_parser.robot
            elif smurf_file.split('.')[-1] in ['yml', 'yaml']:
                pass
                # TODO


class RobotModelImporter(bpy.types.Operator):
    """Importer for MARS-compatible model or scene files"""
    bl_idname = "obj.import_robot_model"
    bl_label = "Import robot model file from various formats"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'

    # creating property for storing the path to the .scn file
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    # set a filter to only consider .scn files (only used internally)
    #filter_glob = bpy.props.StringProperty(default="*.*",options={'HIDDEN'})

    @classmethod
    def poll(cls, context):
        return context is not None

    def execute(self, context):
        # get the chosen file path
        #directory, filename = os.path.split(self.filepath)
        modeltype = self.filepath.split('.')[-1]

        if modeltype == 'scene':
            importer = MARSModelParser(self.filepath)
        elif modeltype == 'urdf':
            importer = URDFModelParser(self.filepath)
        elif modeltype == 'smurf' or modeltype == 'yml' or modeltype == 'yaml':
            importer = SMURFModelParser(self.filepath)
        else:
            print("Unknown model format, aborting import...")

        cleanUpScene()
        importer.parseModel()
        importer.createBlenderModel()

        return {'FINISHED'}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}

# Register and add to the file selector
bpy.utils.register_class(RobotModelImporter)


#def apply_rel_ids_old(self, nodes):
#    '''
#    Collect the absolute poses for all nodes.

#   Does not work like that since it calculates poses relative to root,
#   but may become useful again.
#    '''
    #base_poses = {}
    #rel_poses = {}
    #for node in nodes:
    #    index = int(node.find('index').text)
    #    xml_position = node.find('position')
    #    xml_rotation = node.find('rotation')
    #    position, rotation = pos_rot_tree_to_lists(xml_position, xml_rotation)
    #    pose = calc_pose_formats(position, rotation)
    #
    #    if node.find('relativeid') is not None:
    #        rel_pose_dict = {'pose': pose,
    #                         'rel_id': int(node.find('relativeid').text)}
    #        rel_poses[index] = rel_pose_dict
    #    else:
    #        base_pose_dict = {'pose': pose}
    #        base_poses[index] = base_pose_dict
    #num_rel_poses = -1
    #while rel_poses:
    #    to_delete = []
    #    # check for infinite loop (happens if referenced node does not exist):
    #    if len(rel_poses) == num_rel_poses:
    #        print('Error: non-existant relative id')
    #        break
    #    num_rel_poses = len(rel_poses)
    #    for rel_index in rel_poses:
    #        rel_pose = rel_poses[rel_index]
    #        base = rel_pose['rel_id']
    #        if base in base_poses:
    #            base_pose = base_poses[base]['pose']
    #            rel = rel_pose['pose']
    #            base_matrix = mathutils.Matrix(base_pose['matrix']).to_4x4()
    #            relative_matrix = mathutils.Matrix(rel['matrix']).to_4x4()
    #            applied_matrix = base_matrix * relative_matrix
    #            loc, rot, scale = applied_matrix.decompose()
    #            applied_pos = [loc.x, loc.y, loc.z]
    #            applied_rot = [rot.w, rot.x, rot.y, rot.z]
    #            base_poses[rel_index] = {'pose': calc_pose_formats(applied_pos, applied_rot)}
    #            to_delete.append(rel_index)
    #    for index in to_delete:
    #        del rel_poses[index]
    #self.applied_rel_id_poses = base_poses


def main():
    # call the newly registered operator
    cleanUpScene()
    bpy.ops.import_robot_model('INVOKE_DEFAULT')

if __name__ == '__main__':
    main()
