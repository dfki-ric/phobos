#!/usr/bin/python

"""
.. module:: phobos.exporter
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadowski

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
from . import joints
from . import sensors
from . import controllers
from phobos.logging import *

#This is a really nice pythonic approach to creating a list of constants
Defaults = namedtuple('Defaults', ['mass', 'idtransform'])
defaults = Defaults(0.001,  # mass
                    [0.0, 0.0, 0.0]  # idtransform
                    )


def register():
    """
    This function registers this module.
    At the moment it does nothing.

    :return: Nothing

    """
    print("Registering importer...")


def unregister():
    """
    This function unregisters this module.
    At the moment it does nothing.

    :return: Nothing

    """

    print("Unregistering importer...")


def cleanUpScene():
    """This function cleans up the scene
    and removes all blender objects, meshes, materials and lights.

    :return: Nothing.

    """
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


class RobotModelParser():
    """Base class for a robot model file parser of a specific type

    """

    def __init__(self, filepath):
        """This init saves the filepath splitted into path and filename and creates an initial empty robot dictionary.

        :param filepath: The filepath you want to export the robot to *WITH FILENAME*
        :type filepath: String.
        :return: Nothing.

        """
        self.filepath = filepath
        self.path, self.filename = os.path.split(self.filepath)
        self.robot = {'links': {},
                      'joints': {},
                      'sensors': {},
                      'motors': {},
                      'controllers': {},
                      'materials': {},
                      'groups': {},
                      'chains': {}
                      }

    def scaleLink(self, link, newlink):
        """Scales newly-created armatures depending on the link's largest collision object.
        The function is very simple and could be improved to scale the links even more appropriate.

        :param link: the link containing the collision objects
         :type link: dict.
        :param newlink: the link you want to scale
        :type newlink: dict.
        :return: Nothing.

        """
        newscale = 0.3
        if len(link['collision']) > 0:
            sizes = []
            for collname in link['collision']:
                collobj = link['collision'][collname]
                colltype = collobj['geometry']['type']
                if colltype == 'sphere':
                    sizes.append(collobj['geometry']['radius'])
                elif colltype == 'cylinder':
                    sizes.append(max(collobj['geometry']['radius'], collobj['geometry']['length']))
                elif colltype == 'mesh':
                    sizes.append(max(collobj['geometry']['scale']))  # TODO: this alone is not very informative
                else:
                    sizes.append(max(collobj['geometry']['size']))
            newscale = max(sizes)
        newlink.scale = (newscale, newscale, newscale)

    def placeChildLinks(self, parent):
        """This function creates the parent-child-lationship for a given parent and all existing children in blender.

        :param parent: This is the parent link you want to set the children for.
        :type: dict.
        :return: Nothing.

        """
        bpy.context.scene.layers = defLayers(defs.layerTypes['link'])
        print(parent['name'] + ', ', end='')
        children = []
        for l in self.robot['links']:
            if 'parent' in self.robot['links'][l] and self.robot['links'][l]['parent'] == parent['name']:
                children.append(self.robot['links'][l])
        for child in children:
            # 1: set parent relationship (this makes the parent inverse the inverse of the parents world transform)
            parentLink = bpy.data.objects[parent['name']]
            childLink = bpy.data.objects[child['name']]
            selectObjects([childLink, parentLink], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            # 2: move to parents origin by setting the world matrix to the parents world matrix
            childLink.matrix_world = parentLink.matrix_world
            # 3: apply local transform as saved in urdf (change matrix_local from identity to urdf)
            urdf_loc = mathutils.Matrix.Translation(child['pose']['translation'])
            urdf_rot = mathutils.Euler(tuple(child['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
            urdfmatrix = urdf_loc * urdf_rot
            childLink.matrix_local = urdfmatrix
            # 4: be happy, as world and basis are now the same and local is the transform to be exported to urdf
            # 5: take care of the rest of the tree
            self.placeChildLinks(child)

    def placeLinkSubelements(self, link):
        """This function finds all subelements for a given link and sets the appropriate relations.
        In this case subelements are interials, visuals and collisions.

        :param link: The parent link you want to set the subelements for
        :return: Nothing.

        """
        bpy.context.scene.layers = defLayers([defs.layerTypes[t] for t in defs.layerTypes])
        parentLink = bpy.data.objects[link['name']]
        if 'inertial' in link:
            if 'pose' in link['inertial']:
                urdf_geom_loc = mathutils.Matrix.Translation(link['inertial']['pose']['translation'])
                urdf_geom_rot = mathutils.Euler(tuple(link['inertial']['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
            else:
                urdf_geom_loc = mathutils.Matrix.Identity(4)
                urdf_geom_rot = mathutils.Matrix.Identity(4)
            inertialname = link['inertial']['name']
            inertialobj = bpy.data.objects[inertialname]
            selectObjects([inertialobj, parentLink], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            inertialobj.matrix_local = urdf_geom_loc * urdf_geom_rot
        for geomsrc in ['visual', 'collision']:
            if geomsrc in link:
                for g in link[geomsrc]:
                    geomelement = link[geomsrc][g]
                    if 'pose' in geomelement:
                        urdf_geom_loc = mathutils.Matrix.Translation(geomelement['pose']['translation'])
                        urdf_geom_rot = mathutils.Euler(tuple(geomelement['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
                    else:
                        urdf_geom_loc = mathutils.Matrix.Identity(4)
                        urdf_geom_rot = mathutils.Matrix.Identity(4)
                    geoname = geomelement['name']
                    geom = bpy.data.objects[geoname]
                    # FIXME: this does not do anything - how to set basis matrix to local?
                    #geom.matrix_world = parentLink.matrix_world
                    #selectObjects([geom], True, 0)
                    #bpy.ops.object.transform_apply(location=True, rotation=True)
                    selectObjects([geom, parentLink], True, 1)
                    bpy.ops.object.parent_set(type='BONE_RELATIVE')
                    geom.matrix_local = urdf_geom_loc * urdf_geom_rot
                    try:
                        geom.scale = geomelement['geometry']['scale']
                    except KeyError:
                        pass

    def attachSensor(self, sensor):
        """This function attaches a given sensor to its parent link.

        :param sensor: The sensor you want to attach to its parent link.
        :type sensor: dict.
        :return: Nothing.

        """
        bpy.context.scene.layers = defLayers([defs.layerTypes[t] for t in defs.layerTypes])
        try:
            parentLink = bpy.data.objects[sensor['link']]
            if 'pose' in sensor:
                urdf_geom_loc = mathutils.Matrix.Translation(sensor['pose']['translation'])
                urdf_geom_rot = mathutils.Euler(tuple(sensor['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
            else:
                urdf_geom_loc = mathutils.Matrix.Identity(4)
                urdf_geom_rot = mathutils.Matrix.Identity(4)
            sensorobj = bpy.data.objects[sensor['name']]
            selectObjects([sensorobj, parentLink], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            sensorobj.matrix_local = urdf_geom_loc * urdf_geom_rot
        except KeyError:
            log("inconsistent data on sensor: "+ sensor['name'], "ERROR")


    def createGeometry(self, viscol, geomsrc):
        #TODO: Write doc
        newgeom = None
        if viscol['geometry'] is not {}:
            dimensions = None
            bpy.ops.object.select_all(action='DESELECT')
            geom = viscol['geometry']
            geomtype = geom['type']
            # create the Blender object
            # tag all objects
            for obj in bpy.data.objects:
                obj.tag = True
            if geomtype == 'mesh':
                bpy.context.scene.layers = defLayers(defs.layerTypes[geomsrc])
                filetype = geom['filename'].split('.')[-1]
                if filetype == 'obj' or filetype == 'OBJ':
                    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, geom['filename']))
                elif filetype == 'stl' or filetype == 'STL':
                    bpy.ops.import_mesh.stl(filepath=os.path.join(self.path, geom['filename']))
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
                newgeom['filename'] = geom['filename']
                #newgeom.select = True
                #if 'scale' in geom:
                #    newgeom.scale = geom['scale']
                #bpy.ops.object.transform_apply(scale=True)
            elif geomtype == 'box':
                dimensions = geom['size']
            elif geomtype == 'cylinder':
                dimensions = (geom['radius'], geom['length'])
            elif geomtype == 'sphere':
                dimensions = geom['radius']
            else:
                log("Could not determine geometry type of " + geomsrc + viscol['name'] + '. Placing empty coordinate system.', "ERROR")
            if dimensions:  # if a standard primitive type is found, create the object
                newgeom = createPrimitive(viscol['name'], geomtype, dimensions, player=geomsrc)
                newgeom.select = True
                bpy.ops.object.transform_apply(scale=True)
            if newgeom is not None:
                newgeom.phobostype = geomsrc
                newgeom['geometry/type'] = geomtype
                if geomsrc == 'visual':
                    try:
                        newgeom.data.materials.append(bpy.data.materials[viscol['material']['name']])
                    except KeyError:
                        log('No material for obj', viscol['name'])
            #FIXME: place empty coordinate system and return...what? Error handling of file import!
        for prop in viscol:
            if prop.startswith('$'):
                for tag in viscol[prop]:
                    newgeom[prop[1:]+'/'+tag] = viscol[prop][tag]
        return newgeom

    def createInertial(self, name, inertial):
        """This function creates the blender representation of a given intertial.

        :param name: The intertials name.
        :param type: String.
        :param inertial: The intertial you want to create in blender form.
        :type intertial: dict.
        :return: Blender object -- the newly created blender intertial object.

        """
        bpy.ops.object.select_all(action='DESELECT')
        inert = createPrimitive('inertial_'+name, 'box', [0.04, 0.04, 0.04], player='inertial')
        inert.select = True
        bpy.ops.object.transform_apply(scale=True)
        for prop in inertial:
            if prop not in ['pose'] and inertial[prop] is not None:
                if not prop.startswith('$'):
                    inert[prop] = inertial[prop]
                else:
                    for tag in inertial[prop]:
                        inert[prop[1:]+'/'+tag] = inertial[prop][tag]
        inert.phobostype = 'inertial'
        return inert

    def createLink(self, link):
        """This function creates the blender representation of a given link

        :param link: The link you want to create a representation of.
        :type link: dict.
        :return: Blender object -- the newly created blender link object.

        """
        bpy.context.scene.layers = defLayers(defs.layerTypes['link'])
        #create base object ( =armature)
        bpy.ops.object.select_all(action='DESELECT')
        #bpy.ops.view3d.snap_cursor_to_center()
        bpy.ops.object.armature_add(layers=defLayers(0))
        newlink = bpy.context.active_object #print(bpy.context.object) #print(bpy.context.scene.objects.active) #bpy.context.selected_objects[0]
        newlink.name = link['name']
        newlink.location = (0.0, 0.0, 0.0)
        newlink.scale = (0.3, 0.3, 0.3) #TODO: make this depend on the largest visual or collision object
        bpy.ops.object.transform_apply(scale=True)
        newlink.phobostype = 'link'
        if newlink.name != link['name']:
            log("Warning, name conflict!")
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
        for prop in link:
            if prop.startswith('$'):
                for tag in link[prop]:
                    newlink['link/'+prop[1:]+'/'+tag] = link[prop][tag]
        return newlink

    def createJoint(self, joint):
        """This function creates the blender representation of a given joint.

        :param joint: The joint you want to create a blender object from.
        :type joint: dict.
        :return: Nothing.

        """
        bpy.context.scene.layers = defLayers(defs.layerTypes['link'])
        link = bpy.data.objects[joint['child']]
        # add joint information
        # link['joint/type'] = joint['type']

        # set axis
        selectObjects([link], clear=True, active=0)
        bpy.ops.object.mode_set(mode='EDIT')
        editbone = link.data.edit_bones[0]
        #oldaxis = editbone.vector
        length = editbone.length
        if 'axis' in joint:
            axis = mathutils.Vector(tuple(joint['axis']))
            #oldaxis.cross(axis) # rotation axis
            editbone.tail = editbone.head + axis.normalized() * length

        # add constraints
        for param in ['effort', 'velocity']:
            try:
                link['joint/max'+param] = joint['limits'][param]
            except KeyError:
                log("Key Error in adding joint constraints") #Todo: more details
        try:
            lower = joint['limits']['lower']
            upper = joint['limits']['upper']
        except KeyError:
            lower = 0.0
            upper = 0.0
        joints.setJointConstraints(link, joint['type'], lower, upper)
        for prop in joint:
            if prop.startswith('$'):
                for tag in joint[prop]:
                    link['joint/'+prop[1:]+'/'+tag] = joint[prop][tag]

    def createMotor(self, motor):
        """This function creates the motor properties in the motors joint object.

        :param motor: The motor you want to create the properties from.
        :type motor: dict.
        :return: Nothing.

        """
        try:
            joint = bpy.data.objects[motor['joint']]
            for prop in motor:
                if prop != 'joint':
                    if not prop.startswith('$'):
                        joint['motor/'+prop] = motor[prop]
                    else:
                        for tag in motor[prop]:
                            joint['motor/'+prop[1:]+'/'+tag] = motor[prop][tag]
        except KeyError:
            log("Joint " + motor['joint'] + " does not exist", "ERROR")

    def createSensor(self, sensor):
        newsensor = sensors.createSensor(sensor)

    def createController(self, controller):
        pass

    def createGroup(self, group):
        pass

    def createChain(self, chain):
        pass

    def createBlenderModel(self): #TODO: solve problem with duplicated links (linklist...namespaced via robotname?)
        """Creates the blender object representation of the imported model.
        For that purpose it uses the former specified robot model dictionary.

        :return: Nothing.

        """
        print("\n\nCreating Blender model...")
        print("Creating links...")
        for l in self.robot['links']:
            print(l + ', ', end='')
            link = self.robot['links'][l]
            self.createLink(link)

        print("\n\nCreating joints...")
        for j in self.robot['joints']:
            print(j + ', ', end='')
            joint = self.robot['joints'][j]
            self.createJoint(joint)

        print("\n\nCreating sensors...")
        for s in self.robot['sensors']:
            sensor = self.robot['sensors'][s]
            self.createSensor(sensor)

        #build tree recursively and correct translation & rotation on the fly
        for l in self.robot['links']:
            if not 'parent' in self.robot['links'][l]:
                root = self.robot['links'][l]
        print("\n\nPlacing links...")
        self.placeChildLinks(root)
        print("\n\nAssigning model name...")
        try:
            rootlink = getRoot(bpy.data.objects[root['name']])
            rootlink['modelname'] = self.robot['name']
        except KeyError:
            link("Could not assign model name to root link.", "ERROR")
        for link in self.robot['links']:
            self.placeLinkSubelements(self.robot['links'][link])
        for sensorname in self.robot['sensors']:
            sensor = self.robot['sensors'][sensorname]
            self.attachSensor(sensor)

        print("\n\nCreating motors...")
        for m in self.robot['motors']:
            motor = self.robot['motors'][m]
            self.createMotor(motor)

        print("\n\nCreating controllers...")
        for c in self.robot['controllers']:
            controller = self.robot['controllers'][c]
            self.createController(controller)

        print("\n\nCreating groups...")
        for g in self.robot['groups']:
            group = self.robot['groups'][g]
            self.createGroup(group)

        print("\n\nCreating chains...")
        for ch in self.robot['chains']:
            chain = self.robot['chains'][ch]
            self.createChain(chain)


class MARSModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a MARS scene

    """

    def __init__(self, filepath):
        """Inits the Parser with the MARS scene file location

        :param filepath: The filepath where the MARS scene lies.
        :type filepath: String.
        :return: Nothing.

        """
        RobotModelParser.__init__(filepath)

    def parseModel(self):
        """This function parses the MARS Scene.
        In fact it isn't implemented yet..

        :return: Nothing.
        """
        print("Parsing MARS scene...")


class URDFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a URDF model

    """

    def __init__(self, filepath):
        """Inits the Parser with the URDF file location

        :param filepath: The filepath where the URDF lies.
        :type filepath: String.
        :return: Nothing.

        """
        RobotModelParser.__init__(self, filepath)

    def parsePose(self, origin):
        """This function parses the robot models pose and returns it as a dictionary.

        :param origin: The origin blender object to parse the pose from.
        :type orign: blender object.
        :return: dict -- The origins pose.

        """
        pose = {}
        if origin is not None:
            pose['translation'] = parse_text(origin.attrib['xyz'])
            pose['rotation_euler'] = parse_text(origin.attrib['rpy'])
        else:
            pose['translation'] = defaults.idtransform
            pose['rotation_euler'] = defaults.idtransform
        return pose

    def parseModel(self):
        """This function parses the whole URDF representation of the robot and builds the robot dictionary from it.
        The created robot is stored in the robot value of the parser and the URDF file is specified by the filepath
        given to the Parser.

        :return: Nothing.

        """
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
        self.robot['links'] = links

        #write joints to dictionary
        joints = {}
        print("\n\nParsing joints..")
        for joint in self.root.iter('joint'):
            if joint.find('parent') is not None: #this is needed as there are "joint" tags e.g. in transmission
                newjoint, pose = self.parseJoint(joint)
                self.robot['links'][newjoint['child']]['pose'] = pose
                joints[newjoint['name']] = newjoint
        self.robot['joints'] = joints

        #find any links that still have no pose (most likely because they had no parent)
        for link in links:
            if not 'pose' in links[link]:
                links[link]['pose'] = self.parsePose(None)

        #write parent-child information to nodes
        print("\n\nWriting parent-child information to nodes..")
        for j in self.robot['joints']:
            joint = self.robot['joints'][j]
            self.robot['links'][joint['child']]['parent'] = joint['parent']
            print(joint['parent'] + ', ', end='')

        #now some debug output
        with open(self.filepath+'_debug.yml', 'w') as outputfile:
            outputfile.write(yaml.dump(self.robot))#, default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries

        materiallist = []  # TODO: build dictionary entry for materials
        print("\n\nParsing materials..")
        for material in self.root.iter('material'):
            newmaterial = {a: material.attrib[a] for a in material.attrib}
            color = material.find('color')
            if color is not None:
                newmaterial['color'] = parse_text(color.attrib['rgba'])
            texture = material.find('texture')
            if texture is not None:
                newmaterial['texture'] = texture.attrib['filename']
            if color is not None or texture is not None:
                print(material.attrib['name'] + ', ', end='')
                materiallist.append(newmaterial)
        for m in materiallist:
            materials.makeMaterial(m['name'], tuple(m['color'][0:3]), (1, 1, 1), m['color'][-1]) #TODO: handle duplicate names? urdf_robotname_xxx?

    def parseLink(self, link):
        """This function parses the link from the given link dict object.

        :param link: The link you want to
        :return:

        """
        print(link.attrib['name'] + ', ', end='')
        newlink = {a: link.attrib[a] for a in link.attrib}

        #parse 'inertial'
        inertial = link.find('inertial')
        if inertial is not None: # 'if Element' yields none if the Element contains no children, thus this notation
            newlink['inertial'] = {}
            newlink['inertial']['pose'] = self.parsePose(inertial.find('origin'))
            mass = inertial.find('mass')
            if mass is not None:
                newlink['inertial']['mass'] = float(mass.attrib['value'])
            inertia = inertial.find('inertia')
            if inertia is not None:
                values = []
                newlink['inertial']['inertia'] = values.append(inertia.attrib[a] for a in inertia.attrib)
            newlink['inertial']['name'] = 'inertial_' + newlink['name']

        #parse visual and collision objects
        for objtype in ['visual', 'collision']:
            newlink[objtype] = {}
            i = 0
            for xmlelement in link.iter(objtype):
                try:
                    elementname = xmlelement.attrib['name']
                except KeyError:
                    elementname = objtype + '_' + str(i) + '_' + newlink['name']
                    i += 1
                newlink[objtype][elementname] = {a: xmlelement.attrib[a] for a in xmlelement.attrib}
                dictelement = newlink[objtype][elementname]
                dictelement['name'] = elementname
                dictelement['pose'] = self.parsePose(xmlelement.find('origin'))
                geometry = xmlelement.find('geometry')
                if geometry is not None:
                    dictelement['geometry'] = {a: parse_text(geometry[0].attrib[a]) for a in geometry[0].attrib}
                    dictelement['geometry']['type'] = geometry[0].tag
                    if geometry[0].tag == 'mesh':
                        dictelement['geometry']['filename'] = geometry[0].attrib['filename']
                        try:
                            dictelement['geometry']['scale'] = parse_text(geometry[0].attrib['scale'])
                        except KeyError:
                            dictelement['geometry']['scale'] = [1.0, 1.0, 1.0]
                material = xmlelement.find('material')
                if material is not None:
                    dictelement['material'] = {'name': material.attrib['name']}
                    # We don't need to do the following, as any material with color or texture
                    # will be parsed in the parsing of materials in parseModel
                    # This might be necessary if there are name conflicts etc.
                    #color = material.find('color')
                    #if color is not None:
                    #    dictelement['material']['color'] = parse_text(color.attrib['rgba'])
        return newlink

    def parseJoint(self, joint):
        print(joint.attrib['name']+', ', end='')
        newjoint = {a: parse_text(joint.attrib[a]) for a in joint.attrib}
        pose = self.parsePose(joint.find('origin'))
        newjoint['parent'] = joint.find('parent').attrib['link']
        newjoint['child'] = joint.find('child').attrib['link']
        axis = joint.find('axis')
        if axis is not None:
            newjoint['axis'] = parse_text(axis.attrib['xyz'])
        limit = joint.find('limit')
        if limit is not None:
            newjoint['limits'] = {a: parse_text(limit.attrib[a]) for a in limit.attrib}
        #calibration
        #dynamics
        #mimic
        #safety_controller
        return newjoint, pose
    
class SRDFModelParser(RobotModelParser):
    """Class derived from RobotModelParser wich parses a SRDF extension file for URDF"""
    
    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)
        
    def parseModel(self, robot):
        collision_Exclusives = self.buildCollisionExclusives()
        collision_Dic = self.buildCollisionDictionary(collision_Exclusives, robot)
        collision_Groups = self.buildCollisionGroups(collision_Dic)
        robot = self.buildBitmasks(collision_Groups, robot)
        return robot
        
    def buildBitmasks(self, collision_Groups, robot):
        bits = len(collision_Groups)
        if bits > 20:
            print("The blender bitmask is not capable of more than 20 bit. The bitmask will be cutted!")
            bits = 20
        for link in robot['links']:
            for i in range(0, bits):
                if link in collision_Groups[i]:
                    for coll in robot['links'][link]['collision']:
                        try:
                            robot['links'][link]['collision'][coll]['bitmask'] += 2**i
                        except KeyError:
                            robot['links'][link]['collision'][coll]['bitmask'] = 2**i
        return robot
    
    def buildCollisionExclusives(self):
        print("\nParsing SRDF extensions from", self.filepath)
        self.tree = ET.parse(self.filepath)
        self.root = self.tree.getroot()
        
        collision_Exclusives = []
        for disabled_coll in self.root.iter('disable_collisions'):
            pair = (disabled_coll.attrib['link1'], disabled_coll.attrib['link2'])
            collision_Exclusives.append(pair)
            #print("Append ", pair, " to collision Exclusives")
        return collision_Exclusives
        
    
    def buildCollisionDictionary(self, collision_exclusives, robot):
        dic = {}
        for pair in collision_exclusives:
            if 'root' in pair or (pair[0] != robot['joints'][pair[1]]['parent'] and pair[1] != robot['joints'][pair[0]]['parent']):
                if pair[0] not in dic:
                    dic[pair[0]]=[]
                    dic[pair[0]].append(pair[1])
                else:
                    if pair[1] not in dic[pair[0]]:
                        dic[pair[0]].append(pair[1])
                if pair[1] not in dic:
                    dic[pair[1]]=[]
                    dic[pair[1]].append(pair[0])
                else:
                    if pair[0] not in dic[pair[1]]:
                        dic[pair[1]].append(pair[0])
            else:
                pass
                #print("Pair: ", pair, " not included")
        print("Collision Dictionary:\n", dic)
        return dic
                
    def checkGroup(self, group, colls):
        cut = []
        for elem in group:
            if elem in colls:
                cut.append(elem)
        if len(cut) == len(group):
            return cut
        else:
            return []

    def processGroup(self, group, link, colls):
        if link in group:
            for coll in colls:
                if coll in group:
                    colls.remove(coll)
        else:
            cut = self.checkGroup(group, colls)
            if len(cut) > 0:
                group.append(link)
                for l in cut:
                    colls.remove(l)
                
    def buildCollisionGroups(self, dic):
        groups=[]
        for link in dic:
            #rint("Current link: ", link)
            colls = dic[link]
            #print("Current colls: ", colls)
            for group in groups:
                #print("Current group: ", group)
                self.processGroup(group, link, colls)
            while len(colls) > 0:
                newgroup = [link, colls.pop()]
                groups.append(newgroup)
                self.processGroup(newgroup, link, colls)
        print ("Number of collision Groups: ", len(groups))
        #print ("Collision Groups:\n", groups)
        return groups
        

class SMURFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a SMURF model"""

    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)

    def parseModel(self):
        print("Parsing SMURF model...")
        #smurf = None
        with open(self.filepath, 'r') as smurffile:
            smurf = yaml.load(smurffile)
        if smurf is None:
            log('No valid SMURF file.', "ERROR")
            return None
        urdffile = None
        srdffile = None
        ymlfiles = [f for f in smurf['files'] if f.endswith('.yml') or f.endswith('.yaml')]
        for f in smurf['files']:
            if f.endswith('.urdf'):
                urdffile = f
            if f.endswith('.srdf'):
                srdffile = f
        # get URDF info
        if urdffile is None:
            log("Did not find URDF file associated with SMURF.", "ERROR")
            return None
        urdfparser = URDFModelParser(os.path.join(self.path, urdffile))
        urdfparser.parseModel()
        if srdffile is not None:
            srdfparser = SRDFModelParser(os.path.join(self.path, srdffile))
            self.robot = srdfparser.parseModel(urdfparser.robot)
        else:
            self.robot = urdfparser.robot
        # make sure all types exist
        typelist = ['links', 'joints', 'materials', 'sensors', 'motors', 'controllers', 'groups', 'chains']
        for key in typelist:
            if key not in self.robot:
                self.robot[key] = {}
        #add the smurf information
        custom_dicts = {}
        for yml in ymlfiles:
            with open(os.path.join(self.path, yml), 'r') as ymlfile:
                ymldict = yaml.load(ymlfile)
            for key in ymldict:
                print(key)
                if key in ['materials', 'sensors', 'motors', 'controllers']:
                    for element in ymldict[key]:
                        if element['name'] not in self.robot[key]:
                            self.robot[key][element['name']] = element
                        else:
                            for tag in element:
                                self.robot[key][element['name']][tag] = element[tag]
                elif key in 'state':
                    pass  # TODO: handle state
                else:
                    custom_dicts[key] = ymldict[key]

        for key in custom_dicts:
            print('assign custom properties:', key, custom_dicts[key])
            for element in custom_dicts[key]:  # iterate over list with custom annotations
                print(element)
                try:
                    objtype = element['type']
                except KeyError:
                    log("Could not find 'type' in custom annotation: " + str(element), "ERROR")
                try:
                    objname = element['name']
                except KeyError:
                    log("Could not find 'name' in custom annotation: " + str(element), "ERROR")
                try:
                    if objtype+'s' in typelist:  #FIXME: this is a total hack!
                        objtype += 's'
                    else:
                        raise TypeError(objtype)
                    if objname in self.robot[objtype]:
                        for tag in element:
                            if tag not in ['type', 'name']:
                                if not '$'+key in self.robot[objtype][objname]:
                                    self.robot[objtype][objname]['$'+key] = {tag: element[tag]}
                                else:
                                    self.robot[objtype][objname]['$'+key][tag] = element[tag]
                    else:
                        raise NameError(objname)
                except TypeError:
                    print("###ERROR: could not find 'type' or 'name' in custom annotation", objtype, objname)
                except NameError:
                    log("Element " + str(objname) + " of type " + str(objtype) + " does not exist in this model.", "ERROR")

        #now some debug output
        with open(self.filepath+'_SMURF_debug.yml', 'w') as outputfile:
            outputfile.write(yaml.dump(self.robot))#, default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries




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

        importer.parseModel()
        importer.createBlenderModel()

        return {'FINISHED'}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}

# Register and add to the file selector
bpy.utils.register_class(RobotModelImporter)


def main():
    # call the newly registered operator
    cleanUpScene()
    bpy.ops.import_robot_model('INVOKE_DEFAULT')

if __name__ == '__main__':
    main()
