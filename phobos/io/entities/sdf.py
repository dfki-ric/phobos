#!/usr/bin/python
# coding=utf-8

"""
Copyright 2017, University of Bremen & DFKI GmbH Robotics Innovation Center

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

File sdf.py

Created on 06 Feb 2017

@author: Simon Reichel
"""

import os
import yaml
import xml.etree.ElementTree as ET
import bpy
import math

from phobos.utils.io import l2str, indent, xmlHeader
import phobos.model.materials as materials
import phobos.utils.general as gUtils
import phobos.utils.io as ioUtils
from phobos.phoboslog import log
from phobos.utils.selection import getRoot
from phobos.utils.editing import getCombinedTransform


class xmlTagger(object):
    """ A simple class to create a syntax conform xml file. The line indentation space can be customized using the
    indent parameter. To nest xml files an initial indentation layer can be provided. The xmlTagger writes an output
    string using the provided functions and takes care of the indentations.
    """

    def __init__(self, indent='  ', initial=0):
        """ Creates a new xml tagger. The line indentation space can be customized using the
        indent parameter. To nest xml files an initial indentation layer can be provided.

            :param indent: the symbol(s) used for indentations.
            :type indent: str
            :param initial: the indentation hierarchy of the root element for the xml file
            :type initial: int
            """
        self.indentation = initial
        self.initial = initial
        self.indent = indent
        self.workingTags = []
        self.output = []

    def ind(self):
        """ Helper function to return the current indentation depending on the hierarchy.

        :return: str -- the current indentation (e.g. "  ").
            """
        return "" + self.indentation * self.indent

    def ascend(self):
        """ Move up one hierarchical layer by finishing the current tag and removing one indentation.

        :exception IndentationError -- trying to move above root hierarchical layer
        """
        if self.indentation > self.initial:
            lasttag = self.workingTags.pop(-1)
            self.indentation -= 1
            self.output.append(self.ind() + "</" + lasttag + ">\n")
        else:
            IndentationError()

    def descend(self, tag, params=None):
        """ Move down one hierarchical layer using the new tag. Optional in-line attributes can be provided in the
        dictionary params (e.g. {'name': 'foo'}.

        :param tag: The tag used to create the new element.
        :type tag: str
        :param params: Optional: in-line attributes to put into the tag syntax
        :type params: dict
        """
        self.workingTags.append(str(tag))
        line = ""

        # create parameter strings by unpacking dictionary
        if params:
            parameters = [key + '="' + str(params[key]) + '" ' for key in params.keys()]
            # remove trailing whitespace
            parameters[-1] = parameters[-1][:-1]
        else:
            parameters = {}

        line += self.ind() + "<" + tag + (" " if len(parameters) > 0 else "")

        # add optional parameters
        if len(parameters) > 0:
            for param in parameters:
                line += param

        # finish line and descend one layer
        self.output.append(line + ">\n")
        self.indentation += 1

    def write(self, text):
        """ Write a custom line to the output. Use to create the header or comments.

        :param text: The line to write (line break has to be included)
        :type text: str
        """
        self.output.append(text)

    def attrib(self, tag, value):
        """ Adds an attribute to the current element. The tag of the attribute is wrapped around its value.

        :param tag: The tag of the attribute.
        :type tag: str
        :param value: The value of the attribute
        :type value: str (will be casted anyway)
        """
        self.output.append(self.ind() + '<' + str(tag) + '>' + str(value) + '</' + tag + '>\n')

    def get_indent(self):
        return self.indentation

    def get_output(self):
        """ Completes all trailing tags until at initial indentation and returns the output as string.

        :return: str -- the finished xml string.
        """
        # ascend to base layer
        while self.indentation > self.initial:
            self.ascend()

        return self.output


def pose(poseobject, posedata, indentation, relative):
    """ Simple wrapper for pose data.
    If relative poses are used the data found in posedata is used.
    Otherwise the pose of the poseobject will be combined with all collected
    links up to the rootobject (see phobos.utils.editing.getCombinedTransform).

    :param poseobject: object to be used for absolute pose
    :param posedata: the original (relative) posedata
    :param indentation: indentation at current level
    :param relative: True for usage of sdf relative pathing
    :return: str -- writable xml line
    """
    # {'matrix': [[-1.0, 0, 0, 0.3], [0, -1.0, 0, 0], [0, 0, 1.0, 0.1], [0, 0, 0, 1.0]],
    #  'rawmatrix': Matrix(((-1.0, 5.642599489874556e-07, 1.5100516392863028e-08, 0.2999999523162842),
    #                       (-5.642599489874556e-07, -1.0, -3.178652008273275e-08, 2.9802322387695312e-08),
    #                       (1.510050751107883e-08, -3.178649876645068e-08, 1.0, 0.09999991953372955),
    #                       (0.0, 0.0, 0.0, 1.0))), 'rotation_quaternion': [0, 0, 0, 1.0],
    #  'rotation_euler': [0, 0, -3.14159], 'translation': [0.3, 0, 0.1]}

    # 'matrix', 'rawmatrix', 'rotation_quaternion', 'rotation_euler', 'translation'

    # TODO location in meters, rotation in radians?!
    # TODO pose could be relative to different frame!
    tagger = xmlTagger(initial=indentation)

    # combine transformations of pose if relative is not used
    if not relative:
        matrix = getCombinedTransform(poseobject, getRoot(poseobject))
        posedata = {'rawmatrix': matrix,
            'matrix': [list(vector) for vector in list(matrix)],
            'translation': list(matrix.to_translation()),
            'rotation_euler': list(matrix.to_euler()),
            'rotation_quaternion': list(matrix.to_quaternion())}

    # only translation and euler rotation are required
    tra = posedata['translation']
    rot = posedata['rotation_euler']
    # convert radians to degree
    rot = [rad * 180. / math.pi for rad in rot]
    result = '{0} {1} {2} {3} {4} {5}'.format(tra[0], tra[1], tra[2], rot[0], rot[1], rot[2])
    tagger.attrib('pose', result)
    return "".join(tagger.get_output())


def frame(frameobj, framedata, indentation, relative):
    """ Simple wrapper for frame data.
    The frameobject is required to add the pose within the frame dependent on
    the relative pose parameter.

    :param frameobj: object to be used for absolute pose
    :param framedata: data as provided by dictionary
    :param indentation: indentation at current level
    :param relative: True when using relative sdf pathing
    :return: str -- writable xml line
    """
    tagger = xmlTagger(initial=indentation)
    tagger.descend('frame', {'name': '...'})
    tagger.write(pose(frameobj, framedata['pose'], tagger.get_indent(),
                      relative))
    tagger.ascend()
    return "".join(tagger.get_output())


def inertial(inertialobj, inertialdata, indentation, relative):
    """ Simple wrapper for link inertial data.
    The inertialobject is required to add the pose within the frame dependent on
    the relative pose parameter.

    :param inertialobj: object to be used for absolute pose
    :param inertialdata: data as provided by dictionary
    :param indentation: indentation at current level
    :return: str -- writable xml line
    """
    # {'mass': 0.5, 'pose': {...}, 'name': 'inertial_leg52', 'inertia': [0.00024, -4e-05, 4e-05, 0.00514, 0, 0.00515]}

    # 'mass', 'pose', 'name', 'inertia'

    tagger = xmlTagger(initial=indentation)
    tagger.descend('inertial')
    if 'mass' in inertialdata:
        tagger.attrib('mass', inertialdata['mass'])
    else:
        log("Object '{0}' without mass!".format(inertialobj.name), "WARNING", "exportSdf")
    if 'inertia' in inertialdata:
        inertia = inertialdata['inertia']
        tagger.descend('inertia')
        tagger.attrib('ixx', inertia[0])
        tagger.attrib('ixy', inertia[1])
        tagger.attrib('ixz', inertia[2])
        tagger.attrib('iyy', inertia[3])
        tagger.attrib('iyz', inertia[4])
        tagger.attrib('izz', inertia[5])
        tagger.ascend()
    else:
        log("Object '{0}' without inertia!".format(inertialobj.name), "WARNING",
            "exportSdf")
    # tagger.write(frame(inertialdata['frame'], tagger.getindent()))
    if 'pose' in inertialdata:
        tagger.write(pose(inertialobj, inertialdata['pose'], tagger.get_indent(),
                      relative))
    else:
        log("Object '{0}' has no inertial pose!".format(inertialobj.name),
            "WARNING", "exportsdf")
    tagger.ascend()
    return "".join(tagger.get_output())


def collision(collisionobj, collisiondata, indentation, relative, modelname):
    """ Simple wrapper for link collision data.
    The collisionobject is required to add the pose within the frame dependent on
    the relative pose parameter.

    :param collisionobj:
    :param collisiondata: data as provided by dictionary
    :param indentation: indentation at current level
    :return: str -- writable xml line
    """
    # {'collision_leg2_lower': {'pose': {...}, 'name': 'collision_leg2_lower',
    #                           'bitmask': 2, 'geometry': {'size': [0.27958, 0.06, 0.06], 'type': 'box'}},
    #  'collision_leg2_foot': {'pose': {...}, 'name': 'collision_leg2_foot',
    #                          'bitmask': 4, 'geometry': {'radius': 0.06, 'type': 'sphere'}}}
    tagger = xmlTagger(initial=indentation)
    tagger.descend('collision', {'name': collisiondata['name']})
    # tagger.attrib('laser_retro', ...)
    # tagger.attrib('max_contacts', ...)
    # tagger.attrib('frame', ...)
    tagger.write(pose(collisionobj, collisiondata['pose'], tagger.get_indent(),
                     relative))
    tagger.write(geometry(collisiondata['geometry'], tagger.get_indent(),
                          modelname))
    # # SURFACE PARAMETERS
    if 'bitmask' in collisiondata:
        tagger.descend('surface')
        # # BOUNCE PART
        # tagger.descend('bounce')
        # tagger.attrib('restitution_coefficient', ...)
        # tagger.attrib('threshold', ...)
        # tagger.ascend()
        # # FRICTION PART
        # tagger.descend('friction')
        # tagger.descend('torsional')
        # tagger.attrib('coefficient', ...)
        # tagger.attrib('use_patch_radius', ...)
        # tagger.attrib('patch_radius', ...)
        # tagger.attrib('surface_radius', ...)
        # tagger.descend('ode')
        # tagger.attrib('slip', ...)
        # tagger.ascend()
        # tagger.ascend()
        # tagger.descend('ode')
        # tagger.attrib('mu', ...)
        # tagger.attrib('mu2', ...)
        # tagger.attrib('fdir1', ...)
        # tagger.attrib('slip1', ...)
        # tagger.attrib('slip2', ...)
        # tagger.ascend()
        # tagger.descend('bullet')
        # tagger.attrib('friction')
        # tagger.attrib('friction2', ...)
        # tagger.attrib('fdir1', ...)
        # tagger.attrib('rolling_friction', ...)
        # tagger.ascend()
        # tagger.ascend()
        # # CONTACT PART
        tagger.descend('contact')
        # tagger.attrib('collide_without_contact', ...)
        # tagger.attrib('collide_without_contact_bitmask', ...)
        bitstring = '0x{0:02d}'.format(collisiondata['bitmask'])
        tagger.attrib('collide_bitmask', bitstring)
        # tagger.attrib('poissons_ratio', ...)
        # tagger.attrib('elastic_modulus', ...)
        # tagger.descend('ode')
        # tagger.attrib('soft_cfm', ...)
        # tagger.attrib('soft_erp', ...)
        # tagger.attrib('kp', ...)
        # tagger.attrib('kd', ...)
        # tagger.attrib('max_vel', ...)
        # tagger.attrib('min_depth', ...)
        # tagger.ascend()
        # tagger.descend('bullet')
        # tagger.attrib('soft_cfm', ...)
        # tagger.attrib('soft_erp', ...)
        # tagger.attrib('kp', ...)
        # tagger.attrib('kd', ...)
        # tagger.attrib('split_impulse', ...)
        # tagger.attrib('split_impulse_penetration_threshold', ...)
        # tagger.ascend()
        # # SOFT CONTACT PART
        # tagger.descend('soft_contact')
        # tagger.descend('dart')
        # tagger.attrib('bone_attachment', ...)
        # tagger.attrib('stiffness', ...)
        # tagger.attrib('damping', ...)
        # tagger.attrib('flesh_mass_fraction', ...)
        # tagger.ascend()
        tagger.ascend()
        tagger.ascend()
    tagger.ascend()
    return "".join(tagger.get_output())


def geometry(geometrydata, indentation, modelname):
    """ Simple wrapper for geometry data of link collisions.

    :param geometrydata: data as provided by dictionary
    :param indentation: indentation at current level
    :param modelname: name used for gazebo model pathing
    :return: str -- writable xml line
    """
    # {'size': [0.23617, 0.06, 0.06], 'type': 'box'}
    # {'radius': 0.06, 'type': 'sphere'}

    tagger = xmlTagger(initial=indentation)
    tagger.descend('geometry')
    # available geometries in geometries.py: box,cylinder,capsule,sphere,mesh
    # if geometrydata['type'] == 'empty':
    #     tagger.attrib('empty', ...)
    if geometrydata['type'] == 'box':
        tagger.descend('box')
        tagger.attrib('size', '{0} {1} {2}'.format(*geometrydata['size']))
        tagger.ascend()
    elif geometrydata['type'] == 'cylinder':
        tagger.descend('cylinder')
        tagger.attrib('radius', geometrydata['radius'])
        tagger.attrib('length', geometrydata['length'])
        tagger.ascend()
    # elif geometrydata['type'] == 'heightmap':
    #     tagger.descend('heightmap')
    #     tagger.attrib('uri', ...)
    #     tagger.attrib('size', ...)
    #     tagger.attrib('pos', ...)
    #     tagger.descend('texture')
    #     tagger.attrib('size', ...)
    #     tagger.attrib('diffuse', ...)
    #     tagger.attrib('normal', ...)
    #     tagger.ascend()
    #     tagger.descend('blend')
    #     tagger.attrib('min_height', ...)
    #     tagger.attrib('fade_dist', ...)
    #     tagger.ascend()
    #     tagger.attrib('use_terrain_paging', ...)
    #     tagger.ascend()
    # elif geometrydata['type'] == 'image':
    #     tagger.descend('image')
    #     tagger.attrib('uri', ...)
    #     tagger.attrib('scale', ...)
    #     tagger.attrib('threshold', ...)
    #     tagger.attrib('height', ...)
    #     tagger.attrib('granularity', ...)
    #     tagger.ascend()
    elif geometrydata['type'] == 'mesh':
        tagger.descend('mesh')
        tagger.attrib('uri',
                      'model://' + modelname + '/meshes/' + geometrydata['filename'] + '.dae')
    #     tagger.descend('submesh')
    #     tagger.attrib('name', ...)
    #     tagger.attrib('center', ...)
    #     tagger.ascend()
        tagger.attrib('scale', '{0} {1} {2}'.format(*geometrydata['scale']))
        tagger.ascend()
    # elif geometrydata['type'] == 'plane':
    #     tagger.descend('plane')
    #     tagger.attrib('normal', ...)
    #     tagger.attrib('size', ...)
    #     tagger.ascend()
    # elif geometrydata['type'] == 'polyline':
    #     tagger.descend('polyline')
    #     tagger.attrib('point', ...)
    #     tagger.attrib('height', ...)
    #     tagger.ascend()
    elif geometrydata['type'] == 'sphere':
        tagger.descend('sphere')
        tagger.attrib('radius', '{0}'.format(geometrydata['radius']))
        tagger.ascend()
    # TODO capsule is not supported by sdf: export as mesh
    # elif geometrydata['type'] == 'capsule':
    #     tagger.descend('mesh')
    #     tagger.attrib('uri',
    #                   'model://' + modelname + '/meshes/' + geometrydata['filename'] + '.dae')
    #     tagger.descend('submesh')
    #     tagger.attrib('name', ...)
    #     tagger.attrib('center', ...)
    #     tagger.ascend()
        # tagger.attrib('scale', '{0} {1} {2}'.format(*geometrydata['scale']))
        # tagger.ascend()
    tagger.ascend()
    return "".join(tagger.get_output())


def visual(visualobj, linkobj, visualdata, indentation, modelname):
    """ Simple wrapper for visual data of links.

    :param visualdata: data as provided by dictionary
    :param indentation: indentation at current level
    :return: str -- writable xml line
    """
    # {'geometry': 'pose': 'material': 'upper_leg', 'name': 'visual_leg2_upper'}
    tagger = xmlTagger(initial=indentation)
    tagger.descend('visual', params={'name': visualdata['name']})
    # tagger.attrib('cast_shadows', ...)
    # tagger.attrib('laser_retro', ...)
    # tagger.attrib('transparency', ...)
    # tagger.descend('meta')
    # tagger.attrib('layer', ...)
    # tagger.ascend()
    # tagger.write(frame(..., tagger.get_indent()))

    # Pose data of the visual is transformed by link
    # TODO fix matrix calculation
    matrix = visualobj.matrix_local
    # matrix = matrix * linkobj.matrix_local
    posedata = {'rawmatrix': matrix,
        'matrix': [list(vector) for vector in list(matrix)],
        'translation': list(matrix.to_translation()),
        'rotation_euler': list(matrix.to_euler()),
        'rotation_quaternion': list(matrix.to_quaternion())}
    tagger.write(pose(visualobj, posedata, tagger.get_indent(), True))
    # tagger.write(material(visualdata['material']), tagger.get_indent())
    tagger.write(geometry(visualdata['geometry'], tagger.get_indent(),
                              modelname))
    # PLUGIN ELEMENT?
    tagger.ascend()
    return "".join(tagger.get_output())

def exportSdf(model, filepath, relativeSDF=False):
    log("Export SDF to " + filepath, "INFO", "exportSdf")
    filename = os.path.join(filepath, model['name'] + '.sdf')
    errors = False

    # 'sensors', 'materials', 'controllers', 'date', 'links', 'chains', 'meshes',
    # 'lights', 'motors', 'groups', 'joints', 'name'
    log('Exporting "{0}"...'.format(model['name'], "DEBUG", "exportSdf"))
    # TODO remove debugging information
    # print('sensors\n')
    # print(model['sensors'])
    # print('materials\n')
    # print(model['materials'])
    # print('controllers\n')
    # print(model['controllers'])
    # print('date\n')
    # print(model['date'])
    print('Model links implemented.')
    # print(model['links'])
    # print('chains\n')
    # print(model['chains'])
    # print('meshes\n')
    # print(model['meshes'])
    # print('lights\n')
    # print(model['lights'])
    # print('motors\n')
    # print(model['motors'])
    # print('groups\n')
    # print(model['groups'])
    print('Model joints implemented.')
    # print(model['joints'])
    # print('name\n')

    # create tagger and add headers
    xml = xmlTagger(indent=indent)
    try:
        xml.write(xmlHeader)
        xml.descend('sdf', {"version": 1.5})

        # xml.descend('world', params={'name': 'default'})
        # xml.descend('include')
        # xml.attrib('uri', 'model://ground_plane')
        # xml.ascend()
        # xml.descend('include')
        # xml.attrib('uri', 'model://sun')
        # xml.ascend()
        # model layer
        modelname = model['name']
        xml.descend('model', params={"name": modelname})

        # static model
        # xml.attrib('static', ...)

        # self collide (all links collide with each other)
        # xml.attrib('self_collide', ...)

        # allows auto disabling of the model when at rest (only jointless models)
        # xml.attrib('allow_auto_disable', ...)

        # include stuff from uri
        # xml.descend('include')
        # xml.attrib('uri', ...)
        # xml.attrib('pose', ...)
        # xml.attrib('name', ...)
        # xml.attrib('static', ...)
        # xml.ascend()

        # nested model element
        # xml.descend('model', params={'name': ...})
        # add wrapper for xml model?
        # xml.ascend()

        # enables wind influence on all links in the model (overriden by link wind property)
        # xml.attrib('enable_wind', ...)

        # frame
        # xml.descend('frame', {'name': ...})
        # xml.attrib('pose', poseVal) OR xml.descend('pose') \\ xml.attrib('frame', otherFrame) \\ xml.ascend()
        # xml.ascend()

        # pose
        # xml.attrib('pose', poseVal) OR xml.descend('pose') \\ xml.attrib('frame', otherFrame) \\ xml.ascend()
        # link
        for linkkey in model['links'].keys():
            link = model['links'][linkkey]
            linkobj = bpy.context.scene.objects[link['name']]
            # 'parent', 'inertial', 'name', 'visual', 'pose', 'collision',
            # 'approxcollision', 'collision_bitmask'
            xml.descend('link', {'name': link['name']})
            # xml.attrib('gravity', ...)
            # xml.attrib('enable_wind', ...)
            # xml.attrib('self_collide', ...)
            # xml.attrib('kinematic', ...)
            # xml.attrib('must_be_base_link', ...)
            # xml.descend('velocity_decay')
            # xml.attrib('linear', ...)
            # xml.attrib('angular', ...)
            # xml.ascend()
            # xml.write(frame(model['frame']), xml.get_indent())
            xml.write(pose(linkobj, link['pose'], xml.get_indent(), relativeSDF))
            xml.write(inertial(linkobj, link['inertial'], xml.get_indent(),
                            relativeSDF))
            if len(link['collision'].keys()) > 0:
                for colkey in link['collision'].keys():
                    xml.write(collision(linkobj, link['collision'][colkey],
                                        xml.get_indent(), relativeSDF, modelname))
            if len(link['visual'].keys()) != 0:
                for visualkey in link['visual'].keys():
                    visualobj = bpy.context.scene.objects[visualkey]
                    xml.write(visual(visualobj, linkobj,
                                     link['visual'][visualkey],
                                     xml.get_indent(), modelname))
            # xml.write(sensor(link['sensor'], xml.get_indent()))
            # xml.descend('projector', {'name': ...})
            # xml.attrib('texture', ...)
            # xml.attrib('fov', ...)
            # xml.attrib('near_clip', ...)
            # xml.attrib('far_clip', ...)
            # xml.write(frame('...', xml.get_indent()))
            # xml.write(pose('...', xml.get_indent()))
            # xml.descend('plugin', {'name': ..., 'filename': ...})
            # # PLUGIN ELEMENT?
            # xml.ascend()
            # xml.ascend()
            # xml.attrib('audio_sink', ...)
            # xml.descend('audio_source')
            # xml.attrib('uri', ...)
            # xml.attrib('pitch', ...)
            # xml.attrib('gain', ...)
            # xml.descend('contact')
            # xml.attrib('collision', ...)
            # xml.ascend()
            # xml.attrib('loop', ...)
            # xml.write(frame('...', xml.get_indent()))
            # xml.write(pose('...', xml.get_indent()))
            # xml.ascend()
            # xml.descend('battery', {'name': ...})
            # xml.attrib('voltage', ...)
            # xml.ascend()
            xml.ascend()

        log('Links exported.', 'DEBUG', 'exportSdf')

        # joint
        for jointkey in model['joints'].keys():
            joint = model['joints'][jointkey]
            xml.descend('joint', {'name': joint['name'], 'type': joint['type']})
            xml.attrib('parent', joint['parent'])
            xml.attrib('child', joint['child'])
            # xml.attrib('gearbox_ratio', ...)
            # xml.attrib('gearbox_reference_body', ...)'
            # xml.attrib('thread_pitch', ...)'
            if 'axis' in joint.keys():
                xml.descend('axis')
                xml.attrib('xyz', l2str(joint['axis']))
                # xml.attrib('use_parent_model_frame', ...)
                # xml.descend('dynamics')
                # xml.attrib('damping', ...)
                # xml.attrib('friction', ...)
                # xml.attrib('spring_reference', ...)
                # xml.attrib('spring_stiffness', ...)
                # xml.ascend()
                xml.descend('limit')
                xml.attrib('lower', joint['limits']['lower'])
                xml.attrib('upper', joint['limits']['upper'])
                xml.attrib('effort', joint['limits']['effort'])
                xml.attrib('velocity', joint['limits']['velocity'])
                # xml.attrib('stiffness', ...)
                # xml.attrib('dissipation', ...)
                xml.ascend()
                xml.ascend()
            # if 'axis2' in joint.keys():
                # xml.descend('axis')
                # xml.attrib('xyz', l2str(joint['axis']))
                # xml.attrib('use_parent_model_frame', ...)
                # xml.descend('dynamics')
                # xml.attrib('damping', ...)
                # xml.attrib('friction', ...)
                # xml.attrib('spring_reference', ...)
                # xml.attrib('spring_stiffness', ...)
                # xml.ascend()
                # xml.descend('limit')
                # xml.attrib('lower', joint['limits']['lower'])
                # xml.attrib('upper', joint['limits']['upper'])
                # xml.attrib('effort', joint['limits']['effort'])
                # xml.attrib('velocity', joint['limits']['velocity'])
                # xml.attrib('stiffness', ...)
                # xml.attrib('dissipation', ...)
                # xml.ascend()
                # xml.ascend()
            # if 'physics' in joint.keys():
                # xml.descend('physics')
                # xml.descend('simbody')
                # xml.attrib('must_be_loop_joint', ...)
                # xml.ascend()
                #
                # xml.descend('ode')
                # xml.attrib('cfm_damping', ...)
                # xml.attrib('implicit_spring_damper', ...)
                # xml.attrib('fudge_factor', ...)
                # xml.attrib('cfm', ...)
                # xml.attrib('erp', ...)
                # xml.attrib('bounce', ...)
                # xml.attrib('max_force', ...)
                # xml.attrib('velocity', ...)
                #
                # xml.descend('limit', ...)
                # xml.attrib('cfm', ...)
                # xml.attrib('erp', ...)
                # xml.ascend()
                #
                # xml.descend('suspension')
                # xml.attrib('cfm', ...)
                # xml.attrib('erp', ...)
                # xml.ascend()
                # xml.ascend()
                #
                # xml.attrib('provide_feedback', ...)
                # xml.ascend()
            # if 'frame' in joint.keys():
                # xml.write(joint['frame'])
            # if 'pose' in joint.keys():
                # xml.descend('pose')
                # xml.attrib('frame', ...)
                # xml.ascend()
            # if 'sensor' in joint.keys():
                # xml.descend('sensor')
                # SENSOR WRAPPER
                # xml.ascend('sensor')
            xml.ascend()

        log("Joints exported.", "DEBUG", "exportSdf")

        # plugin
        # xml.descend('plugin')
        # xml.attrib('name', ...)
        # xml.attrib('filename', ...)
        # OPTIONAL xml.descend('otherplugin')
        # xml.ascend()

        # gripper
        # xml.descend('gripper', {'name': ...})
        # xml.descend('grasp_check')
        # xml.attrib('detach_steps')
        # xml.attrib('attach_steps')
        # xml.attrib('min_contact_count')
        # xml.ascend()
        # xml.attrib('gripper_link')
        # xml.attrib('palm_link')
        # xml.ascend()

    # TODO remove this when finished
    except Exception as e:
        import sys
        import traceback
        e = sys.exc_info()[0]
        print(e)
        print(traceback.format_exc())
        errors=True
        log("Error in export!", "ERROR", "exportsdf")
    finally:
        outputtext = xml.get_output()

        log("Writing model data to " + filename, "DEBUG", "exportSdf")
        with open(filename, 'w') as outputfile:
            outputfile.writelines(outputtext)
    finishmessage="Export finished with " + ("no " if not errors else "") + "errors."
    log(finishmessage, "INFO", "exportModelToSDF")


def importSdf():
    pass

# registering export functions of types with Phobos
entity_type_dict = {'sdf': {'export': exportSdf,
                            'import': importSdf,
                            'extensions': ('sdf', 'xml')}
                    }
