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
from xml.dom import minidom
from xml.etree import ElementTree as ET
from xml.etree.ElementTree import Element, SubElement

import bpy

from phobos.utils.io import xmlHeader
from phobos.utils.io import indent as phobosindentation
from phobos.utils.io import l2str as list_to_string
import phobos.utils.general as gUtils
from phobos.utils.blender import getPhobosPreferences
from phobos.phoboslog import log

# For future updates of the SDF spec version look at:
# https://bitbucket.org/osrf/sdformat/src/ --> Migration.md
# https://bitbucket.org/osrf/sdformat/src/ --> sdf/Migration.md
# https://bitbucket.org/osrf/sdformat/src/ --> sdf/*.sdf files
sdfversion = "1.5"

class xmlTagger(object):
    """ A simple class to create a syntax conform xml file. The line
    indentation space can be customized using the indent parameter.
    To nest xml files an initial indentation layer can be provided.
    The xmlTagger writes an output string using the provided functions and
    takes care of the indentations.
    """

    def __init__(self, indent='  ', initial=0):
        """ Creates a new xml tagger. The line indentation space can be
        customized using the indent parameter. To nest xml files an initial
        indentation layer can be provided.

            :param indent: the symbol(s) used for indentations.
            :type indent: str
            :param initial: indentation hierarchy of root element for xml file
            :type initial: int
            """
        self.indentation = initial
        self.initial = initial
        self.indent = indent
        self.workingTags = []
        self.output = []

    def ind(self):
        """ Helper function to return the current indentation depending on the
        hierarchy.

        :return: str -- the current indentation (e.g. "  ").
            """
        return "" + self.indentation * self.indent

    def ascend(self):
        """ Move up one hierarchical layer by finishing the current tag and
        removing one indentation.

        :exception IndentationError -- trying to move above root hierarchical
        layer
        """
        if self.indentation > self.initial:
            lasttag = self.workingTags.pop(-1)
            self.indentation -= 1
            self.output.append(self.ind() + "</" + lasttag + ">\n")
        else:
            IndentationError()

    def descend(self, tag, params=None):
        """ Move down one hierarchical layer using the new tag.
        Optional in-line attributes can be provided in the
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
            parameters = [key + '="' +
                          str(params[key]) + '" ' for key in params.keys()]
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
        """ Adds an attribute to the current element. The tag of the attribute
        is wrapped around its value.

        :param tag: The tag of the attribute.
        :type tag: str
        :param value: The value of the attribute
        :type value: str (will be casted anyway)
        """
        self.output.append(self.ind() + '<' + str(tag) +
                           '>' + str(value) + '</' + tag + '>\n')

    def get_indent(self):
        return self.indentation

    def get_output(self):
        """ Completes all trailing tags until at initial indentation and
        returns the output as string.

        :return: str -- the finished xml string.
        """
        # ascend to base layer
        while self.indentation > self.initial:
            self.ascend()

        return self.output


def getIndentedETString(elementtree):
    """Return the specified elementtree as an indented string which can be saved to a file.

    The indentation is based on the phobos.utils.io.indent variable.

    :param elementtree: the elementtree to be converted to string
    :return: string representation of the elementtree
    """
    return minidom.parseString(ET.tostring(elementtree)).toprettyxml(indent=phobosindentation)

def exportSDFPose(relativepose, indentation, poseobject=None):
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
    tagger = xmlTagger(initial=indentation)

    # relative poses are written to file as they are
    if not poseobject:
        posedata = relativepose
    # world poses are created by combined transform of the pose
    else:
        matrix = poseobject.matrix_world
        # FINAL remove when done
        # matrix = getCombinedTransform(poseobject, getRoot(poseobject))
        posedata = {'rawmatrix': matrix,
                    'matrix': [list(vector) for vector in list(matrix)],
                    'translation': list(matrix.to_translation()),
                    'rotation_euler': list(matrix.to_euler()),
                    'rotation_quaternion': list(matrix.to_quaternion())}

    # only translation and euler rotation are required
    tra = posedata['translation']
    rot = posedata['rotation_euler']
    result = '{0} {1} {2} {3} {4} {5}'.format(
        tra[0], tra[1], tra[2], rot[0], rot[1], rot[2])
    tagger.attrib('pose', result)
    return "".join(tagger.get_output())


def exportSDFFrame(framedata, indentation, relative):
    """ Simple wrapper for frame data.
    The name of the frameobject (has to be a key in framedata) is used to
    define the object pose.

    :param framedata: data as provided by dictionary
    :param indentation: indentation at current level
    :param relative: True to apply a pose to the frame object itself (NOT
    SUPPORTED YET)
    :return: str -- writable xml line
    """
    tagger = xmlTagger(initial=indentation)
    tagger.descend('frame', {'name': framedata['name']})
    # relative frame pose is not supported yet
    # tagger.write(exportSDFPose(framedata['pose'], tagger.get_indent(),
    # relative))
    tagger.ascend()
    return "".join(tagger.get_output())


def exportSDFInertial(inertialdata, indentation):
    """ Simple wrapper for link inertial data.
    The inertial object is required to determine the position (pose) of the
    object.
    If relative poses are used the data found in inertialdata is used.
    Otherwise the pose of the inertialobject will be combined with all
    collected links up to the rootobject (see
    phobos.utils.editing.getCombinedTransform).

    :param inertialobj: object to be used for absolute pose
    :type inertialobj: Blender object.
    :param inertialdata: data as provided by dictionary (should contain mass
    and inertia)
    :type inertialdata: dict.
    :param indentation: indentation at current level
    :type indentation: int.
    :return: str -- writable xml line
    """

    # 'mass', 'pose', 'name', 'inertia'

    tagger = xmlTagger(initial=indentation)
    tagger.descend('inertial')
    if 'mass' in inertialdata:
        tagger.attrib('mass', inertialdata['mass'])
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
    if 'pose' in inertialdata:
        tagger.write(exportSDFPose(inertialdata['pose'], tagger.get_indent()))
    tagger.ascend()
    return "".join(tagger.get_output())


def exportSDFCollision(collisionobj, collisiondata, indentation, modelname):
    """ Simple wrapper for link collision data.
    The collision object is required to determine the position (pose) of the
    object.
    If relative poses are used the data found in collisiondata is used.
    Otherwise the pose of the collisionobject will be combined with all
    collected links up to the rootobject (see
    phobos.utils.editing.getCombinedTransform).

    :param collisionobj: object to be used for absolute pose
    :param collisiondata: data as provided by dictionary (should contain name,
    geometry, [bitmask])
    :param indentation: indentation at current level
    :param relative: True for usage of sdf relative pathing
    :param modelname: the name of the model (required for geometry)
    :return: str -- writable xml line
    """
    # {'collision_leg2_lower': {'pose': {...}, 'name': '...',
    #                           'bitmask': 2, 'geometry': {...}},
    #  'collision_leg2_foot': {'pose': {...}, 'name': '...',
    #                          'bitmask': 4, 'geometry': {...}}}
    tagger = xmlTagger(initial=indentation)
    tagger.descend('collision', {'name': collisiondata['name']})
    # OPT: tagger.attrib('laser_retro', ...)
    # OPT: tagger.attrib('max_contacts', ...)
    # OPT: tagger.attrib('frame', ...)
    # Write collisionposition always relative to link!
    tagger.write(exportSDFPose(collisiondata['pose'], tagger.get_indent()))
    tagger.write(exportSDFGeometry(collisiondata['geometry'], tagger.get_indent(), modelname))
    # # SURFACE PARAMETERS
    if 'bitmask' in collisiondata:
        tagger.descend('surface')
        # # BOUNCE PART
        # OPT: tagger.descend('bounce')
        # OPT: tagger.attrib('restitution_coefficient', ...)
        # OPT: tagger.attrib('threshold', ...)
        # tagger.ascend()
        # # FRICTION PART
        # OPT: tagger.descend('friction')
        # OPT: tagger.descend('torsional')
        # OPT: tagger.attrib('coefficient', ...)
        # OPT: tagger.attrib('use_patch_radius', ...)
        # OPT: tagger.attrib('patch_radius', ...)
        # OPT: tagger.attrib('surface_radius', ...)
        # OPT: tagger.descend('ode')
        # OPT: tagger.attrib('slip', ...)
        # tagger.ascend()
        # tagger.ascend()
        # OPT: tagger.descend('ode')
        # OPT: tagger.attrib('mu', ...)
        # OPT: tagger.attrib('mu2', ...)
        # OPT: tagger.attrib('fdir1', ...)
        # OPT: tagger.attrib('slip1', ...)
        # OPT: tagger.attrib('slip2', ...)
        # tagger.ascend()
        # OPT: tagger.descend('bullet')
        # OPT: tagger.attrib('friction')
        # OPT: tagger.attrib('friction2', ...)
        # OPT: tagger.attrib('fdir1', ...)
        # OPT: tagger.attrib('rolling_friction', ...)
        # tagger.ascend()
        # tagger.ascend()
        # # CONTACT PART
        # TODO: Optional. Add if clause
        tagger.descend('contact')
        # OPT: tagger.attrib('collide_without_contact', ...)
        # OPT: tagger.attrib('collide_without_contact_bitmask', ...)
        # TODO: Optional. Add if clause
        bitstring = '0x{0:02d}'.format(collisiondata['bitmask'])
        tagger.attrib('collide_bitmask', bitstring)
        # OPT: tagger.attrib('poissons_ratio', ...)
        # OPT: tagger.attrib('elastic_modulus', ...)
        # OPT: tagger.descend('ode')
        # OPT: tagger.attrib('soft_cfm', ...)
        # OPT: tagger.attrib('soft_erp', ...)
        # OPT: tagger.attrib('kp', ...)
        # OPT: tagger.attrib('kd', ...)
        # OPT: tagger.attrib('max_vel', ...)
        # OPT: tagger.attrib('min_depth', ...)
        # tagger.ascend()
        # OPT: tagger.descend('bullet')
        # OPT: tagger.attrib('soft_cfm', ...)
        # OPT: tagger.attrib('soft_erp', ...)
        # OPT: tagger.attrib('kp', ...)
        # OPT: tagger.attrib('kd', ...)
        # REQ: tagger.attrib('split_impulse', ...)
        # REQ: tagger.attrib('split_impulse_penetration_threshold', ...)
        # tagger.ascend()
        # # SOFT CONTACT PART
        # OPT: tagger.descend('soft_contact')
        # OPT: tagger.descend('dart')
        # REQ: tagger.attrib('bone_attachment', ...)
        # REQ: tagger.attrib('stiffness', ...)
        # REQ: tagger.attrib('damping', ...)
        # REQ: tagger.attrib('flesh_mass_fraction', ...)
        # tagger.ascend()
        tagger.ascend()
        tagger.ascend()
    tagger.ascend()
    return "".join(tagger.get_output())


def exportSDFGeometry(geometrydata, indentation, modelname):
    """ Simple wrapper for geometry data of link collisions.

    :param geometrydata: data as provided by dictionary
    :param indentation: indentation at current level
    :param modelname: name used for gazebo model pathing
    :return: str -- writable xml line
    """
    # {'size': [0.23617, 0.06, 0.06], 'type': 'box'}
    # {'radius': 0.06, 'type': 'sphere'}
    import phobos.utils.io as ioUtils

    tagger = xmlTagger(initial=indentation)
    tagger.descend('geometry')
    # available geometries in geometries.py: box,cylinder,capsule,sphere,mesh
    # OPT: if geometrydata['type'] == 'empty':
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
    #     OPT: tagger.descend('heightmap')
    #     REQ: tagger.attrib('uri', ...)
    #     OPT: tagger.attrib('size', ...)
    #     OPT: tagger.attrib('pos', ...)
    #     OPT: tagger.descend('texture')
    #     REQ: tagger.attrib('size', ...)
    #     REQ: tagger.attrib('diffuse', ...)
    #     REQ: tagger.attrib('normal', ...)
    #     tagger.ascend()
    #     OPT: tagger.descend('blend')
    #     REQ: tagger.attrib('min_height', ...)
    #     REQ: tagger.attrib('fade_dist', ...)
    #     tagger.ascend()
    #     OPT: tagger.attrib('use_terrain_paging', ...)
    #     tagger.ascend()
    # elif geometrydata['type'] == 'image':
    #     OPT: tagger.descend('image')
    #     REQ: tagger.attrib('uri', ...)
    #     REQ: tagger.attrib('scale', ...)
    #     REQ: tagger.attrib('threshold', ...)
    #     REQ: tagger.attrib('height', ...)
    #     REQ: tagger.attrib('granularity', ...)
    #     tagger.ascend()
    elif geometrydata['type'] == 'mesh':
        tagger.descend('mesh')
        meshtype = ioUtils.getExpSettings().outputMeshtype
        tagger.attrib('uri', 'model://' + modelname + '/meshes/' +
                      geometrydata['filename'] + '.{0}'.format(meshtype))
    #     OPT: tagger.descend('submesh')
    #     REQ: tagger.attrib('name', ...)
    #     OPT: tagger.attrib('center', ...)
    #     tagger.ascend()
    #     TODO: Optional. Add if clause
        tagger.attrib('scale', '{0} {1} {2}'.format(*geometrydata['scale']))
        tagger.ascend()
    # elif geometrydata['type'] == 'plane':
    #     OPT: tagger.descend('plane')
    #     REQ: tagger.attrib('normal', ...)
    #     REQ: tagger.attrib('size', ...)
    #     tagger.ascend()
    # elif geometrydata['type'] == 'polyline':
    #     OPT: tagger.descend('polyline')
    #     REQ: tagger.attrib('point', ...)
    #     REQ: tagger.attrib('height', ...)
    #     tagger.ascend()
    elif geometrydata['type'] == 'sphere':
        tagger.descend('sphere')
        tagger.attrib('radius', '{0}'.format(geometrydata['radius']))
        tagger.ascend()
    # OPTIONAL capsule is not supported by sdf: export as mesh
    # elif geometrydata['type'] == 'capsule':
    #     tagger.descend('mesh')
    #     tagger.attrib('uri', 'model://' + modelname + '/meshes/' +
    #                   geometrydata['filename'] + '.dae')
    #     tagger.descend('submesh')
    #     tagger.attrib('name', ...)
    #     tagger.attrib('center', ...)
    #     tagger.ascend()
    #     tagger.attrib('scale', '{0} {1} {2}'.format(*geometrydata['scale']))
    #     tagger.ascend()
    tagger.ascend()
    return "".join(tagger.get_output())


def exportSDFVisual(visualobj, linkobj, visualdata, indentation, modelname):
    """ Simple wrapper for visual data of links.
    The visual object is required to determine the position (pose) of the
    object.
    If relative poses are used the data found in visualdata (key pose) is used.
    Otherwise the pose of the visual object will be combined with all
    collected links up to the rootobject (see
    phobos.utils.editing.getCombinedTransform).

    :param visualobj: object to be used for pose
    :param visualdata: data as provided by dictionary (should contain name,
    geometry)
    :param indentation: indentation at current level
    :param relative: True for usage of sdf relative pathing
    :param modelname: the name of the model (required for geometry)
    :return: str -- writable xml line
    """

    # geometry': 'pose': 'material': 'upper_leg', 'name': 'visual_leg2_upper'
    tagger = xmlTagger(initial=indentation)
    tagger.descend('visual', params={'name': visualdata['name']})
    # OPT: tagger.attrib('cast_shadows', ...)
    # OPT: tagger.attrib('laser_retro', ...)
    # OPT: tagger.attrib('transparency', ...)
    # OPT: tagger.descend('meta')
    # OPT: tagger.attrib('layer', ...)
    # tagger.ascend()
    # OPT: tagger.write(exportSDFFrame(..., tagger.get_indent()))

    # Pose data of the visual is transformed by link --> use local matrix
    matrix = visualobj.matrix_local
    posedata = {'rawmatrix': matrix,
                'matrix': [list(vector) for vector in list(matrix)],
                'translation': list(matrix.to_translation()),
                'rotation_euler': list(matrix.to_euler()),
                'rotation_quaternion': list(matrix.to_quaternion())}
    # overwrite absolute position of the visual object
    tagger.write(exportSDFPose(posedata, tagger.get_indent()))

    # write material data if available
    if 'material' in visualdata:
        tagger.write(exportSDFMaterial(visualdata['material'], tagger.get_indent()))

    tagger.write(exportSDFGeometry(visualdata['geometry'], tagger.get_indent(),
                          modelname))
    tagger.ascend()
    return "".join(tagger.get_output())


def exportSDFMaterial(materialdata, indentation):
    """ Simple wrapper for material data of visual objects.
    The materialdata is the model dictionary of the specific material.

    :param materialdata: the material information
    :param visualdata: data as provided by dictionary (should contain
    diffuseColor, specularColor etc)
    :param indentation: indentation at current level
    :return: str -- writable xml line
    """
    tagger = xmlTagger(initial=indentation)
    tagger.descend('material')
    alpha = materialdata[
        'transparency'] if 'transparency' in materialdata else '1.0'
    # {'mat_wheel': {'ambientColor': {'r': 0.02562, 'g': 0.20235, 'b': 0.0156},
    # 'users': 2, 'specularColor': {'r': 0.5, 'g': 0.5, 'b': 0.5},
    # 'name': 'mat_wheel', 'shininess': 25.0,
    # 'diffuseColor': {'r': 0.02562, 'g': 0.20235, 'b': 0.0156},
    # 'emissionColor': {'r': 1.0, 'g': 0, 'b': 0.62306}

    # OPT: tagger.descend('plugin', ...)
    # OPT: tagger.descend('shader', ...)
    # OPT: tagger.attrib('lighting', ...)

    ambient = materialdata['ambientColor']
    tagger.attrib('ambient', '{0} {1} {2} {3}'.format(
                  ambient['r'], ambient['g'], ambient['g'], alpha))

    diffuse = materialdata['diffuseColor']
    tagger.attrib('diffuse', '{0} {1} {2} {3}'.format(
        diffuse['r'], diffuse['g'], diffuse['b'], alpha))

    specular = materialdata['specularColor']
    tagger.attrib('specular', '{0} {1} {2} {3}'.format(
        specular['r'], specular['g'], specular['b'], 1.0))

    if 'emissionColor' in materialdata:
        emission = materialdata['emissionColor']
        tagger.attrib('emissive', '{0} {1} {2} {3}'.format(
            emission['r'], emission['g'], emission['b'], 1.0))
    tagger.ascend()
    return "".join(tagger.get_output())


def exportGazeboModelConf(model):
    """Creates a model.config element from the specified information.

    :param model: the model dictionary of the phobos model
    :returns: xml.etree.ElementTree.Element of the model
    """
    modelconf = Element('model')
    SubElement(modelconf, 'name').text = model['name']
    SubElement(modelconf, 'version').text = 'DUMMY'

    SubElement(modelconf, 'sdf', version=sdfversion).text = model['name'] + '.sdf'

    phobosprefs = getPhobosPreferences()
    if phobosprefs.username != '':
        username = phobosprefs.username
    else:
        username = "Undefined"
    if phobosprefs.useremail != '':
        useremail = phobosprefs.useremail
    else:
        useremail = "Undefined"

    authorEL = SubElement(modelconf, 'author')
    SubElement(authorEL, 'name').text = username
    SubElement(authorEL, 'email').text = useremail
    # TODO allow user to edit a txt file in blender which contains the description OR take README?
    # SubElement(modelconf, 'description').text = PARSEFROMTXTEDITOR

    return modelconf


def exportSDF(model, filepath):
    """ Export function used for the entity.
    This exports a model SDF file as well as its model.conf to the specified filepath.
    """
    log("Export SDF (version " + sdfversion + ") to " + filepath, "INFO", "exportSdf")
    filename = os.path.join(filepath, model['name'] + '.sdf')
    modelconffile = os.path.join(filepath, 'model.config')
    errors = False

    modelconf = exportGazeboModelConf(model)

    # TODO add version to model dictionary!
    # 'sensors', 'materials', 'controllers', 'date', 'links', 'chains',
    # 'meshes', 'lights', 'motors', 'groups', 'joints', 'name'
    log('Exporting "{0}"...'.format(model['name']), "DEBUG", "exportSdf")
    # FINAL remove debugging information
    # print('sensors\n')
    # print(model['sensors'])
    print('Model materials implemented.')
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
    xml = xmlTagger(indent=phobosindentation)
    try:
        xml.write(xmlHeader)
        xml.descend('sdf', {"version": sdfversion})

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

        # OPT: static model
        # xml.attrib('static', ...)

        # OPT: self collide (all links collide with each other)
        # xml.attrib('self_collide', ...)

        # OPT: allows auto disabling of the model when at rest (only jointless
        # models)
        # xml.attrib('allow_auto_disable', ...)

        # include stuff from uri
        # OPT: xml.descend('include')
        # REQ: xml.attrib('uri', ...)
        # OPT: xml.attrib('pose', ...)
        # OPT: xml.attrib('name', ...)
        # OPT: xml.attrib('static', ...)
        # xml.ascend()

        # nested model element
        # OPT: xml.descend('model', params={'name': ...})
        # add wrapper for xml model?
        # xml.ascend()

        # enables wind influence on all links in the model (overriden by link
        # wind property)
        # OPT: xml.attrib('enable_wind', ...)

        # frame
        # OPT: xml.descend('frame', {'name': ...})
        # OPT: xml.attrib('pose', poseVal) OR xml.descend('pose') \\
        # xml.attrib('frame', otherFrame) \\ xml.ascend()
        # xml.ascend()

        # pose
        # OPT: xml.attrib('pose', poseVal) OR xml.descend('pose') \\
        # xml.attrib('frame', otherFrame) \\ xml.ascend()
        # link
        for linkkey in model['links'].keys():
            link = model['links'][linkkey]
            # 'parent', 'inertial', 'name', 'visual', 'pose', 'collision',
            # 'approxcollision', 'collision_bitmask'
            linkobj = bpy.context.scene.objects[link['name']]
            xml.descend('link', {'name': link['name']})
            # OPT: xml.attrib('gravity', ...)
            # OPT: xml.attrib('enable_wind', ...)
            # OPT: xml.attrib('self_collide', ...)
            # OPT: xml.attrib('kinematic', ...)
            # OPT: xml.attrib('must_be_base_link', ...)
            # OPT: xml.descend('velocity_decay')
            # OPT: xml.attrib('linear', ...)
            # OPT: xml.attrib('angular', ...)
            # xml.ascend()
            # OPT: xml.write(exportSDFFrame(model['frame']), xml.get_indent())
            # TODO Optional. Add if clause
            xml.write(exportSDFPose(link['pose'], xml.get_indent(), poseobject=linkobj))
            # inertial data might be missing
            if link['inertial']:
                xml.write(exportSDFInertial(link['inertial'], xml.get_indent()))
            else:
                log('No inertial data for "{0}"...'.format(link['name']), "WARNING", "exportSdf")

            # collision data might be missing
            if len(link['collision'].keys()) > 0:
                for colkey in link['collision'].keys():
                    colliname = link['collision'][colkey]['name']
                    collisionobj = bpy.context.scene.objects[colliname]
                    xml.write(exportSDFCollision(collisionobj,
                                        link['collision'][colkey],
                                        xml.get_indent(), modelname))
            else:
                log('No collision data for "{0}"...'.format(link['name'],
                                                            "WARNING",
                                                            "exportSdf"))

            # there might be no visual objects
            if len(link['visual'].keys()) > 0:
                for visualkey in link['visual'].keys():
                    visualobj = bpy.context.scene.objects[visualkey]
                    visualdata = link['visual'][visualkey]

                    # add material information to the visualdata if available
                    if 'material' in visualdata:
                        material = model['materials'][visualdata['material']]
                        visualdata['material'] = material
                    xml.write(exportSDFVisual(visualobj, linkobj, visualdata,
                                     xml.get_indent(), modelname))
            else:
                log('No visual data for "{0}"...'.format(link['name'],
                                                         "WARNING",
                                                         "exportSdf"))

            # OPT: xml.write(sensor(link['sensor'], xml.get_indent()))
            # OPT: xml.descend('projector', {'name': ...})
            # REQ: xml.attrib('texture', ...)
            # OPT: xml.attrib('fov', ...)
            # OPT: xml.attrib('near_clip', ...)
            # OPT: xml.attrib('far_clip', ...)
            # OPT: xml.write(exportSDFFrame('...', xml.get_indent()))
            # OPT: xml.write(exportSDFPose('...', xml.get_indent()))
            # OPT: xml.descend('plugin', {'name': ..., 'filename': ...})
            # TODO Add plugin element?
            # xml.ascend()
            # xml.ascend()
            # OPT: xml.attrib('audio_sink', ...)
            # OPT: xml.descend('audio_source')
            # REQ: xml.attrib('uri', ...)
            # OPT: xml.attrib('pitch', ...)
            # OPT: xml.attrib('gain', ...)
            # OPT: xml.descend('contact')
            # REQ: xml.attrib('collision', ...)
            # xml.ascend()
            # OPT: xml.attrib('loop', ...)
            # OPT: xml.write(exportSDFFrame('...', xml.get_indent()))
            # OPT: xml.write(exportSDFPose('...', xml.get_indent()))
            # xml.ascend()
            # OPT: xml.descend('battery', {'name': ...})
            # REQ: xml.attrib('voltage', ...)
            # xml.ascend()
            xml.ascend()

        log('Links exported.', 'DEBUG', 'exportSdf')

        # FINAL move somewhere else
        # Joint mapping from URDF to SDF
        jointmapping = {
            'revolute': 'revolute',
            'continuous': 'revolute',
            'prismatic': 'prismatic',
            'fixed': 'fixed',
            'floating': 'TODO',
            'planar': 'TODO'
        }

        # joint
        for jointkey in model['joints'].keys():
            joint = model['joints'][jointkey]
            # use sdf joint names instead URDF
            sdftype = jointmapping[joint['type']]
            xml.descend('joint', {'name': joint['name'], 'type': sdftype})
            # FINAL remove when all joints are finished
            if sdftype == 'TODO':
                log("joint type '" + joint['type'] + "' at joint '" +
                    joint['name'] + "' not supported yet.", 'ERROR')
            xml.attrib('parent', joint['parent'])
            xml.attrib('child', joint['child'])
            # OPT: xml.attrib('gearbox_ratio', ...)
            # OPT: xml.attrib('gearbox_reference_body', ...)'
            # OPT: xml.attrib('thread_pitch', ...)'
            if 'axis' in joint.keys():
                xml.descend('axis')
                # axis is defined in local coord space of parent link
                xml.attrib('xyz', list_to_string(joint['axis']))
                # TODO make this consistent with annotation objects
                xml.attrib('use_parent_model_frame', '1')
                # OPT: xml.descend('dynamics')
                # OPT: xml.attrib('damping', ...)
                # OPT: xml.attrib('friction', ...)
                # REQ: xml.attrib('spring_reference', ...)
                # REQ: xml.attrib('spring_stiffness', ...)
                # xml.ascend()
                if 'limits' in joint.keys():
                    xml.descend('limit')
                    # can be omitted for continuous joint
                    if 'lower' in joint['limits'].keys():
                        xml.attrib('lower', joint['limits']['lower'])
                    else:
                        log("The lower limit is missing for joint '" +
                            joint['name'] + ".", "WARNING")
                        xml.attrib('lower', '')
                    if 'upper' in joint['limits'].keys():
                        log("The upper limit is missing for joint '" +
                            joint['name'] + "'.", "WARNING")
                        xml.attrib('upper', joint['limits']['upper'])
                    else:
                        xml.attrib('upper', '')
                    if 'effort' in joint['limits'].keys():
                        xml.attrib('effort', joint['limits']['effort'])
                    if 'velocity' in joint['limits'].keys():
                        xml.attrib('velocity', joint['limits']['velocity'])
                    # OPT: xml.attrib('stiffness', ...)
                    # OPT: xml.attrib('dissipation', ...)
                    xml.ascend()
                xml.ascend()
            # if 'axis2' in joint.keys():
                # OPT: xml.descend('axis2')
                # REQ: xml.attrib('xyz', list_to_string(joint['axis']))
                # REQ: xml.attrib('use_parent_model_frame', ...)
                # OPT: xml.descend('dynamics')
                # OPT: xml.attrib('damping', ...)
                # OPT: xml.attrib('friction', ...)
                # REQ: xml.attrib('spring_reference', ...)
                # REQ: xml.attrib('spring_stiffness', ...)
                # xml.ascend()
                # OPT: xml.descend('limit')
                # OPT: xml.attrib('lower', joint['limits']['lower'])
                # OPT: xml.attrib('upper', joint['limits']['upper'])
                # OPT: xml.attrib('effort', joint['limits']['effort'])
                # OPT: xml.attrib('velocity', joint['limits']['velocity'])
                # OPT: xml.attrib('stiffness', ...)
                # OPT: xml.attrib('dissipation', ...)
                # xml.ascend()
                # xml.ascend()
            # if 'physics' in joint.keys():
                # OPT: xml.descend('physics')
                # OPT: xml.descend('simbody')
                # OPT: xml.attrib('must_be_loop_joint', ...)
                # xml.ascend()
                #
                # OPT: xml.descend('ode')
                # OPT: xml.attrib('cfm_damping', ...)
                # OPT: xml.attrib('implicit_spring_damper', ...)
                # OPT: xml.attrib('fudge_factor', ...)
                # OPT: xml.attrib('cfm', ...)
                # OPT: xml.attrib('erp', ...)
                # OPT: xml.attrib('bounce', ...)
                # OPT: xml.attrib('max_force', ...)
                # OPT: xml.attrib('velocity', ...)
                #
                # OPT: xml.descend('limit', ...)
                # REQ: xml.attrib('cfm', ...)
                # REQ: xml.attrib('erp', ...)
                # xml.ascend()
                #
                # OPT: xml.descend('suspension')
                # REQ: xml.attrib('cfm', ...)
                # REQ: xml.attrib('erp', ...)
                # xml.ascend()
                # xml.ascend()
                #
                # OPT: xml.attrib('provide_feedback', ...)
                # xml.ascend()
            # if 'frame' in joint.keys():
                # OPT: xml.write(exportSDFFrame(joint['frame']))
            # if 'pose' in joint.keys():
                # OPT: xml.write(exportSDFPose(...))
                # xml.ascend()
            # if 'sensor' in joint.keys():
                # OPT: xml.write(sensor('sensor'))
            xml.ascend()

        log("Joints exported.", "DEBUG", "exportSdf")

        # plugin
        # OPT: xml.descend('plugin', params={'name': ..., 'filename': ...)
        # xml.ascend()

        # gripper
        # OPT: xml.descend('gripper', {'name': ...})
        # OPT: xml.descend('grasp_check')
        # OPT: xml.attrib('detach_steps')
        # OPT: xml.attrib('attach_steps')
        # OPT: xml.attrib('min_contact_count')
        # xml.ascend()
        # REQ/OPT?: xml.attrib('gripper_link')
        # REQ: xml.attrib('palm_link')
        # xml.ascend()

    # FINAL remove this when finished
    except Exception:
        import sys
        import traceback
        sys.exc_info()[0]
        print(traceback.format_exc())
        errors = True
        log("Error in export!", "ERROR", "exportsdf")
    finally:
        outputtext = xml.get_output()

        log("Writing model sdf file to " + filename, "DEBUG")
        with open(filename, 'w') as outputfile:
            outputfile.writelines(outputtext)

        log("Writing model.config file to " + modelconffile, "DEBUG")
        with open(modelconffile, 'w') as outputfile:
            outputfile.write(getIndentedETString(modelconf))
        # modelconfTree.write(modelconffile, encoding="UTF-8", xml_declaration=True)

    finishmessage = "Export finished with " + \
        ("no " if not errors else "") + "errors."
    log(finishmessage, "INFO", "exportModelToSDF")


def parseSDFPose(pose):
    posedict = {}
    if pose is not None:
        xyzrpy = gUtils.parse_text(pose.text)
        posedict['translation'] = xyzrpy[:3]
        posedict['rotation_euler'] = xyzrpy[3:]
    else:
        posedict['translation'] = [0.0, 0.0, 0.0]
        posedict['rotation_euler'] = [0.0, 0.0, 0.0]
    return posedict


def parseSDFInertial(link):
    inertial_dict = {}
    inertial_data = link.find('inertial')
    # Element.find() yields None, not []
    if inertial_data is not None:
        log("   Parsing inertial.", 'DEBUG')
        inertial_dict['pose'] = parseSDFPose(inertial_data.find('pose'))
        mass = inertial_data.find('mass')
        if mass is not None:
            inertial_dict['mass'] = float(mass.text)
        inertia = inertial_data.find('inertia')

        # collect inertia matrix from sorted subelements (ixx, ixy, ixz, ...)
        if inertia is not None:
            inertial_dict['inertia'] = [float(elem.text) for elem in sorted(
                list(inertia), key=lambda el: el.tag)]
        inertial_dict['name'] = 'inertial_' + link.attrib['name']

        # TODO delete me
        import yaml
        print(yaml.dump(inertial_dict))
        return inertial_dict

        # TODO add frame support
    else:
        log("   No inertial defined for link {}.".format(link.attrib['name']), 'DEBUG')
        return None


def parseSDFGeometry(geometry, link, sdfpath):
    import os.path as path
    # gather generic properties from geometry definition
    geometrydict = {elem.tag: gUtils.parse_text(elem.text) for elem in list(geometry[0])}
    geometrydict['type'] = geometry[0].tag

    # gather mesh information
    if geometrydict['type'] == 'mesh':
        # interpret filename
        filepath = geometry[0].find('uri')
        if filepath is None:
            log("      URI for mesh not defined!", 'ERROR')
            return
        filepath = filepath.text

        # check filepath and include model folder of gazebo
        if 'model://' in filepath:
            phobosprefs = getPhobosPreferences()
            filepath = filepath.replace('model://', '')
            filepath = path.join(phobosprefs.gazebomodelfolder, filepath)
        else:
            filepath = path.normpath(path.join(path.dirname(sdfpath), filepath))
        log("       Filepath for mesh: {}".format(filepath), 'DEBUG')

        if not path.exists(filepath):
            log("     Mesh file does not exist: {} Replaced mesh with simple box.".format(filepath),
                'WARNING')
            geometrydict = {'type': 'box', 'size': [1, 1, 1]}
        else:
            geometry['filename'] = filepath

            # TODO add submesh support

            # read scale for meshes only
            if 'scale' in geometry[0].attrib:
                geometrydict['scale'] = gUtils.parse_text(geometry[0].attrib['scale'])
            else:
                geometrydict['scale'] = [1.0, 1.0, 1.0]

    # TODO remove me
    import yaml
    print(yaml.dump(geometrydict))
    return geometrydict


def parseSDFMaterial(visualname, material):
    materialdict = {}

    # sdf has no material names, so we initialize with visual name
    materialdict['name'] = 'mat_' + visualname

    # TODO add support
    # materialdict['sdf/script']
    # materialdict['sdf/shader']
    # materialdict['sdf/lighting']

    # gather material colors
    for color in ['ambient', 'diffuse', 'specular', 'emissive']:
        if material.find(color) is not None:
            materialdict[color] = gUtils.parse_text(material.find(color).text)

    materialdict['diffuse_intensity'] = 1.

    # TODO remove me
    import yaml
    print(yaml.dump(materialdict))
    return materialdict


def parseSDFLink(link, filepath):
    # collect all parameters which can be parsed as generic sdf annotations
    genericparams = [attrib for attrib in link.attrib
                     if attrib not in ['velocity_decay', 'frame', 'pose', 'inertial', 'collision',
                                       'visual', 'sensor', 'projector', 'audio_source', 'battery']]
    newlink = {'$sdf/' + a: link.attrib[a] for a in genericparams}

    newlink['children'] = []

    # TODO add support for other parameters
    # velocity_decay
    # frame

    newlink['pose'] = parseSDFPose(link.find('pose'))

    # parse inertial
    inertial = parseSDFInertial(link)
    if inertial:
        newlink['inertial'] = inertial

    materials = {}
    for objtype in ['visual', 'collision']:
        log("   Parsing {} elements...".format(objtype), 'DEBUG')
        objectsdict = {}
        i = 1
        for elem in link.iter(objtype):
            if 'name' not in elem.attrib:
                name = '{0}_{1:01d}_{2}'.format(objtype, i, newlink['name'])
                log("   No name for {} object! Assigning {} instead.".format(objtype, name),
                    'WARNING')
                i += 1
            else:
                name = elem.attrib['name']
            log("     {} element {}:".format(objtype[0].upper() + objtype[1:], name), 'DEBUG')

            elemdict = {'name': name}
            if objtype == 'collision':
                # TODO implement support for this
                # elemdict['sdf/laser_retro']
                # elemdict['sdf/max_contacts']
                # elemdict['sdf/frame']
                elemdict['pose'] = parseSDFPose(elem.find('pose'))
                # geometry is parsed below
                # TODO implement support
                # elemdict['sdf/surface']
            else:
                # TODO implement support for this
                # elemdict['sdf/cast_shadows']
                # elemdict['sdf/laser_retro']
                # elemdict['sdf/transparency']
                # elemdict['sdf/meta']
                # elemdict['sdf/frame']
                elemdict['pose'] = parseSDFPose(elem.find('pose'))
                # geometry is parsed below

                # undefined materials will be set to phobos_error
                if elem.find('material') is None:
                    log(("       No material defined for {} {} in link {}! Defaulting " +
                         "to error material.").format(objtype, name, newlink['name']), 'ERROR')
                    elemdict['material'] = 'phobos_error'
                # defined materials are collected in dictionary and linked with name
                else:
                    newmat = parseSDFMaterial(name, elem.find('material'))
                    elemdict['material'] = newmat['name']
                    materials[newmat['name']] = newmat
                # TODO implement support for this
                # elemdict['sdf/plugin']

            if elem.find('geometry') is None:
                log("   No geometry defined for {} {} in link {}! Skipped..".format(
                    objtype, name, newlink['name']), 'ERROR')
                continue
            elemdict['geometry'] = parseSDFGeometry(elem.find('geometry'), link, filepath)
            objectsdict[name] = elemdict
        newlink[objtype] = objectsdict

    # TODO parse sensors

    # TODO add projector, audio sink, audio_source, battery support

    if newlink == {}:
        log("Link information for " + newlink['name'] + " is empty.", 'WARNING')
    return newlink, materials



# registering export functions of types with Phobos
entity_type_dict = {'sdf': {
    'export': exportSDF,
    'import': importSDF,
    'extensions': ('sdf', 'xml')}}
