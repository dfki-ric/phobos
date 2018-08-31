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
import shutil
import glob
from xml.dom import minidom
from xml.etree import ElementTree as ET
from xml.etree.ElementTree import Element, SubElement

import bpy

import phobos.defs as defs
from phobos.utils.io import xmlHeader
from phobos.utils.io import indent as phobosindentation
from phobos.utils.io import l2str as list_to_string
from phobos.utils.io import getExpSettings
import phobos.utils.general as gUtils
from phobos.utils.blender import getPhobosPreferences
from phobos.phoboslog import log

# For future updates of the SDF spec version look at:
# https://bitbucket.org/osrf/sdformat/src/ --> Migration.md
# https://bitbucket.org/osrf/sdformat/src/ --> sdf/Migration.md
# https://bitbucket.org/osrf/sdformat/src/ --> sdf/*.sdf files
sdfversion = "1.5"

# Joint mapping from URDF to SDF
jointmapping = {
    'revolute': 'revolute',
    'continuous': 'revolute',
    'prismatic': 'prismatic',
    'fixed': 'fixed',
    'floating': 'TODO',
    'planar': 'TODO'
}


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
        posedata = gUtils.roundFloatsInDict(posedata, getExpSettings().decimalPlaces)

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
        tagger.descend('contact')
        # OPT: tagger.attrib('collide_without_contact', ...)
        # OPT: tagger.attrib('collide_without_contact_bitmask', ...)
        if 'bitmask' in collisiondata:
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
        meshtype = getExpSettings().export_sdf_mesh_type
        tagger.attrib('uri', 'model://' + modelname + '/meshes/' +
                      geometrydata['filename'] + '.{0}'.format(meshtype))
    #     OPT: tagger.descend('submesh')
    #     REQ: tagger.attrib('name', ...)
    #     OPT: tagger.attrib('center', ...)
    #     tagger.ascend()
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


def exportSDFLink(linkdict, linkobj, modelname, materials, indentation):
    # 'parent', 'inertial', 'name', 'visual', 'pose', 'collision',
    # 'approxcollision', 'collision_bitmask'
    tagger = xmlTagger(initial=indentation)
    tagger.descend('link', {'name': linkdict['name']})
    # OPT: tagger.attrib('gravity', ...)
    # OPT: tagger.attrib('enable_wind', ...)
    # OPT: tagger.attrib('self_collide', ...)
    # OPT: tagger.attrib('kinematic', ...)
    # OPT: tagger.attrib('must_be_base_link', ...)
    # OPT: tagger.descend('velocity_decay')
    # OPT: tagger.attrib('linear', ...)
    # OPT: tagger.attrib('angular', ...)
    # tagger.ascend()
    # OPT: tagger.write(exportSDFFrame(model['frame']), tagger.get_indent())
    tagger.write(exportSDFPose(linkdict['pose'], tagger.get_indent(), poseobject=linkobj))
    # inertial data might be missing
    if linkdict['inertial']:
        tagger.write(exportSDFInertial(linkdict['inertial'], tagger.get_indent()))
    else:
        log('No inertial data for "{0}"...'.format(linkdict['name']), "WARNING", "exportSdf")

    # collision data might be missing
    if linkdict['collision']:
        for colkey in linkdict['collision']:
            colliname = linkdict['collision'][colkey]['name']
            collisionobj = bpy.context.scene.objects[colliname]
            tagger.write(exportSDFCollision(collisionobj, linkdict['collision'][colkey],
                                            tagger.get_indent(), modelname))
    else:
        log('No collision data for "{0}"...'.format(linkdict['name']), 'WARNING')

    # there might be no visual objects
    if linkdict['visual']:
        for visualkey in linkdict['visual']:
            visualobj = bpy.context.scene.objects[visualkey]
            visualdata = linkdict['visual'][visualkey]

            # add material information to the visualdata if available
            if 'material' in visualdata:
                material = materials[visualdata['material']]
                visualdata['material'] = material
            tagger.write(exportSDFVisual(visualobj, linkobj, visualdata,
                                         tagger.get_indent(), modelname))
    else:
        log('No visual data for "{0}"...'.format(linkdict['name']), "WARNING")

    # OPT: tagger.write(sensor(linkdict['sensor'], tagger.get_indent()))
    # OPT: tagger.descend('projector', {'name': ...})
    # REQ: tagger.attrib('texture', ...)
    # OPT: tagger.attrib('fov', ...)
    # OPT: tagger.attrib('near_clip', ...)
    # OPT: tagger.attrib('far_clip', ...)
    # OPT: tagger.write(exportSDFFrame('...', tagger.get_indent()))
    # OPT: tagger.write(exportSDFPose('...', tagger.get_indent()))
    # OPT: tagger.descend('plugin', {'name': ..., 'filename': ...})
    # TODO Add plugin element?
    # tagger.ascend()
    # tagger.ascend()
    # OPT: tagger.attrib('audio_sink', ...)
    # OPT: tagger.descend('audio_source')
    # REQ: tagger.attrib('uri', ...)
    # OPT: tagger.attrib('pitch', ...)
    # OPT: tagger.attrib('gain', ...)
    # OPT: tagger.descend('contact')
    # REQ: tagger.attrib('collision', ...)
    # tagger.ascend()
    # OPT: tagger.attrib('loop', ...)
    # OPT: tagger.write(exportSDFFrame('...', tagger.get_indent()))
    # OPT: tagger.write(exportSDFPose('...', tagger.get_indent()))
    # tagger.ascend()
    # OPT: tagger.descend('battery', {'name': ...})
    # REQ: tagger.attrib('voltage', ...)
    # tagger.ascend()
    tagger.ascend()
    return "".join(tagger.get_output())


def exportSDFJoint(jointdict, indentation):
    tagger = xmlTagger(initial=indentation)
    # use sdf joint names instead URDF
    sdftype = jointmapping[jointdict['type']]
    tagger.descend('joint', {'name': jointdict['name'], 'type': sdftype})
    # FINAL remove when all joints are finished
    if sdftype == 'TODO':
        log("joint type '{}' at joint '{}' not supported yet.".format(
            jointdict['type'], jointdict['name']), 'ERROR')
    tagger.attrib('parent', jointdict['parent'])
    tagger.attrib('child', jointdict['child'])
    # OPT: tagger.attrib('gearbox_ratio', ...)
    # OPT: tagger.attrib('gearbox_reference_body', ...)'
    # OPT: tagger.attrib('thread_pitch', ...)'
    if 'axis' in jointdict:
        tagger.descend('axis')
        # axis is defined in local coord space of parent link
        tagger.attrib('xyz', list_to_string(jointdict['axis']))
        # TODO derive pose from model frame optionally
        # tagger.attrib('use_parent_model_frame', '1')
        # OPT: tagger.descend('dynamics')
        # OPT: tagger.attrib('damping', ...)
        # OPT: tagger.attrib('friction', ...)
        # REQ: tagger.attrib('spring_reference', ...)
        # REQ: tagger.attrib('spring_stiffness', ...)
        # tagger.ascend()
        if 'limits' in jointdict:
            tagger.descend('limit')
            # can be omitted for continuous joint
            if 'lower' in jointdict['limits']:
                tagger.attrib('lower', jointdict['limits']['lower'])
            else:
                log("Lower limit is missing for joint '{}'.".format(jointdict['name']), 'WARNING')
                tagger.attrib('lower', '')
            if 'upper' in jointdict['limits'].keys():
                log("Upper limit is missing for joint '{}'.".format(jointdict['name']), 'WARNING')
                tagger.attrib('upper', jointdict['limits']['upper'])
            else:
                tagger.attrib('upper', '')
            if 'effort' in jointdict['limits'].keys():
                tagger.attrib('effort', jointdict['limits']['effort'])
            if 'velocity' in jointdict['limits'].keys():
                tagger.attrib('velocity', jointdict['limits']['velocity'])
            # OPT: tagger.attrib('stiffness', ...)
            # OPT: tagger.attrib('dissipation', ...)
            tagger.ascend()
        tagger.ascend()
    # if 'axis2' in jointdict:
        # OPT: tagger.descend('axis2')
        # REQ: tagger.attrib('xyz', list_to_string(jointdict['axis']))
        # REQ: tagger.attrib('use_parent_model_frame', ...)
        # OPT: tagger.descend('dynamics')
        # OPT: tagger.attrib('damping', ...)
        # OPT: tagger.attrib('friction', ...)
        # REQ: tagger.attrib('spring_reference', ...)
        # REQ: tagger.attrib('spring_stiffness', ...)
        # tagger.ascend()
        # OPT: tagger.descend('limit')
        # OPT: tagger.attrib('lower', jointdict['limits']['lower'])
        # OPT: tagger.attrib('upper', jointdict['limits']['upper'])
        # OPT: tagger.attrib('effort', jointdict['limits']['effort'])
        # OPT: tagger.attrib('velocity', jointdict['limits']['velocity'])
        # OPT: tagger.attrib('stiffness', ...)
        # OPT: tagger.attrib('dissipation', ...)
        # tagger.ascend()
        # tagger.ascend()
    # if 'physics' in jointdict:
        # OPT: tagger.descend('physics')
        # OPT: tagger.descend('simbody')
        # OPT: tagger.attrib('must_be_loop_joint', ...)
        # tagger.ascend()
        #
        # OPT: tagger.descend('ode')
        # OPT: tagger.attrib('cfm_damping', ...)
        # OPT: tagger.attrib('implicit_spring_damper', ...)
        # OPT: tagger.attrib('fudge_factor', ...)
        # OPT: tagger.attrib('cfm', ...)
        # OPT: tagger.attrib('erp', ...)
        # OPT: tagger.attrib('bounce', ...)
        # OPT: tagger.attrib('max_force', ...)
        # OPT: tagger.attrib('velocity', ...)
        #
        # OPT: tagger.descend('limit', ...)
        # REQ: tagger.attrib('cfm', ...)
        # REQ: tagger.attrib('erp', ...)
        # tagger.ascend()
        #
        # OPT: tagger.descend('suspension')
        # REQ: tagger.attrib('cfm', ...)
        # REQ: tagger.attrib('erp', ...)
        # tagger.ascend()
        # tagger.ascend()
        #
        # OPT: tagger.attrib('provide_feedback', ...)
        # tagger.ascend()
    # if 'frame' in jointdict:
        # OPT: tagger.write(exportSDFFrame(jointdict['frame']))
    # We cannot write a different joint pose, as the link is already transformed for the
    # z axis to be rotated properly
    # tagger.write(exportSDFPose(jointdict['pose'], tagger.get_indent()))
    # if 'sensor' in jointdict:
        # OPT: tagger.write(sensor('sensor'))
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
    SubElement(modelconf, 'description').text = model['description']

    return modelconf


def exportSDF(model, filepath):
    """ Export function used for the entity.
    This exports a model SDF file as well as its model.conf to the specified filepath.
    """
    log("Export SDF (version " + sdfversion + ") to " + filepath, "INFO", "exportSdf")
    filename = os.path.join(filepath, model['name'] + '.sdf')

    if getExpSettings().export_sdf_model_config:
        modelconffile = os.path.join(filepath, 'model.config')
    else:
        modelconffile = None
    errors = False

    modelconf = exportGazeboModelConf(model)

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

        # include element
        # OPT: xml.descend('include')
        # REQ: xml.attrib('uri', ...)
        # OPT: xml.attrib('pose', ...)
        # OPT: xml.attrib('name', ...)
        # OPT: xml.attrib('static', ...)
        # xml.ascend()

        # nested model element
        # TODO add wrapper for xml model?
        # OPT: xml.descend('model', params={'name': ...})
        # xml.ascend()

        # enables wind influence on all links in the model (overriden by link
        # wind property)
        # OPT: xml.attrib('enable_wind', ...)

        # frame
        # OPT: xml.descend('frame', {'name': ...})
        # OPT: xml.attrib('pose', poseVal) OR xml.descend('pose') \\
        # xml.attrib('frame', otherFrame) \\ xml.ascend()
        # xml.ascend()

        # pose can be ommitted as we can move the rootlink around
        # OPT: xml.attrib('pose', poseVal) OR xml.descend('pose') \\
        # xml.attrib('frame', otherFrame) \\ xml.ascend()

        # links
        for linkkey in model['links']:
            link = model['links'][linkkey]
            linkobj = link['object']
            xml.write(exportSDFLink(link, linkobj, modelname, model['materials'], xml.get_indent()))
        log('Links exported.', 'DEBUG', 'exportSdf')

        # joints
        for jointkey in model['joints'].keys():
            joint = model['joints'][jointkey]

            xml.write(exportSDFJoint(joint, xml.get_indent()))
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

        if modelconffile:
            log("Writing model.config file to " + modelconffile, "DEBUG")
            with open(modelconffile, 'w') as outputfile:
                outputfile.write(getIndentedETString(modelconf))
            # modelconfTree.write(modelconffile, encoding="UTF-8", xml_declaration=True)

        if getExpSettings().export_sdf_to_gazebo_models:
            phobosprefs = getPhobosPreferences()
            modelpath = os.path.join(phobosprefs.gazebomodelfolder, modelname)
            log("Copying model to Gazebo model folder: {}".format(os.path.relpath(modelpath)), 'INFO')
            if not os.path.exists(modelpath):
                os.makedirs(modelpath)
            else:
                log("Overwriting existing Gazebo model in model folder!", 'WARNING')
            if not os.path.exists(os.path.join(modelpath, 'meshes')):
                os.makedirs(os.path.join(modelpath, 'meshes'))

            log(" Copying SDF file to Gazebo models...", 'DEBUG')
            # copy sdf file
            shutil.copy2(filename, modelpath)

            log(" Copying model.config file to Gazebo models...", 'DEBUG')
            # copy model config file
            if modelconffile:
                shutil.copy2(modelconffile, modelpath)

            log(" Copying meshes to Gazebo models...", 'DEBUG')
            sdfmeshtype = getExpSettings().export_sdf_mesh_type
            meshfolder = os.path.join(os.path.dirname(filepath), 'meshes', sdfmeshtype)
            for file in glob.glob(meshfolder + "/*." + sdfmeshtype):
                meshfilename = os.path.basename(file)
                log("   Copying {} mesh file.".format(meshfilename), 'DEBUG')
                shutil.copy2(file, os.path.join(modelpath, 'meshes', meshfilename))

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
        log("Pose undefined. Defaulting to [0., 0., 0.].", 'WARNING')
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
            geometrydict['filename'] = filepath

            # TODO add submesh support

            # read scale for meshes only
            if geometry[0].find('scale') is not None:
                geometrydict['scale'] = gUtils.parse_text(geometry[0].find('scale').text)
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

    sdfannos = {}
    genericparams = [elem.tag for elem in list(material)
                     if elem.tag not in [
                         'ambient', 'diffuse', 'specular', 'emissive', 'shader', 'script']]
    sdfannos.update({a: gUtils.parse_text(material.find(a).text) for a in genericparams})
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
    genericparams = [elem.tag for elem in list(link) if elem.tag not in [
                         'velocity_decay', 'frame', 'pose', 'inertial', 'collision', 'visual',
                         'sensor', 'projector', 'audio_source', 'battery']]
    newlink = {}
    sdfannos = {}
    sdfannos.update({a: gUtils.parse_text(link.find(a).text) for a in genericparams})

    newlink['name'] = link.attrib['name']
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
            elemsdfannos = {}
            if objtype == 'collision':
                genparams = [generic.tag for generic in list(elem)
                             if generic.tag not in ['pose', 'frame', 'surface', 'geometry']]
                elemsdfannos.update({a: gUtils.parse_text(elem.find(a).text) for a in genparams})
                # TODO implement support for this
                # elemdict['sdf/frame']
                elemdict['pose'] = parseSDFPose(elem.find('pose'))
                # geometry is parsed below
                # TODO implement support
                # elemdict['sdf/surface']
            else:
                genparams = [generic.tag for generic in list(elem) if generic.tag not in [
                    'pose', 'frame', 'geometry', 'meta', 'material', 'plugin']]
                elemsdfannos.update({a: gUtils.parse_text(elem.find(a).text) for a in genparams})
                # TODO implement support for this
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
            elemdict['annotations'] = {'sdf': elemsdfannos}
            objectsdict[name] = elemdict
        newlink[objtype] = objectsdict

    sensors = parseSDFSensors(link.findall('sensor'))

    # TODO add projector, audio sink, audio_source, battery support

    newlink['annotations'] = {'sdf': sdfannos}
    import yaml
    print(yaml.dump(newlink))
    if newlink == {}:
        log("Link information for " + newlink['name'] + " is empty.", 'WARNING')
    return newlink, materials, sensors


def parseSDFJointPhysics(physics):
    # TODO add support for this
    return {}


def parseSDFSensors(sensors):
    sensorsdict = {}

    for sensor in sensors:
        newsensor = {}
        newsensor['name'] = sensor.attrib['name']
        newsensor['type'] = sensor.attrib['type']

        # other params
        # always_on
        # update_rate
        # visualize
        # topic
        # frame
        sensorsdict['pose'] = parseSDFPose(sensor.find('pose'))
        # plugin

        # parse the content of the sensor type to properties for the sensor
        genparams = [elem.tag for elem in list(sensor.find(newsensor['type']))]
        props = {}
        props.update({'sdf/' + a: gUtils.parse_text(sensor.find(newsensor['type']).find(a).text)
                      for a in genparams})
        newsensor['props'] = props

        sensorsettings = defs.def_settings['sensors']
        # find def settings for this sensor type
        for sendef in sensorsettings:
            if sensorsettings[sendef]['type'] == newsensor['type']:
                newsensor['shape'] = sensorsettings[sendef]['shape']
                newsensor['size'] = sensorsettings[sendef]['size']
                break
        else:
            log("Could not find definition settings for {} sensor {}! Defaulting to box.",
                'WARNING')
            newsensor['shape'] = 'box'
            newsensor['size'] = [1., 1., 1.]
        sensorsdict[newsensor['name']] = newsensor
    return sensorsdict


def parseSDFAxis(axis):
    axisdict = {}
    sdfannos = {}

    if 'initial_position' in list(axis):
        sdfannos['initial_position'] = gUtils.parse_text(axis.find('initial_position').text)

    axisdict['xyz'] = gUtils.parse_text(axis.find('xyz').text)

    if axis.find('use_parent_model_frame') is not None:
        axisdict['use_parent_model_frame'] = bool(axis.find('use_parent_model_frame').text)

    if 'dynamics' in list(axis):
        dynamics = axis.find('dynamics')
        axisdict['dynamics']['spring_reference'] = gUtils.parse_number(
            dynamics.find('spring_reference').text)
        axisdict['dynamics']['spring_stiffness'] = gUtils.parse_number(
            dynamics.find('spring_stiffness').text)

        if 'damping' in list(dynamics):
            axisdict['dynamics']['damping'] = gUtils.parse_number(dynamics.find('damping').text)
        if 'friction' in list(dynamics):
            axisdict['dynamics']['friction'] = gUtils.parse_number(dynamics.find('friction').text)

    # parse optional limits
    axisdict['limits'] = {}
    limits = axis.find('limit')
    if limits is not None:
        for opt_limit in ['lower', 'upper', 'effort', 'velocity', 'stiffness', 'dissipation']:
            if opt_limit in list(limits):
                axisdict['limits'][opt_limit] = gUtils.parse_number(limits.find(opt_limit).text)

    axisdict['annotations'] = {'sdf': sdfannos}

    return axisdict


def parseSDFJoint(joint):
    jointdict = {'name': joint.attrib['name'], 'type': joint.attrib['type']}
    jointdict['parent'] = joint.find('parent').text
    jointdict['child'] = joint.find('child').text

    # include all generic parameters not defined in this function
    genparams = [elem.tag for elem in list(joint)
                 if elem.tag not in ['parent', 'child', 'axis', 'axis2', 'physics', 'frame',
                                     'pose', 'sensor']]
    sdfannos = {}
    sdfannos.update({a: gUtils.parse_text(joint.find(a).text) for a in genparams})

    axis = joint.find('axis')
    if axis is not None:
        sdfaxis = parseSDFAxis(axis)
        jointdict['xyz'] = sdfaxis['xyz']
        sdfannos['axis/use_parent_model_frame'] = sdfaxis['use_parent_model_frame']
        jointdict['limits'] = sdfaxis['limits']
        if 'dynamics' in sdfaxis:
            jointdict['dynamics'] = sdfaxis['dynamics']

    # TODO how to handle the second joint axis?
    axis2 = joint.find('axis2')
    if axis2 is not None:
        sdfaxis2 = parseSDFAxis(axis2)

    if 'physics' in list(joint):
        jointdict['physics'] = parseSDFJointPhysics(joint.find('physics'))

    # TODO add frame support

    pose = parseSDFPose(joint.find('pose'))

    sensors = parseSDFSensors(joint.findall('sensor'))
    jointdict['annotations'] = {'sdf': sdfannos}

    # TODO delete me
    import yaml
    print('JOINT:', yaml.dump(jointdict))
    print('POSE:', yaml.dump(pose))
    print('SENSORS:', yaml.dump(sensors))
    return jointdict, pose, sensors


def importSDF(filepath):
    model = {}

    log("Parsing SDF model from " + filepath, 'INFO')

    if not os.path.exists(filepath):
        log("Could not open SDF file. File not found: " + filepath, 'ERROR')
        return {}

    # load element tree from file
    tree = ET.parse(filepath)
    sdfroot = tree.getroot()
    root = sdfroot.find('model')
    model['name'] = root.attrib['name']

    # include all generic parameters not defined in this function
    genparams = [elem.tag for elem in list(sdfroot)
                 if elem.tag not in ['include', 'model', 'frame', 'pose', 'link', 'joint',
                                     'plugin', 'gripper']]
    sdfannos = {}
    sdfannos.update({a: gUtils.parse_text(sdfroot.find(a).text) for a in genparams})
    # TODO add support
    # sdfannos['include']
    # sdfannos['model']
    # sdfannos['frame']
    # sdfannos['pose']

    links = {}
    materials = {}
    sensors = {}
    log("Parsing links...", 'INFO')
    for link in root.iter('link'):
        log(" Adding link {}.".format(link.attrib['name']), 'DEBUG')
        newlink, linkmats, newsensors = parseSDFLink(link, filepath)
        links[link.attrib['name']] = newlink
        materials.update(linkmats)
        sensors.update(newsensors)
    model['links'] = links
    # TODO cleanup duplicate materials

    model['materials'] = materials

    # TODO delete me
    import yaml
    print(yaml.dump(materials))

    joints = {}
    log("Parsing joints...", 'INFO')
    for joint in root.iter('joint'):
        # this is needed as there are "joint" tags e.g. in transmission
        if joint.find('parent') is not None:
            # parse joint from elementtree
            log(" Adding joint {} ...".format(joint.attrib['name']), 'DEBUG')
            newjoint, pose, newsensors = parseSDFJoint(joint)
            model['links'][newjoint['child']]['pose'] = pose
            joints[newjoint['name']] = newjoint

            # add parent-child hierarchy to link information
            parentlink = model['links'][newjoint['parent']]
            childlink = model['links'][newjoint['child']]
            childlink['parent'] = newjoint['parent']
            parentlink['children'].append(newjoint['child'])
            log("   ... and connected parent link {} to {}.".format(
                parentlink['name'], childlink['name']), 'DEBUG')
    model['joints'] = joints
    sensors.update(newsensors)
    model['sensors'] = sensors

    # find any links that still have no pose (most likely because they had no parent)
    for link in model['links']:
        if 'pose' not in model['links'][link]:
            links[link]['pose'] = parseSDFPose(None)

    # TODO include these as model annotations
    # sdfannos['plugin']
    # sdfannos['gripper']

    model['annotations'] = {'sdf': sdfannos}
    return model

# registering export functions of types with Phobos
entity_type_dict = {'sdf': {
    'export': exportSDF,
    'import': importSDF,
    'extensions': ('sdf', 'xml')}}
