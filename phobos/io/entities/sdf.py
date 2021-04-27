#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

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
import phobos.utils.selection as sUtils
import phobos.model.models as models
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
    'planar': 'TODO',
}


class xmlTagger(object):
    """A simple class to create a syntax conform xml file. The line
    indentation space can be customized using the indent parameter.
    To nest xml files an initial indentation layer can be provided.
    The xmlTagger writes an output string using the provided functions and
    takes care of the indentations.

    Args:

    Returns:

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
        """Helper function to return the current indentation depending on the
        hierarchy.
        
        :return: str -- the current indentation (e.g. "  ").

        Args:

        Returns:

        """
        return "" + self.indentation * self.indent

    def ascend(self):
        """Move up one hierarchical layer by finishing the current tag and
        removing one indentation.
        
        :exception IndentationError -- trying to move above root hierarchical
        layer

        Args:

        Returns:

        """
        if self.indentation > self.initial:
            lasttag = self.workingTags.pop(-1)
            self.indentation -= 1
            self.output.append(self.ind() + "</" + lasttag + ">\n")
        else:
            IndentationError()

    def descend(self, tag, attribs=None):
        """Move down one hierarchical layer using the new tag.
        Optional in-line attributes can be provided in the
        dictionary attribs (e.g. {'name': 'foo'}.

        Args:
          tag(str): tag to descend with
          attribs: (Default value = None)

        Returns:
          None: None

        """
        self.workingTags.append(str(tag))
        line = ""

        # create parameter strings by unpacking dictionary
        if attribs:
            parameters = [key + '="' + str(attribs[key]) + '" ' for key in attribs.keys()]
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
        """Write a custom line to the output. Use to create the header or comments.

        Args:
          text(str): The line to write (line break has to be included)

        Returns:

        """
        self.output.append(text)

    def attrib(self, tag, value):
        """Adds an attribute to the current element. The tag of the attribute
        is wrapped around its value.

        Args:
          tag(str): The tag of the attribute.
          value(str (will be casted anyway): The value of the attribute

        Returns:

        """
        self.output.append(self.ind() + '<' + str(tag) + '>' + str(value) + '</' + tag + '>\n')

    def get_indent(self):
        """TODO Missing documentation"""
        return self.indentation

    def get_output(self):
        """Completes all trailing tags until at initial indentation and
        returns the output as string.
        
        :return: str -- the finished xml string.

        Args:

        Returns:

        """
        # ascend to base layer
        while self.indentation > self.initial:
            self.ascend()

        return self.output


def getIndentedETString(elementtree):
    """Return the specified elementtree as an indented string which can be saved to a file.
    
    The indentation is based on the phobos.utils.io.indent variable.

    Args:
      elementtree: the elementtree to be converted to string

    Returns:
      string representation of the elementtree

    """
    return minidom.parseString(ET.tostring(elementtree)).toprettyxml(indent=phobosindentation)


def exportSDFPose(relativepose, indentation, poseobject=None, relative=False):
    """Simple wrapper for pose data.
    If relative poses are used the data found in posedata is used.
    Otherwise the pose of the poseobject will be combined with all collected
    links up to the rootobject (see phobos.utils.editing.getCombinedTransform).

    Args:
      relativepose(dict): posedata of the object
      poseobject(bpy.obj, optional): object to be used for absolute pose (Default value = None)
      indentation(int): indentation at current level
      relative(bool, optional): True for usage of sdf relative pathing (Default value = False)

    Returns:
      : str -- writable xml line

    """
    tagger = xmlTagger(initial=indentation)

    # relative poses are written to file as they are
    if not poseobject:
        if relativepose:
            posedata = relativepose
        else:
            posedata = {'translation': [0., 0., 0.], 'rotation_euler': [0., 0., 0.]}
    # world poses are created by combined transform of the pose
    else:
        if relative:
            matrix = poseobject.matrix_local
        else:
            matrix = poseobject.matrix_world
        # FINAL remove when done
        # matrix = getCombinedTransform(poseobject, getRoot(poseobject))
        posedata = {
            'rawmatrix': matrix,
            'matrix': [list(vector) for vector in list(matrix)],
            'translation': list(matrix.to_translation()),
            'rotation_euler': list(matrix.to_euler()),
            'rotation_quaternion': list(matrix.to_quaternion()),
        }
        posedata = gUtils.roundFloatsInDict(posedata, getExpSettings().decimalPlaces)

    # only translation and euler rotation are required
    tra = posedata['translation']
    rot = posedata['rotation_euler']
    result = '{0} {1} {2} {3} {4} {5}'.format(tra[0], tra[1], tra[2], rot[0], rot[1], rot[2])
    tagger.attrib('pose', result)
    return "".join(tagger.get_output())


def exportSDFFrame(framedata, indentation, relative):
    """Simple wrapper for frame data.
    The name of the frameobject (has to be a key in framedata) is used to
    define the object pose.

    Args:
      framedata: data as provided by dictionary
      indentation: indentation at current level
      relative: True to apply a pose to the frame object itself (NOT
    SUPPORTED YET)

    Returns:
      : str -- writable xml line

    """
    tagger = xmlTagger(initial=indentation)
    tagger.descend('frame', {'name': framedata['name']})
    # relative frame pose is not supported yet
    # tagger.write(exportSDFPose(framedata['pose'], tagger.get_indent(),
    # relative))
    tagger.ascend()
    return "".join(tagger.get_output())


def exportSDFInertial(inertialdata, indentation):
    """Simple wrapper for link inertial data.
    The inertial object is required to determine the position (pose) of the
    object.
    If relative poses are used the data found in inertialdata is used.
    Otherwise the pose of the inertialobject will be combined with all
    collected links up to the rootobject (see
    phobos.utils.editing.getCombinedTransform).

    Args:
      inertialobj(Blender object.): object to be used for absolute pose
      inertialdata(dict.): data as provided by dictionary (should contain mass
    and inertia)
      indentation(int.): indentation at current level

    Returns:
      : str -- writable xml line

    """
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
    """Simple wrapper for link collision data.
    The collision object is required to determine the position (pose) of the
    object.
    If relative poses are used the data found in collisiondata is used.
    Otherwise the pose of the collisionobject will be combined with all
    collected links up to the rootobject (see
    phobos.utils.editing.getCombinedTransform).

    Args:
      collisionobj: object to be used for absolute pose
      collisiondata: data as provided by dictionary (should contain name,
    geometry, [bitmask])
      indentation: indentation at current level
      relative: True for usage of sdf relative pathing
      modelname: the name of the model (required for geometry)

    Returns:
      : str -- writable xml line

    """
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
    """Simple wrapper for geometry data of link collisions.

    Args:
      geometrydata: data as provided by dictionary
      indentation: indentation at current level
      modelname: name used for gazebo model pathing

    Returns:
      : str -- writable xml line

    """
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
        tagger.attrib(
            'uri',
            'model://'
            + modelname
            + '/meshes/'
            + geometrydata['filename']
            + '.{0}'.format(meshtype),
        )
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
    """Simple wrapper for visual data of links.
    The visual object is required to determine the position (pose) of the
    object.
    If relative poses are used the data found in visualdata (key pose) is used.
    Otherwise the pose of the visual object will be combined with all
    collected links up to the rootobject (see
    phobos.utils.editing.getCombinedTransform).

    Args:
      visualobj: object to be used for pose
      visualdata: data as provided by dictionary (should contain name,
    geometry)
      indentation: indentation at current level
      relative: True for usage of sdf relative pathing
      modelname: the name of the model (required for geometry)
      linkobj: 

    Returns:
      : str -- writable xml line

    """
    tagger = xmlTagger(initial=indentation)
    tagger.descend('visual', attribs={'name': visualdata['name']})
    # OPT: tagger.attrib('cast_shadows', ...)
    # OPT: tagger.attrib('laser_retro', ...)
    # OPT: tagger.attrib('transparency', ...)
    # OPT: tagger.descend('meta')
    # OPT: tagger.attrib('layer', ...)
    # tagger.ascend()
    # OPT: tagger.write(exportSDFFrame(..., tagger.get_indent()))

    # Pose data of the visual is transformed by link --> use local matrix
    matrix = visualobj.matrix_local
    posedata = {
        'rawmatrix': matrix,
        'matrix': [list(vector) for vector in list(matrix)],
        'translation': list(matrix.to_translation()),
        'rotation_euler': list(matrix.to_euler()),
        'rotation_quaternion': list(matrix.to_quaternion()),
    }
    # overwrite absolute position of the visual object
    tagger.write(exportSDFPose(posedata, tagger.get_indent()))

    # write material data if available
    if 'material' in visualdata:
        tagger.write(exportSDFMaterial(visualdata['material'], tagger.get_indent()))

    tagger.write(exportSDFGeometry(visualdata['geometry'], tagger.get_indent(), modelname))
    tagger.ascend()
    return "".join(tagger.get_output())


def exportSDFMaterial(materialdata, indentation):
    """Simple wrapper for material data of visual objects.
    The materialdata is the model dictionary of the specific material.

    Args:
      materialdata: the material information
      visualdata: data as provided by dictionary (should contain
    diffuseColor, specularColor etc)
      indentation: indentation at current level

    Returns:
      : str -- writable xml line

    """
    tagger = xmlTagger(initial=indentation)
    tagger.descend('material')
    alpha = materialdata['transparency'] if 'transparency' in materialdata else '1.0'

    # OPT: tagger.descend('plugin', ...)
    # OPT: tagger.descend('shader', ...)
    # OPT: tagger.attrib('lighting', ...)

    ambient = materialdata['ambientColor']
    tagger.attrib(
        'ambient', '{0} {1} {2} {3}'.format(ambient['r'], ambient['g'], ambient['g'], alpha)
    )

    diffuse = materialdata['diffuseColor']
    tagger.attrib(
        'diffuse', '{0} {1} {2} {3}'.format(diffuse['r'], diffuse['g'], diffuse['b'], alpha)
    )

    specular = materialdata['specularColor']
    tagger.attrib(
        'specular', '{0} {1} {2} {3}'.format(specular['r'], specular['g'], specular['b'], 1.0)
    )

    if 'emissionColor' in materialdata:
        emission = materialdata['emissionColor']
        tagger.attrib(
            'emissive', '{0} {1} {2} {3}'.format(emission['r'], emission['g'], emission['b'], 1.0)
        )
    tagger.ascend()
    return "".join(tagger.get_output())


def exportSDFLink(linkdict, linkobj, modelname, materials, sensors, indentation):
    """

    Args:
      linkdict: 
      linkobj: 
      modelname: 
      materials: 
      sensors: 
      indentation: 

    Returns:

    """
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
        log("No inertial data for '{0}'...".format(linkdict['name']), 'WARNING')

    # collision data might be missing
    if linkdict['collision']:
        for colkey in linkdict['collision']:
            collisionobj = sUtils.getObjectByName(colkey, phobostypes=('collision'))
            tagger.write(
                exportSDFCollision(
                    collisionobj, linkdict['collision'][colkey], tagger.get_indent(), modelname
                )
            )
    else:
        log("No collision data for '{0}'...".format(linkdict['name']), 'WARNING')

    # there might be no visual objects
    if linkdict['visual']:
        for visualkey in linkdict['visual']:

            # TODO absolute pose export is not working
            if visualkey in bpy.context.scene.objects:
                visualobj = bpy.context.scene.objects[visualkey]
            else:
                visualobj = None
            visualobj = sUtils.getObjectByName(visualkey, phobostypes=('visual'))
            visualdata = linkdict['visual'][visualkey]

            # add material information to the visualdata if available
            if 'material' in visualdata:
                material = materials[visualdata['material']]
                visualdata['material'] = material
            tagger.write(
                exportSDFVisual(visualobj, linkobj, visualdata, tagger.get_indent(), modelname)
            )
    else:
        log("No visual data for '{0}'...".format(linkdict['name']), 'WARNING')

    if sensors:
        for sensor in sensors:
            tagger.write(exportSDFSensor(sensor, tagger.get_indent()))
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
    """

    Args:
      jointdict: 
      indentation: 

    Returns:

    """
    tagger = xmlTagger(initial=indentation)
    # use sdf joint names instead URDF
    sdftype = jointmapping[jointdict['type']]
    tagger.descend('joint', {'name': jointdict['name'], 'type': sdftype})
    # FINAL remove when all joints are finished
    if sdftype == 'TODO':
        log(
            "Joint type '{}' at joint '{}' not supported yet.".format(
                jointdict['type'], jointdict['name']
            ),
            'ERROR',
        )
    tagger.attrib('parent', jointdict['parent'])
    tagger.attrib('child', jointdict['child'])
    # We know that the joints in phobos are equal to links -> use zeros
    tagger.write(exportSDFPose(None, indentation + 1))

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


def exportSDFSensor(sensordict, indentation):
    """

    Args:
      sensordict: 
      indentation: 

    Returns:

    """
    tagger = xmlTagger(initial=indentation)

    tagger.descend('sensor', {'name': sensordict['name'], 'type': sensordict['type']})

    if sensordict['type'] not in [
        'altimeter',
        'camera',
        'contact',
        'gps',
        'imu',
        'logical_camera',
        'magnetometer',
        'ray',
        'rfidtag',
        'rfid',
        'sonar',
        'transceiver',
        'force_torque',
    ]:
        log(
            "Sensor type not supported by SDF '{}'! Skipping sensor {}...".format(
                sensordict['type'], sensordict['name']
            )
        )
        return ''

    # write generic params
    for generic in ['always_on', 'update_rate', 'visualize', 'topic']:
        if generic in sensordict:
            tagger.attrib(generic, sensordict[generic])

    tagger.write(exportSDFPose(sensordict['pose'], tagger.get_indent()))

    # TODO add plugin and frame support

    if sensordict['type'] == 'altimeter':
        altimeter = sensordict
        tagger.descend('altimeter')
        if 'vertical_position' in altimeter and 'noise' in altimeter['vertical_position']:
            tagger.descend('vertical_position')
            tagger.descend('noise', {'type': altimeter['vertical_position']['noise']['type']})
            for elem in [key for key in altimeter['vertical_position']['noise'] if key != 'type']:
                tagger.attrib(elem, altimeter['vertical_position']['noise'][elem])
            tagger.ascend()
            tagger.ascend()

        if 'vertical_velocity' in altimeter and 'noise' in altimeter['vertical_velocity']:
            tagger.descend('vertical_velocity')
            tagger.descend('noise', {'type': altimeter['vertical_velocity']['noise']['type']})
            for elem in [key for key in altimeter['vertical_velocity']['noise'] if key != 'type']:
                tagger.attrib(elem, altimeter['vertical_velocity']['noise'][elem])
            tagger.ascend()
            tagger.ascend()
        tagger.ascend()

    elif sensordict['type'] == 'camera':
        cam = sensordict
        tagger.descend('camera', {'name': cam['name']} if 'name' in cam else {})
        tagger.attrib('horizontal_fov', cam['horizontal_fov'])

        tagger.descend('image')
        tagger.attrib('width', cam['width'])
        tagger.attrib('height', cam['height'])
        if 'image' in cam and 'format' in cam['image']:
            tagger.attrib('format', cam['image']['format'])
        tagger.ascend()

        tagger.descend('clip')
        tagger.attrib('near', cam['clip']['near'])
        tagger.attrib('far', cam['clip']['far'])
        tagger.ascend()

        if 'save' in cam:
            tagger.descend('save', {'enabled': cam['save']['enabled']})
            tagger.attrib('path', cam['save']['path'])
            tagger.ascend()

        if 'depth_cam' in cam:
            tagger.descend('depth_cam')
            tagger.attrib('output', cam['output'])
            tagger.ascend()

        if 'noise' in cam:
            tagger.descend('noise')
            tagger.attrib('type', cam['noise']['type'])
            if 'mean' in cam['noise']:
                tagger.attrib('mean', cam['noise']['mean'])
            if 'stddev' in cam['noise']:
                tagger.attrib('stddev', cam['noise']['stddev'])
            tagger.ascend()

        if 'distortion' in cam:
            tagger.descend('distortion')
            params = ['k1', 'k2', 'k3', 'p1', 'p2', 'center']
            for distortparam in params:
                if distortparam in cam['distortion']:
                    tagger.attrib(distortparam, cam['distortion'][distortparam])
            tagger.ascend()

        if 'lens' in cam:
            tagger.descend('lens')
            tagger.attrib('type', cam['lens']['type'])
            tagger.attrib('scale_to_hfov', cam['lens']['scale_to_hfov'])

            if 'custom_function' in cam['lens']:
                tagger.descend('custom_function')
                tagger.attrib('fun', cam['lens']['custom_function']['fun'])
                for par in ['c1', 'c2', 'c3', 'f']:
                    if par in cam['lens']['custom_function']:
                        tagger.attrib(par, cam['lens']['custom_function'][par])
                tagger.ascend()
            if 'cutoff_angle' in cam['lens']:
                tagger.attrib('cutoff_angle', cam['lens']['cutoff_angle'])
            if 'env_texture_size' in cam['lens']:
                tagger.attrib('env_texture_size', cam['lens']['env_texture_size'])
        # TODO why do the camera settings override pose and frame again?
        # sensor pose and frame should be enough for that... -> otherwise add pose and frame here
        tagger.ascend()

    elif sensordict['type'] == 'contact':
        tagger.descend('contact')
        # sdf contact supports only a single element, not a list
        sensordict = models.replace_object_links(sensordict)
        tagger.attrib('collision', sensordict['collision'][0])
        tagger.attrib('topic', sensordict['topic'])
        tagger.ascend()

    elif sensordict['type'] == 'gps':
        gps = sensordict
        tagger.descend('gps')
        for sensmode in ['position_sensing', 'velocity_sensing']:
            if sensmode in gps:
                tagger.descend(sensmode)
                for direction in ['horizontal', 'vertical']:
                    if direction in gps[sensmode]:
                        tagger.descend(direction)
                        tagger.descend('noise', {'type': gps[sensmode][direction]['noise']['type']})
                        for val in ['mean', 'stddev', 'bias_mean', 'bias_stddev', 'precision']:
                            if val in gps[sensmode][direction]['noise']:
                                tagger.attrib(val, gps[sensmode][direction]['noise'][val])
                        tagger.ascend()
                        tagger.ascend()
                tagger.ascend()
        tagger.ascend()

    elif sensordict['type'] == 'imu':
        imu = sensordict
        tagger.descend('imu')
        if 'orientation_reference_frame' in imu:
            tagger.descend('orientation_reference_frame')
            tagger.attrib('localization', imu['orientation_reference_frame']['localization'])
            if 'custom_rpy' in imu['orientation_reference_frame']:
                tagger.attrib(
                    'custom_rpy',
                    '{} {} {}'.format(
                        *[i for i in imu['orientation_reference_frame']['custom_rpy']]
                    ),
                )
                # TODO add support for parent_frame
            if 'grav_dir_x' in imu['orientation_reference_frame']:
                tagger.attrib(
                    'grav_dir_x',
                    '{} {} {}'.format(
                        *[i for i in imu['orientation_reference_frame']['grav_dir_x']]
                    ),
                )
                # TODO add support for parent_frame
            tagger.ascend()
        if 'topic' in imu:
            tagger.attrib('topic', imu['topic'])

        for mode in ['angular_velocity', 'linear_acceleration']:
            if mode in imu:
                tagger.descend(mode)
                for direction in ['x', 'y', 'z']:
                    if direction in imu[mode]:
                        tagger.descend(direction)
                        tagger.descend('noise', {'type': imu[mode][direction]['noise']['type']})
                        for val in ['mean', 'sttdev', 'bias_mean', 'bias_sttdev', 'precision']:
                            if val in imu[mode][direction]['noise']:
                                tagger.attrib(val, imu[mode][direction]['noise'][val])
                        tagger.ascend()
                        tagger.ascend()
                tagger.ascend()
        tagger.ascend()

    elif sensordict['type'] == 'logical_camera':
        logcam = sensordict
        tagger.descend('logical_camera')
        tagger.attrib('near', logcam['near'])
        tagger.attrib('far', logcam['far'])
        tagger.attrib('aspect_ratio', logcam['aspect_ratio'])
        tagger.attrib('horizontal_fov', logcam['horizontal_fov'])

    elif sensordict['type'] == 'magnetometer':
        magn = sensordict
        tagger.descend('magnetometer')
        for direction in ['x', 'y', 'z']:
            if direction in magn:
                tagger.descend(direction)
                tagger.descend('noise', {'type': magn[direction]['noise']['type']})
                for val in ['mean', 'sttdev', 'bias_mean', 'bias_sttdev', 'precision']:
                    if val in magn[direction]['noise']:
                        tagger.attrib(val, magn[direction]['noise'][val])
                tagger.ascend()
                tagger.ascend()
        tagger.ascend()

    elif sensordict['type'] == 'ray':
        ray = sensordict
        tagger.descend('ray')

        tagger.descend('scan')
        for direction in ['horizontal', 'vertical']:
            tagger.descend(direction)
            tagger.attrib('samples', ray['scan'][direction]['samples'])
            tagger.attrib('resolution', ray['scan'][direction]['resolution'])
            tagger.attrib('min_angle', ray['scan'][direction]['min_angle'])
            tagger.attrib('max_angle', ray['scan'][direction]['max_angle'])
            tagger.ascend()
        tagger.ascend()

        tagger.descend('range')
        tagger.attrib('min', ray['min_distance'])
        tagger.attrib('max', ray['max_distance'])
        if 'range' in ray and 'resolution' in ray['range']:
            tagger.attrib('resolution', ray['range']['resolution'])
        tagger.ascend()

        if 'noise' in ray:
            tagger.descend('noise')
            tagger.attrib('type', ray['noise']['type'])
            if 'mean' in ray['noise']:
                tagger.attrib('mean', ray['noise']['mean'])
            if 'stddev' in ray['noise']:
                tagger.attrib('stddev', ray['noise']['stddev'])
            tagger.ascend()
        tagger.ascend()

    elif sensordict['type'] == 'rfidtag':
        tagger.attrib('rfidtag', '')

    elif sensordict['type'] == 'rfid':
        tagger.attrib('rfid', '')

    elif sensordict['type'] == 'sonar':
        sonar = sensordict
        tagger.descend('sonar')
        tagger.attrib('min', sonar['min_dist'])
        tagger.attrib('max', sonar['max_dist'])
        tagger.attrib('radius', sonar['radius'])
        tagger.ascend()

    elif sensordict['type'] == 'transceiver':
        transc = sensordict
        tagger.descend('transceiver')
        # required first
        tagger.attrib('gain', transc['gain'])
        tagger.attrib('power', transc['power'])

        for opt in ['essid', 'frequency', 'min_frequency', 'max_frequency', 'sensitivity']:
            if opt in transc:
                tagger.attrib(opt, transc[opt])
        tagger.ascend()

    elif sensordict['type'] == 'force_torque':
        fortor = sensordict
        tagger.descend('force_torque')
        # TODO add frame support
        if 'measure_direction' in fortor:
            tagger.attrib('measure_direction', fortor['measure_direction'])
        tagger.ascend()
    tagger.ascend()
    return "".join(tagger.get_output())


def exportGazeboModelConf(model):
    """Creates a model.config element from the specified information.

    Args:
      model: the model dictionary of the phobos model

    Returns:
      : xml.etree.ElementTree.Element of the model

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
    """Export function used for the entity.
    This exports a model SDF file as well as its model.conf to the specified filepath.

    Args:
      model: 
      filepath: 

    Returns:

    """
    log("Export SDF (version {}) to {}.".format(sdfversion, filepath), 'INFO')
    filename = os.path.join(filepath, model['name'] + '.sdf')

    if getExpSettings().export_sdf_model_config:
        modelconffile = os.path.join(filepath, 'model.config')
    else:
        modelconffile = None
    errors = False

    annotationdict = models.gatherAnnotations(model)
    if 'sdf' in annotationdict:
        for category in annotationdict['sdf']:
            for list_obj in annotationdict['sdf'][category]:
                model[category + 's'][list_obj['name']].update(list_obj)
        del annotationdict['sdf']

    modelconf = exportGazeboModelConf(model)

    log("Exporting '{0}'...".format(model['name']), "DEBUG")
    # FINAL remove debugging information
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

        # xml.descend('world', attribs={'name': 'default'})
        # xml.descend('include')
        # xml.attrib('uri', 'model://ground_plane')
        # xml.ascend()
        # xml.descend('include')
        # xml.attrib('uri', 'model://sun')
        # xml.ascend()
        # model layer
        modelname = model['name']
        xml.descend('model', attribs={"name": modelname})

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
        # OPT: xml.descend('model', attribs={'name': ...})
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
            sensors = []
            for sens in model['sensors']:
                if model['sensors'][sens]['link'] == linkkey:
                    sensors.append(model['sensors'][sens])
            xml.write(
                exportSDFLink(
                    link, linkobj, modelname, model['materials'], sensors, xml.get_indent()
                )
            )
        log("Links exported.", 'DEBUG')

        # joints
        for jointkey in model['joints'].keys():
            joint = model['joints'][jointkey]

            xml.write(exportSDFJoint(joint, xml.get_indent()))
        log("Joints exported.", 'DEBUG')

        # plugin
        # OPT: xml.descend('plugin', attribs={'name': ..., 'filename': ...)
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
        log("Error in export!", 'ERROR')
    finally:
        outputtext = xml.get_output()

        log("Writing model sdf file to {}.".format(filename), 'DEBUG')
        with open(filename, 'w') as outputfile:
            outputfile.writelines(outputtext)

        if modelconffile:
            log("Writing model.config file to {}.".format(modelconffile), 'DEBUG')
            with open(modelconffile, 'w') as outputfile:
                outputfile.write(getIndentedETString(modelconf))
            # modelconfTree.write(modelconffile, encoding="UTF-8", xml_declaration=True)

        if getExpSettings().export_sdf_to_gazebo_models:
            phobosprefs = getPhobosPreferences()
            modelpath = os.path.join(phobosprefs.gazebomodelfolder, modelname)
            log(
                "Copying model to Gazebo model folder: {}".format(os.path.relpath(modelpath)),
                'INFO',
            )
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

    finishmessage = "Export finished with " + ("no " if not errors else "") + "errors."
    log(finishmessage, 'INFO')


def parseSDFPose(pose):
    """

    Args:
      pose: 

    Returns:

    """

    posedict = {}
    if pose is not None:
        if 'frame' in pose.attrib:
            posedict['parentframe'] = pose.attrib['frame']
        xyzrpy = gUtils.parse_text(pose.text)
        posedict['translation'] = xyzrpy[:3]
        posedict['rotation_euler'] = xyzrpy[3:]
    else:
        log("Pose undefined. Defaulting to [0., 0., 0.].", 'WARNING')
        posedict['translation'] = [0.0, 0.0, 0.0]
        posedict['rotation_euler'] = [0.0, 0.0, 0.0]
    return posedict


def parseSDFInertial(link):
    """

    Args:
      link: 

    Returns:

    """
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
            inertial_dict['inertia'] = [
                float(elem.text) for elem in sorted(list(inertia), key=lambda el: el.tag)
            ]
        inertial_dict['name'] = 'inertial_' + link.attrib['name']

        # TODO delete me
        import json

        print(json.dumps(inertial_dict))
        return inertial_dict

        # TODO add frame support
    else:
        log("   No inertial defined for link {}.".format(link.attrib['name']), 'DEBUG')
        return None


def parseSDFGeometry(geometry, link, sdfpath):
    """

    Args:
      geometry: 
      link: 
      sdfpath: 

    Returns:

    """
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
            log(
                "     Mesh file does not exist: {} Replaced mesh with simple box.".format(filepath),
                'WARNING',
            )
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
    import json

    print(json.dumps(geometrydict))
    return geometrydict


def parseSDFMaterial(visualname, material):
    """

    Args:
      visualname: 
      material: 

    Returns:

    """
    materialdict = {}

    # sdf has no material names, so we initialize with visual name
    materialdict['name'] = 'mat_' + visualname

    sdfannos = {}
    genericparams = [
        elem.tag
        for elem in list(material)
        if elem.tag not in ['ambient', 'diffuse', 'specular', 'emissive', 'shader', 'script']
    ]
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
    import json

    print(json.dumps(materialdict))
    return materialdict


def parseSDFLink(link, filepath):
    """

    Args:
      link: 
      filepath: 

    Returns:

    """
    # collect all parameters which can be parsed as generic sdf annotations
    genericparams = [
        elem.tag
        for elem in list(link)
        if elem.tag
        not in [
            'velocity_decay',
            'frame',
            'pose',
            'inertial',
            'collision',
            'visual',
            'sensor',
            'projector',
            'audio_source',
            'battery',
        ]
    ]
    newlink = {}
    sdfannos = {}
    sdfannos.update({a: gUtils.parse_text(link.find(a).text) for a in genericparams})

    newlink['name'] = link.attrib['name']
    newlink['children'] = []

    # TODO add support for other parameters
    # velocity_decay
    # frame

    # We need to reuse this, since the pose in sdf is relative
    # The pose of all children of the link has to be corrected if they are links
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
                log(
                    "   No name for {} object! Assigning {} instead.".format(objtype, name),
                    'WARNING',
                )
                i += 1
            else:
                name = elem.attrib['name']
            log("     {} element {}:".format(objtype[0].upper() + objtype[1:], name), 'DEBUG')

            elemdict = {'name': name}
            elemsdfannos = {}
            if objtype == 'collision':
                genparams = [
                    generic.tag
                    for generic in list(elem)
                    if generic.tag not in ['pose', 'frame', 'surface', 'geometry']
                ]
                elemsdfannos.update({a: gUtils.parse_text(elem.find(a).text) for a in genparams})
                # TODO implement support for this
                # elemdict['sdf/frame']
                elemdict['pose'] = parseSDFPose(elem.find('pose'))
                # geometry is parsed below
                # TODO implement support
                # elemdict['sdf/surface']
            else:
                genparams = [
                    generic.tag
                    for generic in list(elem)
                    if generic.tag
                    not in ['pose', 'frame', 'geometry', 'meta', 'material', 'plugin']
                ]
                elemsdfannos.update({a: gUtils.parse_text(elem.find(a).text) for a in genparams})
                # TODO implement support for this
                # elemdict['sdf/meta']
                # elemdict['sdf/frame']
                elemdict['pose'] = parseSDFPose(elem.find('pose'))
                # geometry is parsed below

                # undefined materials will be set to phobos_error
                if elem.find('material') is None:
                    log(
                        (
                            "       No material defined for {} {} in link {}! Defaulting "
                            + "to error material."
                        ).format(objtype, name, newlink['name']),
                        'ERROR',
                    )
                    elemdict['material'] = 'phobos_error'
                # defined materials are collected in dictionary and linked with name
                else:
                    newmat = parseSDFMaterial(name, elem.find('material'))
                    elemdict['material'] = newmat['name']
                    materials[newmat['name']] = newmat
                # TODO implement support for this
                # elemdict['sdf/plugin']

            if elem.find('geometry') is None:
                log(
                    "   No geometry defined for {} {} in link {}! Skipped..".format(
                        objtype, name, newlink['name']
                    ),
                    'ERROR',
                )
                continue
            elemdict['geometry'] = parseSDFGeometry(elem.find('geometry'), link, filepath)
            elemdict['annotations'] = {'sdf': elemsdfannos}
            objectsdict[name] = elemdict
        newlink[objtype] = objectsdict

    sensors = parseSDFSensors(link.findall('sensor'))

    # TODO add projector, audio sink, audio_source, battery support

    newlink['annotations'] = {'sdf': sdfannos}
    # TODO delete me
    import json

    print(json.dumps(newlink))
    if newlink == {}:
        log("Link information for " + newlink['name'] + " is empty.", 'WARNING')
    return newlink, materials, sensors


def parseSDFJointPhysics(physics):
    """

    Args:
      physics: 

    Returns:

    """
    # TODO add support for this
    return {}


def parseSDFSensors(sensors):
    """

    Args:
      sensors: 

    Returns:

    """
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
        props.update(
            {
                'sdf/' + a: gUtils.parse_text(sensor.find(newsensor['type']).find(a).text)
                for a in genparams
            }
        )
        newsensor['props'] = props

        sensorsettings = defs.def_settings['sensors']
        # find def settings for this sensor type
        for sendef in sensorsettings:
            if sensorsettings[sendef]['type'] == newsensor['type']:
                newsensor['shape'] = sensorsettings[sendef]['shape']
                newsensor['size'] = sensorsettings[sendef]['size']
                break
        else:
            log(
                "Could not find definition settings for {} sensor {}! Defaulting to box.", 'WARNING'
            )
            newsensor['shape'] = 'box'
            newsensor['size'] = [1., 1., 1.]
        sensorsdict[newsensor['name']] = newsensor
    return sensorsdict


def parseSDFAxis(axis):
    """

    Args:
      axis: 

    Returns:

    """
    axisdict = {}
    sdfannos = {}

    if 'initial_position' in list(axis):
        sdfannos['initial_position'] = gUtils.parse_text(axis.find('initial_position').text)

    axisdict['xyz'] = gUtils.parse_text(axis.find('xyz').text)

    if axis.find('use_parent_model_frame') is not None:
        axisdict['use_parent_model_frame'] = bool(axis.find('use_parent_model_frame').text)
    else:
        axisdict['use_parent_model_frame'] = False

    if 'dynamics' in list(axis):
        dynamics = axis.find('dynamics')
        axisdict['dynamics']['spring_reference'] = gUtils.parse_number(
            dynamics.find('spring_reference').text
        )
        axisdict['dynamics']['spring_stiffness'] = gUtils.parse_number(
            dynamics.find('spring_stiffness').text
        )

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
    """

    Args:
      joint: 

    Returns:

    """
    jointdict = {'name': joint.attrib['name'], 'type': joint.attrib['type']}
    jointdict['parent'] = joint.find('parent').text
    jointdict['child'] = joint.find('child').text

    # include all generic parameters not defined in this function
    genparams = [
        elem.tag
        for elem in list(joint)
        if elem.tag
        not in ['parent', 'child', 'axis', 'axis2', 'physics', 'frame', 'pose', 'sensor']
    ]
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
    import json

    print('JOINT:', json.dumps(jointdict))
    print('POSE:', json.dumps(pose))
    print('SENSORS:', json.dumps(sensors))
    return jointdict, pose, sensors


def importSDF(filepath):
    """

    Args:
      filepath: 

    Returns:

    """
    from numpy import pi, sign

    model = {}

    log("Parsing SDF model from " + filepath, 'INFO')

    if not os.path.exists(filepath):
        log("Could not open SDF file. File not found: " + filepath, 'ERROR')
        return {}

    # load element tree from file
    tree = ET.parse(filepath)
    sdfroot = tree.getroot()

    # Check for nested sdf
    if sdfroot.find('world'):
        root = sdfroot.find('world')
    else:
        root = sdfroot.find('model')
    if root.attrib['name']:
        model['name'] = root.attrib['name']
    else:
        model['name'] = 'SDFImport'

    # include all generic parameters not defined in this function
    genparams = [
        elem.tag
        for elem in list(sdfroot)
        if elem.tag
        not in ['include', 'model', 'frame', 'pose', 'link', 'joint', 'plugin', 'gripper']
    ]
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
    import json

    print(json.dumps(materials))

    joints = {}
    log("Parsing joints...", 'INFO')
    for joint in root.iter('joint'):
        # this is needed as there are "joint" tags e.g. in transmission
        if joint.find('parent') is not None:
            # parse joint from elementtree
            log(" Adding joint {} ...".format(joint.attrib['name']), 'DEBUG')
            newjoint, pose, newsensors = parseSDFJoint(joint)

            joints[newjoint['name']] = newjoint

            # add parent-child hierarchy to link information
            parentlink = model['links'][newjoint['parent']]
            childlink = model['links'][newjoint['child']]

            childlink['parent'] = newjoint['parent']
            parentlink['children'].append(newjoint['child'])

            log(
                "   ... and connected parent link {} to {}.".format(
                    parentlink['name'], childlink['name']
                ),
                'DEBUG',
            )

            # Correct the pose
            childlink['pose']['translation'] = [
                y - x
                for x, y in zip(parentlink['pose']['translation'], childlink['pose']['translation'])
            ]
            # Correct the angle
            if 'xyz' in newjoint and abs(newjoint['xyz'][-1]) != 1:
                print(newjoint['xyz'])
                if abs(newjoint['xyz'][0]) == 1:
                    childlink['pose']['rotation_euler'] = [0., sign(newjoint['xyz'][0]) * pi, 0.]
                elif abs(newjoint['xyz'][1]) == 1:
                    childlink['pose']['rotation_euler'] = [sign(newjoint['xyz'][1]) * pi, 0., 0.]
                else:
                    log(
                        "   Parsing of connection from link {} to {} not properly supported right now.".format(
                            parentlink['name'], childlink['name']
                        ),
                        'WARNING',
                    )
    model['joints'] = joints

    # TODO FIXME This does not work, since the parent is not given
    # sensors.update(newsensors)
    # model['sensors'] = sensors

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
entity_type_dict = {'sdf': {'export': exportSDF, 'import': importSDF, 'extensions': ('sdf', 'xml')}}
