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

import bpy
import mathutils
import os
import yaml
import xml.etree.ElementTree as ET

from phobos.utils.io import l2str, xmlline, indent, xmlHeader
import phobos.model.materials as materials
import phobos.utils.general as gUtils
import phobos.utils.io as ioUtils
from phobos.phoboslog import log


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
        self.indentation = 0
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
        else:
            parameters = {}

        line += self.ind() + "<" + tag + " "

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
        self.output.append(self.ind() + '<' + str(tag) + '> ' + str(value) + ' </' + tag + '>\n')

    def getOutput(self):
        """ Completes all trailing tags until at initial indentation and returns the output as string.

        :return: str -- the finished xml string.
        """
        # ascend to base layer
        while self.indentation > self.initial:
            self.ascend()

        return self.output


def exportSdf(model, filepath):
    log("Export SDF to" + filepath, "INFO", "exportSdf")
    filename = os.path.join(filepath, model['name'] + '.sdf')

    # 'sensors', 'materials', 'controllers', 'date', 'links', 'chains', 'meshes', 'lights', 'motors', 'groups', 'joints', 'name'
    # print('sensors\n')
    # print(model['sensors'])
    # print('materials\n')
    # print(model['materials'])
    # print('controllers\n')
    # print(model['controllers'])
    # print('date\n')
    # print(model['date'])
    # print('links\n')
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
    #print('joints\n')
    #print(model['joints'])
    # print('name\n')
    # print(model['name'])

    # create tagger and add headers
    xml = xmlTagger()
    xml.write(xmlHeader)
    xml.descend('sdf', {"version": 1.5})

    # model layer
    xml.descend('model', params={"name": model['name']})

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
    # xml.descend('link', {'name': ...})
    # wrapper for links?
    # xml.ascend()

    # joint
    for jointkey in model['joints'].keys():
        joint = model['joints'][jointkey]
        xml.descend('joint', {'name': joint['name'], 'type': joint['type']})
        xml.attrib('parent', joint['parent'])
        xml.attrib('child', joint['child'])
        # xml.attrib('gearbox_ratio', ...)
        # xml.attrib('gearbox_reference_body', ...)'
        # xml.attrib('thread_pitch', ...)'
        xml.descend('axis')
        xml.attrib('xyz', '...')
        xml.ascend()
        xml.ascend()

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

    outputtext = xml.getOutput()

    with open(filename, 'w') as outputfile:
        outputfile.writelines(outputtext)
    log("Writing model data to " + filename, "INFO", "exportModelToSDF")

def importSdf():
    pass

# registering export functions of types with Phobos
entity_type_dict = {'sdf': {'export': exportSdf,
                            'import': importSdf,
                            'extensions': ('sdf', 'xml')}
                    }
