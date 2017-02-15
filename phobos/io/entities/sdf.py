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

def exportSdf(model, filepath):
    log("Export SDF to" + filepath, "INFO", "exportSdf")

def importSdf():
    pass

# registering export functions of types with Phobos
entity_type_dict = {'sdf': {'export': exportSdf,
                            'import': importSdf,
                            'extensions': ('sdf', 'xml')}
                    }
