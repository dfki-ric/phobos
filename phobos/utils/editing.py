#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.utils.general
    :platform: Unix, Windows, Mac
    :synopsis: This module contains general functions to use in operators and custom scripts

.. moduleauthor:: Kai von Szadowski

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
"""


def addDictionaryToObj(dict, obj, category=None):
    # DOCU add some docstring
    for key, value in dict:
        obj[(category+'/'+key) if category else key] = value


def getCombinedTransform(obj, effectiveparent):
    # DOCU add some docstring
    parent = obj.parent
    matrix = obj.matrix_local
    while parent != effectiveparent and parent is not None:
        matrix = parent.matrix_local * matrix
        parent = parent.parent
    return matrix
