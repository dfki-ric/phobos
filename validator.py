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

File validator.py

Created on 26 March 2015

@author: Ole Schwiegert
"""

from copy import deepcopy as dc


def check_dict(dic, validator, messages):
    check_dict_alg(dic, validator, [], messages, validator)


def check_dict_alg(dic, validator, entry_list, messages, whole_validator):
    for node in validator:
        new_list = dc(entry_list)
        node_value = validator[node]
        if not ('isReference' in node_value and len(entry_list) == 0):
            if is_operator(node):
                handle_operator(node, dic, validator, new_list, messages, whole_validator)
            elif is_leaf(node_value):
                new_list.append(node)
                check_leaf(node_value, dic, new_list, messages)
            else:
                new_list.append(node)
                check_dict_alg(dic, node_value, new_list, messages, whole_validator)


def is_leaf(node_value):
    return isinstance(node_value, dict) and 'required' in node_value


def is_operator(node):
    return node.startswith('$')


def check_leaf(leaf_value, dic, entry_list, messages):
    value = traverse_dict(dic, entry_list)
    default_value = leaf_value['default']
    required_type = type(default_value)
    required = leaf_value['required']
    print("Checking leaf " + str(entry_list))
    if required and value is None:
        messages.append("The required value in " + str(entry_list) + " cannot be found!")
        print("The required value in " + str(entry_list) + " cannot be found!")
    if value is not None and not isinstance(value, required_type):
        messages.append("The required value in " + str(entry_list) + " doesn't match expected type " + str(required_type))
        print("The required value in " + str(entry_list) + " doesn't match expected type " + str(required_type))


def handle_operator(node, dic, validator, entry_list, messages, whole_validator):
    if node == '$reference':
        new_list = dc(entry_list)
        new_list.append(validator[node])
        check_dict_alg(dic, whole_validator[validator[node]], new_list, messages, whole_validator)
    elif node == '$forElem':
        for elem in traverse_dict(dic, entry_list):
            new_list = dc(entry_list)
            new_list.append(elem)
            check_dict_alg(dic, validator['$forElem'], new_list, messages, whole_validator)
    elif node.startswith('$selection__'):
        pass


def traverse_dict(dic, entry_list):
    length = len(entry_list)
    if length > 0:
        element = entry_list[0]
        if isinstance(dic, dict) and length > 1 and element in dic:
            return traverse_dict(dic[element], entry_list[1:])
        elif isinstance(dic, dict) and length == 1 and element in dic:
            return dic[element]
    return None