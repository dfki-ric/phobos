#!/usr/bin/python
# coding=utf-8

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

File validation.py

Created on 26 March 2015

@author: Ole Schwiegert
"""

from copy import deepcopy as dc

import phobos.defs as defs
import phobos.utils.naming as nUtils
from phobos.phoboslog import log


checkMessages = {"NoObject": []}


def generateCheckMessages(param1, param2):
    # DOCU Parameter?
    """This function is just for generating a blender friendly list for an operator.
    """
    return [(x,)*3 for x in list(checkMessages.keys())]


def check_dict(dic, validator, messages):
    """This function validates a given dictionary against a validation.
    It writes all messages to the given messages list

    Args:
      dic(dict): The dictionary you want to validate.
      validator(dict): The validation you want to validate against.
      messages(dict): The message list you want to append the error messages to.

    Returns:

    """
    check_dict_alg(dic, validator, [], messages, validator, "NoObject")


def check_dict_alg(dic, validator, entry_list, messages, whole_validator, current_elem):
    """This function does the real validation work by working through the validation.

    Args:
      dic(dict): The dictionary you want to validate.
      validator(dict): The validation you want to validate against.
      entry_list(list: list): This list contains all keys you have to traverse to get the correct value in the dictionary.
      messages(dict): The message list you want to append the error messages to.
      whole_validator(dict): This is a copy of the whole validation needed when referencing to a top level key.
      current_elem(str): The current element the alg is checking.

    Returns:

    """
    for node in validator:
        new_list = dc(entry_list)
        node_value = validator[node]
        if node != 'isReference':
            if not ('isReference' in node_value and len(entry_list) == 0):
                if is_operator(node):
                    handle_operator(node, dic, validator, new_list, messages, whole_validator, current_elem)
                elif is_leaf(node_value):
                    new_list.append(node)
                    check_leaf(node_value, dic, new_list, messages, current_elem)
                else:
                    new_list.append(node)
                    check_dict_alg(dic, node_value, new_list, messages, whole_validator, current_elem)


def is_leaf(node_value):
    """This function checks whether a validation node is a leaf or not.

    Args:
      node_value(dict): The value of the node you want to check.

    Returns:
      bool.

    """
    return isinstance(node_value, dict) and 'required' in node_value


def is_operator(node):
    """This function checks whether a validation node is an operator or not.

    Args:
      node(str): The node key you want to check.

    Returns:
      bool.

    """
    return node.startswith('$')


def check_leaf(leaf_value, dic, entry_list, messages, current_elem):
    """This function checks the dictionary against a specific validation leaf and entry_list. Writing the
    messages into the given list.

    Args:
      leaf_value(dict): The leaf value used for validation.
      dic(dict): The dictionary you want to validate.
      entry_list(list: list): The keys navigating you to the dictionary value to validate against the validation leaf.
      messages(dict): The list you want to append the messages to.
      current_elem:

    Returns:

    """
    value = traverse_dict(dic, entry_list)
    default_value = leaf_value['default']
    required_type = type(default_value)
    required = leaf_value['required']
    # messages.append("Checking leaf " + str(entry_list))
    if required and value is None:
        add_message(messages, current_elem, "The required value in " + str(entry_list) + " cannot be found!")
    if value is not None and not isinstance(value, required_type):
        add_message(messages, current_elem, "The required value in " + str(entry_list) + " doesn't match expected type " + str(required_type))


def handle_operator(node, dic, validator, entry_list, messages, whole_validator, current_elem):
    """This function handles an operator and decides how to continue the validation process.

    Args:
      node(str): The operator to handle.
      dic(dict): The dict you want to validate.
      validator(dict): The validation you want to validate the dic with.
      entry_list(list: list): The list of keys to navigate to the value in the dictionary.
      messages(dict): The list to append the messages to.
      whole_validator(dict): The whole validation to reach top level keys in case of a reference operator.
      current_elem:

    Returns:

    """
    if node == '$reference':
        new_list = dc(entry_list)
        new_list.append(validator[node])
        check_dict_alg(dic, whole_validator[validator[node]], new_list, messages, whole_validator, current_elem)
    elif node == '$forElem':
        traversed_dic = traverse_dict(dic, entry_list)
        if traversed_dic is not None:
            for elem in traversed_dic:
                new_list = dc(entry_list)
                new_list.append(elem)
                check_dict_alg(dic, validator['$forElem'], new_list, messages, whole_validator, elem)
        else:
            add_message(messages, current_elem, "Error in traversing dict!")
    elif node.startswith('$selection__'):
        select_type = node.split('__')[1]
        select_dic = traverse_dict(dic, entry_list)
        if select_type in select_dic:
            select = select_dic[select_type]
            rest_validator = validator[node][select]
            check_dict_alg(dic, rest_validator, entry_list, messages, whole_validator, current_elem)
        else:
            add_message(messages, current_elem, "Could not find " + select_type + " in " + str(entry_list))
    elif node.startswith('$exists__'):
        # TODO handle it somehow...
        pass


def traverse_dict(dic, entry_list):
    """This function traverses a dictionary with a given list of keys and returns the value or None if the
    keys are not found.

    Args:
      dic(dict): The dictionary to traverse.
      entry_list(list: list): The list of keys you want to traverse with.

    Returns:
      dict.

    """
    length = len(entry_list)
    if length > 0:
        element = entry_list[0]
        if isinstance(dic, dict) and length > 1 and element in dic:
            return traverse_dict(dic[element], entry_list[1:])
        elif isinstance(dic, dict) and length == 1 and element in dic:
            return dic[element]
    return None


def add_message(messages, key, message):
    """This function adds a message to the messages dictionary.

    Args:
      messages(dict): The dictionary containing the messages.
      key(str): The messages corresponding key (node name).
      message(str): The message to append to a specific key.

    Returns:
      None.

    """
    if key in messages:
        messages[key].append(message)
    else:
        messages[key] = [message]


class ValidateMessage():
    """Resembles a message from a validation describing an issue in the model/object etc."""

    def __init__(self, message, level, obj=None, operator=None, information=None):
        """Create a new ValidateMessage object.

        Args:
            message (str): message of the issue
            level (str): severity level of the issue
            obj (bpy.types.Object, optional): Blender object in which the issue occured
            operator (str, optional): bl_idname of the operator to fix the issue
            information (dict, str, optional): information which is used to fix the issue
        """
        self.message = message
        self.level = level
        self.obj = obj
        self.operator = operator
        self.information = information

    def __lt__(self, other):
        """Returns true if the error message of this object is less than the other.

        Returns:
            bool: error message of this obj < error message of the other obj
        """
        return self.message.__lt__(other.message)

    def log(self):
        log(self.message +
            ('\n' + 4 * ' ' + self.information['log_info'] if 'log_info' in self.information
             else '') + " @" + nUtils.getObjectName(self.obj), self.level)

    def __eq__(self, other):
        if isinstance(other, str):
            return self.message == other
        elif isinstance(other, ValidateMessage):
            return self.message == other.message and self.obj == other.obj
        else:
            return False


def validateObjectNames(obj):
    """Check for naming errors of the specified object.

    This checks:
        - the *phobostype*/names

    Args:
        obj (bpy.types.Object): object to be checked

    Returns:
        list: :class:ValidateMessage list
    """
    phobtype = obj.phobostype
    objname = obj.name
    errors = []

    if phobtype + '/name' in obj:
        nameset = set([ptype[0] for ptype in defs.phobostypes if ptype[0] + '/name' in obj])

        # links are allowed to contain additional joint information
        if phobtype == 'link' and 'joint' in nameset:
            nameset = nameset.difference(set(['joint']))
        nameset = nameset.difference(set([phobtype]))

        for key in nameset:
            errors.append(ValidateMessage(
                "Redundant name: '" + key + "/name'!",
                "WARNING",
                obj,
                "phobos.fix_object_names",
                key + '/name'
            ))

    # TODO add unique name checks etc
    return errors

def validateJoint(link, adjust=False):
    """Checks for errors in the joint definitions of the specified link.

        If autocomplete is set, the missing dictionary entries are complemented.

    Args:
        link (bpy.types.Object): link object which forms the joint
        autocomplete (bool): add missing keys to the link object

    Returns:
        list(ValidateMessage) -- error messages of the validation
    """
    errors = []

    # a link without parent can not be a joint
    if not link.parent:
        errors.append(ValidateMessage(
            "No joint parent!",
            'WARNING',
            None, {}))

    # make sure the joint type is validated
    errors.extend(validateJointType(link, adjust=adjust))

    return errors


def validateJointType(link, adjust=False):
    """Validate the joint type of the specified link.

    If adjust is `True`, the validation errors are fixed on the fly.

    Args:
        link (bpy.types.Object): link representing the joint to validate
        adjust (bool): if True, the validation errors are fixed on the fly.

    Returns:
        list(ValidateMessage) -- validation errors
    """
    import phobos.model.joints as jointmodel
    errors = []
    if 'joint/type' not in link:
        errors.append(ValidateMessage(
            "No joint type specified!",
            'WARNING',
            link,
            "phobos.define_joint_constraints",
            {}))
        # make sure we get no KeyErrors from this
        if adjust:
            link['joint/type'] = 'undefined'

    joint_type, crot = jointmodel.getJointType(link)

    # warn user if the constraints do not match the specified joint type
    if 'joint/type' in link and joint_type != link['joint/type']:
        if not adjust:
            errors.append(ValidateMessage(
                "The specified joint type does not match the constraints:",
                'WARNING',
                link, None,
                {'log_info': str("'" + link['joint/type'] + "' should be set to '" +
                                 joint_type + "' instead.")}))
        else:
            link['joint/type'] = joint_type
            errors.append(ValidateMessage(
                "Adjusted joint type to '" + joint_type + "'.",
                'INFO', link, None, {}))

    return errors


def validateLink(link, objectlist=None):
    errors = []

    # TODO add validation tests

    return errors


def validateObjectPose(obj):
    errors = []

    # TODO add validation tests

    return errors


def validateMaterial(material):
    """Validate the specified material.

    Args:
        material (bpy.types.Material): material to validate

    Returns:
        list(ValidateMessage) -- validation errors for the material
    """
    errors = []

    # there are always 18 slots, regardless of whether they are filled or not
    for tex in material.texture_slots:
        if tex is not None:
            try:
                # regular diffuse color texture
                if tex.use_map_color_diffuse:
                    # grab the first texture
                    material.texture_slots[0].texture.image.filepath.replace('//', '')
            except (KeyError, AttributeError):
                errors.append(ValidateMessage(
                    "Diffuse texture incomplete/undefined.",
                    'WARNING',
                    material, None, {}))
            try:
                # normal map
                if tex.use_map_normal:
                    # grab the first texture
                    material.texture_slots[0].texture.image.filepath.replace('//', '')
            except (KeyError, AttributeError):
                errors.append(ValidateMessage(
                    "Normal texture incomplete/undefined.",
                    'WARNING',
                    material, None, {}))
            try:
                # displacement map
                if tex.use_map_displacement:
                    # grab the first texture
                    material.texture_slots[0].texture.image.filepath.replace('//', '')
            except (KeyError, AttributeError):
                errors.append(ValidateMessage(
                    "Displacement texture incomplete/undefined.",
                    'WARNING',
                    material, None, {}))
    return errors


def validate(name):
    def validation(function):
        def validation_wrapper(obj, *args, logging=False, **kwargs):
            if name == 'joint':
                errors = validateJoint(obj, *args, **kwargs)
            elif name == 'joint_type':
                errors = validateJointType(obj, *args, **kwargs)
            elif name == 'material':
                errors = validateMaterial(obj, *args, **kwargs)
            elif name == 'link':
                errors = validateLink(obj, *args, **kwargs)
            elif name == 'object_pose':
                errors = validateObjectPose(obj, *args, **kwargs)
            else:
                log('This validation type is not defined!', 'ERROR')
                errors = []

            kwargs['errors'] = errors

            if logging:
                for error in errors:
                    error.log()

            return function(obj, *args, logging=logging, **kwargs)

        return validation_wrapper
    return validation
