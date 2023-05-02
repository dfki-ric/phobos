#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

from copy import deepcopy as dc

import bpy

from .. import defs
from ..phoboslog import log
from ..utils import naming as nUtils

checkMessages = {"NoObject": []}


def generateCheckMessages(param1, param2):
    """

    Args:
      param1: 
      param2: 

    Returns:

    """
    # DOCU Parameter?
    """This function is just for generating a blender friendly list for an operator.
    """
    return [(x,) * 3 for x in list(checkMessages.keys())]


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
      entry_list: 

    Returns:

    """
    for node in validator:
        new_list = dc(entry_list)
        node_value = validator[node]
        if node != 'isReference':
            if not ('isReference' in node_value and len(entry_list) == 0):
                if is_operator(node):
                    handle_operator(
                        node, dic, validator, new_list, messages, whole_validator, current_elem
                    )
                elif is_leaf(node_value):
                    new_list.append(node)
                    check_leaf(node_value, dic, new_list, messages, current_elem)
                else:
                    new_list.append(node)
                    check_dict_alg(
                        dic, node_value, new_list, messages, whole_validator, current_elem
                    )


def is_leaf(node_value):
    """This function checks whether a validation node is a leaf or not.

    Args:
      node_value(dict): The value of the node you want to check.

    Returns:
      : bool.

    """
    return isinstance(node_value, dict) and 'required' in node_value


def is_operator(node):
    """This function checks whether a validation node is an operator or not.

    Args:
      node(str): The node key you want to check.

    Returns:
      : bool.

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
      entry_list: 

    Returns:

    """
    value = traverse_dict(dic, entry_list)
    default_value = leaf_value['default']
    required_type = type(default_value)
    required = leaf_value['required']
    # messages.append("Checking leaf " + str(entry_list))
    if required and value is None:
        add_message(
            messages, current_elem, "The required value in " + str(entry_list) + " cannot be found!"
        )
    if value is not None and not isinstance(value, required_type):
        add_message(
            messages,
            current_elem,
            "The required value in "
            + str(entry_list)
            + " doesn't match expected type "
            + str(required_type),
        )


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
      entry_list: 

    Returns:

    """
    if node == '$reference':
        new_list = dc(entry_list)
        new_list.append(validator[node])
        check_dict_alg(
            dic, whole_validator[validator[node]], new_list, messages, whole_validator, current_elem
        )
    elif node == '$forElem':
        traversed_dic = traverse_dict(dic, entry_list)
        if traversed_dic is not None:
            for elem in traversed_dic:
                new_list = dc(entry_list)
                new_list.append(elem)
                check_dict_alg(
                    dic, validator['$forElem'], new_list, messages, whole_validator, elem
                )
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
            add_message(
                messages, current_elem, "Could not find " + select_type + " in " + str(entry_list)
            )
    elif node.startswith('$exists__'):
        # TODO handle it somehow...
        pass


def traverse_dict(dic, entry_list):
    """This function traverses a dictionary with a given list of keys and returns the value or None if the
    keys are not found.

    Args:
      dic(dict): The dictionary to traverse.
      entry_list(list: list): The list of keys you want to traverse with.
      entry_list: 

    Returns:
      : dict.

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
      : None.

    """
    if key in messages:
        messages[key].append(message)
    else:
        messages[key] = [message]


class ValidateMessage:
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
        """TODO Missing documentation"""
        log(
            self.message
            + str(
                ''
                if not isinstance(self.obj, bpy.types.Object)
                else " @" + nUtils.getObjectName(self.obj)
            )
            + str(
                ('\n' + 4 * ' ' + self.information['log_info'])
                if 'log_info' in self.information
                else ''
            ),
            self.level,
        )

    def __eq__(self, other):
        if isinstance(other, str):
            return self.message == other
        elif isinstance(other, ValidateMessage):
            return self.message == other.message and self.obj == other.obj
        else:
            return False


def validateJoint(link, adjust=False):
    """Checks for errors in the joint definitions of the specified link.
    
        If autocomplete is set, the missing dictionary entries are complemented.

    Args:
      link(bpy.types.Object): link object which forms the joint
      autocomplete(bool): add missing keys to the link object
      adjust: (Default value = False)

    Returns:
      : list(ValidateMessage) -- error messages of the validation

    """
    errors = []

    # a link without parent can not be a joint
    if not link.parent:
        errors.append(ValidateMessage("No joint parent!", 'WARNING', None, {}))

    # make sure the joint type is validated
    errors.extend(validateJointType(link, adjust=adjust))

    return errors


def validateJointType(link, adjust=False):
    """Validate the joint type of the specified link.
    
    If adjust is `True`, the validation errors are fixed on the fly.

    Args:
      link(bpy.types.Object): link representing the joint to validate
      adjust(bool, optional): if True, the validation errors are fixed on the fly. (Default value = False)

    Returns:
      : list(ValidateMessage) -- validation errors

    """
    import phobos.blender.model.joints as jointmodel
    import phobos.blender.utils.io as ioUtils

    errors = []
    if 'joint/type' not in link:
        errors.append(
            ValidateMessage(
                "No joint type specified!", 'WARNING', link, "phobos.define_joint_constraints", {}
            )
        )
        # make sure we get no KeyErrors from this
        if adjust:
            link['joint/type'] = 'undefined'

    joint_type, crot = jointmodel.getJointType(link)

    # warn user if the constraints do not match the specified joint type
    if 'joint/type' in link and joint_type != link['joint/type']:
        if not adjust:
            errors.append(
                ValidateMessage(
                    "The specified joint type does not match the constraints:",
                    'WARNING',
                    link,
                    None,
                    {
                        'log_info': str(
                            "'"
                            + link['joint/type']
                            + "' should be set to '"
                            + joint_type
                            + "' instead."
                        )
                    },
                )
            )
        else:
            # fix joint type and assign new resource object
            link['joint/type'] = joint_type
            resource_obj = ioUtils.getResource(('joint', joint_type))
            if resource_obj:
                log("Assigned resource to {}.".format(link.name), 'DEBUG')
                link.pose.bones[0].custom_shape = resource_obj

            errors.append(
                ValidateMessage(
                    "Adjusted joint type to '" + joint_type + "'.", 'INFO', link, None, {}
                )
            )

    return errors


def validateLink(link, objectlist=None):
    """

    Args:
      link: 
      objectlist: (Default value = None)

    Returns:

    """
    errors = []

    # TODO add validation tests

    return errors


def validateObjectPose(obj, adjust=False, **kwargs):
    """

    Args:
      obj: 
      adjust: (Default value = False)
      **kwargs: 

    Returns:

    """
    errors = []

    # TODO add validation tests

    return errors


def validateMaterial(material, adjust=False):
    """Validate the specified material.

    Args:
      material(bpy.types.Material): material to validate
      adjust: (Default value = False)

    Returns:
      : list(ValidateMessage) -- validation errors for the material

    """
    errors = []

    if not material:
        errors.append(ValidateMessage("No material defined.", 'WARNING', material, None, {}))
        return errors, material

    if isinstance(material, bpy.types.Object):
        # there are always 18 slots, regardless of whether they are filled or not
        for tex in material.texture_slots:
            if tex is not None:
                try:
                    # regular diffuse color texture
                    if tex.use_map_color_diffuse:
                        # grab the first texture
                        material.texture_slots[0].texture.image.filepath.replace('//', '')
                except (KeyError, AttributeError):
                    errors.append(
                        ValidateMessage(
                            "Diffuse texture incomplete/undefined.", 'WARNING', material, None, {}
                        )
                    )
                try:
                    # normal map
                    if tex.use_map_normal:
                        # grab the first texture
                        material.texture_slots[0].texture.image.filepath.replace('//', '')
                except (KeyError, AttributeError):
                    errors.append(
                        ValidateMessage(
                            "Normal texture incomplete/undefined.", 'WARNING', material, None, {}
                        )
                    )
                try:
                    # displacement map
                    if tex.use_map_displacement:
                        # grab the first texture
                        material.texture_slots[0].texture.image.filepath.replace('//', '')
                except (KeyError, AttributeError):
                    errors.append(
                        ValidateMessage(
                            "Displacement texture incomplete/undefined.",
                            'WARNING',
                            material,
                            None,
                            {},
                        )
                    )
    else:
        if not hasattr(material, "name"):
            if adjust:
                material = {'name': 'phobos_error'}
                loglevel = 'WARNING'
            else:
                loglevel = 'ERROR'
            errors.append(
                ValidateMessage("Material name not defined.", 'ERROR', material, None, {})
            )
            return errors, material

        if 'diffuse' not in material:
            if adjust:
                material['diffuse'] = (1., 1., 1., 1.)
                loglevel = 'WARNING'
            else:
                loglevel = 'ERROR'
            errors.append(
                ValidateMessage("Material diffuse color not defined.", 'ERROR', material, None, {})
            )
        elif len(material['diffuse']) != 4:
            if adjust:
                if len(material['diffuse']) == 3:
                    material['diffuse'] = tuple(material['diffuse'] + [1.])
                loglevel = 'WARNING'
            else:
                loglevel = 'ERROR'
            errors.append(
                ValidateMessage(
                    "Material diffuse color definition insufficient.", loglevel, material, None, {}
                )
            )

        if 'diffuse_intensity' not in material:
            errors.append(
                ValidateMessage(
                    "Material diffuse intensity not defined.", 'WARNING', material, None, {}
                )
            )
            if adjust:
                material['diffuse_intensity'] = 1.
    return errors, material


def validateGeometryType(obj, *args, adjust=False, geometry_dict=None, **kwargs):
    """

    Args:
      obj: 
      *args: 
      adjust: (Default value = False)
      geometry_dict: (Default value = None)

    Returns:

    """
    errors = []
    # make sure the geometry is defined
    if 'geometry/type' not in obj and (not geometry_dict or 'type' not in geometry_dict):
        errors.append(
            ValidateMessage("Geometry type undefined!", 'ERROR', obj, 'phobos.define_geometry', {})
        )
        return errors

    if not geometry_dict:
        geometry_dict = {'type': obj['geometry/type']}

    # check for valid geometry type
    geometry_types = [entry[0] for entry in defs.geometrytypes]
    if geometry_dict['type'] not in geometry_types:
        errors.append(
            ValidateMessage(
                "Geometry type not supported!",
                'ERROR',
                obj,
                'phobos.define_geometry',
                {'log_info': geometry_dict['type']},
            )
        )
        return errors

    if adjust:
        # ensure the geometry type is added to the object
        obj['geometry/type'] = geometry_dict['type']

    return errors


def validateInertiaData(obj, *args, adjust=False):
    """Validates an inertia dictionary or object.
    
    This checks for the *inertia* and *mass* values in the dictionary (*inertia* and
    *mass* for an object respectively).
    
    Also, the inertia values are checked to be positive definite (for diagonal, determinant and
    eigenvalues).
    
    If adjust is set, values are adjusted/fixed for the returned dict/object. E.g. this sets a
    negative mass to 1e-3.

    Args:
      obj(dict/bpy.types.Object): inertia dictionary or object to validate
      *args: other arguments
      adjust: if True, bad values will be fixed/complemented (Default value = False)

    Returns:
      tuple: list of :class:`ValidateMessage`\ s and the fixed dictionary/object

    """
    from phobos.blender.model.inertia import inertiaListToMatrix, inertiaMatrixToList
    from phobos.blender.utils.io import getExpSettings
    import numpy

    errors = []

    # [TODO v2.1.0] REVIEW this
    expsetting = 10**(-getExpSettings().urdfDecimalPlaces)

    if obj.phobostype != 'inertial':
        errors.append(
            ValidateMessage(
                "Object '{0}' is not of phobostype 'inertial'.".format(obj.name),
                "ERROR",
                obj
            )
        )

    # check dictionary parameters (most of the time pre object creation)
    inertia = (1e-3, 0., 0., 1e-3, 0., 1e-3)
    mass = 1e-3
    if isinstance(obj, dict):
        missing = []
        if 'inertia' not in obj:
            missing.append('inertia')
        else:
            inertia = obj['inertia']

        if 'mass' not in obj:
            missing.append('mass')
        else:
            mass = obj['mass']

        if missing:
            errors.append(
                ValidateMessage(
                    "Inertia dictionary not fully defined!",
                    'WARNING',
                    None,
                    None,
                    {
                        'log_info': "Missing: "
                        + ' '.join(["'{0}'".format(miss) for miss in missing])
                        + " Set to default 1e-3."
                    },
                )
            )
    # check existing object properties
    elif isinstance(obj, bpy.types.Object):
        missing = []
        if 'inertia' not in obj:
            errors.append(
                ValidateMessage(
                    "Inertia not defined!",
                    'WARNING',
                    obj,
                    'phobos.generate_inertial_objects',
                    {'log_info': "Set to default 1e-3."},
                )
            )
            missing.append('inertia')

        if 'mass' not in obj:
            errors.append(
                ValidateMessage(
                    "Mass is not defined!",
                    'WARNING',
                    obj,
                    'phobos.generate_inertial_objects',
                    {'log_info': "Set to default 1e-3."},
                )
            )
            missing.append('mass')

    else:
        raise AssertionError(type(obj))

    # Check inertia vector for various properties
    inertia = numpy.array(inertiaListToMatrix(inertia))

    if any(element <= 0. for element in inertia.diagonal()):
        errors.append(
            ValidateMessage(
                "Negative semidefinite main diagonal in inertia data!",
                'WARNING',
                None if isinstance(obj, dict) else obj,
                None,
                {'log_info': "Diagonal: " + str(inertia.diagonal())},
            )
        )

    # Calculate the determinant if consistent, quick check
    if numpy.linalg.det(inertia) <= 0.:
        errors.append(
            ValidateMessage(
                "Negative semidefinite determinant in inertia data! Checking singular values.",
                'WARNING',
                None if isinstance(obj, dict) else obj,
                None,
                {'log_info': "Determinant: " + str(numpy.linalg.det(inertia))},
            )
        )

        # Calculate the eigenvalues if not consistent
        if any(element <= 0. for element in numpy.linalg.eigvals(inertia)):
            # Apply singular value decomposition and correct the values
            S, V = numpy.linalg.eig(inertia)
            S[S <= expsetting] = expsetting
            errors.append(
                ValidateMessage(
                    "Negative semidefinite eigenvalues in inertia data!",
                    'WARNING',
                    None if isinstance(obj, dict) else obj,
                    None,
                    {'log_info': "Eigenvalues: " + str(numpy.linalg.eigvals(inertia))},
                )
            )
            inertia = V.dot(numpy.diag(S).dot(V.T))

    if mass <= 0.:
        errors.append(
            ValidateMessage(
                "Mass is {}!".format('zero' if mass == 0. else 'negative'),
                'WARNING',
                None if isinstance(obj, dict) else obj,
                None,
                {} if not adjust else {'log_info': "Adjusted to 1e-3."},
            )
        )
        mass = expsetting

    inertia = inertiaMatrixToList(inertia)

    if adjust:
        obj['inertia'] = inertia
        obj['mass'] = mass

    return errors, obj


def validateVisual(obj, *args, adjust=False, geometry_dict=None, **kwargs):
    """

    Args:
      obj: 
      *args: 
      adjust: (Default value = False)
      geometry_dict: (Default value = None)

    Returns:

    """
    errors = []

    return errors


def validate(name):
    """

    Args:
      name: 

    Returns:

    """

    def validation(function):
        """

        Args:
          function: 

        Returns:

        """

        def validation_wrapper(obj, *args, logging=False, **kwargs):
            """

            Args:
              obj: 
              *args: 
              logging: (Default value = False)
              **kwargs: 

            Returns:

            """
            if name == 'joint':
                errors = validateJoint(obj, *args, **kwargs)
            elif name == 'joint_type':
                errors = validateJointType(obj, *args, **kwargs)
            elif name == 'material':
                errors, obj = validateMaterial(obj, *args, **kwargs)
            elif name == 'link':
                errors = validateLink(obj, *args, **kwargs)
            elif name == 'object_pose':
                errors = validateObjectPose(obj, *args, **kwargs)
            elif name == 'geometry_type':
                errors = validateGeometryType(obj, *args, **kwargs)
            elif name == 'inertia_data':
                errors, obj = validateInertiaData(obj, *args, **kwargs)
            elif name == 'visual':
                errors = validateVisual(obj, *args, **kwargs)
            else:
                log("This validation type is not defined! '{}'".format(name), 'ERROR')
                errors = []

            kwargs['errors'] = errors

            if logging:
                for error in errors:
                    error.log()

            return function(obj, *args, logging=logging, **kwargs)

        return validation_wrapper

    return validation
