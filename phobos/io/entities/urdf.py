#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

from os import path
import re
import json
import xml.etree.ElementTree as ET

import bpy
import mathutils
from phobos.utils.io import l2str, xmlline, indent, xmlHeader
import phobos.model.materials as matModel
import phobos.utils.general as gUtils
import phobos.utils.io as ioUtils
from phobos.phoboslog import log


def sort_urdf_elements(elems):
    """Sort a collection of elements. By default, this method simply wraps the standard
    'sorted()' method. This is done in order to be able to easily change the element ordering.

    Args:
      elems: a collection

    Returns:
      : sorted colletion

    """
    return sorted(elems)


def writeURDFGeometry(output, element, filepath):
    """This functions writes the URDF geometry for a given element at the end of a given String.

    Args:
      output(str): The String to append the URDF output string on.
      element(dict): A certain element to parse into URDF.
      filepath: 

    Returns:
      : str -- The extended String

    """
    geometry = element['geometry']
    try:
        output.append(indent * 4 + '<geometry>\n')
        if geometry['type'] == 'box':
            output.append(xmlline(5, 'box', ['size'], [l2str(geometry['size'])]))
        elif geometry['type'] == "cylinder":
            output.append(
                xmlline(
                    5, 'cylinder', ['radius', 'length'], [geometry['radius'], geometry['length']]
                )
            )
        elif geometry['type'] == "sphere":
            output.append(xmlline(5, 'sphere', ['radius'], [geometry['radius']]))
        elif geometry['type'] == 'mesh':
            meshpath = ioUtils.getOutputMeshpath(path.dirname(filepath), None, None) + geometry['filename'] + '.' + ioUtils.getOutputMeshtype()
            if not "package://" in meshpath:
                meshpath = path.relpath(meshpath, filepath)
            output.append(
                xmlline(
                    5,
                    'mesh',
                    ['filename', 'scale'],
                    [
                        meshpath,
                        l2str(geometry['scale']),
                    ],
                )
            )
        else:
            raise TypeError("Unknown geometry type")
        output.append(indent * 4 + '</geometry>\n')
    except (KeyError, TypeError) as err:
        log("Misdefined geometry in element " + element['name'] + " " + str(err), "ERROR")


def exportUrdf(model, outpath):
    """This functions writes the URDF of a given model into a file at the given filepath.
    An existing file with this path will be overwritten.

    Args:
      model(dict): Dictionary of the model to be exported as URDF.
      outpath(str): The path of the exported file.

    Returns:

    """
    log("Export URDF to " + outpath, "INFO")
    filename = path.join(outpath, model['name'] + '.urdf')

    stored_element_order = None
    # CHECK test Windows path consistency
    order_file_name = model['name'] + '_urdf_order'
    if order_file_name in bpy.data.texts:
        stored_element_order = json.loads(bpy.data.texts[order_file_name].as_string())

    output = [xmlHeader, indent + '<robot name="' + model['name'] + '">\n\n']
    # export link information
    if stored_element_order is None:
        sorted_link_keys = sorted(model['links'])
    else:
        sorted_link_keys = stored_element_order['links']
        new_keys = []
        for link_key in model['links']:
            if link_key not in sorted_link_keys:
                new_keys.append(link_key)
        sorted_link_keys += sort_urdf_elements(new_keys)
    for l in sorted_link_keys:
        if l in model['links']:
            link = model['links'][l]
            output.append(indent * 2 + '<link name="' + link['name'] + '">\n')
            if 'mass' in link['inertial'] and 'inertia' in link['inertial']:
                output.append(indent * 3 + '<inertial>\n')
                if 'pose' in link['inertial']:
                    output.append(
                        xmlline(
                            4,
                            'origin',
                            ['xyz', 'rpy'],
                            [
                                l2str(link['inertial']['pose']['translation']),
                                l2str(link['inertial']['pose']['rotation_euler']),
                            ],
                        )
                    )
                output.append(xmlline(4, 'mass', ['value'], [str(link['inertial']['mass'])]))
                output.append(
                    xmlline(
                        4,
                        'inertia',
                        ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'],
                        [str(i) for i in link['inertial']['inertia']],
                    )
                )
                output.append(indent * 3 + '</inertial>\n')
            # visual object
            if link['visual']:
                if stored_element_order is None:
                    sorted_visual_keys = sorted(link['visual'])
                else:
                    sorted_visual_keys = stored_element_order['viscol'][link['name']]['visual']
                    new_keys = []
                    for vis_key in link['visual']:
                        if vis_key not in sorted_visual_keys:
                            new_keys.append(vis_key)
                    sorted_visual_keys += sort_urdf_elements(new_keys)
                for v in sorted_visual_keys:
                    if v in link['visual']:
                        vis = link['visual'][v]
                        output.append(indent * 3 + '<visual name="' + vis['name'] + '">\n')
                        output.append(
                            xmlline(
                                4,
                                'origin',
                                ['xyz', 'rpy'],
                                [
                                    l2str(vis['pose']['translation']),
                                    l2str(vis['pose']['rotation_euler']),
                                ],
                            )
                        )
                        writeURDFGeometry(output, vis, outpath)
                        if 'material' in vis:
                            # FIXME: change back to 1 when implemented in urdfloader
                            if model['materials'][vis['material']]['users'] == 0:
                                mat = model['materials'][vis['material']]
                                output.append(
                                    indent * 4 + '<material name="' + mat['name'] + '">\n'
                                )
                                color = mat['diffuseColor']
                                output.append(
                                    indent * 5
                                    + '<color rgba="'
                                    + l2str([color[num] for num in ['r', 'g', 'b']])
                                    + ' '
                                    + str(mat["transparency"])
                                    + '"/>\n'
                                )
                                if 'diffuseTexture' in mat:
                                    output.append(
                                        indent * 5
                                        + '<texture filename="'
                                        + mat['diffuseTexture']
                                        + '"/>\n'
                                    )
                                output.append(indent * 4 + '</material>\n')
                            else:
                                output.append(
                                    indent * 4 + '<material name="' + vis["material"] + '"/>\n'
                                )
                        output.append(indent * 3 + '</visual>\n')
            # collision object
            if link['collision']:
                if stored_element_order is None:
                    sorted_collision_keys = sorted(link['collision'])
                else:
                    sorted_collision_keys = stored_element_order['viscol'][link['name']][
                        'collision'
                    ]
                    new_keys = []
                    for col_key in link['collision']:
                        if col_key not in sorted_collision_keys:
                            new_keys.append(col_key)
                    sorted_collision_keys += sort_urdf_elements(new_keys)
                for c in sorted_collision_keys:
                    if c in link['collision']:
                        col = link['collision'][c]
                        output.append(indent * 3 + '<collision name="' + col['name'] + '">\n')
                        output.append(
                            xmlline(
                                4,
                                'origin',
                                ['xyz', 'rpy'],
                                [
                                    l2str(col['pose']['translation']),
                                    l2str(col['pose']['rotation_euler']),
                                ],
                            )
                        )
                        writeURDFGeometry(output, col, outpath)
                        output.append(indent * 3 + '</collision>\n')
            output.append(indent * 2 + '</link>\n\n')
    # export joint information
    missing_values = False
    if stored_element_order is None:
        sorted_joint_keys = sorted(model['joints'])
    else:
        sorted_joint_keys = stored_element_order['joints']
        new_keys = []
        for joint_key in model['joints']:
            if joint_key not in sorted_joint_keys:
                new_keys.append(joint_key)
        sorted_joint_keys += sort_urdf_elements(new_keys)
    for j in sorted_joint_keys:
        if j in model['joints']:
            joint = model['joints'][j]
            output.append(
                indent * 2 + '<joint name="' + joint['name'] + '" type="' + joint["type"] + '">\n'
            )
            child = model['links'][joint["child"]]
            output.append(
                xmlline(
                    3,
                    'origin',
                    ['xyz', 'rpy'],
                    [l2str(child['pose']['translation']), l2str(child['pose']['rotation_euler'])],
                )
            )
            output.append(indent * 3 + '<parent link="' + joint["parent"] + '"/>\n')
            output.append(indent * 3 + '<child link="' + joint["child"] + '"/>\n')
            if 'axis' in joint:
                output.append(indent * 3 + '<axis xyz="' + l2str(joint['axis']) + '"/>\n')
            if 'limits' in joint:
                for limit_value in ['effort', 'velocity']:
                    if limit_value not in joint['limits']:
                        log(
                            "joint '"
                            + joint['name']
                            + "' does not specify a maximum "
                            + limit_value
                            + "!",
                            "WARNING",
                        )
                        missing_values = True
                used_limits = []
                for limit in ['lower', 'upper', 'effort', 'velocity']:
                    if limit in joint['limits']:
                        used_limits.append(limit)
                output.append(
                    xmlline(3, 'limit', used_limits, [joint['limits'][p] for p in used_limits])
                )
            elif joint['type'] in ['revolute', 'prismatic']:
                log(
                    "joint '"
                    + joint['name']
                    + "' does not specify limits, though its type is "
                    + joint['type']
                    + "!",
                    "WARNING",
                )
                missing_values = True
            output.append(indent * 2 + '</joint>\n\n')
    # export material information
    if missing_values:
        log("Created URDF is invalid due to missing values!", "WARNING")
    if stored_element_order is None:
        sorted_material_keys = sorted(model['materials'])
    else:
        sorted_material_keys = stored_element_order['materials']
        new_keys = []
        for material_key in model['materials']:
            if material_key not in sorted_material_keys:
                new_keys.append(material_key)
        sorted_material_keys += sort_urdf_elements(new_keys)
    for m in sorted_material_keys:
        if m in model['materials']:
            # FIXME: change back to 1 when implemented in urdfloader
            if model['materials'][m]['users'] > 0:
                output.append(indent * 2 + '<material name="' + m + '">\n')
                color = model['materials'][m]['diffuseColor']
                transparency = (
                    model['materials'][m]['transparency']
                    if 'transparency' in model['materials'][m]
                    else 0.0
                )
                output.append(
                    indent * 3
                    + '<color rgba="'
                    + l2str([color[num] for num in ['r', 'g', 'b']])
                    + ' '
                    + str(1.0 - transparency)
                    + '"/>\n'
                )
                if 'diffuseTexture' in model['materials'][m]:
                    output.append(
                        indent * 3
                        + '<texture filename="'
                        + model['materials'][m]['diffuseTexture']
                        + '"/>\n'
                    )
                output.append(indent * 2 + '</material>\n\n')
    # finish the export
    output.append(indent + '</robot>\n')
    with open(filename, 'w') as outputfile:
        outputfile.write(''.join(output))
    # FIXME: different joint transformations needed for fixed joints
    log("Writing model data to " + filename, "INFO")


def store_element_order(element_order, path):
    """This function dumps whatever pythonic yaml structure to a given filepath and appends
    *_element_order_debug.yml* to the end of path.

    Args:
      element_order(dict): The dictionary you want to dump into a yaml file.
      path(str): The path you want to write the yaml dump to.

    Returns:

    """
    # TODO delete me?
    # element_order = {}
    # link_order = []
    # viscol_order = {}
    # for link_key in model['links']:
    #    link = model['links'][link_key]
    #    link_element_order = {}
    #    link_order.append(link['name'])
    #    visual_order = []
    #    for visual_key in link['visual']:
    #        visual_order.append(link['visual'][visual_key]['name'])
    #    link_element_order['visual'] = visual_order
    #    collision_order = []
    #    for collision_key in link['collision']:
    #        collision_order.append(link['collision'][collision_key]['name'])
    #    link_element_order['collision'] = collision_order
    #    viscol_order[link['name']] = link_element_order
    # element_order['viscol'] = viscol_order
    # element_order['links'] = link_order
    # joint_order = []
    # for joint_key in model['joints']:
    #    joint_order.append(model['joints'][joint_key]['name'])
    # element_order['joints'] = joint_order

    stream = open(path + '_element_order_debug.yml', 'w')
    stream.write(json.dumps(element_order, indent=2))
    stream.close()


def calc_pose_formats(position, rotation, pivot=(0, 0, 0)):
    """Create a dictionary containing various representations of the pose
    represented by 'position' and 'rotation':
    
        - translation == position
        - rotation_quaternion == rotation
        - rotation_euler: euler angles
        - matrix: position and rotation in 4x4 matrix form

    Args:
      position(list): The position to include in the dictionary.
      rotation(list): The rotation to include into the dictionary, either euler angle or quaternion.
      pivot(list, optional): The pivot point. (Default value = (0)
      0: 
      0): 

    Returns:
      : dict

    """
    px, py, pz = position
    if len(rotation) == 3:
        rot = mathutils.Euler(rotation).to_quaternion()
        # TODO delete me?
        # print(rotation)
    else:
        rot = mathutils.Quaternion(rotation)

    # TODO delete me?
    # if angle_offset is not 0.0:
    #    axis_vec = mathutils.Vector(axis)

    #    offset_matrix = mathutils.Matrix.Rotation(angle_offset, 4, axis_vec) #get_axis_rotation_matrix(axis, angle_offset)
    #    rot_matrix = rot.to_matrix().to_4x4()
    #    applied_matrix = rot_matrix * offset_matrix
    #    rot = applied_matrix.to_quaternion()

    rw, rx, ry, rz = rot
    pose_dict = {}

    neg_pivot_translation = mathutils.Matrix.Translation((-pivot[0], -pivot[1], -pivot[2]))
    pivot_translation = mathutils.Matrix.Translation(pivot)
    rotation_matrix = mathutils.Quaternion(rot).to_matrix().to_4x4()
    translation = mathutils.Matrix.Translation(position)
    # TODO delete me?
    # print()
    # print("translation:", translation)
    # print("neg_pivot_translation:", neg_pivot_translation)
    # print("rotation_matrix:", rotation_matrix)
    # print("pivot_translation", pivot_translation)
    # print()
    # transformation_matrix = translation * neg_pivot_translation * rotation_matrix * pivot_translation
    transformation_matrix = translation * rotation_matrix * neg_pivot_translation

    rm = transformation_matrix
    # TODO: this is not good
    matrix = [
        [rm[0][0], rm[0][1], rm[0][2], rm[0][3]],
        [rm[1][0], rm[1][1], rm[1][2], rm[1][3]],
        [rm[2][0], rm[2][1], rm[2][2], rm[2][3]],
        [rm[3][0], rm[3][1], rm[3][2], rm[3][3]],
    ]
    pose_dict['matrix'] = matrix
    loc, rot, sca = transformation_matrix.decompose()
    pose_dict['translation'] = [loc.x, loc.y, loc.z]
    pose_dict['rotation_quaternion'] = [rot.w, rot.x, rot.y, rot.z]
    euler = rot.to_euler()
    pose_dict['rotation_euler'] = [euler.x, euler.y, euler.z]

    # TODO delete me?
    # translation = [px, py, pz]
    # quaternion = mathutils.Quaternion([rw, rx, ry, rz])
    # euler = quaternion.to_euler()
    ##print(euler)
    # pose_dict['translation'] = translation
    ##pose_dict['rotation_quaternion'] = [rw, rx, ry, rz]
    # pose_dict['rotation_euler'] = [euler.x, euler.y, euler.z]
    # rm = quaternion.to_matrix()
    # matrix = [[rm[0][0], rm[0][1], rm[0][2], px],
    #          [rm[1][0], rm[1][1], rm[1][2], py],
    #          [rm[2][0], rm[2][1], rm[2][2], pz],
    #          [0.0,      0.0,      0.0,      1.0]]
    #
    # pose_dict['matrix'] = matrix

    # print()
    # print('pose_dict:', pose_dict)
    # print()

    return pose_dict


def add_quaternion(rot1, rot2):
    """Adds two rotations in quaternion format and returns the result as tuple.

    Args:
      rot1(list): The first summand.
      rot2(list): The second summand.

    Returns:
      : tuple

    """
    quat1 = mathutils.Quaternion(rot1)
    quat2 = mathutils.Quaternion(rot2)
    quat_sum = quat1 @ quat2
    return (quat_sum.w, quat_sum.x, quat_sum.y, quat_sum.z)


def handle_missing_geometry(no_visual_geo, no_collision_geo, link_dict):
    """Handle missing visual and collision geometry.
    I hope it was meant like that ...
    # DOCU this documentation needs update

    Args:
      no_visual_geo: 
      no_collision_geo: 
      link_dict: 

    Returns:

    """
    if no_visual_geo or no_collision_geo:
        # TODO change to log?
        print("\n### WARNING: Missing geometry information in", link_dict['name'], ".")
    if no_visual_geo:
        # TODO change to log?
        print(
            'Affected visual elements are:',
            no_visual_geo,
            '.\n\
                Trying to get missing information from collision elements.',
        )
        for visual in no_visual_geo:
            try:
                link_dict['visual'][visual]['geometry'] = link_dict['collision'][
                    'collision' + no_visual_geo[len('visual') :]
                ]['geometry']
            except:
                # TODO: print something or handle it?
                pass
    elif no_collision_geo:
        # TODO change to log?
        print(
            'Affected collision elements are:',
            no_visual_geo,
            '.\n\
                Trying to get missing information from visual elements.',
        )
        for collision in no_collision_geo:
            try:
                link_dict['collision'][collision]['geometry'] = link_dict['visual'][
                    'visual' + no_collision_geo[len('collision') :]
                ]['geometry']
            except:
                # TODO: print something or handle it?
                pass


def get_phobos_joint_name(mars_name, has_limits):
    """This function gets a mars joint name and returns the corresponding urdf joint type.

    Args:
      mars_name(str): The mars name for the joint.
      has_limits(bool): Additional information to determine correct urdf joint name if mars_name is *hinge*.

    Returns:
      : str

    """
    if mars_name == 'hinge':
        if has_limits:
            return 'revolute'
        return 'continuous'
    if mars_name == 'slider':
        return 'prismatic'
    return 'fixed'


def parsePose(origin):
    """This function parses the robot models pose and returns it as a dictionary.

    Args:
      origin: The origin blender object to parse the pose from.

    Returns:
      : dict -- The origins pose.

    """
    pose = {}
    if origin is not None:
        try:
            pose['translation'] = gUtils.parse_text(origin.attrib['xyz'])
        except KeyError:
            pose['translation'] = [0.0, 0.0, 0.0]
        try:
            pose['rotation_euler'] = gUtils.parse_text(origin.attrib['rpy'])
        except KeyError:
            pose['rotation_euler'] = [0.0, 0.0, 0.0]
    else:
        pose['translation'] = [0.0, 0.0, 0.0]
        pose['rotation_euler'] = [0.0, 0.0, 0.0]
    return pose


def importUrdf(filepath):
    """Parses the URDF representation of the model and builds a model dictionary from it.
    
    The URDF file is opened from the filepath. If it does not exist, an empty dictionary is
    returned.

    Args:
      filepath: str

    Returns:
      dict -- model representation of the URDF file

    """
    model = {}

    log("Parsing URDF model from " + filepath, 'INFO')

    if not path.exists(filepath):
        log("Could not open URDF file. File not found: " + filepath, 'ERROR')
        return {}

    # load element tree from file
    tree = ET.parse(filepath)
    root = tree.getroot()
    model['name'] = root.attrib['name']
    if 'version' in root.attrib:
        model['version'] = root.attrib['version']

    links = {}
    log("Parsing links...", 'INFO')
    for link in root.iter('link'):
        log(" Adding link {}.".format(link.attrib['name']), 'DEBUG')
        links[link.attrib['name']] = parseLink(link, filepath)
    model['links'] = links

    joints = {}
    log("Parsing joints...", 'INFO')
    for joint in root.iter('joint'):
        # this is needed as there are "joint" tags e.g. in transmission
        if joint.find('parent') is not None:
            # parse joint from elementtree
            log(" Adding joint {} ...".format(joint.attrib['name']), 'DEBUG')
            newjoint, pose = parseJoint(joint)
            model['links'][newjoint['child']]['pose'] = pose
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
    model['joints'] = joints

    # find any links that still have no pose (most likely because they had no parent)
    for link in links:
        if 'pose' not in links[link]:
            links[link]['pose'] = parsePose(None)

    log("Parsing materials...", 'INFO')
    materials = {}
    for material in root.iter('material'):
        color = material.find('color')

        # add only materials with specified color
        if color is not None:
            log(" Adding material {}.".format(material.attrib['name']), 'DEBUG')
            newmat = {a: material.attrib[a] for a in material.attrib}
            newmat['diffuse'] = gUtils.parse_text(color.attrib['rgba'])
            newmat['specular'] = (1., 1., 1.)

            # duplicates are overwritten, but not silent
            if newmat['name'] in materials:
                log(" Overwriting duplicate material {}!".format(newmat['name']), 'WARNING')
            materials[newmat['name']] = newmat
    model['materials'] = materials

    return model


def parseLink(link, urdffilepath):
    """Parses a URDF link xml definition.

    Args:
      link(xml.etree.ElementTree.ElementTree): xml representation of the link
      urdffilepath(str): path of originating urdf file (for filename handling)

    Returns:
      : dict -- model representation of the link

    """
    newlink = {a: link.attrib[a] for a in link.attrib}
    newlink['children'] = []
    inertial = parseInertial(link)
    if inertial:
        newlink['inertial'] = inertial

    for objtype in ['visual', 'collision']:
        log('   Parsing ' + objtype + ' elements...', 'DEBUG')
        newlink[objtype] = {}
        for xmlelement in link.iter(objtype):
            # generate name for visual/collision representation
            if 'name' not in xmlelement.attrib:
                elementname = objtype + '_' + str(len(newlink[objtype])) + '_' + newlink['name']
            else:
                elementname = xmlelement.attrib['name']

            # assign values to element dictionary
            elementdict = {a: xmlelement.attrib[a] for a in xmlelement.attrib}
            elementdict['name'] = elementname
            elementdict['pose'] = parsePose(xmlelement.find('origin'))

            # gather material
            material = xmlelement.find('material')
            if material is not None:
                elementdict['material'] = material.attrib['name']

            # objects without geometry skip the last part
            geometry = xmlelement.find('geometry')
            if geometry is None:
                newlink[objtype][elementname] = elementdict
                continue

            # gather geometry information for visual/collision
            elementdict['geometry'] = {
                a: gUtils.parse_text(geometry[0].attrib[a]) for a in geometry[0].attrib
            }
            elementdict['geometry']['type'] = geometry[0].tag

            # gather mesh information
            if geometry[0].tag == 'mesh':
                # interpret filename
                filename = geometry[0].attrib['filename']
                filepath = path.normpath(path.join(path.dirname(urdffilepath), filename))
                log(
                    "     Filepath for element "
                    + elementname
                    + ': '
                    + filepath, #path.relpath(filepath, start=urdffilepath),
                    'DEBUG',
                )

                # Remove 'urdf/package://{package_name}' to workaround the lack of rospack here,
                # assuming the urdf file is in the 'urdf' folder and meshes are in the 'meshes'
                # folder at the same level.
                if 'package://' in filepath:
                    filepath = re.sub(r'(.*)urdf/package://([^/]+)/(.*)', '\\1\\3', filepath)
                elementdict['geometry']['filename'] = filepath

                # read scale
                if 'scale' in geometry[0].attrib:
                    elementdict['geometry']['scale'] = gUtils.parse_text(
                        geometry[0].attrib['scale']
                    )
                else:
                    elementdict['geometry']['scale'] = [1.0, 1.0, 1.0]
            newlink[objtype][elementname] = elementdict

    if newlink == {}:
        log("Link information for " + newlink['name'] + " is empty.", 'WARNING')
    return newlink


def parseInertial(link_xml):
    """Parses the URDF xml definition of inertial data.

    Args:
      link_xml(ElementTree.Element): xml representation of 'inertial' field of URDF link

    Returns:
      : dict -- of inertial data

    """
    inertial_dict = {}
    inertial_data = link_xml.find('inertial')
    if inertial_data is not None:  # Element.find() yields None, not []
        inertial_dict['pose'] = parsePose(inertial_data.find('origin'))
        mass = inertial_data.find('mass')
        if mass is not None:
            inertial_dict['mass'] = float(mass.attrib['value'])
        inertia = inertial_data.find('inertia')
        if inertia is not None:
            inertial_dict['inertia'] = [
                float(inertia.attrib[a]) for a in sorted(inertia.attrib.keys())
            ]
        inertial_dict['name'] = 'inertial_' + link_xml.attrib['name']
        return inertial_dict
    else:
        return None


def parseJoint(joint):
    """

    Args:
      joint: 

    Returns:

    """
    jointdict = {a: joint.attrib[a] for a in joint.attrib}
    pose = parsePose(joint.find('origin'))
    jointdict['parent'] = joint.find('parent').attrib['link']
    jointdict['child'] = joint.find('child').attrib['link']
    axis = joint.find('axis')
    if axis is not None:
        jointdict['axis'] = gUtils.parse_text(axis.attrib['xyz'])
    limit = joint.find('limit')
    if limit is not None:
        jointdict['limits'] = {a: gUtils.parse_text(limit.attrib[a]) for a in limit.attrib}
    for category in ('dynamics', 'calibration', 'safety_controller', 'mimic'):
        data = joint.find(category)
        try:
            jointdict[category] = {a: gUtils.parse_text(data.attrib[a]) for a in data.attrib}
        except AttributeError:
            pass  # no such category
    return jointdict, pose


# registering export functions of types with Phobos
entity_type_dict = {
    'urdf': {'export': exportUrdf, 'import': importUrdf, 'extensions': ('urdf', 'xml')}
}
