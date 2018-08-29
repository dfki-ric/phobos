#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.export.smurf
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadkowski

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

File smurf.py

Created on 12 Sep 2016
"""

import os
import yaml
import bpy
import phobos.defs as defs
import phobos.model.models as models
import phobos.utils.io as ioUtils
import phobos.utils.blender as bUtils
from phobos.io.entities.urdf import sort_urdf_elements
from phobos.phoboslog import log


def deriveEntity(root, outpath):
    """Derives the dictionary for a SMURF entity from the phobos model dictionary.
    
    # TODO savetosubfolder is not a parameter

    Args:
      root(bpy.types.Object): The smurf root object.
      outpath(str): The path to export the smurf to.
      savetosubfolder(bool): If True the export path has a subfolder for this smurf entity.

    Returns:
      dict - An entry for the scenes entitiesList

    """
    entitypose = models.deriveObjectPose(root)
    entity = models.initObjectProperties(root, 'entity', ['link', 'joint', 'motor'])
    if 'parent' not in entity and 'joint/type' in root and root['joint/type'] == 'fixed':
        entity['parent'] = 'world'
    entity["position"] = entitypose["translation"]
    entity["rotation"] = entitypose["rotation_quaternion"]

    # check model data if entity is a reference
    # FIXME: this part is broken but not used at the moment anyways
    if "isReference" in entity:
        entity.pop("isReference")
        bpy.ops.scene.reload_models_and_poses_operator()
        modelsPosesColl = bUtils.getPhobosPreferences().models_poses
        for robot_model in modelsPosesColl:
            if (root["model/name"] == robot_model.robot_name) and (root["entity/pose"] == robot_model.label):
                pass
        entity['file'] = os.path.join(os.path.relpath(robot_model.path, outpath), root["name"] + ".smurf")
        """
        with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
            robots = yaml.load(f.read())
            sourcepath = robots[smurf["model/name"]]
            for filename in os.listdir(sourcepath):
                fullpath = os.path.join(sourcepath, filename)
                if os.path.isfile(fullpath):
                    shutil.copy2(fullpath, os.path.join(smurf_outpath, filename))
                else:
                    # remove old folders to prevent errors in copytree
                    shutil.rmtree(os.path.join(smurf_outpath, filename), True)
                    shutil.copytree(fullpath, os.path.join(smurf_outpath, filename))
        """
    else:
        modelpath = os.path.join(outpath, root['model/name'], 'smurf')
        # TODO why the spacing between the paths?
        log("Scene paths: " + outpath + ' ' + modelpath, "DEBUG")
        entity['file'] = os.path.join(os.path.relpath(modelpath, os.path.dirname(outpath)), root['model/name']+".smurf")
    return entity


def gatherAnnotations(model):
    """Gathers custom properties annotating elements of the robot
    across the model. These annotations were created in the model.py
    module and are marked with a leading '$'.

    Args:
      model(dict): The robot model dictionary.

    Returns:
      dict -- A dictionary of the gathered annotations.

    """
    # TODO check this stuff
    annotations = {}
    elementlist = []
    types = ('links', 'joints', 'sensors', 'motors', 'controllers', 'materials')
    # gather information from directly accessible types
    for objtype in types:
        for elementname in model[objtype]:
            tmpdict = model[objtype][elementname]
            tmpdict['temp_type'] = objtype[:-1]
            elementlist.append(tmpdict)

    # add information from types hidden in links
    for linkname in model['links']:
        for objtype in ('collision', 'visual'):
            if objtype in model['links'][linkname]:
                for elementname in model['links'][linkname][objtype]:
                    tmpdict = model['links'][linkname][objtype][elementname]
                    tmpdict['temp_type'] = objtype
                    elementlist.append(tmpdict)
        if 'inertial' in model['links'][linkname]:
            tmpdict = model['links'][linkname]['inertial']
            tmpdict['temp_type'] = 'inertial'
            elementlist.append(tmpdict)

    # loop through the list of annotated elements and categorize the data
    for element in elementlist:
        delkeys = []
        for key in element.keys():
            if key.startswith('$'):
                category = key[1:]
                if category not in annotations:
                    annotations[category] = {}
                if element['temp_type'] not in annotations[category]:
                    annotations[category][element['temp_type']] = []
                tmpdict = {k: element[key][k] for k in element[key]}
                tmpdict['name'] = element['name']
                annotations[category][element['temp_type']].append(tmpdict)
                delkeys.append(key)
        delkeys.append('temp_type')
        for key in delkeys:
            if key in element:
                del element[key]
    return annotations


def deriveRefinedCollisionData(model):
    """This function collects all collision bitmasks in a given model.

    Args:
      model(dict): The robot model to search in.

    Returns:
      dict -- a dictionary containing all bitmasks with corresponding element name (key).

    """
    collisiondata = {}
    for linkname in model['links']:
        for elementname in model['links'][linkname]['collision']:
            element = model['links'][linkname]['collision'][elementname]
            data = {key: element[key] for key in (a for a in element.keys() if a not in ['geometry', 'name', 'pose'])}
            if data:
                data['name'] = model['links'][linkname]['collision'][elementname]['name']
                data['link'] = linkname
                collisiondata[elementname] = data
    return collisiondata


def gatherLevelOfDetailSettings(model):
    """This function collects all level of detail settings in a given model.

    Args:
      model: The robot model to search in.

    Returns:
      dict -- a dictionary containing all bitmasks with corresponding element name (key).

    """
    lods = {}
    for linkname in model['links']:
        for elementname in model['links'][linkname]['visual']:
            element = model['links'][linkname]['visual'][elementname]
            if 'lod' in element:
                lods[elementname] = {'name': elementname, 'lod': element['lod']}
    return lods


def sort_for_yaml_dump(dictionary, category):
    """Sorts the objects of the specified category in the dictionary and returns them.

    If the category sorting is unknown, return the dictionary instead.

    Supported categories are: *materials*, *motors*, *sensors*, *simulation*.

    Args:
        structure (dict): dictionary to sort
        category (str): category of the dictionary to sort

    Returns:
        list -- the elements of the specified category of the original dictionary in sorted order
    """
    if category in ['materials', 'motors']:
        return {category: sort_dict_list(dictionary[category], 'name')}
    elif category == 'sensors':
        dictionary[category] = replace_object_links(dictionary[category])

        return {category: sort_dict_list(dictionary[category], 'name')}
    elif category == 'simulation':
        return_dict = {}
        for viscol in ['collision', 'visual']:
            return_dict[viscol] = sort_dict_list(dictionary[viscol], 'name')
        return return_dict
    return dictionary


def replace_object_links(dictionary):
    if isinstance(dictionary, list):
        newlist = []
        for item in dictionary:
            newlist.append(replace_object_links(item))
        return newlist

    for key, value in dictionary.items():
        if isinstance(value, list):
            if (all([isinstance(item, dict) for item in value]) and
                    all([('name' in item and 'object' in item) for item in value])):
                dictionary[key] = sorted([item['name'] for item in value])

        elif isinstance(value, dict):
            replace_object_links(value)

    return dictionary


def sort_dict_list(dict_list, sort_key):
    """TODO Please add doc ASAP

    Args:
      dict_list: param sort_key:
      sort_key: 

    Returns:

    """
    sorted_dict_list = []
    sort_key_values = []
    for dictionary in dict_list:
        sort_key_values.append(dictionary[sort_key])
    #FIXME: This is really complicated! Either there is an in-built function or dictionary['name'] would suffice
    for value in sort_urdf_elements(sort_key_values):
        for dictionary in dict_list:
            if dictionary[sort_key] == value:
                sorted_dict_list.append(dictionary)
                break
        # TODO: delete found dictionary to save time
    return sorted_dict_list


def exportSmurf(model, path):
    """This function exports a given model to a specific path as a smurf representation.

    :param model: The model you want to export.
    :type model: dict
    :param path: The path you want to save the smurf file *without file name!*
    :type param: str

    """
    collisiondata = deriveRefinedCollisionData(model)
    # TODO delete me?
    # capsules = []
    # capsules = gatherCollisionCapsules(model)
    lodsettings = gatherLevelOfDetailSettings(model)

    exportdata = {'state': False,  # model['state'] != {}, # TODO: handle state
                  'materials': model['materials'] != {},
                  'sensors': model['sensors'] != {},
                  'motors': model['motors'] != {},
                  'controllers': model['controllers'] != {},
                  'collision': collisiondata != {},
                  'visuals': lodsettings != {},
                  'lights': model['lights'] != {},
                  'submechanisms': model['submechanisms'] != []
                  }

    # create all filenames
    smurf_filename = model['name'] + ".smurf"
    filenames = {'state': model['name'] + "_state.yml",
                 'materials': model['name'] + "_materials.yml",
                 'sensors': model['name'] + "_sensors.yml",
                 'motors': model['name'] + "_motors.yml",
                 'controllers': model['name'] + "_controllers.yml",
                 'collision': model['name'] + "_collision.yml",
                 'visuals': model['name'] + "_visuals.yml",
                 'lights': model['name'] + "_lights.yml",
                 'submechanisms': model['name'] + "_submechanisms.yml"
                 }
    fileorder = ['collision', 'visuals', 'materials', 'motors', 'sensors',
                 'controllers', 'state', 'lights', 'submechanisms']
    urdf_path = '../urdf/'
    urdf_filename = model['name'] + '.urdf'

    # gather annotations and data from text files
    annotationdict = gatherAnnotations(model)

    # $mars annotated properties overwrite custom properties of objects for smurf
    if 'mars' in annotationdict:
        for category in annotationdict['mars']:
            for list_obj in annotationdict['mars'][category]:
                model[category + 's'][list_obj['name']].update(list_obj)
        del annotationdict['mars']

    for category in annotationdict:
        # TODO use os.path?
        filenames[category] = model['name'] + '_' + category + '.yml'
        fileorder.append(category)
        exportdata[category] = True

    customdatalist = []
    for text in bpy.data.texts:
        if text.name.startswith(model['name']+'::'):
            dataname = text.name.split('::')[-1]
            customdatalist.append(dataname)
            # TODO use os.path?
            filenames[dataname] = model['name'] + '_' + dataname + '.yml'
            fileorder.append(dataname)
            exportdata[dataname] = True

    infostring = ' definition SMURF file for "' + model['name'] + '", ' + model["date"] + "\n\n"

    # write model information
    log("Writing SMURF model to " + smurf_filename, "INFO")
    # CHECK are these filepaths failsafe in Windows?
    modeldata = {"date": model["date"],
                 "files": [urdf_path + urdf_filename] + [filenames[f] for f in fileorder if exportdata[f]]}
    # append custom data
    with open(os.path.join(path, smurf_filename), 'w') as op:
        op.write('# main SMURF file of model "' + model['name'] + '"\n')
        op.write('# created with Phobos ' + defs.version + ' - ' + defs.repository + '\n\n')
        op.write("SMURF version: " + defs.version + "\n")
        op.write("modelname: " + model['name'] + "\n")
        op.write(yaml.dump(modeldata, default_flow_style=False))

    # TODO delete me?
    # #write semantics (SRDF information in YML format)
    # if export['semantics']:
    #     with open(path + filenames['semantics'], 'w') as op:
    #         op.write('#semantics'+infostring)
    #         op.write("modelname: "+model['name']+'\n')
    #         semantics = {}
    #         if model['groups'] != {}:
    #             semantics['groups'] = model['groups']
    #         if model['chains'] != {}:
    #             semantics['chains'] = model['chains']
    #         op.write(yaml.dump(semantics, default_flow_style=False))

    # TODO: implement everything but joints
    # write state (state information of all joints, sensor & motor activity etc.)
    if exportdata['state']:
        states = []
        # gather all states
        for jointname in model['joints']:
            joint = model['joints'][jointname]
            # this should always be the case, but testing doesn't hurt
            if 'state' in joint:
                tmpstate = joint['state'].copy()
                tmpstate['name'] = jointname
                states.append(joint['state'])
        with open(os.path.join(path, filenames['state']), 'w') as op:
            op.write('#state' + infostring)
            op.write("modelname: " + model['name'] + '\n')
            # TODO am I still needed?
            op.write(yaml.dump(states))  # , default_flow_style=False))

    # write materials, sensors, motors & controllers
    for data in ['materials', 'sensors', 'motors', 'controllers', 'lights']:
        if exportdata[data]:
            log("Writing {} to smurf file.".format(data), 'DEBUG')
            with open(os.path.join(path, filenames[data]), 'w') as op:
                op.write('#' + data + infostring)

                op.write(yaml.dump(sort_for_yaml_dump({data: list(model[data].values())}, data),
                                   default_flow_style=False))

    # write additional collision information
    if exportdata['collision']:
        with open(os.path.join(path, filenames['collision']), 'w') as op:
            op.write('#collision data' + infostring)
            # TODO delete me?
            #op.write(yaml.dump({'collision': list(bitmasks.values())}, default_flow_style=False))
            op.write(yaml.dump({'collision': [collisiondata[key] for key in sorted(collisiondata.keys())]},
                               default_flow_style=False))

    # write visual information (level of detail, ...)
    if exportdata['visuals']:
        with open(os.path.join(path, filenames['visuals']), 'w') as op:
            op.write('#visual data' + infostring)
            op.write(yaml.dump({'visuals': list(lodsettings.values())}, default_flow_style=False))

    # write additional information
    for category in annotationdict.keys():
        if exportdata[category]:
            outstring = '#' + category + infostring
            for elementtype in annotationdict[category]:
                outstring += elementtype + ':\n'
                outstring += yaml.dump(annotationdict[category][elementtype],
                                       default_flow_style=False) + "\n"
            with open(os.path.join(path, filenames[category]), 'w') as op:
                op.write(outstring)

    # write custom data from textfiles
    for data in customdatalist:
        if exportdata[data]:
            with open(os.path.join(path, filenames[data]), 'w') as op:
                op.write('#' + data + infostring)
                op.write(yaml.dump({data: list(model[data].values())}, default_flow_style=False))

    # write submechanisms
    if model['submechanisms']:
        with open(os.path.join(path, filenames['submechanisms']), 'w') as op:
            op.write('#submechanisms' + infostring)
            op.write(yaml.dump({'submechanisms': model['submechanisms']}))#, default_flow_style=False))

    # TODO delete me?
    ## write custom yml files
    #if bpy.data.window_managers[0].exportCustomData:
    #    log("Exporting custom files to to " + path + "...", "INFO")
    #    for text in customtexts:
    #        with open(os.path.join(path, text.name), 'w') as op:
    #            op.write('\n'.join(line.body for line in text.lines))


# CHECK why is this class disabled?
# class SMURFModelParser(RobotModelParser):
#     """Class derived from RobotModelParser which parses a SMURF model"""
#
#     def __init__(self, filepath):
#         RobotModelParser.__init__(self, filepath)
#
#     def parseModel(self):
#         print("Parsing SMURF model...")
#         #smurf = None
#         # TODO check for filename consistency (for Windows)
#         with open(self.filepath, 'r') as smurffile:
#             smurf = yaml.load(smurffile)
#         if smurf is None:
#             log('No valid SMURF file.', "ERROR")
#             return None
#         urdffile = None
#         srdffile = None
#         # TODO check filename consistency (Windows)
#         ymlfiles = [f for f in smurf['files'] if f.endswith('.yml') or f.endswith('.yaml')]
#         for f in smurf['files']:
#             if f.endswith('.urdf'):
#                 urdffile = f
#             if f.endswith('.srdf'):
#                 srdffile = f
#         # get URDF info
#         if urdffile is None:
#             log("Did not find URDF file associated with SMURF.", "ERROR")
#             return None
#         urdfparser = URDFModelParser(os.path.join(self.path, urdffile))
#         urdfparser.parseModel()
#         if srdffile is not None:
#             srdfparser = SRDFModelParser(os.path.join(self.path, srdffile))
#             self.robot = srdfparser.parseModel(urdfparser.robot)
#         else:
#             self.robot = urdfparser.robot
#         # make sure all types exist
#         typelist = ['links', 'joints', 'materials', 'sensors', 'motors', 'controllers', 'groups', 'chains']
#         for key in typelist:
#             if key not in self.robot:
#                 self.robot[key] = {}
#         #add the smurf information
#         custom_dicts = {}
#         for yml in ymlfiles:
#             with open(os.path.join(self.path, yml), 'r') as ymlfile:
#                 ymldict = yaml.load(ymlfile)
#             for key in ymldict:
#                 print(key)
#                 if key in ['materials', 'sensors', 'motors', 'controllers']:
#                     for element in ymldict[key]:
#                         if element['name'] not in self.robot[key]:
#                             self.robot[key][element['name']] = element
#                         else:
#                             for tag in element:
#                                 self.robot[key][element['name']][tag] = element[tag]
#                 elif key in 'state':
#                 # TODO: handle state
#                     pass
#                 else:
#                     custom_dicts[key] = ymldict[key]
#
#         for key in custom_dicts:
#             print('assign custom properties:', key, custom_dicts[key])
#             for element in custom_dicts[key]:  # iterate over list with custom annotations
#                 print(element)
#                 try:
#                     objtype = element['type']
#                 except KeyError:
#                     log("Could not find 'type' in custom annotation: " + str(element), "ERROR")
#                 try:
#                     objname = element['name']
#                 except KeyError:
#                     log("Could not find 'name' in custom annotation: " + str(element), "ERROR")
#                 try:
#                 #FIXME: this is a total hack!
#                     if objtype+'s' in typelist:
#                         objtype += 's'
#                     else:
#                         raise TypeError(objtype)
#                     if objname in self.robot[objtype]:
#                         for tag in element:
#                             if tag not in ['type', 'name']:
#                                 if not '$'+key in self.robot[objtype][objname]:
#                                     self.robot[objtype][objname]['$'+key] = {tag: element[tag]}
#                                 else:
#                                     self.robot[objtype][objname]['$'+key][tag] = element[tag]
#                     else:
#                         raise NameError(objname)
#                 except TypeError:
#                     print("ERROR: could not find 'type' or 'name' in custom annotation", objtype, objname)
#                 except NameError:
#                     log("Element " + str(objname) + " of type " + str(objtype) + " does not exist in this model.", "ERROR")
#
#         #now some debug output
#         with open(self.filepath+'_SMURF_debug.yml', 'w') as outputfile:
#             outputfile.write(yaml.dump(self.robot))#, default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries


# registering import/export functions of types with Phobos
entity_type_dict = {'smurf': {'export': exportSmurf,
                              'derive': deriveEntity,
                              'extensions': ('smurf',)}
                    }
