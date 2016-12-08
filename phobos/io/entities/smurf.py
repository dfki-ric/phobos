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
import shutil
import yaml
from datetime import datetime
import bpy
import phobos.model.models as models
import phobos.utils.selection as sUtils
import phobos.io.meshes as meshes
import phobos.defs as defs
from phobos.utils.io import securepath
from phobos.io.entities.urdf import sort_urdf_elements
from phobos.phoboslog import log


def deriveEntity(entity, outpath, savetosubfolder):
    """Derives the dictionary for a SMURF entity.

    :param entity: The smurf root object.
    :type entity: bpy.types.Object
    :param outpath: The path to export the smurf to.
    :type outpath: str
    :param savetosubfolder: If True the export path has a subfolder for this smurf entity.
    :type savetosubfolder: bool
    :return: dict - An entry for the scenes entitiesList

    """

    smurf = entity

    # determine outpath for the smurf export
    # differentiate between full model and baked reference
    if "entity/isReference" in smurf:
        bpy.ops.scene.reload_models_and_poses_operator()
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        for robot_model in modelsPosesColl:
            if (smurf["modelname"] == robot_model.robot_name) and (smurf["entity/pose"] == robot_model.label):
                entitypose = models.deriveObjectPose(smurf)
                entry      = models.initObjectProperties(smurf, 'entity', ['link', 'joint', 'motor'])
                entry.pop("isReference");

                entry['file'] = os.path.join(os.path.relpath(robot_model.path,outpath), smurf["modelname"] + ".smurf")
                if 'parent' not in entry and 'joint/type' in smurf and smurf['joint/type'] == 'fixed':
                    entry['parent'] = 'world'
                entry["position"] = entitypose["translation"]
                entry["rotation"] = entitypose["rotation_quaternion"]

        '''
        with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
            robots = yaml.load(f.read())
            sourcepath = robots[smurf["modelname"]]
            for filename in os.listdir(sourcepath):
                fullpath = os.path.join(sourcepath, filename)
                if os.path.isfile(fullpath):
                    shutil.copy2(fullpath, os.path.join(smurf_outpath, filename))
                else:
                    # remove old folders to prevent errors in copytree
                    shutil.rmtree(os.path.join(smurf_outpath, filename), True)
                    shutil.copytree(fullpath, os.path.join(smurf_outpath, filename))
        '''
    else:
        smurf_outpath = securepath(os.path.join(outpath, entity["modelname"]) if savetosubfolder else outpath)
        log("smurf_outpath: " + outpath, "DEBUG", "exportSMURFsScene")

        log("Exporting " + smurf["entity/name"] + " as a smurf entity to " + smurf_outpath, "INFO", "deriveSMURFEntity",
            "\n\n")
        subfolder = smurf["modelname"] if savetosubfolder else ""
        sUtils.selectObjects(sUtils.getChildren(smurf), clear=True)
        sUtils.selectObjects(sUtils.getChildren(smurf), clear=True)  # re-select for mesh export
        model, objectlist = models.buildModelDictionary(smurf)
        export(model, objectlist, smurf_outpath)
        entitypose = models.deriveObjectPose(smurf)
        entry = models.initObjectProperties(smurf, 'entity', ['link', 'joint', 'motor'])

        entry['file'] = (os.path.join(subfolder, smurf["modelname"]+".smurf") if os.path.isfile(smurf_outpath)
                         else os.path.join(subfolder, "smurf", smurf["modelname"]+".smurf"))
        if 'parent' not in entry and 'joint/type' in smurf and smurf['joint/type'] == 'fixed':
            entry['parent'] = 'world'
        entry["position"] = entitypose["translation"]
        entry["rotation"] = entitypose["rotation_quaternion"]
    return entry





def gatherAnnotations(model):
    """This function gathers custom properties annotating elements of the robot
    across the model. These annotations were created in the model.py
    module and are marked with a leading '$'.

    :param model: The robot model dictionary.
    :type model: dict
    :return: dict -- A dictionary of the gathered annotations.

    """
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
            del element[key]
    return annotations


def deriveRefinedCollisionData(model):
    """This function collects all collision bitmasks in a given model.

    :param model: The robot model to search in.
    :type model: dict
    :return: dict -- a dictionary containing all bitmasks with corresponding element name (key).

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

    :param model: The robot model to search in.
    :return: dict -- a dictionary containing all bitmasks with corresponding element name (key).

    """
    lods = {}
    for linkname in model['links']:
        for elementname in model['links'][linkname]['visual']:
            element = model['links'][linkname]['visual'][elementname]
            if 'lod' in element:
                lods[elementname] = {'name': elementname, 'lod': element['lod']}
    return lods


def sort_for_yaml_dump(structure, category):
    """Please add doc ASAP
    :param structure:
    :param category:
    :return:

    """
    if category in ['materials', 'motors', 'sensors']:
        return {category: sort_dict_list(structure[category], 'name')}
    elif category == 'simulation':
        return_dict = {}
        for viscol in ['collision', 'visual']:
            return_dict[viscol] = sort_dict_list(structure[viscol], 'name')
        return return_dict
    else:
        return structure


def sort_dict_list(dict_list, sort_key):
    """Please add doc ASAP
    :param dict_list:
    :param sort_key:
    :return:

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



def exportModelToSMURF(model, path):
    """This function exports a given model to a specific path as a smurf representation.

    :param model: The model you want to export.
    :type model: dict
    :param path: The path you want to save the smurf file *without file name!*
    :type param: str

    """
    collisiondata = deriveRefinedCollisionData(model)
    capsules = []
    #capsules = gatherCollisionCapsules(model)
    lodsettings = gatherLevelOfDetailSettings(model)

    exportdata = {'state': False,  # model['state'] != {}, # TODO: handle state
              'materials': model['materials'] != {},
              'sensors': model['sensors'] != {},
              'motors': model['motors'] != {},
              'controllers': model['controllers'] != {},
              'collision': collisiondata != {},
              'visuals': lodsettings != {},
              'lights': model['lights'] != {}
              }
    # create all filenames
    smurf_filename = model['modelname'] + ".smurf"
    if bpy.data.worlds[0].phobosexportsettings.structureExport:
        urdf_filename = "../urdf/" + model['modelname'] + ".urdf"
    else:
        urdf_filename = model['modelname'] + ".urdf"
    filenames = {'state': model['modelname'] + "_state.yml",
                 'materials': model['modelname'] + "_materials.yml",
                 'sensors': model['modelname'] + "_sensors.yml",
                 'motors': model['modelname'] + "_motors.yml",
                 'controllers': model['modelname'] + "_controllers.yml",
                 'collision': model['modelname'] + "_collision.yml",
                 'visuals': model['modelname'] + "_visuals.yml",
                 'lights': model['modelname'] + "_lights.yml",
                 }
    fileorder = ['collision', 'visuals', 'materials', 'motors', 'sensors', 'controllers', 'state', 'lights']

    # gather annotations and data from text files
    annotationdict = gatherAnnotations(model)
    for category in annotationdict:
        filenames[category] = model['modelname'] + '_' + category + '.yml'
        fileorder.append(category)
        exportdata[category] = True

    customdatalist = []
    for text in bpy.data.texts:
        if text.name.startswith(model['modelname']+'::'):
            dataname = text.name.split('::')[-1]
            customdatalist.append(dataname)
            filenames[dataname] = model['modelname'] + '_' + dataname + '.yml'
            fileorder.append(dataname)
            exportdata[dataname] = True

    infostring = ' definition SMURF file for "' + model['modelname'] + '", ' + model["date"] + "\n\n"

    # write model information
    log("Writing SMURF model to " + smurf_filename, "INFO", "exportModelToSMURF")
    modeldata = {"date": model["date"], "files": [urdf_filename] + [filenames[f] for f in fileorder if exportdata[f]]}
    # append custom data
    with open(os.path.join(path, smurf_filename), 'w') as op:
        op.write('# main SMURF file of model "' + model['modelname'] + '"\n')
        op.write('# created with Phobos ' + defs.version + ' - https://github.com/rock-simulation/phobos\n\n')
        op.write("SMURF version: " + defs.version + "\n")
        op.write("modelname: " + model['modelname'] + "\n")
        op.write(yaml.dump(modeldata, default_flow_style=False))

    # write urdf
    exportModelToURDF(model, os.path.join(path, urdf_filename))

    # #write semantics (SRDF information in YML format)
    # if export['semantics']:
    #     with open(path + filenames['semantics'], 'w') as op:
    #         op.write('#semantics'+infostring)
    #         op.write("modelname: "+model['modelname']+'\n')
    #         semantics = {}
    #         if model['groups'] != {}:
    #             semantics['groups'] = model['groups']
    #         if model['chains'] != {}:
    #             semantics['chains'] = model['chains']
    #         op.write(yaml.dump(semantics, default_flow_style=False))

    # write state (state information of all joints, sensor & motor activity etc.) #TODO: implement everything but joints
    if exportdata['state']:
        states = []
        # gather all states
        for jointname in model['joints']:
            joint = model['joints'][jointname]
            if 'state' in joint:  # this should always be the case, but testing doesn't hurt
                tmpstate = joint['state'].copy()
                tmpstate['name'] = jointname
                states.append(joint['state'])
        with open(path + filenames['state'], 'w') as op:
            op.write('#state' + infostring)
            op.write("modelname: " + model['modelname'] + '\n')
            op.write(yaml.dump(states))  #, default_flow_style=False))

    # write materials, sensors, motors & controllers
    for data in ['materials', 'sensors', 'motors', 'controllers', 'lights']:
        if exportdata[data]:
            with open(path + filenames[data], 'w') as op:
                op.write('#' + data + infostring)
                op.write(yaml.dump(sort_for_yaml_dump({data: list(model[data].values())}, data),
                                   default_flow_style=False))
                #op.write(yaml.dump({data: list(model[data].values())}, default_flow_style=False))

    # write additional collision information
    if exportdata['collision']:
        with open(path + filenames['collision'], 'w') as op:
            op.write('#collision data' + infostring)
            #op.write(yaml.dump({'collision': list(bitmasks.values())}, default_flow_style=False))
            op.write(yaml.dump({'collision': [collisiondata[key] for key in sorted(collisiondata.keys())]},
                               default_flow_style=False))

    # write visual information (level of detail, ...)
    if exportdata['visuals']:
        with open(path + filenames['visuals'], 'w') as op:
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
            with open(path + filenames[category], 'w') as op:
                op.write(outstring)

    # write custom data from textfiles
    for data in customdatalist:
        if exportdata[data]:
            with open(path + filenames[data], 'w') as op:
                op.write('#' + data + infostring)
                op.write(yaml.dump({data: list(model[data].values())}, default_flow_style=False))

    ## write custom yml files
    #if bpy.data.worlds[0].exportCustomData:
    #    log("Exporting custom files to to " + path + "...", "INFO", "exportModelToSMURF")
    #    for text in customtexts:
    #        with open(os.path.join(path, text.name), 'w') as op:
    #            op.write('\n'.join(line.body for line in text.lines))





