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
import json
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
      : dict - An entry for the scenes entitiesList

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
            if (root["model/name"] == robot_model.robot_name) and (
                root["entity/pose"] == robot_model.label
            ):
                pass
        entity['file'] = os.path.join(
            os.path.relpath(robot_model.path, outpath), root["name"] + ".smurf"
        )
        """
        with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
            robots = json.loads(f.read())
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
        entity['file'] = os.path.join(
            os.path.relpath(modelpath, os.path.dirname(outpath)), root['model/name'] + ".smurf"
        )
    return entity


def deriveRefinedCollisionData(model):
    """This function collects all collision bitmasks in a given model.

    Args:
      model(dict): The robot model to search in.

    Returns:
      : dict -- a dictionary containing all bitmasks with corresponding element name (key).

    """
    collisiondata = {}
    for linkname in model['links']:
        for elementname in model['links'][linkname]['collision']:
            element = model['links'][linkname]['collision'][elementname]
            data = {
                key: element[key]
                for key in (a for a in element.keys() if a not in ['geometry', 'name', 'pose'])
            }
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
      : dict -- a dictionary containing all bitmasks with corresponding element name (key).

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
      structure(dict): dictionary to sort
      category(str): category of the dictionary to sort
      dictionary:

    Returns:
      : list -- the elements of the specified category of the original dictionary in sorted order

    """
    if category in ['materials', 'motors']:
        return {category: sort_dict_list(dictionary[category], 'name')}
    elif category == 'sensors':
        dictionary[category] = models.replace_object_links(dictionary[category])

        return {category: sort_dict_list(dictionary[category], 'name')}
    elif category == 'simulation':
        return_dict = {}
        for viscol in ['collision', 'visual']:
            return_dict[viscol] = sort_dict_list(dictionary[viscol], 'name')
        return return_dict
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
    # FIXME: This is really complicated! Either there is an in-built function or dictionary['name'] would suffice
    for value in sort_urdf_elements(sort_key_values):
        for dictionary in dict_list:
            if dictionary[sort_key] == value:
                sorted_dict_list.append(dictionary)
                break
        # TODO: delete found dictionary to save time
    return sorted_dict_list


def exportSmurf(model, path):
    """This function exports a given model to a specific path as a smurf representation.

    Args:
      model(dict): The model you want to export.
      path: The path you want to save the smurf file *without file name!*

    Returns:

    """
    collisiondata = deriveRefinedCollisionData(model)
    lodsettings = gatherLevelOfDetailSettings(model)

    exportdata = {
        'state': False,  # model['state'] != {}, # TODO: handle state
        'materials': model['materials'] != {},
        'sensors': model['sensors'] != {},
        'motors': model['motors'] != {},
        'controllers': model['controllers'] != {},
        'collision': collisiondata != {},
        'visuals': lodsettings != {},
        'lights': model['lights'] != {},
        'submechanisms': model['submechanisms'] != [],
    }

    # create all filenames
    smurf_filename = model['name'] + ".smurf"
    filenames = {
        'state': model['name'] + "_state.yml",
        'materials': model['name'] + "_materials.yml",
        'sensors': model['name'] + "_sensors.yml",
        'motors': model['name'] + "_motors.yml",
        'controllers': model['name'] + "_controllers.yml",
        'collision': model['name'] + "_collision.yml",
        'visuals': model['name'] + "_visuals.yml",
        'lights': model['name'] + "_lights.yml",
        'submechanisms': model['name'] + "_submechanisms.yml",
    }
    fileorder = [
        'collision',
        'visuals',
        'materials',
        'motors',
        'sensors',
        'controllers',
        'state',
        'lights',
        'submechanisms',
    ]
    urdf_path = '../urdf/'
    urdf_filename = model['name'] + '.urdf'

    # gather annotations and data from text files
    annotationdict = models.gatherAnnotations(model)

    # $mars annotated properties overwrite custom properties of objects for smurf
    if 'mars' in annotationdict:
        for category in annotationdict['mars']:
            for list_obj in annotationdict['mars'][category]:
                model[category + 's'][list_obj['name']].update(list_obj)
        del annotationdict['mars']

    for category in annotationdict:
        # TODO use os.path?
        if category != 'sdf':
            filenames[category] = model['name'] + '_' + category + '.yml'
            fileorder.append(category)
            exportdata[category] = True

    customdatalist = []
    for text in bpy.data.texts:
        if text.name.startswith(model['name'] + '::'):
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
    modeldata = {
        "SMURF version": defs.version,
        "modelname": model['name'],
        "date": model["date"],
        "files": [urdf_path + urdf_filename] + [filenames[f] for f in fileorder if exportdata[f]],
    }
    # append custom data
    with open(os.path.join(path, smurf_filename), 'w') as op:
        op.write('# main SMURF file of model "' + model['name'] + '"\n')
        op.write('# created with Phobos ' + defs.version + ' - ' + defs.repository + '\n\n')
        op.write(json.dumps(modeldata, indent=2))

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
    #         op.write(json.dumps(semantics, indent=2))

    # for smurf, we parse the controller parameters into the motors

    for motor in model['motors']:
        motordict = model['motors'][motor]
        controllerparams = {}
        if 'controller' in  motordict and motordict['controller'] in model['controllers']:
            controllerparams = {
                key: value
                for key, value in model['controllers'][motordict['controller']].items()
                if (key not in ['name', 'target'])
            }
            motordict.update(controllerparams)
            del motordict['controller']
        else:
            log("No controller assigned to motor {}!".format(motordict['name']), 'WARNING')

        # PID controlled motors get their min and max values from the joint limits
        if motordict['type'] == 'PID':
            try:
                joint = model['joints'][motordict['joint']]
                if 'limits' in joint:
                    motordict['minValue'] = joint['limits']['lower']
                    motordict['maxValue'] = joint['limits']['upper']
            except KeyError:
                log(
                    "Missing data in motor {}! No limits given for type PID. Motor might be incomplete.".format(
                        motordict['name']
                    ),
                    "WARNING",
                )
        # direct controllers are called generic_dc in mars
        elif motordict['type'] == 'direct':
            motordict['type'] = 'generic_dc'
            try:
                motordict['minValue'] = -1. * motordict["maxSpeed"]
                motordict['maxValue'] = motordict["maxSpeed"]
            except KeyError:
                log(
                    "Missing data in motor {}! No maxSpeed given for motor type direct. Motor might be incomplete.".format(
                        motordict['name']
                    ),
                    "WARNING",
                )

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
            op.write(json.dumps(states, indent=2))

    # write materials, sensors, motors & controllers
    for data in ['materials', 'motors', 'sensors', 'controllers', 'lights']:
        if exportdata[data]:
            log("Writing {} to smurf file.".format(data), 'DEBUG')
            with open(os.path.join(path, filenames[data]), 'w') as op:
                op.write('#' + data + infostring)
                op.write(
                    json.dumps(
                        sort_for_yaml_dump({data: list(model[data].values())}, data),
                        indent=2
                    )
                )

    # write additional collision information
    if exportdata['collision']:
        with open(os.path.join(path, filenames['collision']), 'w') as op:
            op.write('#collision data' + infostring)
            # TODO delete me?
            # op.write(json.dumps({'collision': list(bitmasks.values())}, indent=2))
            op.write(
                json.dumps(
                    {'collision': [collisiondata[key] for key in sorted(collisiondata.keys())]},
                    indent=2
                )
            )

    # write visual information (level of detail, ...)
    if exportdata['visuals']:
        with open(os.path.join(path, filenames['visuals']), 'w') as op:
            op.write('#visual data' + infostring)
            op.write(json.dumps({'visuals': list(lodsettings.values())}, indent=2))

    # write additional information
    for category in annotationdict.keys():
        if category != 'sdf':
            if exportdata[category]:
                outstring = '#' + category + infostring
                for elementtype in annotationdict[category]:
                    outstring += elementtype + ':\n'
                    outstring += (
                        json.dumps(annotationdict[category][elementtype], indent=2)
                        + "\n"
                    )
                with open(os.path.join(path, filenames[category]), 'w') as op:
                    op.write(outstring)

    # write custom data from textfiles
    for data in customdatalist:
        if exportdata[data]:
            with open(os.path.join(path, filenames[data]), 'w') as op:
                op.write('#' + data + infostring)
                op.write(json.dumps({data: list(model[data].values())}, indent=2))

    # write submechanisms
    if model['submechanisms']:
        with open(os.path.join(path, filenames['submechanisms']), 'w') as op:
            op.write('#submechanisms' + infostring)
            op.write(
                json.dumps({'submechanisms': model['submechanisms']}, indent=2)
            )

    # TODO delete me?
    ## write custom yml files
    # if bpy.data.window_managers[0].exportCustomData:
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
#             smurf = json.loads(smurffile)
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
#                 ymldict = json.loads(ymlfile)
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
#             outputfile.write(json.dumps(self.robot)) #last parameter prevents inline formatting for lists and dictionaries


# registering import/export functions of types with Phobos
entity_type_dict = {
    'smurf': {'export': exportSmurf, 'derive': deriveEntity, 'extensions': ('smurf',)}
}
