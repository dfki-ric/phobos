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
import itertools
import yaml
from datetime import datetime
import bpy
import phobos.robotdictionary as robotdictionary
import phobos.utils.selection as sUtils
import phobos.utils.export as eUtils
import phobos.defs as defs
from phobos.utils.general import securepath
from phobos.logging import log


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
                entitypose = robotdictionary.deriveObjectPose(smurf)
                entry      = robotdictionary.initObjectProperties(smurf, 'entity', ['link', 'joint', 'motor'])
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
        model, objectlist = robotdictionary.buildModelDictionary(smurf)
        export(model, objectlist, smurf_outpath)
        entitypose = robotdictionary.deriveObjectPose(smurf)
        entry = robotdictionary.initObjectProperties(smurf, 'entity', ['link', 'joint', 'motor'])

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


def sort_urdf_elements(elems):
    """
    Sort a collection of elements. By default, this method simply wraps the standard 'sorted()' method.
    This is done in order to be able to easily change the element ordering.

    :param elems: a collection
    :return: sorted colletion
    """
    return sorted(elems)


def get_sorted_keys(dictionary):
    """Sorts a dictionaries keys.

    :param dictionary: The dictionary to sort the keys from
    :type dictionary: dict
    :return: the dictionary's sorted keys

    """
    return sorted(dictionary.keys())


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


def exportModelToYAML(model, filepath):
    """This function exports a given robot model to a specified filepath as YAML.

    :param model: The robot model to export
    :type model: dict -- the generated robot model dictionary
    :param filepath:  The filepath to export the robot to. *WITH filename!*
    :type filepath: str

    """
    log("phobos YAML export: Writing model data to " + filepath, "INFO", "exportModelToYAML")
    with open(filepath, 'w') as outputfile:
        outputfile.write('# YAML dump of robot model "' + model['modelname'] + '", ' + datetime.now().strftime(
            "%Y%m%d_%H:%M") + "\n")
        outputfile.write("# created with Phobos" + defs.version + " - https://github.com/rock-simulation/phobos\n\n")
        outputfile.write(yaml.dump(
            model))  # default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries


def exportModelToSRDF(model, path):
    """This function exports the SRDF-relevant data from the dictionary to a specified path.
    Further detail on different elements of SRDF:

    <group>
    Groups in SRDF can contain *links*, *joints*, *chains* and other *groups* (the latter two of which have to be specified
    upstream. As nested groups is just a shortcut for adding links and joints to a group, it is not supported and the
    user will have to add all links and joints explicitly to each group.
    Originally both links and their associated parent joints were added. SRDF however implicitly assumes this, so the
    current implementation only adds the links.

    <chain>
    Chains are elements to simplify defining groups and are supported. The dictionary also contains a list of all
    elements belonging to that chain, which is discarded and not written to SRDF, however. It might be written to SMURF
    in the future.

    <link_sphere_approximatio>
    SRDF defines the convention that if no sphere is defined, one large sphere is
    assumed for that link. If one wants to have no sphere at all, it is necessary to define a sphere of radius 0.
    As one large sphere can be explicitly added by the user and should be if that is what he intends (WYSIWYG),
    we add a sphere of radius 0 by default if no sphere is specified.

    <passive_joint>
    Marks a joint as passive.

    <disable_collisions>
    Disables collisions between pairs of links to simplify collision checking and avoid collisions
    of parents and children at their joints.


    Currently not supported:
    - <group_state>
    - <virtual_joint>

    :param model: a robot model dictionary.
    :type model: dict
    :param path: the outpath for the file.
    :type path: str

    """
    output = []
    output.append(xmlHeader)
    output.append(indent + '<robot name="' + model['modelname'] + '">\n\n')
    sorted_group_keys = get_sorted_keys(model['groups'])
    for groupname in sorted_group_keys:
        output.append(indent * 2 + '<group name="' + groupname + '">\n')
        # TODO: once groups are implemented, this should be sorted aswell:
        for member in model['groups'][groupname]:
            output.append(indent * 3 + '<' + member['type'] + ' name="' + member['name'] + '" />\n')
        output.append(indent * 2 + '</group>\n\n')
    sorted_chain_keys = get_sorted_keys(model['chains'])
    for chainname in sorted_chain_keys:
        output.append(indent * 2 + '<group name="' + chainname + '">\n')
        chain = model['chains'][chainname]
        output.append(indent * 3 + '<chain base_link="' + chain['start'] + '" tip_link="' + chain['end'] + '" />\n')
        output.append(indent * 2 + '</group>\n\n')
    #for joint in model['state']['joints']:
    #    pass
    # passive joints
    sorted_joint_keys = get_sorted_keys(model['joints'])
    for joint in sorted_joint_keys:
        try:
            if model['joints'][joint]['passive']:
                output.append(indent * 2 + '<passive_joint name="' + model['links'][joint]['name'] + '"/>\n\n')
        except KeyError:
            pass
    sorted_link_keys = get_sorted_keys(model['links'])
    for link in sorted_link_keys:
        if len(model['links'][link]['approxcollision']) > 0:
            output.append(indent * 2 + '<link_sphere_approximation link="' + model['links'][link]['name'] + '">\n')
            # TODO: there does not seem to be a way to sort the spheres if there are multiple
            for sphere in model['links'][link]['approxcollision']:
                output.append(xmlline(3, 'sphere', ('center', 'radius'), (l2str(sphere['center']), sphere['radius'])))
            output.append(indent * 2 + '</link_sphere_approximation>\n\n')
        else:
            output.append(indent * 2 + '<link_sphere_approximation link="' + model['links'][link]['name'] + '">\n')
            output.append(xmlline(3, 'sphere', ('center', 'radius'), ('0.0 0.0 0.0', '0')))
            output.append(indent * 2 + '</link_sphere_approximation>\n\n')
    # calculate collision-exclusive links
    collisionExclusives = []
    for combination in itertools.combinations(model['links'], 2):
        link1 = model['links'][combination[0]]
        link2 = model['links'][combination[1]]
        # TODO: we might want to automatically add parent/child link combinations
        try:
            if link1['collision_bitmask'] & link2['collision_bitmask'] == 0:
                #output.append(xmlline(2, 'disable_collisions', ('link1', 'link2'), (link1['name'], link2['name'])))
                collisionExclusives.append((link1['name'], link2['name']))
        except KeyError:
            pass


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
    if bpy.data.worlds[0].structureExport:
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


def export(model, objectlist, path=None):
    """Configures and performs export of selected or specified model.

    :param path: The path to export the model to.
    :type path: str
    :param model: The model to be exported.
    :type model: dict

    """
    # check for valid model and objectlist
    if not model or not objectlist:
        return  # TODO: Check if there could be a valid model to export without a single object

    # set up path
    if not path:
        if bpy.data.worlds[0].relativePath:
            outpath = securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"), bpy.data.worlds[0].path)))
        else:
            outpath = securepath(os.path.expanduser(bpy.data.worlds[0].path))
    else:
        outpath = path
    if not outpath.endswith(os.path.sep):
        outpath += os.path.sep
    meshoutpath = securepath(os.path.join(outpath, 'meshes'))
    log("Export path: " + outpath, "DEBUG", "export")

    # parse export settings
    yaml = bpy.data.worlds[0].exportYAML
    urdf = bpy.data.worlds[0].exportURDF
    srdf = bpy.data.worlds[0].exportSRDF
    smurf = bpy.data.worlds[0].exportSMURF
    meshexp = bpy.data.worlds[0].exportMeshes
    texexp = bpy.data.worlds[0].exportTextures
    objexp = bpy.data.worlds[0].useObj
    bobjexp = bpy.data.worlds[0].useBobj
    stlexp = bpy.data.worlds[0].useStl
    daeexp = bpy.data.worlds[0].useDae

    # export data
    if yaml or urdf or smurf:
        if yaml:
            exportModelToYAML(model, outpath + model["modelname"] + "_dict.yml")
        if srdf:
            exportModelToSRDF(model, outpath + model["modelname"] + ".srdf")
        if smurf:
            if bpy.data.worlds[0].structureExport:
                securepath(os.path.join(outpath, 'smurf'))
                securepath(os.path.join(outpath, 'urdf'))
                exportModelToSMURF(model, os.path.join(outpath, 'smurf/'))
            else:
                exportModelToSMURF(model, outpath)
        elif urdf:
            if bpy.data.worlds[0].structureExport:
                securepath(os.path.join(outpath, 'urdf'))
                exportModelToURDF(model, os.path.join(outpath, 'urdf', model["modelname"] + ".urdf"))
            else:
                exportModelToURDF(model, outpath + model["modelname"] + ".urdf")
    if meshexp:
        meshnames = set()
        exportobjects = set()
        for obj in objectlist:
            try:
                if ((obj.phobostype == 'visual' or obj.phobostype == 'collision') and
                        (obj['geometry/type'] == 'mesh') and (obj.data.name not in meshnames)):
                    meshnames.add(obj.data.name)
                    exportobjects.add(obj)
                    for lod in obj.lod_levels:
                        if lod.object.data.name not in meshnames:
                            meshnames.add(lod.object.data.name)
                            exportobjects.add(lod.object)
            except KeyError:
                log("Undefined geometry type in object " + obj.name + ", skipping mesh export", "ERROR", "export")

        if exportobjects:  # if there are meshes to export
            show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269
            if show_progress:
                wm = bpy.context.window_manager
                wm.progress_begin(0, float(len(exportobjects)))
                i = 1
            log("Exporting " + str(len(exportobjects)) + " meshes to " + meshoutpath + "...", "INFO", "export")
            for expobj in exportobjects:
                if objexp:
                    eUtils.exportObj(meshoutpath, expobj)
                if bobjexp:
                    eUtils.exportBobj(meshoutpath, expobj)
                if stlexp:
                    eUtils.exportStl(meshoutpath, expobj)
                if daeexp:
                    eUtils.exportDae(meshoutpath, expobj)
                if show_progress:
                    wm.progress_update(i)
                    i += 1
            if show_progress:
                wm.progress_end()

    if texexp:
        log("Exporting textures to " + os.path.join(outpath, 'textures') + "...", "INFO", "export")
        securepath(os.path.join(outpath, 'textures'))
        for materialname in model['materials']:
            mat = model['materials'][materialname]
            for texturetype in ['diffuseTexture', 'normalTexture', 'displacementTexture']:
                if texturetype in mat:
                    texpath = os.path.join(os.path.expanduser(bpy.path.abspath('//')), mat[texturetype])
                    if os.path.isfile(texpath):
                        shutil.copy(texpath, os.path.join(outpath, 'textures', os.path.basename(mat[texturetype])))


# information for the registration in the exporter
entity_type_name = 'smurf'
