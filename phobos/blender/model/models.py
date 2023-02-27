#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------


import bpy

from .. import defs
from ..phoboslog import log
from ..utils import naming as nUtils
from ..utils import selection as sUtils


def recursive_dictionary_cleanup(dictionary):
    """Recursively enrich the dictionary and replace object links with names etc.

    These patterns are replaced:
        [phobostype, bpyobj] -> {'object': bpyobj, 'name': getObjectName(bpyobj, phobostype)}

    Args:
      dictionary(dict): dictionary to enrich

    Returns:
      : dict -- dictionary with replace/enriched patterns

    """
    for key, value in dictionary.items():
        # handle everything as list, so we can loop over it
        unlist = False
        if not isinstance(value, list):
            value = [value]
            unlist = True

        itemlist = []
        for item in value:
            if isinstance(item, list) and item:
                # (phobostype, bpyobj) -> {'object': bpyobj, 'name': getObjectName(bpyobj)}
                if (
                        len(item) == 2
                        and isinstance(item[0], str)
                        and (item[0] in ['joint'] + [enum[0] for enum in defs.phobostypes])
                        and isinstance(item[1], bpy.types.Object)
                ):
                    itemlist.append(
                        {
                            'object': item[1],
                            'name': nUtils.getObjectName(item[1], phobostype=item[0]),
                        }
                    )

            # recursion on subdictionaries
            elif isinstance(item, dict):
                itemlist.append(recursive_dictionary_cleanup(item))
            else:
                itemlist.append(item)

        # extract single items back out of the list
        dictionary[key] = itemlist if not unlist else itemlist[0]
    return dictionary


def initObjectProperties(
        obj, phobostype=None, ignoretypes=(), includeannotations=True, ignorename=False
):
    """Initializes phobos dictionary of *obj*, including information stored in custom properties.

    Args:
      obj(bpy_types.Object): object to derive initial properties from.
      phobostype(str, optional): limit parsing of data fields to this phobostype (Default value = None)
      ignoretypes(list, optional): list of properties ignored while initializing the objects properties. (Default value = ())
      ignorename(bool, optional): whether or not to add the object's name (Default value = False)
      includeannotations: (Default value = True)

    Returns:
      : dict -- phobos properties of the object

    """
    # allow duplicated names differentiated by types
    props = {} if ignorename else {'name': nUtils.getObjectName(obj, phobostype)}

    # if no phobostype is defined, everything is parsed
    if not phobostype:
        for key, value in obj.items():
            # transform Blender id_arrays into lists
            if hasattr(value, 'to_list'):
                value = list(value)
            elif hasattr(value, 'to_dict'):
                value = value.to_dict()
            props[key] = value

    # search for type-specific properties if phobostype is defined
    else:
        for key, value in obj.items():
            # transform Blender id_arrays into lists
            if hasattr(value, 'to_list'):
                value = list(value)
            elif hasattr(value, 'to_dict'):
                value = value.to_dict()

            # remove phobostype namespaces for the object
            if key.startswith(phobostype + '/'):
                if key.count('/') == 1:
                    props[key.replace(phobostype + '/', '')] = value
                # TODO make this work for all levels of hierarchy
                elif key.count('/') == 2:
                    category, specifier = key.split('/')[1:]
                    if '$' + category not in props:
                        props['$' + category] = {}
                    props['$' + category][specifier] = value

            # ignore two-level specifiers if phobostype is not present
            elif key.count('/') == 1:
                category, specifier = key.split('/')
                if category not in ignoretypes:
                    if '$' + category not in props:
                        props['$' + category] = {}
                    props['$' + category][specifier] = value

    # collect phobostype specific annotations from child objects
    if includeannotations:
        annotationobjs = sUtils.getImmediateChildren(obj, ('annotation',), selected_only=True)
        for annot in annotationobjs:
            log(
                "  Adding annotations from {}.".format(
                    nUtils.getObjectName(annot, phobostype='annotation')
                ),
                'DEBUG',
            )
            props.update(
                initObjectProperties(
                    annot, phobostype, ignoretypes, includeannotations, ignorename=True
                )
            )

    # recursively enrich the property dictionary
    props = recursive_dictionary_cleanup(props)

    return props


#
#
# def deriveGroupEntry(group):
#     """Derives a list of phobos link skeletons for a provided group object.
#
#     Args:
#       group(bpy_types.Group): The blender group to extract the links from.
#
#     Returns:
#       : list
#
#     """
#     links = []
#     for obj in group.objects:
#         if obj.phobostype == 'link':
#             links.append({'type': 'link', 'name': nUtils.getObjectName(obj)})
#         else:
#             log(
#                 "Group "
#                 + group.name
#                 + " contains "
#                 + obj.phobostype
#                 + ': '
#                 + nUtils.getObjectName(obj),
#                 "ERROR",
#             )
#     return links
#
#
# def deriveChainEntry(obj):
#     """Derives a phobos dict entry for a kinematic chain ending in the provided object.
#
#     Args:
#       obj: return:
#
#     Returns:
#
#     """
#     returnchains = []
#     if 'endChain' in obj:
#         chainlist = obj['endChain']
#     for chainName in chainlist:
#         chainclosed = False
#         parent = obj
#         chain = {'name': chainName, 'start': '', 'end': nUtils.getObjectName(obj), 'elements': []}
#         while not chainclosed:
#             # FIXME: use effectiveParent
#             if parent.parent is None:
#                 log("Unclosed chain, aborting parsing chain " + chainName, "ERROR")
#                 chain = None
#                 break
#             chain['elements'].append(parent.name)
#             # FIXME: use effectiveParent
#             parent = parent.parent
#             if 'startChain' in parent:
#                 startchain = parent['startChain']
#                 if chainName in startchain:
#                     chain['start'] = nUtils.getObjectName(parent)
#                     chain['elements'].append(nUtils.getObjectName(parent))
#                     chainclosed = True
#         if chain is not None:
#             returnchains.append(chain)
#     return returnchains
#
#
# def deriveTextData(modelname):
#     """Collect additional data stored for a specific model.
#
#     Args:
#       modelname: Name of the model for which data should be derived.
#
#     Returns:
#       : A dictionary containing additional data.
#
#     """
#     datadict = {}
#     datatextfiles = [text for text in bpy.data.texts if text.name.startswith(modelname + '::')]
#     for text in datatextfiles:
#         try:
#             dataname = text.name.split('::')[-1]
#         except IndexError:
#             log("Possibly invalidly named model data text file: " + modelname, "WARNING")
#         try:
#             data = json.loads(bUtils.readTextFile(text.name))
#         except:
#             log("Invalid formatting of data file: " + dataname, "ERROR")
#         if data:
#             datadict[dataname] = data
#     return datadict


# TODO remove unused function? (there are also undefined variables in it)
# def deriveModelDictionaryFromSubmodel(modelname):
#     """Derive a dictionary from a submodel.

#     :param modelname: name of the submodel to derive from
#     :type modelname: str

#     :return: representation of the submodel
#     :rtype: dict
#     """
#     assert isinstance(modelname, str), "Modelname is not a string: " + type(modelname)
#     model = {
#         'links': {},
#         'joints': {},
#         'sensors': {},
#         'motors': {},
#         'controllers': {},
#         'materials': {},
#         'meshes': {},
#         'lights': {},
#         'groups': {},
#         'chains': {}
#     }

#     # collect general model properties
#     model['date'] = datetime.now().strftime("%Y%m%d_%H:%M")
#     model['name'] = modelname

#     # collect all submodels
#     submodels = [a for a in bpy.data.objects if a.phobostype == 'submodel']

#     # namespace links, joints, motors etc for each submodel
#     for subm in submodels:
#         print('-----------------------', subm.name, subm['submodel/name'], '\n')
#         rootlink = [r for r in bpy.data.objects if sUtils.isRoot(r)
#                     and r['model/name'] == subm['submodel/name']][0]
#         adict = deriveModelDictionary(rootlink)
#         for l in adict['links']:
#             model['links'][namespaced(l, subm.name)] = namespaceLink(
#                 adict['links'][l], subm.name)
#         for j in adict['joints']:
#             model['joints'][namespaced(j, subm.name)] = namespaceJoint(
#                 adict['joints'][j], subm.name)
#         for m in adict['motors']:
#             model['motors'][namespaced(m, subm.name)] = namespaceMotor(
#                 adict['motors'][m], subm.name)
#         for mat in adict['materials']:
#             if mat not in model['materials']:
#                 model['materials'][mat] = adict['materials'][mat]
#         for mesh in adict['meshes']:
#             model['meshes'][namespaced(mesh, subm.name)] = adict['meshes'][mesh]
#         print('\n\n')

#     for subm in submodels:
#         rootlink = [r for r in bpy.data.objects if sUtils.isRoot(r)
#                     and r['model/name'] == subm['submodel/name']][0]
#         if subm.parent:
#             # get interfaces and parents
#             parentsubmodelname = subm.parent.parent.parent['submodel/name']
#             parentinterfacename = subm.parent.parent['interface/name']
#             parentsubmodel = [r for r in bpy.data.objects if sUtils.isRoot(r)
#                               and r['model/name'] == parentsubmodelname][0]
#             parentinterface = [i for i in sUtils.getChildren(parentsubmodel,
#                                                              ('interface',))
#                                if i['interface/name'] == parentinterfacename][0]
#             parentlinkname = parentinterface.parent.name

#             # derive link pose for root link
#             matrix = eUtils.getCombinedTransform(subm, subm.parent.parent.parent)
#             pose = {'rawmatrix': matrix,
#                     'matrix': [list(vector) for vector in list(matrix)],
#                     'translation': list(matrix.to_translation()),
#                     'rotation_euler': list(matrix.to_euler()),
#                     'rotation_quaternion': list(matrix.to_quaternion())
#                     }
#             model['links'][namespaced(rootlink.name, subm.name)]['pose'] = pose

#             # derive additional joint
#             model['joints'][a.name] = deriveJoint(rootlink, adjust=True)
#             # print(json.dumps(model['joints'][a.name]))
#             model['joints'][a.name]['name'] = namespaced(rootlink.name, a.name)
#             model['joints'][a.name]['parent'] = namespaced(parentlinkname, a.parent.parent.parent.name)
#             model['joints'][a.name]['child'] = namespaced(rootlink.name, a.name)
#             # print(json.dumps(model['joints'][a.name]))
#     return model

#
# def gatherAnnotations(model):
#     """Gathers custom properties annotating elements of the robot
#     across the model. These annotations were created in the model.py
#     module and are marked with a leading '$'.
#
#     Args:
#       model(dict): The robot model dictionary.
#       ignore_keys(list): Ignored annotation categories (Default value = [])
#
#     Returns:
#       : dict -- A dictionary of the gathered annotations.
#
#     """
#     # TODO check this stuff
#     annotations = {}
#     elementlist = []
#     types = ('links', 'joints', 'sensors', 'motors', 'controllers', 'materials')
#     # gather information from directly accessible types
#     for objtype in types:
#         for elementname in model[objtype]:
#             tmpdict = model[objtype][elementname]
#             tmpdict['temp_type'] = objtype[:-1]
#             elementlist.append(tmpdict)
#
#     # add information from types hidden in links
#     for linkname in model['links']:
#         for objtype in ('collision', 'visual'):
#             if objtype in model['links'][linkname]:
#                 for elementname in model['links'][linkname][objtype]:
#                     tmpdict = model['links'][linkname][objtype][elementname]
#                     tmpdict['temp_type'] = objtype
#                     elementlist.append(tmpdict)
#         if 'inertial' in model['links'][linkname]:
#             tmpdict = model['links'][linkname]['inertial']
#             tmpdict['temp_type'] = 'inertial'
#             elementlist.append(tmpdict)
#
#     # loop through the list of annotated elements and categorize the data
#     for element in elementlist:
#         delkeys = []
#         for key in element.keys():
#             if key.startswith('$'):
#                 category = key[1:]
#                 # ignore motor properties for link and joint types
#                 if category == "motor":
#                     if element['temp_type'] == "link" or element['temp_type'] == "joint":
#                         continue
#                 if category not in annotations:
#                     annotations[category] = {}
#                 if element['temp_type'] not in annotations[category]:
#                     annotations[category][element['temp_type']] = []
#                 tmpdict = {k: element[key][k] for k in element[key]}
#                 tmpdict['name'] = element['name']
#                 annotations[category][element['temp_type']].append(tmpdict)
#                 delkeys.append(key)
#         delkeys.append('temp_type')
#         for key in delkeys:
#             if key in element:
#                 del element[key]
#
#     return annotations
#
#
#
# def createGroup(group):
#     """
#
#     Args:
#       group:
#
#     Returns:
#
#     """
#     # TODO lots of code missing here... make it a dev branch
#     pass
#
#
# def createChain(group):
#     """
#
#     Args:
#       group:
#
#     Returns:
#
#     """
#     # TODO lots of code missing here... make it a dev branch
#     pass
