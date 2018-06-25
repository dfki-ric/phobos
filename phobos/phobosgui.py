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

File phobosgui.py

Created on 6 Jan 2014

@author: Kai von Szadkowski, Simon Reichel
"""

import sys
import inspect

import bpy
# import bgl
from bpy.props import (BoolProperty, IntProperty, StringProperty, EnumProperty,
                       PointerProperty, CollectionProperty, FloatProperty)
from bpy.types import AddonPreferences

from phobos.io import entities
from phobos.io import meshes
from phobos.io import scenes
from phobos.io import libraries
from phobos.phoboslog import loglevels
import phobos.utils.io as ioUtils

from . import defs
from . import display


class ModelPoseProp(bpy.types.PropertyGroup):
    # DOCU missing class description
    robot_name = StringProperty()
    label = StringProperty()
    hide = BoolProperty(default=True)
    parent = StringProperty()
    icon = StringProperty()
    type = StringProperty()
    path = StringProperty()
    model_file = StringProperty()
    preview = StringProperty()


class PhobosPrefs(AddonPreferences):
    """The general Phobos addon settings are stored in this class.
    They can be edited in the User Preferences of Blender under the Addon tab.

    Args:

    Returns:

    """
    bl_idname = __package__

    logfile = StringProperty(
        name="logfile",
        subtype="FILE_PATH",
        default="."
    )

    loglevel = EnumProperty(
        name="loglevel",
        items=tuple(((l,) * 3 for l in loglevels)),
        default="ERROR"
    )

    logtofile = BoolProperty(
        name="logtofile",
        default=False
    )

    logtoterminal = BoolProperty(
        name="logtoterminal",
        default=True
    )

    modelsfolder = StringProperty(
        name="modelsfolder",
        subtype="DIR_PATH",
        default=''
    )

    configfolder = StringProperty(
        name="configfolder",
        subtype="DIR_PATH",
        description="Path to the system-dependent config folder of Phobos.",
        default=''
    )

    exportpluginsfolder = StringProperty(
        name='exportpluginsfolder',
        subtype='DIR_PATH',
        default='.'
    )

    models_poses = CollectionProperty(type=ModelPoseProp)

    def draw(self, context):
        layout = self.layout
        layout.label(text="Log Settings")
        layout.prop(self, "logfile", text="log file path")
        layout.prop(self, "logtofile", text="write to logfile")
        layout.prop(self, "logtoterminal", text="write to terminal")
        layout.prop(self, "loglevel", text="log level")
        layout.separator()
        layout.label(text="Folders")
        layout.prop(self, "modelsfolder", text="models folder")
        layout.prop(self, "configfolder", text="config folder")

prev_collections = {}
phobosIcon = 0


class PhobosExportSettings(bpy.types.PropertyGroup):
    # DOCU missing class description

    def updateExportPath(self, context):
        # DOCU missing description
        if not bpy.data.window_managers[0].phobosexportsettings.path.endswith('/'):
            bpy.data.window_managers[0].phobosexportsettings.path += '/'

    def getMeshTypeListForEnumProp(self, context):
        # DOCU missing description
        return sorted([(mt,) * 3 for mt in meshes.mesh_types])

    path = StringProperty(name='path', subtype='DIR_PATH', default='../',
                          update=updateExportPath)
    # TODO: CHECK which props are visible in GUI?
    selectedOnly = BoolProperty(name="Selected only", default=True,
                                description="Export only selected objects")
    decimalPlaces = IntProperty(name="decimals", description="Number of " +
                                "decimal places to export", default=5)
    exportTextures = BoolProperty(name='Export textures', default=True)
    outputMeshtype = EnumProperty(items=getMeshTypeListForEnumProp,
                                  name='link',
                                  description="Mesh type to use in exported " +
                                  "entity/scene files.")


class Mesh_Export_UIList(bpy.types.UIList):
    # DOCU missing class description
    # CHECK is this class in use

    def draw_item(self, context, layout, data, item, icon, active_data,
                  active_propname, index):
        # TODO remove this code?
        # assert(isinstance(item, bpy.types.MaterialTextureSlot)
        ma = data
        slot = item
        tex = slot.texture if slot else None
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            if tex:
                layout.prop(tex, "name", text="",
                            emboss=False, icon_value=icon)
            else:
                layout.label(text="", icon_value=icon)
            if tex and isinstance(item, bpy.types.MaterialTextureSlot):
                layout.prop(ma, "use_textures", text="", index=index)
        elif self.layout_type == 'GRID':
            layout.alignment = 'CENTER'
            layout.label(text="", icon_value=icon)


class Models_Poses_UIList(bpy.types.UIList):
    # DOCU missing class description
    # CHECK is this class in use?

    def draw_item(self, context, layout, data, item, icon, active_data,
                  active_propname, index):
        self.use_filter_show = False
        im = item
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        if im.name in modelsPosesColl.keys():
            coll_item = modelsPosesColl[im.name]
            if coll_item.type == "robot_name":
                layout.label(text=coll_item.label,
                             translate=False, icon=coll_item.icon)
            else:
                sLayout = layout.split(0.1)
                sLayout.label(text="")
                if im.filepath != '':
                    sLayout.label(text=coll_item.label,
                                  translate=False, icon_value=icon)
                else:
                    sLayout.label(text=coll_item.label,
                                  translate=False, icon=coll_item.icon)

    def filter_items(self, context, data, propname):
        images = getattr(data, propname)
        flt_flags = [self.bitflag_filter_item] * len(images)

        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses

        # Filter items. Only show robots. Hide all other images
        for idx, im in enumerate(images):
            if im.name in modelsPosesColl.keys():
                curr_model = modelsPosesColl[im.name]
                if curr_model.hide and not (curr_model.type == "robot_name"):
                    flt_flags[idx] &= ~self.bitflag_filter_item
            else:
                flt_flags[idx] &= ~self.bitflag_filter_item

        # FIXME remove this (never used)
        helper_funcs = bpy.types.UI_UL_list
        # Reorder by name
        flt_neworder = []
        noPreviewIndex = 0
        for im in images:
            newIndex = 0
            if im.name in modelsPosesColl.keys():
                newIndex = modelsPosesColl.keys().index(im.name)
            else:
                newIndex = len(modelsPosesColl) + noPreviewIndex
                noPreviewIndex += 1
            flt_neworder.append(newIndex)

        return flt_flags, flt_neworder


def showPreview(self, value):
    # CHECK this should be a class function
    bpy.ops.scene.change_preview()


class PhobosToolsPanel(bpy.types.Panel):
    """Contains all general phobos tools in the Phobos viewport toolbar"""
    bl_idname = "TOOLS_PT_PHOBOS_TOOLS"
    bl_label = "General Tools"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'

    def draw_header(self, context):
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        layout = self.layout

        # Tools & Selection Menu
        tsinlayout = layout.split()
        tsc1 = tsinlayout.column(align=True)
        tsc1.label(text="Select...", icon='HAND')
        tsc1.operator('phobos.select_root', text='Root')
        tsc1.operator('phobos.select_model', text='Robot')
        tsc1.operator('phobos.select_objects_by_phobostype',
                      text="by Phobostype")
        tsc1.operator('phobos.select_objects_by_name', text="by Name")
        tsc2 = tsinlayout.column(align=True)
        tsc2.label(text="Tools", icon='MODIFIER')
        tsc2.operator('phobos.sort_objects_to_layers', icon='IMGDISPLAY')
        tsc2.operator('phobos.set_xray')
        tsc2.operator('phobos.toggle_namespaces')
        tsc2.operator('phobos.measure_distance')
        tsc2.operator('phobos.validate')


# TODO move this to a better place (utils)
def getMatrixData(coord, space):
    if space == 'local':
        matrix = bpy.context.active_object.matrix_local
    elif space == 'world':
        matrix = bpy.context.active_object.matrix_world
    if coord == 'x':
        return matrix.to_translation()[0]
    elif coord == 'y':
        return matrix.to_translation()[1]
    elif coord == 'z':
        return matrix.to_translation()[2]
    elif coord == 'rotx':
        return matrix.to_euler()[0]
    elif coord == 'roty':
        return matrix.to_euler()[1]
    elif coord == 'rotz':
        return matrix.to_euler()[2]


# CHECK will this stay here? Give it its own file?
class MatrixPropGroup(bpy.types.PropertyGroup):
    from bpy.props import FloatProperty

    loc_x_local = FloatProperty(
        name='location x',
        get=lambda self: getMatrixData('x', 'local'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='X coordinate in the local space')
    loc_y_local = FloatProperty(
        name='location y',
        get=lambda self: getMatrixData('y', 'local'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='Y coordinate in the local space')
    loc_z_local = FloatProperty(
        name='location z',
        get=lambda self: getMatrixData('z', 'local'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='Z coordinate in the local space')
    rot_x_local = FloatProperty(
        name='rotation x',
        get=lambda self: getMatrixData('rotx', 'local'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around local x axis')
    rot_y_local = FloatProperty(
        name='rotation y',
        get=lambda self: getMatrixData('roty', 'local'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around local y axis')
    rot_z_local = FloatProperty(
        name='rotation z',
        get=lambda self: getMatrixData('rotz', 'local'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around local z axis')
    loc_x_world = FloatProperty(
        name='location x',
        get=lambda self: getMatrixData('x', 'world'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='X coordinate in the world space')
    loc_y_world = FloatProperty(
        name='location y',
        get=lambda self: getMatrixData('y', 'world'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='Y coordinate in the world space')
    loc_z_world = FloatProperty(
        name='location z',
        get=lambda self: getMatrixData('z', 'world'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='Z coordinate in the world space')
    rot_x_world = FloatProperty(
        name='rotation x',
        get=lambda self: getMatrixData('rotx', 'world'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around world x axis')
    rot_y_world = FloatProperty(
        name='rotation y',
        get=lambda self: getMatrixData('roty', 'world'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around world y axis')
    rot_z_world = FloatProperty(
        name='rotation z',
        get=lambda self: getMatrixData('rotz', 'world'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around world z axis')


# CHECK will this stay here? Give it its own file?
class PhobosMatrixPanel(bpy.types.Panel):
    """Contains summary information and editing possibilities in the Buttons
    Window

    Args:

    Returns:

    """
    bl_idname = "INFOBAR_PT_PHOBOS_TOOLS"
    bl_label = "Phobos Matrix Information"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "object"

    def draw_header(self, context):
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        layout = self.layout

        obj = context.active_object

        matrixes = layout.split()
        localcol = matrixes.column(align=True)
        worldcol = matrixes.column(align=True)

        # Local data first
        localcol.label(text='local', icon='ROTACTIVE')
        # add all location properties
        for locprop in dir(obj.phobosmatrixinfo):
            if locprop.startswith('loc') and locprop.endswith('local'):
                localcol.prop(obj.phobosmatrixinfo, locprop,
                              text=locprop[4] + ' location')

        localcol.separator()
        # add all rotation properties
        for rotatprop in dir(obj.phobosmatrixinfo):
            if rotatprop.startswith('rot') and rotatprop.endswith('local'):
                localcol.prop(obj.phobosmatrixinfo, rotatprop,
                              text=rotatprop[4] + ' rotation')

        # world data second
        worldcol.label(text='world', icon='WORLD')
        # add all location properties
        for locprop in dir(obj.phobosmatrixinfo):
            if locprop.startswith('loc') and locprop.endswith('world'):
                worldcol.prop(obj.phobosmatrixinfo, locprop,
                              text=locprop[4] + ' location')
        worldcol.separator()
        # add all rotation properties
        for rotatprop in dir(obj.phobosmatrixinfo):
            if rotatprop.startswith('rot') and rotatprop.endswith('world'):
                worldcol.prop(obj.phobosmatrixinfo, rotatprop,
                              text=rotatprop[4] + ' rotation')


class PhobosObjectInformationPanel(bpy.types.Panel):
    """Contains information like parent, immediate children etc. in the
    Buttons Window

    Args:

    Returns:

    """
    bl_idname = "OBJINFO_PT_PHOBOS_TOOLS"
    bl_label = "Phobos Object Information"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "object"

    def draw_header(self, context):
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        import phobos.utils.selection as sUtils
        layout = self.layout
        obj = context.active_object
        modelname = ''
        rootname = ''

        root = sUtils.getRoot(obj)
        if 'modelname' in root.keys():
            modelname = root['modelname']
        rootname = root.name

        datatop = layout.split()
        datatopl = datatop.column(align=True)
        datatopr = datatop.column(align=True)
        # dataright = layout.column(align=True)

        datatopl.operator('phobos.name_model', text=('Part of model: ' +
                          modelname), icon='POSE_DATA')

        datatopr.operator('phobos.select_root', text=('Root object: ' +
                          rootname), icon='OOPS')
        # layout.operator('phobos.name_model', text='Test', emboss=False)


ignoredProps = set([
    'cycles', 'cycles_visibility', 'phobosmatrixinfo', 'phobostype', 'name',
    'pose', 'size', 'scale', 'parent'
])


class PhobosPropertyInformationPanel(bpy.types.Panel):
    """Contains all properties sorted in different categories"""
    bl_idname = "PROPINFO_PT_PHOBOS_TOOLS"
    bl_label = "Phobos Property Information"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "object"

    def addProp(self, prop, value, layout, params):
        # get the existing layout columns
        leftLayout = layout[1]
        rightLayout = layout[2]

        # put the value left or right?
        if layout[3][0] <= layout[3][1]:
            layout[3][0] += len(prop)
            column = leftLayout
        else:
            layout[3][1] += len(prop)
            column = rightLayout

        # add all properties in sequence
        for i in range(len(prop)):
            subtable = column.split(percentage=0.45)
            colL = subtable.column()
            colR = subtable.column()
            # use custom params (like icons etc) from the dictionary
            if type(value[i]) is float:
                value[i] = '{0:.4f}'.format(value[i])

            # use custom properties for special operators or icons
            # use custom properties for special operators or icons
            if params[i]:
                colL.label(text='{0}'.format(prop[i]))
                if 'operator' in params[i]:
                    colR.operator(params[i]['operator'],
                                  text='{0}'.format(value[i]))
                else:
                    colR.label(text='{0}'.format(value[i]),
                               **params[i]['infoparams'])
            else:
                colL.label(text='{0}'.format(prop[i]))
                colR.label(text='{0}'.format(value[i]))

    def addObjLink(self, prop, value, layout, params):
        # this list is used to force labelling of special keywords
        labels = ['joint', 'child']

        # get the existing layout columns
        leftLayout = layout[1]
        rightLayout = layout[2]
        # put the link left or right?
        if layout[3][0] <= layout[3][1]:
            layout[3][0] += 1
            column = leftLayout
        else:
            layout[3][1] += 1
            column = rightLayout

        label = [prop if prop in labels else ''][0]
        # use custom params (like icons etc) from the dictionary
        if params:
            op = column.operator('phobos.goto_object',
                                 text=(label +
                                       [': ' if len(label) > 0 else ''][0] +
                                       '{1}'.format(label, value.name)),
                                 **params['infoparams'])
            op.objectname = value.name
        else:
            op = column.operator('phobos.goto_object',
                                 text=(label +
                                       [': ' if len(label) > 0 else ''][0] +
                                       '{1}'.format(label, value.name)))
            op.objectname = value.name

    def checkParams(self, item):
        if item in supportedProps:
            params = supportedProps[item]
        else:
            params = None

        return params

    def draw_header(self, context):
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        from phobos.model.models import deriveDictEntry
        from phobos.model.models import deriveFullLinkInformation
        import bpy
        layout = self.layout
        originalLayout = layout
        obj = context.active_object

        # add phobostype
        box = originalLayout.box()
        table = box.split()
        leftLayout = table.column()
        rightLayout = table.column()
        leftLayout.label('Phobostype')
        # OPT: change icon_value from phobostypeIcons
        rightLayout.label(obj.phobostype, icon_value=phobosIcon)

        # derive object information as dictionary
        if obj.phobostype == 'link':
            dictprops = deriveFullLinkInformation(obj)
        else:
            dictprops = deriveDictEntry(obj)
        if dictprops is not None:
            proplist = dictprops.keys()

            # add parent object if available
            if obj.parent:
                leftLayout.label('Parent object')
                op = rightLayout.operator('phobos.goto_object',
                                        text='{0}'.format(obj.parent.name))
                op.objectname = obj.parent.name

            categories = {}

            # check whether the general category is needed
            generalProp = set([propname for propname in proplist
                            if type(dictprops[propname]) is not dict])
            if len(generalProp - ignoredProps) > 0:
                general = originalLayout.box()
                generalTable = general.split()
                generalL = generalTable.column()
                generalR = generalTable.column()
                categories['general'] = [general, generalL, generalR, [0, 0]]

            # remove the unimportant properties and iterate over the rest
            proplist = set(proplist) - ignoredProps
            for prop in proplist:
                params = self.checkParams(prop)
                value = dictprops[prop]

                # check for categories
                if type(value) is dict:
                    category = prop

                    # look for existing category layout
                    if category in categories:
                        layout = categories[category]
                    # add a new category layout
                    else:
                        # use custom icons for supported categories
                        if category in supportedCategories:
                            originalLayout.label(
                                category.upper(),
                                icon_value=supportedCategories[category][
                                    'icon_value'])
                        else:
                            originalLayout.label(category.upper())

                        # create column hierarchy
                        box = originalLayout.box()
                        catTable = box.split()
                        catL = catTable.column()
                        catR = catTable.column()
                        categories[category] = [box, catL, catR, [0, 0]]
                        layout = categories[category]

                    # add each subproperty to the layout
                    for propT2 in dictprops[category].keys():
                        params = self.checkParams(category + '/' + propT2)

                        value = dictprops[category][propT2]
                        if propT2 not in ignoredProps:
                            # is it a linkable object?
                            if type(value) is bpy.types.Object or (
                                    type(value) is str and
                                    value in context.scene.objects):
                                # a string identifier for an object
                                if type(value) is str:
                                    value = context.scene.objects[value]
                                self.addObjLink(propT2, value, layout, params)
                            # is it another dictionary with values?
                            elif type(value) is dict:
                                props = value.keys()
                                values = [value[key] for key in props]
                                paramkeys = [category + '/' + propT2 + '/' +
                                            propkey for propkey in props]
                                paramlist = []
                                for paramkey in paramkeys:
                                    paramlist.append(self.checkParams(paramkey))
                                props = [propT2 + '/' +
                                        propkey for propkey in props]
                                self.addProp(props, values, layout, paramlist)
                            # just another value
                            else:
                                self.addProp([propT2], [value], layout, [params])
                # just a value for the general category
                else:
                    layout = categories['general']
                    if type(value) is bpy.types.Object or (
                            type(value) is str and value in context.scene.objects):
                        if type(value) is str:
                            value = context.scene.objects[value]
                        self.addObjLink(prop, value, layout, params)
                    else:
                        self.addProp([prop], [value], layout, [params])


class PhobosModelPanel(bpy.types.Panel):
    """Contains all model editing tools in the Phobos viewport toolbar"""
    bl_idname = "TOOLS_PT_PHOBOS_MODEL"
    bl_label = "Model Editing"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'

    def draw_header(self, context):
        # TODO decide on icon
        # self.layout.label(icon='OUTLINER_DATA_ARMATURE')
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        layout = self.layout

        # Robot Model Menu
        inlayout = layout.split()
        rc1 = inlayout.column(align=True)
        rc2 = inlayout.column(align=True)
        rc1.operator('phobos.name_model')
        rc2.operator('phobos.set_version')

        inlayout = layout.split()
        c1 = inlayout.column(align=True)
        c2 = inlayout.column(align=True)
        c1.operator('phobos.set_phobostype')
        c1.operator('phobos.batch_rename')
        c2.label(text="Custom properties")
        c2.operator('phobos.rename_custom_property', text="Rename", icon='SYNTAX_OFF')
        c2.operator('phobos.batch_property', text="Edit", icon='GREASEPENCIL')
        c2.operator('phobos.edityamldictionary', text="Edit Dictionary", icon='TEXT')
        c2.operator('phobos.copy_props', text="Copy", icon='GHOST')

        # Kinematics
        layout.separator()
        kinlayout = layout.split()
        kc1 = kinlayout.column(align=True)
        kc2 = kinlayout.column(align=True)
        kc1.label(text='Kinematics', icon='POSE_DATA')
        kc1.operator("phobos.create_links")
        kc1.operator('phobos.merge_links')
        kc1.operator('phobos.define_joint_constraints')
        kc1.operator("phobos.create_mimic_joint")
        kc1.operator('phobos.add_kinematic_chain', icon='CONSTRAINT')
        kc1.operator('phobos.assign_submechanism')
        kc1.operator('phobos.set_model_root')
        kc2.label(text='Visual/Collision', icon='GROUP')
        kc2.operator('phobos.create_collision_objects')
        kc2.operator('phobos.define_geometry')
        kc2.operator('phobos.set_collision_group')
        #kc2.operator('phobos.smoothen_surface')

        # Hardware
        layout.separator()
        minlayout = layout.split()
        hw1 = minlayout.column(align=True)
        hw1.label(text="Hardware", icon='MOD_SCREW')
        hw1.operator('phobos.add_motor')
        hw1.operator("phobos.add_annotations")
        hw1.operator('phobos.create_interface')

        # Masses & Inertia
        mc1 = minlayout.column(align=True)
        mc1.label(text="Masses & Inertia", icon='PHYSICS')
        mc1.operator('phobos.calculate_mass')
        mc1.operator('phobos.set_mass')
        mc1.operator('phobos.create_inertials')
        # mc1.operator('phobos.create_link_inertials')
        mc1.operator('phobos.edit_inertia')


# TODO bring this back or just delete it
# class PhobosScenePanel(bpy.types.Panel):
#     """A Custom Panel in the Phobos viewport toolbar"""
#     bl_idname = "TOOLS_PT_PHOBOS_SCENE"
#     bl_label = "Scene Editing"
#     bl_space_type = 'VIEW_3D'
#     bl_region_type = 'TOOLS'
#     bl_category = 'Phobos'
#
#     def draw_header(self, context):
#         # TODO decide on icon
#         # self.layout.label(icon='SCENE_DATA')
#         pcoll = prev_collections["phobos"]
#         phobosIcon = pcoll["phobosIcon"]
#         self.layout.label(icon_value=phobosIcon)
#
#     def draw(self, context):
#         layout = self.layout
#         layout.label(text="Scene Editing", icon="WORLD")
#         iinlayout = layout.split()
#         ic2 = iinlayout.column(align=True)
#         ic2.operator('phobos.define_entity')
#         hw1.operator("phobos.add_heightmap")
#
#         layout.label(text="Robotmodels and Poses", icon="MOD_ARMATURE")
#         #layout.operator("scene.load_backed_models_operator", text="Load Models", icon="LIBRARY_DATA_DIRECT")
#         #layout.operator("scene.reload_models_and_poses_operator", text="Reload Models and Poses", icon="LIBRARY_DATA_DIRECT")
#
#
#         modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
#         for model_pose in modelsPosesColl:
#             if not model_pose.name in bpy.data.images.keys():
#                 if model_pose.type == 'robot_name':
#                     bpy.data.images.new(model_pose.name,0,0)
#                 elif 'robot_pose':
#                     if model_pose.preview != '':
#                         if os.path.split(model_pose.preview)[-1] in bpy.data.images.keys():
#                             bpy.data.images[os.path.split(model_pose.preview)[-1]].reload()
#                         im = bpy.data.images.load(model_pose.preview)
#                         model_pose.name  = im.name
#                         #im.name = model_pose.name
#                         im.gl_load(0, bgl.GL_LINEAR, bgl.GL_LINEAR)
#                     else:
#                         print(model_pose.name)
#                         bpy.data.images.new(model_pose.name, 0, 0)
#
#         layout.template_list("Models_Poses_UIList", "",  bpy.data, "images", context.scene, "active_ModelPose")
#
# #        layout.template_preview(bpy.data.textures[context.scene.active_bakeModel])
# #        layout.template_preview(preview_mat.active_texture,
# #                                parent=preview_mat,
# #                                slot=preview_mat.texture_slots[preview_mat.active_texture_index],
# #                                preview_id="phobos_model_preview")
#
#         layout.operator('scene.phobos_import_selected_lib_robot', text="Import Selected Robot Bake", icon="IMPORT")
#         pinlayout = layout.split()
#         pc1 = pinlayout.column(align=True)
#         pc1.operator('phobos.store_pose2', text='Store Current Pose')
#         pc2 = pinlayout.column(align=True)
#         pc2.operator('phobos.load_pose2', text='Load Selected Pose')
#
#         layout.operator("phobos.export_current_poses", text="Export Selected Pose", icon="OUTLINER_OB_ARMATURE")
#         layout.operator("phobos.export_all_poses", text="Export All Poses", icon="OUTLINER_OB_ARMATURE")


class PhobosExportPanel(bpy.types.Panel):
    """Contains the export settings for models/meshes etc. in the Phobos viewport toolbar"""
    bl_idname = "TOOLS_EXPORT_PT_PHOBOS"
    bl_label = "Export"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'

    def draw_header(self, context):
        # TODO decide on icon
        # self.layout.label(icon='EXPORT')
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        expsets = bpy.data.window_managers[0].phobosexportsettings
        layout = self.layout

        # export robot model options
        layout.prop(expsets, "path")
        ginlayout = self.layout.split()
        g1 = ginlayout.column(align=True)
        # FIXME remove this?
        # g1.prop(expsets, "relativePaths")
        g1.prop(expsets, "exportTextures")
        g1.prop(expsets, "selectedOnly")
        g2 = ginlayout.column(align=True)
        g2.prop(expsets, "decimalPlaces")

        layout.separator()

        # Settings for mesh and entity export
        inlayout = self.layout.split()

        cmodel = inlayout.column(align=True)
        cmodel.label(text="Models")
        for entitytype in ioUtils.getEntityTypesForExport():
            typename = "export_entity_" + entitytype
            cmodel.prop(bpy.data.window_managers[0], typename)

        cmesh = inlayout.column(align=True)
        cmesh.label(text="Meshes")
        for meshtype in sorted(meshes.mesh_types):
            if 'export' in meshes.mesh_types[meshtype]:
                typename = "export_mesh_" + meshtype
                cmesh.prop(bpy.data.window_managers[0], typename)
        cmesh.prop(bpy.data.window_managers[0].phobosexportsettings, 'outputMeshtype')

        cscene = inlayout.column(align=True)
        cscene.label(text="Scenes")
        for scenetype in ioUtils.getSceneTypesForExport():
            typename = "export_scene_" + scenetype
            cscene.prop(bpy.data.window_managers[0], typename)

        # TODO delete me?
        # c2.prop(expsets, "exportCustomData", text="Export custom data")

        # TODO delete me?
        # layout.separator()
        # layout.label(text="Baking")
        # layout.operator("phobos.export_bake", text="Bake Robot Model", icon="OUTLINER_OB_ARMATURE")
        # layout.operator("phobos.create_robot_instance", text="Create Robot Lib Instance", icon="RENDERLAYERS")

        #  self.layout.prop(expsets, "heightmapMesh", text="export heightmap as mesh")

        layout.separator()
        layout.operator("phobos.export_model", icon="EXPORT")
        layout.operator("phobos.export_scene", icon="WORLD_DATA")


class PhobosImportPanel(bpy.types.Panel):
    """Contains the import settings in the Phobos viewport toolbar"""
    bl_idname = "TOOLS_IMPORT_PT_PHOBOS"
    bl_label = "Import"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'

    def draw_header(self, context):
        # TODO decide on icon
        # self.layout.label(icon='IMPORT')
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        self.layout.operator("phobos.import_robot_model",
                             text="Import Robot Model", icon="IMPORT")


class PhobosSubmodelsPanel(bpy.types.Panel):
    bl_idname = "TOOLS_SUBMODELS_PT_PHOBOS"
    bl_label = "Submodels"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos Models'

    def draw_header(self, context):
        #self.layout.label(icon='IMPORT')
        pass

    def draw(self, context):
        self.layout.operator("phobos.define_submodel")
        self.layout.operator("phobos.add_submodel")
        self.layout.operator("phobos.toggle_interfaces")
        self.layout.operator("phobos.connect_interfaces")


class PhobosObjectPanel(bpy.types.Panel):
    """Contains the custom properties of objects in the Buttons Window"""
    bl_idname = "phobos.PT_PHOBOS"
    bl_label = "Phobos properties"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"
    bl_category = 'Phobos'

    def draw_header(self, context):
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        layout = self.layout

        # TODO what is this?
        # the following for real pre-defined rather than custom properties
        # row_type.prop(bpy.context.active_object, "type")
        row_type = layout.row()
        row_type.label(icon="OBJECT_DATA")
        # row_type.prop_enum(bpy.context.active_object, '["type"]', "node")
        # row_type.prop_enum(bpy.context.active_object, 'phobostype')
        row_type.prop(bpy.context.active_object, 'phobostype')

        # FIXME this box is empty most of the time...
        box_props = layout.box()
        try:
            for prop in defs.type_properties[bpy.context.active_object.phobostype]:
                # CHECK what is this for?
                # box_props.label(prop)
                if prop in bpy.context.active_object:
                    box_props.prop(bpy.context.active_object,
                                   '["' + prop + '"]')
        except KeyError:
            # FIXME this should be logged, right?
            print("Key could not be found.")
            # TODO delete me?
            # else:
            #    bpy.context.active_object[prop] = defs.type_properties[bpy.context.active_object.phobostype+"_default"]


class PhobosModelLibraryPanel(bpy.types.Panel):
    # DOCU add some docstring and update bl_idname
    bl_idname = "TOOLS_PT_PHOBOS_LOCALMODELS"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "Phobos Models"
    bl_label = "Local Model Library"

    def draw_header(self, context):
        pcoll = prev_collections["phobos"]
        phobosIcon = pcoll["phobosIcon"]
        #self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        layout = self.layout
        wm = context.window_manager
        modelsfolder = bpy.context.user_preferences.addons["phobos"].preferences.modelsfolder
        if modelsfolder == '':
            layout.label('Model folder not configured.')
            return

        layout.operator("phobos.update_model_library", icon="FILE_REFRESH")

        if wm.category != '-':
            layout.prop(wm, 'category')

            if wm.modelpreview != '-':
                layout.template_icon_view(wm, 'modelpreview', show_labels=True, scale=5.0)
                layout.prop(wm, 'modelpreview')
                layout.operator("phobos.import_model_from_library", icon="IMPORT")
            else:
                layout.label('No models in this category.')
        else:
            layout.label('Model library is empty.')

def get_operator_manuals():
    """Returns a tuple with the Phobos wiki Operator page and pairs of operator
    names and wiki page anchor names to allow for linking from Blender to wiki.
    :return: tuple

    Args:

    Returns:

    """
    # CHECK does the linking work with the new wiki?
    url_manual_prefix = "https://github.com/dfki-ric/phobos/wiki/Operators#"
    url_manual_ops = tuple(('bpy.ops.phobos.' + opname, opname.replace('_', '-'),)
                           for opname in dir(bpy.ops.phobos) if not opname.startswith("__"))
    return url_manual_prefix, url_manual_ops


class PhobosDisplayPanel(bpy.types.Panel):
    bl_idname = "TOOLS_DISPLAY_PT_PHOBOS"
    bl_label = "Display"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'

    def draw_header(self, context):
        #self.layout.label(icon='IMPORT')
        pass

    def draw(self, context):
        wm = context.window_manager
        self.layout.prop(wm, "draw_phobos_infos")
        if wm.draw_phobos_infos:
            self.layout.separator()
            self.layout.label('Draw...')
            dlayout = self.layout.split()
            dc1 = dlayout.column(align=True)
            dc2 = dlayout.column(align=True)
            dc1.prop(wm, "draw_jointaxes")
            dc1.prop(wm, "jointaxes_length")
            dc1.prop(wm, "draw_submechanisms")
            dc1.prop(wm, "draw_progress")
            dc2.prop(wm, "draw_messages")
            dc2.prop(wm, 'phobos_msg_count')
            dc2.prop(wm, 'phobos_msg_offset')


def register():
    print("\nRegistering phobosgui...")

    # add phobostype to Blender objects
    bpy.types.Object.phobostype = EnumProperty(
        items=defs.phobostypes,
        name="type",
        description="Phobos object type")

    # add display properties to window manager
    bpy.types.WindowManager.draw_phobos_infos = BoolProperty(
        name='Draw Phobos Infos', default=False, update=display.start_draw_operator,
        description="Draw additional data visualization for Phobos items in 3D View.")

    bpy.types.WindowManager.draw_jointaxes = BoolProperty(
        name='Joint Axes', default=True)

    bpy.types.WindowManager.jointaxes_length = FloatProperty(
        name='Length', default=0.3)

    bpy.types.WindowManager.draw_submechanisms = BoolProperty(
        name='Submechanisms', default=True)

    bpy.types.WindowManager.draw_messages = BoolProperty(
        name='Messages', default=True)

    bpy.types.WindowManager.phobos_msg_count = IntProperty(
        name='show', default=5, min=0, max=20,
        description="How many Phobos log messages to show on screen")

    bpy.types.WindowManager.phobos_msg_offset = IntProperty(
        name='offset', default=0, min=0, max=50,
        description="The Phobos log message index to start with")

    bpy.types.WindowManager.draw_progress = BoolProperty(
        name='Progress', default=True)

    bpy.types.WindowManager.progress = FloatProperty(
        name='Progress', default=0,
        description="Progress value of custom Phobos progress bar.")

    # Add settings to world to preserve settings for every model
    for meshtype in meshes.mesh_types:
        if 'export' in meshes.mesh_types[meshtype]:
            typename = "export_mesh_" + meshtype
            setattr(bpy.types.WindowManager, typename, BoolProperty(name=meshtype))

    for entitytype in entities.entity_types:
        if 'export' in entities.entity_types[entitytype]:
            typename = "export_entity_" + entitytype
            setattr(bpy.types.WindowManager, typename, BoolProperty(name=entitytype))

    for scenetype in scenes.scene_types:
        if 'export' in scenes.scene_types[scenetype]:
            typename = "export_scene_" + scenetype
            setattr(bpy.types.WindowManager, typename, BoolProperty(name=scenetype))

    # Load custom icons
    import os
    pcoll = bpy.utils.previews.new()

    # load a preview thumbnail of a file and store in the previews collection
    pcoll.load("phobosIcon", os.path.join(os.path.dirname(__file__),
               "phobosIcon.png"), 'IMAGE')
    prev_collections["phobos"] = pcoll

    global phobosIcon
    pcoll = prev_collections["phobos"]
    phobosIcon = pcoll["phobosIcon"].icon_id

    # OPT: Icons for phobostypes will be added here
    global phobostypeIcons
    pcoll = prev_collections["phobos"]
    phobostypeIcons = {}

    # this needs to be registered after all contained data is set
    global supportedProps
    supportedProps = {
        'phobostype': {
            'infoparams': {
                'icon_value': phobosIcon
            }
        },
        'geometry/type': {
            'infoparams': {}
        },
        'motor/type': {
            'operator': 'phobos.add_motor',
            'infoparams': {
                'icon_value': phobosIcon
            }
        },
        'joint/type': {
            'operator': 'phobos.define_joint_constraints',
            'infoparams': {}
        },
        'geometry/type': {
            'operator': 'phobos.define_geometry',
            'infoparams': {}
        }
    }

    global supportedCategories
    supportedCategories = {
        # CHECK this might become interesting in the future otherwise just delete it
    }

    # TODO delete me?
    # Register classes (cannot be automatic, as panels are placed in gui in the registering order)
    #     for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
    #         try:
    #             if classdef.__bases__[0] != bpy.types.Panel:
    #                 bpy.utils.register_class(classdef)
    #         except ValueError:
    #             print('Error with class registration:', key, classdef)
    bpy.utils.register_class(ModelPoseProp)
    bpy.utils.register_class(PhobosPrefs)
    bpy.utils.register_class(PhobosExportSettings)
    # TODO delete me?
    # bpy.utils.register_class(Mesh_Export_UIList)
    # bpy.utils.register_class(Models_Poses_UIList)

    # CHECK is this needed and right?
    bpy.utils.register_class(MatrixPropGroup)
    bpy.utils.register_class(PhobosMatrixPanel)
    bpy.utils.register_class(PhobosObjectInformationPanel)
    bpy.utils.register_class(PhobosPropertyInformationPanel)
    bpy.types.Object.phobosmatrixinfo = PointerProperty(type=MatrixPropGroup)
    bpy.types.Scene.phobospropcategories = EnumProperty(items=[])

    bpy.utils.register_class(PhobosToolsPanel)
    bpy.utils.register_class(PhobosDisplayPanel)
    bpy.utils.register_class(PhobosModelPanel)
    # TODO delete me?
    # bpy.utils.register_class(PhobosScenePanel)
    bpy.utils.register_class(PhobosSubmodelsPanel)
    bpy.utils.register_class(PhobosExportPanel)
    bpy.utils.register_class(PhobosImportPanel)
    bpy.utils.register_class(PhobosObjectPanel)

    # add phobos settings to scene/world
    bpy.types.WindowManager.phobosexportsettings = PointerProperty(
        type=PhobosExportSettings)
    # TODO move other stuff to windowmanager instead of world
    bpy.types.Scene.active_ModelPose = bpy.props.IntProperty(
        name="Index of current pose", default=0, update=showPreview)
    bpy.types.Scene.preview_visible = bpy.props.BoolProperty(
        name="Is the draw preview operator running", default=False)
    bpy.types.Scene.redraw_preview = bpy.props.BoolProperty(
        name="Should we redraw the preview_template", default=False)

    # Add manuals to operator buttons
    bpy.utils.register_manual_map(get_operator_manuals)

    # TODO delete me?
    # Read in model and pose data from the respective folders
    # loadModelsAndPoses()
    libraries.register()


def unregister():
    print("Unregistering phobosgui...")
    libraries.unregister()

    # Unregister icons
    for pcoll in prev_collections.values():
        bpy.utils.previews.remove(pcoll)
    prev_collections.clear()

    # Unregister classes
    for key, classdef in inspect.getmembers(sys.modules[__name__],
                                            inspect.isclass):
        bpy.utils.unregister_class(classdef)

    # Remove manuals from buttons
    bpy.utils.unregister_manual_map(get_operator_manuals)
