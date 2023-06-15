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

import bpy
from bpy.props import (
    BoolProperty,
    IntProperty,
    StringProperty,
    EnumProperty,
    PointerProperty,
    CollectionProperty,
    FloatProperty,
)
from bpy.types import AddonPreferences

from . import display, defs, reserved_keys
from .io import blender2phobos
from .model import mechanisms
from .phoboslog import log, LOGLEVELS
from .utils import io as ioUtils
from .utils import naming as nUtils
from .utils import selection as sUtils
from .utils import validation as validation

from .. import defs as phobos_defs
from ..commandline_logging import setup_logger_level
from ..utils.resources import get_blender_resources_path


class ModelPoseProp(bpy.types.PropertyGroup):
    """TODO Missing documentation"""

    # DOCU missing class description
    robot_name : StringProperty()
    label : StringProperty()
    hide : BoolProperty(default=True)
    parent : StringProperty()
    icon : StringProperty()
    type : StringProperty()
    path : StringProperty()
    model_file : StringProperty()
    preview : StringProperty()


def set_loglevel(self, level):
    setup_logger_level(LOGLEVELS[level] if level != LOGLEVELS.index("NONE") else 1)
    self["loglevel"] = level


def get_username():
    try:
        user = os.getlogin()
    except:
        user = ""
    return user


class PhobosPrefs(AddonPreferences):
    """The general Phobos addon settings are stored in this class.
    They can be edited in the User Preferences of Blender under the Addon tab.

    Args:

    Returns:

    """

    bl_idname = "phobos"

    modelsfolder: StringProperty(name="modelsfolder", subtype="DIR_PATH",
                                 default=os.path.expanduser(os.path.join("~", "phobos-models")))

    username : StringProperty(
        name='username',
        default=get_username(),
        description="Name of the user/company (used for export information etc.)",
    )

    useremail : StringProperty(
        name='useremail',
        default='None',
        description="E-mail adress of the user/company (used for export information etc.)",
    )

    logactive : BoolProperty(default=False, name='logactive', description="Activate logging")

    logfile : StringProperty(name="logfile", subtype="FILE_PATH", default=".")

    loglevel : EnumProperty(
        name="loglevel", items=tuple(((l,) * 3 for l in LOGLEVELS)), default="WARNING",
        set=set_loglevel, get=lambda self: self["loglevel"] if "loglevel" in self else LOGLEVELS.index("WARNING")
    )

    logtofile : BoolProperty(name="logtofile", default=False)

    logtoterminal : BoolProperty(name="logtoterminal", default=True)

    models_poses : CollectionProperty(type=ModelPoseProp)

    # obj optional information
    axis_forward_items = (
        (item, item + ' forward', item) for item in ('X', 'Y', 'Z', '-X', '-Y', '-Z')
    )
    axis_up_items = ((item, item + ' up', item) for item in ('X', 'Y', 'Z', '-X', '-Y', '-Z'))
    obj_axis_forward : EnumProperty(
        items=axis_forward_items,
        name='Forward',
        description="Forward axis of the obj export.",
        default='Y',
    )
    obj_axis_up : EnumProperty(
        items=axis_up_items, name='Up', description="Up axis of the obj export.", default='Z'
    )

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout

        box = layout.box()
        box.label(text="Folders")
        box.prop(self, "modelsfolder", text="models folder")
        layout.separator()

        box = layout.box()
        box.label(text="User information")
        box.prop(self, "username", text="user name")
        box.prop(self, "useremail", text="user email")
        layout.separator()

        box = layout.box()
        row = box.row()
        row.label(text="Logging")
        box.prop(self, "logfile", text="log file")
        box.prop(self, "logtofile", text="write to logfile")
        box.prop(self, "logtoterminal", text="write to terminal")
        box.prop(self, "loglevel", text="log level")

        box = layout.box()
        row1 = box.row()
        row1.label(text="OBJ export frame")
        row2 = box.row()
        row2.label(text="Some loaders e.g. OSG use different conventions.See Phobos-Documentation on meshes. "
                   "The default is Z-Up Y-Forward. "
                   "You should adapt this only if you are working e.g. with an old version of MARS.")
        box.prop(self, 'obj_axis_forward')
        box.prop(self, 'obj_axis_up')


prev_collections = {}
phobosIcon = 0


class PhobosExportSettings(bpy.types.PropertyGroup):
    """TODO Missing documentation"""

    # DOCU missing class description

    def updateExportPath(self, context):
        """
        Makes sure that a valid path is given, if the path field is empty
        """
        if bpy.data.filepath and bpy.context.scene.phobosexportsettings.path == "":
            bpy.context.scene.phobosexportsettings.path = "//"
        elif (not bpy.data.filepath and bpy.context.scene.phobosexportsettings.path.startswith("//")) or\
                bpy.context.scene.phobosexportsettings.path == "":
            bpy.context.scene.phobosexportsettings.path = bpy.context.preferences.addons["phobos"].preferences.modelsfolder

    def getXMLTypeListForEnumProp(self, context):
        """

        Args:
          context:

        Returns:

        """
        # DOCU missing description
        return [(mt,) * 3 for mt in phobos_defs.KINEMATIC_TYPES]
        # As soon smurf will hold this information
        # return [(mt,) * 3 for mt in ["None"] + phobos_defs.KINEMATIC_TYPES]

    def getMeshTypeListForEnumProp(self, context):
        """

        Args:
          context:

        Returns:

        """
        # DOCU missing description
        return [(mt,) * 3 for mt in phobos_defs.MESH_TYPES]

    path : StringProperty(
        name='path',
        subtype='DIR_PATH',
        default=(""
                 if bpy.context.preferences.addons["phobos"].preferences is None
                 else bpy.context.preferences.addons["phobos"].preferences.modelsfolder),
        update=updateExportPath
    )

    # TODO: CHECK which props are visible in GUI?
    selectedOnly : BoolProperty(
        name="Selected only", default=False, description="Export only selected objects"
    )

    applyMeshScale: BoolProperty(
        name="Apply mesh scale", default=False, description="Before Exporting Mesh scales are applied."
    )

    # smurfDecimalPlaces : IntProperty(
    #     name="decimals", description="Number of " + "decimal places to export in smurf", default=5, min=3
    # )
    urdfDecimalPlaces : IntProperty(
        name="decimals", description="Number of " + "decimal places to export in urdf", default=5, min=3
    )
    sdfDecimalPlaces : IntProperty(
        name="decimals", description="Number of " + "decimal places to export in sdf", default=5, min=3
    )
    ensurePositiveSemiDefinite: BoolProperty(
        name="Ensure Positive-Semi-Definite Inertias",
        description="Ensure that the Inertia matrices are positive semi definite, forces diagonal of zero-matrices to 1e-5",
        default=False
    )
    enforceZero: BoolProperty(
        name="Force almost zero to zero",
        description="Depending on the decimal places values that are close to zero will be set to zero",
        default=False
    )

    exportTextures : BoolProperty(name='Export textures', default=True)
    # outputMeshtype : EnumProperty(
    #     items=getMeshTypeListForEnumProp,
    #     name='link',
    #     description="Mesh type to use in exported " + "entity/scene files.",
    # )
    ros_rel_choices = ["relative", "ros_package", "relative + ros_style", "ros_package + relative"]
    urdfOutputPathtype : EnumProperty(
        items=tuple(((l,) * 3 for l in ros_rel_choices)),
        name='file path',
        description="Defines how pathes are generated in " + "entity/scene files.",
    )

    sdfOutputPathtype: EnumProperty(
        items=tuple(((l,) * 3 for l in ros_rel_choices)),
        name='file path',
        description="Defines how pathes are generated in " + "entity/scene files.",
    )

    # prefixExport : StringProperty(
    #     name='prefix export',
    #     default="",
    #     description="Use the given string to prefix all items (links, visuals, collisions, etc.)",
    # )

    # TODO make this default to the model name
    rosPackageName : StringProperty(name='ROS package name', default='robot_name_model')

    export_smurf_xml_type : EnumProperty(
        items=getXMLTypeListForEnumProp,
        name='Kinematic representation',
        description="Type of Kinematic representation to use in exported SMURF.",
    )

    export_urdf_mesh_type : EnumProperty(
        items=getMeshTypeListForEnumProp,
        name='URDF mesh type',
        description="Mesh type to use in exported URDF files.",
    )

    export_sdf_mesh_type : EnumProperty(
        items=getMeshTypeListForEnumProp,
        name='SDF mesh type',
        description="Mesh type to use in exported SDF files.",
    )

    export_sdf_model_config : BoolProperty(
        default=False,
        name='Export Gazebo model.config',
        description='Export model.config file along with the SDF file.',
    )

    export_sdf_to_gazebo_models : BoolProperty(
        default=False,
        name='Export to Gazebo models folder',
        description='Export model to the Gazebo models folder.',
    )

    export_default_submodel_abstract : BoolProperty(
        default=False,
        name='Serial/abstract',
        description='Export a submodel only with the abstract joints.',
    )

    export_default_submodel_floatingbase: BoolProperty(
        default=False,
        name='Floating base',
        description='Export a submodel that has a floating base joint structrure before the root.',
    )



# class Mesh_Export_UIList(bpy.types.UIList):
#     """TODO Missing documentation"""
#
#     # DOCU missing class description
#     # CHECK is this class in use? -> no, therefore commented
#
#     def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
#         """
#
#         Args:
#           context:
#           layout:
#           data:
#           item:
#           icon:
#           active_data:
#           active_propname:
#           index:
#
#         Returns:
#
#         """
#         # TODO remove this code?
#         # assert(isinstance(item, bpy.types.MaterialTextureSlot)
#         ma = data
#         slot = item
#         tex = slot.texture if slot else None
#         if self.layout_type in {'DEFAULT', 'COMPACT'}:
#             if tex:
#                 layout.prop(tex, "name", text="", emboss=False, icon_value=icon)
#             else:
#                 layout.label(text="", icon_value=icon)
#             if tex and isinstance(item, bpy.types.MaterialTextureSlot):
#                 layout.prop(ma, "use_textures", text="", index=index)
#         elif self.layout_type == 'GRID':
#             layout.alignment = 'CENTER'
#             layout.label(text="", icon_value=icon)


# class Models_Poses_UIList(bpy.types.UIList):
#     """TODO Missing documentation"""
#
#     # DOCU missing class description
#     # CHECK is this class in use? -> no, therefore commented
#
#     def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
#         """
#
#         Args:
#           context:
#           layout:
#           data:
#           item:
#           icon:
#           active_data:
#           active_propname:
#           index:
#
#         Returns:
#
#         """
#         self.use_filter_show = False
#         im = item
#         modelsPosesColl = bpy.context.preferences.addons["phobos"].preferences.models_poses
#         if im.name in modelsPosesColl.keys():
#             coll_item = modelsPosesColl[im.name]
#             if coll_item.type == "robot_name":
#                 layout.label(text=coll_item.label, translate=False, icon=coll_item.icon)
#             else:
#                 sLayout = layout.split(0.1)
#                 sLayout.label(text="")
#                 if im.filepath != '':
#                     sLayout.label(text=coll_item.label, translate=False, icon_value=icon)
#                 else:
#                     sLayout.label(text=coll_item.label, translate=False, icon=coll_item.icon)
#
#     def filter_items(self, context, data, propname):
#         """
#
#         Args:
#           context:
#           data:
#           propname:
#
#         Returns:
#
#         """
#         images = getattr(data, propname)
#         flt_flags = [self.bitflag_filter_item] * len(images)
#
#         modelsPosesColl = bpy.context.preferences.addons["phobos"].preferences.models_poses
#
#         # Filter items. Only show robots. Hide all other images
#         for idx, im in enumerate(images):
#             if im.name in modelsPosesColl.keys():
#                 curr_model = modelsPosesColl[im.name]
#                 if curr_model.hide and not (curr_model.type == "robot_name"):
#                     flt_flags[idx] &= ~self.bitflag_filter_item
#             else:
#                 flt_flags[idx] &= ~self.bitflag_filter_item
#
#         # FIXME remove this (never used)
#         helper_funcs = bpy.types.UI_UL_list
#         # Reorder by name
#         flt_neworder = []
#         noPreviewIndex = 0
#         for im in images:
#             newIndex = 0
#             if im.name in modelsPosesColl.keys():
#                 newIndex = modelsPosesColl.keys().index(im.name)
#             else:
#                 newIndex = len(modelsPosesColl) + noPreviewIndex
#                 noPreviewIndex += 1
#             flt_neworder.append(newIndex)
#
#         return flt_flags, flt_neworder


def showPreview(self, value):
    """

    Args:
      value:

    Returns:

    """
    # CHECK this should be a class function
    bpy.ops.scene.change_preview()


class PhobosToolsPanel(bpy.types.Panel):
    """Contains all general phobos tools in the Phobos viewport toolbar"""

    bl_idname = "TOOLS_PT_PHOBOS_TOOLS"
    bl_label = "General Tools"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Phobos'

    def draw_header(self, context):
        """

        Args:
          context:

        Returns:

        """
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout

        # Tools & Selection Menu
        tsinlayout = layout.split()
        tsc1 = tsinlayout.column(align=True)
        tsc1.label(text="Select...", icon='HAND')
        tsc1.operator('phobos.select_root', text='Root')
        tsc1.operator('phobos.select_model', text='Robot')
        tsc1.operator('phobos.select_objects_by_phobostype', text="by Phobostype")
        tsc1.operator('phobos.select_objects_by_name', text="by Name")
        tsc2 = tsinlayout.column(align=True)
        tsc2.label(text="Tools", icon='MODIFIER')
        tsc2.operator('phobos.sort_objects_to_layers', icon='IMGDISPLAY')
        tsc2.operator('phobos.set_xray')
        # [TODO v2.1.0] REVIEW this
        # tsc2.operator('phobos.toggle_namespaces')
        tsc2.operator('phobos.measure_distance')
        # [TODO v2.1.0] Re-add this
        # tsc2.operator('phobos.validate')


# TODO-REFACTOR move this to utils.misc
def getMatrixData(coord, space):
    """

    Args:
      coord:
      space:

    Returns:

    """
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
    """TODO Missing documentation"""

    from bpy.props import FloatProperty

    loc_x_local : FloatProperty(
        name='location x',
        get=lambda self: getMatrixData('x', 'local'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='X coordinate in the local space',
    )
    loc_y_local : FloatProperty(
        name='location y',
        get=lambda self: getMatrixData('y', 'local'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='Y coordinate in the local space',
    )
    loc_z_local : FloatProperty(
        name='location z',
        get=lambda self: getMatrixData('z', 'local'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='Z coordinate in the local space',
    )
    rot_x_local : FloatProperty(
        name='rotation x',
        get=lambda self: getMatrixData('rotx', 'local'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around local x axis',
    )
    rot_y_local : FloatProperty(
        name='rotation y',
        get=lambda self: getMatrixData('roty', 'local'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around local y axis',
    )
    rot_z_local : FloatProperty(
        name='rotation z',
        get=lambda self: getMatrixData('rotz', 'local'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around local z axis',
    )
    loc_x_world : FloatProperty(
        name='location x',
        get=lambda self: getMatrixData('x', 'world'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='X coordinate in the world space',
    )
    loc_y_world : FloatProperty(
        name='location y',
        get=lambda self: getMatrixData('y', 'world'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='Y coordinate in the world space',
    )
    loc_z_world : FloatProperty(
        name='location z',
        get=lambda self: getMatrixData('z', 'world'),
        unit='LENGTH',
        subtype='DISTANCE',
        description='Z coordinate in the world space',
    )
    rot_x_world : FloatProperty(
        name='rotation x',
        get=lambda self: getMatrixData('rotx', 'world'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around world x axis',
    )
    rot_y_world : FloatProperty(
        name='rotation y',
        get=lambda self: getMatrixData('roty', 'world'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around world y axis',
    )
    rot_z_world : FloatProperty(
        name='rotation z',
        get=lambda self: getMatrixData('rotz', 'world'),
        unit='ROTATION',
        subtype='ANGLE',
        description='Rotation around world z axis',
    )


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
        """

        Args:
          context:

        Returns:

        """
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout

        obj = context.active_object

        matrixes = layout.split()
        localcol = matrixes.column(align=True)
        worldcol = matrixes.column(align=True)

        # Local data first
        #todo: localcol.label(text='local', icon='ROTACTIVE')
        localcol.label(text='local')
        # add all location properties
        for locprop in dir(obj.phobosmatrixinfo):
            if locprop.startswith('loc') and locprop.endswith('local'):
                localcol.prop(obj.phobosmatrixinfo, locprop, text=locprop[4] + ' location')

        localcol.separator()
        # add all rotation properties
        for rotatprop in dir(obj.phobosmatrixinfo):
            if rotatprop.startswith('rot') and rotatprop.endswith('local'):
                localcol.prop(obj.phobosmatrixinfo, rotatprop, text=rotatprop[4] + ' rotation')

        # world data second
        worldcol.label(text='world', icon='WORLD')
        # add all location properties
        for locprop in dir(obj.phobosmatrixinfo):
            if locprop.startswith('loc') and locprop.endswith('world'):
                worldcol.prop(obj.phobosmatrixinfo, locprop, text=locprop[4] + ' location')
        worldcol.separator()
        # add all rotation properties
        for rotatprop in dir(obj.phobosmatrixinfo):
            if rotatprop.startswith('rot') and rotatprop.endswith('world'):
                worldcol.prop(obj.phobosmatrixinfo, rotatprop, text=rotatprop[4] + ' rotation')


class PhobosObjectInformationPanel(bpy.types.Panel):
    """Contains information like parent, immediate children etc. in the Buttons Window."""

    bl_idname = "OBJINFO_PT_PHOBOS_TOOLS"
    bl_label = "Phobos Object Information"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "object"

    def draw_header(self, context):
        """

        Args:
          context:

        Returns:

        """
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """

        layout = self.layout
        obj = context.active_object
        modelname = ''
        rootname = ''

        root = sUtils.getRoot(obj)
        if 'model/name' in root:
            modelname = root['model/name']
        rootname = root.name

        row = layout.row()
        descr = row.column()
        content = row.column()

        #todo: descr.label(text='Part of model', icon='POSE_DATA')
        descr.label(text='Part of model')
        content.operator('phobos.name_model', text=modelname, icon='OUTLINER_DATA_FONT')

        #todo: descr.label(text='Root object', icon='OOPS')
        descr.label(text='Root object')
        if obj == root:
            content.label(text="selected", icon='MATCUBE')
        else:
            content.operator('phobos.select_root', text=rootname, icon='FILE_PARENT')

        # add parent object if available
        if obj.parent:
            descr.label(text='Parent object', icon='CONSTRAINT')
            goto_op = content.operator(
                'phobos.goto_object', icon='FILE_PARENT', text='{0}'.format(obj.parent.name)
            )
            goto_op.objectname = obj.parent.name

        layout.separator()
        row = layout.row()
        row.label(icon="OBJECT_DATA", text="Phobostype")
        row.prop(context.active_object, 'phobostype', text="")

        # show object name as button
        row = layout.row()
        row.label(icon='COPY_ID', text="Object name")
        row.operator(
            'phobos.change_object_name', icon='OUTLINER_DATA_FONT', text=nUtils.getObjectName(obj)
        )


class PhobosModelWarningsPanel(bpy.types.Panel):
    """TODO Missing documentation"""

    bl_idname = "MODELWARNINGS_PT_PHOBOS_TOOLS"
    bl_label = "Phobos Model Warnings"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "object"

    def draw_header(self, context):
        """

        Args:
          context:

        Returns:

        """
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        obj = context.active_object

        # show naming errors in the UI
        # errors = validation.validateObjectNames(obj)
        # if errors:
        #     layout.separator()
        #     errorname = ""
        #     for error in sorted(errors):
        #         if errorname != error.message[:5]:
        #             row = layout.row(align=True)
        #             errorname = error.message[:5]
        #             col1 = row.column(align=True)
        #             col2 = row.column(align=True)
        #             col3 = row.column(align=True)
        #         if error.level == 'WARNING':
        #             col1.label(icon='ERROR')
        #         elif error.level == 'ERROR':
        #             col1.label(icon='CANCEL')
        #         else:
        #             col1.label(icon='QUESTION')
        #
        #         col2.label(text=error.message)
        #         col3.operator(error.operator, text="Fix")


ignoredProps = set(
    [
        'cycles',
        'cycles_visibility',
        'phobosmatrixinfo',
        'phobostype',
        'name',
        'pose',
        'size',
        'scale',
        'parent',
        'approxcollision',
    ]
)


# [TODO v2.0.0] Improve e.g. add edit tools
class PhobosPropertyInformationPanel(bpy.types.Panel):
    """Contains all properties sorted in different categories"""

    bl_idname = "PROPINFO_PT_PHOBOS_TOOLS"
    bl_label = "Phobos Property Information"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "object"

    def addProp(self, props, values, layout, guiparams):
        """Add a property/list of properties to the specified layout category.

        Args:
          props(list/str): property or list of property names to add
          values(list): list of or single float, str etc which corresponds to the property name
          layout(list): sublayout description as defined in :func:draw
          guiparams:

        Returns:
          None: None

        """
        # get the existing layout columns
        column = layout[1]

        # add all properties in sequence
        for _, (prop, value, param) in enumerate(zip(props, values, guiparams)):
            # objects are shown as goto operator
            if isinstance(value, bpy.types.Object):
                self.addObjLink(prop, value, column, param)
                continue
            elif prop in ["link", "joint", "parent", "child"] and type(value) == str:
                obj_value = sUtils.getObjectByName(value)
                if obj_value is not None:
                    self.addObjLink(prop, obj_value, column, param, label=value)
                    continue

            subtable = column.split(factor=0.45)
            descr = subtable.column()
            content = subtable.column()

            # add without additional settings
            descr.label(text='{0}'.format(prop))

            # show operators as button and other as label
            if 'operator' in param:
                content.operator(param['operator'], text=str(value))  # , **param['infoparams'])
            # show property as a customprop
            elif 'object' in param and 'customprop' in param:
                content.prop(param['object'], '["{0}"]'.format(param['customprop']), text="")
            # just show it as text
            else:
                content.label(text=str(value) if not hasattr(value, "to_list") else str(value.to_list()))

    def addObjLink(self, prop, value, column, param, label=None):
        """Add an object link, which is a clickable goto button in the GUI.

        Args:
          prop:
          value:
          column:
          guiparams(dict): parameters for the GUI of Blender

        Returns:
          None: None

        """
        # this list is used to force labelling of special keywords
        subtable = column.split(factor=0.45)
        descr = subtable.column()
        content = subtable.column()

        descr.label(text=prop)
        # use custom params (like icons etc) from the dictionary by passing **params
        goto_op = content.operator(
            'phobos.goto_object',
            text=value.name if label is None else label,
            # **guiparams['infoparams'],
            icon='FILE_PARENT'
        )
        goto_op.objectname = value.name

    def draw_header(self, context):
        """

        Args:
          context:

        Returns:

        """
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        obj = context.active_object

        # derive object information as dictionary
        joint = None
        motor = None
        if obj.phobostype == 'link' and not sUtils.isRoot(obj):
            obj_repr = blender2phobos.deriveLink(obj)
            if any([k.startswith("joint/") for k in obj.keys()]):
                joint = blender2phobos.deriveJoint(obj)
            if any([k.startswith("motor/") for k in obj.keys()]):
                motor = blender2phobos.deriveMotor(obj)
        else:
            obj_repr = blender2phobos.deriveRepresentation(obj, logging=False, adjust=False)

        # categories saves the sublayouts as a dictionary, which contains this information:
        #  surrounding_box, left_column, right_column, [index_row, index_col]-> for next entry
        categories = {}

        keys = getattr(reserved_keys, obj.phobostype.upper()+"_KEYS", None)
        if keys is not None:
            if obj.phobostype in supportedCategories:
                layout.label(text=obj.phobostype[0].upper()+obj.phobostype[1:], icon=supportedCategories[obj.phobostype]['icon_value'])
            else:
                layout.label(text=obj.phobostype[0].upper()+obj.phobostype[1:])
            box = layout.box()
            row = box.split()
            column = row.column()
            guiparams = {'object': obj}
            categories[obj.phobostype] = [box, column, [0, 0]]
            for key in keys:
                _key = key if obj.phobostype != "link" else "link/"+key
                try:
                    if hasattr(obj_repr, key):
                        getattr(obj_repr, key)
                except AssertionError as e:
                    log(f"PhobosPropertyInformationPanel: Cannot display {str(obj.name)}.{key} due to internal assertion error: {e.args}",
                        "DEBUG")
                    continue
                if _key in obj:
                    self.addProp([key], [obj[_key]], categories[obj.phobostype], [guiparams])
                elif hasattr(obj_repr, key) and type(getattr(obj_repr, key)) in [str, float, int]:
                    self.addProp([key], [getattr(obj_repr, key)], categories[obj.phobostype], [guiparams])
                else:
                    log(f"PhobosPropertyInformationPanel: {str(obj.name)} has no entry for {key}", "DEBUG")

        if obj.phobostype == "link" and not sUtils.isRoot(obj):
            if joint is not None:
                layout.label(text="Joint", icon=supportedCategories["joint"]['icon_value'])
                box = layout.box()
                row = box.split()
                column = row.column()
                categories["joint"] = [box, column, [0, 0]]
                guiparams = {'object': obj}
                for key in reserved_keys.JOINT_KEYS:
                    _key = "joint/" + key
                    if _key in obj:
                        self.addProp([key], [obj[_key]], categories["joint"], [guiparams])
                    else:
                        try:
                            if hasattr(joint, key) and key in ["parent", "child"] and sUtils.getObjectByName(getattr(joint, key)) is not None:
                                self.addProp([key], [sUtils.getObjectByName(getattr(joint, key))], categories["joint"], [guiparams])
                            else:
                                log(f"PhobosPropertyInformationPanel: {str(obj.name)} has no entry for {key}", "DEBUG")
                        except AssertionError:
                            log(f"PhobosPropertyInformationPanel: {str(obj.name)}: {key} can't be displayed without the representation.Robot instance!", "DEBUG")

            if motor is not None:
                layout.label(text="Motor", icon=supportedCategories["motor"]['icon_value'])
                box = layout.box()
                row = box.split()
                column = row.column()
                categories["motor"] = [box, column, [0, 0]]
                guiparams = {'object': obj}
                for key in reserved_keys.MOTOR_KEYS:
                    _key = "motor/" + key
                    if _key in obj:
                        self.addProp([key], [obj[_key]], categories["motor"], [guiparams])
                    else:
                        log(f"PhobosPropertyInformationPanel: {str(obj.name)} has no entry for {key}", "DEBUG")
            if any([k.startswith("pose/") for k in obj.keys()]):
                layout.label(text="Pose", icon=supportedCategories["pose"]['icon_value'])
                box = layout.box()
                row = box.split()
                column = row.column()
                categories["pose"] = [box, column, [0, 0]]
                guiparams = {'object': obj}
                for _key in obj.keys():
                    if _key.startswith("pose/"):
                        key = _key[5:]
                        if _key in obj:
                            self.addProp([key], [obj[_key]], categories["pose"], [guiparams])
                        else:
                            log(f"PhobosPropertyInformationPanel: {str(obj.name)} has no entry for {key}", "DEBUG")

        # The following has been replaced and is obsolete
        # # generate general category only if needed
        # props = set([key for key in proplist if not isinstance(dictprops[key], dict)])
        # if props - ignoredProps:
        #     box = layout.box()
        #     row = box.split()
        #     left = row.column()
        #     right = row.column()
        #     categories['general'] = [box, left, right, [0, 0]]
        #
        # # remove the unimportant properties and iterate over the rest
        # proplist = set(proplist) - ignoredProps
        # for prop in sorted(proplist):
        #     guiparams = self.checkParams(prop)
        #     value = dictprops[prop]
        #
        #     # just a value for the general category
        #     if not isinstance(value, dict):
        #         # show properties as customproperties
        #         guiparams['object'] = obj
        #         if prop in obj:
        #             guiparams['customprop'] = prop
        #         elif obj.phobostype + '/' + prop in obj:
        #             guiparams['customprop'] = obj.phobostype + '/' + prop
        #         self.addProp([prop], [value], categories['general'], [guiparams])
        #         continue
        #
        #     # check for categories
        #     category = prop
        #
        #     # add a new category layout
        #     if category not in categories:
        #         # use custom icons for supported categories
        #         if category in supportedCategories:
        #             if isinstance(supportedCategories[category]['icon_value'], int):
        #                 layout.label(
        #                     text=category.upper(), icon_value=supportedCategories[category]['icon_value']
        #                 )
        #             else:
        #                 layout.label(
        #                     text=category.upper(), icon=supportedCategories[category]['icon_value']
        #                 )
        #         else:
        #             layout.label(text=category.upper())
        #
        #         # create column hierarchy
        #         box = layout.box()
        #         box = box.split()
        #         left = box.column()
        #         right = box.column()
        #         categories[category] = [box, left, right, [0, 0]]
        #
        #     # add each subproperty to the layout
        #     for prop_t2 in sorted(dictprops[category]):
        #         guiparams = self.checkParams(category + '/' + prop_t2)
        #
        #         value = dictprops[category][prop_t2]
        #
        #         # skip ignored properties
        #         if prop_t2 in ignoredProps:
        #             continue
        #
        #         # is it another dictionary with values?
        #         elif isinstance(value, dict):
        #             # gather keys, guiparams etc as lists
        #             props = value.keys()
        #             values = [value[key] for key in props]
        #             paramkeys = [category + '/' + prop_t2 + '/' + propkey for propkey in props]
        #             paramlist = [self.checkParams(key) for key in paramkeys]
        #             props = [prop_t2 + '/' + propkey for propkey in props]
        #
        #             # add the properties from the list
        #             self.addProp(props, values, categories[category], paramlist)
        #         # just another value
        #         else:
        #             self.addProp([prop_t2], [value], categories[category], [guiparams])


class PhobosModelPanel(bpy.types.Panel):
    """Contains all model editing tools in the Phobos viewport toolbar"""

    bl_idname = "TOOLS_PT_PHOBOS_MODEL"
    bl_label = "Model Editing"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Phobos'

    def draw_header(self, context):
        """

        Args:
          context:

        Returns:

        """
        # TODO decide on icon
        # self.layout.label(icon='OUTLINER_DATA_ARMATURE')
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
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
        c1.label(text='General', icon='MESH_CUBE')
        c1.operator('phobos.set_phobostype')
        c1.operator('phobos.batch_rename')
        c1.operator('phobos.set_model_root')

        c2.label(text="Custom properties", icon='WORDWRAP_ON')
        c2.operator('phobos.rename_custom_property', text="Rename", icon='OUTLINER_DATA_FONT')
        c2.operator('phobos.batch_property', text="Edit", icon='GREASEPENCIL')
        #todo: c2.operator('phobos.copy_props', text="Copy", icon='GHOST')
        c2.operator("phobos.add_annotations")

        # Kinematics
        layout.separator()
        kinlayout = layout.split()
        kc1 = kinlayout.column(align=True)
        kc2 = kinlayout.column(align=True)
        kc1.label(text='Kinematics', icon='ARMATURE_DATA')
        kc1.operator("phobos.create_links")
        kc1.operator('phobos.merge_links')
        kc1.operator('phobos.dissolve_link')
        kc1.operator('phobos.define_joint_constraints')
        kc1.operator("phobos.create_mimic_joint")
        # [Todo v2.1.0] kc1.operator('phobos.add_kinematic_chain', icon='CONSTRAINT')
        kc1.operator('phobos.parent')

        # Visual/Collisions
        kc2.label(text='Visual/Collision', icon='GROUP')
        kc2.operator('phobos.define_geometry')
        kc2.operator('phobos.smoothen_surface')
        kc2.operator('phobos.create_collision_objects')
        kc2.operator('phobos.set_collision_group')

        # Mechanics
        layout.separator()
        mechlayout = layout.split()
        mc1 = mechlayout.column(align=True)
        mc2 = mechlayout.column(align=True)
        #todo: mc1.label(text='Mechanisms', icon='SCRIPTWIN')
        mc1.label(text='Mechanisms')
        mc1.operator('phobos.assign_submechanism')
        mc1.operator('phobos.select_submechanism')

        # Poses
        mc2.label(text='Poses', icon='POSE_HLT')
        mc2.operator('phobos.store_pose')
        mc2.operator('phobos.load_pose')
        mc2.operator('phobos.delete_pose')

        # Hardware
        layout.separator()
        minlayout = layout.split()
        hw1 = minlayout.column(align=True)
        hw1.label(text="Hardware", icon='MOD_SCREW')
        hw1.operator('phobos.add_motor')
        # [TODO v2.1.0] hw1.operator('phobos.add_controller')
        hw1.operator('phobos.add_sensor')
        hw1.operator('phobos.create_interface')

        # Masses & Inertia
        mc1 = minlayout.column(align=True)
        mc1.label(text="Masses & Inertia", icon='PHYSICS')
        mc1.operator('phobos.calculate_mass')
        mc1.operator('phobos.generate_inertial_objects')
        mc1.operator('phobos.edit_inertial_data')


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
#         modelsPosesColl = bpy.context.preferences.addons["phobos"].preferences.models_poses
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
    bl_region_type = 'UI'
    bl_category = 'Phobos'

    def draw_header(self, context):
        """

        Args:
          context:

        Returns:

        """
        # TODO decide on icon
        # self.layout.label(icon='EXPORT')
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        expsets = bpy.context.scene.phobosexportsettings
        layout = self.layout

        # export robot model options
        layout.prop(expsets, "path")
        ginlayout = self.layout.split()
        g1 = ginlayout.column(align=True)
        g1.prop(expsets, "exportTextures")
        g1.prop(expsets, "selectedOnly")
        g1.prop(expsets, "applyMeshScale")
        g2 = ginlayout.column(align=True)
        g2.prop(expsets, "ensurePositiveSemiDefinite")
        g2.prop(expsets, "enforceZero")

        layout.separator()

        # Settings for mesh and entity export
        inlayout = self.layout.split()

        cmodel = inlayout.column(align=True)
        cmodel.label(text="Models")
        for entitytype in phobos_defs.EXPORT_TYPES:
            cmodel.prop(bpy.context.scene, "export_entity_"+entitytype)

        cmesh = inlayout.column(align=True)
        cmesh.label(text="Meshes")
        for meshtype in phobos_defs.MESH_TYPES:
            typename = "export_mesh_" + meshtype
            cmesh.prop(bpy.context.scene, typename)

        # Settings for submodel export
        inlayout2 = self.layout.split()

        csubm = inlayout2.column(align=True)
        csubm.label(text="Default Submodels")
        csubm.prop(expsets, "export_default_submodel_abstract")
        csubm.prop(expsets, "export_default_submodel_floatingbase")

        # [TODO v2.1.0] make this work again in an extra Phobos Scenes tab
        # cscene = inlayout.column(align=True)
        # cscene.label(text="Scenes")
        # for scenetype in ioUtils.getSceneTypesForExport():
        #     typename = "export_scene_" + scenetype
        #     cscene.prop(bpy.context.scene, typename)

        if getattr(bpy.context.scene, 'export_entity_smurf', False):
            layout.separator()
            box = layout.box()
            box.label(text='SMURF export')
            box.prop(ioUtils.getExpSettings(), 'export_smurf_xml_type')
            # box.prop(ioUtils.getExpSettings(), 'smurfDecimalPlaces')
        if getattr(bpy.context.scene, 'export_entity_urdf', False):
            layout.separator()
            box = layout.box()
            box.label(text='URDF export')
            box.prop(ioUtils.getExpSettings(), 'urdfDecimalPlaces')
            box.prop(ioUtils.getExpSettings(), 'export_urdf_mesh_type')
            box.prop(bpy.context.scene.phobosexportsettings, 'urdfOutputPathtype')
        if getattr(bpy.context.scene, 'export_entity_sdf', False):
            layout.separator()
            box = layout.box()
            box.label(text='SDF export')
            box.prop(ioUtils.getExpSettings(), 'sdfDecimalPlaces')
            box.prop(ioUtils.getExpSettings(), 'export_sdf_mesh_type')
            box.prop(bpy.context.scene.phobosexportsettings, 'sdfOutputPathtype')
            # doesn't work properly therefore excluded
            # box.prop(ioUtils.getExpSettings(), 'export_sdf_model_config', icon='RENDERLAYERS')
            # box.prop(ioUtils.getExpSettings(), 'export_sdf_to_gazebo_models', icon='EXPORT')

        if getattr(bpy.context.scene.phobosexportsettings, 'urdfOutputPathtype', "relative").startswith("ros_package") or \
                getattr(bpy.context.scene.phobosexportsettings, 'sdfOutputPathtype', "relative").startswith("ros_package"):
            layout.separator()
            box = layout.box()
            box.prop(expsets, "rosPackageName")

        # TODO delete me?
        # c2.prop(expsets, "exportCustomData", text="Export custom data")

        # TODO delete me?
        # layout.separator()
        # layout.label(text="Baking")
        # layout.operator("phobos.export_bake", text="Bake Robot Model", icon="OUTLINER_OB_ARMATURE")
        # layout.operator("phobos.create_robot_instance", text="Create Robot Lib Instance", icon="RENDERLAYERS")

        # self.layout.prop(expsets, "heightmapMesh", text="export heightmap as mesh")

        layout.separator()
        splitlayout = layout.split()
        #c1 = splitlayout.column()
        c2 = splitlayout.column()
        #c1.label(text="Export Configuration") # doesn't work properly therefore excluded
        #c1.operator('phobos.move_to_scene', text='Move to configuration', icon='SCREEN_BACK')
        #c1.operator(
        #    'phobos.safely_remove_objects_from_scene', text='Remove from configuration', icon='X'
        #)
        c2.label(text="Export")
        c2.operator("phobos.export_model", icon="EXPORT")
        # [TODO v2.1.0] make this work again in an extra Phobos Scenes tab
        # c2.operator("phobos.export_scene", icon="WORLD_DATA") # doesn't work properly therefore excluded

    def check(self, context):
        """

        Args:
          context:

        Returns:

        """
        return True


class PhobosImportPanel(bpy.types.Panel):
    """Contains the import settings in the Phobos viewport toolbar"""

    bl_idname = "TOOLS_IMPORT_PT_PHOBOS"
    bl_label = "Import"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Phobos'

    def draw_header(self, context):
        """

        Args:
          context:

        Returns:

        """
        # TODO decide on icon
        # self.layout.label(icon='IMPORT')
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        self.layout.operator("phobos.import_robot_model", text="Import Robot Model", icon="IMPORT")

# TODO-REFACTOR this completely according to submodels in core or delete
# class PhobosSubmodelsPanel(bpy.types.Panel):
#     """TODO Missing documentation"""
#
#     bl_idname = "TOOLS_SUBMODELS_PT_PHOBOS"
#     bl_label = "Submodels"
#     bl_space_type = 'VIEW_3D'
#     bl_region_type = 'UI'
#     bl_category = 'Phobos Models'
#
#     def draw_header(self, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         self.layout.label(icon_value=phobosIcon)
#
#     def draw(self, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         self.layout.operator("phobos.define_submodel")
#         self.layout.operator("phobos.add_submodel")
#         self.layout.operator("phobos.toggle_interfaces")
#         self.layout.operator("phobos.connect_interfaces")


# class PhobosModelLibraryPanel(bpy.types.Panel):
#     """TODO Missing documentation"""
#
#     # DOCU add some docstring and update bl_idname
#     bl_idname = "TOOLS_PT_PHOBOS_LOCALMODELS"
#     bl_space_type = 'VIEW_3D'
#     bl_region_type = 'UI'
#     bl_category = "Phobos Models"
#     bl_label = "Local Model Library"
#
#     def draw_header(self, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         self.layout.label(icon_value=phobosIcon)
#
#     def draw(self, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         layout = self.layout
#         wm = context.window_manager
#         modelsfolder = bpy.context.preferences.addons["phobos"].preferences.modelsfolder
#         if modelsfolder == '':
#             layout.label(text='Model folder not configured.')
#             return
#
#         layout.operator("phobos.update_model_library", icon="FILE_REFRESH")
#
#         if wm.category != '-':
#             layout.prop(wm, 'category')
#
#             if wm.modelpreview != '-':
#                 layout.template_icon_view(wm, 'modelpreview', show_labels=True, scale=5.0)
#                 layout.prop(wm, 'modelpreview')
#                 layout.operator("phobos.import_model_from_library", icon="IMPORT")
#             else:
#                 layout.label(text='No models in this category.')
#         else:
#             layout.label(text='Model library is empty.')


def get_operator_manuals():
    """Returns a tuple with the Phobos wiki Operator page and pairs of operator
    names and wiki page anchor names to allow for linking from Blender to wiki.
    :return: tuple

    Args:

    Returns:

    """
    # CHECK does the linking work with the new wiki?
    url_manual_prefix = "https://github.com/dfki-ric/phobos/wiki/Operators#"
    url_manual_ops = tuple(
        ('bpy.ops.phobos.' + opname, opname.replace('_', '-'))
        for opname in dir(bpy.ops.phobos)
        if not opname.startswith("__")
    )
    return url_manual_prefix, url_manual_ops


class PhobosDisplayPanel(bpy.types.Panel):
    """TODO Missing documentation"""

    bl_idname = "TOOLS_DISPLAY_PT_PHOBOS"
    bl_label = "Display"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Phobos'

    def draw_header(self, context):
        """

        Args:
          context:

        Returns:

        """
        self.layout.label(icon_value=phobosIcon)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        wm = context.window_manager
        layout = self.layout

        if not wm.drawing_status:
            layout.operator('phobos.display_information', icon='INFO')
        else:
            layout.prop(wm, 'drawing_status', icon='INFO')
            layout.separator()
            dlayout = layout.split()
            dc1 = dlayout.column(align=True)
            dc2 = dlayout.column(align=True)
            dc1.prop(wm, "draw_jointaxes")
            dc1.prop(wm, "draw_jointnames")
            dc1.prop(wm, "jointaxes_length")
            dc1.prop(wm, "draw_submechanisms")
            dc1.prop(wm, "draw_progress")
            dc2.prop(wm, "draw_messages")
            dc2.prop(wm, 'phobos_msg_count')
            dc2.prop(wm, 'phobos_msg_offset')


REGISTER_CLASSES = [
    ModelPoseProp,
    PhobosPrefs,
    PhobosExportSettings,
    # CHECK is this needed and right?
    MatrixPropGroup,
    PhobosMatrixPanel,
    PhobosObjectInformationPanel,
    PhobosPropertyInformationPanel,
    PhobosModelWarningsPanel,
    PhobosToolsPanel,
    PhobosDisplayPanel,
    PhobosModelPanel,
#    PhobosSubmodelsPanel,
    PhobosExportPanel,
    PhobosImportPanel,
]


def register():
    """TODO Missing documentation"""
    print("\nRegistering phobosgui...")


    # add phobostype to Blender objects
    bpy.types.Object.phobostype = EnumProperty(
        items=defs.phobostypes, name="type", description="Phobos object type"
    )

    # register drawing functions
    display.register()

    # add display properties to window manager
    bpy.types.WindowManager.draw_jointaxes = BoolProperty(name='Joint Axes', default=True)

    bpy.types.WindowManager.jointaxes_length = FloatProperty(name='Length', default=0.3)

    bpy.types.WindowManager.draw_submechanisms = BoolProperty(name='Submechanisms', default=True)

    bpy.types.WindowManager.draw_jointnames = BoolProperty(name='Joint names', default=True)

    bpy.types.WindowManager.draw_messages = BoolProperty(name='Messages', default=True)

    bpy.types.WindowManager.phobos_msg_count = IntProperty(
        name='show',
        default=5,
        min=0,
        max=20,
        description="How many Phobos log messages to show on screen",
    )

    bpy.types.WindowManager.phobos_msg_offset = IntProperty(
        name='offset',
        default=0,
        min=0,
        max=50,
        description="The Phobos log message index to start with",
    )

    bpy.types.WindowManager.draw_progress = BoolProperty(name='Progress', default=True)

    bpy.types.WindowManager.progress = FloatProperty(
        name='Progress', default=0, description="Progress value of custom Phobos progress bar."
    )

    # add i/o settings to scene to preserve settings for every model
    for meshtype in phobos_defs.MESH_TYPES:
        typename = "export_mesh_" + meshtype
        setattr(bpy.types.Scene, typename, BoolProperty(name=meshtype, default=False))

    for entitytype in phobos_defs.EXPORT_TYPES:
        typename = "export_entity_" + entitytype
        setattr(bpy.types.Scene, typename, BoolProperty(name=entitytype, default=entitytype == "smurf"))

    # [TODO v2.1.0] Re-add scene export
    # for scenetype in scenes.scene_types:
    #     if 'export' in scenes.scene_types[scenetype]:
    #         typename = "export_scene_" + scenetype
    #         setattr(bpy.types.Scene, typename, BoolProperty(name=scenetype, default=False))

    # Load custom icons

    pcoll = bpy.utils.previews.new()

    # load a preview thumbnail of a file and store in the previews collection
    pcoll.load("phobosIcon", get_blender_resources_path("images", "phobosIcon.png"), 'IMAGE')
    prev_collections["phobos"] = pcoll

    global phobosIcon
    pcoll = prev_collections["phobos"]
    phobosIcon = pcoll["phobosIcon"].icon_id

    # OPT: Icons for phobostypes will be added here
    global phobostypeIcons
    pcoll = prev_collections["phobos"]
    phobostypeIcons = {}

    #: contains categories which are supported by the Phobosgui (non-generic)
    #: Currently, only ``icon_value``s can be added (either as integer or string from Blender)
    global supportedCategories
    supportedCategories = {
        'collision': {'icon_value': 'PHYSICS'},
        'visual': {'icon_value': 'RESTRICT_VIEW_OFF'},
        'inertial': {'icon_value': 'SHADING_TEXTURE'},
        'joint': {'icon_value': 'CON_KINEMATIC'},
        'sensor': {'icon_value': 'OUTLINER_OB_FORCE_FIELD'},
        'pose': {'icon_value': 'POSE_HLT'},
        'link': {'icon_value': 'LINK_BLEND'},
        'motor': {'icon_value': 'PREFERENCES'}
    }

    # TODO delete me?
    # Register classes (cannot be automatic, as panels are placed in gui in the registering order)
    #     for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
    #         try:
    #             if classdef.__bases__[0] != bpy.types.Panel:
    #                 bpy.utils.register_class(classdef)
    #         except ValueError:
    #             print('Error with class registration:', key, classdef)

    for cls in REGISTER_CLASSES:
        bpy.utils.register_class(cls)
    # TODO delete me?
    # bpy.utils.register_class(Mesh_Export_UIList)
    # bpy.utils.register_class(Models_Poses_UIList)

    bpy.types.Object.phobosmatrixinfo = PointerProperty(type=MatrixPropGroup)
    bpy.types.Scene.phobospropcategories = EnumProperty(items=[])

    # TODO delete me?
    # bpy.utils.register_class(PhobosScenePanel)

    # add phobos settings to scene
    bpy.types.Scene.phobosexportsettings = PointerProperty(type=PhobosExportSettings)
    bpy.types.Scene.active_ModelPose = bpy.props.IntProperty(
        name="Index of current pose", default=0, update=showPreview
    )
    bpy.types.Scene.preview_visible = bpy.props.BoolProperty(
        name="Is the draw preview operator running", default=False
    )
    bpy.types.Scene.redraw_preview = bpy.props.BoolProperty(
        name="Should we redraw the preview_template", default=False
    )

    # Add manuals to operator buttons
    bpy.utils.register_manual_map(get_operator_manuals)

    # TODO delete me?
    # Read in model and pose data from the respective folders
    # loadModelsAndPoses()
    mechanisms.register()

    print('  ... successful.')


def unregister():
    """TODO Missing documentation"""
    print("Unregistering phobosgui...")
    mechanisms.unregister()

    print("Unregistering display...")
    display.unregister()

    # Unregister icons
    print("Unregistering icons...")
    for pcoll in prev_collections.values():
        bpy.utils.previews.remove(pcoll)
    prev_collections.clear()

    print("Unregistering classes...")
    # Unregister classes
    for cls in REGISTER_CLASSES:
        bpy.utils.unregister_class(cls)

    print("Unregistering manuals...")
    # Remove manuals from buttons
    bpy.utils.unregister_manual_map(get_operator_manuals)

    print('  ... successful.')
