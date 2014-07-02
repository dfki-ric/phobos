'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtmisctools.py

Created on 6 Jan 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
import math
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, FloatVectorProperty, EnumProperty
import marstools.mtupdate as mtupdate
import marstools.mt_oldexport as mt_oldexport
import marstools.mtmaterials as mtmaterials
import marstools.mtutility as mtutility
import marstools.mtdefs as mtdefs


def register():
    print("Registering mtmisctools...")

def unregister():
    print("Unregistering mtmisctools...")
    #bpy.utils.unregister_class(ExportModelOperator)
    #bpy.utils.unregister_class(ImportModelOperator)
    #bpy.utils.unregister_class(CreateMARSPropsOperator)
    #bpy.utils.unregister_class(BatchEditPropertyOperator)
    #bpy.utils.unregister_class(SmoothenSurfaceOperator)
    #bpy.utils.unregister_class(BatchSmoothenSurfaceOperator)
    #del bpy.types.VIEW3D_MT_object.append(add_object_button)


# class ImportModelOperator(Operator):#
#     """ExportModelOperator"""
#     bl_idname = "object.mt_export_robot"
#     bl_label = "Initialise MARS properties for all objects"
#     bl_options = {'REGISTER', 'UNDO'}
#
#     def execute(self, context):
#         #add selction of all layers bpy.ops.object.select_all()
#         mtoldimport.main()

#TODO: Do we still need this operator? May be covered by AddSensorOperator (mtsensors)
# class AddObjectsToSensorOperator(Operator):
#     """AddObjectsToSensorOperator"""
#     bl_idname = "object.mt_add_to_sensor"
#     bl_label = "Add selected objects to the list of the selected sensors."
#     bl_options = {'REGISTER', 'UNDO'}
#
#     def execute(self, context):
#         for obj in bpy.context.selected_objects:
#             # the problem is that python will mess up the right order, to a simple "for" will not work
#             # we need to look at the entire tree starting from the root and then check whether or not
#             # each object is selected
#         return{'FINISHED'}


class CalculateMassOperator(Operator):
    """CalculateMassOperator"""
    bl_idname = "object.mt_calculate_mass"
    bl_label = "Display mass of the selected objects in a pop-up window."

    def execute(self, context):
        mass = 0
        names = ""
        for obj in bpy.context.selected_objects:
            if obj.MARStype == "link":
                mass += obj["mass"]
                names += obj.name + " "
        bpy.ops.error.message('INVOKE_DEFAULT', type="mass of "+names, message=str(mass))
        return {'FINISHED'}

class NameModelOperator(Operator):
    """NameModelOperator"""
    bl_idname = "object.mt_name_model"
    bl_label = "Name model by assigning 'modelname' property to root node "
    bl_options = {'REGISTER', 'UNDO'}

    modelname = StringProperty(
        name = "modelname",
        default = "",
        description = "name of the robot model to be assigned")

    def execute(self, context):
        root = mtutility.getRoot(bpy.context.active_object)
        root["modelname"] = self.modelname
        return {'FINISHED'}

class SelectObjectsByMARSType(Operator):
    """SelectObjectsByType"""
    bl_idname = "object.mt_select_objects_by_marstype"
    bl_label = "Select objects in the scene by MARStype"
    bl_options = {'REGISTER', 'UNDO'}

    seltype = EnumProperty (
            items = mtdefs.marstypes,
            name = "MARStype",
            default = "link",
            description = "MARS object type")

    def execute(self, context):
        objlist = []
        for obj in bpy.data.objects:
            if obj.MARStype == self.seltype:
                objlist.append(obj)
        mtutility.selectObjects(objlist, True)
        return {'FINISHED'}

class SelectObjectsByName(Operator):
    """SelectObjectsByName"""
    bl_idname = "object.mt_select_objects_by_name"
    bl_label = "Select objects in the scene by their name"
    bl_options = {'REGISTER', 'UNDO'}

    namefragment = StringProperty (
            name = "name contains",
            default = '',
            description = "part of a MARS object name")

    def execute(self, context):
        objlist = []
        for obj in bpy.data.objects:
            if self.namefragment in obj.name:
                objlist.append(obj)
        mtutility.selectObjects(objlist, True)
        return {'FINISHED'}


class SelectRootOperator(Operator):
    """SelectRootOperator"""
    bl_idname = "object.mt_select_root"
    bl_label = "Select root object(s) of currently selected object(s)"

    def execute(self, context):
        roots = set()
        for obj in bpy.context.selected_objects:
            roots.add(mtutility.getRoot(obj))
        mtutility.selectObjects(list(roots), True)
        bpy.context.scene.objects.active = list(roots)[0]
        return {'FINISHED'}

class SelectModelOperator(Operator):
    """SelectModelOperator"""
    bl_idname = "object.mt_select_model"
    bl_label = "Select all objects of model(s) containing the currently selected object(s)"

    modelname = StringProperty(
        name = "modelname",
        default = "",
        description = "name of the model to be selected")

    def execute(self, context):
        selection = []
        if self.modelname:
            print("MARStools: Selecting model", self.modelname)
            roots = mtutility.getRoots()
            for root in roots:
                if root["modelname"] == self.modelname:
                    selection = mtutility.getChildren(root)
        else:
            print("MARStools: No model name provided, deriving from selection...")
            roots = set()
            for obj in bpy.context.selected_objects:
                print("Selecting", mtutility.getRoot(obj).name)
                roots.add(mtutility.getRoot(obj))
            for root in list(roots):
                selection.extend(mtutility.getChildren(root))
        mtutility.selectObjects(list(selection), True)
        return {'FINISHED'}


class CheckModelOperator(Operator):
    """CheckModelOperator"""
    bl_idname = "object.mt_check_model"
    bl_label = "Check if the robot model is valid."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        notifications, faulty_objects = checkModel(bpy.context.selected_objects)
        bpy.ops.error.message('INVOKE_DEFAULT', type="Errors", message=notifications)
        #Deselect all objects and select those with errors
        #bpy.ops.object.select_all() # alternatively:
        for obj in bpy.data.objects: obj.selected = False
        for obj in faulty_objects:
            obj.selected = True
        return {'FINISHED'}


def checkModel(objlist, correct = False):
    notifications = ""
    faulty_objects = []
    for obj in objlist:
        if obj.MARStype == "link":
            if not ("mass" in obj) or ('mass' in obj and float(obj['mass'] == 0)):
                notifications += "Error, object '" + obj.name + "' has no attribute 'mass' or zero mass.\n"
                faulty_objects.append(obj)
                if correct:
                    obj['mass'] = 0.001
    return notifications, faulty_objects


class UpdateMarsModelsOperator(Operator):
    """UpdateMarsModelsOperator"""
    bl_idname = "object.mt_update_models"
    bl_label = "Update MARS properties for all objects"
    bl_options = {'REGISTER', 'UNDO'}

    print("MARStools: Updating MARS properties for selected objects...")

    def execute(self, context):
        mtmaterials.createMARSMaterials()
        mtcreateprops.main(mtutility.getRoots())
        return {'FINISHED'}


class BatchEditPropertyOperator(Operator):
    """Batch-Edit Property Operator"""
    bl_idname = "object.mt_batch_property"
    bl_label = "Edit custom property of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    property_name = StringProperty(
        name = "property_name",
        default = "",
        description = "custom property name")

    property_value = StringProperty(
        name = "property_value",
        default = "",
        description = "custom property value")

    def execute(self, context):
        value = mtutility.parse_number(self.property_value)
        for obj in bpy.context.selected_objects:
            obj[self.property_name] = value
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'

class SetGeometryType(Operator):
    """Set Geometry Type Operator"""
    bl_idname = "object.mt_set_geometry_type"
    bl_label = "Edit geometry type of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    geomType = EnumProperty (
            items = mtdefs.geometrytypes,
            name = "geometryType",
            default = "box",
            description = "MARS geometry type")

    def execute(self, context):
        for obj in bpy.context.selected_objects:
            if obj.MARStype == 'collision' or obj.MARStype == 'visual':
                obj['geometryType'] = self.geomType
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'


class SetMARSType(Operator):
    """Set MARStype Operator"""
    bl_idname = "object.mt_set_marstype"
    bl_label = "Edit MARStype of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    marstype = EnumProperty (
            items = mtdefs.marstypes,
            name = "MARStype",
            default = "undefined",
            description = "MARStype")

    def execute(self, context):
        for obj in bpy.context.selected_objects:
            obj.MARStype = self.marstype
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'

class PartialRename(Operator):
    """Partial Rename Operator"""
    bl_idname = "object.mt_partial_rename"
    bl_label = "Replace part of the name of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    find = StringProperty(
        name = "find",
        default = "",
        description = "find string")

    replace = StringProperty(
        name = "replace",
        default = "",
        description = "replace with")

    def execute(self, context):
        for obj in bpy.context.selected_objects:
            obj.name = obj.name.replace(self.find, self.replace)
        return {'FINISHED'}
def replaceNameElement(prop, old, new):

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'


class SmoothenSurfaceOperator(Operator):
    """SmoothenSurfaceOperator"""
    bl_idname = "object.mt_smoothen_surface"
    bl_label = "Smoothen Selected Objects"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        wm = bpy.context.window_manager
        total = float(len(bpy.context.selected_objects))
        wm.progress_begin(0, total)
        i = 1
        for obj in bpy.context.selected_objects:
            if obj.type != 'MESH':
                continue
            bpy.context.scene.objects.active = obj
            bpy.ops.object.mode_set(mode = 'EDIT')
            bpy.ops.mesh.select_all()
            bpy.ops.mesh.normals_make_consistent()
            bpy.ops.object.mode_set(mode = 'OBJECT')
            bpy.ops.object.shade_smooth()
            bpy.ops.object.modifier_add(type='EDGE_SPLIT')
            wm.progress_update(i)
            i += 1
        wm.progress_end()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'


# The following function is adapted from Bret Battey's adaptation
# (http://bathatmedia.blogspot.de/2012/08/duplicating-objects-in-blender-26.html) of
# Nick Keeline "Cloud Generator" addNewObject
# from object_cloud_gen.py (an addon that comes with the Blender 2.6 package)
#
def duplicateObject(scene, name, copyobj, material, layers):
    """Returns a copy of the provided object"""

    # Create new mesh
    mesh = bpy.data.meshes.new(name)

    # Create new object associated with the mesh
    ob_new = bpy.data.objects.new(name, mesh)

    # Copy data block from the old object into the new object
    ob_new.data = copyobj.data.copy()
    ob_new.scale = copyobj.scale
    ob_new.location = copyobj.location
    ob_new.data.materials.append(bpy.data.materials[material])

    # Link new object to the given scene and select it
    scene.objects.link(ob_new)
    ob_new.layers = layers
    #ob_new.select = True

    return ob_new

class AddGravityVector(Operator):
    """Add Gravity Operator"""
    bl_idname = "object.mt_add_gravity"
    bl_label = "Add a vector representing gravity in the scene"
    bl_options = {'REGISTER', 'UNDO'}

    property_name = FloatVectorProperty(
        name = "gravity_vector",
        default = (0, 0, -9.81),
        description = "gravity vector")

    def execute(self, context):
        bpy.ops.object.empty_add(type='SINGLE_ARROW')
        bpy.context.active_object.name = "gravity"
        bpy.ops.transform.rotate(value=(math.pi), axis=(1.0, 0.0, 0.0))
        return {'FINISHED'}

# the following code is used to directly add buttons to current operator menu
# - we don't need that if we create a custom toolbar with pre-defined buttons
# def add_object_button(self, context):
#     self.layout.operator(
#         BatchEditPropertyOperator.bl_idname,
#         text=BatchEditPropertyOperator.__doc__,
#         icon='PLUGIN')
