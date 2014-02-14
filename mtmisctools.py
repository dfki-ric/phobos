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
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty
import marstools.mtcreateprops as mtcreateprops
import marstools.mtexport as mtexport
import marstools.mtmaterials as mtmaterials
import marstools.mtutility as mtutility


def register():
    print("Registering mtmisctools...")
    bpy.types.World.path = StringProperty(name = "path")
    print("    Added 'pathath' to Object properties.")
    bpy.types.World.filename = StringProperty(name = "filename")
    print("    Added 'filename' to Object properties.")
    bpy.types.World.exportBobj = BoolProperty(name = "exportBobj")
    print("    Added 'exportBobj' to Object properties.")
    bpy.types.World.exportMesh = BoolProperty(name = "exportMesh")
    print("    Added 'exportMesh' to Object properties.")
    #bpy.utils.register_class(ExportModelOperator)
    #bpy.utils.register_class(ImportModelOperator)
    #bpy.utils.register_class(CreateMARSPropsOperator)
    #bpy.utils.register_class(BatchEditPropertyOperator)
    #bpy.utils.register_class(SmoothenSurfaceOperator)
    #bpy.utils.register_class(BatchSmoothenSurfaceOperator)
    #bpy.types.VIEW3D_MT_object.append(add_object_button)


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
#         mtimport.main()

class ShowMotorTypesOperator(Operator):
    """ShowMotorTypesOperator"""
    bl_idname = "object.mt_show_motor_types"
    bl_label = "Use colors to mark different motor types in the scene."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
#        types = {}
#        n_indicators = 0
#        print("Preparing to indicate motor types...")
#         for obj in bpy.context.selected_objects:
#             if "spec_motor" in obj:
#                 if not (obj["spec_motor"] in types):
#                     n_indicators += 1
#                     types[obj["spec_motor"]] = "indicator" + str(n_indicators)
#                 duplicateObject(bpy.context.scene, "ind_"+obj.name, obj, types[obj["spec_motor"]], mtutility.defLayers([10]))
        types = {}
        n_indicators = 0
        for obj in bpy.context.selected_objects:
            if "spec_motor" in obj:
                if not (obj["spec_motor"] in types):
                    n_indicators += 1
                    types[obj["spec_motor"]] = "indicator" + str(n_indicators)
                if not types[obj["spec_motor"]] in obj.data.materials:
                    obj.data.materials.append(bpy.data.materials[types[obj["spec_motor"]]])
                    obj.data.materials.pop(0, update_data=True)
        bpy.data.scenes[0].update()
        #TODO: REALLY refresh the scene
        return{'FINISHED'}

class UnshowMotorTypesOperator(Operator):
    """UnshowMotorTypesOperator"""
    bl_idname = "object.mt_unshow_motor_types"
    bl_label = "Revert the color changes made by the ShowMotorTypes operator."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        print("Preparing to indicate motor types...")
        for obj in bpy.context.selected_objects:
            obj.data.materials.append(bpy.data.materials["Joint Discs"])
            obj.data.materials.pop(0, update_data=True)
        bpy.data.scenes[0].update()
        return{'FINISHED'}

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
            if obj.MARStype == "body":
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

    #Todo functions:
    # - check for duplicate names
    # - check whether there are two roots
    # - check whether all indices registered with sensors exist
    # - check for doubles in registered indices of sensors
    # - check if all joints have a sensible node2

    def execute(self, context):
        notifications = ""
        faulty_objects = []
        for obj in bpy.context.selected_objects:
            if obj.MARStype == "body":
                if not ("mass" in obj):
                    print('CheckModel: Error, object "' + obj.name + '" has no attribute "mass".')
                    notifications += 'CheckModel: Error, object "' + obj.name + '" has no attribute "mass".'
                    faulty_objects.append(obj)
                else:
                    if obj["mass"] == 0 or obj["mass"] == 0.0 or obj["mass"] == "0":
                        print('CheckModel: Error, object "' + obj.name + '" has no mass.')
                        notifications += 'CheckModel: Error, object "' + obj.name + '" has no mass.\n'
                        faulty_objects.append(obj)
        bpy.ops.error.message('INVOKE_DEFAULT', type="Errors", message=notifications)

        #Deselect all objects and select those with errors
        bpy.ops.object.select_all() # alternatively: for obj in bpy.data.objects: obj.selected = False
        for obj in faulty_objects:
            obj.selected = True
        return {'FINISHED'}

class ExportModelOperator(Operator):
    """ExportModelOperator"""
    bl_idname = "object.mt_export_robot"
    bl_label = "Export the selected model(s)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        #TODO: add selection of all layers bpy.ops.object.select_all()
        mtexport.main()
        return {'FINISHED'}


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
        for obj in bpy.context.selected_objects:
            obj[self.property_name] = self.property_value
        return {'FINISHED'}

    #TODO: Do we need the following?
    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'

class SmoothenSurfaceOperator(Operator):
    """SmoothenSurfaceOperator"""
    bl_idname = "object.mt_smoothen_surface"
    bl_label = "Smoothen Active Object"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        bpy.ops.object.mode_set(mode = 'EDIT')
        bpy.ops.mesh.select_all()
        bpy.ops.mesh.normals_make_consistent()
        bpy.ops.object.mode_set(mode = 'OBJECT')
        bpy.ops.object.modifier_add(type='EDGE_SPLIT')
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'

class BatchSmoothenSurfaceOperator(Operator):
    """BatchSmoothenSurfaceOperator"""
    bl_idname = "object.mt_batch_smoothen_surface"
    bl_label = "Smoothen Selected Objects"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        for obj in bpy.context.selected_objects:
            bpy.context.scene.objects.active = obj
            bpy.ops.object.mode_set(mode = 'EDIT')
            bpy.ops.mesh.select_all()
            bpy.ops.mesh.normals_make_consistent()
            bpy.ops.object.mode_set(mode = 'OBJECT')
            bpy.ops.object.modifier_add(type='EDGE_SPLIT')
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


# the following code is used to directly add buttons to current operator menu
# - we don't need that if we create a custom toolbar with pre-defined buttons
# def add_object_button(self, context):
#     self.layout.operator(
#         BatchEditPropertyOperator.bl_idname,
#         text=BatchEditPropertyOperator.__doc__,
#         icon='PLUGIN')
