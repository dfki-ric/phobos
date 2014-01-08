'''
Created on 6 Jan 2014

@author: kavonszadkowski
'''

import bpy
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty
import marstools.mtcreateprops as mtcreateprops
import marstools.mtexport as mtexport
import marstools.mtmaterials as mtmaterials


def register():
    print("Registering mtmisctools...")
    bpy.types.World.exportPath = StringProperty(name = "exportPath")
    print("    Added 'exportPath' to Object properties.")
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



class ExportModelOperator(Operator):#
    """ExportModelOperator"""
    bl_idname = "object.mt_export_robot"
    bl_label = "Initialise MARS properties for all objects"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        #add selction of all layers bpy.ops.object.select_all()
        mtexport.main()
        return {'FINISHED'}


class CreateMARSPropsOperator(Operator):
    """CreateMARSPropsOperator"""
    bl_idname = "object.mt_create_props"
    bl_label = "Initialise MARS properties for all objects"
    bl_options = {'REGISTER', 'UNDO'}

    print("Creating MARS properties for selected objects...")

    def execute(self, context):
        mtcreateprops.main()
        return {'FINISHED'}

class BatchEditPropertyOperator(Operator):
    """Batch-Edit Property Operator"""
    bl_idname = "object.mt_batch_property"
    bl_label = "Edit custom property"
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


# the following code is used to directly add buttons to current operator menu
# - we don't need that if we create a custom toolbar with pre-defined buttons
# def add_object_button(self, context):
#     self.layout.operator(
#         BatchEditPropertyOperator.bl_idname,
#         text=BatchEditPropertyOperator.__doc__,
#         icon='PLUGIN')



# if script is run directly, register contained classes
if __name__ == "__main__":
    register()