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
from bpy.props import StringProperty, BoolProperty, FloatVectorProperty, EnumProperty, FloatProperty
import marstools.mtupdate as mtupdate
import marstools.mtmaterials as mtmaterials
import marstools.mtutility as mtutility
import marstools.mtdefs as mtdefs
import marstools.mtinertia as mtinertia
import marstools.mtrobotdictionary as mtrobotdictionary
from datetime import datetime as dt


def register():
    print("Registering mtmisctools...")

def unregister():
    print("Unregistering mtmisctools...")

class CalculateMassOperator(Operator):
    """CalculateMassOperator"""
    bl_idname = "object.mt_calculate_mass"
    bl_label = "Display mass of the selected objects in a pop-up window."

    def execute(self, context):
        mass = mtutility.calculateMass(bpy.context.selected_objects)
        bpy.ops.error.message('INVOKE_DEFAULT', type="mass", message=str(mass))
        return {'FINISHED'}

class SetMassOperator(Operator):
    """SetMassOperator"""
    bl_idname = "object.mt_set_mass"
    bl_label = "Sets the mass of the selected object(s)."
    bl_options = {'REGISTER', 'UNDO'}

    mass = FloatProperty(
        name = 'mass',
        default = 0.001,
        description = 'mass in kg')

    def execute(self, context):
        for obj in bpy.context.selected_objects:
            if obj.MARStype in ['visual', 'collision', 'inertial']:
                obj['mass'] = self.mass
                t = dt.now()
                obj['masschanged'] = t.isoformat()
        return {'FINISHED'}

class SyncMassesOperator(Operator):
    """SyncMassesOperator"""
    bl_idname = "object.mt_sync_masses"
    bl_label = "Synchronize masses among the selected object(s)."
    bl_options = {'REGISTER', 'UNDO'}

    synctype = EnumProperty (
            items = (("vtc", "visual to collision", "visual to collision"),
                     ("ctv", "collision to visual", "collision to visual"),
                     ("lto", "latest to oldest", "latest to oldest")),
            name = "synctype",
            default = "vtc",
            description = "MARS object type")

    writeinertial = BoolProperty(
                name = 'write_inertial',
                default = True,
                description = 'write mass to inertial'
                )

    def execute(self, context):
        sourcelist = []
        targetlist = []
        processed = []
        links = [obj.name for obj in bpy.context.selected_objects if obj.MARStype == 'link']
        t = dt.now()
        objdict = {obj.name: obj for obj in bpy.context.selected_objects}
        for obj in objdict.keys():
            if objdict[obj].MARStype in ['visual', 'collision']:
                basename = obj.replace(objdict[obj].MARStype+'_', '')
                if (objdict[obj].parent.name in links
                    and basename not in processed
                    and 'visual_' + basename in objdict.keys()
                    and 'collision_' + basename in objdict.keys()): #if both partners are present
                    processed.append(basename)
        for basename in processed:
            if self.synctype == "vtc":
                sourcelist.append('visual_' + basename)
                targetlist.append('collision_' + basename)
            elif self.synctype == "ctv":
                targetlist.append('visual_' + basename)
                sourcelist.append('collision_' + basename)
            else: #latest to oldest
                tv = mtutility.datetimeFromIso(objdict['visual_'+basename]['masschanged'])
                tc = mtutility.datetimeFromIso(objdict['collision_'+basename]['masschanged'])
                if tc < tv: #if collision information is older than visual information
                    sourcelist.append('visual_' + basename)
                    targetlist.append('collision_' + basename)
                else:
                    targetlist.append('visual_' + basename)
                    sourcelist.append('collision_' + basename)
        for i in range(len(sourcelist)):
            objdict[targetlist[i]]['mass'] = objdict[sourcelist[i]]['mass']
            objdict[targetlist[i]]['masschanged'] = objdict[sourcelist[i]]['masschanged']
        for linkname in links:
            masssum = 0.0
            collision_children = mtinertia.getInertiaRelevantObjects(links[linkname])
            for coll in collision_children:
                masssum += coll['mass']
            if self.writeinertial:
                try:
                    inertial = bpy.data.objects['inertial_' + linkname]
                    if not 'mass' in inertial or inertial['mass'] != masssum:
                        inertial['mass'] = masssum
                        inertial['masschanged'] = t.isoformat()
                except KeyError:
                    print("###Warning: no inertial object for link", linkname)
            else:
                links[linkname]['mass'] = masssum
                links[linkname]['masschanged'] = t.isoformat()

        return {'FINISHED'}

class ShowDistanceOperator(Operator):
    """ShowDistanceOperator"""
    bl_idname = "object.mt_show_distance"
    bl_label = "Shows distance between two selected objects in world coordinates."
    bl_options = {'REGISTER', 'UNDO'}

    distance = FloatProperty(
        name = "distance",
        default = 0.0,
        description = "distance between objects")

    def execute(self, context):
        self.distance = mtutility.distance(bpy.context.selected_objects)
        print(self.distance)
        return {'FINISHED'}

class SetXRayOperator(Operator):
    """SetXrayOperator"""
    bl_idname = "object.mt_set_xray"
    bl_label = "Shows the selected/chosen objects via X-Ray."
    bl_options = {'REGISTER', 'UNDO'}

    objects = EnumProperty(
        name = "objects",
        default = 'selected',
        items = (('all',)*3, ('selected',)*3) + mtdefs.marstypes,
        description = "show objects via x-ray")

    show = BoolProperty(
        name = "show",
        default = True,
        description = "set to")

    namepart = StringProperty(
        name = "name",
        default = "",
        description = "name contains")

    def execute(self, context):
        if self.objects == 'all':
            objlist = bpy.data.objects
        elif self.objects == 'select':
            objlist = bpy.context.selected_objects
        elif self.objects == 'by name':
            objlist = [obj for obj in bpy.data.objects if obj.name.find(self.namepart) > 0]
        else:
            objlist = [obj for obj in bpy.data.objects if obj.MARStype == self.objects]
        for obj in objlist:
            obj.show_x_ray = self.show
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


class UpdateMarsModelsOperator(Operator):
    """UpdateMarsModelsOperator"""
    bl_idname = "object.mt_update_models"
    bl_label = "Update MARS properties for all objects"
    bl_options = {'REGISTER', 'UNDO'}

    property_fix = BoolProperty(
        name = 'fix',
        default = False,
        description = "try to fix detected errors?")

    print("MARStools: Updating MARS properties for selected objects...")

    def execute(self, context):
        mtmaterials.createMARSMaterials() #TODO: this should move to initialization
        mtupdate.updateModels(mtutility.getRoots(), self.property_fix)
        return {'FINISHED'}


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
        if value == '':
            for obj in bpy.context.selected_objects:
                if self.property_name in obj:
                    del(obj[self.property_name])
        else:
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

class SetInertia(Operator):
    """Set Inertia Operator"""
    bl_idname = "object.mt_set_inertia"
    bl_label = "Set inertia of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    #inertiamatrix = FloatVectorProperty (
    #        name = "inertia",
    #        default = [0, 0, 0, 0, 0, 0, 0, 0, 0],
    #        subtype = 'MATRIX',
    #        size = 9,
    #        description = "set inertia for a link")

    inertiavector = FloatVectorProperty (
            name = "inertiavec",
            default = [0, 0, 0, 0, 0, 0],
            subtype = 'NONE',
            size = 6,
            description = "set inertia for a link"
            )

    def execute(self, context):
        #m = self.inertiamatrix
        #inertialist = []#[m[0], m[1], m[2], m[4], m[5], m[8]]
        #obj['inertia'] = ' '.join(inertialist)
        for obj in context.selected_objects:
            obj['inertia'] = ' '.join([str(i) for i in self.inertiavector])
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and ob.MARStype == 'inertial'

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
        show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269;
        if show_progress:
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
            if show_progress:
                wm.progress_update(i)
                i += 1
        if show_progress:
            wm.progress_end()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'


class CreateInertialOperator(Operator):
    """CreateInertialOperator"""
    bl_idname = "object.create_inertial"
    bl_label = "Adds Bone Constraints to the joint (link)"
    bl_options = {'REGISTER', 'UNDO'}

    writeinertial = BoolProperty(
                name = 'automate_links',
                default = False,
                description = 'automate links'
                )

    def execute(self, context):
        links = []
        viscols = set()
        for obj in bpy.context.selected_objects:
            if obj.MARStype == 'link':
                links.append(obj)
            elif obj.MARStype in ['visual', 'collision']:
                viscols.add(obj)
        for link in links:
            if self.automate_links:
                viscols.add(mtinertia.getInertiaRelevantObjects(link))
            else:
                inertial = mtinertia.createInertial(link)
                inertial['mass'] = link['mass']
                inertial['inertia'] = mtinertia.calculateInertia(
                                      link, mtrobotdictionary.deriveGeometry(link))
        for obj in viscols:
            mtinertia.createInertial(obj)
        return {'FINISHED'}


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
