#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.operators.io
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators import/export

.. moduleauthor:: Kai von Szadowski, Ole Schwiegert

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
"""

from phobos.logging import startLog, endLog, log
import phobos.defs as defs
import phobos.exporter as exporter
from phobos.export import entity_types
import phobos.importer as importer
import phobos.links as links
import bpy
import bgl
import yaml
import os
import glob
import time
import phobos.utils.selection as sUtils
import phobos.robotdictionary as robotdictionary
from bpy.types import Operator
from bpy.props import EnumProperty, StringProperty, FloatProperty, IntProperty


def generateLibEntries(param1, param2): #FIXME: parameter?
    with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
        return [("None",)*3] + [(entry,)*3 for entry in yaml.load(f.read())]


def loadBackedModels():
    modelsfolder = os.path.abspath(bpy.context.user_preferences.addons["phobos"].preferences.modelfolder)
    robots_found = []

    for root, dirs, files in os.walk(modelsfolder):
        for file in files:
            if os.path.splitext(file)[-1] == '.smurf':
                robots_found.append(os.path.join(root, file))
    robots_dict = dict()
    for robot in robots_found:
        with open(robot, 'r') as robot_smurf:
            robot_yml   = yaml.load(robot_smurf)
            model_name  = robot_yml["modelname"]
            robot_files = robot_yml["files"]
            for file in robot_files:
                if file.split('_')[-1] == "poses.yml":
                    if model_name not in robots_dict:
                        robots_dict[model_name] = []
                    with open(os.path.join(os.path.dirname(robot),file)) as poses:
                        poses_yml = yaml.load(poses)
                        for pose in poses_yml['poses']:
                            robots_dict[model_name].append({"posename": pose['name']})
                            robots_dict[model_name][-1]["robotpath"] = os.path.dirname(robot)

    bpy.context.scene.bakeModels.clear()
    model_count = 0
    for model_name in robots_dict.keys():
        item = bpy.context.scene.bakeModels.add()
        item.name  = "model_preview_"+str(model_count)
        item.label = model_name
        item.type  = "robot_name"
        if item.hide:
            item.icon = "RIGHTARROW"
        else:
            item.icon  = "DOWNARROW_HLT"
        if not item.name in bpy.data.textures.keys():
            bpy.data.textures.new(item.name,type="NONE")
        current_parent = item.name
        pose_count = 0
        for pose in robots_dict[model_name]:
            item = bpy.context.scene.bakeModels.add()
            item.parent     = current_parent
            item.name       = "model_preview_"+str(model_count)+'_'+str(pose_count)
            item.label      = pose["posename"]
            item.path       = pose["robotpath"]
            item.type       = "robot_model"
            item.robot_name = model_name
            search_path = pose["robotpath"]
            if os.path.split(search_path)[-1] == "smurf":
                search_path = os.path.dirname(search_path)
            for file in (glob.glob(search_path + "/**/" + model_name + "_" + pose['posename'] + ".*")+glob.glob(search_path + "/" + model_name + "_" + pose['posename'] + ".*")):
                if (os.path.splitext(file)[-1].lower() == ".stl") or (os.path.splitext(file)[-1].lower() == ".obj"):
                    item.model_file = os.path.join(search_path, file)
                if (os.path.splitext(file)[-1].lower() == ".png"):
                    item.preview = os.path.join(search_path, file)

            item.icon = "X_VEC"
            print(item.preview)
            if item.preview != '':
                if os.path.split(item.preview)[-1] in bpy.data.images.keys():
                    bpy.data.images[os.path.split(item.preview)[-1]].reload()
                im = bpy.data.images.load(item.preview)

                im.gl_load(0, bgl.GL_LINEAR, bgl.GL_LINEAR)
                tex = None
                if not item.name in bpy.data.textures.keys():
                    tex = bpy.data.textures.new(item.name,type='IMAGE')
                else:
                    tex = bpy.data.textures[item.name]
                tex.image = im
            else:
                bpy.data.textures.new(item.name, type="NONE")
            pose_count += 1
        model_count += 1

    # Debug
#    item = bpy.context.scene.bakeModels.add()
#    item.name = "model_preview_1"
#    item.label = "Compi"
#    item.type = "robot_name"
#    if item.hide:
#        item.icon = "RIGHTARROW"
#    else:
#        item.icon = "DOWNARROW_HLT"
#    if not item.name in bpy.data.textures.keys():
#        tex = bpy.data.textures.new(item.name, type='NONE')


class LoadBackedModelsOperator(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "scene.load_backed_models_operator"
    bl_label = "Load backed models"

    def execute(self, context):
        loadBackedModels()
        return {'FINISHED'}


class ImportLibRobot(Operator):
    """Import a baked robot into the robot library"""
    bl_idname = "object.phobos_import_lib_robot"
    bl_label = "Import Baked Robot"
    bl_options = {'REGISTER', 'UNDO'}

    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        startLog(self)
        libPath = os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml")
        path, file = os.path.split(self.filepath)
        if file.endswith(".bake"):
            with open(self.filepath, "r") as f:
                info = yaml.load(f.read())
            if not os.path.isfile(libPath):
                open(libPath, "a").close()
            with open(libPath, "r+") as f:
                robot_lib = yaml.load(f.read())
                robot_lib = robot_lib if robot_lib is not None else {}
                robot_lib[info["name"]] = path
                f.seek(0)
                f.write(yaml.dump(robot_lib))
                f.truncate()
        else:
            log("This is no robot bake!", "ERROR")
        endLog()
        return {"FINISHED"}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}


class ImportSelectedLibRobot(Operator):
    """Import a baked robot into the robot library"""
    bl_idname = "scene.phobos_import_selected_lib_robot"
    bl_label = "Import Baked Robot"
    #bl_options = {'REGISTER', 'UNDO'}
    obj_name = StringProperty(
        name="New Smurf Entity Name",
        default="New Robot",
        description="Name of new Smurf Entity"
    )

    def invoke(self, context, event):
        wm = context.window_manager
        selected_robot = bpy.context.scene.bakeModels[bpy.data.textures[bpy.context.scene.active_bakeModel].name]
        if selected_robot.model_file != '':
            return wm.invoke_props_dialog(self,width=300,height=100)
        else:
            return {"CANCELLED"}

    def draw(self, context):
        row = self.layout
        row.prop(self, "obj_name")

    def execute(self, context):
        startLog(self)
        log("Import robot bake", "INFO")
        selected_robot = bpy.context.scene.bakeModels[bpy.data.textures[bpy.context.scene.active_bakeModel].name]
        if (selected_robot.type != "robot_name"):
            if os.path.splitext(selected_robot.model_file)[-1] == ".obj":
                bpy.ops.import_scene.obj(filepath=selected_robot.model_file,
                                         axis_forward='-Z',
                                         axis_up='Y',
                                         filter_glob="*.obj;*.mtl",
                                         use_edges=True,
                                         use_smooth_groups=True,
                                         use_split_objects=True,
                                         use_split_groups=True,
                                         use_groups_as_vgroups=False,
                                         use_image_search=True,
                                         split_mode='ON',
                                         global_clamp_size=0)
            elif os.path.splitext(selected_robot.model_file)[-1] == ".stl":
                bpy.ops.import_mesh.stl(filepath=selected_robot.model_file,
                                        axis_forward='Y',
                                        axis_up='Z',
                                        filter_glob="*.stl",
                                        files=[],
                                        directory="",
                                        global_scale=1,
                                        use_scene_unit=True,
                                        use_facet_normal=False)
            robot_obj = bpy.context.selected_objects[0]
            bpy.context.scene.objects.active = robot_obj
            robot_obj.name = self.obj_name
            robot_obj["modelname"] = selected_robot.robot_name
            robot_obj["entity/name"] = self.obj_name
            robot_obj["entity/type"] = "smurf"
            robot_obj["entity/pose"] = selected_robot.label
            robot_obj["entity/isReference"] = True
            robot_obj.phobostype = 'entity'

        endLog()
        return {'FINISHED'}


class CreateRobotInstance(Operator):
    """Create a new instance of the selected robot lib entry"""
    bl_idname = "object.phobos_create_robot_instance"
    bl_label = "Create Robot Instance"
    bl_options = {'REGISTER', 'UNDO'}

    bakeObj = EnumProperty(
        name="Robot Lib Entries",
        items=generateLibEntries,
        description="The robot lib entries")

    robName = StringProperty(
        name="Instance Name",
        default="instance",
        description="The instance's name"
    )

    def execute(self, context):
        if self.bakeObj == "None":
            return {"FINISHED"}
        with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
            robot_lib = yaml.load(f.read())
        root = links.createLink(1.0, name=self.robName + "::" + self.bakeObj)
        root["modelname"] = self.bakeObj
        root["entity/name"] = self.robName
        root["isInstance"] = True
        bpy.ops.import_mesh.stl(filepath=os.path.join(robot_lib[self.bakeObj], "bake.stl"))
        bpy.ops.view3d.snap_selected_to_cursor(use_offset=False)
        obj = context.active_object
        obj.name = self.robName + "::visual"
        obj.phobostype = "visual"
        sUtils.selectObjects([root, obj], clear=True, active=0)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
        return {"FINISHED"}

    @classmethod
    def poll(self, context):
        return os.path.isfile(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"))


class ExportSceneOperator(Operator):
    """Export the selected model(s) in a scene"""
    bl_idname = "object.phobos_export_scene"
    bl_label = "Export Scene"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        exporter.exportSMURFsScene()
        endLog()
        return {'FINISHED'}


class ExportModelOperator(Operator):
    """Export the selected model(s)"""
    bl_idname = "object.phobos_export_robot"
    bl_label = "Export Model"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        root = sUtils.getRoot(context.selected_objects[0])
        if root.phobostype != 'link':
            log("Selection includes objects not parented to any model root, please adapt selection.", "ERROR", "ExportModelOperator")
        else:
            model, objectlist = robotdictionary.buildModelDictionary(root)
            if 'smurf' in entity_types:
                entity_types['smurf'].export(model, objectlist)
            else:
                log("No export available for a robot entity!", "ERROR", __name__+".ExportModelOperator")
            endLog()
        return {'FINISHED'}


class ExportBakeOperator(Operator):
    """Bake the selected model"""
    bl_idname = "object.phobos_export_bake"
    bl_label = "Export Bake"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        roots = sUtils.getRootsOfObjs(context.selected_objects)
        for root in roots:
            sUtils.selectChildren(root)
            model, objectlist = robotdictionary.buildModelDictionary(root)
            exporter.bakeModel(objectlist, model["modelname"],'pose1')
        endLog()
        return {'FINISHED'}

'''
class ExportCurrentPoseOperator(Operator):
    """Bake the selected model"""
    bl_idname = "object.phobos_export_all_poses"
    bl_label = "Export All Poses"
    #bl_options = {'REGISTER', 'UNDO'}
    decimate_type = EnumProperty(name="Decimate Type",
                                 items=[('COLLAPSE','Collapse','COLLAPSE'),('UNSUBDIV','Un-Subdivide','UNSUBDIV'),('DISSOLVE','Planar','DISSOLVE')])
    decimate_ratio = FloatProperty(name="Ratio",default=0.15)
    decimate_iteration = IntProperty(name="Iterations",default=1)
    decimate_angle_limit = FloatProperty(name="Angle Limit",default=5)

    def invoke(self, context, event):
        wm = context.window_manager
        bpy.context.scene.render.resolution_x=256
        bpy.context.scene.render.resolution_y=256
        bpy.context.scene.render.resolution_percentage=100
        return wm.invoke_props_dialog(self,width=300,height=100)

    def draw(self, context):
        row = self.layout
        row.label(text="Model Export Properties:")
        row.prop(self, "decimate_type")
        if self.decimate_type == 'COLLAPSE':
            row.prop(self, "decimate_ratio")
        elif self.decimate_type == 'UNSUBDIV':
            row.prop(self, "decimate_iteration")
        elif self.decimate_type == 'DISSOLVE':
            row.prop(self, "decimate_angle_limit")
        rd = bpy.context.scene.render
        image_settings = rd.image_settings
        row.label(text="Preview Properties:")
        row.label(text="Resolution:")
        row.prop(rd, "resolution_x", text="X")
        row.prop(rd, "resolution_y", text="Y")
        row.prop(rd, "resolution_percentage", text="")
        #row.label(text="File Format:")
        #row.template_image_settings(image_settings, color_management=False)

    def check(self,context):
        return True

    def execute(self, context):
        # TODO
        startLog(self)
        root = sUtils.getRoot(context.selected_objects[0])

        sUtils.selectChildren(root)
        model, objectlist = robotdictionary.buildModelDictionary(root)
        poses = robotdictionary.getPoses(model["modelname"])
        i = 0
        for pose in poses:
            bpy.context.window_manager.progress_begin(0, len(poses))
            bpy.context.window_manager.progress_update(i)
            i += 1
            root.select = True
            sUtils.selectChildren(root)
            bpy.context.scene.objects.active = root
            robotdictionary.loadPose(model["modelname"], pose)
            model, objectlist = robotdictionary.buildModelDictionary(root)
            parameter = self.decimate_ratio
            if self.decimate_type == 'UNSUBDIV':
                parameter = self.decimate_iteration
            elif self.decimate_type == 'DISSOLVE':
                parameter = self.decimate_angle_limit
            exporter.bakeModel(objectlist, model["modelname"],pose,decimate_type=self.decimate_type, decimate_parameter=parameter)
        bpy.context.window_manager.progress_end()
        root.select = True
        sUtils.selectChildren(root)
        bpy.context.scene.objects.active = root
        endLog()
        return {'FINISHED'}
'''

class ExportAllPosesOperator(Operator):
    """Bake the selected model"""
    bl_idname = "object.phobos_export_all_poses"
    bl_label = "Export All Poses"
    #bl_options = {'REGISTER', 'UNDO'}
    decimate_type = EnumProperty(name="Decimate Type",
                                 items=[('COLLAPSE','Collapse','COLLAPSE'),('UNSUBDIV','Un-Subdivide','UNSUBDIV'),('DISSOLVE','Planar','DISSOLVE')])
    decimate_ratio = FloatProperty(name="Ratio",default=0.15)
    decimate_iteration = IntProperty(name="Iterations",default=1)
    decimate_angle_limit = FloatProperty(name="Angle Limit",default=5)

    def invoke(self, context, event):
        wm = context.window_manager
        bpy.context.scene.render.resolution_x=256
        bpy.context.scene.render.resolution_y=256
        bpy.context.scene.render.resolution_percentage=100
        return wm.invoke_props_dialog(self,width=300,height=100)

    def draw(self, context):
        row = self.layout
        row.label(text="Model Export Properties:")
        row.prop(self, "decimate_type")
        if self.decimate_type == 'COLLAPSE':
            row.prop(self, "decimate_ratio")
        elif self.decimate_type == 'UNSUBDIV':
            row.prop(self, "decimate_iteration")
        elif self.decimate_type == 'DISSOLVE':
            row.prop(self, "decimate_angle_limit")
        rd = bpy.context.scene.render
        image_settings = rd.image_settings
        row.label(text="Preview Properties:")
        row.label(text="Resolution:")
        row.prop(rd, "resolution_x", text="X")
        row.prop(rd, "resolution_y", text="Y")
        row.prop(rd, "resolution_percentage", text="")
        #row.label(text="File Format:")
        #row.template_image_settings(image_settings, color_management=False)

    def check(self,context):
        return True

    def execute(self, context):
        startLog(self)
        root = sUtils.getRoot(context.selected_objects[0])

        sUtils.selectChildren(root)
        model, objectlist = robotdictionary.buildModelDictionary(root)
        poses = robotdictionary.getPoses(model["modelname"])
        i = 0
        for pose in poses:
            bpy.context.window_manager.progress_begin(0, len(poses))
            bpy.context.window_manager.progress_update(i)
            i += 1
            root.select = True
            sUtils.selectChildren(root)
            bpy.context.scene.objects.active = root
            robotdictionary.loadPose(model["modelname"], pose)
            model, objectlist = robotdictionary.buildModelDictionary(root)
            parameter = self.decimate_ratio
            if self.decimate_type == 'UNSUBDIV':
                parameter = self.decimate_iteration
            elif self.decimate_type == 'DISSOLVE':
                parameter = self.decimate_angle_limit
            exporter.bakeModel(objectlist, model["modelname"],pose,decimate_type=self.decimate_type, decimate_parameter=parameter)
        bpy.context.window_manager.progress_end()
        root.select = True
        sUtils.selectChildren(root)
        bpy.context.scene.objects.active = root
        endLog()
        return {'FINISHED'}


class RobotModelImporter(bpy.types.Operator):
    """Import robot model file from various formats"""
    bl_idname = "obj.import_robot_model"
    bl_label = ""
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'

    # creating property for storing the path to the .scn file
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    @classmethod
    def poll(cls, context):
        return context is not None

    def execute(self, context):
        modeltype = self.filepath.split('.')[-1]
        if modeltype == 'scene':
            imp = importer.MARSModelParser(self.filepath)
        elif modeltype == 'urdf':
            imp = importer.URDFModelParser(self.filepath)
        elif modeltype == 'smurf' or modeltype == 'yml' or modeltype == 'yaml':
            imp = importer.SMURFModelParser(self.filepath)
        elif modeltype == 'scn':
            imp = importer.MARSModelParser(self.filepath, zipped=True)
        else:
            print("Unknown model format, aborting import...")
        importer.cleanUpScene()
        imp.parseModel()
        imp.createBlenderModel()
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}
