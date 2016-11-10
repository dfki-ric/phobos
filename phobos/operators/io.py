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

import os
import yaml
import sys
import inspect

import bpy
import bgl
import glob
from bpy.types import Operator
from bpy.props import EnumProperty, StringProperty, FloatProperty, IntProperty

import phobos.defs as defs
from phobos.logging import startLog, endLog, log
import phobos.model.models as models
import phobos.model.links as links
import phobos.utils.selection as sUtils
import phobos.io.entities.urdf as urdf
import phobos.io.scenes.smurfs as smurfs
import phobos.io.entities.smurf as smurf
import phobos.io.scenes.smurfs as smurfs
from phobos.io.entities import entities


def generateLibEntries(param1, param2): #FIXME: parameter?
    with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
        return [("None",)*3] + [(entry,)*3 for entry in yaml.load(f.read())]


def loadModelsAndPoses():
    if bpy.context.user_preferences.addons["phobos"].preferences.modelsfolder:
        modelsfolder = os.path.abspath(bpy.context.user_preferences.addons["phobos"].preferences.modelsfolder)
    else:
        modelsfolder = ''
    modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
    robots_found = []
    print(modelsfolder)
    for root, dirs, files in os.walk(modelsfolder):
        for file in files:
            if os.path.splitext(file)[-1] == '.smurf':
                robots_found.append(os.path.join(root, file))
    robots_dict = dict()
    for robot in robots_found:
        with open(robot, 'r') as robot_smurf:
            robot_yml = yaml.load(robot_smurf)
            model_name = robot_yml["modelname"]
            robot_files = robot_yml["files"]
            for file in robot_files:
                if file.split('_')[-1] == "poses.yml":
                    if model_name not in robots_dict:
                        robots_dict[model_name] = []
                    with open(os.path.join(os.path.dirname(robot), file)) as poses:
                        poses_yml = yaml.load(poses)
                        for pose in poses_yml['poses']:
                            robots_dict[model_name].append({"posename": pose['name']})
                            robots_dict[model_name][-1]["robotpath"] = os.path.dirname(robot)

    modelsPosesColl.clear()
    for model_name in robots_dict.keys():
        item = modelsPosesColl.add()
        item.robot_name = model_name
        item.name       = model_name
        item.label      = model_name
        item.type       = "robot_name"
        if item.hide:
            item.icon   = "RIGHTARROW"
        else:
            item.icon   = "DOWNARROW_HLT"
        current_parent = item.name
        for pose in robots_dict[model_name]:
            item        = modelsPosesColl.add()
            item.parent = current_parent
            item.name   = model_name+'_'+pose["posename"]
            item.label  = pose["posename"]
            item.path   = pose["robotpath"]
            item.type   = "robot_pose"
            item.robot_name = model_name
            item.icon   = "X_VEC"
            search_path = pose["robotpath"]
            if os.path.split(search_path)[-1] == "smurf":
                search_path = os.path.dirname(search_path)
            for file in (glob.glob(search_path + "/**/" + model_name + "_" + pose['posename'] + ".*") +
                         glob.glob(search_path + "/" + model_name + "_" + pose['posename'] + ".*")):
                if (os.path.splitext(file)[-1].lower() == ".stl") or \
                   (os.path.splitext(file)[-1].lower() == ".obj"):
                        item.model_file = os.path.join(search_path, file)
                if (os.path.splitext(file)[-1].lower() == ".png"):
                    item.preview = os.path.join(search_path, file)
                    item.name = os.path.split(file)[-1]


class ReloadModelsAndPosesOperator(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "scene.reload_models_and_poses_operator"
    bl_label = "Reload Models and Poses"

    def execute(self, context):
        loadModelsAndPoses()
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        for model_pose in modelsPosesColl:
            if not model_pose.name in bpy.data.images.keys():
                if model_pose.type == 'robot_name':
                    bpy.data.images.new(model_pose.name, 0, 0)
                elif 'robot_pose':
                    if model_pose.preview != '':
                        if os.path.split(model_pose.preview)[-1] in bpy.data.images.keys():
                            bpy.data.images[os.path.split(model_pose.preview)[-1]].reload()
                        im = bpy.data.images.load(model_pose.preview)
                        model_pose.name = im.name
                        # im.name = model_pose.name
                        im.gl_load(0, bgl.GL_LINEAR, bgl.GL_LINEAR)
                    else:
                        bpy.data.images.new(model_pose.name, 0, 0)
            else:
                bpy.data.images[model_pose.name].reload()
                bpy.data.images[model_pose.name].gl_load(0, bgl.GL_LINEAR, bgl.GL_LINEAR)
        return {'FINISHED'}


class ImportLibRobot(Operator):
    """Import a baked robot into the robot library"""
    bl_idname = "phobos.import_lib_robot"
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

    obj_name = StringProperty(
        name="New Smurf Entity Name",
        default="New Robot",
        description="Name of new Smurf Entity"
    )

    @classmethod
    def poll(self, context):
        result = False
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        root = None
        #print("modelfile: ("+modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].model_file+")")
        if context.scene.objects.active != None:
            root = sUtils.getRoot(context.scene.objects.active)
        try:
            if ( not root
                 or not sUtils.isModelRoot(root)
                 or bpy.data.images[activeModelPoseIndex].name in modelsPosesColl.keys()
                 and modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].model_file != ''
                 and len(bpy.context.selected_objects) == 0
                 or modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].robot_name != root["modelname"]
                 ):
                result = True
        except KeyError:
            result = False
        return result

    def invoke(self, context, event):
        wm = context.window_manager
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose

        selected_robot = modelsPosesColl[bpy.data.images[activeModelPoseIndex].name]
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
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        selected_robot = modelsPosesColl[bpy.data.images[activeModelPoseIndex].name]
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
    bl_idname = "phobos.create_robot_instance"
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
    bl_idname = "phobos.export_scene"
    bl_label = "Export Scene"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        exporter.exportSMURFsScene()
        endLog()
        return {'FINISHED'}


class ExportModelOperator(Operator):
    """Export the selected model(s)"""
    bl_idname = "phobos.export_robot"
    bl_label = "Export Model"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        root = sUtils.getRoot(context.selected_objects[0])
        if root.phobostype != 'link':
            log("Selection includes objects not parented to any model root, please adapt selection.", "ERROR", "ExportModelOperator")
        else:
            model, objectlist = models.buildModelDictionary(root)
            if 'smurf' in entity_types:
                entity_types['smurf'].export(model, objectlist)
            else:
                log("No export available for a robot entity!", "ERROR", __name__+".ExportModelOperator")
            endLog()
        return {'FINISHED'}


class ExportCurrentPoseOperator(Operator):
    """Bake the selected model"""
    bl_idname = "phobos.export_current_poses"
    bl_label = "Export Selected Pose"

    decimate_type = EnumProperty(name="Decimate Type",
                                 items=[('COLLAPSE','Collapse','COLLAPSE'),('UNSUBDIV','Un-Subdivide','UNSUBDIV'),('DISSOLVE','Planar','DISSOLVE')])
    decimate_ratio = FloatProperty(name="Ratio",default=0.15)
    decimate_iteration = IntProperty(name="Iterations",default=1)
    decimate_angle_limit = FloatProperty(name="Angle Limit",default=5)

    @classmethod
    def poll(self, context):
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        return (context.selected_objects and context.active_object and sUtils.isModelRoot(context.active_object)
                and bpy.data.images[activeModelPoseIndex].name in modelsPosesColl.keys()
                and modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].robot_name == context.active_object['modelname']
                and modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].type == 'robot_pose')

    def invoke(self, context, event):
        wm = context.window_manager
        bpy.context.scene.render.resolution_x = 256
        bpy.context.scene.render.resolution_y = 256
        bpy.context.scene.render.resolution_percentage = 100
        return wm.invoke_props_dialog(self, width=300, height=100)

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
        #image_settings = rd.image_settings
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

        modelsPosesColl = bpy.context.user_preferences.addons['phobos'].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        selected_robot = modelsPosesColl[bpy.data.images[activeModelPoseIndex].name]

        objectlist = sUtils.getChildren(root, selected_only=True, include_hidden=False)
        sUtils.selectObjects([root] + objectlist, clear=True, active=0)
        models.loadPose(selected_robot.robot_name, selected_robot.label)
        parameter = self.decimate_ratio
        if self.decimate_type == 'UNSUBDIV':
            parameter = self.decimate_iteration
        elif self.decimate_type == 'DISSOLVE':
            parameter = self.decimate_angle_limit
        exporter.bakeModel(objectlist, root['modelname'], selected_robot.label, decimate_type=self.decimate_type,
                           decimate_parameter=parameter)
        sUtils.selectObjects([root] + objectlist, clear=True, active=0)
        bpy.ops.scene.reload_models_and_poses_operator()
        endLog()
        return {'FINISHED'}


class ExportAllPosesOperator(Operator):
    """Bake the selected model"""
    bl_idname = "phobos.export_all_poses"
    bl_label = "Export All Poses"
    #bl_options = {'REGISTER', 'UNDO'}
    decimate_type = EnumProperty(name="Decimate Type",
                                 items=[('COLLAPSE','Collapse','COLLAPSE'),('UNSUBDIV','Un-Subdivide','UNSUBDIV'),('DISSOLVE','Planar','DISSOLVE')])
    decimate_ratio = FloatProperty(name="Ratio",default=0.15)
    decimate_iteration = IntProperty(name="Iterations",default=1)
    decimate_angle_limit = FloatProperty(name="Angle Limit",default=5)

    @classmethod
    def poll(self, context):
        return bpy.context.selected_objects and context.active_object and sUtils.isModelRoot(context.active_object)

    def invoke(self, context, event):
        wm = context.window_manager
        bpy.context.scene.render.resolution_x = 256
        bpy.context.scene.render.resolution_y = 256
        bpy.context.scene.render.resolution_percentage = 100
        return wm.invoke_props_dialog(self, width=300, height=100)

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
        #image_settings = rd.image_settings
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
        objectlist = sUtils.getChildren(root, selected_only=True, include_hidden=False)
        sUtils.selectObjects(objectlist)
        poses = models.getPoses(root['modelname'])
        bpy.context.window_manager.progress_begin(0, len(poses))
        i = 1
        for pose in poses:
            sUtils.selectObjects([root] + objectlist, clear=True, active=0)
            models.loadPose(root['modelname'], pose)
            parameter = self.decimate_ratio
            if self.decimate_type == 'UNSUBDIV':
                parameter = self.decimate_iteration
            elif self.decimate_type == 'DISSOLVE':
                parameter = self.decimate_angle_limit
            exporter.bakeModel(objectlist, root['modelname'],pose,decimate_type=self.decimate_type, decimate_parameter=parameter)
            bpy.context.window_manager.progress_update(i)
            i += 1
        bpy.context.window_manager.progress_end()
        sUtils.selectObjects([root] + objectlist, clear=True, active=0)
        bpy.ops.scene.reload_models_and_poses_operator()
        endLog()
        return {'FINISHED'}


class RobotModelImporter(bpy.types.Operator):
    """Import robot model file from various formats"""
    bl_idname = "phobos.import_robot_model"
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


def register():
    print("Registering operators.io...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.register_class(classdef)


def unregister():
    print("Unregistering operators.io...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.unregister_class(classdef)
