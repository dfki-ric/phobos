__author__ = 'kavonszadkowski'

    GUI:

    #bpy.types.World.showBodies = BoolProperty(name="showBodies", update=SetVisibleLayers)
    #bpy.types.World.showJoints = BoolProperty(name="showJoints", update=SetVisibleLayers)
    #bpy.types.World.showConstraints = BoolProperty(name="showConstraints", update=SetVisibleLayers)
    #bpy.types.World.showJointSpheres = BoolProperty(name="showJointSpheres", update=SetVisibleLayers)
    #bpy.types.World.showSensors = BoolProperty(name="showSensors", update=SetVisibleLayers)
    #bpy.types.World.showNames = BoolProperty(name="showNames", update=SetVisibleLayers)
    #bpy.types.World.showDecorations = BoolProperty(name="showDecorations", update=SetVisibleLayers)
    ##bpy.types.World.showMotorTypes = BoolProperty(name = "showMotorTypes", update=showMotorTypes)
    #bpy.types.World.manageLayers = BoolProperty(name="manage layers", update=manageLayers)
    #bpy.types.World.useDefaultLayers = BoolProperty(name="use default layers", update=useDefaultLayers)
    #bpy.types.World.linkLayer = IntProperty(name="link", update=manageLayers)


        #hw2 = hwlayout.column(align=True)
        #hw2.label(text="Templates", icon='FILE_BLANK')
        #ol.template_list("RENDERLAYER_UL_renderlayers", "", bpy.context, "layers", bpy.context.selected_objects, "active_index", rows=3)


        # # Inspection Menu
        # layout.separator()
        # layout.label(text="Inspect Robot", icon='VIEWZOOM')
        # iinlayout = layout.split()
        # ic1 = iinlayout.column(align=True)
        #
        # ic2 = iinlayout.column(align=True)
        #
        # ic2.operator('phobos.select_error', text="Select Erroneous Object")

        ## Pose Menu
        #layout.separator()
        #layout.label(text="Poses")
        #pinlayout = layout.split()
        #pc1 = pinlayout.column(align=True)
        #pc1.operator('phobos.store_pose', text='Store Current Pose')
        #pc2 = pinlayout.column(align=True)
        #pc2.operator('phobos.load_pose', text='Load Pose')

        #for root in utility.getRoots():
        #    linspect1.operator('phobos.select_model', text=root["modelname"]).modelname = \
        #     root["modelname"] if "modelname" in root else root.name


        # class PhobosSenConPanel(bpy.types.Panel):
#     """A Custom Panel in the Phobos viewport toolbar"""
#     bl_idname = "TOOLS_SENCON_PT_PHOBOS"
#     bl_label = "phobos: Sensors & Controllers"
#     bl_space_type = 'VIEW_3D'
#     bl_region_type = 'TOOLS'
#     bl_category = 'Phobos'
#
#     def draw_header(self, context):
#         self.layout.label(icon='GAME')
#
#     def draw(self, context):
#         slayout = self.layout.split()
#         sc1 = slayout.column(align=True)
#         # create sensor creation buttons
#         #row_sensors.label(text="Add Sensors / Controllers")
#
#         #sensor_split = row_sensors.split()
#         #n_sensortypes = int(len(defs.sensortypes))
#         #half_n_sensortypes = int(n_sensortypes / 2)
#         #col_sensor_1 = sensor_split.column(align=True)
#         #for i in range(half_n_sensortypes):  #sensor in defs.sensorTypes:
#         #    sensor = defs.sensortypes[i]
#         #    #col_sensor_1.operator('phobos.add_sensor_'+sensor, text=sensor)
#         #    col_sensor_1.operator('phobos.add_sensor', text=sensor).sensor_type = sensor
#         #col_sensor_2 = sensor_split.column(align=True)
#         #for i in range(n_sensortypes - half_n_sensortypes):
#         #    sensor = defs.sensortypes[i + half_n_sensortypes]
#         #    col_sensor_2.operator('phobos.add_sensor', text=sensor).sensor_type = sensor
#         #    #col_sensor_2.operator('phobos.add_sensor_'+sensor, text=sensor)
#         sc2 = slayout.column(align=True)
#


# class PhobosVisPanel(bpy.types.Panel):
#     """A Custom Panel in the Phobos viewport toolbar"""
#     bl_idname = "TOOLS_VIS_PT_PHOBOS"
#     bl_label = "phobos: Visibility"
#     bl_space_type = 'VIEW_3D'
#     bl_region_type = 'TOOLS'
#     bl_category = 'Phobos'
#
#     def draw_header(self, context):
#         self.layout.label(icon = 'VISIBLE_IPO_ON')
#
#     def draw(self, context):
#         lvis = self.layout
#         # Visibility
#         lsplit = lvis.column(align=True)
#         #lsplit = layout.split()
#         lsplit.prop(bpy.data.worlds[0], "showBodies")
#         lsplit.prop(bpy.data.worlds[0], "showJoints")
#         lsplit.prop(bpy.data.worlds[0], "showJointSpheres")
#         lsplit.prop(bpy.data.worlds[0], "showSensors")
#         lsplit.prop(bpy.data.worlds[0], "showDecorations")
#         lsplit.prop(bpy.data.worlds[0], "showConstraints")
#         lsplit.prop(bpy.data.worlds[0], "showNames")
#         lsplit.prop(bpy.data.worlds[0], "showMotorTypes")



# def SetVisibleLayers(self, context):
#     """Set active Layers according to Phobos world data."""
#     layers = [False] * 20
#     layers[0] = bpy.data.worlds[0].showBodies
#     layers[1] = bpy.data.worlds[0].showJoints
#     layers[2] = bpy.data.worlds[0].showJointSpheres
#     layers[3] = bpy.data.worlds[0].showSensors
#     layers[4] = bpy.data.worlds[0].showDecorations
#     layers[5] = bpy.data.worlds[0].showConstraints
#     onetrue = False
#     for b in layers:
#         onetrue = onetrue or b
#     if not onetrue:
#         bpy.data.worlds[0].showBodies = True
#         layers[0] = True
#     bpy.context.scene.layers = layers
#     for obj in bpy.data.objects:
#         obj.show_name = bpy.data.worlds[0].showNames


# def setWorldView(b):
#     bpy.data.worlds[0].showBodies = b[0]
#     bpy.data.worlds[0].showJoints = b[1]
#     bpy.data.worlds[0].showJointSpheres = b[2]
#     bpy.data.worlds[0].showSensors = b[3]
#     bpy.data.worlds[0].showDecorations = b[4]
#     bpy.data.worlds[0].showConstraints = b[5]
#     bpy.data.worlds[0].showNames = b[6]
#     applyWorldView()


# def applyWorldView():
#     layers = [False] * 20
#     layers[0] = bpy.data.worlds[0].showBodies
#     layers[1] = bpy.data.worlds[0].showJoints
#     layers[2] = bpy.data.worlds[0].showJointSpheres
#     layers[3] = bpy.data.worlds[0].showSensors
#     layers[4] = bpy.data.worlds[0].showDecorations
#     layers[5] = bpy.data.worlds[0].showConstraints
#     layers[6] = bpy.data.worlds[0].showNames
#     bpy.context.scene.layers = layers


# def manageLayers(self, context):
#     if bpy.data.worlds[0].manageLayers:
#         pass  # TODO: not so important


# def useDefaultLayers(self, context):
#     pass  # TODO: not so important


# def showMotorTypes(self, context):
# """Changes materials of joints to indicate different motor types."""
#     if bpy.data.worlds[0].showMotorTypes:
#         types = {}
#         n_indicators = 0
#         for obj in bpy.context.selected_objects:
#             if obj.phobostype == "joint":
#                 if "spec_motor" in obj:
#                     if not (obj["spec_motor"] in types):
#                         n_indicators += 1
#                         types[obj["spec_motor"]] = "indicator" + str(n_indicators)
#                     if not types[obj["spec_motor"]] in obj.data.materials:
#                         obj.data.materials.append(bpy.data.materials[types[obj["spec_motor"]]])
#                         obj.data.materials.pop(0, update_data=True)
#         bpy.data.scenes[0].update()
#     else:
#         for obj in bpy.context.selected_objects:
#             if obj.phobostype == "joint":
#                 obj.data.materials.append(bpy.data.materials["Joint Discs"])
#                 obj.data.materials.pop(0, update_data=True)
#     bpy.data.scenes[0].update()


# def get_axis_rotation_matrix(axis, angle):
#     '''
#     unnecessary
#     '''
#
#     import math
#
#     u, v, w = axis
#     u_sqr = u**2
#     v_sqr = v**2
#     w_sqr = w**2
#     sin_theta = math.sin(angle)
#     cos_theta = math.cos(angle)
#
#     a11 = u_sqr + (v_sqr + w_sqr) * cos_theta
#     a12 = u * v * (1 - cos_theta) - w * sin_theta
#     a13 = u * w * (1 - cos_theta) + v * sin_theta
#     a14 = 0.0
#     a21 = u * v * (1 - cos_theta) + w * sin_theta
#     a22 = v_sqr + (u_sqr + w_sqr) * cos_theta
#     a23 = v * w * (1 - cos_theta) - u * sin_theta
#     a24 = 0.0
#     a31 = u * w * (1 - cos_theta) - v * sin_theta
#     a32 = v * w * (1 - cos_theta) + u * sin_theta
#     a33 = w_sqr + (u_sqr + v_sqr) * cos_theta
#     a34 = 0.0
#     a41 = 0.0
#     a42 = 0.0
#     a43 = 0.0
#     a44 = 1.0
#
#     rotation_matrix = [[a11, a12, a13, a14],
#                        [a21, a22, a23, a24],
#                        [a31, a32, a33, a34],
#                        [a41, a42, a43, a44]]
#
#     return mathutils.Matrix(rotation_matrix).to_4x4()

# Register and add to the file selector
#bpy.utils.register_class(RobotModelImporter)


# from urdf.py
#def apply_rel_ids_old(self, nodes):
#    '''
#    Collect the absolute poses for all nodes.

#   Does not work like that since it calculates poses relative to root,
#   but may become useful again.
#    '''
    #base_poses = {}
    #rel_poses = {}
    #for node in nodes:
    #    index = int(node.find('index').text)
    #    xml_position = node.find('position')
    #    xml_rotation = node.find('rotation')
    #    position, rotation = pos_rot_tree_to_lists(xml_position, xml_rotation)
    #    pose = calc_pose_formats(position, rotation)
    #
    #    if node.find('relativeid') is not None:
    #        rel_pose_dict = {'pose': pose,
    #                         'rel_id': int(node.find('relativeid').text)}
    #        rel_poses[index] = rel_pose_dict
    #    else:
    #        base_pose_dict = {'pose': pose}
    #        base_poses[index] = base_pose_dict
    #num_rel_poses = -1
    #while rel_poses:
    #    to_delete = []
    #    # check for infinite loop (happens if referenced node does not exist):
    #    if len(rel_poses) == num_rel_poses:
    #        print('Error: non-existant relative id')
    #        break
    #    num_rel_poses = len(rel_poses)
    #    for rel_index in rel_poses:
    #        rel_pose = rel_poses[rel_index]
    #        base = rel_pose['rel_id']
    #        if base in base_poses:
    #            base_pose = base_poses[base]['pose']
    #            rel = rel_pose['pose']
    #            base_matrix = mathutils.Matrix(base_pose['matrix']).to_4x4()
    #            relative_matrix = mathutils.Matrix(rel['matrix']).to_4x4()
    #            applied_matrix = base_matrix * relative_matrix
    #            loc, rot, scale = applied_matrix.decompose()
    #            applied_pos = [loc.x, loc.y, loc.z]
    #            applied_rot = [rot.w, rot.x, rot.y, rot.z]
    #            base_poses[rel_index] = {'pose': calc_pose_formats(applied_pos, applied_rot)}
    #            to_delete.append(rel_index)
    #    for index in to_delete:
    #        del rel_poses[index]
    #self.applied_rel_id_poses = base_poses
