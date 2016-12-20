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


#------------------------------------------
# entities.py
#----------------------------------

# def export(model, objectlist, path=None, structured=True, entity_types=('smurf'), mesh_format='obj'):
#     """Configures and performs export of selected or specified model.
#
#     :param path: The path to export the model to.
#     :type path: str
#     :param model: The model to be exported.
#     :type model: dict
#
#     """
#     # check for valid model and objectlist
#     if not model or not objectlist:
#         return  # TODO: Check if there could be a valid model to export without a single object
#
#     # set up path
#     if not path:
#         if os.path.isabs(bpy.data.worlds[0].phobosexportsettings.path:
#             outpath = securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"),
#                                                                  bpy.data.worlds[0].phobosexportsettings.path)))
#         else:
#             outpath = securepath(os.path.expanduser(bpy.data.worlds[0].phobosexportsettings.path))
#     else:
#         outpath = path
#     if not outpath.endswith(os.path.sep):
#         outpath += os.path.sep
#     meshoutpath = securepath(os.path.join(outpath, 'meshes'))
#     log("Export path: " + outpath, "DEBUG", "export")
#
#     # parse export settings
#     yaml = bpy.data.worlds[0].phobosexportsettings.exportYAML
#     urdf = bpy.data.worlds[0].phobosexportsettings.exportURDF
#     srdf = bpy.data.worlds[0].phobosexportsettings.exportSRDF
#     smurf = bpy.data.worlds[0].phobosexportsettings.exportSMURF
#     meshexp = bpy.data.worlds[0].phobosexportsettings.exportMeshes
#     texexp = bpy.data.worlds[0].phobosexportsettings.exportTextures
#     objexp = bpy.data.worlds[0].phobosexportsettings.useObj
#     stlexp = bpy.data.worlds[0].phobosexportsettings.useStl
#     daeexp = bpy.data.worlds[0].phobosexportsettings.useDae
#
#     # export data
#     if yaml or urdf or smurf:
#         if yaml:
#             exportModelToYAML(model, outpath + model["modelname"] + "_dict.yml")
#         if srdf:
#             exportModelToSRDF(model, outpath + model["modelname"] + ".srdf")
#         if smurf:
#             if bpy.data.worlds[0].phobosexportsettings.structureExport:
#                 securepath(os.path.join(outpath, 'smurf'))
#                 securepath(os.path.join(outpath, 'urdf'))
#                 exportModelToSMURF(model, os.path.join(outpath, 'smurf/'))
#             else:
#                 exportModelToSMURF(model, outpath)
#         elif urdf:
#             if bpy.data.worlds[0].phobosexportsettings.structureExport:
#                 securepath(os.path.join(outpath, 'urdf'))
#                 exportModelToURDF(model, os.path.join(outpath, 'urdf', model["modelname"] + ".urdf"))
#             else:
#                 exportModelToURDF(model, outpath + model["modelname"] + ".urdf")
#     if meshexp:
#         meshnames = set()
#         exportobjects = set()
#         for obj in objectlist:
#             try:
#                 if ((obj.phobostype == 'visual' or obj.phobostype == 'collision') and
#                         (obj['geometry/type'] == 'mesh') and (obj.data.name not in meshnames)):
#                     meshnames.add(obj.data.name)
#                     exportobjects.add(obj)
#                     for lod in obj.lod_levels:
#                         if lod.object.data.name not in meshnames:
#                             meshnames.add(lod.object.data.name)
#                             exportobjects.add(lod.object)
#             except KeyError:
#                 log("Undefined geometry type in object " + obj.name + ", skipping mesh export", "ERROR", "export")
#
#         if exportobjects:  # if there are meshes to export
#             show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269
#             if show_progress:
#                 wm = bpy.context.window_manager
#                 wm.progress_begin(0, float(len(exportobjects)))
#                 i = 1
#             log("Exporting " + str(len(exportobjects)) + " meshes to " + meshoutpath + "...", "INFO", "export")
#             for expobj in exportobjects:
#                 if objexp:
#                     meshes.meshes.exportObj(meshoutpath, expobj)
#                 if stlexp:
#                     meshes.meshes.exportStl(meshoutpath, expobj)
#                 if daeexp:
#                     meshes.meshes.exportDae(meshoutpath, expobj)
#                 if show_progress:
#                     wm.progress_update(i)
#                     i += 1
#             if show_progress:
#                 wm.progress_end()
#
#     if texexp:
#         log("Exporting textures to " + os.path.join(outpath, 'textures') + "...", "INFO", "export")
#         securepath(os.path.join(outpath, 'textures'))
#         for materialname in model['materials']:
#             mat = model['materials'][materialname]
#             for texturetype in ['diffuseTexture', 'normalTexture', 'displacementTexture']:
#                 if texturetype in mat:
#                     texpath = os.path.join(os.path.expanduser(bpy.path.abspath('//')), mat[texturetype])
#                     if os.path.isfile(texpath):
#                         shutil.copy(texpath, os.path.join(outpath, 'textures', os.path.basename(mat[texturetype])))

# class RobotModelParser():
#     """Base class for a robot model file parser of a specific type
#
#     """
#
#     def __init__(self, filepath):
#         """This init saves the filepath splitted into path and filename and creates an initial empty robot dictionary.
#
#         :param filepath: The filepath you want to export the robot to *WITH FILENAME*
#         :type filepath: str
#
#         """
#         self.filepath = filepath
#         self.path, self.filename = os.path.split(self.filepath)
#         self.robot = {'links': {},
#                       'joints': {},
#                       'sensors': {},
#                       'motors': {},
#                       'controllers': {},
#                       'materials': {},
#                       'groups': {},
#                       'chains': {}
#                       }
#         if os.access(self.path, os.W_OK):
#             self.tmp_path = self.path
#         elif os.access(os.path.expanduser('.'), os.W_OK):
#             self.tmp_path = os.path.expanduser('.')
#         else:
#             raise Exception('WTF? No write permission for home dir.')
#
#     def praefixNames(self, name, praefix):
#         """This function takes a name and a praefix and praefixes the name with it if its not already praefixed with it.
#
#         :param name: The name to praefix.
#         :type name: str
#         :param praefix:  The praefix to use.
#         :type praefix: str
#         :return: str - The praefixed name
#
#         """
#         prae = praefix + "_"
#         if (name.startswith(prae)):
#             return name
#         else:
#             return prae+name
#
#     def scaleLink(self, link, newlink):
#         """Scales newly-created armatures depending on the link's largest collision object.
#         The function is very simple and could be improved to scale the links even more appropriate.
#
#         :param link: the link containing the collision objects
#         :type link: dict
#         :param newlink: the link you want to scale.
#         :type newlink: dict
#
#         """
#         newscale = 0.3
#         if len(link['collision']) > 0:
#             sizes = []
#             for collname in link['collision']:
#                 collobj = link['collision'][collname]
#                 colltype = collobj['geometry']['type']
#                 if colltype == 'sphere':
#                     sizes.append(collobj['geometry']['radius'])
#                 elif colltype == 'cylinder':
#                     sizes.append(max(collobj['geometry']['radius'], collobj['geometry']['length']))
#                 elif colltype == 'mesh':
#                     sizes.append(max(collobj['geometry']['scale']))  # TODO: this alone is not very informative
#                 else:
#                     sizes.append(max(collobj['geometry']['size']))
#             newscale = max(sizes)
#         newlink.scale = (newscale, newscale, newscale)
#
#     def placeChildLinks(self, parent):
#         """This function creates the parent-child-relationship for a given parent and all existing children in blender.
#
#         :param parent: This is the parent link you want to set the children for.
#         :type: dict
#
#         """
#         bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes['link'])
#         print(parent['name'] + ', ', end='')
#         children = []
#         for l in self.robot['links']:
#             if 'parent' in self.robot['links'][l] and self.robot['links'][l]['parent'] == parent['name']:
#                 children.append(self.robot['links'][l])
#         for child in children:
#             # 1: set parent relationship (this makes the parent inverse the inverse of the parents world transform)
#             parentLink = bpy.data.objects[self.praefixNames(parent['name'], "link")]
#             childLink = bpy.data.objects[self.praefixNames(child['name'], "link")]
#             sUtils.selectObjects([childLink, parentLink], True, 1)
#             bpy.ops.object.parent_set(type='BONE_RELATIVE')
#             # 2: move to parents origin by setting the world matrix to the parents world matrix
#             childLink.matrix_world = parentLink.matrix_world        # removing this line does not seem to make a difference
#
#             # #bpy.context.scene.objects.active = childLink
#             # if 'pivot' in child:
#             #     pivot = child['pivot']
#             #     cursor_location = bpy.context.scene.cursor_location
#             #     bpy.context.scene.cursor_location = mathutils.Vector((-pivot[0]*0.3, -pivot[1]*0.3, -pivot[2]*0.3))
#             #     bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
#             #     bpy.context.scene.cursor_location = cursor_location
#
#             # 3: apply local transform as saved in urdf (change matrix_local from identity to urdf)
#             location = mathutils.Matrix.Translation(child['pose']['translation'])
#             rotation = mathutils.Euler(tuple(child['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
#             transform_matrix = location * rotation
#             childLink.matrix_local = transform_matrix
#             # 4: be happy, as world and basis are now the same and local is the transform to be exported to urdf
#             # 5: take care of the rest of the tree
#             self.placeChildLinks(child)
#
#     def placeLinkSubelements(self, link):
#         """This function finds all subelements for a given link and sets the appropriate relations.
#         In this case subelements are interials, visuals and collisions.
#
#         :param link: The parent link you want to set the subelements for
#         :type link: dict
#
#         """
#         bpy.context.scene.layers = bUtils.defLayers([defs.layerTypes[t] for t in defs.layerTypes])
#         parentLink = bpy.data.objects[self.praefixNames(link['name'], "link")]
#         if 'inertial' in link:
#             if 'pose' in link['inertial']:
#                 urdf_geom_loc = mathutils.Matrix.Translation(link['inertial']['pose']['translation'])
#                 urdf_geom_rot = mathutils.Euler(tuple(link['inertial']['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
#             else:
#                 urdf_geom_loc = mathutils.Matrix.Identity(4)
#                 urdf_geom_rot = mathutils.Matrix.Identity(4)
#             geoname = link['inertial']['name'] #g
#             try:
#                 geom = bpy.data.objects[geoname]
#             except:
#                 print('ERROR: missing subelement')
#                 return
#             inertialname = link['inertial']['name']
#             inertialobj = bpy.data.objects[inertialname]
#             sUtils.selectObjects([inertialobj, parentLink], True, 1)
#             bpy.ops.object.parent_set(type='BONE_RELATIVE')
#             inertialobj.matrix_local = urdf_geom_loc * urdf_geom_rot
#         for geomsrc in ['visual', 'collision']:
#             if geomsrc in link:
#                 for g in link[geomsrc]:
#                     geomelement = link[geomsrc][g]
#                     if 'pose' in geomelement:
#                         urdf_geom_loc = mathutils.Matrix.Translation(geomelement['pose']['translation'])
#                         urdf_geom_rot = mathutils.Euler(tuple(geomelement['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
#                     else:
#                         urdf_geom_loc = mathutils.Matrix.Identity(4)
#                         urdf_geom_rot = mathutils.Matrix.Identity(4)
#                     geoname = self.praefixNames(geomelement['name'], geomsrc)
#                     geom = bpy.data.objects[geoname]
#                     # FIXME: this does not do anything - how to set basis matrix to local?
#                     #geom.matrix_world = parentLink.matrix_world
#                     #selectObjects([geom], True, 0)
#                     #bpy.ops.object.transform_apply(location=True, rotation=True)
#                     sUtils.selectObjects([geom, parentLink], True, 1)
#                     bpy.ops.object.parent_set(type='BONE_RELATIVE')
#                     geom.matrix_local = urdf_geom_loc * urdf_geom_rot
#                     try:
#                         geom.scale = geomelement['geometry']['scale']
#                     except KeyError:
#                         pass
#
#                     sUtils.selectObjects([geom, parentLink], True, 1)
#
#     def _get_object(self, link_name):
#         """
#         """
#         objects = bpy.context.scene.objects
#         obj = objects[link_name]
#         return obj
#
#     def _apply_joint_angle_offsets(self):
#         """This function applies joint angle offsets in blender to all links in self.robot.
#
#         """
#         links = self.robot['links']
#         for link_name in links:
#             link = links[link_name]
#             if 'axis1' in link:
#                 # move edit bone y axis to axis1
#                 bpy.ops.object.mode_set(mode='EDIT')
#
#                 axis_vec = mathutils.Vector(link['axis1'])
#                 bpy.data.objects[link_name].data.bones.active.tail = axis_vec
#
#                 #print()
#                 #print('link name:', link_name)
#                 #print("axis:", axis_vec)
#
#                 bpy.ops.object.mode_set(mode='OBJECT')
#                 pass
#             if 'angle_offset' in link:
#                 obj = self._get_object(link_name)
#                 angle = -(link['angle_offset'])
#                 bpy.ops.object.mode_set(mode='POSE')
#                 pose_bone = obj.pose.bones['Bone']
#                 bpy.ops.pose.armature_apply()
#                 bpy.ops.object.mode_set(mode='OBJECT')
#                 y_axis = mathutils.Vector([0, 1, 0])
#                 bone_matrix = pose_bone.matrix
#                 loc, rot, scale = bone_matrix.decompose()
#                 axis = rot * y_axis
#                 offset_matrix = mathutils.Matrix.Rotation(angle, 4, axis)
#                 pose_bone.matrix = offset_matrix * bone_matrix
#
#
#     def createController(self, controller):
#         """This function creates a specified controller.
#
#         :param controller: The controller to create.
#         :type controller: dict
#
#         """
#         controllers.addController(controller)
#
#     def createLight(self, light):
#         """This function creates a specified light.
#
#         :param light: The light to create.
#         :type light: dict
#
#         """
#         lights.addLight(light)
#
#     def createGroup(self, group):
#         """
#
#         :param group:
#
#         """
#         pass
#
#     def createChain(self, chain):
#         """
#
#         :param chain:
#
#         """
#         pass
#
#     def createBlenderModel(self): #TODO: solve problem with duplicated links (linklist...namespaced via robotname?)
#         """Creates the blender object representation of the imported model.
#         For that purpose it uses the former specified robot model dictionary.
#
#         """
#         print("\n\nCreating Blender model...")
#         print("Creating links...")
#         for l in self.robot['links']:
#             #print(l + ', ', end='')
#             link = self.robot['links'][l]
#             #print(link['name'])
#             self.createLink(link)
#
#         print("\n\nCreating joints...")
#         for j in self.robot['joints']:
#             print(j + ', ', end='')
#             joint = self.robot['joints'][j]
#             self.createJoint(joint)
#
#         #build tree recursively and correct translation & rotation on the fly
#         print("\n\nPlacing links...")
#         for l in self.robot['links']:
#             if not 'parent' in self.robot['links'][l]:
#                 root = self.robot['links'][l]
#                 self.placeChildLinks(root)
#                 print("\n\nAssigning model name...")
#                 try:
#                     rootlink = sUtils.getRoot(bpy.data.objects[self.praefixNames(root['name'], "link")])
#                     rootlink['modelname'] = self.robot['name']
#                 except KeyError:
#                     log("Could not assign model name to root link.", "ERROR")
#         for link in self.robot['links']:
#             self.placeLinkSubelements(self.robot['links'][link])
#
#
#         print("\n\nCreating sensors...")
#         for s in self.robot['sensors']:
#             sensor = self.robot['sensors'][s]
#             self.createSensor(sensor)
#
#         for sensorname in self.robot['sensors']:
#             sensor = self.robot['sensors'][sensorname]
#             self.attachSensor(sensor)
#
#         print("\n\nCreating motors...")
#         for m in self.robot['motors']:
#             motor = self.robot['motors'][m]
#             self.createMotor(motor)
#
#         print("\n\nCreating controllers...")
#         for c in self.robot['controllers']:
#             controller = self.robot['controllers'][c]
#             self.createController(controller)
#
#         print("\n\nCreating groups...")
#         for g in self.robot['groups']:
#             group = self.robot['groups'][g]
#             self.createGroup(group)
#
#         print("\n\nCreating chains...")
#         for ch in self.robot['chains']:
#             chain = self.robot['chains'][ch]
#             self.createChain(chain)
#
#         print("\n\nCreating lights...")
#         for light in self.robot['lights']:
#             self.createLight(self.robot['lights'][light])
#
#         #self._apply_joint_angle_offsets()
#
#         # remove tmp dir containing extracted object files
#         if os.path.isdir(os.path.join(self.tmp_path, tmp_dir_name)):
#             shutil.rmtree(os.path.join(self.tmp_path, tmp_dir_name))
#
#         print('Done!')
#
#
#     def _debug_output(self):
#         """Writes the robot dictionary to a yaml file in the source file's directory
#
#         """
#         with open(self.filepath + '_ref_debug.yml', 'w') as outputfile:
#             outputfile.write(yaml.dump(self.robot)) #last parameter prevents inline formatting for lists and dictionaries
