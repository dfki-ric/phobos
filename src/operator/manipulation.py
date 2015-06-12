#!/usr/bin/python

"""
.. module:: phobos.operator.manipulation
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators to manipulate blender objects

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


class ShareMesh(Operator):
    """ShareMeshOperator
    This operator takes the data block of the active element and sets it for all other selected elements.
    It also tags the objects with an visual/export/unifiedMesh custom property telling the meshes name.

    """

    bl_idname = "object.phobos_share_mesh"
    bl_label = "Unifies the selected objects meshes by setting all meshes to the active objects one"
    bl_options = {'REGISTER', 'UNDO'}

    meshName = StringProperty(
        name="shared Meshes Name",
        default='',
        description="The shared meshes name")

    def execute(self, context):
        """Executes the operator and unifies the selected objects meshes

        :param context: The blender context to work with
        :return: Blender result
        """
        startLog(self)
        objects = context.selected_objects
        source = context.active_object
        sMProp = 'geometry/' + defs.reservedProperties['SHAREDMESH']
        newName = source.name if self.meshName == "" else self.meshName
        log(source.name, "INFO")
        for obj in objects:
            if 'phobostype' in obj and obj.phobostype in ("visual", "collision") and obj != source:
                log("Setting data for: " + obj.name, "INFO")
                obj.data = source.data
                obj[sMProp] = newName
        if sMProp in source: del obj[sMProp]
        source['geometry/filename'] = newName
        log("Successfully shared the meshes for selected objects!", "INFO")
        endLog()
        return {'FINISHED'}


class UndoShareMesh(Operator):
    """UndoShareMeshOperator
    This operator undos the mesh sharing of all selected objects by delinking all objects mesh data and deleting the
    geometry/sharedMeshFile tags.
    """

    bl_idname = "object.phobos_undo_share_mesh"
    bl_label = "Undos the shared mesh operation on all selected objects"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        """Executes the operator and undos all mesh sharing

        :param context: The blender context to work with
        :return: Blender result.
        """
        bpy.ops.object.make_single_user(type='SELECTED_OBJECTS', object=True, obdata=True)
        sMProp = 'geometry/' + defs.reservedProperties['SHAREDMESH']
        for obj in context.selected_objects:
            if sMProp in obj:
                del obj[sMProp]
            if 'geometry/filename' in obj:
                del obj['geometry/filename']
        return {'FINISHED'}


class SortObjectsToLayersOperator(Operator):
    """SortObjectsToLayersOperator

    """
    bl_idname = "object.phobos_sort_objects_to_layers"
    bl_label = "Sorts all selected objects to their according layers"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        for obj in context.selected_objects:
            try:
                phobosType = obj.phobostype
                if phobosType != 'controller' and phobosType != 'undefined':
                    layers = 20 * [False]
                    layers[defs.layerTypes[phobosType]] = True
                    obj.layers = layers
                if phobosType == 'undefined':
                    log("The phobostype of the object '" + obj.name + "' is undefined")
            except AttributeError:
                log("The object '" + obj.name + "' has no phobostype", "ERROR")  # Handle this as error or warning?
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        if len(context.selected_objects) > 0:
            return True
        else:
            return False


class AddChainOperator(Operator):
    """AddChainOperator

    """
    bl_idname = "object.phobos_add_chain"
    bl_label = "Adds a chain between two selected objects."
    bl_options = {'REGISTER', 'UNDO'}

    chainname = StringProperty(
        name='chainname',
        default='new_chain',
        description='name of the chain to be created')

    def execute(self, context):
        endobj = context.active_object
        for obj in context.selected_objects:
            if obj is not context.active_object:
                startobj = obj
                break
        if not 'startChain' in startobj:
            startobj['startChain'] = [self.chainname]
        else:
            namelist = startobj['startChain']
            if self.chainname not in namelist:
                namelist.append(self.chainname)
            startobj['startChain'] = namelist
        if not 'endChain' in endobj:
            endobj['endChain'] = [self.chainname]
        else:
            namelist = endobj['endChain']
            if self.chainname not in namelist:
                namelist.append(self.chainname)
            endobj['endChain'] = namelist
        return {'FINISHED'}


class SetMassOperator(Operator):
    """SetMassOperator

    """
    bl_idname = "object.phobos_set_mass"
    bl_label = "Sets the mass of the selected object(s)."
    bl_options = {'REGISTER', 'UNDO'}

    mass = FloatProperty(
        name='mass',
        default=0.001,
        description='mass (of active object) in kg')

    userbmass = BoolProperty(
        name='use rigid body mass',
        default=False,
        description='If True, mass entry from rigid body data is used.')

    @classmethod
    def poll(cls, context):
        for obj in context.selected_objects:
            if obj.phobostype in ['visual', 'collision', 'inertial']:
                return True
        return False

    def invoke(self, context, event):
        if 'mass' in context.active_object:
            self.mass = context.active_object['mass']
        return self.execute(context)

    def execute(self, context):
        startLog(self)
        for obj in bpy.context.selected_objects:
            if obj.phobostype in ['visual', 'collision', 'inertial']:
                try:
                    oldmass = obj['mass']
                except KeyError:
                    log("The object '" + obj.name + "' has no mass")
                    oldmass = None
                if self.userbmass:
                    try:
                        obj['mass'] = obj.rigid_body.mass
                    except AttributeError:
                        obj['mass'] = 0.001
                        # print("### Error: object has no rigid body properties.")
                        log("The object '" + obj.name + "' has no rigid body properties. Set mass to 0.001", "ERROR")
                else:
                    obj['mass'] = self.mass
                if obj['mass'] != oldmass:
                    t = dt.now()
                    obj['masschanged'] = t.isoformat()
        endLog()
        return {'FINISHED'}


class SyncMassesOperator(Operator):
    """SyncMassesOperator

    """
    bl_idname = "object.phobos_sync_masses"
    bl_label = "Synchronize masses among the selected object(s)."
    bl_options = {'REGISTER', 'UNDO'}

    synctype = EnumProperty(
        items=(("vtc", "visual to collision", "visual to collision"),
               ("ctv", "collision to visual", "collision to visual"),
               ("lto", "latest to oldest", "latest to oldest")),
        name="synctype",
        default="vtc",
        description="Phobos object type")

    updateinertial = BoolProperty(
        name='robotupdate inertial',
        default=False,
        description='update inertials'
    )

    def execute(self, context):
        sourcelist = []
        targetlist = []
        processed = set()
        links = [obj.name for obj in bpy.context.selected_objects if obj.phobostype == 'link']
        t = dt.now()
        objdict = {obj.name: obj for obj in bpy.data.objects if obj.phobostype in ['visual', 'collision']
                   and obj.parent.name in links}
        # gather all name bases of objects for which both visual and collision are present
        for obj in objdict.keys():
            basename = obj.replace(objdict[obj].phobostype + '_', '')
            if 'visual_' + basename in objdict.keys() and 'collision_' + basename in objdict.keys():
                processed.add(basename)
        # fill source and target lists for syncing
        for basename in processed:
            if self.synctype == "vtc":
                sourcelist.append('visual_' + basename)
                targetlist.append('collision_' + basename)
            elif self.synctype == "ctv":
                targetlist.append('visual_' + basename)
                sourcelist.append('collision_' + basename)
            else:  # latest to oldest
                try:
                    tv = utility.datetimeFromIso(objdict['visual_' + basename]['masschanged'])
                    tc = utility.datetimeFromIso(objdict['collision_' + basename]['masschanged'])
                    if tc < tv:  # if collision information is older than visual information
                        sourcelist.append('visual_' + basename)
                        targetlist.append('collision_' + basename)
                    else:
                        targetlist.append('visual_' + basename)
                        sourcelist.append('collision_' + basename)
                except KeyError:
                    print(basename, "has insufficient data for time-based synchronisation of masses.")
        # sync the mass values
        for i in range(len(sourcelist)):
            try:
                objdict[targetlist[i]]['mass'] = objdict[sourcelist[i]]['mass']
            except KeyError:
                print("No mass information in object", targetlist[i])
            if self.synctype != "vtc" and self.synctype != "ctv":
                objdict[targetlist[i]]['masschanged'] = objdict[sourcelist[i]]['masschanged']

        for linkname in links:
            masssum = 0.0
            link = bpy.data.objects[linkname]
            viscols = inertia.getInertiaRelevantObjects(link)
            for obj in viscols:
                masssum += obj['mass']
            link['mass'] = masssum
            link['masschanged'] = t.isoformat()
            if self.updateinertial:
                inertia.createInertials(link)
        return {'FINISHED'}


class SetXRayOperator(Operator):
    """SetXrayOperator

    """
    bl_idname = "object.phobos_set_xray"
    bl_label = "Shows the selected/chosen objects via X-Ray."
    bl_options = {'REGISTER', 'UNDO'}

    objects = EnumProperty(
        name="show objects:",
        default='selected',
        items=(('all',) * 3, ('selected',) * 3, ('by name',) * 3) + defs.phobostypes,
        description="show objects via x-ray")

    show = BoolProperty(
        name="show",
        default=True,
        description="set to")

    namepart = StringProperty(
        name="name contains",
        default="",
        description="part of a name for objects to be selected in 'by name' mode")

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT' or context.mode == 'POSE'

    def draw(self, context):
        layout = self.layout
        layout.label(text="Select Items for X-Ray view")

        layout.prop(self, "objects")
        layout.prop(self, "show", text="x-ray enabled" if self.show else "x-ray disabled")
        if self.objects == 'by name':
            layout.prop(self, "namepart")

    def execute(self, context):
        if self.objects == 'all':
            objlist = bpy.data.objects
        elif self.objects == 'selected':
            objlist = bpy.context.selected_objects
        elif self.objects == 'by name':
            objlist = [obj for obj in bpy.data.objects if obj.name.find(self.namepart) >= 0]
        else:
            objlist = [obj for obj in bpy.data.objects if obj.phobostype == self.objects]
        for obj in objlist:
            obj.show_x_ray = self.show
        return {'FINISHED'}


class UpdatePhobosModelsOperator(Operator):
    """UpdatePhobosModelsOperator

    """
    bl_idname = "object.phobos_update_models"
    bl_label = "Update Phobos properties for all objects"
    bl_options = {'REGISTER', 'UNDO'}

    property_fix = BoolProperty(
        name='fix',
        default=False,
        description="try to fix detected errors?")

    print("phobos: Updating Phobos properties for selected objects...")

    def execute(self, context):
        materials.createPhobosMaterials()  # TODO: this should move to initialization
        robotupdate.updateModels(utility.getRoots(), self.property_fix)
        return {'FINISHED'}


class SetPhobosType(Operator):
    """Set phobostype Operator

    """
    bl_idname = "object.phobos_set_phobostype"
    bl_label = "Edit phobostype of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    phobostype = EnumProperty(
        items=defs.phobostypes,
        name="phobostype",
        default="undefined",
        description="phobostype")

    def execute(self, context):
        for obj in bpy.context.selected_objects:
            obj.phobostype = self.phobostype
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'


class BatchEditPropertyOperator(Operator):
    """Batch-Edit Property Operator

    """
    bl_idname = "object.phobos_batch_property"
    bl_label = "Edit custom property of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    property_name = StringProperty(
        name="property_name",
        default="",
        description="custom property name")

    property_value = StringProperty(
        name="property_value",
        default="",
        description="custom property value")

    def execute(self, context):
        value = utility.parse_number(self.property_value)
        if value == '':
            for obj in bpy.context.selected_objects:
                if self.property_name in obj:
                    del (obj[self.property_name])
        else:
            for obj in bpy.context.selected_objects:
                obj[self.property_name] = value
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'


class CopyCustomProperties(Operator):
    """Copy Custom Properties Operator

    """
    bl_idname = "object.phobos_copy_props"
    bl_label = "Edit custom property of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    empty_properties = BoolProperty(
        name='empty',
        default=False,
        description="empty properties?")

    def execute(self, context):
        slaves = context.selected_objects
        master = context.active_object
        slaves.remove(master)
        props = robotdictionary.cleanObjectProperties(dict(master.items()))
        for obj in slaves:
            if self.empty_properties:
                for key in obj.keys():
                    del (obj[key])
            for key in props.keys():
                obj[key] = props[key]
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        obs = context.selected_objects
        ob = context.active_object
        return len(obs) > 0 and ob is not None and ob.mode == 'OBJECT'


class RenameCustomProperty(Operator):
    """Rename Custom Properties Operator

    """
    bl_idname = "object.phobos_rename_custom_property"
    bl_label = "Edit custom property of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    find = StringProperty(
        name="find property name:",
        default='',
        description="name to be searched for")

    replace = StringProperty(
        name="replacement name:",
        default='',
        description="new name to be replaced with")

    overwrite = BoolProperty(
        name='overwrite existing properties',
        default=False,
        description="If a property of the specified replacement name exists, overwrite it?"
    )

    def execute(self, context):
        startLog(self)
        for obj in context.selected_objects:
            if self.find in obj and self.replace != '':
                if self.replace in obj:
                    # print("### Error: property", self.replace, "already present in object", obj.name)
                    log("Property '" + self.replace + "' already present in object '" + obj.name + "'", "ERROR")
                    if self.overwrite:
                        log("Replace property, because overwrite option was set")
                        obj[self.replace] = obj[self.find]
                        del obj[self.find]
                else:
                    obj[self.replace] = obj[self.find]
                    del obj[self.find]
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class SetGeometryType(Operator):
    """Set Geometry Type Operator

    """
    bl_idname = "object.phobos_set_geometry_type"
    bl_label = "Edit geometry type of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    geomType = EnumProperty(
        items=defs.geometrytypes,
        name="type",
        default="box",
        description="Phobos geometry type")

    def execute(self, context):
        startLog(self)
        for obj in bpy.context.selected_objects:
            if obj.phobostype == 'collision' or obj.phobostype == 'visual':
                obj['geometry/type'] = self.geomType
            else:
                log("The object '" + obj.name + "' is no collision or visual")
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class EditInertia(Operator):
    """Edit Inertia Operator

    """
    bl_idname = "object.phobos_edit_inertia"
    bl_label = "Edit inertia of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    # inertiamatrix = FloatVectorProperty (
    # name = "inertia",
    # default = [0, 0, 0, 0, 0, 0, 0, 0, 0],
    # subtype = 'MATRIX',
    #        size = 9,
    #        description = "set inertia for a link")

    inertiavector = FloatVectorProperty(
        name="inertiavec",
        default=[0, 0, 0, 0, 0, 0],
        subtype='NONE',
        size=6,
        description="set inertia for a link"
    )

    def invoke(self, context, event):
        if 'inertia' in context.active_object:
            self.inertiavector = mathutils.Vector(context.active_object['inertia'])
        return self.execute(context)

    def execute(self, context):
        #m = self.inertiamatrix
        #inertialist = []#[m[0], m[1], m[2], m[4], m[5], m[8]]
        #obj['inertia'] = ' '.join(inertialist)
        for obj in context.selected_objects:
            if obj.phobostype == 'inertial':
                obj['inertia'] = self.inertiavector  #' '.join([str(i) for i in self.inertiavector])
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and ob.phobostype == 'inertial' and len(
            context.selected_objects) > 0


class SmoothenSurfaceOperator(Operator):
    """SmoothenSurfaceOperator

    """
    bl_idname = "object.phobos_smoothen_surface"
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
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all()
            bpy.ops.mesh.normals_make_consistent()
            bpy.ops.object.mode_set(mode='OBJECT')
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
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class SetOriginToCOMOperator(Operator):
    """SetOriginToCOMOperator

    """
    bl_idname = "object.phobos_set_origin_to_com"
    bl_label = "Set Origin to COM"
    bl_options = {'REGISTER', 'UNDO'}

    com_shift = FloatVectorProperty(
        name="CAD origin shift",
        default=(0.0, 0.0, 0.0,),
        subtype='TRANSLATION',
        unit='LENGTH',
        size=3,
        precision=6,
        description="distance between objects")

    cursor_location = FloatVectorProperty(
        name="CAD origin",
        default=(0.0, 0.0, 0.0,),
        subtype='TRANSLATION',
        unit='LENGTH',
        size=3,
        precision=6,
        description="distance between objects")

    def execute(self, context):
        master = context.active_object
        slaves = context.selected_objects
        to_cadorigin = self.cursor_location - master.matrix_world.to_translation()
        com_shift_world = to_cadorigin + self.com_shift
        for s in slaves:
            utility.selectObjects([s], True, 0)
            context.scene.cursor_location = s.matrix_world.to_translation() + com_shift_world
            bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
        utility.selectObjects(slaves, True, slaves.index(master))
        context.scene.cursor_location = self.cursor_location.copy()
        return {'FINISHED'}

    def invoke(self, context, event):
        self.cursor_location = context.scene.cursor_location.copy()
        return self.execute(context)

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class CreateInertialOperator(Operator):
    """CreateInertialOperator

    """
    bl_idname = "object.create_inertial_objects"
    bl_label = "Creates inertial objects based on existing objects"
    bl_options = {'REGISTER', 'UNDO'}

    auto_compute = BoolProperty(
        name='calculate automatically',
        default=True,
        description='calculate inertia automatically'
    )

    preserve_children = BoolProperty(
        name='preserve child inertials',
        default=False,
        description='preserve child inertials'
    )

    def execute(self, context):
        links = [obj for obj in bpy.context.selected_objects if obj.phobostype == 'link']
        show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269
        if show_progress:
            wm = bpy.context.window_manager
            total = float(len(links))
            wm.progress_begin(0, total)
            i = 1
        for link in links:
            inertia.createInertials(link, not self.auto_compute, self.preserve_children)
            if show_progress:
                wm.progress_update(i)
                i += 1
        if show_progress:
            wm.progress_end()
        return {'FINISHED'}


class AddGravityVector(Operator):
    """Add Gravity Operator

    """
    bl_idname = "object.phobos_add_gravity"
    bl_label = "Add a vector representing gravity in the scene"
    bl_options = {'REGISTER', 'UNDO'}

    property_name = FloatVectorProperty(
        name="gravity_vector",
        default=(0, 0, -9.81),
        description="gravity vector")

    def execute(self, context):
        bpy.ops.object.empty_add(type='SINGLE_ARROW')
        bpy.context.active_object.name = "gravity"
        bpy.ops.transform.rotate(value=(math.pi), axis=(1.0, 0.0, 0.0))
        return {'FINISHED'}


class EditYAMLDictionary(Operator):
    """Edit a dictionary as YAML in text file."""
    bl_idname = 'object.phobos_edityamldictionary'
    bl_label = "Edit object dictionary as YAML"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        ob = context.active_object
        textfilename = ob.name + dt.now().strftime("%Y%m%d_%H:%M")
        variablename = ob.name.translate({ord(c): "_" for c in "!@#$%^&*()[]{};:,./<>?\|`~-=+"}) \
                       + "_data"
        tmpdict = dict(ob.items())
        for key in tmpdict:
            if hasattr(tmpdict[key], 'to_list'):  # transform Blender id_arrays into lists
                tmpdict[key] = list(tmpdict[key])
        contents = [variablename + ' = """',
                    yaml.dump(utility.cleanObjectProperties(tmpdict),
                              default_flow_style=False) + '"""\n',
                    "# ------- Hit 'Run Script' to save your changes --------",
                    "import yaml", "import bpy",
                    "tmpdata = yaml.load(" + variablename + ")",
                    "for key, value in tmpdata.items():",
                    "    bpy.context.active_object[key] = value",
                    "bpy.ops.text.unlink()"
                    ]
        utility.createNewTextfile(textfilename, '\n'.join(contents))
        utility.openScriptInEditor(textfilename)
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class CreateCollisionObjects(Operator):
    """Select n bodies to create collision objects for.

    """
    bl_idname = "object.create_collision_objects"
    bl_label = "Create collision objects for all selected Links"
    bl_options = {'REGISTER', 'UNDO'}

    property_colltype = EnumProperty(
        name='coll_type',
        default='box',
        description="collision type",
        items=defs.geometrytypes)

    def execute(self, context):
        """Executes this blender operator and creates collision objects for the selected bodies.

        :param context: The blender context this function should work with.

        :return: set -- the blender specific return set.

        """

        startLog(self)
        visuals = []
        for obj in bpy.context.selected_objects:
            if obj.phobostype == "visual":
                visuals.append(obj)
            obj.select = False

        if not visuals:
            # bpy.ops.error.message('INVOKE_DEFAULT', type="CreateCollisions Error", message="Not enough bodies selected.")
            log("Not enough bodies selected.", "ERROR")
            return {'CANCELLED'}
        for vis in visuals:
            nameparts = vis.name.split('_')
            if nameparts[0] == 'visual':
                nameparts[0] = 'collision'
            collname = '_'.join(nameparts)
            materialname = vis.data.materials[0].name if len(vis.data.materials) > 0 else "None"
            bBox = vis.bound_box
            center = utility.calcBoundingBoxCenter(bBox)
            rotation = Matrix.Identity(4)
            size = list(vis.dimensions)
            if self.property_colltype in ['cylinder', 'capsule']:
                axes = ('X', 'Y', 'Z')
                long_side = axes[size.index(max(size))]
                # xyequal = (size[0] - size[1])
                length = max(size)
                radii = [s for s in size if s != length]
                radius = max(radii) / 2 if radii != [] else length / 2
                size = (radius, length)
                if long_side == 'X':
                    rotation = Matrix.Rotation(math.pi / 2, 4, 'Y')
                elif long_side == 'Y':
                    rotation = Matrix.Rotation(math.pi / 2, 4, 'X')
                    # FIXME: apply rotation for moved cylinder object?
            elif self.property_colltype == 'sphere':
                size = max(size) / 2
            rotation_euler = (vis.matrix_world * rotation).to_euler()
            center = vis.matrix_world.to_translation() + vis.matrix_world.to_quaternion() * center
            if self.property_colltype != 'capsule':
                ob = utility.createPrimitive(collname, self.property_colltype, size,
                                             defs.layerTypes['collision'], materialname, center,
                                             rotation_euler)
            elif self.property_colltype == 'capsule':
                length = max(length - 2 * radius, 0.001)  # prevent length from turning negative
                size = (radius, length)
                zshift = length / 2
                ob = utility.createPrimitive(collname, 'cylinder', size,
                                             defs.layerTypes['collision'], materialname, center,
                                             rotation_euler)
                sph1 = utility.createPrimitive('tmpsph1', 'sphere', radius,
                                               defs.layerTypes['collision'], materialname,
                                               center + rotation * Vector((0, 0, zshift)),
                                               rotation_euler)
                sph2 = utility.createPrimitive('tmpsph2', 'sphere', radius,
                                               defs.layerTypes['collision'], materialname,
                                               center - rotation * Vector((0, 0, zshift)),
                                               rotation_euler)
                utility.selectObjects([ob, sph1, sph2], True, 0)
                bpy.ops.object.join()
                ob['length'] = length
                ob['radius'] = radius
            elif self.property_colltype == 'mesh':
                pass
                # TODO: copy mesh!!
            ob.phobostype = 'collision'
            ob['geometry/type'] = self.property_colltype
            if vis.parent:
                ob.select = True
                bpy.ops.object.transform_apply(scale=True)
                vis.parent.select = True
                bpy.context.scene.objects.active = vis.parent
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
                # ob.parent_type = vis.parent_type
                # ob.parent_bone = vis.parent_bone
        endLog()
        return {'FINISHED'}


class SetCollisionGroupOperator(Operator):
    """Sets the collision group of the selected object(s).

    """
    bl_idname = "object.phobos_set_collision_group"
    bl_label = "Sets the collision group of the selected object(s)."
    bl_options = {'REGISTER', 'UNDO'}

    groups = BoolVectorProperty(
        name='collision groups',
        size=20,
        subtype='LAYER',
        default=(False,) * 20,
        description='collision groups')

    @classmethod
    def poll(self, context):
        """This function checks if the context is valid in terms of executing this operator.

        :param context: The blender context to check.
        :return: bool -- Whether the context is valid or not.

        """
        for obj in context.selected_objects:
            if obj.phobostype == 'collision':
                return True
        return False

    def invoke(self, context, event):
        """This function invokes this operator.

        :param context: The blender context this operator should work with.
        :param event: Blender specific event *not used in this function*
        :return: set -- the blender specific return set.

        """
        try:
            self.groups = context.active_object.rigid_body.collision_groups
        except AttributeError:
            pass  # TODO: catch properly
        return self.execute(context)

    def execute(self, context):
        """This function executes this blender operator and sets the collision groups for the selected object(s).

        :param context: The blender context this operator should work with.
        :return: set -- the blender specific return set.

        """
        active_object = context.active_object
        for obj in context.selected_objects:
            if obj.phobostype == 'collision':
                try:
                    obj.rigid_body.collision_groups = self.groups
                except AttributeError:
                    context.scene.objects.active = obj
                    bpy.ops.rigidbody.object_add(type='ACTIVE')
                    obj.rigid_body.kinematic = True
                    obj.rigid_body.collision_groups = self.groups
        context.scene.objects.active = active_object
        return {'FINISHED'}


class DefineJointConstraintsOperator(Operator):
    """DefineJointConstraintsOperator

    """
    bl_idname = "object.define_joint_constraints"
    bl_label = "Adds Bone Constraints to the joint (link)"
    bl_options = {'REGISTER', 'UNDO'}

    passive = BoolProperty(
        name='passive',
        default=False,
        description='makes the joint passive (no actuation)'
    )

    useRadian = BoolProperty(
        name='useRadian',
        default=True,
        description='use degrees or rad for joints'
    )

    joint_type = EnumProperty(
        name='joint_type',
        default='revolute',
        description="type of the joint",
        items=defs.jointtypes)

    lower = FloatProperty(
        name="lower",
        default=0.0,
        description="lower constraint of the joint")

    upper = FloatProperty(
        name="upper",
        default=0.0,
        description="upper constraint of the joint")

    maxeffort = FloatProperty(
        name="max effort (N or Nm)",
        default=0.0,
        description="maximum effort of the joint")

    maxvelocity = FloatProperty(
        name="max velocity (m/s or rad/s)",
        default=0.0,
        description="maximum velocity of the joint. If you uncheck radian, you can enter °/sec here")

    spring = FloatProperty(
        name="spring constant",
        default=0.0,
        description="spring constant of the joint")

    damping = FloatProperty(
        name="damping constant",
        default=0.0,
        description="damping constant of the joint")

    # TODO: invoke function to read all values in

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "joint_type", text="joint_type")
        layout.prop(self, "passive", text="makes the joint passive (no actuation)")
        layout.prop(self, "useRadian", text="use radian")
        if self.joint_type != 'fixed':
            layout.prop(self, "maxeffort",
                        text="max effort [" + ('Nm]' if self.joint_type in ['revolute', 'continuous'] else 'N]'))
            if self.joint_type in ['revolute', 'continuous']:
                layout.prop(self, "maxvelocity", text="max velocity [" + ("rad/s]" if self.useRadian else "°/s]"))
            else:
                layout.prop(self, "maxvelocity", text="max velocity [m/s]")
        if self.joint_type in ('revolute', 'prismatic'):
            layout.prop(self, "lower", text="lower [rad]" if self.useRadian else "lower [°]")
            layout.prop(self, "upper", text="upper [rad]" if self.useRadian else "upper [°]")
            layout.prop(self, "spring", text="spring constant [N/m]")
            layout.prop(self, "damping", text="damping constant")


    def invoke(self, context, event):
        aObject = context.active_object
        if 'joint/type' not in aObject and 'motor/type' in aObject:
            self.maxvelocity = aObject['motor/maxSpeed']
            self.maxeffort = aObject['motor/maxEffort']
        return self.execute(context)

    def execute(self, context):
        """This function executes this operator and sets the constraints and joint type for all selected links.
        rad/s is the default unit. rpm will be transformed into rad/s

        :param context: The blender context this operator works with.
        :return: Blender result.

        """
        lower = 0
        upper = 0
        velocity = 0
        if self.joint_type in ('revolute', 'prismatic'):
            if not self.useRadian:
                lower = math.radians(self.lower)
                upper = math.radians(self.upper)
            else:
                lower = self.lower
                upper = self.upper
        if not self.useRadian:
            velocity = self.maxvelocity * ((2 * math.pi) / 360)  # from °/s to rad/s
        else:
            velocity = self.maxvelocity
        for link in context.selected_objects:
            bpy.context.scene.objects.active = link
            setJointConstraints(link, self.joint_type, lower, upper, self.spring, self.damping)
            if self.joint_type != 'fixed':
                link['joint/maxeffort'] = self.maxeffort
                link['joint/maxvelocity'] = velocity
            else:
                if "joint/maxeffort" in link: del link["joint/maxeffort"]
                if "joint/maxvelocity" in link: del link["joint/maxvelocity"]
            if self.passive:
                link['joint/passive'] = "$true"
            else:
                pass  # FIXME: add default motor here or upon export?
                # upon export might have the advantage of being able
                # to check for missing motors in the model checking
        return {'FINISHED'}


class AttachMotorOperator(Operator):
    """AttachMotorOperator

    """
    bl_idname = "object.attach_motor"
    bl_label = "Attaches motor values to selected joints"
    bl_options = {'REGISTER', 'UNDO'}

    P = FloatProperty(
        name="P",
        default=1.0,
        description="P-value")

    I = FloatProperty(
        name="I",
        default=0.0,
        description="I-value")

    D = FloatProperty(
        name="D",
        default=0.0,
        description="D-value")

    vmax = FloatProperty(
        name="maximum velocity [m/s] or [rad/s]",
        default=1.0,
        description="maximum turning velocity of the motor")

    taumax = FloatProperty(
        name="maximum torque [Nm]",
        default=1.0,
        description="maximum torque a motor can apply")

    motortype = EnumProperty(
        name='motor_type',
        default='PID',
        description="type of the motor",
        items=defs.motortypes)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "motortype", text="motor_type")
        layout.prop(self, "taumax", text="maximum torque [Nm]")
        layout.prop(self, "vmax", text="maximum velocity [m/s] or [rad/s]")
        if self.motortype == 'PID':
            layout.prop(self, "P", text="P")
            layout.prop(self, "I", text="I")
            layout.prop(self, "D", text="D")

    def invoke(self, context, event):
        aObject = context.active_object
        if 'motor/type' not in aObject and 'joint/type' in aObject and aObject['joint/type'] != 'fixed':
            self.taumax = aObject['joint/maxeffort']
            self.vmax = aObject['joint/maxvelocity']
        return self.execute(context)

    def execute(self, context):
        """This function executes this operator and attaches a motor to all selected links.

        :param context: The blender context this operator works with.
        :return:Blender result.

        """
        for joint in bpy.context.selected_objects:
            if joint.phobostype == "link":
                # TODO: these keys have to be adapted
                if self.motortype == 'PID':
                    joint['motor/p'] = self.P
                    joint['motor/i'] = self.I
                    joint['motor/d'] = self.D
                joint['motor/maxSpeed'] = self.vmax
                joint['motor/maxEffort'] = self.taumax
                #joint['motor/type'] = 'PID' if self.motortype == 'PID' else 'DC'
                joint['motor/type'] = self.motortype
        return {'FINISHED'}


class CreateLinkOperator(Operator):
    """Create Link Operator

    """
    bl_idname = "object.phobos_create_link"
    bl_label = "Create link(s), optionally based on existing objects"
    bl_options = {'REGISTER', 'UNDO'}

    type = EnumProperty(
        items=(('3D cursor',) * 3,
               ('selected objects',) * 3),
        default='selected objects',
        name='location',
        description='Where to create new link(s)?'
    )

    size = FloatProperty(
        name="visual link size",
        default=0.2,
        description="size of the created link"
    )

    parenting = BoolProperty(
        name='parenting',
        default=False,
        description='parent associated objects to created links?'
    )

    parentobject = BoolProperty(
        name='parent object(s)',
        default=False,
        description='parent objects to newly created links?'
    )

    namepartindices = StringProperty(
        name="name segment indices",
        description="allows reusing parts of objects' names, specified as e.g. '2 3'",
        default=''
    )

    separator = StringProperty(
        name="separator",
        description="seperator to split object names with, e.g. '_'",
        default='_'
    )

    prefix = StringProperty(
        name="prefix",
        description="prefix to put before names, e.g. 'link'",
        default='link'
    )

    def execute(self, context):
        """This function executes the operator and creates a link.

        :param context: The blender context this operator works with.
        :type context: Blender context.
        :return: Blender result.
        """
        if self.type == '3D cursor':
            createLink(self.size)
        else:
            for obj in bpy.context.selected_objects:
                tmpnamepartindices = [int(p) for p in self.namepartindices.split()]
                deriveLinkfromObject(obj, scale=self.size, parenting=self.parenting, parentobjects=self.parentobject,
                                     namepartindices=tmpnamepartindices, separator=self.separator,
                                     prefix=self.prefix)
        return {'FINISHED'}


class AddSensorOperator(Operator):
    """AddSensorOperator"""
    bl_idname = "object.phobos_add_sensor"
    bl_label = "Add/Update a sensor"
    bl_options = {'REGISTER', 'UNDO'}

    sensor_type = EnumProperty(
        name="sensor_type",
        default="CameraSensor",
        items=tuple([(type,) * 3 for type in defs.sensortypes]),
        description="tyoe of the sensor to be created"
    )

    custom_type = StringProperty(
        name="custom_type",
        default='',
        description="type of the custom sensor to be created"
    )

    sensor_name = StringProperty(
        name="sensor_name",
        default='new_sensor',
        description="name of the sensor"
    )

    add_link = BoolProperty(name="add_link", default=True, description="add additional link as sensor mounting")

    # the following is a set of all properties that exist within MARS' sensors
    # TODO: we should get rid of gui-settings such as hud and rename stuff (eg. maxDistance - maxDist)
    width = IntProperty(name='width', default=0, description='width')
    height = IntProperty(name='height', default=0, description='height')
    resolution = FloatProperty(name='resolution', default=0, description='resolution')
    horizontal_resolution = FloatProperty(name='horizontal_resolution', default=0, description='horizontal_resolution')
    opening_width = FloatProperty(name='opening_width', default=0, description='opening_width')
    opening_height = FloatProperty(name='opening_height', default=0, description='opening_height')
    maxDistance = FloatProperty(name='maxDistance', default=0, description='maxDistance')
    maxDist = FloatProperty(name='maxDist', default=0, description='maxDist')
    verticalOpeningAngle = FloatProperty(name='verticalOpeningAngle', default=0, description='verticalOpeningAngle')
    horizontalOpeningAngle = FloatProperty(name='horizontalOpeningAngle', default=0,
                                           description='horizontalOpeningAngle')
    hud_pos = IntProperty(name='hud_pos', default=0, description='hud_pos')
    hud_width = IntProperty(name='hud_width', default=0, description='hud_width')
    hud_height = IntProperty(name='hud_height', default=0, description='hud_height')
    updateRate = FloatProperty(name='updateRate', default=0, description='updateRate')
    vertical_offset = FloatProperty(name='vertical_offset', default=0, description='vertical_offset')
    horizontal_offset = FloatProperty(name='horizontal_offset', default=0, description='horizontal_offset')
    gain = FloatProperty(name='gain', default=0, description='gain')
    left_limit = FloatProperty(name='left_limit', default=0, description='left_limit')
    right_limit = FloatProperty(name='right_limit', default=0, description='right_limit')
    rttResolutionX = FloatProperty(name='rttResolutionX', default=0, description='rttResolutionX')
    rttResolutionY = FloatProperty(name='rttResolutionY', default=0, description='rttResolutionY')
    numRaysHorizontal = FloatProperty(name='numRaysHorizontal', default=0, description='numRaysHorizontal')
    numRaysVertical = FloatProperty(name='numRaysVertical', default=0, description='numRaysVertical')
    draw_rays = BoolProperty(name='draw_rays', default=False, description='draw_rays')
    depthImage = BoolProperty(name='depthImage', default=False, description='depthImage')
    show_cam = BoolProperty(name='show_cam', default=False, description='show_cam')
    only_ray = BoolProperty(name='only_ray', default=False, description='only_ray')
    ping_pong_mode = BoolProperty(name='ping_pong_mode', default=False, description='ping_pong_mode')
    bands = IntProperty(name='bands', default=0, description='bands')
    lasers = IntProperty(name='lasers', default=0, description='lasers')
    extension = FloatVectorProperty(name='extension', default=(0, 0, 0), description='extension')

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "sensor_name", text="name of the sensor")
        layout.prop(self, "sensor_type", text="sensor type")
        if self.sensor_type in ['CameraSensor', 'ScanningSonar', 'RaySensor',
                                'MultiLevelLaserRangeFinder', 'RotatingRaySensor']:
            layout.prop(self, "add_link", "attach sensor to new link?")
        if self.sensor_type == "custom":
            layout.prop(self, "custom_type", text="enter custom type")
        else:
            for key in defs.sensorProperties[self.sensor_type]:
                layout.prop(self, key, text=key)

    def execute(self, context):
        # create a dictionary holding the sensor definition
        sensor = {'name': self.sensor_name,
                  'type': self.custom_type if self.sensor_type == 'Custom' else self.sensor_type,
                  'props': {}
                  }
        parent = context.active_object
        for key in defs.sensorProperties[self.sensor_type]:
            if type(defs.sensorProperties[self.sensor_type][key]) == type(True):
                value = getattr(self, key)
                sensor['props'][key] = '$true' if value else '$false'
            else:
                sensor['props'][key] = getattr(self, key)
        # type-specific settings
        if sensor['type'] in ['CameraSensor', 'ScanningSonar', 'RaySensor',
                              'MultiLevelLaserRangeFinder', 'RotatingRaySensor']:
            if self.add_link:
                link = links.createLink(scale=0.1, position=context.active_object.matrix_world.to_translation(),
                                        name='link_' + self.sensor_name)
                sensorObj = createSensor(sensor, link.name, link.matrix_world)
            else:
                sensorObj = createSensor(sensor, context.active_object.name, context.active_object.matrix_world)
            if self.add_link:
                utility.selectObjects([parent, link], clear=True, active=0)
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
                utility.selectObjects([link, sensorObj], clear=True, active=0)
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
            cameraRotLock(sensorObj)
        elif sensor['type'] in ['Joint6DOF']:
            createSensor(sensor, context.active_object, context.active_object.matrix_world)
        elif 'Node' in sensor['type']:
            createSensor(sensor, [obj for obj in context.selected_objects if obj.phobostype == 'collision'],
                         mathutils.Matrix.Translation(context.scene.cursor_location))
        elif 'Motor' in sensor['type'] or 'Joint' in sensor['type']:
            createSensor(sensor, [obj for obj in context.selected_objects if obj.phobostype == 'link'],
                         mathutils.Matrix.Translation(context.scene.cursor_location))
        return {'FINISHED'}