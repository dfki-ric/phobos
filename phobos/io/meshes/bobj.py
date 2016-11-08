#!/usr/bin/python
# coding=utf-8

"""
Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this software.  If not, see <http://www.gnu.org/licenses/>.

File bobj.py

Created on 6 Jan 2014

@author: Malte Langosz, Kai von Szadkowski
"""


import struct
import os
import bpy
import mathutils
from bpy_extras.io_utils import unpack_list, unpack_face_list

def roundVector(v, n):
    """Returns a mathutils.Vector with its components rounded to n digits.

    """
    return round(v.x, n), round(v.y, n), round(v.z, n)

def exportBobj(path, obj):
    """This function exports an object to the specified path as a .bobj

    :param path: The path to export the object to. *without filename!*
    :type path: str
    :param obj: The blender object you want to export.
    :type: bpy.types.Object

    """
    bpy.ops.object.select_all(action='DESELECT')
    obj.select = True
    bpy.context.scene.objects.active = obj
    #TODO: make this exception-handled
    totverts = totuvco = totno = 1

    globalNormals = {}

    # ignore dupli children
    if obj.parent and obj.parent.dupli_type in {'VERTS', 'FACES'}:
        print(obj.name, 'is a dupli child - ignoring')
        return

    mesh = obj.to_mesh(bpy.context.scene, True, 'PREVIEW')
    #mesh.transform(obj.matrix_world)

    faceuv = len(mesh.uv_textures)
    if faceuv:
        uv_layer = mesh.uv_textures.active.data[:]

    if bpy.app.version[0] * 100 + bpy.app.version[1] >= 265:
        face_index_pairs = [(face, index) for index, face in enumerate(mesh.tessfaces)]
    else:
        face_index_pairs = [(face, index) for index, face in enumerate(mesh.faces)]

    mesh.calc_normals()

    me_verts = mesh.vertices[:]

    out = open(os.path.join(path, obj.data.name + "." + 'bobj'), "wb")

    for v in mesh.vertices:
        out.write(struct.pack('ifff', 1, v.co[0], v.co[1], v.co[2]))

    if faceuv:
        uv = uvkey = f_index = uv_index = None

        #uv_face_mapping = [[0, 0, 0, 0]] * len(face_index_pairs)  # a bit of a waste for tri's :/
        uv_face_mapping = [[0, 0, 0, 0] for i in range(len(face_index_pairs))]

        uv_dict = {}  # could use a set() here
        if bpy.app.version[1] >= 65:
            uv_layer = mesh.tessface_uv_textures.active.data[:]
        else:
            uv_layer = mesh.uv_textures.active.data
        for f, f_index in face_index_pairs:
            for uv_index, uv in enumerate(uv_layer[f_index].uv):
                uvkey = round(uv[0], 6), round(uv[1], 6)
                try:
                    uv_face_mapping[f_index][uv_index] = uv_dict[uvkey]
                except:  # TODO: what can really go wrong here?
                    uv_face_mapping[f_index][uv_index] = uv_dict[uvkey] = len(uv_dict)
                    out.write(struct.pack('iff', 2, uv[0], uv[1]))

        del uv, uvkey, uv_dict, f_index, uv_index

    for f, f_index in face_index_pairs:
        if f.use_smooth:
            for v_idx in f.vertices:
                v = me_verts[v_idx]
                noKey = roundVector(v.normal, 6)
                if noKey not in globalNormals:
                    globalNormals[noKey] = totno
                    totno += 1
                    out.write(struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2]))
        else:
            # Hard, 1 normal from the face.
            noKey = roundVector(f.normal, 6)
            if noKey not in globalNormals:
                globalNormals[noKey] = totno
                totno += 1
                out.write(struct.pack('ifff', 3, noKey[0], noKey[1], noKey[2]))

    for f, f_index in face_index_pairs:
        f_smooth = f.use_smooth
        # write smooth info for face?

        f_v_orig = [(vi, me_verts[v_idx]) for vi, v_idx in enumerate(f.vertices)]

        if len(f_v_orig) == 3:
            f_v_iter = (f_v_orig, )
        else:
            f_v_iter = (f_v_orig[0], f_v_orig[1], f_v_orig[2]), (f_v_orig[0], f_v_orig[2], f_v_orig[3])

        for f_v in f_v_iter:
            da = struct.pack('i', 4)
            out.write(da)

            if faceuv:
                if f_smooth:  # Smoothed, use vertex normals
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi],
                                              globalNormals[roundVector(v.normal, 6)]))
                else:  # No smoothing, face normals
                    no = globalNormals[roundVector(f.normal, 6)]
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, totuvco + uv_face_mapping[f_index][vi], no))
            else:  # No UV's
                if f_smooth:  # Smoothed, use vertex normals
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, 0, globalNormals[roundVector(v.normal, 6)]))
                else:  # No smoothing, face normals
                    no = globalNormals[roundVector(f.normal, 6)]
                    for vi, v in f_v:
                        out.write(struct.pack('iii', v.index + totverts, 0, no))
    out.close()


def get_fmt_sizes():
    for fmt in ['ifff', 'iff', 'i', 'iii']:
        print(fmt, struct.calcsize(fmt))


def line_value(line_split):     # copy from blender addons; should be imported from there
    """
    Returns 1 string represneting the value for this line
    None will be returned if theres only 1 word
    """
    length = len(line_split)
    if length == 1:
        return None

    elif length == 2:
        return line_split[1]

    elif length > 2:
        return b' '.join(line_split[1:])


def create_materials(filepath, relpath,                                             # copy from blender addons; should be imported from there
                     material_libs, unique_materials, unique_material_images,
                     use_image_search, float_func):
    """
    Create all the used materials in this obj,
    assign colors and images to the materials from all referenced material libs
    """
    DIR = os.path.dirname(filepath)
    context_material_vars = set()

    def load_material_image(blender_material, context_material_name, imagepath, type):
        """Sets textures defined in .mtl file
        """

        texture = bpy.data.textures.new(name=type, type='IMAGE')

        # Absolute path - c:\.. etc would work here
        image = obj_image_load(imagepath, DIR, use_image_search, relpath)

        if image is not None:
            texture.image = image

        # Adds textures for materials (rendering)
        if type == 'Kd':
            mtex = blender_material.texture_slots.add()
            mtex.texture = texture
            mtex.texture_coords = 'UV'
            mtex.use_map_color_diffuse = True

            # adds textures to faces (Textured/Alt-Z mode)
            # Only apply the diffuse texture to the face if the image has not been set with the inline usemat func.
            unique_material_images[context_material_name] = image  # set the texface image

        elif type == 'Ka':
            mtex = blender_material.texture_slots.add()
            mtex.use_map_color_diffuse = False

            mtex.texture = texture
            mtex.texture_coords = 'UV'
            mtex.use_map_ambient = True

        elif type == 'Ks':
            mtex = blender_material.texture_slots.add()
            mtex.use_map_color_diffuse = False

            mtex.texture = texture
            mtex.texture_coords = 'UV'
            mtex.use_map_specular = True

        elif type == 'Bump':
            mtex = blender_material.texture_slots.add()
            mtex.use_map_color_diffuse = False

            mtex.texture = texture
            mtex.texture_coords = 'UV'
            mtex.use_map_normal = True

        elif type == 'D':
            mtex = blender_material.texture_slots.add()
            mtex.use_map_color_diffuse = False

            mtex.texture = texture
            mtex.texture_coords = 'UV'
            mtex.use_map_alpha = True
            blender_material.use_transparency = True
            blender_material.transparency_method = 'Z_TRANSPARENCY'
            if "alpha" not in context_material_vars:
                blender_material.alpha = 0.0
            # Todo, unset deffuse material alpha if it has an alpha channel

        elif type == 'disp':
            mtex = blender_material.texture_slots.add()
            mtex.use_map_color_diffuse = False

            mtex.texture = texture
            mtex.texture_coords = 'UV'
            mtex.use_map_displacement = True

        elif type == 'refl':
            mtex = blender_material.texture_slots.add()
            mtex.use_map_color_diffuse = False

            mtex.texture = texture
            mtex.texture_coords = 'REFLECTION'
            mtex.use_map_color_diffuse = True
        else:
            raise Exception("invalid type %r" % type)

    # Add an MTL with the same name as the obj if no MTLs are spesified.
    temp_mtl = os.path.splitext((os.path.basename(filepath)))[0] + b'.mtl'

    if os.path.exists(os.path.join(DIR, temp_mtl)) and temp_mtl not in material_libs:
        material_libs.append(temp_mtl)
    del temp_mtl

    # Create new materials
    for name in unique_materials:
        if name is not None:
            unique_materials[name] = bpy.data.materials.new(name.decode('utf-8', "replace"))
            unique_material_images[name] = None  # assign None to all material images to start with, add to later.

    unique_materials[None] = None
    unique_material_images[None] = None

    for libname in material_libs:
        # print(libname)
        mtlpath = os.path.join(DIR, libname)
        if not os.path.exists(mtlpath):
            print ("\tMaterial not found MTL: %r" % mtlpath)
        else:
            context_material = None
            mtl = open(mtlpath, 'rb')
            for line in mtl:  # .readlines():
                line = line.strip()
                if not line or line.startswith(b'#'):
                    pass
                elif line.startswith(b'newmtl'):
                    context_material_name = line_value(line.split())
                    context_material = unique_materials.get(context_material_name)
                    context_material_vars.clear()

                elif context_material:
                    # we need to make a material to assign properties to it.
                    line_split = line.split()
                    line_lower = line.lower().lstrip()
                    if line_lower.startswith(b'ka'):
                        context_material.mirror_color = float_func(line_split[1]), float_func(line_split[2]), float_func(line_split[3])
                    elif line_lower.startswith(b'kd'):
                        context_material.diffuse_color = float_func(line_split[1]), float_func(line_split[2]), float_func(line_split[3])
                    elif line_lower.startswith(b'ks'):
                        context_material.specular_color = float_func(line_split[1]), float_func(line_split[2]), float_func(line_split[3])
                    elif line_lower.startswith(b'ns'):
                        context_material.specular_hardness = int((float_func(line_split[1]) * 0.51))
                    elif line_lower.startswith(b'ni'):  # Refraction index
                        context_material.raytrace_transparency.ior = max(1, min(float_func(line_split[1]), 3))  # between 1 and 3
                        context_material_vars.add("ior")
                    elif line_lower.startswith(b'd'):  # dissolve (trancparency)
                        context_material.alpha = float_func(line_split[1])
                        context_material.use_transparency = True
                        context_material.transparency_method = 'Z_TRANSPARENCY'
                        context_material_vars.add("alpha")
                    elif line_lower.startswith(b'tr'):  # trancelucency
                        context_material.translucency = float_func(line_split[1])
                    elif line_lower.startswith(b'tf'):
                        # rgb, filter color, blender has no support for this.
                        pass
                    elif line_lower.startswith(b'illum'):
                        illum = int(line_split[1])

                        do_ambient = True
                        do_highlight = False
                        do_reflection = False
                        do_transparency = False
                        do_glass = False
                        do_fresnel = False
                        do_raytrace = False

                        # inline comments are from the spec, v4.2
                        if illum == 0:
                            # Color on and Ambient off
                            do_ambient = False
                        elif illum == 1:
                            # Color on and Ambient on
                            pass
                        elif illum == 2:
                            # Highlight on
                            do_highlight = True
                        elif illum == 3:
                            # Reflection on and Ray trace on
                            do_reflection = True
                            do_raytrace = True
                        elif illum == 4:
                            # Transparency: Glass on
                            # Reflection: Ray trace on
                            do_transparency = True
                            do_reflection = True
                            do_glass = True
                            do_raytrace = True
                        elif illum == 5:
                            # Reflection: Fresnel on and Ray trace on
                            do_reflection = True
                            do_fresnel = True
                            do_raytrace = True
                        elif illum == 6:
                            # Transparency: Refraction on
                            # Reflection: Fresnel off and Ray trace on
                            do_transparency = True
                            do_reflection = True
                            do_raytrace = True
                        elif illum == 7:
                            # Transparency: Refraction on
                            # Reflection: Fresnel on and Ray trace on
                            do_transparency = True
                            do_reflection = True
                            do_fresnel = True
                            do_raytrace = True
                        elif illum == 8:
                            # Reflection on and Ray trace off
                            do_reflection = True
                        elif illum == 9:
                            # Transparency: Glass on
                            # Reflection: Ray trace off
                            do_transparency = True
                            do_reflection = True
                            do_glass = True
                        elif illum == 10:
                            # Casts shadows onto invisible surfaces

                            # blender cant do this
                            pass

                        if do_ambient:
                            context_material.ambient = 1.0
                        else:
                            context_material.ambient = 0.0

                        if do_highlight:
                            # FIXME, how else to use this?
                            context_material.specular_intensity = 1.0

                        if do_reflection:
                            context_material.raytrace_mirror.use = True
                            context_material.raytrace_mirror.reflect_factor = 1.0

                        if do_transparency:
                            context_material.use_transparency = True
                            context_material.transparency_method = 'RAYTRACE' if do_raytrace else 'Z_TRANSPARENCY'
                            if "alpha" not in context_material_vars:
                                context_material.alpha = 0.0

                        if do_glass:
                            if "ior" not in context_material_vars:
                                context_material.raytrace_transparency.ior = 1.5

                        if do_fresnel:
                            context_material.raytrace_mirror.fresnel = 1.0  # could be any value for 'ON'

                        """
                        if do_raytrace:
                            context_material.use_raytrace = True
                        else:
                            context_material.use_raytrace = False
                        """
                        # XXX, this is not following the OBJ spec, but this was
                        # written when raytracing wasnt default, annoying to disable for blender users.
                        context_material.use_raytrace = True

                    elif line_lower.startswith(b'map_ka'):
                        img_filepath = line_value(line.split())
                        if img_filepath:
                            load_material_image(context_material, context_material_name, img_filepath, 'Ka')
                    elif line_lower.startswith(b'map_ks'):
                        img_filepath = line_value(line.split())
                        if img_filepath:
                            load_material_image(context_material, context_material_name, img_filepath, 'Ks')
                    elif line_lower.startswith(b'map_kd'):
                        img_filepath = line_value(line.split())
                        if img_filepath:
                            load_material_image(context_material, context_material_name, img_filepath, 'Kd')
                    elif line_lower.startswith((b'map_bump', b'bump')):  # 'bump' is incorrect but some files use it.
                        img_filepath = line_value(line.split())
                        if img_filepath:
                            load_material_image(context_material, context_material_name, img_filepath, 'Bump')
                    elif line_lower.startswith((b'map_d', b'map_tr')):  # Alpha map - Dissolve
                        img_filepath = line_value(line.split())
                        if img_filepath:
                            load_material_image(context_material, context_material_name, img_filepath, 'D')

                    elif line_lower.startswith((b'map_disp', b'disp')):  # reflectionmap
                        img_filepath = line_value(line.split())
                        if img_filepath:
                            load_material_image(context_material, context_material_name, img_filepath, 'disp')

                    elif line_lower.startswith((b'map_refl', b'refl')):  # reflectionmap
                        img_filepath = line_value(line.split())
                        if img_filepath:
                            load_material_image(context_material, context_material_name, img_filepath, 'refl')
                    else:
                        print("\t%r:%r (ignored)" % (filepath, line))
            mtl.close()

def split_mesh(verts_loc, faces, unique_materials, filepath, SPLIT_OB_OR_GROUP):    # copy from blender addons; should be imported from there
    """
    Takes vert_loc and faces, and separates into multiple sets of
    (verts_loc, faces, unique_materials, dataname)
    """

    filename = os.path.splitext((os.path.basename(filepath)))[0]

    if not SPLIT_OB_OR_GROUP or not faces:
        # use the filename for the object name since we aren't chopping up the mesh.
        return [(verts_loc, faces, unique_materials, filename)]

    def key_to_name(key):
        # if the key is a tuple, join it to make a string
        if not key:
            return filename  # assume its a string. make sure this is true if the splitting code is changed
        else:
            return key

    # Return a key that makes the faces unique.
    face_split_dict = {}

    oldkey = -1  # initialize to a value that will never match the key

    for face in faces:
        key = face[4]

        if oldkey != key:
            # Check the key has changed.
            try:
                verts_split, faces_split, unique_materials_split, vert_remap = face_split_dict[key]
            except KeyError:
                faces_split = []
                verts_split = []
                unique_materials_split = {}
                vert_remap = {}

                face_split_dict[key] = (verts_split, faces_split, unique_materials_split, vert_remap)

            oldkey = key

        face_vert_loc_indices = face[0]

        # Remap verts to new vert list and add where needed
        for enum, i in enumerate(face_vert_loc_indices):
            map_index = vert_remap.get(i)
            if map_index is None:
                map_index = len(verts_split)
                vert_remap[i] = map_index  # set the new remapped index so we only add once and can reference next time.
                verts_split.append(verts_loc[i])  # add the vert to the local verts

            face_vert_loc_indices[enum] = map_index  # remap to the local index

            matname = face[2]
            if matname and matname not in unique_materials_split:
                unique_materials_split[matname] = unique_materials[matname]

        faces_split.append(face)

    # remove one of the itemas and reorder
    return [(value[0], value[1], value[2], key_to_name(key)) for key, value in list(face_split_dict.items())]

def mesh_untessellate(me, fgon_edges):       # copy from blender addons; should be imported from there
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(me)
    verts = bm.verts[:]
    get = bm.edges.get
    edges = [get((verts[key[0]], verts[key[1]])) for key in fgon_edges]
    try:
        bmesh.ops.dissolve_edges(bm, edges=edges, use_verts=False)
    except:
        # Possible dissolve fails for some edges
        # but dont fail silently unless this is a real bug.
        import traceback
        traceback.print_exc()

    bm.to_mesh(me)
    bm.free()


def create_mesh(new_objects,        # copy from blender addons; should be imported from there
                has_ngons,
                use_ngons,
                use_edges,
                verts_loc,
                verts_tex,
                faces,
                unique_materials,
                unique_material_images,
                unique_smooth_groups,
                vertex_groups,
                dataname,
                ):
    """
    Takes all the data gathered and generates a mesh, adding the new object to new_objects
    deals with ngons, sharp edges and assigning materials
    """
    from bpy_extras.mesh_utils import ngon_tessellate

    if not has_ngons:
        use_ngons = False

    if unique_smooth_groups:
        sharp_edges = {}
        smooth_group_users = {context_smooth_group: {} for context_smooth_group in list(unique_smooth_groups.keys())}
        context_smooth_group_old = -1

    # Split ngons into tri's
    fgon_edges = set()  # Used for storing fgon keys
    if use_edges:
        edges = []

    context_object = None

    # reverse loop through face indices
    for f_idx in range(len(faces) - 1, -1, -1):

        (face_vert_loc_indices,
         face_vert_tex_indices,
         context_material,
         context_smooth_group,
         context_object,
         ) = faces[f_idx]

        len_face_vert_loc_indices = len(face_vert_loc_indices)

        if len_face_vert_loc_indices == 1:
            faces.pop(f_idx)  # cant add single vert faces

        elif not face_vert_tex_indices or len_face_vert_loc_indices == 2:  # faces that have no texture coords are lines
            if use_edges:
                # generators are better in python 2.4+ but can't be used in 2.3
                # edges.extend( (face_vert_loc_indices[i], face_vert_loc_indices[i+1]) for i in xrange(len_face_vert_loc_indices-1) )
                edges.extend([(face_vert_loc_indices[i], face_vert_loc_indices[i + 1]) for i in range(len_face_vert_loc_indices - 1)])

            faces.pop(f_idx)
        else:

            # Smooth Group
            if unique_smooth_groups and context_smooth_group:
                # Is a part of of a smooth group and is a face
                if context_smooth_group_old is not context_smooth_group:
                    edge_dict = smooth_group_users[context_smooth_group]
                    context_smooth_group_old = context_smooth_group

                for i in range(len_face_vert_loc_indices):
                    i1 = face_vert_loc_indices[i]
                    i2 = face_vert_loc_indices[i - 1]
                    if i1 > i2:
                        i1, i2 = i2, i1

                    try:
                        edge_dict[i1, i2] += 1
                    except KeyError:
                        edge_dict[i1, i2] = 1

            # NGons into triangles
            if has_ngons and len_face_vert_loc_indices > 4:

                ngon_face_indices = ngon_tessellate(verts_loc, face_vert_loc_indices)
                faces.extend([([face_vert_loc_indices[ngon[0]],
                                face_vert_loc_indices[ngon[1]],
                                face_vert_loc_indices[ngon[2]],
                                ],
                               [face_vert_tex_indices[ngon[0]],
                                face_vert_tex_indices[ngon[1]],
                                face_vert_tex_indices[ngon[2]],
                                ],
                               context_material,
                               context_smooth_group,
                               context_object,
                              )
                             for ngon in ngon_face_indices]
                            )

                # edges to make ngons
                if use_ngons:
                    edge_users = {}
                    for ngon in ngon_face_indices:
                        for i in (0, 1, 2):
                            i1 = face_vert_loc_indices[ngon[i]]
                            i2 = face_vert_loc_indices[ngon[i - 1]]
                            if i1 > i2:
                                i1, i2 = i2, i1

                            try:
                                edge_users[i1, i2] += 1
                            except KeyError:
                                edge_users[i1, i2] = 1

                    for key, users in edge_users.items():
                        if users > 1:
                            fgon_edges.add(key)

                # remove all after 3, means we dont have to pop this one.
                faces.pop(f_idx)

    # Build sharp edges
    if unique_smooth_groups:
        for edge_dict in list(smooth_group_users.values()):
            for key, users in list(edge_dict.items()):
                if users == 1:  # This edge is on the boundry of a group
                    sharp_edges[key] = None

    # map the material names to an index
    material_mapping = {name: i for i, name in enumerate(unique_materials)}  # enumerate over unique_materials keys()

    materials = [None] * len(unique_materials)

    for name, index in list(material_mapping.items()):
        materials[index] = unique_materials[name]

    me = bpy.data.meshes.new(dataname.decode('utf-8', "replace"))

    # make sure the list isnt too big
    for material in materials:
        me.materials.append(material)

    me.vertices.add(len(verts_loc))
    me.tessfaces.add(len(faces))

    # verts_loc is a list of (x, y, z) tuples
    me.vertices.foreach_set("co", unpack_list(verts_loc))

    # faces is a list of (vert_indices, texco_indices, ...) tuples
    # XXX faces should contain either 3 or 4 verts
    # XXX no check for valid face indices
    me.tessfaces.foreach_set("vertices_raw", unpack_face_list([f[0] for f in faces]))

    if verts_tex and me.tessfaces:
        me.tessface_uv_textures.new()

    context_material_old = -1  # avoid a dict lookup
    mat = 0  # rare case it may be un-initialized.
    me_faces = me.tessfaces

    for i, face in enumerate(faces):
        if len(face[0]) < 2:
            pass  # raise Exception("bad face")
        elif len(face[0]) == 2:
            if use_edges:
                edges.append(face[0])
        else:

            blender_face = me.tessfaces[i]

            (face_vert_loc_indices,
             face_vert_tex_indices,
             context_material,
             context_smooth_group,
             context_object,
             ) = face

            if context_smooth_group:
                blender_face.use_smooth = True

            if context_material:
                if context_material_old is not context_material:
                    mat = material_mapping[context_material]
                    context_material_old = context_material

                blender_face.material_index = mat
#                blender_face.mat= mat

            if verts_tex:

                blender_tface = me.tessface_uv_textures[0].data[i]

                if context_material:
                    image = unique_material_images[context_material]
                    if image:  # Can be none if the material dosnt have an image.
                        blender_tface.image = image

                # BUG - Evil eekadoodle problem where faces that have vert index 0 location at 3 or 4 are shuffled.
                if len(face_vert_loc_indices) == 4:
                    if face_vert_loc_indices[2] == 0 or face_vert_loc_indices[3] == 0:
                        face_vert_tex_indices = face_vert_tex_indices[2], face_vert_tex_indices[3], face_vert_tex_indices[0], face_vert_tex_indices[1]
                else:  # length of 3
                    if face_vert_loc_indices[2] == 0:
                        face_vert_tex_indices = face_vert_tex_indices[1], face_vert_tex_indices[2], face_vert_tex_indices[0]
                # END EEEKADOODLE FIX

                # assign material, uv's and image
                blender_tface.uv1 = verts_tex[face_vert_tex_indices[0]]
                blender_tface.uv2 = verts_tex[face_vert_tex_indices[1]]
                blender_tface.uv3 = verts_tex[face_vert_tex_indices[2]]

                if len(face_vert_loc_indices) == 4:
                    blender_tface.uv4 = verts_tex[face_vert_tex_indices[3]]

#                for ii, uv in enumerate(blender_face.uv):
#                    uv.x, uv.y=  verts_tex[face_vert_tex_indices[ii]]
    del me_faces
#     del ALPHA

    if use_edges and not edges:
        use_edges = False

    if use_edges:
        me.edges.add(len(edges))

        # edges should be a list of (a, b) tuples
        me.edges.foreach_set("vertices", unpack_list(edges))
#         me_edges.extend( edges )

#     del me_edges

    # Add edge faces.
#     me_edges= me.edges

    def edges_match(e1, e2):
        return (e1[0] == e2[0] and e1[1] == e2[1]) or (e1[0] == e2[1] and e1[1] == e2[0])

    me.validate()
    me.update(calc_edges=use_edges)

    if unique_smooth_groups and sharp_edges:
        import bmesh
        bm = bmesh.new()
        bm.from_mesh(me)
        # to avoid slow iterator lookups later / indexing verts is slow in bmesh
        bm_verts = bm.verts[:]

        for sharp_edge in sharp_edges.keys():
            vert1 = bm_verts[sharp_edge[0]]
            vert2 = bm_verts[sharp_edge[1]]
            if vert1 != vert2:
                edge = bm.edges.get((vert1, vert2))
                if edge is not None:
                    me.edges[edge.index].use_edge_sharp = True

        bm.free()
        del bm

    mesh_untessellate(me, fgon_edges)

    ob = bpy.data.objects.new(me.name, me)
    new_objects.append(ob)

    # Create the vertex groups. No need to have the flag passed here since we test for the
    # content of the vertex_groups. If the user selects to NOT have vertex groups saved then
    # the following test will never run
    for group_name, group_indices in vertex_groups.items():
        group = ob.vertex_groups.new(group_name.decode('utf-8', "replace"))
        group.add(group_indices, 1.0, 'REPLACE')

def create_nurbs(context_nurbs, vert_loc, new_objects):     # copy from blender addons; should be imported from there
    """
    Add nurbs object to blender, only support one type at the moment
    """
    deg = context_nurbs.get(b'deg', (3,))
    curv_range = context_nurbs.get(b'curv_range')
    curv_idx = context_nurbs.get(b'curv_idx', [])
    parm_u = context_nurbs.get(b'parm_u', [])
    parm_v = context_nurbs.get(b'parm_v', [])
    name = context_nurbs.get(b'name', b'ObjNurb')
    cstype = context_nurbs.get(b'cstype')

    if cstype is None:
        print('\tWarning, cstype not found')
        return
    if cstype != b'bspline':
        print('\tWarning, cstype is not supported (only bspline)')
        return
    if not curv_idx:
        print('\tWarning, curv argument empty or not set')
        return
    if len(deg) > 1 or parm_v:
        print('\tWarning, surfaces not supported')
        return

    cu = bpy.data.curves.new(name.decode('utf-8', "replace"), 'CURVE')
    cu.dimensions = '3D'

    nu = cu.splines.new('NURBS')
    nu.points.add(len(curv_idx) - 1)  # a point is added to start with
    nu.points.foreach_set("co", [co_axis for vt_idx in curv_idx for co_axis in (vert_loc[vt_idx] + (1.0,))])

    nu.order_u = deg[0] + 1

    # get for endpoint flag from the weighting
    if curv_range and len(parm_u) > deg[0] + 1:
        do_endpoints = True
        for i in range(deg[0] + 1):

            if abs(parm_u[i] - curv_range[0]) > 0.0001:
                do_endpoints = False
                break

            if abs(parm_u[-(i + 1)] - curv_range[1]) > 0.0001:
                do_endpoints = False
                break

    else:
        do_endpoints = False

    if do_endpoints:
        nu.use_endpoint_u = True

    # close
    '''
    do_closed = False
    if len(parm_u) > deg[0]+1:
        for i in xrange(deg[0]+1):
            #print curv_idx[i], curv_idx[-(i+1)]

            if curv_idx[i]==curv_idx[-(i+1)]:
                do_closed = True
                break

    if do_closed:
        nu.use_cyclic_u = True
    '''

    ob = bpy.data.objects.new(name.decode('utf-8', "replace"), cu)

    new_objects.append(ob)

def load(#operator, context,
         filepath,
         global_clamp_size=0.0,
         use_ngons=True,
         use_smooth_groups=True,
         use_edges=True,
         use_split_objects=True,
         use_split_groups=True,
         use_image_search=True,
         use_groups_as_vgroups=False,
         relpath=None,
         global_matrix=None,
         ):
    print('\nimporting bobj %r' % filepath)

    filepath = os.fsencode(filepath)

    #if global_matrix is None:
    #    global_matrix = mathutils.Matrix()
    global_matrix = mathutils.Matrix()

    #if use_split_objects or use_split_groups:
    #    use_groups_as_vgroups = False
    use_groups_as_vgroups = False

    verts_loc = []
    verts_tex = []
    faces = []
    material_libs = []
    vertex_groups = {}

    float_func = float      # not always right

    context_material = None
    context_smooth_group = None
    context_object = None
    context_vgroup = None

    context_nurbs = {}
    nurbs = []
    context_parm = b''

    has_ngons = False

    unique_materials = {}
    unique_material_images = {}
    unique_smooth_groups = {}

    context_multi_line = b''

    stream = open(filepath, 'rb')

    read_bytes = stream.read()
    offset = 0
    while offset < len(read_bytes):
        data = None
        marker = struct.unpack('i', read_bytes[offset:offset+4])[0]
        #print('marker:', marker)
        offset += 4
        if marker == 1:
            data = struct.unpack('fff', read_bytes[offset:offset+12])
            verts_loc.append(data)
            offset += 12
        elif marker == 2:
            data = struct.unpack('ff', read_bytes[offset:offset+8])
            verts_tex.append((float_func(data[0]), float_func(data[1])))
            offset += 8
        elif marker == 3:
            #data = struct.unpack('fff', read_bytes[offset:offset+12])
            offset += 12
            # Nothing to do here

        elif marker == 4:

            face_vert_loc_indices = []
            face_vert_tex_indices = []

            faces.append((face_vert_loc_indices,
                          face_vert_tex_indices,
                          context_material,
                          context_smooth_group,
                          context_object,
                          ))

            for i in range(3):      #really?
                data = struct.unpack('iii', read_bytes[offset:offset+12])
                vert = data

                vert_loc_index = int(vert[0]) - 1
                #print('vert_loc_index:', vert_loc_index)
                if vert_loc_index < 0:
                    vert_loc_index = len(verts_loc) + vert_loc_index + 1
                face_vert_loc_indices.append(vert_loc_index)
                if len(vert) > 1 and vert[1]:
                    vert_tex_index = int(vert[1]) - 1
                    if vert_tex_index < 0:
                        vert_tex_index = len(verts_tex) + vert_tex_index + 1
                    face_vert_tex_indices.append(vert_tex_index)
                else:
                    # dummy
                    face_vert_tex_indices.append(0)

                offset += 12

            if len(face_vert_loc_indices) > 4:
                has_ngons = True


            #if face_vert_loc_indices == [4, 3, 7]:
            #    face_vert_loc_indices = [0, 1, 2]

        else:
            print('ERROR: unable to read from here')
            break

        # more?

    stream.close()


    #relpath = None
    #use_image_search = True
    #use_split_objects = True
    #use_split_groups = True
    #use_ngons = True
    #use_edges = True



    create_materials(filepath, relpath, material_libs, unique_materials, unique_material_images, use_image_search, float_func)

    if bpy.ops.object.select_all.poll():
        bpy.ops.object.select_all(action='DESELECT')

    scene = bpy.context.scene

    new_objects = []

    if use_split_objects or use_split_groups:
        SPLIT_OB_OR_GROUP = True
    else:
        SPLIT_OB_OR_GROUP = False

    for verts_loc_split, faces_split, unique_materials_split, dataname in split_mesh(verts_loc, faces, unique_materials, filepath, SPLIT_OB_OR_GROUP):
        create_mesh(new_objects,
                    has_ngons,
                    use_ngons,
                    use_edges,
                    verts_loc_split,
                    verts_tex,
                    faces_split,
                    unique_materials_split,
                    unique_material_images,
                    unique_smooth_groups,
                    vertex_groups,
                    dataname,
                    )

    for context_nurbs in nurbs:
        create_nurbs(context_nurbs, verts_loc, new_objects)

    # Create new obj
    for obj in new_objects:
        base = scene.objects.link(obj)
        base.select = True

        # we could apply this anywhere before scaling.
        obj.matrix_world = global_matrix

    scene.update()

    axis_min = [1000000000] * 3
    axis_max = [-1000000000] * 3

    if global_clamp_size:
        # Get all object bounds
        for ob in new_objects:
            for v in ob.bound_box:
                for axis, value in enumerate(v):
                    if axis_min[axis] > value:
                        axis_min[axis] = value
                    if axis_max[axis] < value:
                        axis_max[axis] = value

        # Scale objects
        max_axis = max(axis_max[0] - axis_min[0], axis_max[1] - axis_min[1], axis_max[2] - axis_min[2])
        scale = 1.0

        while global_clamp_size < max_axis * scale:
            scale = scale / 10.0

        for obj in new_objects:
            obj.scale = scale, scale, scale

