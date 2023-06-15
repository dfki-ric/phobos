#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import struct
from copy import deepcopy

import numpy as np
import trimesh

from .geometry import identical
from ..commandline_logging import get_logger
from ..defs import BPY_AVAILABLE
from ..utils import misc, xml as xml_utils

log = get_logger(__name__)


def as_trimesh(scene_or_mesh, scale=None, silent=False):
    assert scene_or_mesh is not None
    if scale is None:
        scale = [1.0, 1.0, 1.0]
    scale = np.asarray(scale)

    if isinstance(scene_or_mesh, trimesh.Scene):
        if not silent:
            log.error("Received a mesh with multiple materials, material information especially textures may be lost.")
        if len(scene_or_mesh.geometry.values()) == 0:
            raise IOError(f"{scene_or_mesh.file_name} seems to be an invalid mesh_file")
        mesh = trimesh.util.concatenate([
            trimesh.Trimesh(vertices=m.vertices, faces=m.faces)
            for m in scene_or_mesh.geometry.values()])
    elif not isinstance(scene_or_mesh, trimesh.Trimesh) and BPY_AVAILABLE:  # we assume it's blender
        import bpy
        blender_mesh = scene_or_mesh if isinstance(scene_or_mesh, bpy.types.Mesh) else scene_or_mesh.data
        if len(set([len(p.vertices) for p in blender_mesh.polygons])) > 1:
            assert BPY_AVAILABLE
            import bmesh
            bm = bmesh.new()
            bm.from_mesh(scene_or_mesh.data)
            bmesh.ops.triangulate(bm, faces=bm.faces[:])
            bm.to_mesh(blender_mesh)
            bm.free()
        vertices = np.asarray([np.asarray(scale * v.co) for v in blender_mesh.vertices])
        faces = np.array([[v for v in p.vertices] for p in blender_mesh.polygons], dtype=np.int64)
        if not silent:
            # [TODO v2.0.0] Resolve the following error
            log.error("Received a blender mesh, porting uv maps to trimesh representation are not yet supported and thus will be lost.")
        mesh = trimesh.Trimesh(vertices=vertices, faces=trimesh.geometry.triangulate_quads(faces))
    else:
        mesh = scene_or_mesh
    assert isinstance(mesh, trimesh.Trimesh), f"Can't convert {type(scene_or_mesh)} to trimesh.Trimesh!"
    if hasattr(mesh, "bounds") and mesh.bounds is None:
        log.debug(f"Given {type(scene_or_mesh)} has bounds == None, this mesh seems empty.")
        return None
    return mesh


def blender_2_mesh_info_dict(mesh):
    """
    Creates the mesh info dict
    Args:
        mesh: bpy.types.Mesh

    Returns:
        {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[3*(n,3)]] intc, ["texture_coords": (n,2) single]}
    """
    if BPY_AVAILABLE:
        import bpy
    assert isinstance(mesh, bpy.types.Mesh)
    mesh.calc_loop_triangles()
    mesh.calc_normals()

    uv_layer = mesh.uv_layers.active
    write_uv = uv_layer is not None and len(uv_layer.data) > 0

    # prepare info dict
    n_info = {
        "vertices": np.array([v.co for v in mesh.vertices], dtype=np.single),
        "vertex_normals": np.array([v.normal for v in mesh.vertices], dtype=np.single),
        "faces": []
    }

    if write_uv:
        n_info["texture_coords"] = np.ndarray(shape=(n_info["vertices"].shape[0], 2), dtype=np.single)
        for loop in mesh.loops:
            n_info["texture_coords"][loop.vertex_index, :] = uv_layer.data[loop.index].uv

    for tri in mesh.loop_triangles:
        n_info["faces"].append([])
        for vIndex in tri.vertices:
            assert vIndex < n_info["vertices"].shape[0]
            if write_uv:
                n_info["faces"][-1].append(np.array([vIndex]*3, dtype=np.intc))
            else:
                n_info["faces"][-1].append(np.array([vIndex, -1, vIndex], dtype=np.intc))

    return n_info


def mesh_info_dict_2_blender(name, vertices, faces, vertex_normals=None, texture_coords=None, **mesh_info_dict):
    """
    Creates the blender mesh from the mesh info dict
    Args:
        name: the name of the mesh
        **mesh_info_dict: {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[3*(n,3)]] intc, ["texture_coords": (n,2) single]}

    Returns:
        bpy.types.Mesh

    Note:
        Remember to link the object to a valid collection

    """
    assert BPY_AVAILABLE
    import bpy
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], [f[0] for f in faces])
    return mesh


def trimesh_2_mesh_info_dict(mesh):
    """
    Creates the mesh info dict
    Args:
        mesh: trimesh.Trimesh

    Returns:
        {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[3*(n,3)]] intc, ["texture_coords": (n,2) single]}
    """
    assert isinstance(mesh, trimesh.Trimesh)
    if hasattr(mesh.visual, "uv"):
        write_uv = True
    else:
        write_uv = False

    out = {}
    # vertices
    N = len(mesh.vertices)
    out["vertices"] = np.array(mesh.vertices, dtype=np.single)

    if write_uv:
        # uv maps
        assert len(mesh.visual.uv) == len(mesh.vertices)
        out["texture_coords"] = np.array(mesh.visual.uv, dtype=np.single)

    # vertex_normals
    assert len(mesh.vertex_normals) == len(mesh.vertices)
    out["vertex_normals"] = np.array(mesh.vertex_normals, dtype=np.single)

    # linking information for each triangle: vertex, uv, normal
    out["faces"] = []
    for face in mesh.faces:
        values2 = []
        assert len(face) == 3
        for v_index in face:
            assert v_index >= 0
            assert v_index < N
            values2.append(np.array([v_index ]*3 if write_uv else [v_index, -1, v_index], dtype=np.intc))
        assert len(values2) == 3
        out["faces"].append(values2)

    return out


def mesh_info_dict_2_trimesh(vertices, faces, vertex_normals=None, texture_coords=None, **mesh_info_dict):
    """
    Creates the trimesh from the mesh info dict
    Args:
        **mesh_info_dict: {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[3*(n,3)]] intc, ["texture_coords": (n,2) single]}

    Returns:
        trimesh.Trimesh
    """
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_normals=vertex_normals)
    if texture_coords is not None:
        mesh.visual = trimesh.visual.TextureVisuals(uv=texture_coords)
    return mesh


def triangulate_faces_in_info_dict(faces, **mesh_info_dict):
    """
    Triangulates the faces of the given mesh info dict.
    Args:
        **mesh_info_dict: {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[n*(n,3)]] intc, ["texture_coords": (n,2) single]}

    Returns:
        {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[3*(n,3)]] intc, ["texture_coords": (n,2) single]}
    """
    new_faces = []
    for face in faces:
        new_faces.append(face[0:3])
        if len(face) > 3:
            for i in range(len(face)-3):
                # [TODO v2.1.0] This handling may create undercuts for non-convex polygonal faces
                new_faces.append([face[0], face[i+2], face[i+3]])
    mesh_info_dict["faces"] = new_faces
    return mesh_info_dict


def write_bobj(filepath, vertices=None, vertex_normals=None, faces=None, texture_coords=None, **mesh_info_dict):
    """
    Writes the mesh_info_dict to bobj format.

    Args:
        filepath: the filepath where to write the file
        **mesh_info_dict: {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[n*(n,3)]] intc, ["texture_coords": (n,2) single]}

    Returns:
        {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[3*(n,3)]] intc, ["texture_coords": (n,2) single]}
    """
    # [Todo v2.1.0] make the bobj export work for more inputs
    # check input
    assert faces is not None
    assert vertex_normals is not None
    assert vertices is not None
    write_uv = texture_coords is not None

    # Make sure we have triangles
    faces = triangulate_faces_in_info_dict(faces)["faces"]

    assert isinstance(vertices, np.ndarray) and vertices.dtype == np.single and vertices.shape[1] == 3
    assert isinstance(vertex_normals, np.ndarray) and vertices.dtype == np.single and vertices.shape[1] == 3
    assert not write_uv or (isinstance(texture_coords, np.ndarray) and texture_coords.dtype == np.single and texture_coords.shape[1] == 2)
    assert type(faces) == list and type(faces[0]) == list and isinstance(faces[0][0], np.ndarray) and faces[0][0].dtype == np.intc
    with open(filepath, "wb") as out:
        # vertices
        N = vertices.shape[0]
        assert N > 0
        key = struct.unpack("f", struct.pack("i", 1))  # data type trick for writing an int in a float array
        out.write(np.c_[np.array([key]*N, dtype=np.single), vertices].tobytes())

        if write_uv:
            # uv maps
            if texture_coords.shape[0] == vertices.shape[0]:
                log.warning(f"UV coords of mesh {filepath} might not be accurate")
            N = texture_coords.shape[0]
            key = struct.unpack("f", struct.pack("i", 2))  # data type trick for writing an int in a float array
            out.write(np.c_[np.array([key]*N, dtype=np.single), texture_coords].tobytes())

        # vertex_normals
        if vertex_normals.shape[0] == vertices.shape[0]:
            log.warning(f"Vertex normals of mesh {filepath} might not be accurate")
        N = vertex_normals.shape[0]
        key = struct.unpack("f", struct.pack("i", 3))  # data type trick for writing an int in a float array
        out.write(np.c_[np.array([key]*N, dtype=np.single), vertex_normals].tobytes())

        # faces
        key = struct.unpack("i", struct.pack("i", 4))  # data type trick for writing an int in a float array
        N = len(faces)
        out_faces = []
        for face in faces:
            out_faces.append(np.concatenate(face))
        out_faces = np.array(out_faces, dtype=np.intc)
        out_faces = out_faces + np.ones(out_faces.shape, dtype=np.intc)
        out.write(np.c_[np.array([key] * N, dtype=np.intc), np.array(out_faces, dtype=np.intc)].tobytes())


def export_mesh(mesh, filepath, urdf_path=None, dae_mesh_color=None):
    """
    Export the mesh to a given filepath with an urdf_path. Detects the format by file ending.
    NOTE: For bobj it's necessary to pass a trimesh.Trimesh as mesh other will raise an AssertionError.

    Args:
        mesh: trimesh.Trimesh or trimesh.Scene
        filepath: the filepath where to write the file
        **mesh_info_dict: {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[n*(n,3)]] intc, ["texture_coords": (n,2) single]}

    Returns:
        {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[3*(n,3)]] intc, ["texture_coords": (n,2) single]}
    """

    if urdf_path is not None and urdf_path.lower().endswith(".urdf"):
        urdf_path = os.path.dirname(urdf_path)

    filepath = xml_utils.read_relative_filename(filepath, urdf_path)

    do_export = mesh is not None
    if do_export and os.path.isfile(filepath):
        existing_mesh = import_mesh(filepath)
        if existing_mesh is not None and identical(mesh, existing_mesh):
            log.debug(f"Skipping export of {filepath} as the mesh file already exists and is identical")
            do_export = False

    if do_export:
        if filepath.lower().endswith("dae"):
            dae_xml = trimesh.exchange.dae.export_collada(mesh)
            if dae_mesh_color is not None:
                dae_xml = dae_xml.decode(json.detect_encoding(dae_xml))
                dae_xml = misc.regex_replace(dae_xml, {
                    "<color>0.0 0.0 0.0 1.0</color>": " <color>" + " ".join(
                        [str(n) for n in dae_mesh_color]) + "</color>"})
                with open(filepath, "w") as f:
                    f.write(dae_xml)
            else:
                with open(filepath, "wb") as f:
                    f.write(dae_xml)
        elif filepath.endswith("bobj"):
            assert isinstance(mesh, trimesh.Trimesh), f"Export to bobj only possible from trimesh.Trimesh not for {type(mesh)}"
            log.debug(f"Exporting {filepath} with {len(mesh.vertices)} vertices...")
            write_bobj(filepath, **trimesh_2_mesh_info_dict(as_trimesh(mesh)))
        else:
            mesh.export(
                file_obj=filepath
            )
    return filepath


def import_mesh(filepath, urdf_path=None):
    """Import the mesh from a given filepath with an urdf_path.
    """

    filepath = xml_utils.read_relative_filename(filepath, urdf_path)

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Mesh file {filepath} does not exist!")
    if filepath.endswith(".bobj"):
        mid = parse_bobj(filepath)
        out = mesh_info_dict_2_trimesh(**mid)
    else:
        out = trimesh.load_mesh(filepath, maintain_order=True)
    if out.bounds is None:
        raise IOError(f"{filepath} seems to contain an invalid mesh!")
    return out


def import_mars_mesh(filepath, urdf_path=None):
    mesh = import_mesh(filepath, urdf_path)

    m = deepcopy(mesh)

    v_ = mesh.vertices

    # Swap according to blender
    v = np.column_stack(
        (v_[:, 0], -v_[:, 2], v_[:, 1])
    )

    m.vertices = v
    trimesh.repair.broken_faces(m)
    m.merge_vertices()

    try:
        trimesh.repair.fix_normals(m)
    except:
        pass

    return m


def parse_obj(filepath):
    # [Todo v2.1.0] make this more stable
    assert os.path.isfile(filepath)
    # deal with material sections
    keys = ["vn", "vt", "vp", "v", "f"]  # , "l"]
    info_names = {"vn": "vertex_normals", "vt": "texture_coords", "vp": "points", "v": "vertices", "f": "faces", "l": "lines"}
    shapes = {
        "vn": 3,
        "vt": 2,
        "vp": 3,
        "v": 3,
    }
    s_info = {k: [] for k in keys}
    with open(filepath, "r") as f:
        for line in f.readlines():
            for key in keys:
                if line.startswith(key):
                    s_info[key].append(line[len(key):].strip())
    n_info = {
        info_names[k]: np.fromstring("\n".join(s_info[k]), sep=" ", dtype=np.single).reshape((-1, shapes[k]))
        for k in shapes.keys()
    }
    for key in [k for k in keys if k not in shapes.keys()]:
        if key == "f":
            n_info[info_names[key]] = [[np.fromstring(corner.strip().replace("//", "/0/"), sep="/", dtype=np.intc) - np.ones(3, dtype=np.intc) for corner in line.strip().split(" ")] for line in s_info[key]]
        else:  # key == "l"
            n_info[info_names[key]] = [np.fromstring(line.strip(), sep=" ", dtype=np.intc) for line in s_info[key]]
    return n_info


def parse_bobj(filepath):
    """
    Parses the mesh_info_dict from bobj format.

    Args:
        filepath: the filepath where to write the file

    Returns:
        {"vertices": (n,3) single, "vertex_normals": (n,3) single, "faces": [n*[3*(n,3)]] intc, ["texture_coords": (n,2) single]}
    """
    assert os.path.isfile(filepath)
    shapes = {
        "vertex_normals": 3,
        "texture_coords": 2,
        "vertices": 3,
        "faces": (3, 3)
    }
    info_names = ["vertices", "texture_coords", "vertex_normals", "faces"]
    info_dict = {}
    with open(filepath, "rb") as f:
        chunk = f.read(4)
        while chunk is not None and len(chunk) > 0:
            i_key = struct.unpack("i", chunk)[0]
            s_key = info_names[i_key-1]
            if i_key-1 in range(3):
                if s_key not in info_dict:
                    info_dict[s_key] = np.frombuffer(f.read(shapes[s_key]*4), dtype=np.single)
                else:
                    info_dict[s_key] = np.vstack([info_dict[s_key], np.frombuffer(f.read(shapes[s_key]*4), dtype=np.single)])
            elif i_key == 4:  # faces
                if s_key not in info_dict:
                    info_dict[s_key] = []
                info_dict[s_key].append([np.frombuffer(f.read(shapes[s_key][0]*4), dtype=np.intc) - np.ones(3, dtype=np.intc) for _ in range(3)])
            else:
                raise IOError("Unknown bobj format!")
            chunk = f.read(4)
    for i_key in range(3):
        if info_names[i_key] in info_dict:
            info_dict[info_names[i_key]] = info_dict[info_names[i_key]].reshape((-1, shapes[info_names[i_key]]))
    return info_dict


def parse_dae(filepath):
    # [TODO v2.1.0]
    log.warning("mesh_info dict can currently not perfectly be parsed from dae")
    return trimesh_2_mesh_info_dict(as_trimesh(import_mesh(filepath)))
