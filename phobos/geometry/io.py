#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import deepcopy
import os
import sys
import numpy as np
import trimesh
import struct
import json

from . import geometry
from .geometry import identical, improve_mesh, reduce_mesh
from ..defs import BPY_AVAILABLE
from ..utils import misc, xml as xml_utils
from ..utils.commandline_logging import get_logger
log = get_logger(__name__)


def as_mesh(scene_or_mesh, scale=None):
    if scale is None:
        scale = [1.0, 1.0, 1.0]
    scale = np.asarray(scale)
    if hasattr(scene_or_mesh, "bounds") and scene_or_mesh.bounds is None:
        return None
    if isinstance(scene_or_mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate([
            trimesh.Trimesh(vertices=m.vertices, faces=m.faces)
            for m in scene_or_mesh.geometry.values()])
    elif not isinstance(scene_or_mesh, trimesh.Trimesh):  # we assume it's blender
        blender_mesh = scene_or_mesh.data
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
        mesh = trimesh.Trimesh(vertices=vertices, faces=trimesh.geometry.triangulate_quads(faces))
    else:
        mesh = scene_or_mesh
    assert isinstance(mesh, trimesh.Trimesh), f"Can't convert {type(scene_or_mesh)} to trimesh.Trimesh!"
    return mesh


def export_bobj(outname, mesh):
    """
    Exports the mesh as bobj.
    TODO: Export UVs, too
    """
    num_normals = 1
    # numUVs = 1
    global_normals = {}

    # if hasattr(mesh.visual, "uv"):
    #     write_uv = True
    #     uv_layer = mesh.visual.uv
    # else:
    #     write_uv = False

    if os.path.isfile(outname):
        # TODO load bobj mesh
        # test_mesh = import_mesh(outname)
        # if all(trimesh.comparison.identifier_simple(mesh) == trimesh.comparison.identifier_simple(test_mesh)):
        log.warning(f"Mesh {outname} does already exist. Skipping export.")
        return
    mesh.fix_normals()
    mesh.merge_vertices()
    mesh = improve_mesh(mesh)

    log.info(f"Exporting {outname} with {len(mesh.vertices)} vertices...")
    if not outname.endswith(".bobj"):
        outname += ".bobj"

    out = open(outname, "wb")
    for v in mesh.vertices:
        out.write(struct.pack('ifff', 1, v[0], v[1], v[2]))

    # uv_face_mapping = {}
    # if write_uv:
    #     for tri in mesh.triangles:
    #         uv_face_mapping[tri] = {}
    #         for loop_index in tri.loops:
    #             uv_face_mapping[tri][loop_index] = numUVs
    #             numUVs += 1
    #             out.write(struct.pack('iff', 2, uv_layer.data[loop_index].uv[0], uv_layer.data[loop_index].uv[1]))

    for tri in mesh.triangles:
        for v in tri:
            n = geometry.round_vector(mesh.vertex_normals[geometry.get_vertex_id(v, mesh)])
            if n not in global_normals:
                global_normals[n] = num_normals
                num_normals += 1
                out.write(struct.pack('ifff', 3, n[0], n[1], n[2]))

    for tri in mesh.triangles:
        da = struct.pack('i', 4)
        out.write(da)
        for v in tri:
            v_index = geometry.get_vertex_id(v, mesh)
            # if write_uv:
            #     uvIndex = tri.loops[i]
            #     #print(uv_face_mapping[tri])
            #     uvFace = uv_face_mapping[tri][uvIndex]
            #     da = struct.pack('iii', v_index + 1, uvFace, global_normals[roundV(v.normal)])
            #     out.write(da)  # vert, uv, normal
            # else:
            da = struct.pack('iii', v_index + 1, 0, global_normals[geometry.round_vector(mesh.vertex_normals[v_index])])
            out.write(da)  # vert, uv, normal

    out.close()
    # print("Exported", outname)


def export_mesh(mesh, filepath, urdf_path=None, dae_mesh_color=None):
    """Export the mesh to a given filepath with an urdf_path.
    """
    mesh = as_mesh(mesh)
    mesh = improve_mesh(mesh)

    if urdf_path is not None and urdf_path.lower().endswith(".urdf"):
        urdf_path = os.path.dirname(urdf_path)

    filepath = xml_utils.read_urdf_filename(filepath, urdf_path)

    do_export = True
    if os.path.isfile(filepath):
        existing_mesh = import_mesh(filepath)
        if identical(mesh, existing_mesh):
            #print("NOTE: Skipping export of", filepath, "as the mesh file already exists and is identical")
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
        else:
            mesh.export(
                file_obj=filepath
            )
    return filepath


def export_mars_mesh(mesh, filepath, urdf_path=None):
    """Export the mesh as obj rotated according to mars to a given filepath with an urdf_path.
    """
    m = as_mesh(mesh)
    m = deepcopy(m)
    m = improve_mesh(m)

    if filepath.split(".")[-1] == 'obj':
        v_ = m.vertices

        # Swap according to blender
        v = np.column_stack(
            (v_[:, 0], v_[:, 2], -v_[:, 1])
        )

        m.vertices = v
        trimesh.repair.broken_faces(m)
        m.merge_vertices()

        try:
            trimesh.repair.fix_normals(m)
        except:
            pass

    return export_mesh(m, filepath, urdf_path=urdf_path)


def export_bobj_mesh(mesh, filepath, urdf_path=None):
    m = as_mesh(mesh)
    m = deepcopy(m)

    # if filepath.split(".")[-1] == 'bobj':
    #     v_ = m.vertices
    #
    #     # Swap according to blender
    #     v = np.column_stack(
    #         (v_[:, 0], v_[:, 2], -v_[:, 1])
    #     )
    #
    #     m.vertices = v
    #     trimesh.repair.broken_faces(m)
    #     m.merge_vertices()
    #
    #     try:
    #         trimesh.repair.fix_normals(m)
    #     except:
    #         pass

    # Check if filepath is abspath
    if not os.path.isabs(filepath) and not urdf_path:
        filepath = os.path.abspath(filepath)
    elif not os.path.isabs(filepath) and urdf_path:
        filepath = os.path.join(
            os.path.abspath(urdf_path), filepath
        )
    elif not os.path.isabs(filepath):
        filepath = os.path.abspath(filepath)

    export_bobj(filepath, m)
    return filepath


def import_mesh(filepath, urdf_path=None):
    """Import the mesh from a given filepath with an urdf_path.
    """

    filepath = xml_utils.read_urdf_filename(filepath, urdf_path)

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Mesh file {filepath} does not exist!")
    out = as_mesh(trimesh.load_mesh(filepath))
    if out is None:
        log.warning(f"{filepath} contains empty mesh!")
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


mesh_types = {
    "dae": {"export": export_mesh, "import": None, "extension": "dae"},
    "stl": {"export": export_mesh, "import": None, "extension": "stl"},
    "obj": {"export": export_mesh, "import": None, "extension": "obj"},
    "mars_obj": {"export": export_mars_mesh, "import": None, "extension": ".mars.obj"},
    "bobj": {"export": export_bobj_mesh, "import": None, "extension": "bobj"},
}