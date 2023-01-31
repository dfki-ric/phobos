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
from ..commandline_logging import get_logger
log = get_logger(__name__)


# ToDo have a mesh representation that deals with either blender or trimesh depending on whats available
def as_mesh(scene_or_mesh, scale=None):
    if scale is None:
        scale = [1.0, 1.0, 1.0]
    scale = np.asarray(scale)
    if hasattr(scene_or_mesh, "bounds") and scene_or_mesh.bounds is None:
        return None
    if isinstance(scene_or_mesh, trimesh.Scene):
        log.error("Received a mesh with multiple materials, material information especially textures may be lost.")
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
        # [TODO pre_v2.0.0] Resolve the following error
        log.error("Received a blender mesh, porting uv maps to trimesh representation are not yet supported and thus will be lost.")
        mesh = trimesh.Trimesh(vertices=vertices, faces=trimesh.geometry.triangulate_quads(faces))
    else:
        mesh = scene_or_mesh
    assert isinstance(mesh, trimesh.Trimesh), f"Can't convert {type(scene_or_mesh)} to trimesh.Trimesh!"
    return mesh


def export_bobj(outname, mesh):
    """
    Exports the mesh as bobj.
    [TODO v2.1.0] Export UVs, too
    """

    if hasattr(mesh.visual, "uv"):
        write_uv = True
    else:
        write_uv = False

    if os.path.isfile(outname):
        # [TODO v2.1.0] load bobj mesh
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

    with open(outname, "wb") as out:
        # vertices
        N = len(mesh.vertices)
        assert N > 0
        A = np.array(mesh.vertices, dtype=np.single)
        key = struct.unpack("f", struct.pack("i", 1))  # data type trick for writing an int in a float array
        out.write(np.c_[np.array([key]*N, dtype=np.single), A].tobytes())

        if write_uv:
            # uv maps
            assert len(mesh.visual.uv) == len(mesh.vertices)
            N = len(mesh.visual.uv)
            A = np.array(mesh.visual.uv, dtype=np.single)
            key = struct.unpack("f", struct.pack("i", 2))  # data type trick for writing an int in a float array
            out.write(np.c_[np.array([key]*N, dtype=np.single), A].tobytes())

        # vertex_normals
        assert len(mesh.vertex_normals) == len(mesh.vertices)
        N = len(mesh.vertex_normals)
        A = np.array(mesh.vertex_normals, dtype=np.single)
        key = struct.unpack("f", struct.pack("i", 3))  # data type trick for writing an int in a float array
        out.write(np.c_[np.array([key]*N, dtype=np.single), A].tobytes())

        # linking information for each triangle: vertex, uv, normal
        values = []
        for tri in mesh.triangles:
            values2 = [4]
            assert len(tri) == 3
            for v in tri:
                v_index = geometry.get_vertex_id(v, mesh)
                assert v_index >= 0
                assert v_index < N
                values2 += [v_index + 1]*3 if write_uv else [v_index + 1, 0, v_index + 1]
            values.append(values2)
            assert len(values2) == 10
        A = np.array(values, dtype=np.intc)
        out.write(A.tobytes())

    # print("Exported", outname)


def export_mesh(mesh, filepath, urdf_path=None, dae_mesh_color=None):
    """Export the mesh to a given filepath with an urdf_path.
    """
    mesh = as_mesh(mesh)
    mesh = improve_mesh(mesh)

    if urdf_path is not None and urdf_path.lower().endswith(".urdf"):
        urdf_path = os.path.dirname(urdf_path)

    filepath = xml_utils.read_urdf_filename(filepath, urdf_path)

    do_export = mesh is not None
    if do_export and os.path.isfile(filepath):
        existing_mesh = import_mesh(filepath)
        if existing_mesh is not None and identical(mesh, existing_mesh):
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
    out = as_mesh(trimesh.load_mesh(filepath, maintain_order=True))
    if out is None:
        log.info(f"{filepath} contains empty mesh!")
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
    "stl": {"export": export_mesh, "import": None, "extension": "stl"},
    "obj": {"export": export_mesh, "import": None, "extension": "obj"},
    "dae": {"export": export_mesh, "import": None, "extension": "dae"},
    "mars_obj": {"export": export_mars_mesh, "import": None, "extension": "mars.obj"},
    "bobj": {"export": export_bobj_mesh, "import": None, "extension": "bobj"},
}
MESH_TYPES = ["stl", "obj", "dae", "mars_obj", "bobj"]