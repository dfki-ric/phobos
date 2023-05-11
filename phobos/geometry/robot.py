#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

import numpy as np
import trimesh

from . import geometry
from . import io
from ..io import representation


def generate_kccd_optimizer_ready_collision(robot, linkname, outputdir, join_first=True, merge_additionally=None,
                                            mars_meshes=False, reduce_meshes=0):
    """
    Takes the convexhulls of all visuals of the link and joins them to one mesh.
    If the join_first option is set to true then the meshes will be joint before the convexhull is generated.

    WARNING: This function will edit the meshes pathes to absolute ones and should therefore only be used with a URDF
    that'S only purpose is to generate the kccd model
    """
    if isinstance(linkname, list):
        [generate_kccd_optimizer_ready_collision(robot, lname, outputdir, merge_additionally=merge_additionally,
                                                 join_first=join_first, mars_meshes=mars_meshes,
                                                 reduce_meshes=reduce_meshes) for lname in linkname]
        return

    if merge_additionally is None:
        merge_additionally = []

    def get_meshes_of_link_and_fixed_children(parent, transform=np.identity(4)):
        out = []
        parent = robot.get_link(parent)
        for visual in parent.visuals:
            m = io.as_trimesh(visual.geometry.load_mesh(), silent=True)
            m.apply_transform(visual.origin.to_matrix())
            m.apply_transform(transform)
            if not join_first:
                out += [m.convex_hull]
            else:
                out += [m]
        children = robot.get_children(parent.name)
        for jchildname in children:
            jchild = robot.get_joint(jchildname)
            if jchild.joint_type == "fixed" or jchildname in merge_additionally:
                out += get_meshes_of_link_and_fixed_children(jchild.child,
                                                             transform.dot(jchild.origin.to_matrix()))
        return out

    assert type(linkname) is str
    link = robot.get_link(linkname)
    joint = robot.get_parent(linkname)
    remove_collision(robot, linkname)
    if joint is None or not (robot.get_joint(joint).joint_type == "fixed" or joint in merge_additionally):
        meshes = get_meshes_of_link_and_fixed_children(linkname)
        if len(meshes) == 0:
            return
        elif len(meshes) == 1:
            mesh = meshes[0]
        else:
            mesh = trimesh.util.concatenate(meshes)
        if join_first:
            mesh = mesh.convex_hull

        mesh = geometry.improve_mesh(mesh)
        if reduce_meshes > 0:
            mesh = geometry.reduce_mesh(mesh, 1-reduce_meshes)

        mesh_representation = representation.Mesh(mesh=mesh, meshname="collision_"+linkname)
        # [TODO pre_v2.0.0] Review whether the mesh export happens on export of this xml anyways
        mesh_representation.provide_mesh_file(targetpath=outputdir, format="stl")
        link.add_aggregate("collision", representation.Collision(
            origin=representation.Pose(rpy=[0, 0, 0], xyz=[0, 0, 0], relative_to=link),
            geometry=mesh_representation,
            name="collision_"+link.name
        ))


def find_zero_pose_collisions(robot):
    coll_mgr = trimesh.collision.CollisionManager()
    for link in robot.links:
        T = robot.get_transformation(link.name)
        for coll in link.collisions:
            if isinstance(coll.geometry, representation.Mesh):
                mesh = coll.geometry.load_mesh()
                if not isinstance(mesh, trimesh.Trimesh) and not isinstance(mesh, trimesh.Scene):
                    mesh = io.as_trimesh(mesh)
                mesh = mesh.convex_hull
            elif isinstance(coll.geometry, representation.Box):
                mesh = trimesh.creation.box(coll.geometry.size)
            elif isinstance(coll.geometry, representation.Sphere):
                mesh = trimesh.creation.icosphere(4, coll.geometry.radius)
            elif isinstance(coll.geometry, representation.Cylinder):
                mesh = trimesh.creation.cylinder(coll.geometry.radius, coll.geometry.length)
            else:
                raise TypeError("Geometry type not known!")
            mesh.apply_transform(T.dot(coll.origin.to_matrix()))
            coll_mgr.add_object(coll.name, mesh)
    colls_exist, zero_pose_colls = coll_mgr.in_collision_internal(return_names=True)
    return zero_pose_colls if colls_exist else None


def replace_geometry(element, shape='box', oriented=False, scale=1.0):
    """
    Replace the geometry of the element with an oriented shape. urdf_path is needed for mesh loading.
    Args:
        element: An geometry element representation.Visual or representation.Collision
        shape: ['box', 'sphere', 'cylinder', 'convex']
        oriented: Whether the bounding box should be oriented to have the minimum volume to cover the element
        scale: whether the created shape shall be scaled by the given value

    Returns:
        None
    """
    if type(element) is list:
        return [replace_geometry(e, shape, oriented, scale) for e in element]

    if element is None:
        return

    if not isinstance(element.geometry, representation.Mesh):
        return

    mesh = io.as_trimesh(element.geometry.load_mesh(), silent=True)
    mesh.apply_transform(element.origin.to_matrix())

    if shape == 'sphere':
        element.geometry, transform = geometry.create_sphere(mesh, scale=scale)
        element.origin = representation.Pose.from_matrix(transform, relative_to=element.link)
    elif shape == 'cylinder':
        element.geometry, transform = geometry.create_cylinder(mesh, scale=scale)
        element.origin = representation.Pose.from_matrix(transform, relative_to=element.link)
    elif shape == 'box':
        element.geometry, transform = geometry.create_box(mesh, oriented=oriented, scale=scale)
        element.origin = representation.Pose.from_matrix(transform, relative_to=element.link)
    elif shape == 'convex':
        element.geometry.to_convex_hull()
        element.geometry.scale = scale
        element.origin = representation.Pose.from_matrix(np.identity(4), relative_to=element.link)
    else:
        raise Exception('Shape {} not implemented. Please choose sphere, cylinder, box or convex.'.format(shape))
    return


def join_collisions(robot, linkname, collisionnames=None, name_id=None, only_return=False):
    """Replaces a series of visuals/collisions with a joined version of those given"""
    link = robot.get_link(linkname)
    assert link is not None

    if collisionnames is not None:
        elements = [e for e in link.collisions if e.name in collisionnames]
    else:
        elements = [e for e in link.collisions]

    if len(elements) <= 1:
        return

    meshes = []
    names = []
    primitives = []
    file_types = set()
    for e in elements:
        if not isinstance(e.geometry, representation.Mesh):
            primitives += [e]
            continue
        else:
            mesh = io.as_trimesh(e.geometry.load_mesh(), silent=True)
            name = e.name
            if name.lower().startswith("collision_"):
                name = name[len("collision_"):]
            if name.lower().endswith("_collision"):
                name = name[:-len("_collision")]
            name = str.replace(name, "/", "")
            names.append(name)
            mesh.apply_transform(e.origin.to_matrix())
            meshes.append(mesh)
            file_types.add(e.geometry.input_type)
            if not only_return:
                link.remove_aggregate(e)

    mesh = trimesh.util.concatenate(meshes)
    if only_return:
        return mesh

    filename = "_".join(names) + "_joined"
    if name_id is not None:
        filename = "_".join([str.replace(linkname, "/", ""), name_id, "joined"])

    mesh_representation = representation.Mesh(meshname=filename, mesh=mesh)
    if len(file_types) == 1:
        mesh_representation.input_type = list(file_types)[0]
    link.add_aggregate("collision", representation.Collision(
        origin=representation.Pose(rpy=[0, 0, 0], xyz=[0, 0, 0], relative_to=link),
        geometry=mesh_representation,
        name="collision_"+link.name
    ))


def replace_collisions(robot, shape='box', oriented=False, exclude=None):
    """Replace all collisions of the robot ( except exclude's ) with a given shape. This can be
    a 'sphere', 'cylinder', 'box' or 'convex'.
    """
    if exclude is None:
        exclude = []
    for link in robot.links:
        if link.name not in exclude:
            replace_collision(robot, link.name, shape=shape, oriented=oriented)
    return


def replace_collision(robot, linkname, shape='box', oriented=False, scale=1.0):
    """Replace the collision(s) stated in linkname of the robot with an oriented shape.
    """
    if isinstance(linkname, list):
        [replace_collision(robot, lname, shape=shape, oriented=oriented, scale=scale) for lname in linkname]
        return

    assert type(linkname) is str
    link = robot.get_link(linkname)
    if link and hasattr(link, 'collision'):
        # print("Processing {}...".format(link.name))
        geometry.replace_geometry(link.collisions, shape=shape, oriented=oriented, scale=scale)
    return


def reduce_mesh_collision(robot, linkname, collisionname=None, reduction=0.4):
    """
    Reduces the mesh collision(s) of the link(s) which have the corresponding collisionname.
    """
    if isinstance(linkname, list):
        [remove_collision(robot, lname, collisionname=collisionname) for lname in linkname]
        return
    if collisionname is not None and not isinstance(collisionname, list):
        collisionname = [collisionname]

    assert type(linkname) is str
    link = robot.get_link(linkname)
    if link is not None:
        for c in link.collisions:
            if (collisionname is None or c.name in collisionname) and isinstance(c.geometry, representation.Mesh):
                c.geometry.reduce_mesh(reduction)


def remove_collision(robot, linkname, collisionname=None):
    """Remove the collision(s) of the link(s) which have the corresponding collisionname.
    """
    if isinstance(linkname, list):
        [remove_collision(robot, lname, collisionname=collisionname) for lname in linkname]
        return
    if collisionname is not None and not isinstance(collisionname, list):
        collisionname = [collisionname]

    assert type(linkname) is str
    link = robot.get_link(linkname)
    if link is not None:
        # print("Processing {}...".format(link.name))
        if collisionname is not None:
            existing_colls = [c.name for c in link.collisions]
            for cname in existing_colls:
                if cname in collisionname:
                    link.remove_aggregate(robot.get_collision_by_name(cname))
        else:
            while len(link.collisions) != 0:
                link.remove_aggregate(link.collisions[0])
    return


def replace_visuals(robot, shape='box', oriented=False, exclude=None):
    """Replace all visuals of the robot ( except exclude's ) with a given shape. This can be
    a 'sphere', 'cylinder', 'box' or 'convex'.
    """
    if exclude is None:
        exclude = []
    for link in robot.links:
        if hasattr(link, 'visual') and link.name not in exclude:
            # print("Processing {}...".format(link.name))
            geometry.replace_geometry(link.visual, shape=shape, oriented=oriented)
    return


def replace_visual(robot, linkname, shape='box', oriented=False):
    """Replace the visual(s) stated in linkname of the robot with an oriented shape.
    """
    if isinstance(linkname, list):
        [replace_visual(robot, lname, shape=shape, oriented=oriented) for lname in linkname]
        return
    assert type(linkname) is str
    link = robot.get_link(linkname)
    if link and hasattr(link, 'visual'):
        # print("Processing {}...".format(link.name))
        geometry.replace_geometry(link.visual, shape=shape, oriented=oriented)
    return


def remove_visual(robot, linkname, visualname=None):
    """Remove the visual(s) of the link(s) which have the corresponding visualname.
    """
    if isinstance(linkname, list):
        [remove_visual(robot, lname, visualname=visualname) for lname in linkname]
        return
    if visualname is not None and not isinstance(visualname, list):
        visualname = [visualname]

    assert type(linkname) is str
    link = robot.get_link(linkname)
    if link is not None:
        # print("Processing {}...".format(link.name))
        if visualname is not None:
            existing_vis = [v.name for v in link.visuals]
            for cname in existing_vis:
                if cname in visualname:
                    link.remove_aggregate(robot.get_visual_by_name(cname))
        else:
            while len(link.visuals) != 0:
                link.remove_aggregate(link.visuals[0])
    return
