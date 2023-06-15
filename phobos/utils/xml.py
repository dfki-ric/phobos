import os

import numpy as np

from . import misc
from ..commandline_logging import get_logger
from ..defs import IMPORT_TYPES
from ..io import representation

log = get_logger(__name__)


def get_joint_info_dict(robot, joint_list):
    """
    Gets the joint information used for joint_limits file from the robot
    """
    out = {"names": [], "elements": []}
    for joint in sorted(joint_list):
        j = robot.get_joint(joint)
        if j is None:
            raise Exception("Joint of name "+joint+" not found in robot!")
        if j.joint_type != "fixed":
            out["names"] += [j.name]
            if j.limit is None:
                log.warning(f"No joint limits defined for joint {j.name} (type: {j.joint_type})")
            out["elements"] += [{
                # "name" : j,
                "max": {
                    "position": j.limit.upper if j.limit is not None else 0,
                    "speed": j.limit.velocity if j.limit is not None else 0,
                    "effort": j.limit.effort if j.limit is not None else 0
                },
                "min": {"position": j.limit.lower if j.limit is not None else 0}
            }]
    return out


def sort_children_by(parent, attr):
    """Recursively sorts the children of the parent by the given attr.ibute"""
    if len(parent) <= 1 or parent[0].get(attr) is None:
        return
    parent[:] = sorted(parent, key=lambda ch: ch.get(attr))
    for child in parent:
        sort_children_by(child, attr)


def transform_object(obj, T, relative_to):
    """ Transform a given object with a given homogeneous transformation T.
    """
    if isinstance(obj, list):
        for o in obj:
            assert transform_object(o, T)
        return True

    if obj is None or not hasattr(obj, "origin") or obj.origin is None:
        return False

    obj.origin.transformed_by(T)
    return True


def adapt_mesh_pathes(robot, new_urdf_dir, copy_to=None):
    # [TODO pre_v2.0.0] Check whether this has become obsolete due to export adaption
    for link in robot.links:
        for geo in link.visuals + link.collisions:
            if isinstance(geo.geometry, representation.Mesh):
                new_mesh_path = read_relative_filename(geo.geometry.filename, robot.xmlfile)
                if copy_to is not None:
                    new_mesh_path = os.path.join(copy_to,
                                                 os.path.basename(geo.geometry.filename).lower().split(".")[-1],
                                                 os.path.basename(geo.geometry.filename)
                                                 )
                    misc.copy(
                        None,
                        os.path.join(os.path.dirname(robot.xmlfile), geo.geometry.filename),
                        new_mesh_path,
                        silent=True
                    )
                geo.geometry.filename = os.path.relpath(new_mesh_path, new_urdf_dir)


def read_relative_filename(filename, start_file_path):
    if start_file_path is None or os.path.isabs(filename):
        return filename
    if not os.path.isabs(start_file_path):
        start_file_path = os.path.abspath(start_file_path)
    if start_file_path.split(".")[-1] in IMPORT_TYPES:
        start_file_path = os.path.dirname(start_file_path)  # /bla/blub/xyz/blib.xyz -> /bla/blub/xyz
    if filename.startswith("package://"):  # ROS Package
        if os.path.basename(start_file_path) in IMPORT_TYPES+["xacro"]:
            package_dir = os.path.dirname(start_file_path)  # /bla/blub/xyz -> /bla/blub
        else:
            raise IOError("Can't derive package_dir from " + start_file_path)
        out = os.path.join(package_dir, filename[len("package://"):].split("/", 1)[-1])
        assert os.path.exists(out), f"Couldn't identify file from {filename}. Tried {out} but it doesn't exist!"
    elif os.path.isabs(filename):
        out = filename
    else:  # normal urdf
        out = os.path.join(start_file_path, filename)
    return os.path.normpath(out)
