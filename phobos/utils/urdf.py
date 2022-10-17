import os
import numpy as np

from . import misc
from ..io import representation
from ..utils.commandline_logging import get_logger
log = get_logger(__name__)


def get_joint_info_dict(robot, joint_list):
    """
    Gets the joint information used for joint_limits file from the urdf of robot
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


def transform_object(obj, T):
    """ Transform a given object with a given homogeneous transformation T.
    """
    if isinstance(obj, list):
        for o in obj:
            assert transform_object(o, T)
        return True

    if obj is None or not hasattr(obj, "origin"):
        return False

    if obj.origin is not None:
        origin = np.matmul(T, obj.origin.to_matrix())
        obj.origin = representation.Pose.from_matrix(origin)
    else:
        obj.origin = representation.Pose.from_matrix(T)
    return True


def adapt_mesh_pathes(robot, new_urdf_dir, copy_to=None):
    for link in robot.links:
        for geo in link.visuals + link.collisions:
            if isinstance(geo.geometry, representation.Mesh):
                new_mesh_path = read_urdf_filename(geo.geometry.filename, robot.xmlfile)
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


def read_urdf_filename(filename, urdf_file_path):
    if urdf_file_path is None or os.path.isabs(filename):
        return filename
    if not os.path.isabs(urdf_file_path):
        urdf_file_path = os.path.abspath(urdf_file_path)
    if urdf_file_path.endswith(".urdf"):
        urdf_file_path = os.path.dirname(urdf_file_path)  # /bla/blub/urdf/blib.urdf -> /bla/blub/urdf
    elif urdf_file_path.endswith(".sdf"):
        urdf_file_path = os.path.dirname(urdf_file_path)  # /bla/blub/sdf/blib.sdf -> /bla/blub/sdf
    if filename.startswith("package://"):  # ROS Package
        if urdf_file_path.endswith("/urdf"):
            package_dir = os.path.dirname(urdf_file_path)  # /bla/blub/urdf -> /bla/blub
        elif urdf_file_path.endswith("/urdf/"):
            package_dir = os.path.dirname(urdf_file_path[:-1])  # /bla/blub/urdf/ -> /bla/blub
        elif urdf_file_path.endswith("/sdf"):
            package_dir = os.path.dirname(urdf_file_path)  # /bla/blub/sdf -> /bla/blub
        elif urdf_file_path.endswith("/sdf/"):
            package_dir = os.path.dirname(urdf_file_path[:-1])  # /bla/blub/sdf/ -> /bla/blub
        else:
            raise IOError("Can't derive package_dir from " + urdf_file_path)
        out = os.path.join(package_dir, filename[len("package://"):])
    elif os.path.isabs(filename):
        out = filename
    else:  # normal urdf
        out = os.path.join(urdf_file_path, filename)
    return os.path.normpath(out)
