import json
import os
import shutil
from copy import deepcopy
from xml.etree import ElementTree as ET

import numpy
import numpy as np
import trimesh
import traceback

from .base import Representation
from .smurf_reflection import SmurfBase
from .xml_factory import singular as _singular, plural as _plural
from .yaml_reflection import to_yaml
from ..defs import BPY_AVAILABLE
from ..geometry import io as mesh_io
from ..geometry.geometry import identical, reduce_mesh, get_reflection_matrix, improve_mesh
from ..utils import misc, git, transform
from ..utils.transform import inv
from ..utils.xml import read_relative_filename

MESH_INFO_KEYS = ["vertex_normals", "texture_coords", "vertices", "faces"]
MESH_DATA_TYPES = ["trimesh.base.Trimesh", "trimesh.scene.scene.Scene", "file_obj", "file_stl", "file_dae", "file_iv"]
if BPY_AVAILABLE:
    import bpy

    def _bpy_mesh_dc(inst, memo={}):
        log.debug(f"Skipping deepcopy of {type(inst)}")
        return inst

    setattr(bpy.types.Mesh, "__deepcopy__", _bpy_mesh_dc)
    setattr(bpy.types.Image, "__deepcopy__", _bpy_mesh_dc)

    del _bpy_mesh_dc

    MESH_DATA_TYPES += ["bpy_types.Mesh"]

from ..commandline_logging import get_logger
log = get_logger(__name__)


def _joint_relative_origin_getter(instance):
    assert instance._related_robot_instance is not None
    assert instance.origin is not None
    assert instance.origin.relative_to is not None
    out = instance.origin
    if instance.origin.relative_to == instance._related_robot_instance.get_parent(instance.link):
        return out
    if instance.origin.relative_to != instance.link:
        assert instance._related_robot_instance is not None, "Trying to get joint_relative_origin while robot is not linked"
        r2x = instance._related_robot_instance.get_transformation
        out = Pose.from_matrix(
            inv(r2x(instance.link)).dot(r2x(instance.origin.relative_to).dot(instance.origin.to_matrix())),
            relative_to=self.link
        )
        out.link_with_robot(instance._related_robot_instance)
    if instance._link.origin is not None:
        out = instance._link.origin.dot(out)
    out.relative_to = instance._related_robot_instance.get_parent(instance.link)
    return out


def _joint_relative_origin_setter(instance, value):
    """This setter should only be used from the urdf import"""
    value = _singular(value)
    assert isinstance(value, Pose), type(value)
    if value.relative_to is None:  # urdf, as sdf provides this value
        instance.origin = value
        if instance._related_robot_instance is not None:
            instance.origin = instance._link.origin.inv().dot(instance.origin)
        instance.origin.relative_to = instance.link


__IMPORTS__ = [x for x in dir() if not x.startswith("__")]


class Pose(Representation, SmurfBase):
    _class_variables = ["xyz", "rpy", "relative_to"]

    def __init__(self, xyz=None, rpy=None, vec=None, relative_to=None, **kwargs):
        Representation.__init__(self)
        SmurfBase.__init__(self, returns=["rotation", "position", "relative_to"])
        self.excludes += ["xyz", "rpy"]
        self.relative_to = relative_to
        self._matrix = np.identity(4)
        if "matrix" in kwargs:
            self._matrix = kwargs["matrix"]
            return
        if vec is not None:
            assert xyz is None and rpy is None
            assert "rotation" not in kwargs and "position" not in kwargs
            assert isinstance(vec, list)
            if len(vec) == 3:
                xyz = np.array(vec)
            else:
                self.from_vec(vec)
                return
        if xyz is not None:
            self.position = xyz
        elif "position" in kwargs:
            self.position = kwargs["position"]
        if rpy is not None:
            self.rotation = rpy
        elif "rotation" in kwargs:
            self.rotation = kwargs["rotation"]

    def check_linkage(self, attribute=None):
        if self.relative_to is None:
            log.error("Pose without definition for relative_to during check_linkage")
        return self.relative_to is not None and super(Pose, self).check_linkage(attribute=attribute)

    def is_zero(self):
        return all((self._matrix == np.identity(4)).flatten())

    # Aliases for backwards compatibility
    @property
    def xyz(self):
        return np.array(self.position).tolist()

    @xyz.setter
    def xyz(self, value):
        self.position = value

    @property
    def rpy(self):
        return np.array(self.rotation).tolist()

    @rpy.setter
    def rpy(self, value):
        self.rotation = value

    @property
    def rotation(self):
        return transform.matrix_to_rpy(self._matrix[0:3, 0:3])

    @rotation.setter
    def rotation(self, value):
        if type(value) in [int, float]:
            self._matrix[0:3, 0:3] = transform.rpy_to_matrix([0, 0, value])
        elif type(value) in [list, tuple, np.ndarray] and len(value) == 3:
            self._matrix[0:3, 0:3] = transform.rpy_to_matrix(value)
        elif BPY_AVAILABLE and hasattr(value, "__len__") and type(value) != str and len(value) == 3:
            self._matrix[0:3, 0:3] = transform.rpy_to_matrix(np.array(value))
        elif type(value) in [list, tuple, np.ndarray] and len(value) == 4:
            self._matrix[0:3, 0:3] = transform.quaternion_to_matrix(value)
        elif BPY_AVAILABLE and hasattr(value, "__len__") and type(value) != str and len(value) == 4:
            self._matrix[0:3, 0:3] = transform.rpy_to_matrix(np.array(value))
        elif type(value) == dict and len(value) == 3:
            if all([k in "rpy" for k in value.keys()]):
                self._matrix[0:3, 0:3] = transform.rpy_to_matrix([value["r"], value["p"], value["y"]])
            elif all([k in "xyz" for k in value.keys()]):
                self._matrix[0:3, 0:3] = transform.rpy_to_matrix([value["x"], value["y"], value["z"]])
            else:
                raise ValueError("Can't parse rotation" + str(value))
        elif type(value) == dict and len(value) == 4:
            self._matrix[0:3, 0:3] = transform.quaternion_to_matrix([value["x"], value["y"], value["z"], value["w"]])
        elif type(value) in [list, np.ndarray]:
            self._matrix[0:3, 0:3] = transform.matrix_to_rpy(value)
        elif value is None:
            self._matrix[0:3, 0:3] = numpy.identity(3)
        else:
            raise ValueError("Can't parse rotation " + str(value))
        # if we have an pi or pi/2, pi/4 approximation let's make pi or pi/2, pi/4 out of it
        # [TODO v2.1.0] re-establich this
        # for i in range(3):
        #     if type(self.rpy[i]) in [float, np.float64, np.float32] and "e" not in str(self.rpy[i]).lower():
        #         in_decimals = len(str(self.rpy[i]).split(".")[1])
        #         if in_decimals >= 3:
        #             for div in [1, 2, 4]:
        #                 if str(self.rpy[i]) == f"%.{in_decimals}f" % (np.pi / div) or \
        #                        self.rpy[i] == np.round(np.pi/div, decimals=in_decimals) or \
        #                        self.rpy[i] == trunc(np.pi/div, decimals=in_decimals):
        #                     self.rpy[i] = np.pi / div

    @property
    def quaternion(self):
        return transform.matrix_to_quaternion(self._matrix[0:3, 0:3])

    @property
    def quaternion_dict(self):
        return {k: v for k,v in zip("xyzw", transform.matrix_to_quaternion(self._matrix[0:3, 0:3]))}

    @property
    def angle_axis(self):
        return transform.quaternion_to_angle_axis(self.quaternion)

    @property
    def position(self):
        return self.to_matrix()[0:3, 3]

    @position.setter
    def position(self, value):
        if value is None:
            self._matrix[0:3, 3] = np.array([0.0, 0.0, 0.0])
        else:
            assert (type(value) in [list, np.ndarray, tuple] or (hasattr(value, "__len__") and type(value) != str)) and len(value) == 3
            self._matrix[0:3, 3] = np.array(value)

    def from_vec(self, vec):
        assert len(vec) == 6, "Invalid length"
        self.position = np.array(vec[:3])
        self.rotation = np.array(vec[3:6])

    @property
    def vec(self):
        xyz = np.array(self.xyz) if self.xyz is not None else np.array([0.0, 0.0, 0.0])
        rpy = np.array(self.rpy) if self.rpy is not None else np.array([0.0, 0.0, 0.0])
        return xyz.tolist() + rpy.tolist()

    @staticmethod
    def from_matrix(T, relative_to):
        return Pose(matrix=np.array(T), relative_to=relative_to)

    def to_matrix(self):
        return self._matrix

    def transformed_by(self, T, relative_to):
        """T.dot(this)"""
        return Pose.from_matrix(
            T.dot(self._matrix),
            relative_to
        )

    def transformed_relative_to(self, relative_to):
        assert self._related_robot_instance is not None
        assert self.relative_to is not None
        r2x = self._related_robot_instance.get_transformation
        return Pose.from_matrix(
            inv(r2x(relative_to)).dot(r2x(self.relative_to).dot(self._matrix)),
            relative_to=relative_to
        )

    def inv(self, relative_to):
        return Pose.from_matrix(
            inv(self._matrix),
            relative_to=relative_to
        )

    def dot(self, other):
        return Pose.from_matrix(
            self._matrix.dot(other.to_matrix()),
            relative_to=self.relative_to
        )

    def stringable(self):
        return False


class Texture(Representation):
    def __init__(self, name=None, filepath=None, posix_path=None, image=None, **kwargs):
        super(Texture, self).__init__()
        self.unique_name = name
        filepath = filepath if posix_path is None else posix_path
        self.input_file = None
        if image is not None:
            self.unique_name = None
            self.image = image
            self.input_type = "img"
            if BPY_AVAILABLE and isinstance(image, bpy.types.Image):
                self.input_file = os.path.normpath(bpy.path.abspath(misc.sys_path(image.filepath)))
                self.input_type = "img_bpy"
                if self.unique_name is None:
                    self.unique_name, ext = os.path.splitext(image.name)
            elif hasattr(image, "filename"):
                # assumes PIL/Pillow image
                self.input_file = os.path.expanduser(misc.sys_path(image.filename))
        elif filepath is not None:
            self.input_file = misc.sys_path(filepath)
            self.input_type = "file"
            self.image = None

        if self.input_file is not None:
            if not os.path.isabs(self.input_file) and "_xmlfile" in kwargs and kwargs["_xmlfile"] is not None:
                self.input_file = os.path.normpath(os.path.join(os.path.dirname(kwargs["_xmlfile"]), self.input_file))
            elif not os.path.isabs(self.input_file) and "_smurffile" in kwargs and kwargs["_smurffile"] is not None:
                self.input_file = os.path.normpath(os.path.join(os.path.dirname(kwargs["_smurffile"]), self.input_file))
            if not os.path.isabs(self.input_file) or not os.path.isfile(self.input_file):
                raise IOError(f"Texture file {self.input_file} couldn't be found!")

            if self.unique_name is None:
                self.unique_name, ext = os.path.splitext(os.path.basename(self.input_file))
        else:
            log.warn("Instantiated an empty texture")

        self.exported = None

    def stringable(self):
        # [TODO v2.1.0]
        # Eventhough we can create a string of a texture, it's not possible yet to have this mesh as link in the robot
        # This is still an open to do, which requires to deepcopy meshes on demand when one instance is changed in a
        # way that is not applicable to other usages of this mesh.
        return False

    def __str__(self):
        return self.filepath

    @property
    def abs_filepath(self):
        if self.exported is None:
            return self.input_file
        else:
            return self.exported

    @property
    def filepath(self):
        if self._related_robot_instance is not None and self._related_robot_instance.xmlfile is not None:
            return os.path.relpath(self.abs_filepath, os.path.dirname(self._related_robot_instance.xmlfile))
        elif self._related_robot_instance is not None and self._related_robot_instance.smurffile is not None:
            return os.path.relpath(self.abs_filepath, os.path.dirname(self._related_robot_instance.smurffile))
        else:
            return self.abs_filepath

    @property
    def posix_path(self):
        return misc.posix_path(self.filepath)

    def load_image(self):
        if BPY_AVAILABLE:
            self.image = bpy.data.images.load(self.input_file)
        else:
            from PIL import Image
            self.image = Image.open(self.input_file)
        return self.image

    def provide_image_file(self, targetpath, format=None):
        if format is None:
            format = "JPG"
        assert format in ["JPG", "PNG"]
        ext = format.lower()
        os.makedirs(targetpath, exist_ok=True)
        targetpath = os.path.join(targetpath, self.unique_name+"."+ext)
        if os.path.isfile(targetpath):
            log.info(f"Texture file {targetpath} already exists, considering this as already exported")
            self.exported = targetpath
            return
        if BPY_AVAILABLE and self.input_type == "img_bpy":
            self.image.filepath_raw = targetpath
            self.image.file_format = format
            self.image.save()
        else:
            # assumes PIL/Pillow image
            self.image.save(targetpath)
        self.exported = targetpath
        return

    def equivalent(self, other):
        return self == other or (
            self.input_file == other.input_file or
            self.abs_filepath == self.abs_filepath
        )


class Material(Representation, SmurfBase):
    _class_variables = ["name", "diffuse", "ambient", "emissive", "specular", "diffuseTexture", "normalTexture"]

    def __init__(self, name=None, diffuse=None, ambient=None, specular=None, emissive=None,
                 diffuseTexture=None, normalTexture=None, transparency=None, shininess=None, **kwargs):
        self.diffuse = misc.color_parser(rgba=diffuse)
        self.ambient = misc.color_parser(rgba=ambient)
        self.specular = misc.color_parser(rgba=specular)
        self.emissive = misc.color_parser(rgba=emissive)
        self.diffuseTexture = _singular(diffuseTexture)
        self.normalTexture = _singular(normalTexture)
        self.transparency = transparency
        if self.transparency is None and self.diffuse is not None:
            self.transparency = 1-self.diffuse[3]
        self.shininess = shininess
        self.original_name = name
        Representation.__init__(self)
        SmurfBase.__init__(self, returns=["name", "diffuseColor", "ambientColor", "specularColor", "emissionColor",
                                          "diffuseTexture", "normalTexture"], **kwargs)
        if name is None or len(name) == 0:
            name = misc.to_hex_color(self.diffuse) + (os.path.basename(self.diffuseTexture) if diffuseTexture is not None else "")
        self.name = name
        self.excludes += ["diffuse", "ambient", "specular", "emissive", "original_name", "users"]

    def check_valid(self):
        # [TODO v2.0.0] REVIEW add other colors here
        if self.diffuse is None and self.diffuseTexture is None:
            raise Exception("Material has neither a color nor texture.")

    def equivalent(self, other):
        # [TODO v2.0.0] REVIEW add other colors here
        return self == other or (
                (self.diffuseTexture == other.diffuseTexture or (self.diffuseTexture is not None and self.diffuseTexture.equivalent(self.diffuseTexture))) and
                other.diffuse == self.diffuse
        )

    def is_delegate(self):
        # [TODO v2.0.0] REVIEW add other colors here
        return self.diffuse is None and self.diffuseTexture is None

    @property
    def diffuse_rgb(self):
        return self.diffuse[0:3]

    @property
    def diffuseColor(self):
        return {k: v for k, v in zip(["r", "g", "b", "a"], self.diffuse)} if self.diffuse is not None else None

    @diffuseColor.setter
    def diffuseColor(self, *args, rgba=None):
        self.diffuse = misc.color_parser(*args, rgba=rgba)

    @property
    def ambient_rgb(self):
        return self.ambient[0:3]

    @property
    def ambientColor(self):
        return {k: v for k, v in zip(["r", "g", "b", "a"], self.ambient)} if self.ambient is not None else None

    @ambientColor.setter
    def ambientColor(self, *args, rgba=None):
        self.ambient = misc.color_parser(*args, rgba=rgba)

    @property
    def specular_rgb(self):
        return self.specular[0:3]

    @property
    def specularColor(self):
        return {k: v for k, v in zip(["r", "g", "b", "a"], self.specular)} if self.specular is not None else None

    @specularColor.setter
    def specularColor(self, *args, rgba=None):
        self.specular = misc.color_parser(*args, rgba=rgba)

    @property
    def emissive_rgb(self):
        return self.emissive[0:3]

    @property
    def emissionColor(self):
        return {k: v for k, v in zip(["r", "g", "b", "a"], self.emissive)} if self.emissive is not None else None

    @emissionColor.setter
    def emissionColor(self, *args, rgba=None):
        self.emissive = misc.color_parser(*args, rgba=rgba)


class Box(Representation):
    _class_variables = ["size"]

    def __init__(self, size=None, **kwargs):
        super().__init__()
        self.size = size

    def scale_geometry(self, x=1, y=1, z=1):
        self.size = (v * s for v, s in zip(self.size, [x, y, z]))

    def get_corners(self):

        return [
            np.array([-self.size[0] / 2, -self.size[1] / 2, -self.size[2] / 2]),
            np.array([ self.size[0] / 2, -self.size[1] / 2, -self.size[2] / 2]),
            np.array([-self.size[0] / 2,  self.size[1] / 2, -self.size[2] / 2]),
            np.array([ self.size[0] / 2,  self.size[1] / 2, -self.size[2] / 2]),
            np.array([-self.size[0] / 2, -self.size[1] / 2,  self.size[2] / 2]),
            np.array([ self.size[0] / 2, -self.size[1] / 2,  self.size[2] / 2]),
            np.array([-self.size[0] / 2,  self.size[1] / 2,  self.size[2] / 2]),
            np.array([ self.size[0] / 2,  self.size[1] / 2,  self.size[2] / 2]),
        ]

    def __str__(self):
        return "box"

    @property
    def extent(self):
        return self.size


class Cylinder(Representation):
    _class_variables = ["radius", "length"]

    def __init__(self, radius=0.0, length=0.0, **kwargs):
        super().__init__()
        self.radius = radius
        self.length = length

    def scale_geometry(self, x=1, y=1, z=1):
        assert x == y
        self.radius *= x
        self.length *= z

    def __str__(self):
        return "cylinder"

    @property
    def extent(self):
        return (self.radius, self.radius, self.length)


class Sphere(Representation):
    _class_variables = ["radius"]

    def __init__(self, radius=0.0, **kwargs):
        super().__init__()
        self.radius = radius

    def scale_geometry(self, x=1, y=1, z=1):
        assert x == y == z
        self.radius *= x

    def __str__(self):
        return "sphere"

    @property
    def extent(self):
        return (self.radius, self.radius, self.radius)


class Mesh(Representation, SmurfBase):
    _class_variables = ["material"]

    def __init__(self, filepath=None, posix_path=None, scale=None, mesh=None, meshname=None, material=None,
                 mesh_orientation=None, fast_init=False, **kwargs):
        SmurfBase.__init__(self, returns=["scale", "exported", "unique_name", "imported"])
        self._operations = []
        self._scale = [1.0, 1.0, 1.0]
        if scale is not None and scale != self.scale:
            self.scale = scale
        self._changed = False
        self._info_in_sync = True
        self.material = material
        filepath = misc.sys_path(filepath if posix_path is None else posix_path)
        if mesh is not None:
            assert meshname is not None and type(meshname) == str
            self.original_mesh_name = meshname
            self._mesh_object = mesh
            self.input_type = str(type(mesh))[8:-2]  # handle: <class 'the type name'>
            assert self.input_type in MESH_DATA_TYPES
            self.input_file = None
            self.unique_name = meshname
            self._mesh_information = None
            if isinstance(mesh, trimesh.Trimesh) and not fast_init:
                self._mesh_information = mesh_io.trimesh_2_mesh_info_dict(mesh)
            elif BPY_AVAILABLE and isinstance(mesh, bpy.types.Mesh):
                if not fast_init:
                    self._mesh_information = mesh_io.blender_2_mesh_info_dict(mesh)
                self._operations.append("_initiated_in_blender")
                self._changed = True
        else:
            if kwargs.get("_xmlfile", None) is not None and not os.path.isabs(filepath):
                filepath = read_relative_filename(filepath, kwargs["_xmlfile"])
            if kwargs.get("_smurffile", None) is not None and not os.path.isabs(filepath):
                filepath = read_relative_filename(filepath, kwargs["_smurffile"])
            elif not os.path.isabs(filepath) and os.path.isfile(filepath):
                filepath = os.path.abspath(filepath)
            elif not os.path.isabs(filepath):
                raise AssertionError(f"Can't get the mesh data, as there is no valid file path given. {kwargs}")
            _meshname, meshext = os.path.splitext(os.path.basename(filepath))
            meshext = meshext[1:]  # remove the dot
            if meshname is None:
                meshname = _meshname
            assert type(meshname) == str
            self.original_mesh_name = _meshname
            self._mesh_object = None
            self._mesh_information = None  # this will get the raw information present from file
            self.input_type = "file_"+meshext.lower()
            self.input_file = filepath
            self.unique_name = meshname
        # [ToDo v2.1.0] deal with different obj mesh axes during import and export
        # This is the default definition for stl and obj in general how the internal axis are defined
        # using this convention exporting stl/obj from blender will have the vertices on the same axes as defined by the link frames in urdf/sdf.
        # OSG has for obj a special handling which changes this https://github.com/openscenegraph/OpenSceneGraph/blob/2e4ae2ea94595995c1fc56860051410b0c0be605/src/osgPlugins/obj/ReaderWriterOBJ.cpp#L208
        # This can only be switched off via an ReaderWriter option https://github.com/openscenegraph/OpenSceneGraph/blob/2e4ae2ea94595995c1fc56860051410b0c0be605/src/osgPlugins/obj/ReaderWriterOBJ.cpp#L60
        mars_mesh = self.input_file is not None and "mars_obj" in self.input_file and ".mars.obj" in self.input_file
        self.mesh_orientation = {
            "up": "Z" if not mars_mesh else "Y",
            "forward": "Y" if not mars_mesh else "-Z"
        } if mesh_orientation is None else mesh_orientation
        self._exported = {}
        if self.input_file is not None:
            git_root = git.get_root(os.path.dirname(self.input_file))
            if git_root is not None:
                _, _, url = git.get_repo_data(git_root)
                self.imported = {
                    "remote": url,
                    "commit": git.revision(git_root),
                    "filepath": os.path.relpath(self.input_file, git_root)
                }
            else:
                self.imported = {
                    "filepath": self.input_file
                }
        else:
            self.imported = "input file not known"
        self.history = [f"Instantiated with filepath={filepath}->{self.input_file}, scale={scale}, mesh={mesh}, meshname={meshname}, "
                        f"material={material}, mesh_orientation={mesh_orientation}, {kwargs}"]
        self.excludes += ["history", "input_type", "input_file", "original_mesh_name", "mesh_object"]

    @property
    def mesh_object(self):
        return self._mesh_object

    @mesh_object.setter
    def mesh_object(self, value):
        assert isinstance(value, trimesh.Trimesh) or isinstance(value, trimesh.Scene) or isinstance(value, bpy.types.Mesh)
        self.history.append(f"manually setting mesh_object by value of {type(value)}")
        self._operations.append("_manual_override")
        self._changed = True
        self._info_in_sync = False
        self._mesh_object = value
        self._mesh_information = None

    def stringable(self):
        # [TODO v2.1.0]
        # Eventhough we can create a string of this mesh, it's not possible yet to have this mesh as link in the robot
        # This is still an open to do, which requires to deepcopy meshes on demand when one instance is changed in a
        # way that is not applicable to other usages of this mesh.
        return False

    def __str__(self):
        return self.unique_name

    def set_unique_name(self, value):
        assert type(value) == str
        self.unique_name = value
        self._operations.append({"set_unique_name": [value]})

    def write_history(self, targetpath):
        # log.debug(f"Writing history to {targetpath}_history.log")
        with open(targetpath+"_history.log", "w+") as f:
            f.write("\n".join(self.history))

    @classmethod
    def history_file_exists(cls, targetpath):
        return os.path.isfile(targetpath+"_history.log")

    @classmethod
    def read_history(cls, targetpath):
        if cls.history_file_exists(targetpath):
            with open(targetpath+"_history.log", "r") as f:
                return [l.strip() for l in f.readlines()]
        return None

    @property
    def abs_filepath(self):
        if len(self._exported) == 0:
            out = self.input_file if self.input_file is not None else self._mesh_object.get("input_file", None)
            if out is None:
                raise IOError(f"This mesh ({self.unique_name}) hasn't been exported nor is an input file known, which could be used.")
            return out
        else:
            assert self._related_robot_instance is not None
            assert self._related_robot_instance.mesh_format is not None
            assert self._related_robot_instance.mesh_format != "input_type" or self.input_type.startswith("file"), \
                f"No input file to derive the format from! {self} {self.input_type}"
            format = self._related_robot_instance.mesh_format
            if format == "input_type":
                format = self.input_type[5:]
            if format not in self._exported:
                raise IOError(f"The mesh {self.unique_name} with the required mesh format ({format}) has not yet been exported.")
            return self._exported[format]["filepath"]

    @property
    def filepath(self):
        if self._related_robot_instance is not None and self._related_robot_instance.xmlfile is not None:
            return os.path.relpath(self.abs_filepath, os.path.dirname(self._related_robot_instance.xmlfile))
        elif self._related_robot_instance is not None and self._related_robot_instance.smurffile is not None:
            return os.path.relpath(self.abs_filepath, os.path.dirname(self._related_robot_instance.smurffile))
        else:
            return self.abs_filepath

    @property
    def posix_path(self):
        return misc.posix_path(self.filepath)

    @property
    def exported(self):
        if self._related_robot_instance is not None:
            out = {}
            for fmt, value in self._exported.items():
                out[fmt] = {
                    k: (
                        v if k != "filepath" else
                        os.path.relpath(v, os.path.dirname(getattr(self._related_robot_instance, "smurffile", self._related_robot_instance.xmlfile)))
                    ) for k, v in value.items()
                }
            return out
        else:
            return self._exported

    @property
    def posix_exported(self):
        return {k: misc.posix_path(v) for k, v in self.exported}

    @exported.setter
    def exported(self, value):
        for fmt, info in value.items():
            self._exported[fmt] = {
                k: v
                if k != "filepath" or self._related_robot_instance is None or os.path.isabs(v)
                else os.path.normpath(os.path.join(read_relative_filename(misc.sys_path(v), getattr(self._related_robot_instance, "smurffile", self._related_robot_instance.xmlfile))))
                for k, v in info.items()
            }

    @property
    def input_file_name(self):
        return os.path.splitext(os.path.basename(self.input_file))[0]

    def file_exists(self):
        return os.path.isfile(self.abs_filepath)

    def available(self):
        return self.file_exists() or self.mesh_object is not None

    def is_lfs_checked_out(self):
        if self.input_file is not None and os.path.isfile(self.input_file):
            if misc.is_binary_file(self.input_file):
                return True
            with open(os.path.realpath(self.input_file), "r") as f:
                if f.read().startswith("version https://git-lfs"):
                    log.error(f"LFS not properly checked out. (Mesh {self.input_file})")
                    return False
        return True

    def is_valid(self):
        if self.input_file is None or not os.path.isfile(self.input_file):
            log.error(f"Mesh {self.input_file} does not exist")
            return False
        if not self.is_lfs_checked_out():
            return False
        return self.has_enough_vertices()

    def has_enough_vertices(self):
        self.load_mesh()
        mesh = deepcopy(mesh_io.as_trimesh(self.mesh_object, silent=True))
        zero_transform = transform.create_transformation(xyz=-mesh.centroid)
        mesh.apply_transform(zero_transform)
        if len(mesh.vertices) <= 3:
            log.debug(f"Mesh {self.unique_name} has less than 4 vertices")
            return False
        elif np.linalg.norm(mesh.bounding_box_oriented.primitive.extents) <= 0.01:
            log.debug(f"Mesh {self.unique_name} is smaller than 1cm")
            return False
        return True

    def approx_volume_and_com(self):
        """
        If the mesh is not watertight this method tries to make it a proper volume (see improve_mesh()).
        Then, If the mesh is then watertight will return directly it's volume and com,
        otherwise returns the volume of the convex_hull and the centroid of the mesh.
        (see https://trimsh.org/trimesh.base.html#trimesh.base.Trimesh.centroid)

        Returns:
            tuple of approx. volume (float) and approx. center of mass ((3, ) float)
        """
        self.load_mesh()
        mesh = mesh_io.as_trimesh(self.mesh_object, silent=True)
        if not mesh.is_volume:
            mesh = improve_mesh(mesh)
        if mesh.is_volume:
            return mesh.volume, mesh.center_mass
        else:
            log.warning(f"Mesh {str(self.mesh_object)} is not watertight taking volume of convex_hull, estimating COM as centroid of faces")
            return mesh.convex_hull.volume, mesh.centroid

    def equivalent(self, other):
        return (not self._changed and not other._changed and self.input_file == other.input_file) or \
               identical(mesh_io.as_trimesh(self.mesh_object, silent=True), mesh_io.as_trimesh(other.mesh_object, silent=True))

    def load_mesh(self, reload=False):
        if self.mesh_object is not None and not reload:
            return self.mesh_object
        assert os.path.isfile(self.input_file), f"Mesh with path {self.input_file} wasn't found!"
        assert self.is_lfs_checked_out(), f"LFS file {self.input_file} not properly checked out!"
        log.info(f"--> loading mesh: {self.input_file}")
        if BPY_AVAILABLE:
            for mesh in bpy.data.meshes:
                if mesh.name.startswith(self.unique_name) and mesh.get("input_file", None) == self.input_file:
                    log.info("Found multi-user mesh")
                    self._mesh_object = bpy.data.meshes[self.unique_name]
                    self._operations.append("_loaded_in_blender")
                    self._changed = True
                    break
        if BPY_AVAILABLE and self.mesh_object is None:
            bpy.ops.object.select_all(action='DESELECT')
            if self.input_type == "file_stl":
                bpy.ops.import_mesh.stl(filepath=self.input_file)
                self._mesh_object = bpy.data.meshes[bpy.context.object.data.name]
                bpy.ops.object.delete()
            elif self.input_type == "file_obj":
                bpy.ops.import_scene.obj(filepath=self.input_file,
                                         axis_forward=self.mesh_orientation["forward"],
                                         axis_up=self.mesh_orientation["up"],
                                         use_split_objects=False,
                                         use_split_groups=False,
                                         use_groups_as_vgroups=True,
                                         split_mode="OFF")
                assert len(bpy.context.selected_objects) == 1
                object = bpy.context.selected_objects[0]
                if object.data.name != self.unique_name:
                    object.data.name = self.unique_name
                self._mesh_object = bpy.data.meshes[self.unique_name]
                # with obj file import, blender only turns the object, not the vertices,
                # leaving a rotation in the matrix_basis, which we here get rid of
                bpy.ops.object.transform_apply(rotation=True)
                if isinstance(mesh_io.import_mesh(self.input_file), trimesh.Trimesh):
                    try:
                        self._mesh_information = mesh_io.parse_obj(self.input_file)
                    except Exception as e:
                        traceback.print_exc()
                        log.warning(f"{self.input_file} can't parse obj for bobj conversion.")
                else:
                    log.debug(f"{self.input_file} can't be converted to bobj")
                bpy.ops.object.delete()
            elif self.input_type == "file_dae":
                bpy.ops.wm.collada_import(filepath=self.input_file)
                self.mesh_information = mesh_io.parse_dae(self.input_file)
                delete_objects = []
                mesh_objects = []
                for obj in bpy.context.selected_objects:
                    if obj.type == "MESH":
                        mesh_objects.append(obj)
                    else:
                        delete_objects.append(obj)
                bpy.ops.object.select_all(action='DESELECT')
                for obj in mesh_objects:
                    obj.select_set(True)
                bpy.context.view_layer.objects.active = mesh_objects[0]
                if len(mesh_objects) > 1:
                    bpy.ops.object.join()
                self._mesh_object = bpy.data.meshes.new_from_object(bpy.context.object)
                self._mesh_object.name = self.unique_name
                bpy.ops.object.select_all(action='DESELECT')
                for obj in delete_objects:
                    obj.select_set(True)
                bpy.context.view_layer.objects.active = delete_objects[0]
                bpy.ops.object.delete()
            elif self.input_type == "file_bobj":
                self.mesh_information = mesh_io.parse_bobj(self.input_file)
                self._mesh_object = mesh_io.mesh_info_dict_2_blender(self.unique_name, **self.mesh_information)
                bpy.context.view_layer.objects.active = bpy.data.objects.new(self.unique_name, self.mesh_object)
            self._operations.append("_loaded_in_blender")
            self._mesh_object["input_file"] = self.input_file
            self.changed = True  # as we there might be unnoticed changes by blender
        elif self.mesh_object is None:
            if self.input_type == "file_stl":
                self._mesh_object = mesh_io.import_mesh(self.input_file)
            elif self.input_type == "file_obj":
                self._mesh_object = mesh_io.import_mesh(self.input_file)
                if isinstance(self.mesh_object, trimesh.Trimesh):
                    try:
                        self._mesh_information = mesh_io.parse_obj(self.input_file)
                    except Exception as e:
                        traceback.print_exc()
                        log.warning(f"{self.input_file} can't parse obj for bobj conversion.")
                else:
                    log.debug(f"{self.input_file} can't be converted to bobj")
            elif self.input_type == "file_dae":
                self._mesh_object = mesh_io.import_mesh(self.input_file)
                self._mesh_information = mesh_io.parse_dae(self.input_file)
            elif self.input_type == "file_bobj":
                self._mesh_information = mesh_io.parse_bobj(self.input_file)
                self._mesh_object = mesh_io.mesh_info_dict_2_trimesh(**self._mesh_information)
        self.history.append(f"loaded {'bpy-Mesh' if BPY_AVAILABLE else 'trimesh'} from {self.input_type} {self.input_file}")
        return self.mesh_object

    def provide_mesh_file(self, targetpath, format=None, throw_on_invalid_bobj=False, use_existing=False, apply_scale=False):
        if format is None and self._related_robot_instance is not None:
            format = self._related_robot_instance.mesh_format
        if format in [None, "input_type"] and self.input_type.startswith("file"):
            format = self.input_type[5:]
        if format is None:
            raise AssertionError("To export meshes you have to specify the format. (format=None)")
        assert os.path.isabs(targetpath)
        ext = format.lower()
        os.makedirs(targetpath, exist_ok=True)
        targetpath = os.path.join(targetpath, self.unique_name+"."+ext)
        if use_existing:
            self._exported[ext] = {
                "operations": self._operations,
                "filepath": targetpath,
                "export_operation": "None"
            }
            if not os.path.isfile(targetpath):
                log.warn(f"The file assumed as existing ({targetpath}), not there. Exporting anyways but you might encounter issues.")
            return
        if apply_scale:
            self.apply_scale()
        self.history.append(f"->trying export of {str(self.mesh_object)} to {targetpath}")
        # log.debug(f"Providing mesh {targetpath}...")
        # if there are no changes we can simply copy
        if self.input_file is not None and "file_"+ext == self.input_type and not self._changed:
            if self.input_file == targetpath:
                log.debug(f"Using existing mesh {targetpath}...")
                self._exported[ext] = {
                    "operations": self._operations,
                    "filepath": targetpath,
                    "export_operation": "None"
                }
                self.history.append(f"->target == input == {self.input_file} for ext {ext}")
                self.write_history(targetpath)
                return
            else:
                log.debug(f"Copying mesh {os.path.relpath(self.input_file, os.path.dirname(targetpath))} to {targetpath}...")
                shutil.copyfile(self.input_file, targetpath)
                self._exported[ext] = {
                    "operations": self._operations,
                    "filepath": targetpath,
                    "export_operation": "copy"
                }
                self.history.append(f"->copying {self.input_file} to {targetpath}")
                self.write_history(targetpath)
                return
        # do nothing if the file is already there and identical
        if os.path.isfile(targetpath) and not self._changed:
            existing_mesh = mesh_io.import_mesh(targetpath)
            o_history = self.read_history(targetpath)
            equiv_histories = False
            if o_history is not None:
                equiv_histories = [x for x in o_history[1:] if not x.startswith("->")] == [x for x in self.history[1:] if not x.startswith("->")]
            if existing_mesh is not None and (equiv_histories or mesh_io.identical(mesh_io.as_trimesh(self.load_mesh(), silent=True), existing_mesh)):
                log.debug(f"Skipping export of {targetpath} as the mesh file already exists and is identical")
                self._exported[ext] = {
                    "operations": self._operations,
                    "filepath": targetpath,
                    "export_operation": "None"
                }
                self.write_history(targetpath)
                return
            elif existing_mesh is not None and o_history is not None:
                s_history = "\n".join(self.history)
                log.info(f"This history:\n{s_history}")
                s_history = "\n".join(o_history)
                log.info(f"Existing history:\n{s_history}")
                raise IOError(f"Can't export mesh {self.unique_name} to {targetpath} because there exists already a file with different content.")
            elif existing_mesh is not None:
                # if there is now history file to this mesh file we will overwrite it
                log.debug(f"Will overwrite mesh without history at {targetpath}")
            else:
                # there is a file but it's not a mesh, we'll overwrite it
                pass
        # load mesh if not yet done
        if self.input_type.startswith("file") and self.mesh_object is None:
            self.load_mesh()
        # only export bobj, if it makes sense for the mesh
        if ext == "bobj" and self._mesh_information is None:
            if throw_on_invalid_bobj:
                raise IOError(
                    f"Couldn't provide mesh {self.unique_name} to {targetpath}, because this mesh can't be exported as bobj")
            return
        elif ext == "bobj" and (not self._info_in_sync and "texture_coords" in self._mesh_information):
            if throw_on_invalid_bobj:
                raise IOError(
                    f"Couldn't provide mesh {self.unique_name} to {targetpath}, because this mesh has been edited and thus the textures might have get mixed up.")
            return
        # export
        log.debug(f"Writing {type(self.mesh_object)} to {targetpath}...")
        assert self.mesh_object is not None
        if ext == "bobj" and self.input_type == "file_obj":
            mesh_io.write_bobj(targetpath, **self._mesh_information)
            self._exported[ext] = {
                "operations": self._operations,
                "filepath": targetpath,
                "export_operation": "write_bobj"
            }
            self.history.append(f"->wrote bobj {targetpath} from {self.input_type}")
            self.write_history(targetpath)
            return
        # export for blender
        if BPY_AVAILABLE and isinstance(self.mesh_object, bpy.types.Mesh):
            from ..blender.utils import blender as bUtils
            objname = "tmp_export_"+self.unique_name
            tmpobject = bUtils.createPrimitive(objname, 'box', (1.0, 1.0, 1.0))
            # copy the mesh here
            tmpobject.data = self.mesh_object
            bpy.ops.object.select_all(action='DESELECT')
            tmpobject.select_set(True)
            if ext == 'obj':
                axis_forward = bpy.context.preferences.addons["phobos"].preferences.obj_axis_forward
                axis_up = bpy.context.preferences.addons["phobos"].preferences.obj_axis_up
                bpy.ops.export_scene.obj(
                    filepath=targetpath,
                    use_selection=True,
                    use_normals=True,
                    use_materials=False,
                    use_mesh_modifiers=True,
                    use_blen_objects=False,
                    axis_forward=axis_forward,
                    axis_up=axis_up,
                )
            elif ext == 'stl':
                bpy.ops.export_mesh.stl(filepath=targetpath, use_selection=True, use_mesh_modifiers=True)
            elif ext == 'dae':
                bpy.ops.wm.collada_export(filepath=targetpath, selected=True)
            elif ext == "bobj":
                log.debug(f"Exporting {targetpath} with {len(self.mesh_object.vertices)} vertices...")
                mesh_io.write_bobj(targetpath, **mesh_io.blender_2_mesh_info_dict(self.mesh_object))
            bpy.ops.object.select_all(action='DESELECT')
            tmpobject.select_set(True)
            bpy.ops.object.delete()
            self._exported[ext] = {
                "operations": self._operations,
                "filepath": targetpath,
                "export_operation": "blender_export"
            }
            self.history.append(f"->wrote {ext} to {targetpath} from {self.input_type}")
            self.write_history(targetpath)
            return
        # export from trimesh
        assert isinstance(self.mesh_object, trimesh.Trimesh) or isinstance(self.mesh_object, trimesh.Scene)
        if ext == "dae":
            dae_xml = trimesh.exchange.dae.export_collada(self.mesh_object)
            if self.material is not None:
                dae_xml = dae_xml.decode(json.detect_encoding(dae_xml))
                dae_xml = misc.regex_replace(dae_xml, {
                    "<color>0.0 0.0 0.0 1.0</color>": " <color>" + " ".join(
                        [str(n) for n in self.material.diffuseColor]) + "</color>"})
                with open(targetpath, "w") as f:
                    f.write(dae_xml)
            else:
                with open(targetpath, "wb") as f:
                    f.write(dae_xml)
            exp_op = "trimesh_export"
        elif ext == "bobj":
            assert isinstance(self.mesh_object, trimesh.Trimesh),\
                f"Export to bobj only possible from trimesh.Trimesh not for {type(self.mesh_object)}"
            log.debug(f"Exporting {targetpath} with {len(self.mesh_object.vertices)} vertices...")
            mesh_io.write_bobj(targetpath, **mesh_io.trimesh_2_mesh_info_dict(self.mesh_object))
            exp_op = "write_bobj"
        else:
            self.mesh_object.export(
                file_obj=targetpath
            )
            exp_op = "trimesh_export"
        self.history.append(f"->wrote {ext} to {targetpath} from {self.input_type}")
        self.write_history(targetpath)
        self._exported[ext] = {
            "operations": self._operations,
            "filepath": targetpath,
            "export_operation": exp_op
        }

    # methods that leave the mesh and mesh_info in sync
    @property
    def scale(self):
        return self._scale

    @scale.setter
    def scale(self, scale):
        if type(scale) in [list, tuple]:
            assert len(scale) == 3
            self._scale = scale
        elif type(scale) in [float, int]:
            self._scale = [scale, scale, scale]
        else:
            raise TypeError(f"Can't scale with {scale}, requires list(3) or float")
        self._operations.append({"scale": [self._scale]})

    def multiply_scale(self, factor):
        if type(factor) in [list, tuple, np.ndarray]:
            assert len(factor) == 3
            self.scale = [v * s for v, s in zip(self.scale, factor)]
        elif type(factor) in [float, int]:
            self.scale = [v * factor for v in self.scale]
        else:
            raise TypeError(f"Can't multiply scale with {factor}, requires list(3) or float")

    # methods that make changes on the mesh
    def apply_scale(self):
        self.load_mesh()
        if isinstance(self.mesh_object, trimesh.Trimesh) or isinstance(self.mesh_object, trimesh.Scene):
            self.mesh_object.apply_transform(np.diag(list(self.scale) + [1]))
        elif BPY_AVAILABLE and isinstance(self.mesh_object, bpy.types.Mesh):
            from ..blender.utils import blender as bUtils
            objname = "tmp_export_" + self.unique_name
            tmpobject = bUtils.createPrimitive(objname, 'box', self.scale)
            # copy the mesh here
            tmpobject.data = self.mesh_object
            bpy.ops.object.select_all(action='DESELECT')
            tmpobject.select_set(True)
            bpy.ops.object.transform_apply(scale=True)
            bpy.ops.object.select_all(action='DESELECT')
            tmpobject.select_set(True)
            bpy.ops.object.delete()
        else:
            raise NotImplentedError()

        self._operations.append("apply_scale")
        self.history.append(f"apply_scale {self.scale}")
        self._changed = True
        self.scale = 1
        if self._mesh_information is not None:
            self._mesh_information["vertices"] = self._mesh_information["vertices"] * np.array(self.scale)
            self._mesh_information["vertex_normals"] = self._mesh_information["vertex_normals"] / np.array(self.scale)


    def improve_mesh(self):
        self.load_mesh()
        if isinstance(self.mesh_object, trimesh.Trimesh):
            self._changed = True
            self._info_in_sync = False
            self._mesh_object = improve_mesh(self.mesh_object)
            self._operations.append("improve_mesh")
            self.history.append(f"improved mesh")
        elif isinstance(self.mesh_object, bpy.types.Mesh):
            raise NotImplementedError("Please optimize the mesh directly in blender")
        else:
            log.warning(f"Can only optimize trimesh.Trimesh not {type(self.mesh_object)}")

    def reduce_mesh(self, factor):
        self.load_mesh()
        if isinstance(self.mesh_object, trimesh.Trimesh):
            self._changed = True
            self._info_in_sync = False
            self._mesh_object = reduce_mesh(self.mesh_object, factor)
            self._operations.append({"reduce_mesh": [factor]})
            self.set_unique_name(misc.edit_name_string(self.unique_name, suffix=f"_red{str(factor).replace('.',',')}"))
            self.history.append(f"reduced mesh factor={factor}")
        elif isinstance(self.mesh_object, bpy.types.Mesh):
            raise NotImplementedError("Please reduce the mesh directly in blender")
        else:
            log.warning(f"Can only reduce trimesh.Trimesh not {type(self.mesh_object)}")

    def to_convex_hull(self):
        self.load_mesh()
        if isinstance(self.mesh_object, trimesh.Trimesh) or isinstance(self.mesh_object, trimesh.Scene):
            self._mesh_object = self.mesh_object.convex_hull
            self._mesh_information = mesh_io.trimesh_2_mesh_info_dict(self.mesh_object)
            self._operations.append("to_convex_hull")
            self._changed = True
            self._info_in_sync = False
            self.set_unique_name(misc.edit_name_string(self.unique_name, suffix="_convex"))
            self.history.append(f"to trimesh convex_hull")
        elif BPY_AVAILABLE and isinstance(self.mesh_object, bpy.types.Mesh):
            import bmesh
            bm = bmesh.new()
            bm.from_mesh(self.mesh_object)
            bmesh.ops.convex_hull(bm, bm.verts)
            bm.to_mesh(self.mesh_object)
            bm.free()
            self._operations.append("to_convex_hull")
            self._changed = True
            self._info_in_sync = False
            self.set_unique_name(misc.edit_name_string(self.unique_name, suffix="_convex"))
            self.history.append(f"to bpy convex_hull")
        else:
            log.error(f"Couldn't create convex hull for mesh {self.unique_name}")

    def mirror(self, mirror_transform=None, name_replacements=None):
        self.load_mesh()
        if mirror_transform is None:
            mirror_transform = get_reflection_matrix()
        if not any([x < 0 for x in mirror_transform.diagonal()]):
            raise AssertionError
        if name_replacements is None:
            name_replacements = {}
        if isinstance(self.mesh_object, trimesh.Trimesh) or isinstance(self.mesh_object, trimesh.Scene):
            self._mesh_object = self.mesh_object.apply_transform(mirror_transform)
            self._operations.append({"mirror": [mirror_transform]})
            try:
                self.mesh_object.fix_normals()
            except AttributeError:
                # [TODO v2.1.0] It seems there is an issue in trimesh, which we have to escape here
                pass
            self._mesh_information = mesh_io.trimesh_2_mesh_info_dict(self.mesh_object)
            self._changed = True
            self._info_in_sync = False
            self.set_unique_name(misc.edit_name_string(self.unique_name, suffix="_mirrored", replacements=name_replacements))
            self.history.append(f"trimesh mirrored {['{:0.2f}'.format(x) for x in mirror_transform.diagonal()]}")
        elif BPY_AVAILABLE and isinstance(self.mesh_object, bpy.types.Mesh):
            import bmesh
            import mathutils
            bm = bmesh.new()
            bm.from_mesh(self.mesh_object)
            bmesh.ops.mirror(bm, bm.faces, mathutils.Matrix(mirror_transform))
            bm.to_mesh(self.mesh_object)
            bm.free()
            self._operations.append({"mirror": [mirror_transform]})
            self._changed = True
            self._info_in_sync = False
            self._mesh_information = mesh_io.blender_2_mesh_info_dict(self.mesh_object)
            self.set_unique_name(misc.edit_name_string(self.unique_name, suffix="_mirrored", replacements=name_replacements))
            self.history.append(f"bpy mirrored {['{:0.2f}'.format(x) for x in mirror_transform.diagonal()]}")
        else:
            log.error(f"Couldn't create convex hull for mesh {self.unique_name}")

    def to_trimesh_mesh(self):
        self.load_mesh()
        if not isinstance(self.mesh_object, trimesh.Trimesh):
            self._changed = True
            self._info_in_sync = False
            self.history.append(f"converted from {type(self.mesh_object)} to trimesh")
            self._mesh_object = mesh_io.as_trimesh(self.mesh_object)
            self._operations.append("to_trimesh_mesh")

    @property
    def x3d_vertices(self):
        self.load_mesh()
        if self.mesh_object is None:
            return None
        tm = mesh_io.as_trimesh(self.mesh_object, silent=True)
        return np.array(tm.vertices).flatten()

    @property
    def x3d_face_normals(self):
        self.load_mesh()
        if self.mesh_object is None:
            return None
        tm = mesh_io.as_trimesh(self.mesh_object, silent=True)
        return np.array(tm.face_normals).flatten()

    @property
    def x3d_faces(self):
        self.load_mesh()
        if self.mesh_object is None:
            return None
        tm = mesh_io.as_trimesh(self.mesh_object, silent=True)
        faces = np.array(tm.faces)
        return np.c_[faces, [-1]*faces.shape[0]].flatten()


class GeometryFactory(Representation):
    @classmethod
    def create(cls, *args, **kwargs):
        if kwargs["type"] == "mesh":
            return Mesh(**kwargs)
        elif kwargs["type"] == "box":
            return Box(**kwargs)
        elif kwargs["type"] == "cylinder":
            return Cylinder(**kwargs)
        elif kwargs["type"] == "sphere":
            return Sphere(**kwargs)
        raise Exception("Can't create geometry from: "+repr(kwargs))


class Collision(Representation, SmurfBase):
    _class_variables = ["name", "link", "geometry", "origin", "bitmask", "primitive"]

    def __init__(self, name=None, link=None, geometry=None, origin=None, bitmask=None, noDataPackage=False,
                 reducedDataPackage=False, ccfm=None, primitive=None, **kwargs):
        _parent_xml = kwargs.get("_parent_xml", None)
        if _parent_xml is not None and link is None:
            link = _parent_xml.attrib.get("name")
        self.original_name = name
        self.primitive = _plural(primitive)
        if name is None or len(name) == 0:
            if link is not None:
                name = str(link) + "_collision"
            elif "_parent_xml" in kwargs:
                link = kwargs["_parent_xml"].attrib["name"]
                name = link + "_collision"
            else:
                name = None
        self.link = link
        self.name = name
        if origin is None:
            origin = Pose()
        self.origin = _singular(origin)
        if self.origin.relative_to is None:
            self.origin.relative_to = self.link
        SmurfBase.__init__(self, returns=['name', 'link', 'geometry', 'primitive'], **kwargs)
        self.geometry = _singular(geometry)
        assert isinstance(self.origin, Pose)
        self.bitmask = bitmask
        if noDataPackage is not None:
            self.noDataPackage = noDataPackage
        if reducedDataPackage is not None:
            self.reducedDataPackage = reducedDataPackage
        if ccfm is not None:
            self.ccfm = ccfm
        if bitmask is not None:
            self.returns += ['bitmask']
        self.excludes += ["origin", "original_name"]

    def add_primitive(self, primitive):
        if type(primitive) in (list, tuple):
            for p in primitive:
                self.add_primitive(p)
        assert isinstance(primitive, Collision)
        assert type(primitive.geometry) in (Box, Sphere, Cylinder)
        self.primitive.append(primitive)

    @property
    def joint_relative_origin(self):
        return _joint_relative_origin_getter(self)

    @joint_relative_origin.setter
    def joint_relative_origin(self, value):
        """This setter should only be used from the urdf import"""
        _joint_relative_origin_setter(self, value)


class Visual(Representation, SmurfBase):
    _class_variables = ["name", "link", "geometry", "material", "origin"]

    def __init__(self, geometry=None, material=None, material_: Material = None, origin=None, name=None, link=None, **kwargs):
        self.original_name = name
        if link is not None:
            link = str(link)
        self.original_name = name
        if name is None or len(name) == 0:
            if link is not None:
                name = str(link) + "_visual"
            elif "_parent_xml" in kwargs:
                link = kwargs["_parent_xml"].attrib["name"]
                name = link + "_visual"
            else:
                name = None
        _parent_xml = kwargs.get("_parent_xml", None)
        if _parent_xml is not None and link is None:
            link = _parent_xml.attrib.get("name")
        self.link = link
        self.name = name
        self.geometry = _singular(geometry)
        material_ = _singular(material_)
        material = _singular(material)
        if type(material) == str and material_ is not None and not material_.is_delegate():
            assert isinstance(material_, Material) and material_.original_name == material
            self.material = material_
        elif isinstance(material, Material):
            assert material_ is None or material_.equivalent(material)
            self.material = material
        elif type(material) == str:
            self.material = material
        else:
            assert material is None and material_ is None
        if origin is None:
            origin = Pose()
        self.origin = _singular(origin)
        if self.origin.relative_to is None:
            self.origin.relative_to = self.link
        SmurfBase.__init__(self, returns=["name", "geometry"], **kwargs)
        assert isinstance(self.origin, Pose)

    @property
    # [TODO v2.0.0] Make the name more self-explanatory, that this is the dummy for exporting only a reference
    def material_(self):
        return None

    def equivalent(self, other):
        return self.geometry.equivalent(other.geometry) and self._material.equivalent(other._material) and \
               self.origin == other.origin

    @property
    def origin_from_root(self):
        assert self._related_robot_instance is not None
        link = [ln for ln in self._related_robot_instance.links if self in ln.visuals]
        assert len(link) == 1
        transformation = self._related_robot_instance.get_transformation(link[0]).dot(self.origin.to_matrix())
        return Pose.from_matrix(transformation)

    @property
    def position_from_root(self):
        return self.origin_from_root.xyz

    @property
    def axis_angle_from_root(self):
        angle, axis = self.origin_from_root.angle_axis
        return [axis[0], axis[1], axis[2], angle]

    @property
    def joint_relative_origin(self):
        return _joint_relative_origin_getter(self)

    @joint_relative_origin.setter
    def joint_relative_origin(self, value):
        """This setter should only be used from the urdf import"""
        _joint_relative_origin_setter(self, value)


class Inertia(Representation, SmurfBase):
    _class_variables = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

    def __init__(self, ixx=1e-16, ixy=0.0, ixz=0.0, iyy=1e-16, iyz=0.0, izz=1e-16, **kwargs):
        super().__init__()
        assert type(ixx) != str and ixx is not None
        self.ixx = ixx
        self.ixy = ixy
        self.ixz = ixz
        self.iyy = iyy
        self.iyz = iyz
        self.izz = izz

    def stringable(self):
        return False

    def to_matrix(self):
        return [[self.ixx, self.ixy, self.ixz],
                [self.ixy, self.iyy, self.iyz],
                [self.ixz, self.iyz, self.izz]]

    def to_list(self):
        return [self.ixx, self.ixy, self.ixz, self.iyy, self.iyz, self.izz]

    @staticmethod
    def from_mass_matrix(M):
        I = M[3::, 3::]
        inertias = {
            'ixx': I[0, 0],
            'ixy': I[0, 1],
            'ixz': I[0, 2],
            'iyy': I[1, 1],
            'iyz': I[1, 2],
            'izz': I[2, 2]
        }

        return Inertia(**inertias)


class Inertial(Representation, SmurfBase):
    _class_variables = ["mass", "inertia", "origin", "link"]

    def __init__(self, mass=0.0, inertia=None, origin=None, link=None, **kwargs):

        self.mass = mass
        self.inertia = _singular(inertia)
        _parent_xml = kwargs.get("_parent_xml", None)
        if _parent_xml is not None and link is None:
            link = _parent_xml.attrib.get("name")
        self.link = link
        if origin is None:
            origin = Pose()
        self.origin = _singular(origin)
        assert type(self.origin) == Pose, f"{origin} is not Pose"
        if self.origin.relative_to is None:
            self.origin.relative_to = self.link
        SmurfBase.__init__(self, **kwargs)
        assert self.origin.relative_to is not None

    def stringable(self):
        return False

    @staticmethod
    def from_mass_matrix(M, origin: Pose, link=None):
        return Inertial(
            mass=M[0, 0],
            link=link,
            inertia=Inertia.from_mass_matrix(M),
            origin=origin
        )

    def to_mass_matrix(self):
        m = self.mass

        I = np.array(self.inertia.to_matrix())

        M = np.zeros((6, 6))
        M[0:3, 0:3] = np.eye(3) * m
        M[3::, 3::] = I
        return M

    @property
    def joint_relative_origin(self):
        return _joint_relative_origin_getter(self)

    @joint_relative_origin.setter
    def joint_relative_origin(self, value):
        """This setter should only be used from the urdf import"""
        _joint_relative_origin_setter(self, value)


class KCCDHull(Representation, SmurfBase):
    _class_variables = ["link", "points", "radius"]

    def __init__(self, points=None, radius=None, frame=None, **kwargs):
        """

        Args:
            points: relative to the link
            radius:
            **kwargs:
        """
        SmurfBase.__init__(returns=["points", "radius", "frame"], **kwargs)
        self._npoints = len(points)
        self.points = points
        self.radius = radius
        self.frame = frame

    @classmethod
    def from_collision(self, coll, npoints=None, iv_mesh=None, kccd_path=None):
        self.frame = coll.link
        if isinstance(coll.geometry, Sphere):
            self._npoints = 1
            self.radius = coll.geometry.radius
            self.points = [coll.origin.position]
        elif isinstance(coll.geometry, Cylinder):
            self._npoints = 2
            self.radius = coll.geometry.radius
            axis = coll.origin.to_matrix()[0:3, 0:3].dot(np.array([coll.geometry.length, 0, 0]))
            self.points = [coll.origin.position - axis, coll.origin.position+axis]
            self.radius = coll.geometry.radius
        elif isinstance(coll.geometry, Box):
            if npoints == 1:
                self._npoints = 1
                self.points = [coll.origin.position]
                self.radius = 2**0.5 * np.linalg.norm(coll.geometry.size)/2
            elif npoints != 8:
                log.warning(f"Can create kccd hull for a box either as sphere with one point or with eight. "
                            f"You asked for {npoints}, falling back to 8.")
            self._npoints = 8
            self.radius = 0.
            self.points = [coll.origin.to_matrix()[0:3, 0:3].dot(c) for c in coll.geometry.get_corners()]
        elif isinstance(coll.geometry, Mesh):
            assert npoints is not None
            assert iv_mesh is not None and os.path.isfile(iv_mesh)
            assert kccd_path is not None and os.path.isdir(kccd_path)
            self._npoints = npoints
            self.points = []
            log.info(f"Covering {iv_mesh} of {self.frame} with {self._npoints}")
            out, _ = misc.execute_shell_command(f"kccdcoveriv {iv_mesh} {self._npoints} {self.frame}",
                                           cwd=kccd_path, silent=True)
            block_lines = out.split("\n")
            for line in block_lines:
                if line.startswith("BODYRADIUS"):
                    self.radius = float(line.split[-1])
                elif line.startswith("BODYPOINT"):
                    line = line.replace("[", "")
                    line = line.replace(",", "")
                    self.points.append(np.array([float(x) for x in line.split()[-3:]]))


class Link(Representation, SmurfBase):
    _class_variables = ["name", "visuals", "collisions", "inertial", "kccd_hull", "origin"]

    def __init__(self, name=None, visuals=None, inertial=None, collisions=None, origin=None,
                 noDataPackage=False, reducedDataPackage=False, is_human=None, kccd_hull=None, joint=None, **kwargs):
        SmurfBase.__init__(self, **kwargs)
        self.name = name
        self.origin = _singular(origin)
        self.is_human = is_human
        self.returns += ['name', "is_human"]
        self.visuals = _plural(visuals)
        self.inertial = _singular(inertial)
        self.collisions = _plural(collisions)
        self.kccd_hull = kccd_hull
        for geo in self.visuals + self.collisions:
            if geo.origin.relative_to is None:
                geo.origin.relative_to = self.name
        if noDataPackage is not None:
            self.noDataPackage = noDataPackage
            self.returns += ['noDataPackage']
        if reducedDataPackage is not None:
            self.reducedDataPackage = reducedDataPackage
            self.returns += ['reducedDataPackage']
        for geo in self.collisions + self.visuals:
            i = 0
            if geo.name is None:
                geo.name = self.name + ("_collision" if isinstance(geo, Collision) else "_visual")
                if i > 0:
                    geo.name += str(i)
        self.excludes += ["inertial"]

    def remove_aggregate(self, elem):
        if isinstance(elem, Visual):
            self.visuals.remove(elem)
        elif isinstance(elem, Collision):
            self.collisions.remove(elem)

    def add_aggregate(self, elem_type, elem):
        if isinstance(elem, Visual) or elem_type.lower() == "visual":
            self.visuals.append(elem)
        elif isinstance(elem, Collision) or elem_type.lower() == "collision":
            self.collisions.append(elem)

    @property
    def materials(self):
        return [visual.material for visual in self.visuals]

    @materials.setter
    def materials(self, value):
        if type(value) == dict:
            for visual in self.visuals:
                if visual.name in value:
                    visual.material = value[visual.name]
        elif type(value) == str:
            for visual in self.visuals:
                visual.material = value

    @property
    def joint_relative_origin(self):
        assert self._related_robot_instance is not None
        assert self.origin is not None
        out = self.origin
        if self.origin.relative_to == self._related_robot_instance.get_parent(self):
            return out
        else:
            assert self.origin.relative_to is not None
            r2x = self._related_robot_instance.get_transformation
            return Pose.from_matrix(
                inv(r2x(self)).dot(r2x(self.origin.relative_to).dot(self.origin.to_matrix())),
                relative_to=self._related_robot_instance.get_parent(self)
            )

    @joint_relative_origin.setter
    def joint_relative_origin(self, value):
        """This setter should only be used from the xml import"""
        assert value is None or isinstance(value, Pose), type(value)
        self.origin = _singular(value)
        assert self.origin is None or self.origin.relative_to is not None


class JointDynamics(Representation):
    def __init__(self, damping=None, friction=None, spring_stiffness=None, spring_reference=None, **kwargs):
        super().__init__()
        self.damping = damping
        self.friction = friction
        self.spring_stiffness = spring_stiffness
        self.spring_reference = spring_reference

    def stringable(self):
        return False

    def is_empty(self):
        return self.damping is None and self.friction is None and self.spring_stiffness is None and self.spring_reference is None


class JointLimit(Representation):
    _class_variables = ["effort", "velocity", "lower", "upper"]

    def __init__(self, effort=None, velocity=None, lower=None, upper=None, **kwargs):
        super().__init__()
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper

    def stringable(self):
        return False

    def is_empty(self):
        return self.effort is None and self.velocity is None and self.lower is None and self.upper is None


class JointMimic(Representation, SmurfBase):
    _class_variables = ["joint", "multiplier", "offset"]

    def __init__(self, joint=None, multiplier=None, offset=None, **kwargs):
        super().__init__()
        self.joint = _singular(joint)
        assert self.joint is not None
        self.multiplier = multiplier
        assert self.multiplier is not None
        self.offset = offset
        assert self.offset is not None
        self.returns += ["joint", "multiplier", "offset"]

    def equivalent(self, other):
        return other.joint == self.joint

    def __eq__(self, other):
        return other.joint == self.joint and other.offset == self.offset and other.multiplier == self.multiplier

    def stringable(self):
        return False

    def is_empty(self):
        return self.joint is None and self.multiplier is None and self.offset is None


class ConstraintAxis(SmurfBase):
    def __init__(self, name, axis, **kwargs):
        kwargs["name"] = name
        kwargs["axis"] = axis
        super(ConstraintAxis, self).__init__(**kwargs)

    def stringable(self):
        return False


class Joint(Representation, SmurfBase):
    TYPES = ['revolute', 'continuous', 'prismatic', 'floating', 'planar', 'fixed']
    ADVANCED_TYPES = ['unknown', "revolute2", "screw", "ball", "universal"]

    type_dict = {
        "parent": "links",
        "child": "links",
        "motor": "motors"
    }
    _class_variables = ["name", "parent", "child", "joint_type", "axis", "limit", "dynamics", "motor", "origin",
                        "joint_dependencies"]

    def __init__(self, name=None, parent=None, child=None, joint_type=None,
                 axis=None, origin=None, limit=None,
                 dynamics=None, safety_controller=None, calibration=None,
                 mimic=None, joint_dependencies=None, motor=None,
                 noDataPackage=False, reducedDataPackage=False, cut_joint=False, constraint_axes=None, **kwargs):
        assert name is not None
        self.name = name
        self.returns = ['name']
        assert parent is not None
        assert child is not None
        assert str(parent) != str(child)
        self.parent = parent if type(parent) == str else parent.name
        assert self.parent is not None
        self.child = child if type(child) == str else child.name
        assert self.child is not None
        self.joint_type = joint_type if joint_type is not None else (kwargs["type"] if "type" in kwargs else None)
        assert self.joint_type is not None, f"Joint type of {self.name} undefined!"
        if axis is not None and np.linalg.norm(axis) != 0.:
            self.axis = (np.array(axis)/np.linalg.norm(axis)).tolist() if joint_type in ['revolute', 'continuous', 'prismatic'] else None
        elif axis is not None and np.linalg.norm(axis) == 0. and joint_type == "fixed":
            log.debug(f'Axis of fixed joint {self.name} is of zero length, setting axis to None!')
            self.axis = None
        elif axis is not None and np.linalg.norm(axis) == 0. and joint_type != "fixed":
            log.error(f'Axis of {joint_type} joint {self.name} is of zero length, setting axis to (0,0,1)!')
            self.axis = [0, 0, 1]
        else:
            self.axis = None
        if origin is None and cut_joint is False:
            origin = Pose(xyz=[0, 0, 0], rpy=[0, 0, 0], relative_to=self.parent)
        self.origin = _singular(origin)
        if self.origin.relative_to is None:
            self.origin.relative_to = self.parent
        self.limit = _singular(limit) if self.joint_type != "fixed" else None
        if joint_dependencies is not None:
            self.joint_dependencies = _plural(joint_dependencies) + _plural(mimic)
        else:
            self.joint_dependencies = _plural(mimic)
        self.cut_joint = cut_joint
        self.constraint_axes = _plural(constraint_axes)
        self.motor = str(motor) if motor is not None else None
        if noDataPackage is not None:
            self.noDataPackage = noDataPackage
            self.returns += ["noDataPackage"]
        if reducedDataPackage is not None:
            self.reducedDataPackage = reducedDataPackage
            self.returns += ["reducedDataPackage"]
        # dynamics
        self.dynamics = _singular(dynamics)
        SmurfBase.__init__(self, **kwargs)
        # [Todo v2.1.0] To safe all information in SMURF we have to add here the transformation from parent_relative_origin, but with the correct key
        self.returns += ["joint_dependencies", "parent", "child"]
        self.excludes += ["limit", "mimic", "axis", "dynamics"]

    def link_with_robot(self, robot, check_linkage_later=False):
        super(Joint, self).link_with_robot(robot, check_linkage_later=True)
        if self.cut_joint and self.origin is None:
            self.origin = Pose.from_matrix(self._related_robot_instance.get_transformation(self.child, self.parent),
                                           relative_to=self.parent)
        for jd in self._joint_dependencies:
            jd.link_with_robot(robot, check_linkage_later=True)
        if self._child.origin is not None and self._child.origin.relative_to is None:
            self._child.origin.relative_to = self
        if not check_linkage_later:
            self.check_linkage()

    def check_linkage(self, attribute=None):
        out = super(Joint, self).check_linkage(attribute=attribute)
        for jd in self._joint_dependencies:
            out &= jd.check_linkage(attribute=attribute)
        return out

    def check_unlinkage(self, attribute=None):
        out = super(Joint, self).check_unlinkage(attribute=attribute)
        for jd in self._joint_dependencies:
            out &= jd.check_unlinkage(attribute=attribute)
        return out

    def unlink_from_robot(self, check_linkage_later=False):
        super(Joint, self).unlink_from_robot(check_linkage_later=True)
        for jd in self._joint_dependencies:
            jd.unlink_from_robot(check_linkage_later=True)
        if not check_linkage_later:
            assert self.check_unlinkage()

    def check_valid(self):
        return (self.joint_type in self.TYPES + self.ADVANCED_TYPES, "Invalid joint type: {}".format(self.joint_type) and  # noqa
                (self._related_robot_instance is None or
                 (self._related_robot_instance.get_link(self.parent) is not None and
                  self._related_robot_instance.get_link(self.child) is not None)))

    @property
    def joint_relative_origin(self):
        assert self.origin is None or self.origin.relative_to is not None
        if self.origin is None:
            return None
        assert self._related_robot_instance is not None, "Trying to get joint_relative_origin while robot is not linked"
        parent_joint_name = self._related_robot_instance.get_parent(self.parent)
        if parent_joint_name is None:
            # This a joint from the root link
            assert str(self.origin.relative_to) == str(self._related_robot_instance.get_root()) == str(self.parent)
            return self.origin
        elif self.origin.relative_to != parent_joint_name:
            r2x = self._related_robot_instance.get_transformation
            out = Pose.from_matrix(
                inv(r2x(parent_joint_name)).dot(r2x(self.origin.relative_to).dot(self.origin.to_matrix())),
                relative_to=parent_joint_name
            )
            out.link_with_robot(self._related_robot_instance)
            return out
        return self.origin

    @joint_relative_origin.setter
    def joint_relative_origin(self, origin):
        origin = _singular(origin)
        assert origin.relative_to is None or origin.relative_to == self.parent
        self.origin = _singular(origin)
        self.origin.relative_to = self.parent

    @property
    def mimic(self):
        if self.joint_dependencies is not None and len(self.joint_dependencies) == 1:
            return self.joint_dependencies[0]
        elif self.joint_dependencies is not None and len(self.joint_dependencies) > 1:
            log.error("Calling mimic on a joint that has more than one dependency, use joint_dependencies instead of mimic")
            return None
        else:
            return None

    @mimic.setter
    def mimic(self, value):
        assert value is None or isinstance(value, JointMimic)
        if value is None and self._joint_dependencies is not None and len(self._joint_dependencies) <= 1:
            self._joint_dependencies = []
        elif isinstance(value, JointMimic) and self._joint_dependencies is not None and len(self._joint_dependencies) == 1:
            self._joint_dependencies[0] = value
        elif isinstance(value, JointMimic) and (self._joint_dependencies is None or len(self._joint_dependencies) == 0):
            self._joint_dependencies = [value]
        else:
            raise ValueError(f"Can not set mimic for joint {str(self)} that depends on multiple joints. "
                             f"Consider using the joint_dependency setter.\n {to_yaml(self._joint_dependencies)}")

    @property
    def springStiffness(self):
        return self.dynamics.spring_stiffness if self.dynamics is not None else None

    @springStiffness.setter
    def springStiffness(self, value):
        if value is None:
            return
        if self.dynamics is None:
            self.dynamics = JointDynamics(spring_stiffness=value)
        else:
            self.dynamics.spring_stiffness = value

    @property  # alias of springStiffness
    def spring_const_constraint_axis1(self):
        return self.springStiffness

    @spring_const_constraint_axis1.setter  # alias of springStiffness
    def spring_const_constraint_axis1(self, value):
        self.springStiffness = value

    @property
    def damping(self):
        return self.dynamics.damping if self.dynamics is not None else None

    @damping.setter
    def damping(self, value):
        if value is None:
            return
        if self.dynamics is None:
            self.dynamics = JointDynamics(damping=value)
        else:
            self.dynamics.damping = value

    @property  # alias of damping
    def damping_const_constraint_axis1(self):
        return self.damping

    @damping_const_constraint_axis1.setter  # alias of damping
    def damping_const_constraint_axis1(self, value):
        self.damping = value

    @property
    def friction(self):
        return self.dynamics.friction if self.dynamics is not None else None

    @friction.setter
    def friction(self, value):
        if value is None:
            return
        if self.dynamics is None:
            self.dynamics = JointDynamics(friction=value)
        else:
            self.dynamics.friction = value

    @property
    def joint_dependencies(self):
        return self._joint_dependencies

    @joint_dependencies.setter
    def joint_dependencies(self, new_val):
        self._joint_dependencies = []
        new_val = _plural(new_val)
        for jd in new_val:
            if type(jd) == dict:
                self._joint_dependencies.append(JointMimic(**jd))
            elif type(jd) == JointMimic:
                self._joint_dependencies.append(jd)
            else:
                raise TypeError(f"Incompatible type for defining a joint dependency: {type(jd)}")
        if self._related_robot_instance is not None:
            for jd in self._joint_dependencies:
                jd.link_with_robot(self._related_robot_instance)


class Interface(Representation, SmurfBase):
    _class_variables = ["name", "origin", "parent"]
    type_dict = {
        "parent": "links"
    }

    def __init__(self, name=None, origin=None, parent=None, type=None, direction=None, **kwargs):
        self.name = name
        assert self.name is not None
        self.type = type
        self.direction = direction
        if origin is None:
            origin = Pose()
        self.origin = _singular(origin)
        self.parent = parent
        assert self.parent is not None
        if self.origin is not None and self.origin.relative_to is None:
            self.origin.relative_to = self.parent
        SmurfBase.__init__(self, returns=["parent", "position", "rotation"], **kwargs)
        self.excludes += ["origin"]

    @property
    def position(self):
        return self.origin.position

    @position.setter
    def position(self, value):
        self.origin.position = value

    @property
    def rotation(self):
        return self.origin.rotation

    @rotation.setter
    def rotation(self, value):
        self.origin.rotation = value


# class PR2Transmission(Representation):
#     def __init__(self, name=None, joint=None, actuator=None, type=None,
#                  mechanicalReduction=1):
#         self.name = name
#         self.type = type
#         self.joint = joint
#         self.actuator = actuator
#         self.mechanicalReduction = mechanicalReduction


class Actuator(Representation):
    _class_variables = ["name", "mechanicalReduction"]

    def __init__(self, name, mechanicalReduction=1, **kwargs):
        super().__init__()
        self.name = name
        self.mechanicalReduction = mechanicalReduction


class TransmissionJoint(Representation):
    _class_variables = ["name", "hardwareInterface"]

    _type_dict = {"name": "joints"}

    def __init__(self, name, hardwareInterfaces=None, **kwargs):
        super().__init__()
        self.name = name
        self.hardwareInterfaces = [] if hardwareInterfaces is None else hardwareInterfaces

    def check_valid(self):
        assert len(self.hardwareInterfaces) > 0, "no hardwareInterface defined"


class Transmission(Representation):
    """ New format: http://wiki.ros.org/urdf/XML/Transmission """
    _class_variables = ["name", "joints", "actuators"]

    def __init__(self, name, joints: TransmissionJoint = None, actuators=None, **kwargs):
        super().__init__()
        self.name = name
        self.joints = [] if joints is None else joints
        self.actuators = [] if actuators is None else actuators

    def check_valid(self):
        assert len(self.joints) > 0, "no joint defined"
        assert len(self.actuators) > 0, "no actuator defined"


class Motor(Representation, SmurfBase):
    BUILD_TYPES = ["DC", "AC", "STEP"]
    TYPES = ["position", "velocity", "force"]
    _class_variables = ["name", "joint"]

    def __init__(self, name=None, joint=None, type="position", **kwargs):
        self.name = name
        self.joint = joint
        SmurfBase.__init__(self, returns=["name", "joint"], **kwargs)
        # This is hardcoded information
        assert self.joint is not None
        self._maxEffort = None
        self._maxSpeed = None
        self._maxValue = None
        self._minValue = None
        self.build_type = kwargs.get("type", None)
        if self.build_type:
            self.build_type = self.build_type.upper()
        self.type = type
        self.returns += ['joint', 'maxEffort', 'maxSpeed', 'maxValue', 'minValue', 'type', "mimic_motor", "mimic_multiplier", "mimic_offset"]

    @property
    def maxEffort(self):
        if self._related_robot_instance is not None:
            return self._joint.limit.effort if self._joint.limit else 0
        else:
            return self._maxEffort

    @maxEffort.setter
    def maxEffort(self, effort):
        if self._related_robot_instance is None:
            self._maxEffort = effort
        else:
            if type(effort) in [float, int] and effort > 0:
                if self._joint is not None and self._joint.limit:
                    self._joint.limit.effort = effort
                else:
                    self._joint.limit = JointLimit(
                        effort=effort,
                        velocity=self.maxSpeed,
                        lower=self.minValue,
                        upper=self.maxValue
                    )

    @property
    def maxValue(self):
        if self._related_robot_instance is not None:
            return self._joint.limit.upper if self._joint.limit else 0
        else:
            return self._maxValue

    @maxValue.setter
    def maxValue(self, maxval):
        if self._related_robot_instance is None:
            self._maxValue = maxval
        else:
            if type(maxval) in [float, int] and maxval >= self.minValue:
                if self._joint and self._joint.limit:
                    self._joint.limit.upper = maxval
                else:
                    self._joint.limit = JointLimit(
                        effort=self.maxEffort,
                        velocity=self.maxSpeed,
                        lower=self.minValue,
                        upper=maxval
                    )

    @property
    def minValue(self):
        if self._related_robot_instance is not None:
            return self._joint.limit.lower if self._joint.limit else 0
        else:
            return self._minValue

    @minValue.setter
    def minValue(self, minval):
        if self._related_robot_instance is None:
            self._minValue = minval
        else:
            if type(minval) in [float, int] and minval <= self.maxValue:
                if self._joint and self._joint.limit:
                    self._joint.limit.lower = minval
                else:
                    self._joint.limit = JointLimit(
                        effort=self.maxEffort,
                        velocity=self.maxSpeed,
                        lower=minval,
                        upper=self.maxValue
                    )

    @property
    def maxSpeed(self):
        if self._related_robot_instance is not None:
            return self._joint.limit.velocity if self._joint.limit else 0
        else:
            return self._maxSpeed

    @maxSpeed.setter
    def maxSpeed(self, speedval):
        if self._related_robot_instance is None:
            self._maxSpeed = speedval
        else:
            if type(speedval) in [float, int] and speedval > 0:
                if self._joint and self._joint.limit:
                    self._joint.limit.velocity = speedval
                else:
                    self._joint.limit = JointLimit(
                        effort=self.maxEffort,
                        velocity=speedval,
                        lower=self.minValue,
                        upper=self.maxValue
                    )

    @property
    def mimic_motor(self):
        if self._related_robot_instance is not None and self._joint.mimic is not None:
            return self._joint.mimic.joint
        return None

    @mimic_motor.setter
    def mimic_motor(self, val):
        pass

    @property
    def mimic_multiplier(self):
        if self._related_robot_instance is not None and self._joint.mimic is not None:
            return self._joint.mimic.multiplier
        return None

    @mimic_multiplier.setter
    def mimic_multiplier(self, val):
        pass

    @property
    def mimic_offset(self):
        if self._related_robot_instance is not None and self._joint.mimic is not None:
            return self._joint.mimic.offset
        return None

    @mimic_offset.setter
    def mimic_offset(self, val):
        pass

    @property
    def plugin_name(self):
        if self.type == "position":
            return "gz::sim::systems::JointPositionController"
        return "gz::sim::systems::JointController"

    @property
    def plugin_filename(self):
        if self.type == "position":
            return "gz-sim-joint-position-controller-system"
        return "gz-sim-joint-controller-system"

    @property
    def force_control(self):
        if self.type == "position":
            return None
        return self.type == "force"


class PluginFactory(Representation):
    @classmethod
    def create(cls, plugin_name, plugin_filename, _xml: ET.Element = None, **kwargs):
        if any(x in plugin_name for x in ["JointController", "JointPositionController"]):
            assert "joint_name" in kwargs
            if "JointPositionController" in plugin_name:
                type = "position"
            elif not kwargs.get("use_force_command", False):
                type = "velocity"
            else:
                type = "force"
            kwargs.pop("use_force_command")
            return Motor(
                name=kwargs["joint_name"]+"_motor",
                type=(
                    "position" if "JointPositionController" in plugin_name
                    else (
                        "velocity" if not kwargs.get("use_force_command", False)
                        else "force"
                    )
                ),
                p=kwargs.pop("p_gain") if "p_gain" in kwargs else None,
                i=kwargs.pop("i_gain") if "i_gain" in kwargs else None,
                d=kwargs.pop("d_gain") if "d_gain" in kwargs else None,
                **kwargs
            )
        log.error(f"Couldn't instantiate plugin from {repr(kwargs)}")
        return None


# [TODO v2.1.0] Check how we can store which properties are defined by which literals to reload them properly from file
class GenericAnnotation(Representation, SmurfBase):
    _class_variables = ["GA_related_link", "GA_related_joint", "GA_related_motor", "GA_related_sensor",
                        "GA_related_visual", "GA_related_collision"]
    _type_dict = {
        "GA_related_link": "link",
        "GA_related_joint": "joint",
        "GA_related_motor": "motor",
        "GA_related_sensor": "sensor",
        "GA_related_visual": "visual",
        "GA_related_collision": "collision",
    }

    def __init__(self, GA_category, GA_name=None, GA_parent=None, GA_parent_type=None, GA_transform: Pose=None,
                 **annotations):
        assert (GA_parent is None and GA_parent_type is None) \
               or GA_parent_type in ["GA_related_"+str(v) for v in self._class_variables],\
            "Unknown GA_parent_type="+str(GA_parent_type)
        setattr(self, "GA_related_"+str(GA_parent_type), GA_parent)
        self._GA_parent_var = "GA_related_"+str(GA_parent_type)
        assert "returns" not in annotations
        self._GA_transform = GA_transform
        self.GA_category = GA_category
        self.GA_name = GA_name

        for k, v in annotations.items():
            setattr(self, "_"+k, v)

            def _getter(instance, varname=k):
                value = getattr(instance, "_" + varname)
                if not self._GA_parent_var.endswith("None") and type(value) == str and value.startswith("$parent."):
                    return getattr(self._GA_parent_var, value[value.find(".") + 1:])
                elif not self._GA_parent_var.endswith("None") and type(value) == str and "$parent" in value:
                    return getattr(self._GA_parent_var, varname)
                elif type(value) == str and "$transform." in value:
                    return getattr(self._GA_transform, value[value.find(".") + 1:])
                elif type(value) == str and "$transform" in value:
                    return self._GA_transform

            def _setter(instance, value, varname=k):
                if "$" in getattr(instance, "_"+varname) and not "$" in value:
                    log.warning(f'{varname} uses the literal: {getattr(instance, "_"+varname)},'
                                f' but you are overriding it with a non literal value: {value}')
                setattr(self, "_"+k, v)

            setattr(self, k, property(_getter, _setter))

        SmurfBase.__init__(self, returns=list(annotations.keys()))

    @property
    def GA_parent(self):
        return getattr(self, self._GA_parent_var)

    def __str__(self):
        if self.GA_name is not None:
            return self.GA_name
        raise NotImplementedError

    def set_unique_name(self, value):
        if self.GA_name is not None:
            self.GA_name = value
        else:
            raise NotImplementedError

    def stringable(self):
        return self.GA_name is not None