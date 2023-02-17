import os
import json
import shutil
from copy import deepcopy

import trimesh
import numpy as np

from .base import Representation
from .smurf_reflection import SmurfBase
from .xml_factory import singular as _singular, plural as _plural
from .yaml_reflection import to_yaml
from ..defs import BPY_AVAILABLE
from ..geometry import io as mesh_io
from ..utils import misc
from ..utils.xml import read_relative_filename
from ..geometry.geometry import identical, reduce_mesh, get_reflection_matrix, improve_mesh
from ..utils.misc import trunc, execute_shell_command, to_hex_color, color_parser, edit_name_string
from ..utils.transform import matrix_to_rpy, round_array, rpy_to_matrix, create_transformation
from ..utils import xml as xml_utils, transform

MESH_INFO_KEYS = ["vertex_normals", "texture_coords", "vertices", "faces"]
MESH_DATA_TYPES = ["trimesh.base.Trimesh", "trimesh.scen.scene.Scene", "file_obj", "file_stl", "file_dae", "file_iv"]
if BPY_AVAILABLE:
    import bpy
    MESH_DATA_TYPES += ["bpy.types.Mesh"]

from ..commandline_logging import get_logger
log = get_logger(__name__)

__IMPORTS__ = [x for x in dir() if not x.startswith("__")]


class Pose(Representation, SmurfBase):
    _class_variables = ["xyz", "rpy", "relative_to"]

    def __init__(self, xyz=None, rpy=None, vec=None, relative_to=None, **kwargs):
        Representation.__init__(self)
        SmurfBase.__init__(self, returns=["rotation", "position"])
        self.excludes += ["xyz", "rpy"]
        self.relative_to = relative_to
        self.xyz = None
        self.rpy = None
        if vec is not None:
            assert xyz is None and rpy is None
            assert "rotation" not in kwargs and "position" not in kwargs
            assert isinstance(vec, list)
            if len(vec) == 3:
                self.position = np.array(vec)
            else:
                self.from_vec(vec)
        else:
            if "position" in kwargs:
                assert xyz is None or xyz == kwargs["position"]
                self.position = kwargs["position"]
            else:
                self.position = xyz
            if "rotation" in kwargs:
                assert rpy is None or rpy == kwargs["rotation"]
                self.rotation = kwargs["rotation"]
            else:
                self.rotation = rpy

    def check_valid(self):
        assert (len(self.xyz) == 3) and \
               (len(self.rpy) == 3)

    # Aliases for backwards compatibility
    @property
    def rotation(self):
        return self.rpy

    @rotation.setter
    def rotation(self, value):
        if type(value) == int:
            self.rpy = [0, 0, value]
        elif type(value) in [list, tuple, np.ndarray] and len(value) == 3:
            self.rpy = value
        elif type(value) in [list, tuple, np.ndarray] and len(value) == 4:
            self.rpy = transform.quaternion_to_rpy(value)
        elif type(value) == dict and len(value) == 3:
            if all([k in "rpy" for k in value.keys()]):
                self.rpy = [value["r"], value["p"], value["y"]]
            elif all([k in "xyz" for k in value.keys()]):
                self.rpy = [value["x"], value["y"], value["z"]]
            else:
                raise ValueError("Can't parse rotation" + str(value))
        elif type(value) == dict and len(value) == 4:
            self.rpy = transform.quaternion_to_rpy([value["x"], value["y"], value["z"], value["w"]])
        elif type(value) in [list, np.ndarray]:
            self.rpy = transform.matrix_to_rpy(value)
        elif value is None:
            self.rpy = [0.0, 0.0, 0.0]
        else:
            raise ValueError("Can't parse rotation " + str(value))
        # if we have an pi or pi/2, pi/4 approximation let's make pi or pi/2, pi/4 out of it
        for i in range(3):
            if type(self.rpy[i]) in [float, np.float64, np.float32] and "e" not in str(self.rpy[i]).lower():
                in_decimals = len(str(self.rpy[i]).split(".")[1])
                if in_decimals >= 3:
                    for div in [1, 2, 4]:
                        if str(self.rpy[i]) == f"%.{in_decimals}f" % (np.pi / div) or \
                               self.rpy[i] == np.round(np.pi/div, decimals=in_decimals) or \
                               self.rpy[i] == trunc(np.pi/div, decimals=in_decimals):
                            self.rpy[i] = np.pi / div
        self.rpy = np.array(self.rpy)

    @property
    def position(self):
        return self.xyz

    @position.setter
    def position(self, value):
        if value is None:
            self.xyz = np.array([0.0, 0.0, 0.0])
        else:
            assert type(value) in [list, np.ndarray] and len(value) == 3
            self.xyz = np.array(value)

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
    def from_matrix(T, dec=16, relative_to=None):
        xyz = T[0:3, 3]
        rpy = matrix_to_rpy(T[0:3, 0:3])
        return Pose(xyz=xyz, rpy=rpy, dec=dec, relative_to=relative_to)

    def to_matrix(self):
        R = rpy_to_matrix(self.rpy if hasattr(self, "rpy") else np.array([0.0, 0.0, 0.0]))
        p = np.array(self.xyz if hasattr(self, "xyz") else np.array([0.0, 0.0, 0.0]))
        T = np.identity(4)
        T[0:3, 3] = p
        T[0:3, 0:3] = R
        T[3, 3] = 1.0
        return T

    def transform(self, T):
        """T.dot(this)"""
        return Pose.from_matrix(
            T.dot(self.to_matrix()),
            self.relative_to
        )

    def stringable(self):
        return False


# [TODO v2.0.0] Handle Textures similarly to meshes

class Material(Representation, SmurfBase):
    _class_variables = ["name", "diffuse", "ambient", "emissive", "specular", "diffuseTexture", "normalTexture"]

    def __init__(self, name=None, diffuse=None, ambient=None, specular=None, emissive=None,
                 diffuseTexture=None, normalTexture=None, **kwargs):
        self.diffuse = color_parser(rgba=diffuse)
        self.ambient = color_parser(rgba=ambient)
        self.specular = color_parser(rgba=specular)
        self.emissive = color_parser(rgba=emissive)
        self.diffuseTexture = diffuseTexture
        self.normalTexture = normalTexture
        self.original_name = name
        SmurfBase.__init__(self, returns=["name", "diffuseColor", "ambientColor", "specularColor", "emissionColor",
                                          "diffuseTexture", "normalTexture"], **kwargs)
        if name is None or len(name) == 0:
            name = to_hex_color(self.diffuse) + (os.path.basename(self.diffuseTexture) if diffuseTexture is not None else "")
        self.name = name
        self.excludes += ["diffuse", "ambient", "specular", "emissive", "original_name", "users"]

    def check_valid(self):
        # [TODO v2.0.0] REVIEW add other colors here
        if self.diffuse is None and self.diffuseTexture is None:
            raise Exception("Material has neither a color nor texture.")

    def equivalent(self, other):
        # [TODO v2.0.0] REVIEW add other colors here
        return other.diffuseTexture == self.diffuseTexture and other.diffuse == self.diffuse

    def is_delegate(self):
        # [TODO v2.0.0] REVIEW add other colors here
        return self.diffuse is None and self.diffuseTexture is None

    @property
    def diffuseColor(self):
        return self.diffuse

    @diffuseColor.setter
    def diffuseColor(self, *args, rgba=None):
        self.diffuse = color_parser(*args, rgba=rgba)

    @property
    def ambientColor(self):
        return self.ambient

    @ambientColor.setter
    def ambientColor(self, *args, rgba=None):
        self.ambient = color_parser(*args, rgba=rgba)

    @property
    def specularColor(self):
        return self.specular

    @specularColor.setter
    def specularColor(self, *args, rgba=None):
        self.specular = color_parser(*args, rgba=rgba)

    @property
    def emissionColor(self):
        return self.emissive

    @emissionColor.setter
    def emissionColor(self, *args, rgba=None):
        self.emissive = color_parser(*args, rgba=rgba)


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

    def __init__(self, filepath=None, scale=None, mesh=None, meshname=None, material=None,
                 mesh_orientation=None,
                 **kwargs):
        SmurfBase.__init__(self)
        self._scale = [1.0, 1.0, 1.0] if scale is None else scale
        self.changed = False
        self.info_in_sync = True
        self.material = material
        if mesh is not None:
            assert meshname is not None
            self._mesh_object = mesh
            self.input_type = str(type(mesh))
            self.input_file = None
            self.unique_name = meshname
            self.mesh_information = None
            if isinstance(mesh, trimesh.Trimesh):
                self.mesh_information = mesh_io.trimesh_2_mesh_info_dict(mesh)
            elif BPY_AVAILABLE and isinstance(mesh, bpy.types.Mesh):
                self.mesh_information = mesh_io.blender_2_mesh_info_dict(mesh)
        else:
            if kwargs.get("xmlfile", None) is not None and not os.path.isabs(filepath):
                filepath = read_relative_filename(filepath, kwargs["xmlfile"])
            if kwargs.get("smurffile", None) is not None and not os.path.isabs(filepath):
                filepath = read_relative_filename(filepath, kwargs["smurffile"])
            elif not os.path.isabs(filepath) and os.path.isfile(filepath):
                filepath = os.path.abspath(filepath)
            elif not os.path.isabs(filepath):
                raise AssertionError("Can't get the mesh data, as there is no valid file path given")
            _meshname, meshext = os.path.splitext(os.path.basename(filepath))
            meshext = meshext[1:]  # remove the dot
            if meshname is None:
                meshname = _meshname
            self._mesh_object = None
            self.mesh_information = None  # this will get the raw information present from file
            self.input_type = "file_"+meshext.lower()
            self.input_file = filepath
            self.unique_name = meshname
        self.original_mesh_name = deepcopy(self.unique_name)
        # [ToDo pre_v2.0.0] deal with different obj mesh axes during import and export
        # This is the default definition for stl and obj in general how the internal axis are defined
        # using this convention exporting stl/obj from blender will have the vertices on the same axes as defined by the link frames in urdf/sdf.
        # OSG has for obj a special handling which changes this https://github.com/openscenegraph/OpenSceneGraph/blob/2e4ae2ea94595995c1fc56860051410b0c0be605/src/osgPlugins/obj/ReaderWriterOBJ.cpp#L208
        # This can only be switched off via an ReaderWriter option https://github.com/openscenegraph/OpenSceneGraph/blob/2e4ae2ea94595995c1fc56860051410b0c0be605/src/osgPlugins/obj/ReaderWriterOBJ.cpp#L60
        mars_mesh = self.input_file is not None and "mars_obj" in self.input_file and ".mars.obj" in self.input_file
        self.mesh_orientation = {
            "up": "Z" if not mars_mesh else "Y",
            "forward": "Y" if not mars_mesh else "-Z"
        } if mesh_orientation is None else mesh_orientation
        self.exported = {}
        self.history = [f"Instantiated id:{id(self)} with filepath={filepath}->{self.input_file}, scale={scale}, mesh={mesh}, meshname={meshname}, "
                        f"material={material}, mesh_orientation={mesh_orientation}, {kwargs}"]

    @property
    def mesh_object(self):
        return self._mesh_object

    @mesh_object.setter
    def mesh_object(self, value):
        assert isinstance(value, trimesh.Trimesh) or isinstance(value, trimesh.Scene) or isinstance(value, bpy.types.Mesh)
        self.history.append(f"manually setting mesh_object by value of {type(value)}")
        self.changed = True
        self.info_in_sync = False
        self._mesh_object = value
        self.mesh_information = None

    def __str__(self):
        return self.unique_name

    def set_unique_name(self, value):
        self.unique_name = value

    def stringable(self):
        # [TODO v2.1.0]
        # Eventhough we can create a string of this mesh, it's not possible yet to have this mesh as link in the robot
        # This is still an open to do, which requires to deepcopy meshes on demand when one instance is changed in a
        # way that is not applicable to other usages of this mesh.
        return False

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
        if len(self.exported) == 0:
            return self.input_file
        else:
            assert self._related_robot_instance is not None
            assert self._related_robot_instance.mesh_format is not None
            if self._related_robot_instance.mesh_format not in self.exported:
                raise IOError(f"The mesh {self.unique_name} with the required mesh format ({self._related_robot_instance.mesh_format}) has not yet been exported.")
            return self.exported[self._related_robot_instance.mesh_format]

    @property
    def filepath(self):
        if self._related_robot_instance is not None and self._related_robot_instance.xmlfile is not None:
            return os.path.relpath(self.abs_filepath, os.path.dirname(self._related_robot_instance.xmlfile))
        else:
            return self.abs_filepath

    @property
    def input_file_name(self):
        return os.path.splitext(os.path.basename(self.input_file))[0]

    def file_exists(self):
        return os.path.isfile(self.abs_filepath)

    def available(self):
        return self.file_exists() or self.mesh_object is not None

    def is_valid(self):
        if self.input_file is None or not os.path.isfile(self.input_file):
            log.error(f"Mesh {self.input_file} does not exist")
            return False
        if self.input_file is not None and os.path.isfile(self.input_file) and self.mesh_object is None:
            with open(os.path.realpath(self.input_file), "rb") as f:
                if b'\0' not in f.read():
                    log.error(f"LFS not properly checked out. (Mesh {self.input_file})")
                    return False
        return self.has_enough_vertices()

    def has_enough_vertices(self):
        self.load_mesh()
        mesh = deepcopy(mesh_io.as_trimesh(self.mesh_object))
        zero_transform = create_transformation(xyz=-mesh.centroid)
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
        mesh = mesh_io.as_trimesh(self.mesh_object)
        if not mesh.is_volume:
            mesh = improve_mesh(mesh)
        if mesh.is_volume:
            return mesh.volume, mesh.center_mass
        else:
            return mesh.convex_hull.volume, mesh.convex_hull.centroid

    def equivalent(self, other):
        return (not self.changed and not other.changed and self.input_file == other.input_file) or\
               identical(mesh_io.as_trimesh(self.mesh_object), mesh_io.as_trimesh(other.mesh_object))

    def load_mesh(self, reload=False):
        if self.mesh_object is not None and not reload:
            return
        assert os.path.isfile(self.input_file), f"Mesh with path {self.input_file} wasn't found!"
        if BPY_AVAILABLE:
            bpy.ops.object.select_all(action='DESELECT')
            if self.input_type == "file_stl":
                bpy.ops.import_mesh.stl(filepath=self.input_file)
                self._mesh_object = bpy.context.object.data
            elif self.input_type == "file_obj":
                bpy.ops.import_scene.obj(filepath=self.input_file,
                                         axis_forward=self.mesh_orientation["forward"],
                                         axis_up=self.mesh_orientation["up"])
                self._mesh_object = bpy.context.object.data
                # with obj file import, blender only turns the object, not the vertices,
                # leaving a rotation in the matrix_basis, which we here get rid of
                bpy.ops.object.transform_apply(rotation=True)
                if isinstance(mesh_io.import_mesh(self.input_file), trimesh.Trimesh):
                    self.mesh_information = mesh_io.parse_obj(self.input_file)
                else:
                    log.debug(f"{self.input_file} can't be converted to bobj")
            elif self.input_type == "file_dae":
                bpy.ops.wm.collada_import(filepath=self.input_file)
                self.mesh_information = mesh_io.parse_dae(self.input_file)
                self._mesh_object = bpy.context.object.data
            elif self.input_type == "file_bobj":
                self.mesh_information = mesh_io.parse_bobj(self.input_file)
                self._mesh_object = mesh_io.mesh_info_dict_2_blender(self.unique_name, **self.mesh_information)
                bpy.context.view_layer.objects.active = bpy.data.objects.new(self.unique_name, self.mesh_object)
            bpy.ops.object.delete()
            self.changed = True  # as we there might be unnoticed changes by blender
        else:
            if self.input_type == "file_stl":
                self._mesh_object = mesh_io.import_mesh(self.input_file)
            elif self.input_type == "file_obj":
                self._mesh_object = mesh_io.import_mesh(self.input_file)
                if isinstance(self.mesh_object, trimesh.Trimesh):
                    self.mesh_information = mesh_io.parse_obj(self.input_file)
                else:
                    log.debug(f"{self.input_file} can't be converted to bobj")
            elif self.input_type == "file_dae":
                self._mesh_object = mesh_io.import_mesh(self.input_file)
                self.mesh_information = mesh_io.parse_dae(self.input_file)
            elif self.input_type == "file_bobj":
                self.mesh_information = mesh_io.parse_bobj(self.input_file)
                self._mesh_object = mesh_io.mesh_info_dict_2_trimesh(**self.mesh_information)
        self.history.append(f"loaded {'bpy-Mesh' if BPY_AVAILABLE else 'trimesh'} from {self.input_type} {self.input_file}")
        return self.mesh_object

    def provide_mesh_file(self, targetpath, format=None, throw_on_invalid_bobj=False):
        if format is None and self._related_robot_instance is not None:
            format = self._related_robot_instance.mesh_format
        elif format is None:
            raise AssertionError("To export meshes you have to specify the format. (format=None)")
        assert os.path.isabs(targetpath)
        ext = format.lower()
        targetpath = os.path.join(targetpath, self.unique_name+"."+ext)
        self.history.append(f"trying export of {type(self.mesh_object)} to {targetpath}")
        # log.debug(f"Providing mesh {targetpath}...")
        # if there are no changes we can simply copy
        if "file_"+ext == self.input_type:
            if not self.changed and self.input_file == targetpath:
                log.debug(f"Using existing mesh {targetpath}...")
                self.exported[ext] = targetpath
                self.history.append(f"->target == input == {self.input_file} for ext {ext}")
                self.write_history(targetpath)
                return
            elif not self.changed:
                log.debug(f"Copying mesh {os.path.relpath(self.input_file, os.path.dirname(targetpath))} to {targetpath}...")
                os.makedirs(os.path.dirname(targetpath), exist_ok=True)
                shutil.copyfile(self.input_file, targetpath)
                self.exported[ext] = targetpath
                self.history.append(f"->copying {self.input_file} to {targetpath}")
                self.write_history(targetpath)
                return
        # do nothing if the file is already there and identical
        if os.path.isfile(targetpath):
            existing_mesh = mesh_io.import_mesh(targetpath)
            o_history = self.read_history(targetpath)
            equiv_histories = False
            if o_history is not None:
                equiv_histories = [x for x in o_history[1:] if not x.startswith("->")] == [x for x in self.history[1:] if not x.startswith("->")]
            if existing_mesh is not None and (equiv_histories or mesh_io.identical(self.mesh_object, existing_mesh)):
                log.debug(f"Skipping export of {targetpath } as the mesh file already exists and is identical")
                self.exported[ext] = targetpath
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
        if ext == "bobj" and self.mesh_information is None:
            if throw_on_invalid_bobj:
                raise IOError(
                    f"Couldn't provide mesh {self.unique_name} to {targetpath}, because this mesh can't be exported as bobj")
            return
        elif ext == "bobj" and (not self.info_in_sync and "texture_coords" in self.mesh_information):
            if throw_on_invalid_bobj:
                raise IOError(
                    f"Couldn't provide mesh {self.unique_name} to {targetpath}, because this mesh has been edited and thus the textures might have get mixed up.")
            return
        # export
        log.debug(f"Writing {type(self.mesh_object)} to {targetpath}...")
        assert self.mesh_object is not None
        if ext == "bobj" and self.input_type == "file_obj":
            mesh_io.write_bobj(targetpath, **self.mesh_information)
            self.exported[ext] = targetpath
            self.history.append(f"->wrote bobj {targetpath} from {self.input_type}")
            self.write_history(targetpath)
            return
        # export for blender
        if BPY_AVAILABLE and isinstance(self.mesh_object, bpy.types.Mesh):
            from ..blender.utils import blender as bUtils
            objname = "tmp_export"+self.unique_name
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
            self.exported[ext] = targetpath
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
        elif ext == "bobj":
            assert isinstance(self.mesh_object, trimesh.Trimesh),\
                f"Export to bobj only possible from trimesh.Trimesh not for {type(self.mesh_object)}"
            log.debug(f"Exporting {targetpath} with {len(self.mesh_object.vertices)} vertices...")
            mesh_io.write_bobj(targetpath, **mesh_io.trimesh_2_mesh_info_dict(self.mesh_object))
        else:
            self.mesh_object.export(
                file_obj=targetpath
            )
        self.history.append(f"->wrote {ext} to {targetpath} from {self.input_type}")
        self.write_history(targetpath)
        self.exported[ext] = targetpath

    # methods that leave the mesh and mesh_info in sync
    @property
    def scale(self):
        return self._scale

    @scale.setter
    def scale(self, scale):
        if type(scale) == list:
            assert len(scale) == 3
            self._scale = [scale]
        elif type(scale) in [float, int]:
            self._scale = [scale, scale, scale]
        else:
            raise TypeError(f"Can't scale with {scale}, requires list(3) or float")

    def multiply_scale(self, factor):
        if type(factor) == list:
            assert len(factor) == 3
            self._scale = [v * s for v, s in zip(self.scale, factor)]
        elif type(factor) in [float, int]:
            self._scale = [v * factor for v in self.scale]
        else:
            raise TypeError(f"Can't multiply scale with {factor}, requires list(3) or float")

    # methods that make changes on the mesh
    def apply_scale(self):
        self.load_mesh()
        if isinstance(self.mesh_object, trimesh.Trimesh) or isinstance(self.mesh_object, trimesh.Scene):
            self.mesh_object.scale(self.scale)
            self.history.append(f"applied scale {self.scale}")
            self.changed = True
            self.scale = 1
            if self.mesh_information is not None:
                self.mesh_information["vertices"] = self.mesh_information["vertices"] * np.array(self.scale)
                self.mesh_information["vertex_normals"] = self.mesh_information["vertex_normals"] / np.array(self.scale)
        elif BPY_AVAILABLE and isinstance(self.mesh_object, bpy.types.Mesh):
            raise NotImplementedError

    def improve_mesh(self):
        self.load_mesh()
        if isinstance(self.mesh_object, trimesh.Trimesh):
            self.changed = True
            self.info_in_sync = False
            self._mesh_object = improve_mesh(self.mesh_object)
            self.history.append(f"improved mesh")
        elif isinstance(self.mesh_object, bpy.types.Mesh):
            raise NotImplementedError("Please optimize the mesh directly in blender")
        else:
            log.warning(f"Can only optimize trimesh.Trimesh not {type(self.mesh_object)}")

    def reduce_mesh(self, factor):
        self.load_mesh()
        if isinstance(self.mesh_object, trimesh.Trimesh):
            self.changed = True
            self.info_in_sync = False
            self._mesh_object = reduce_mesh(self.mesh_object, factor)
            self.unique_name = edit_name_string(self.unique_name, suffix=f"_red{str(factor).replace('.',',')}")
            self.history.append(f"reduced mesh factor={factor}")
        elif isinstance(self.mesh_object, bpy.types.Mesh):
            raise NotImplementedError("Please reduce the mesh directly in blender")
        else:
            log.warning(f"Can only reduce trimesh.Trimesh not {type(self.mesh_object)}")

    def to_convex_hull(self):
        self.load_mesh()
        if isinstance(self.mesh_object, trimesh.Trimesh) or isinstance(self.mesh_object, trimesh.Scene):
            self._mesh_object = self.mesh_object.convex_hull
            self.mesh_information = mesh_io.trimesh_2_mesh_info_dict(self.mesh_object)
            self.changed = True
            self.info_in_sync = False
            self.unique_name = edit_name_string(self.unique_name, suffix="_convex")
            self.history.append(f"to trimesh convex_hull")
        elif BPY_AVAILABLE and isinstance(self.mesh_object, bpy.types.Mesh):
            import bmesh
            bm = bmesh.new()
            bm.from_mesh(self.mesh_object)
            bmesh.ops.convex_hull(bm, bm.verts)
            bm.to_mesh(self.mesh_object)
            bm.free()
            self.changed = True
            self.info_in_sync = False
            self.unique_name = edit_name_string(self.unique_name, suffix="_convex")
            self.history.append(f"to bpy convex_hull")
        else:
            log.error(f"Couldn't create convex hull for mesh {self.unique_name}")

    def mirror(self, mirror_transform=None, name_replacements=None):
        self.load_mesh()
        if mirror_transform is None:
            mirror_transform = get_reflection_matrix()
        if name_replacements is None:
            name_replacements = {}
        if isinstance(self.mesh_object, trimesh.Trimesh) or isinstance(self.mesh_object, trimesh.Scene):
            self._mesh_object = self.mesh_object.apply_transform(mirror_transform)
            self.mesh_object.fix_normals()
            self.mesh_information = mesh_io.trimesh_2_mesh_info_dict(self.mesh_object)
            self.changed = True
            self.info_in_sync = False
            self.unique_name = edit_name_string(self.unique_name, suffix="_mirrored", replacements=name_replacements)
            self.history.append(f"trimesh mirrored {['{:0.2f}'.format(x) for x in mirror_transform.diagonal()]}")
        elif BPY_AVAILABLE and isinstance(self.mesh_object, bpy.types.Mesh):
            import bmesh
            import mathutils
            bm = bmesh.new()
            bm.from_mesh(self.mesh_object)
            bmesh.ops.mirror(bm, bm.faces, mathutils.Matrix(mirror_transform))
            bm.to_mesh(self.mesh_object)
            bm.free()
            self.changed = True
            self.info_in_sync = False
            self.mesh_information = mesh_io.blender_2_mesh_info_dict(self.mesh_object)
            self.unique_name = edit_name_string(self.unique_name, suffix="_mirrored", replacements=name_replacements)
            self.history.append(f"bpy mirrored {['{:0.2f}'.format(x) for x in mirror_transform.diagonal()]}")
        else:
            log.error(f"Couldn't create convex hull for mesh {self.unique_name}")

    def to_trimesh_mesh(self):
        self.load_mesh()
        if not isinstance(self.mesh_object, trimesh.Trimesh):
            self.changed = True
            self.info_in_sync = False
            self.history.append(f"converted from {type(self.mesh_object)} to trimesh")
            self._mesh_object = mesh_io.as_trimesh(self.mesh_object)


class GeometryFactory(Representation):
    @classmethod
    def create(cls, *args, **kwargs):
        if kwargs["type"] == "mesh":
            print(repr(args), repr(kwargs))
            return Mesh(**kwargs)
        elif kwargs["type"] == "box":
            return Box(**kwargs)
        elif kwargs["type"] == "cylinder":
            return Cylinder(**kwargs)
        elif kwargs["type"] == "sphere":
            return Sphere(**kwargs)
        raise Exception("Can't create geometry from: "+repr(kwargs))


class Collision(Representation, SmurfBase):
    _class_variables = ["name", "link", "geometry", "origin", "bitmask"]

    def __init__(self, name=None, link=None, geometry=None, origin=None, bitmask=None, noDataPackage=False,
                 reducedDataPackage=False, ccfm=None, **kwargs):
        if type(link) is str:
            link = link
        elif link is not None:
            link = link.name
        self.original_name = name
        if name is None or len(name) == 0:
            if link is not None:
                name = str(link) + "_collision"
            elif "_parent_xml" in kwargs:
                link = kwargs["_parent_xml"].attrib["name"]
                name = link + "_collision"
            else:
                name = None
        SmurfBase.__init__(self, name=name, link=link, returns=['name', 'link'], **kwargs)
        self.geometry = _singular(geometry)
        if origin is None:
            origin = Pose()
        self.origin = _singular(origin)
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
        self.excludes += ["origin", "original_name", "geometry"]


class Visual(Representation, SmurfBase):
    _class_variables = ["name", "geometry", "material", "origin"]

    def __init__(self, geometry=None, material=None, material_: Material = None, origin=None, name=None, **kwargs):
        super().__init__()
        self.original_name = name
        if name is None or len(name) == 0:
            if "_parent_xml" in kwargs:
                link = kwargs["_parent_xml"].attrib["name"]
                name = link + "_collision"
            else:
                name = None
        self.name = name
        self.geometry = _singular(geometry)
        material_ = _singular(material_)
        material = _singular(material)
        if type(material) == str:
            self.material = material
            if material_ is not None:
                assert isinstance(material_, Material) and material_.original_name == material
                self.material = material_
        elif isinstance(material, Material):
            assert material_ is None or material_.equivalent(material)
            self.material = material_
        if origin is None:
            origin = Pose()
        self.origin = _singular(origin)
        assert isinstance(self.origin, Pose)

    @property
    # [TODO v2.0.0] Make the name more selfexplanatory, that this is the dummy for exporting only a reference
    def material_(self):
        return None

    def equivalent(self, other):
        return self.geometry.equivalent(other.geometry) and self._material.equivalent(other._material) and \
               self.origin == other.origin


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
    _class_variables = ["mass", "inertia", "origin"]

    def __init__(self, mass=0.0, inertia=None, origin=None, **kwargs):
        super().__init__()
        self.mass = mass
        self.inertia = _singular(inertia)
        if origin is None:
            origin = Pose()
        self.origin = _singular(origin)
        assert type(self.origin) == Pose, f"{origin} is not Pose"

    def stringable(self):
        return False

    @staticmethod
    def from_mass_matrix(M, origin: Pose):
        return Inertial(
            mass=M[0, 0],
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
            out, _ = execute_shell_command(f"kccdcoveriv {iv_mesh} {self._npoints} {self.frame}",
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
    _class_variables = ["name", "visuals", "collisions", "inertial", "kccd_hull"]

    def __init__(self, name=None, visuals=None, inertial=None, collisions=None, #origin=None,
                 noDataPackage=False, reducedDataPackage=False, is_human=None, kccd_hull=None, **kwargs):
        # assert origin is None  # Unused but might be neccesary for sdf
        SmurfBase.__init__(self, **kwargs)
        self.name = name
        self.is_human = is_human
        self.returns += ['name', "is_human"]
        self.visuals = []
        if visuals is not None:
            self.visuals = visuals
        self.inertial = _singular(inertial)
        self.collisions = []
        if collisions is not None:
            self.collisions = collisions
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
        for geo in self.collisions:
            i = 0
            if geo.name is None:
                geo.name = self.name + "_collision"
                if i > 0:
                    geo.name += str(i)

        for geo in self.visuals:
            i = 0
            if geo.name is None:
                geo.name = self.name + "_visual"
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
        self.name = name
        self.returns = ['name']
        assert parent is not None
        assert child is not None
        self.parent = parent if type(parent) == str else parent.name
        assert self.parent is not None
        self.child = child if type(child) == str else child.name
        assert self.child is not None
        self.joint_type = joint_type if joint_type is not None else (kwargs["type"] if "type" in kwargs else None)
        assert self.joint_type is not None, f"Joint type of {self.name} undefined!"
        if axis is not None:
            self.axis = (np.array(axis)/np.linalg.norm(axis)).tolist() if joint_type != "fixed" else None
        else:
            self.axis = None
        if origin is None and cut_joint is False:
            origin = Pose(xyz=[0, 0, 0], rpy=[0, 0, 0], relative_to=self.parent)
        self._origin = _singular(origin)
        self.limit = _singular(limit)
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
        self.returns += ["joint_dependencies", "parent", "child"]
        self.excludes += ["limit", "mimic", "axis", "dynamics"]

    def link_with_robot(self, robot, check_linkage_later=False):
        super(Joint, self).link_with_robot(robot, check_linkage_later=True)
        if self.cut_joint and self.origin is None:
            self.origin = Pose.from_matrix(self._related_robot_instance.get_transformation(self.child, self.parent),
                                           relative_to=self.parent)
        if not check_linkage_later:
            self.check_linkage()

    def check_valid(self):
        return (self.joint_type in self.TYPES + self.ADVANCED_TYPES, "Invalid joint type: {}".format(self.joint_type) and  # noqa
                (self._related_robot_instance is None or
                 (self._related_robot_instance.get_link(self.parent) is not None and
                  self._related_robot_instance.get_link(self.child) is not None)))

    @property
    def origin(self):
        if self._origin is not None and self._origin.relative_to is None:
            self._origin.relative_to = self.parent
        return self._origin

    @origin.setter
    def origin(self, origin: Pose):
        self._origin = _singular(origin)

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
    TYPES = ["dc", "ac", "step"]
    _class_variables = ["name", "joint"]

    def __init__(self, name=None, joint=None, **kwargs):
        SmurfBase.__init__(self, name=name, joint=joint, type="dc", **kwargs)
        # This is hardcoded information
        assert self.joint is not None
        self._maxEffort = None
        self._maxSpeed = None
        self._maxValue = None
        self._minValue = None
        self.type = type
        self.returns += ['joint', 'maxEffort', 'maxSpeed', 'maxValue', 'minValue']

    @property
    def maxEffort(self):
        if self._joint:
            return self._joint.limit.effort if self._joint.limit else 0
        else:
            return 0

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
        if self._joint:
            return self._joint.limit.upper if self._joint.limit else 0
        else:
            return 0

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
        if self._joint:
            return self._joint.limit.lower if self._joint.limit else 0
        else:
            return 0

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
        if self._joint:
            return self._joint.limit.velocity if self._joint.limit else 0
        else:
            return 0

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
        if self._joint.mimic is not None:
            return self._joint.mimic.joint

    @mimic_motor.setter
    def mimic_motor(self, val):
        pass

    @property
    def mimic_multiplier(self):
        if self._joint.mimic is not None:
            return self._joint.mimic.multiplier

    @mimic_multiplier.setter
    def mimic_multiplier(self, val):
        pass

    @property
    def mimic_offset(self):
        if self._joint.mimic is not None:
            return self._joint.mimic.offset

    @mimic_offset.setter
    def mimic_offset(self, val):
        pass
