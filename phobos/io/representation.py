import os

import numpy as np

from .base import Representation
from .smurf_reflection import SmurfBase
from .xml_factory import singular as _singular
from ..geometry.io import import_mesh, import_mars_mesh
from ..utils.transform import matrix_to_rpy, round_array, rpy_to_matrix

__IMPORTS__ = [x for x in dir() if not x.startswith("__")]


class Pose(Representation):
    _class_variables = ["xyz", "rpy", "relative_to"]

    def __init__(self, xyz=None, rpy=None, vec=None, extra=None, relative_to=None):
        super().__init__()
        self.xyz = xyz
        self.rpy = rpy
        self.relative_to = relative_to
        if vec is not None:
            assert isinstance(vec, list)
            count = len(vec)
            if count == 3:
                self.xyz = vec
            else:
                self.from_vec(vec)
        elif extra is not None:
            assert xyz is None, "Cannot specify 6-length vector and 3-length vector"  # noqa
            assert len(extra) == 3, "Invalid length"
            self.rpy = extra

    def check_valid(self):
        assert (self.xyz is None or len(self.xyz) == 3) and \
               (self.rpy is None or len(self.rpy) == 3)

    # Aliases for backwards compatibility
    @property
    def rotation(self):
        return self.rpy

    @rotation.setter
    def rotation(self, value):
        self.rpy = value

    @property
    def position(self):
        return self.xyz

    @position.setter
    def position(self, value):
        self.xyz = value

    def from_vec(self, vec):
        assert len(vec) == 6, "Invalid length"
        self.xyz = vec[:3]
        self.rpy = vec[3:6]

    @property
    def vec(self):
        xyz = self.xyz if self.xyz else [0, 0, 0]
        rpy = self.rpy if self.rpy else [0, 0, 0]
        return xyz + rpy

    @staticmethod
    def from_matrix(T, dec=6, relative_to=None):
        xyz = T[0:3, 3]
        rpy = matrix_to_rpy(T[0:3, 0:3])
        return Pose(xyz=round_array(xyz, dec=dec), rpy=round_array(rpy, dec=dec), relative_to=relative_to)

    def to_matrix(self):
        R = rpy_to_matrix(self.rpy if hasattr(self, "rpy") else [0.0, 0.0, 0.0])
        p = np.array(self.xyz if hasattr(self, "xyz") else [0.0, 0.0, 0.0])
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


class Color(Representation):
    _class_variables = ["rgba"]

    def __init__(self, *args, rgba=None):
        super().__init__()
        # What about named colors?
        count = len(args)
        if rgba is not None:
            self.rgba = rgba
        elif count == 4 or count == 3:
            self.rgba = args
        elif count == 1:
            self.rgba = args[0]
        elif count == 0:
            self.rgba = None
        if self.rgba is not None:
            if len(self.rgba) == 3:
                self.rgba += [1.]
            if len(self.rgba) != 4:
                raise Exception(f'Invalid color argument count for argument "{self.rgba}"')


class Texture(Representation):
    filename = None

    def __init__(self, filename=None):
        super().__init__()
        self.filename = filename


# class JointDynamics(Representation):
#     def __init__(self, damping=None, friction=None):
#         self.damping = damping
#         self.friction = friction


class Box(Representation):
    _class_variables = ["size"]

    def __init__(self, size=None):
        super().__init__()
        self.size = size

    def scale_geometry(self, x=1, y=1, z=1):
        self.size = (v * s for v, s in zip(self.size, [x, y, z]))


class Cylinder(Representation):
    _class_variables = ["radius", "length"]

    def __init__(self, radius=0.0, length=0.0):
        super().__init__()
        self.radius = radius
        self.length = length

    def scale_geometry(self, x=1, y=1, z=1):
        assert x == y
        self.radius *= x
        self.length *= z


class Sphere(Representation):
    _class_variables = ["radius"]

    def __init__(self, radius=0.0):
        super().__init__()
        self.radius = radius

    def scale_geometry(self, x=1, y=1, z=1):
        assert x == y == z
        self.radius *= x


class Mesh(Representation):
    _class_variables = ["filename", "scale"]

    def __init__(self, filename=None, scale=None):
        self._filename = None
        super().__init__()
        self.filename = filename
        self.scale = scale

    @property
    def filename(self):
        if self._related_robot_instance is None or self._related_robot_instance.xmlfile is None:
            return self._filename
        else:
            return os.path.relpath(self._filename, os.path.dirname(os.path.abspath(self._related_robot_instance.xmlfile)))

    @filename.setter
    def filename(self, new_val):
        if self._related_robot_instance is None or self._related_robot_instance.xmlfile is None:
            self._filename = new_val
        elif not os.path.isabs(self._filename):
            self._filename = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(self._related_robot_instance.xmlfile)), self._filename))

    def scale_geometry(self, x=1, y=1, z=1, overwrite=False):
        if overwrite or self.scale is None:
            self.scale = [x, y, z]
        else:
            self.scale = [v * s for v, s in zip(self.scale, [x, y, z])]

    def load_mesh(self, urdf_path=None, mars_mesh=False):
        if mars_mesh:
            return import_mars_mesh(self.filename, urdf_path)
        return import_mesh(self.filename, urdf_path)

    def link_with_robot(self, robot, check_linkage_later=False):
        super(Mesh, self).link_with_robot(robot)
        self.filename = self._filename


class Collision(Representation, SmurfBase):
    _class_variables = ["name", "link", "geometry", "origin", "bitmask"]

    def __init__(self, name=None, link=None, geometry=None, origin=None, bitmask=None, noDataPackage=False,
                 reducedDataPackage=False, ccfm=None, **kwargs):
        if type(link) is str:
            link = link
        elif link is not None:
            link = link.name
        SmurfBase.__init__(self, name=name, link=link, **kwargs)
        self.geometry = _singular(geometry)
        self.origin = _singular(origin)

        self.returns += ['name', 'link']
        self.bitmask = bitmask
        if noDataPackage is not None:
            self.noDataPackage = noDataPackage
        if reducedDataPackage is not None:
            self.reducedDataPackage = reducedDataPackage
        if ccfm is not None:
            self.ccfm = ccfm
        if bitmask is not None:
            self.returns += ['bitmask']

    def __str__(self):
        return self.name

    def link_with_robot(self, robot):
        super(Collision, self).link_with_robot(robot)
        self.origin.link_with_robot(robot)
        if isinstance(self.geometry, Mesh):
            self.geometry.link_with_robot(robot)

    def unlink_from_robot(self):
        super(Collision, self).unlink_from_robot()
        self.origin.unlink_from_robot()


class Material(Representation, SmurfBase):
    _class_variables = ["name", "color", "texture"]

    def __init__(self, name=None, color=None, texture=None, **kwargs):
        self.name = name
        self.color = _singular(color)
        self.texture = _singular(texture)
        kwargs["name"] = name
        if "diffuseColor" not in kwargs and self.color is not None:
            kwargs["diffuseColor"] = {"r": self.color.rgba[0], "g": self.color.rgba[1], "b": self.color.rgba[2], "a": self.color.rgba[3]}
        if "diffuseTexture" not in kwargs and self.texture is not None:
            kwargs["diffuseTexture"] = self.texture.filename
        SmurfBase.__init__(self, **kwargs)
        self.excludes += ["color"]

    def __str__(self):
        return self.name

    def check_valid(self):
        if self.color is None and self.texture is None:
            raise Exception("Material has neither a color nor texture.")

    def equivalent(self, other):
        return other.texture == self.texture and other.color.rgba == self.color.rgba

    def is_delegate(self):
        return self.color is None and self.texture is None


class Visual(Representation, SmurfBase):
    _class_variables = ["name", "geometry", "material", "origin"]

    def __init__(self, robot=None, geometry=None, material=None, origin=None, name=None):
        super().__init__(robot=robot)
        self.name = name
        self.geometry = _singular(geometry)
        self.material = _singular(material)
        self.origin = _singular(origin)

    def __str__(self):
        return self.name

    def link_with_robot(self, robot):
        super(Visual, self).link_with_robot(robot)
        self.origin.link_with_robot(robot)
        # due to the specialty of having materials in both robot and visuals, we check whether those two are in sync
        if self.material is not None:
            robot_material = robot.get_material(self.material)
            if robot_material is None:
                robot.add_aggregate("material", self._material)
                return
            if id(self._material) != id(robot_material):
                if self._material.is_delegate() or self._material.equivalent(robot_material):
                    self._material = robot_material
                else:
                    new_mat_name = self._material.name
                    index = 1
                    while robot.get_material(new_mat_name) is not None:
                        new_mat_name = self._material.name + "_" + str(index)
                        index += 1
                    print("WARNING: Ambiguous material in ", self.name, "renamed ", self._material.name, "to", new_mat_name)
                    self._material.name = new_mat_name
        if isinstance(self.geometry, Mesh):
            self.geometry.link_with_robot(robot)

    def unlink_from_robot(self):
        super(Visual, self).unlink_from_robot()
        self.origin.unlink_from_robot()
        if isinstance(self._material, Material):
            self._material.unlink_from_robot()

    def equivalent(self, other):
        return self.geometry.equivalent(other.geometry) and self._material.equivalent(other._material) and self.origin == other.origin


class Inertia(Representation):
    _class_variables = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

    def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
        super().__init__()
        assert type(ixx) != str and ixx is not None
        self.ixx = ixx
        self.ixy = ixy
        self.ixz = ixz
        self.iyy = iyy
        self.iyz = iyz
        self.izz = izz

    def to_matrix(self):
        return [[self.ixx, self.ixy, self.ixz],
                [self.ixy, self.iyy, self.iyz],
                [self.ixz, self.iyz, self.izz]]

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


class Inertial(Representation):
    _class_variables = ["mass", "inertia", "origin"]

    def __init__(self, mass=0.0, inertia=None, origin=None):
        super().__init__()
        self.mass = mass
        self.inertia = _singular(inertia)
        self.origin = _singular(origin)

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

    def link_with_robot(self, robot):
        super(Inertial, self).link_with_robot(robot)
        self.origin.link_with_robot(robot)

    def unlink_from_robot(self):
        super(Inertial, self).unlink_from_robot()
        self.origin.unlink_from_robot()


class JointLimit(Representation):
    _class_variables = ["effort", "velocity", "lower", "upper"]

    def __init__(self, effort=None, velocity=None, lower=None, upper=None):
        super().__init__()
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper


class JointMimic(Representation):
    _class_variables = ["joint", "multiplier", "offset"]

    def __init__(self, joint=None, multiplier=None, offset=None):
        super().__init__()
        self.joint = joint
        self.multiplier = multiplier
        self.offset = offset


class Joint(Representation, SmurfBase):
    TYPES = ['unknown', 'revolute', 'continuous', 'prismatic',
             'floating', 'planar', 'fixed']

    type_dict = {
        "parent": "link",
        "child": "link",
    }
    _class_variables = ["name", "parent", "child", "joint_type", "axis", "limit", "dynamics", "mimic", "motor"]

    def __init__(self, name=None, parent=None, child=None, joint_type=None,
                 axis=None, origin=None, limit=None,
                 dynamics=None, safety_controller=None, calibration=None,
                 mimic=None, motor=None,
                 noDataPackage=False, reducedDataPackage=False,
                 damping_const_constraint_axis1=None, springDamping=None, springStiffness=None,
                 spring_const_constraint_axis1=None, **kwargs):
        SmurfBase.__init__(self, **kwargs)
        self.name = name
        self.returns = ['name']
        self.parent = parent if type(parent) == str else parent.name
        self.child = child if type(child) == str else child.name
        self.joint_type = joint_type
        self.axis = axis
        self._origin = _singular(origin)
        self.limit = _singular(limit)
        self.dynamics = _singular(dynamics)
        self.mimic = _singular(mimic)
        self.motor = (motor if type(motor) == str else motor.name) if motor is not None else None
        if noDataPackage is not None:
            self.noDataPackage = noDataPackage
            self.returns += ["noDataPackage"]
        if reducedDataPackage is not None:
            self.reducedDataPackage = reducedDataPackage
            self.returns += ["reducedDataPackage"]
        if damping_const_constraint_axis1 is not None:
            self.damping_const_constraint_axis1 = damping_const_constraint_axis1
            self.returns += ["damping_const_constraint_axis1"]
        if springDamping is not None:
            self.springDamping = springDamping
            self.returns += ["springDamping"]
        if springStiffness is not None:
            self.springStiffness = springStiffness
            self.returns += ["springStiffness"]
        if spring_const_constraint_axis1 is not None:
            self.spring_const_constraint_axis1 = spring_const_constraint_axis1
            self.returns += ["spring_const_constraint_axis1"]
        self.excludes += ["limit"]

    def __str__(self):
        return self.name

    def check_valid(self):
        assert self.joint_type in self.TYPES, "Invalid joint type: {}".format(self.joint_type)  # noqa

    @property
    def origin(self):
        if self._origin.relative_to is None:
            self._origin.relative_to = self.parent
        return self._origin

    @origin.setter
    def origin(self, origin: Pose):
        self._origin = _singular(origin)

    def link_with_robot(self, robot, check_linkage_later=False):
        super(Joint, self).link_with_robot(robot, check_linkage_later=True)
        if self.mimic is not None:
            self.mimic.link_with_robot(robot)
        if not check_linkage_later:
            self.check_linkage()

    def unlink_from_robot(self):
        super(Joint, self).unlink_from_robot()
        if self.mimic is not None:
            self.mimic.unlink_from_robot()


class Link(Representation, SmurfBase):
    _class_variables = [ "name", "visuals", "collisions", "inertial"]

    def __init__(self, name=None, visuals=None, inertial=None, collisions=None,
                 origin=None, noDataPackage=False, reducedDataPackage=False, **kwargs):
        assert origin is None  # Unused but might be neccesary for sdf
        SmurfBase.__init__(self, **kwargs)
        self.name = name
        self.returns += ['name']
        self.visuals = []
        if visuals is not None:
            self.visuals = visuals
        self.inertial = _singular(inertial)
        self.collisions = []
        if collisions is not None:
            self.collisions = collisions
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

    def __str__(self):
        return self.name

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

    def link_with_robot(self, robot, check_linkage_later=False):
        super(Link, self).link_with_robot(robot, check_linkage_later=True)
        for vis in self.visuals:
            vis.link_with_robot(robot)
        if not check_linkage_later:
            self.check_linkage()

    def unlink_from_robot(self):
        super(Link, self).unlink_from_robot()
        for vis in self.visuals:
            vis.unlink_from_robot()


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

    def __init__(self, name, mechanicalReduction=1):
        super().__init__()
        self.name = name
        self.mechanicalReduction = mechanicalReduction


class TransmissionJoint(Representation):
    _class_variables = ["name", "hardwareInterface"]

    _type_dict = {"name": "joint"}

    def __init__(self, name, hardwareInterfaces=None):
        super().__init__()
        self.name = name
        self.hardwareInterfaces = [] if hardwareInterfaces is None else hardwareInterfaces

    def check_valid(self):
        assert len(self.hardwareInterfaces) > 0, "no hardwareInterface defined"


class Transmission(Representation):
    """ New format: http://wiki.ros.org/urdf/XML/Transmission """
    _class_variables = ["name", "joints", "actuators"]

    def __init__(self, name, joints: TransmissionJoint=None, actuators=None):
        super().__init__()
        self.name = name
        self.joints = [] if joints is None else joints
        self.actuators = [] if actuators is None else actuators

    def check_valid(self):
        assert len(self.joints) > 0, "no joint defined"
        assert len(self.actuators) > 0, "no actuator defined"

    def link_with_robot(self, robot, check_linkage_later=False):
        super(Transmission, self).link_with_robot(robot)
        for j in self.joints:
            j.link_with_robot(robot)


class Motor(Representation, SmurfBase):
    _class_variables = ["name", "joint"]

    def __init__(self, name=None, joint=None, **kwargs):
        SmurfBase.__init__(self, name=name, joint=joint,**kwargs)
        # This is hardcoded information
        self._maxEffort = None
        self._maxSpeed = None
        self._maxValue = None
        self._minValue = None
        self.returns += ['joint', 'maxEffort', 'maxSpeed', 'maxValue', 'minValue']

    def __str__(self):
        return self.name

    def link_with_robot(self, robot, check_linkage_later=False):
        super(Motor, self).link_with_robot(robot)
        setattr(self._joint, "motor", self)
        if not check_linkage_later:
            self.check_linkage()

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
