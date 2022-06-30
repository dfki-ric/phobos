from phobos.io.base import Representation
from phobos.io.xml_factory import singular as _singular
from phobos.geometry.io import import_mesh as _import_mesh, import_mars_mesh as _import_mars_mesh


class Pose(Representation):
    def __init__(self, xyz=None, rpy=None, vec=None, extra=None):
        self.xyz = xyz
        self.rpy = rpy
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
    def rotation(self): return self.rpy

    @rotation.setter
    def rotation(self, value): self.rpy = value

    @property
    def position(self): return self.xyz

    @position.setter
    def position(self, value): self.xyz = value

    def from_vec(self, vec):
        assert len(vec) == 6, "Invalid length"
        self.xyz = vec[:3]
        self.rpy = vec[3:6]

    @property
    def vec(self):
        xyz = self.xyz if self.xyz else [0, 0, 0]
        rpy = self.rpy if self.rpy else [0, 0, 0]
        return xyz + rpy


class Color(Representation):
    def __init__(self, *args, rgba=None):
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


# class JointDynamics(Representation):
#     def __init__(self, damping=None, friction=None):
#         self.damping = damping
#         self.friction = friction


class Box(Representation):
    def __init__(self, size=None):
        self.size = size

    def scale_geometry(self, x=1, y=1, z=1):
        self.size = (v*s for v,s in zip(self.size, [x,y,z]))


class Cylinder(Representation):
    def __init__(self, radius=0.0, length=0.0):
        self.radius = radius
        self.length = length

    def scale_geometry(self, x=1, y=1, z=1):
        assert x == y
        self.radius *= x
        self.length *= z


class Sphere(Representation):
    def __init__(self, radius=0.0):
        self.radius = radius

    def scale_geometry(self, x=1, y=1, z=1):
        assert x == y == z
        self.radius *= x


class Mesh(Representation):
    def __init__(self, filename=None, scale=None):
        self.filename = filename
        self.scale = scale

    def scale_geometry(self, x=1, y=1, z=1, overwrite=False):
        if overwrite:
            self.scale = [x, y, z]
        else:
            self.scale = [v*s for v, s in zip(self.scale, [x, y, z])]

    def load_mesh(self, urdf_path=None, mars_mesh=False):
        if mars_mesh:
            return _import_mars_mesh(self.filename, urdf_path)
        return _import_mesh(self.filename, urdf_path)


class Collision(Representation):
    def __init__(self, geometry=None, origin=None, name=None):
        self.geometry = _singular(geometry)
        self.name = name
        self.origin = _singular(origin)


class Texture(Representation):
    def __init__(self, filename=None):
        self.filename = filename


class Material(Representation):
    def __init__(self, name=None, color=None, texture=None):
        self.name = name
        self.color = _singular(color)
        self.texture = _singular(texture)

    def check_valid(self):
        if self.color is None and self.texture is None:
            raise Exception("Material has neither a color nor texture.")


# class LinkMaterial(Material):
#     def check_valid(self):
#         pass
#
#


class Visual(Representation):
    def __init__(self, geometry=None, material=None, origin=None, name=None):
        self.geometry = _singular(geometry)
        self.material = _singular(material)
        self.name = name
        self.origin = _singular(origin)


class Inertia(Representation):
    KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

    def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
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


class Inertial(Representation):
    def __init__(self, mass=0.0, inertia=None, origin=None):
        self.mass = mass
        self.inertia = _singular(inertia)
        self.origin = _singular(origin)


class JointLimit(Representation):
    def __init__(self, effort=None, velocity=None, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper


class JointMimic(Representation):
    def __init__(self, joint=None, multiplier=None, offset=None):
        self.joint = joint
        self.multiplier = multiplier
        self.offset = offset


class Joint(Representation):
    TYPES = ['unknown', 'revolute', 'continuous', 'prismatic',
             'floating', 'planar', 'fixed']

    def __init__(self, name=None, parent=None, child=None, type=None,
                 axis=None, origin=None,
                 limit=None, dynamics=None, safety_controller=None,
                 calibration=None, mimic=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.type = type
        self.axis = axis
        self.origin = _singular(origin)
        self.limit = _singular(limit)
        self.dynamics = _singular(dynamics)
        self.mimic = _singular(mimic)

    def check_valid(self):
        assert self.type in self.TYPES, "Invalid joint type: {}".format(self.type)  # noqa

    # Aliases
    @property
    def joint_type(self): return self.type

    @joint_type.setter
    def joint_type(self, value): self.type = value


class Link(Representation):
    def __init__(self, name=None, visuals=None, inertial=None, collisions=None,
                 origin=None):
        self.name = name
        self.visuals = []
        if visuals is not None:
            self.visuals = visuals
        self.inertial = _singular(inertial)
        self.collisions = []
        if collisions is not None:
            self.collisions = collisions
        assert origin is None

    def __get_visual(self):
        """Return the first visual or None."""
        if self.visuals:
            return self.visuals[0]

    def __set_visual(self, visual):
        """Set the first visual."""
        if self.visuals:
            self.visuals[0] = visual
        else:
            self.visuals.append(visual)
        if visual:
            self.add_aggregate('visual', visual)

    def __get_collision(self):
        """Return the first collision or None."""
        if self.collisions:
            return self.collisions[0]

    def __set_collision(self, collision):
        """Set the first collision."""
        if self.collisions:
            self.collisions[0] = collision
        else:
            self.collisions.append(collision)
        if collision:
            self.add_aggregate('collision', collision)

    # Properties for backwards compatibility
    visual = property(__get_visual, __set_visual)
    collision = property(__get_collision, __set_collision)

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



# class PR2Transmission(Representation):
#     def __init__(self, name=None, joint=None, actuator=None, type=None,
#                  mechanicalReduction=1):
#         self.name = name
#         self.type = type
#         self.joint = joint
#         self.actuator = actuator
#         self.mechanicalReduction = mechanicalReduction
#
#
# xmlr.reflect(PR2Transmission, tag='pr2_transmission', params=[
#     name_attribute,
#     xmlr.Attribute('type', str),
#     xmlr.Element('joint', 'element_name'),
#     xmlr.Element('actuator', 'element_name'),
#     xmlr.Element('mechanicalReduction', float)
# ])
#
#
# class Actuator(Representation):
#     def __init__(self, name=None, mechanicalReduction=1):
#         self.name = name
#         self.mechanicalReduction = None
#
#
# xmlr.reflect(Actuator, tag='actuator', params=[
#     name_attribute,
#     xmlr.Element('mechanicalReduction', float, required=False)
# ])
#
#
# class TransmissionJoint(Representation):
#     def __init__(self, name=None):
#         self.name = name
#         self.hardwareInterfaces = []
#
#     def check_valid(self):
#         assert len(self.hardwareInterfaces) > 0, "no hardwareInterface defined"
#
#
# xmlr.reflect(TransmissionJoint, tag='joint', params=[
#     name_attribute,
#     xmlr.AggregateElement('hardwareInterface', str),
# ])
#
#
# class Transmission(Representation):
#     """ New format: http://wiki.ros.org/urdf/XML/Transmission """
#
#     def __init__(self, name=None):
#
#         self.name = name
#         self.joints = []
#         self.actuators = []
#
#     def check_valid(self):
#         assert len(self.joints) > 0, "no joint defined"
#         assert len(self.actuators) > 0, "no actuator defined"
#
#
# xmlr.reflect(Transmission, tag='new_transmission', params=[
#     name_attribute,
#     xmlr.Element('type', str),
#     xmlr.AggregateElement('joint', TransmissionJoint),
#     xmlr.AggregateElement('actuator', Actuator)
# ])
#
# xmlr.add_type('transmission',
#               xmlr.DuckTypedFactory('transmission',
#                                     [Transmission, PR2Transmission]))


class Robot(Representation):
    SUPPORTED_VERSIONS = ["1.0"]

    def __init__(self, name=None, version=None, links=None, joints=None, materials=None, transmissions=None):
        self.name = name
        if version is None:
            version = "1.0"
        elif type(version) is not str:
            version = str(version)
        if version not in self.SUPPORTED_VERSIONS:
            raise ValueError("Invalid version; only %s is supported" % (','.join(self.SUPPORTED_VERSIONS)))

        self.version = version

        self.joints = []
        self.links = []

        self.joint_map = {}
        self.link_map = {}

        self.parent_map = {}
        self.child_map = {}

        if joints is not None:
            for joint in joints:
                self.add_aggregate("joint", joint)
        if links is not None:
            for link in links:
                self.add_aggregate("link", link)

        self.materials = materials if materials is not None else []
        self.transmissions = transmissions if transmissions is not None else []


    def add_aggregate(self, typeName, elem):
        if typeName == 'joint':
            joint = elem
            self.joint_map[joint.name] = joint
            self.parent_map[joint.child] = (joint.name, joint.parent)
            if joint.parent in self.child_map:
                self.child_map[joint.parent].append((joint.name, joint.child))
            else:
                self.child_map[joint.parent] = [(joint.name, joint.child)]
            if elem not in self.joints:
                self.joints += [elem]
        elif typeName == 'link':
            link = elem
            self.link_map[link.name] = link
            if elem not in self.links:
                self.links += [elem]

    def add_link(self, link):
        self.add_aggregate('link', link)

    def add_joint(self, joint):
        self.add_aggregate('joint', joint)

    def get_chain(self, root, tip, joints=True, links=True, fixed=True):
        chain = []
        if links:
            chain.append(tip)
        link = tip
        while link != root:
            (joint, parent) = self.parent_map[link]
            if joints:
                if fixed or self.joint_map[joint].joint_type != 'fixed':
                    chain.append(joint)
            if links:
                chain.append(parent)
            link = parent
        chain.reverse()
        return chain

    def get_root(self):
        root = None
        for link in self.link_map:
            if link not in self.parent_map:
                assert root is None, "Multiple roots detected, invalid URDF."
                root = link
        assert root is not None, "No roots detected, invalid URDF."
        return root

    def post_read_xml(self):
        if self.version is None:
            self.version = "1.0"

        split = self.version.split(".")
        if len(split) != 2:
            raise ValueError("The version attribute should be in the form 'x.y'")

        if split[0] == '' or split[1] == '':
            raise ValueError("Empty major or minor number is not allowed")

        if int(split[0]) < 0 or int(split[1]) < 0:
            raise ValueError("Version number must be positive")

        if self.version not in self.SUPPORTED_VERSIONS:
            raise ValueError("Invalid version; only %s is supported" % (','.join(self.SUPPORTED_VERSIONS)))
