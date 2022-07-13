import numpy as np

from .base import Representation
from .representation import Pose
from .smurf_reflection import SmurfBase
from ..io import representation
from ..utils import transform

__IMPORTS__ = [x for x in dir() if not x.startswith("__")]


class Sensor(Representation, SmurfBase):
    def __init__(self, name: str = None, joint=None, link=None, rate=None, sensortype=None, origin=None, **kwargs):
        if link is not None and type(link) != str:
            link = link.name
        if joint is not None and type(joint) != str:
            joint = joint.name
        super(SmurfBase).__init__(name=name, joint=joint, link=link, rate=rate, origin=origin, **kwargs)
        self.type = sensortype
        self.returns += ['type']
        self.excludes += ['origin']

    @property
    def position_offset(self):
        pos = self.origin.position() if hasattr(self, "origin") else [0, 0, 0]
        return {"x": pos[0], "y": pos[1], "z": pos[2]}

    @property
    def orientation_offset(self):
        quat = transform.matrix_to_quaternion(self.origin.to_matrix()[0:3, 0:3]) \
            if hasattr(self, "origin") else [0, 0, 0, 1]
        return {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]}

    def transform(self, transformation):
        if hasattr(self, "origin"):
            self.origin.transform(transformation)
        else:
            self.origin = Pose.from_matrix(transformation)

    def get_refl_vars(self):
        if self.position_offset != {"x": 0, "y": 0, "z": 0}:
            self.returns += ["position_offset"]
        if self.orientation_offset != {"x": 0, "y": 0, "z": 0, "w": 1}:
            self.returns += ["orientation_offset"]

        return super(Sensor, self).get_refl_vars()


class Joint6DOF(Sensor):
    name = None
    link = None

    def __init__(self, name=None, link=None, **kwargs):
        super().__init__(name=name, joint=None, link=link, sensortype='Joint6DOF', **kwargs)
        self.returns += ['link']
        self.sdf_type = "force_torque"


class RotatingRaySensor(Sensor):
    name = None
    link = None
    bands = 0
    draw_rays = False
    horizontal_offset = 0
    horizontal_resolution = 0
    opening_width = np.pi * 2
    lasers = 32
    max_distance = 5.0
    min_distance = 0
    opening_height = 0.8
    vertical_offset = 0

    def __init__(
            self, name=None, link=None,
            bands=8, draw_rays=False,
            horizontal_offset=0, opening_width=np.pi * 2, horizontal_resolution=0.03,
            lasers=32, max_distance=5.0, min_distance=0.0, opening_height=0.7,
            vertical_offset=0, **kwargs):
        super().__init__(name=name, joint=None, link=link, sensortype='RotatingRaySensor', **kwargs)
        self.bands = bands
        self.draw_rays = draw_rays
        self.horizontal_offset = horizontal_offset
        self.horizontal_resolution = horizontal_resolution
        self.opening_width = opening_width
        self.lasers = lasers
        self.max_distance = max_distance
        self.min_distance = min_distance
        self.opening_height = opening_height
        self.vertical_offset = vertical_offset
        if not isinstance(link, representation.Link):
            print(link)
            raise AssertionError("Parsed invalid link")
        self.returns += ['link', 'bands', 'draw_rays',
                         'horizontal_offset', 'horizontal_resolution', 'vertical_offset',
                         'opening_width', 'opening_height', 'max_distance', 'lasers']
        self.sdf_type = "lidar"

    @property
    def min_horizontal_angle(self):
        return self.horizontal_offset

    @property
    def max_horizontal_angle(self):
        return self.horizontal_offset + self.opening_width

    @property
    def min_vertical_angle(self):
        return self.vertical_offset

    @property
    def max_vertical_angle(self):
        return self.vertical_offset + self.opening_height

    @property
    def vertical_resolution(self):
        return self.opening_height / self.lasers


class CameraSensor(Sensor):
    name = None
    link = None
    height = 480
    width = 640
    hud_height = 240
    hud_width = 0
    opening_height = 90
    opening_width = 90
    depth_image = True
    show_cam = False
    frame_offset = None

    def __init__(
            self, name=None, link=None,
            height=480, width=640, hud_height=240, hud_width=0,
            opening_height=90, opening_width=90,
            depth_image=True, show_cam=False, frame_offset: representation.Pose = None, **kwargs):
        super().__init__(name=name, joint=None, link=link, sensortype='CameraSensor', **kwargs)
        self.height = height
        self.width = width
        self.hud_height = hud_height
        self.hud_width = hud_width
        self.opening_height = opening_height if opening_height is not None else opening_width * height / width
        self.opening_width = opening_width
        self.depth_image = depth_image
        self.show_cam = show_cam
        self.frame_offset = frame_offset
        self.returns += ['link', 'height', 'width', 'hud_height', 'hud_width',
                         'opening_height', 'opening_width', 'depth_image', 'show_cam', 'frame_offset']
        self.sdf_type = "camera"

    @property
    def depths(self):
        return


class IMU(Sensor):
    name = None
    link = None
    frame = None

    def __init__(self, name=None, link=None, frame=None, **kwargs):
        super().__init__(name=name, joint=None, link=link, frame=frame, sensortype='NodeIMU', **kwargs)
        self.returns += ['link', 'id']
        self.sdf_type = "imu"

    @property
    def id(self):
        return [self.frame]


class MultiSensor(Sensor):
    def __init__(self, name=None, targets=None, sensortype='MultiSensor', **kwargs):
        super().__init__(name=name, sensortype=sensortype, **kwargs)
        self.targets = targets if isinstance(targets, list) else []
        self.returns += ['id']
        self.excludes += ['_id']

    def add_target(self, target):
        self.targets = self.targets + [target if type(target) == str else target.name]

    @property
    def id(self):
        return [t.name for t in self.targets]  # if self._id else None

    @id.setter
    def id(self, targets):
        for t in targets:
            self.add_target(t)
        return

    def remove_target(self, target):
        self.targets = [t for t in self.targets if t != (target if type(target) == str else target.name)]

    def is_empty(self):
        return len(self.targets) == 0


class MotorCurrent(MultiSensor):
    type_dict = {"targets": "joint"}
    name = None
    targets = None

    def __init__(self, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        super().__init__(name=name, targets=targets, sensortype='MotorCurrent', **kwargs)
        self.returns += ['link']


class JointPosition(MultiSensor):
    type_dict = {"targets": "joint"}
    name = None
    targets = None

    def __init__(self, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        super().__init__(name=name, targets=targets, sensortype='JointPosition', **kwargs)
        self.returns += ['link']


class JointVelocity(MultiSensor):
    type_dict = {"targets": "joint"}
    name = None
    targets = None

    def __init__(self, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        if not all([isinstance(t, representation.Joint) for t in targets]):
            print(targets)
            raise AssertionError("Parsed invalid joint")

        super().__init__(name=name, targets=targets, sensortype='JointVelocity', **kwargs)
        self.returns += ['link']


class NodeContact(MultiSensor):
    type_dict = {"targets": "link"}
    name = None
    link = None
    targets = None

    def __init__(self, name=None, link=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        super().__init__(name=name, targets=targets, link=link, sensortype='NodeContact', **kwargs)

        self.returns += ['link']
        self.sdf_type = "contact"


class NodeContactForce(MultiSensor):
    type_dict = {"targets": "link"}
    name = None
    link = None
    targets = None

    def __init__(self, name=None, link=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        super().__init__(name=name, targets=targets, link=link, sensortype='NodeContactForce', **kwargs)

        self.returns += ['link']


class NodeCOM(MultiSensor):
    type_dict = {"targets": "link"}
    name = None
    targets = None

    def __init__(self, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        super().__init__(name=name, targets=targets, sensortype='NodeCOM', **kwargs)


class NodePosition(MultiSensor):
    type_dict = {"targets": "link"}
    name = None
    targets = None

    def __init__(self, name: str = None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        super().__init__(name=name, targets=targets, sensortype='NodePosition', **kwargs)


class NodeRotation(MultiSensor):
    type_dict = {"targets": "link"}
    name = None
    targets = None

    def __init__(self, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        super().__init__(name=name, targets=targets, sensortype='NodeRotation', **kwargs)


def sensor_factory(name, parent=None, sdf_type=None, origin=None, **kwargs):
    if sdf_type is None and "bands" in kwargs or "lasers" in kwargs:
        sdf_type = "lidar"
    if parent is None:
        parent = origin.relative_to
    if sdf_type == "camera":
        return CameraSensor(
            name=name,
            link=parent,
            **kwargs
        )
    elif sdf_type == "contact":
        return NodeContact(
            name=name,
            **kwargs
        )
    # elif sdf_type == "imu":
    #     raise NotImplemented
    elif sdf_type == "lidar":
        return RotatingRaySensor(
            name=name,
            horizontal_offset=kwargs["min_horizontal_angle"],
            opening_width=kwargs["max_horizontal_angle"] - kwargs["min_horizontal_angle"],
            vertical_offset=kwargs["min_vertical_angle"],
            opening_height=kwargs["max_vertical_angle"] - kwargs["min_vertical_angle"],
            draw_rays="visualize" in kwargs,
            **kwargs
        )
    elif sdf_type == "force_torque":
        return Joint6DOF(
            name=name,
            **kwargs
        )
