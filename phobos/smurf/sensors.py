__IMPORTS__ = ["np", "representation", "SmurfAnnotation", "transform"]

import numpy as np

from .base import SmurfAnnotation
from ..io import representation
from ..utils import transform


class Sensor(SmurfAnnotation):
    def __init__(self, robot=None, name=None, joint=None, link=None, sensortype=None, **kwargs):
        super().__init__(robot=robot, name=name, joint=joint, link=link, **kwargs)
        self.type = sensortype
        self.returns += ['type']
        self.excludes += ['origin']
        if "origin" in kwargs.keys():
            self.exclude += ['origin']
            if isinstance(kwargs["origin"], representation.Pose):
                self.origin = np.identity(4)
                self.origin[0:3, 0:3] = transform.rpy_to_matrix(kwargs["origin"].rpy)
                self.origin[0:3, 3] = kwargs["origin"].xyz
            assert self.origin.shape == (4, 4)

    @property
    def position_offset(self):
        pos = self.origin[0:3, 3] if hasattr(self, "origin") else [0, 0, 0]
        return {"x": pos[0], "y": pos[1], "z": pos[2]}

    @property
    def orientation_offest(self):
        quat = transform.matrix_to_quaternion(self.origin[0:3, 0:3]) if hasattr(self, "origin") else [0, 0, 0, 1]
        return {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]}

    def transform(self, transformation):
        if hasattr(self, "origin"):
            self.origin = transformation.dot(self.origin)
        else:
            self.origin = transformation

    def get_refl_vars(self):
        if self.position_offset != {"x": 0, "y": 0, "z": 0}:
            self.returns += ["position_offset"]
        if self.orientation_offest != {"x": 0, "y": 0, "z": 0, "w": 1}:
            self.returns += ["orientation_offset"]

        return super(Sensor, self).get_refl_vars()


class Joint6DOF(Sensor):
    def __init__(self, robot=None, name=None, link=None, **kwargs):
        super().__init__(robot=robot, name=name, joint=None, link=link, sensortype='Joint6DOF', **kwargs)
        if not isinstance(link, representation.Link):
            print(link)
            raise AssertionError("Parsed invalid link")
        self.returns += ['link']


class RotatingRaySensor(Sensor):
    def __init__(
            self, robot=None, name=None, link=None,
            bands=8, draw_rays=False,
            horizontal_offset=0, opening_width=np.pi() * 2, horizontal_resolution=0.03,
            lasers=32, max_distance=5.0, min_distance=0.0, opening_height=0.7,
            vertical_offset=0, **kwargs):
        super().__init__(robot=robot, name=name, joint=None, link=link, sensortype='RotatingRaySensor', **kwargs)
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
    def __init__(
            self, robot=None, name=None, link=None,
            height=480, width=640, hud_height=240, hud_width=0,
            opening_height=90, opening_width=90,
            depth_image=True, show_cam=False, frame_offset: representation.Pose = None, **kwargs):
        super().__init__(robot=robot, name=name, joint=None, link=link, sensortype='CameraSensor', **kwargs)
        self.height = height
        self.width = width
        self.hud_height = hud_height
        self.hud_width = hud_width
        self.opening_height = opening_height
        self.opening_width = opening_width
        self.depth_image = depth_image
        self.show_cam = show_cam
        self.frame_offset = frame_offset

        if not isinstance(link, representation.Link):
            print(link)
            raise AssertionError("Parsed invalid link")

        self.returns += ['link', 'height', 'width', 'hud_height', 'hud_width',
                         'opening_height', 'opening_width', 'depth_image', 'show_cam', 'frame_offset']

    @property
    def depths(self):
        return


class IMU(Sensor):
    def __init__(self, robot=None, name=None, link=None, frame=None, **kwargs):
        if type(link) is str:
            link = robot.get_link(link)
        if link is None:
            raise ValueError("Frame of sensor '" + name + "' not found!")
        assert isinstance(link, representation.Link)
        if frame is None:
            frame = link
        if type(frame) is str:
            frame = robot.get_link(frame)
        if frame is None:
            raise ValueError("Frame of sensor '" + name + "' not found!")
        if not isinstance(frame, representation.Link):
            print(frame)
            raise AssertionError("Parsed invalid link")

        super().__init__(robot=robot, name=name, joint=None, link=link, sensortype='NodeIMU', **kwargs)
        self.id = [frame]
        self.returns += ['link', 'id']


class MultiSensor(Sensor):
    def __init__(self, robot=None, name=None, targets=None, sensortype='MultiSensor', **kwargs):
        super().__init__(robot=robot, name=name, sensortype=sensortype, **kwargs)
        self._id = targets if isinstance(targets, list) else []
        self.returns += ['id']
        self.excludes += ['_id']

    def add_target(self, target):
        if self._id:
            if isinstance(target, type(self._id[0])):
                self._id.append(target)
            else:
                raise Exception(
                    "Please provide similar type to add to sensor {}.\n You try to add {} to an arrray of {}".format(
                        self.name, type(target), type(self._id)))
        elif isinstance(target, (representation.Joint, representation.Link, representation.Collision)):
            self._id = [target]
        return

    @property
    def id(self):
        return [t.name for t in self._id]  # if self._id else None

    @id.setter
    def id(self, targets):
        for t in targets:
            self.add_target(t)
        return

    def remove_target(self, target):
        if target in self._id:
            self._id.remove(target)

    def is_empty(self):
        return len(self._id) == 0


class MotorCurrent(MultiSensor):
    def __init__(self, robot=None, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        if not all([isinstance(t, representation.Joint) for t in targets]):
            print(targets)
            raise AssertionError("Parsed invalid joint")

        super().__init__(robot=robot, name=name, targets=targets, sensortype='MotorCurrent', **kwargs)
        self.returns += ['link']


class JointPosition(MultiSensor):
    def __init__(self, robot=None, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        targets = [robot.get_joint(t) if type(t) is str else t for t in targets]

        if not all([isinstance(t, representation.Joint) for t in targets]):
            print(targets)
            raise AssertionError("Parsed invalid joint")

        super().__init__(robot=robot, name=name, targets=targets, sensortype='JointPosition', **kwargs)
        self.returns += ['link']


class JointVelocity(MultiSensor):
    def __init__(self, robot=None, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        if not all([isinstance(t, representation.Joint) for t in targets]):
            print(targets)
            raise AssertionError("Parsed invalid joint")

        super().__init__(robot=robot, name=name, targets=targets, sensortype='JointVelocity', **kwargs)
        self.returns += ['link']


class NodeContactForce(MultiSensor):
    def __init__(self, robot=None, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        _targets = []
        for coll in targets:
            if type(coll) is str:
                coll_object = robot.get_collision_by_name(coll)
                if coll_object is None:
                    raise NameError("There is no Collision with the name " + coll)
                else:
                    _targets += [coll_object]
            elif isinstance(coll, representation.Link):
                _targets += [coll]
            else:
                raise TypeError("Received a target that is neither a Collision nor a link name! Type:", type(coll))

        targets = _targets

        if not all([isinstance(t, representation.Collision) for t in targets]):
            print(targets)
            raise AssertionError("Parsed invalid collision")

        super().__init__(robot=robot, name=name, targets=targets, sensortype='NodeContactForce', **kwargs)

        self.returns += ['link']


class NodeCOM(MultiSensor):
    def __init__(self, robot=None, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]
        _targets = []
        for link in targets:
            if type(link) is str:
                if robot.get_link(link) is None:
                    raise NameError("There is no link with the name " + link)
                else:
                    _targets += [robot.get_link(link)]
            elif isinstance(link, representation.Link):
                _targets += [link]
            else:
                raise TypeError("Received a target that is neither a Link nor a link name! Type:", type(link))

        targets = _targets

        if not all([isinstance(t, representation.Link) for t in targets]):
            print(targets)
            raise AssertionError("Parsed invalid link")

        super().__init__(robot=robot, name=name, targets=targets, sensortype='NodeCOM', **kwargs)
        self.returns += ['link']


class NodePosition(MultiSensor):
    def __init__(self, robot=None, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]
        _targets = []
        for link in targets:
            if type(link) is str:
                if robot.get_link(link) is None:
                    raise NameError("There is no link with the name " + link)
                else:
                    _targets += [robot.get_link(link)]
            elif isinstance(link, representation.Link):
                _targets += [link]
            else:
                raise TypeError("Received a target that is neither a Link nor a link name! Type:", type(link))

        targets = _targets

        if not all([isinstance(t, representation.Link) for t in targets]):
            print(targets)
            raise AssertionError("Parsed invalid link")

        super().__init__(robot=robot, name=name, targets=targets, sensortype='NodePosition', **kwargs)
        self.returns += ['link']


class NodeRotation(MultiSensor):
    def __init__(self, robot=None, name=None, targets=None, **kwargs):
        if not isinstance(targets, list):
            targets = [targets]

        _targets = []
        for link in targets:
            if type(link) is str:
                if robot.get_link(link) is None:
                    raise NameError("There is no link with the name " + link)
                else:
                    _targets += [robot.get_link(link)]
            elif isinstance(link, representation.Link):
                _targets += [link]
            else:
                raise TypeError("Received a target that is neither a Link nor a link name! Type:", type(link))

        targets = _targets

        if not all([isinstance(t, representation.Link) for t in targets]):
            print(targets)
            raise AssertionError("Parsed invalid link")

        super().__init__(robot=robot, name=name, targets=targets, sensortype='NodeRotation', **kwargs)
        self.returns += ['link']
