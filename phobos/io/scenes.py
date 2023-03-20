from .base import Representation
from . import representation
from .smurf_reflection import SmurfBase
from .xml_factory import singular as _singular, plural as _plural

from ..commandline_logging import get_logger
log = get_logger(__name__)

__IMPORTS__ = [x for x in dir() if not x.startswith("__")]


class Gravity(Representation, SmurfBase):
    def __init__(self, gravity_list):
        super(Gravity, self).__init__()
        self.gravity = gravity_list

    @property
    def gravity_list(self):
        return self._gravity

    @property
    def gravity(self):
        return {k: v for k, v in zip(["x", "y", "z"], self._gravity)}

    @gravity.setter
    def gravity(self, value):
        if type(value) == list:
            assert len(value) == 3
            self._gravity = value
        elif type(value) == dict:
            assert len(value.values()) == 3
        else:
            raise ValueError


class ODE(Representation, SmurfBase):
    def __init__(self, cfm, erp):
        super(ODE, self).__init__(cfm=cfm, erp=erp)


class Physics(Representation, SmurfBase):
    def __init__(self, ode, gravity):
        assert type(ode) == ODE
        assert type(gravity) == Gravity
        super(Physics, self).__init__(ode=ode, gravity=gravity)


class Frame(Representation):
    _class_variables = ["name", "attached_to", "origin"]
    _type_dict = {"attached_to": "frames"}

    def __init__(self, name=None, attached_to=None, origin=None):
        super(Frame, self).__init__()
        self.name = name
        self.attached_to = attached_to
        self.origin = origin


class Entity(Representation, SmurfBase):
    _class_variables = ["origin"]

    def __init__(self, name=None, model=None, file=None, type=None, origin=None, frames=None):
        self.model = model
        self.origin = origin
        self._file = file
        self._frames = []
        for frame in frames:
            if "::" not in _plural(frame):
                self._frames.append(frame)
        SmurfBase.__init__(self, name=name, type=type if type is not None else self.file.rsplit(".", 1)[-1])

    @property
    def file(self):
        return self.model.smurffile if self.model.smurffile is not None else self.model.xmlfile

    @property
    def frames(self):
        if self._related_robot_instance is not None:
            out = []
            for entity in self._related_robot_instance.entites:
                if str(entity.origin.relative_to).startswith(str(self)):
                    link = entity.get_link(entity.origin.relative_to)
                    out.append(Frame(
                        name=str(entity)+"::"+str(link),
                        attached_to=str(link),
                        origin=representation.Pose(relative_to=link)
                    ))
            return out
        return None


# [TODO v2.1.0] Add SDF-/MARS-Scene support
class Environment(Representation, SmurfBase):
    def __init__(self):
        super(Environment, self).__init__()


class World(Representation, SmurfBase):
    def __init__(self, entities, frames, physics=None):
        super(World, self).__init__()
        self.entities = _plural(entities)
        self.physics = _singular(physics)
        self._frames = []
        for frame in frames:
            if "::" not in _plural(frame):
                self._frames.append(frame)

    def get_aggregate(self, typeName, elem):
        elem = str(elem)
        if typeName.startswith("frame"):
            if elem == "WORLD":
                return Frame(name="WORLD")
            if "::" in elem:
                entity, link = elem.split("::")
                assert self.get_aggregate("entities", entity) is not None
                assert entity.get_aggregate("links", link) is not None
                return Frame(
                    name=elem,
                    attached_to="WORLD",
                    origin=entity.origin.dot(representation.Pose.from_matrix(entity.get_transformation(link)))
                )
            else:
                for e in self._frames:
                    if str(e) == elem:
                        return e
        elif len(getattr(self, typeName, None)) == list:
            for e in getattr(self, typeName):
                if str(e) == elem:
                    return e
        else:
            raise TypeError(f"World has no {typeName}")

