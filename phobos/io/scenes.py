from .base import Representation
from .smurf_reflection import SmurfBase

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
    def __init__(self, cfm=None, erp=None):
        super(ODE, self).__init__(cfm=cfm, erp=erp)


class Physics(Representation, SmurfBase):
    def __init__(self, ode=None, gravity=None):
        assert ode is None or type(ode) == ODE
        assert gravity is None or type(gravity) == Gravity
        super(Physics, self).__init__(ode=ode, gravity=gravity)


class Frame(Representation):
    _class_variables = ["name", "attached_to", "origin"]
    _type_dict = {"attached_to": "frames"}

    def __init__(self, name=None, attached_to=None, origin=None):
        super(Frame, self).__init__()
        self.name = name
        self.attached_to = attached_to
        self.origin = origin


# [TODO v2.1.0] Add SDF-/MARS-Scene support
class Environment(Representation, SmurfBase):
    def __init__(self):
        super(Environment, self).__init__()


