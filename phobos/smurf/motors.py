from ..io.base import Representation
from ..io.representation import JointLimit
from ..io.smurf_reflection import SmurfBase


class Motor(Representation, SmurfBase):
    type_dict = {
        "joint": "joint"
    }

    def __init__(self, name=None, joint=None, **kwargs):
        SmurfBase.__init__(name=name, joint=joint, link=None, **kwargs)
        # This is hardcoded information
        self.returns += ['joint', 'maxEffort', 'maxSpeed', 'maxValue', 'minValue']

    def linking_callback(self):
        setattr(self._joint, "motor", self)

    @property
    def maxEffort(self):
        if self._joint:
            return self._joint.limit.effort if self._joint.limit else 0
        else:
            return 0

    @maxEffort.setter
    def maxEffort(self, effort):
        if type(effort) in [float, int] and effort > 0:
            if self._joint and self._joint.limit:
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
        return self._joint.mimic.joint

    @property
    def mimic_multiplier(self):
        return self._joint.mimic.multiplier

    @property
    def mimic_offset(self):
        return self._joint.mimic.offset
