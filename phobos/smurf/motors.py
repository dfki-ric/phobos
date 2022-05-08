from .base import SmurfAnnotation
from ..io import representation


class Motor(SmurfAnnotation):
    def __init__(self, robot=None, name=None, joint=None, **kwargs):
        super().__init__(robot=robot, name=name, joint=joint, link=None, **kwargs)
        # This is hardcoded information
        self.returns += ['joint', 'maxEffort', 'maxSpeed', 'maxValue', 'minValue', 'reducedDataPackage',
                         'noDataPackage']

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
                self._joint.limit = representation.JointLimit(
                    effort=self.maxEffort,
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
                self._joint.limit = representation.JointLimit(
                    effort=self.maxEffort,
                    velocity=self.maxSpeed,
                    lower=self.minValue,
                    upper=self.maxValue
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
                self._joint.limit = representation.JointLimit(
                    effort=self.maxEffort,
                    velocity=self.maxSpeed,
                    lower=self.minValue,
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
                self._joint.limit = representation.JointLimit(
                    effort=self.maxEffort,
                    velocity=self.maxSpeed,
                    lower=self.minValue,
                    upper=self.maxValue
                )


class MimicMotor(Motor):
    def __init__(self, robot=None, name=None, joint=None, mimic_motor=None, mimic_multiplier=1,
                 mimic_offset=0, **kwargs):
        super().__init__(robot=robot, name=name, joint=joint, **kwargs)

        self.mimic_motor = mimic_motor
        self.mimic_multiplier = mimic_multiplier
        self.mimic_offset = mimic_offset
        # self._mimic = mimic_motor
        self.returns += ['mimic_motor', 'mimic_multiplier', 'mimic_offset']
        # self.excludes += ['_mimic']

    # # Define the mimic motor, which is a motor
    # @property
    # def mimic_motor(self):
    #     return self._mimic.name if self._mimic else None
    #
    # @mimic_motor.setter
    # def mimic_motor(self, motor):
    #     if not isinstance(motor, Motor):
    #         self._mimic = None
    #     else:
    #         self._mimic = motor
