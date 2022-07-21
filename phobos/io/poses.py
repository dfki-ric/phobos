from ..io.smurf_reflection import SmurfBase


class JointPose(SmurfBase):
    _class_variables = ["joint"]

    def __init__(self, robot=None, joint=None, position=None):
        super().__init__(robot=robot, name=None, joint=joint)
        self._position = position

        self.excludes += ['_position', 'name']
        self.returns += ['pose']

    @property
    def position(self):
        if self._joint and self._joint.limit:
            return min(self._joint.limit.upper, max(self._joint.limit.lower, self._position))
        else:
            return self._position

    @position.setter
    def position(self, positionval):
        if type(positionval) in [float, int]:
            self._position = positionval

    @property
    def pose(self):
        return {str(self.joint): self.position}

    @pose.setter
    def pose(self, value):
        self.position = value


class JointPoseSet(SmurfBase):
    def __init__(self, robot=None, name=None, configuration=None):
        super().__init__(robot=robot, name=name)

        self.configuration = None

        if type(configuration) == dict:
            self.set_joints(configuration, robot)
        elif type(configuration) == list:
            self.configuration = configuration

        assert all(type(x) == JointPose for x in self.configuration)

        self.excludes += ['configuration']
        self.returns += ['joints']

    def remove_joint(self, jointname):
        for joint_pose in self.configuration:
            if str(joint_pose.joint) == jointname:
                self.configuration.remove(joint_pose)
                break

    def set_joint_pose(self, jointname, value):
        for joint_pose in self.configuration:
            if str(joint_pose.joint) == jointname:
                joint_pose.position = value
                break

    @property
    def joints(self):
        returns = {}
        for jpose in self.configuration:
            returns.update(jpose.pose)
        return returns

    def set_joints(self, configuration, robot):
        if isinstance(configuration, dict):
            self.configuration = []
            for joint, position in configuration.items():
                # Check for joint
                if robot.get_joint(joint):
                    c_joint = robot.get_joint(joint)
                    self.configuration.append(
                        JointPose(robot=robot, joint=c_joint, position=position)
                    )

    def link_with_robot(self, robot, check_linkage_later=False):
        super(JointPoseSet, self).link_with_robot(robot, check_linkage_later=True)
        self.configuration.link_with_robot(robot, check_linkage_later=True)
        if not check_linkage_later:
            self.check_linkage()

    def unlink_from_robot(self):
        super(JointPoseSet, self).unlink_from_robot()
        self.configuration.unlink_from_robot()
