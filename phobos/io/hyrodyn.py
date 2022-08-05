from .smurf_reflection import SmurfBase
from .representation import JointMimic
from ..utils import tree

__IMPORTS__ = [x for x in dir() if not x.startswith("__")]


class ConstraintAxis(SmurfBase):
    def __init__(self, name, axis, **kwargs):
        kwargs["name"] = name
        kwargs["axis"] = axis
        super(ConstraintAxis, self).__init__(**kwargs)


class LoopConstraint(SmurfBase):
    _class_variables = ["cut_joint", "predecessor_body", "successor_body"]
    type_dict = {
        "cut_joint": "joint",
        "predecessor_body": "link",
        "successor_body": "link"
    }

    def __init__(self, cut_joint, predecessor_body, successor_body, constraint_axes=None):
        kwargs = {"cut_joint": cut_joint,
                  "predecessor_body": predecessor_body,
                  "successor_body": successor_body}
        if constraint_axes is None:
            constraint_axes = []
        assert all([isinstance(ax, ConstraintAxis) for ax in constraint_axes])
        kwargs["constraint_axes"] = constraint_axes
        super(LoopConstraint, self).__init__(**kwargs)


class MultiJointDependency(SmurfBase):
    _class_variables = ["joint", "joint_dependencies"]

    def __init__(self, joint, joint_dependencies=None, name=None):
        if joint_dependencies is None:
            joint_dependencies = []
        assert all([isinstance(jd, JointMimic) for jd in joint_dependencies])
        kwargs = {"joint": joint,
                  "joint_dependencies": joint_dependencies}
        if name is not None:
            kwargs["name"] = name
        super(MultiJointDependency, self).__init__(**kwargs)

    def add_dependency(self, joint_dependency):
        assert isinstance(joint_dependency, JointMimic)
        self.joint_dependencies += [joint_dependency]

    def is_empty(self):
        return len(self.joint_dependencies) == 0

    def link_with_robot(self, robot, check_linkage_later=False):
        super(MultiJointDependency, self).link_with_robot(robot)
        for jd in self.joint_dependencies:
            jd.link_with_robot(robot)
            jd.check_linkage()

    def unlink_from_robot(self):
        super(MultiJointDependency, self).unlink_from_robot()
        for jd in self.joint_dependencies:
            jd.unlink_from_robot()


class HyrodynAnnotation(SmurfBase):
    type_dict = {
        "jointnames_spanningtree": "joint",
        "jointnames_active": "joint",
        "jointnames_independent": "joint",
        "jointnames_dependent": "joint",
        "jointnames": "joint"
    }

    def __init__(self, name, contextual_name,
                 jointnames_spanningtree=None, jointnames_active=None,
                 jointnames_independent=None, jointnames_dependent=None,
                 jointnames=None, file_path=None,
                 loop_constraints=None, multi_joint_dependencies=None,
                 type=None, around=None, auto_gen=False):
        kwargs = {
            "name": name,
            "contextual_name": contextual_name,
            "jointnames_spanningtree": jointnames_spanningtree,
            "jointnames": jointnames,
            "multi_joint_dependencies": multi_joint_dependencies,
            "loop_constraints": loop_constraints,
            "auto_gen": auto_gen
        }
        if file_path is not None and file_path is not False:
            kwargs["file_path"] = file_path
        if jointnames_independent is not None:
            kwargs["jointnames_independent"] = jointnames_independent
        if jointnames_dependent is not None:
            kwargs["jointnames_dependent"] = jointnames_dependent
        if jointnames_active is not None:
            kwargs["jointnames_active"] = jointnames_active
        if type is not None:
            kwargs["type"] = type
        if around is not None:
            kwargs["around"] = around
        super().__init__(**kwargs)
        self.returns = [key for key in kwargs.keys() if key not in self.excludes]

    def set_unique_name(self, value):
        self.contextual_name = value

    def __str__(self):
        return self.contextual_name

    def is_empty(self):
        return len(self.jointnames) == 0

    def is_valid(self, robot):
        joints = self.get_joints()
        out = all([robot.get_joint(j) is not None for j in joints])
        # (not done as this is computationally expensive) check if the joints are in a correct order
        # ToDO check if all fixed joints in the area of this submechanism are included
        return out

    def fill_jointnames(self, robot):
        if len(self.jointnames_spanningtree) == 0:
            self.jointnames = []
        else:
            _, self.jointnames = robot.get_links_and_joints_in_subtree(start=self.get_root(robot), stop=self.get_leaves(robot))

    def get_joints(self):
        return list(set(([] if self.jointnames is None else self.jointnames) + self.jointnames_spanningtree))

    def get_root(self, robot):
        return tree.find_common_root(robot, self.get_joints())

    def get_root_joints(self, robot):
        out = [jn for jn in self.get_joints() if jn in robot.get_children(self.get_root(robot))]
        assert len(out) > 0
        return out

    def get_leaves(self, robot):
        return tree.find_leaves(robot, self.get_joints())

    def get_relative_links(self, robot):
        links = []
        for jointname in self.get_joints():
            joint = robot.get_joint(jointname)
            links += [joint.child, joint.parent]
        return list(set(links))

    def link_with_robot(self, robot, check_linkage_later=False):
        self.fill_jointnames(robot)
        super(HyrodynAnnotation, self).link_with_robot(robot)


class Submechanism(HyrodynAnnotation):
    _class_variables = ["name", "jointnames", "jointnames_spanningtree", "jointnames_active", "jointnames_independent"]

    def __init__(self, name, contextual_name,
                 jointnames_spanningtree, jointnames_active, jointnames_independent, jointnames=None,
                 file_path=None, type="numerical",
                 loop_constraints=None, multi_joint_dependencies=None, auto_gen=False):
        super(Submechanism, self).__init__(
            name=name, contextual_name=contextual_name,
            jointnames_spanningtree=jointnames_spanningtree,
            jointnames_active=jointnames_active, jointnames_independent=jointnames_independent,
            jointnames=jointnames, file_path=file_path,
            loop_constraints=loop_constraints, multi_joint_dependencies=multi_joint_dependencies,
            type=type, around=None, auto_gen=auto_gen
        )
        if multi_joint_dependencies is None:
            multi_joint_dependencies = []
        if loop_constraints is None:
            loop_constraints = []
        self._multi_joint_dependencies = multi_joint_dependencies
        self._loop_constraints = loop_constraints
        self.returns += ["multi_joint_dependencies"]

    @property
    def multi_joint_dependencies(self):
        if self._related_robot_instance is not None:
            for j in self.get_joints():
                joint = self._related_robot_instance.get_joint(j, verbose=True)
                assert joint is not None
                if len(joint.joint_dependencies) > 1:
                    self._multi_joint_dependencies.append({"name": j.name, "depends_on": j.joint_dependencies})
        if self._multi_joint_dependencies is not None and len(self._multi_joint_dependencies) > 0:
            return self._multi_joint_dependencies
        else:
            return None

    @multi_joint_dependencies.setter
    def multi_joint_dependencies(self, value):
        if self._related_robot_instance is None:
            self._multi_joint_dependencies = value
        else:
            #Todo
            pass

    @property
    def loop_constraints(self):
        if self._related_robot_instance is not None:
            for j in self.get_joints():
                joint = self._related_robot_instance.get_joint(j)
                if joint.cut_joint:
                    self._loop_constraints.append(LoopConstraint(cut_joint=joint.name, predecessor_body=joint.parent,
                                                                 successor_body=joint.child,
                                                                 constraint_axes=joint.constraint_axes))
        if self._loop_constraints is not None and len(self._loop_constraints) > 0:
            return self._loop_constraints
        else:
            return None

    @loop_constraints.setter
    def loop_constraints(self, value):
        if self._related_robot_instance is None:
            self._loop_constraints = value
        else:
            #Todo
            pass

    def link_with_robot(self, robot, check_linkage_later=False):
        #Todo
        super(Submechanism, self).link_with_robot(robot, check_linkage_later=True)

    def unlink_from_robot(self):
        #Todo
        super(Submechanism, self).unlink_from_robot()


class Exoskeleton(HyrodynAnnotation):
    _class_variables = ["name", "jointnames", "jointnames_spanningtree", "jointnames_dependent", "around"]

    def __init__(self, name, around, contextual_name,
                 jointnames_spanningtree, jointnames_dependent,
                 jointnames=None, file_path=None,
                 loop_constraints=None, multi_joint_dependencies=None,
                 auto_gen=False):
        super(Exoskeleton, self).__init__(
            name=name, contextual_name=contextual_name,
            jointnames_spanningtree=jointnames_spanningtree, jointnames_dependent=jointnames_dependent,
            jointnames=jointnames, file_path=file_path,
            around=around, auto_gen=auto_gen, type=None
        )
