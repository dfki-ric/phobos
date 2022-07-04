from ..io.smurf_reflection import SmurfBase
from ..utils import tree


class ConstraintAxis(SmurfBase):
    def __init__(self, name, axis, **kwargs):
        kwargs["name"] = name
        kwargs["axis"] = axis
        super(ConstraintAxis, self).__init__(**kwargs)


class LoopConstraint(SmurfBase):
    def __init__(self, cut_joint, predecessor_body, successor_body, constraint_axes=None):
        kwargs = {"cut_joint": cut_joint,
                  "predecessor_body": predecessor_body,
                  "successor_body": successor_body}
        if constraint_axes is None:
            constraint_axes = []
        assert all([isinstance(ax, ConstraintAxis) for ax in constraint_axes])
        kwargs["constraint_axes"] = constraint_axes
        super(LoopConstraint, self).__init__(**kwargs)


class JointDependency(SmurfBase):
    type_dict = {
        "joint_name": "joint"
    }

    def __init__(self, joint_name, multiplier, offset):
        kwargs = {"joint": joint_name,
                  "multiplier": multiplier,
                  "offset": offset}
        super(JointDependency, self).__init__(**kwargs)


class MultiJointDependency(SmurfBase):
    type_dict = {
        "joint": "joint"
    }

    def __init__(self, joint, joint_dependencies=None, name=None):
        if joint_dependencies is None:
            joint_dependencies = []
        assert all([isinstance(jd, JointDependency) for jd in joint_dependencies])
        kwargs = {"joint": joint,
                  "joint_dependencies": joint_dependencies}
        if name is not None:
            kwargs["name"] = name
        super(MultiJointDependency, self).__init__(**kwargs)

    def add_dependency(self, joint_dependency):
        assert isinstance(joint_dependency, JointDependency)
        self.joint_dependencies += [joint_dependency]

    def is_empty(self):
        return len(self.joint_dependencies) == 0


class HyrodynAnnotation(SmurfBase):
    type_dict = {
        "jointnames_spanningtree": "joint",
        "jointnames_active": "joint",
        "jointnames_independent": "joint",
        "jointnames_dependent": "joint",
        "jointnames": "joint"
    }

    def __init__(self, robot, name,
                 jointnames_spanningtree=None, jointnames_active=None,
                 jointnames_independent=None, jointnames_dependent=None,
                 jointnames=None, contextual_name=None, file_path=None,
                 loop_constraints=None, multi_joint_dependencies=None,
                 type=None, around=None, auto_gen=False):
        if multi_joint_dependencies is None:
            multi_joint_dependencies = []
        if loop_constraints is None:
            loop_constraints = []
        kwargs = {
            "name": name,
            "contextual_name": contextual_name if contextual_name is not None else name,
            "jointnames_spanningtree": jointnames_spanningtree,
            "jointnames": None,
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
        super().__init__(robot=robot, **kwargs)
        self.fill_jointnames(robot)
        assert self.jointnames is not None
        if jointnames is not None:
            self.jointnames += [j for j in jointnames if j not in self.jointnames]
        self.returns = [key for key in kwargs.keys() if key not in self.excludes]

    def get_refl_vars(self):
        out = super(HyrodynAnnotation, self).get_refl_vars()
        if len(self.loop_constraints) == 0 and "loop_constraints" in out:
            out = [o for o in out if o != "loop_constraints"]
        if len(self.multi_joint_dependencies) == 0 and "multi_joint_dependencies" in out:
            out = [o for o in out if o != "multi_joint_dependencies"]
        return out

    def is_empty(self):
        return len(self.jointnames) == 0

    def is_valid(self, robot):
        joints = self.get_joints()
        out = all([robot.get_joint(j) is not None for j in joints])
        for lc in self.loop_constraints:
            out &= robot.get_link(lc.predecessor_body) is not None
            out &= robot.get_link(lc.successor_body) is not None
        for mjd in self.multi_joint_dependencies:
            out &= robot.get_joint(mjd.joint) is not None
            for jd in mjd.joint_dependencies:
                out &= robot.get_joint(jd.joint) is not None
        # (not done as this is computationally expensive) check if the joints are in a correct order
        # ToDO check if all fixed joints in the area of this submechanism are included
        return out

    def fill_jointnames(self, robot):
        if len(self.jointnames_spanningtree) == 0:
            self.jointnames = []
        else:
            submodel = robot.instantiate_submodel(definition={
                "name": "#sub_mech#",
                "start": self.get_root(robot),
                "stop": self.get_leaves(robot)
            })
            self.jointnames = [j.name for j in submodel.joints]

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


class Submechanism(HyrodynAnnotation):
    def __init__(self, robot, name,
                 jointnames_spanningtree, jointnames_active, jointnames_independent, jointnames=None,
                 contextual_name=None, file_path=None, type="numerical",
                 loop_constraints=None, multi_joint_dependencies=None, auto_gen=False):
        super(Submechanism, self).__init__(
            robot, name,
            jointnames_spanningtree=jointnames_spanningtree,
            jointnames_active=jointnames_active, jointnames_independent=jointnames_independent,
            contextual_name=contextual_name, jointnames=jointnames, file_path=file_path,
            loop_constraints=loop_constraints, multi_joint_dependencies=multi_joint_dependencies,
            type=type, around=None, auto_gen=auto_gen
        )


class Exoskeleton(HyrodynAnnotation):
    def __init__(self, robot, name, around,
                 jointnames_spanningtree, jointnames_dependent,
                 jointnames=None, contextual_name=None, file_path=None,
                 loop_constraints=None, multi_joint_dependencies=None,
                 auto_gen=False):
        super(Exoskeleton, self).__init__(
            robot, name,
            jointnames_spanningtree=jointnames_spanningtree, jointnames_dependent=jointnames_dependent,
            jointnames=jointnames, contextual_name=contextual_name, file_path=file_path,
            loop_constraints=loop_constraints, multi_joint_dependencies=multi_joint_dependencies,
            around=around, auto_gen=auto_gen, type=None
        )
