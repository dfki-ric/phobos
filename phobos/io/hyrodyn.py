import numpy

from .representation import ConstraintAxis
from .smurf_reflection import SmurfBase
from .xml_factory import plural as _plural
from ..commandline_logging import get_logger
from ..utils import tree
from ..utils.transform import matrix_to_quaternion

log = get_logger(__name__)

__IMPORTS__ = [x for x in dir() if not x.startswith("__")]


# This class is only for smurf formatting and isn't used for data storage
class LoopConstraint(SmurfBase):
    _class_variables = ["joint", "predecessor_body", "successor_body"]
    _type_dict = {
        "joint": "joints",
        "predecessor_body": "links",
        "successor_body": "links"
    }
    _handle_ambiguous = False

    def __init__(self, cut_joint, predecessor_body=None, successor_body=None, constraint_axes=None, **kwargs):
        self.joint = cut_joint
        assert self.joint is not None
        assert predecessor_body is not None
        assert successor_body is not None
        assert constraint_axes is not None
        self.predecessor_body = predecessor_body
        self.successor_body = successor_body
        self._constraint_axes = [ConstraintAxis(**ca) for ca in _plural(constraint_axes)]
        SmurfBase.__init__(
            self,
            returns=["cut_joint", "predecessor_body", "successor_body", "constraint_axes"],
            **kwargs
        )
        self.excludes += ["joint"]

    @property
    def cut_joint(self):
        return self.joint

    @property
    def constraint_axes(self):
        if type(self.joint) == str:
            return self._constraint_axes
        return self.joint.constraint_axes

    def stringable(self):
        return False


# This class is only for smurf formatting and isn't used for data storage
class MultiJointDependency(SmurfBase):
    _class_variables = ["name", "depends_on"]

    def __init__(self, joint=None, **kwargs):
        self.joint = joint
        super(MultiJointDependency, self).__init__(returns=["name", "depends_on"], **kwargs)

    def is_empty(self):
        return len(self.joint.joint_dependencies) == 0

    @property
    def name(self):
        return str(self.joint)

    @property
    def depends_on(self):
        return self.joint.joint_dependencies


class HyrodynAnnotation(SmurfBase):
    type_dict = {
        "jointnames_spanningtree": "joints",
        "jointnames_active": "joints",
        "jointnames_independent": "joints",
        "jointnames_dependent": "joints",
        "jointnames": "joints"
    }

    def __init__(self, name, contextual_name,
                 jointnames_spanningtree=None, jointnames_active=None,
                 jointnames_independent=None, jointnames_dependent=None,
                 jointnames=None, file_path=None,
                 type=None, around=None, auto_gen=False, **kwargs):
        kwargs = {
            "name": name,
            "contextual_name": contextual_name,
            "jointnames_spanningtree": jointnames_spanningtree if jointnames_spanningtree is not None else [],
            "jointnames": jointnames,
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
        return len(self.get_joints()) == 0

    def is_only_fixed(self):
        return self.jointnames is not None and len(self.jointnames) > 0 and len(self.get_joints()) == len(self.jointnames)

    def is_valid(self, robot):
        joints = self.get_joints()
        out = all([robot.get_joint(j) is not None for j in joints])
        # (not done as this is computationally expensive) check if the joints are in a correct order
        # [TODO v2.1.0] check if all fixed joints in the area of this submechanism are included
        return out

    def get_joints(self):
        return list(set(([] if self.jointnames is None else self.jointnames) + self.jointnames_spanningtree))

    def get_root(self, robot):
        return tree.find_common_root(robot, self.get_joints())

    def get_root_joints(self, robot):
        "Returns the joints that are connected to the root link of this submechanism"
        root_joints = [jn for jn in self.get_joints() if jn in robot.get_children(self.get_root(robot))]
        return root_joints

    def get_leaves(self, robot, **kwargs):
        return tree.find_leaves(robot, self.get_joints())

    def get_children(self, robot):
        children = set()
        for joint in self.get_joints():
            children.update(robot.get_children(robot.get_joint(joint).child))
        return children.difference([str(j) for j in self.get_joints()])

    def get_relative_links(self, robot):
        links = []
        for jointname in self.get_joints():
            joint = robot.get_joint(jointname)
            links += [joint.child, joint.parent]
        return list(set(links))

    def get_internal_links(self, robot):
        return [link for link in self.get_relative_links(robot) if link != self.get_root(robot) and link not in self.get_leaves(robot)]

    def link_with_robot(self, robot, check_linkage_later=False):
        super(HyrodynAnnotation, self).link_with_robot(robot)

    def regenerate(self, robot, absorb_fixed_upwards=False, absorb_fixed_downwards=False):
        raise NotImplementedError

    def get_index(self, robot):
        indices = [[str(j) for j in robot.get_joints_ordered_df()].index(str(smj)) for smj in self.get_joints()]
        return min(indices)

    def get_rotation_convention(self):
        """For a linked submechanism this returns the frame convention of this submechanism relative to the root"""
        assert self._related_robot_instance is not None
        if self.type in ["serial", "R"] or len(self.jointnames_spanningtree) <= 1:
            return None

        rotation_convention = None
        rotations = []
        for joint in self._jointnames_spanningtree:
            T = self._related_robot_instance.get_transformation(joint.child)
            rotations.append(matrix_to_quaternion(T[0:3, 0:3]))
        rotation_convention = numpy.average(rotations, axis=0)

        diffs = [rotation_convention - l for l in rotations]
        if any([numpy.linalg.norm(d) > 1e-3 for d in diffs]):
            # check if there's only one deviant
            log.warning(f"Frame in submechanisms {str(self)} orientations don't rely all to the same convention (xyzw)\n"+"\n".join([str(r) for r in rotations]))
            new_diffs = [rotations[0] - l for l in rotations]
            deviants = numpy.where([numpy.linalg.norm(d) > 1e-3 for d in new_diffs])
            if len(deviants) == 1:
                rotation_convention = numpy.average([r for i, r in enumerate(rotations) if i not in deviants], axis=0)
                log.warning("Guessing main convention: " + str(rotation_convention))
            elif len(deviants == len(rotations)-1):
                rotation_convention = numpy.average([r for i, r in enumerate(rotations) if i in deviants], axis=0)
                log.warning("Guessing main convention: " + str(rotation_convention))
            else:
                return None
        return rotation_convention


class Submechanism(HyrodynAnnotation):
    _class_variables = ["name", "jointnames", "jointnames_spanningtree", "jointnames_active", "jointnames_independent", "loop_constraints"]

    def __init__(self, name, contextual_name,
                 jointnames_spanningtree=None, jointnames_active=None, jointnames_independent=None, jointnames=None,
                 file_path=None, type="numerical",
                 loop_constraints=None, multi_joint_dependencies=None, auto_gen=False, **kwargs):
        if jointnames_independent is None:
            jointnames_independent = []
        if jointnames_active is None:
            jointnames_active = []
        if jointnames_spanningtree is None:
            jointnames_spanningtree = []
        super(Submechanism, self).__init__(
            name=name, contextual_name=contextual_name,
            jointnames_spanningtree=jointnames_spanningtree,
            jointnames_active=jointnames_active, jointnames_independent=jointnames_independent,
            jointnames=jointnames, file_path=file_path,
            type=type, around=None, auto_gen=auto_gen, **kwargs
        )
        if multi_joint_dependencies is None:
            multi_joint_dependencies = []
        if loop_constraints is None:
            loop_constraints = []
        self._multi_joint_dependencies = multi_joint_dependencies
        self.loop_constraints = [LoopConstraint(**lc) for lc in loop_constraints]
        self.returns += ["multi_joint_dependencies", "loop_constraints"]
        self._has_unlinked_changes = multi_joint_dependencies is not None or loop_constraints is not None

    @property
    def multi_joint_dependencies(self):
        if self._related_robot_instance is not None:
            # update the internal multi_joint_dependencies
            self._multi_joint_dependencies = []
            for j in self.get_joints() + [lc.cut_joint for lc in self.loop_constraints]:
                joint = self._related_robot_instance.get_joint(j, verbose=True)
                assert joint is not None, f"(see above for more info)\n{self.get_joints()}\n{[lc.cut_joint for lc in self.loop_constraints]}"
                if len(joint.joint_dependencies) > 1:
                    self._multi_joint_dependencies.append(MultiJointDependency(joint).to_yaml())
        if self._multi_joint_dependencies is not None and len(self._multi_joint_dependencies) > 0:
            return self._multi_joint_dependencies
        else:
            return []

    @multi_joint_dependencies.setter
    def multi_joint_dependencies(self, value):
        if self._related_robot_instance is None:
            self._multi_joint_dependencies = value
        else:
            # [ToDo v2.1.0] set the joint dependencies from here
            raise RuntimeError("Please set the joint_dependencies of the corresponding joint.")

    def link_with_robot(self, robot, check_linkage_later=False):
        super(Submechanism, self).link_with_robot(robot, check_linkage_later=True)
        if self._multi_joint_dependencies is not None and len(self._multi_joint_dependencies) > 0:
            for mjd in self._multi_joint_dependencies:
                joint = robot.get_joint(mjd["name"])
                joint.joint_dependencies = []
                for jd in mjd["depends_on"]:
                    joint.joint_dependencies = joint.joint_dependencies + [jd]
        if not check_linkage_later:
            assert self.check_linkage()

    def unlink_from_robot(self, check_linkage_later=False):
        mjd = self.multi_joint_dependencies
        super(Submechanism, self).unlink_from_robot()
        self._multi_joint_dependencies = mjd
        if not check_linkage_later:
            assert self.check_unlinkage()

    def regenerate(self, robot, absorb_fixed_upwards=False, absorb_fixed_downwards=False):
        """
        Sorts all joints depth first and fills the jointsnames entry already with all fied joints that are inbetween the submechanism
        Fixed joints that are between this and other submechanisms have to be managed by the robot.
        Args:
            robot: The robot instance to which this submechanism belongs

        Returns:
            None
        """
        jointnames = set([str(joint) for joint in self.get_joints() if robot.get_joint(joint) is not None])
        if absorb_fixed_upwards:
            root = tree.skip_upwards_over_fixed(robot, self.get_root(robot), only_single_parents=True)
        else:
            root = self.get_root(robot)
        leaves = self.get_leaves(robot)
        if absorb_fixed_downwards:
            _leaves = []
            for l in leaves:
                _leaves += tree.skip_downwards_over_fixed(robot, l, self)
            leaves = list(set(_leaves))
        for leave in leaves:
            chain = robot.get_chain(root, robot.get_link(leave, verbose=True), links=False)
            jointnames = jointnames.union(chain)
        self.jointnames = sorted(jointnames.union(self.jointnames_spanningtree), key=lambda x: [str(y) for y in robot.get_joints_ordered_df()].index(x))
        self.jointnames_spanningtree = sorted(self.jointnames_spanningtree, key=lambda x: [str(y) for y in robot.get_joints_ordered_df()].index(x))
        self.jointnames_active = sorted(self.jointnames_active, key=lambda x: self.jointnames_spanningtree.index(x))
        self.jointnames_independent = sorted(self.jointnames_independent, key=lambda x: self.jointnames_spanningtree.index(x))


class Exoskeleton(HyrodynAnnotation):
    _class_variables = ["name", "jointnames", "jointnames_spanningtree", "jointnames_dependent", "around"]

    def __init__(self, name, around,
                 jointnames_spanningtree=None, jointnames_dependent=None,
                 jointnames=None, file_path=None, contextual_name=None,
                 auto_gen=False):
        super(Exoskeleton, self).__init__(
            name=name, contextual_name=contextual_name if contextual_name is not None else name,
            jointnames_spanningtree=jointnames_spanningtree, jointnames_dependent=jointnames_dependent,
            jointnames=jointnames, file_path=file_path,
            around=around, auto_gen=auto_gen, type=None
        )
        self.file_path = None
        self.returns += ["jointnames_dependent", "jointnames", "jointnames_spanningtree"]

    def regenerate(self, robot, absorb_fixed_upwards=False, absorb_fixed_downwards=False):
        joints_df = robot.get_joints_ordered_df()
        self.jointnames = [str(j) for j in joints_df if j._child.is_human]
        self.jointnames_spanningtree = [str(j) for j in joints_df if j._child.is_human and j.joint_type != "fixed"]
        self.jointnames_dependent = [str(j.mimic.joint) for j in joints_df if j._child.is_human and j.joint_type != "fixed"]

    def get_leaves(self, robot, include_dependent=False, **kwargs):
        return tree.find_leaves(robot, self.get_joints() + (self.jointnames_dependent if include_dependent and self.jointnames_dependent is not None else []))

    def reduce_to_match(self, joints):
        joints = [str(j) for j in joints]
        if self.jointnames is not None:
            self.jointnames = [str(j) for j in self.jointnames if str(j) in joints]
        self.jointnames_spanningtree = [str(j) for j in self.jointnames_spanningtree if str(j) in joints]
        self.jointnames_dependent = [str(j) for j in self.jointnames_dependent if str(j) in joints]
