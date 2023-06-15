import numpy as np


def skip_upwards_over_fixed(robot, link_name, only_single_parents=True):
    """
    From the given link upwards gets the rootest joint while skipping any fixed joint that is not yet in a submechanism
    """
    j = robot.get_joint(robot.get_parent(link_name))
    if j is not None and j.joint_type == 'fixed' and\
            not any([str(j) in [str(sj) for sj in sm.get_joints()]
                     for sm in robot.submechanisms])\
            and (not only_single_parents or len(robot.get_children(j.parent)) == 1):
        return skip_upwards_over_fixed(robot, j.parent, only_single_parents)
    elif j is not None and j.joint_type == 'fixed' and\
            not any([str(j) in [str(sj) for sj in sm.get_joints()]
                     for sm in robot.submechanisms])\
            and (only_single_parents and len(robot.get_children(j.parent)) > 1):
        return j.parent
    else:
        return link_name


def skip_downwards_over_fixed(robot, link_name, submechanism):
    """
    Starting from the given joint names returns the end of all branches that are fixed and not already present in any
    submechanism
    """
    out = set()
    children = robot.get_joint(robot.get_children(link_name))
    if len(children) == 0:
        return [link_name]
    all_children_in_submech = all([str(child) in [str(sj) for sj in submechanism.get_joints()] for child in children])
    for j in children:
        if (j.joint_type == 'fixed' and not any([str(j) in [str(sj) for sj in sm.get_joints()] for sm in robot.submechanisms])) or all_children_in_submech:
            out = out.union(skip_downwards_over_fixed(robot, j.child, submechanism))
        else:
            out.add(link_name)
    return list(out)


def find_common_root(input_model, input_spanningtree):
    """
    Finds the closest common root of the given input spanning tree
    :param input_model:
    :param input_spanningtree:
    :return:
    """
    intersection = set([ln.name for ln in input_model.links])
    chains = []
    for jointname in input_spanningtree:
        joint = input_model.get_joint(jointname, verbose=True)
        assert joint is not None
        chain = input_model.get_chain(input_model.get_root(), joint.parent, joints=False)
        chains += [chain]
        intersection = intersection & set(chain)
    intersection = sorted(list(intersection), key=lambda x: input_model.get_link_level(x))
    return intersection[-1]


def find_leaves(input_model, input_spanningtree):
    """
    Finds the leaves in the given spanning tree
    :param input_model:
    :param input_spanningtree:
    :return:
    """
    leaves = []
    for jointname in input_spanningtree:
        joint = input_model.get_joint(jointname)
        children = input_model.get_children(joint.child)
        if not any([x in input_spanningtree for x in children]):
            leaves += [joint.child]
    return leaves


def find_close_ancestor_links(robot, linkname):
    """
    Returns list of ancestors for the given linkname, that are only a rotational transformation apart.
    This is used to exclude collision checking for those links.
    """
    assert type(linkname) is str
    only_rot_tree = []
    next_children = []

    def go_tree_down(pjoint):
        tree = []
        far_children = []
        if pjoint is not None and np.allclose(pjoint.origin.xyz, [0, 0, 0], atol=1e-2):
            tree.append(pjoint.child)
            jointnames = robot.get_children(pjoint.child)
            for j_name in jointnames:
                pjoint = robot.get_joint(j_name, verbose=True)
                assert pjoint is not None
                res, fc = go_tree_down(pjoint)
                tree += res
                far_children += fc
        elif pjoint is not None:
            far_children += [pjoint.child]
        return tree, far_children

    jointname = robot.get_parent(linkname)
    if jointname is None:
        # we're dealing with the root link
        next_parent = None
        only_rot_tree += [linkname]
        for child in robot.get_children(linkname):
            result, nc = go_tree_down(robot.get_joint(child))
            only_rot_tree += result
            next_children += nc
    else:
        joint = robot.get_joint(jointname)
        # Go up through the tree
        prev_joint = None
        while np.allclose(joint.origin.xyz, [0, 0, 0], atol=1e-2):
            only_rot_tree.append(joint.parent)
            jointname = robot.get_parent(joint.parent)
            if jointname is None:
                # we have reached the root of the robot
                break
            prev_joint = joint
            joint = robot.get_joint(jointname)
        next_parent = joint.parent

        # as we have found the root of this only-rotational subtree we can go it down, too to find children and siblings
        if len(only_rot_tree) == 0:
            # we are at the root of the only rot. subtree
            if np.allclose(joint.origin.xyz, [0, 0, 0], atol=1e-2):
                only_rot_tree += [joint.parent]
            only_rot_tree += [joint.child]
            for child in robot.get_children(joint.child):
                result, nc = go_tree_down(robot.get_joint(child))
                only_rot_tree += result
                next_children += nc
        else:
            result, nc = go_tree_down(prev_joint)
            only_rot_tree += result
            next_children += nc

    return list(set(only_rot_tree)), next_parent, list(set(next_children))


def get_joints_depth_first(robot, start_link, independent_joints=None):
    joints = []
    start_link = robot.get_link(str(start_link))
    assert start_link is not None
    children = sorted(robot.get_children(str(start_link)), key=lambda x: str(x))
    if independent_joints is not None:
        indep_children = sorted([child for child in children if child in independent_joints], key=lambda x: str(x))
        other_children = sorted([child for child in children if child not in indep_children], key=lambda x: str(x))
        assert len(set(indep_children) & set(other_children)) == 0
        assert len(children) == len(indep_children) + len(other_children)
        children = indep_children + other_children
    for child in children:
        joint = robot.get_joint(str(child), verbose=True)
        assert joint is not None
        joints += [joint]
        try:
            joints += get_joints_depth_first(robot, str(joint.child), independent_joints=independent_joints)
        except Exception as e:
            print(joint.to_urdf_string(), joint.child, joint._child)
            raise e
    return joints


def get_joints(robot, joint_desc):
    """
    Provides a list of jointnames from the given robot specified by the given joint descriptor
    Args:
        robot: the robot instance
        joint_desc: joint descriptor: ALL, INDEPENDENT, ACTIVE, list of jointnames

    Returns:
        list of jointnames
    """
    if joint_desc is None:
        joint_desc = "ALL"
    robot_joint_names = [jnt.name for jnt in robot.joints]
    if type(joint_desc) == list:
        return list(set(joint_desc))
    elif type(joint_desc) == str:
        if robot.submechanisms is not None and len(robot.submechanisms) > 0 and joint_desc.upper() != "ALL":
            if joint_desc.upper() == "INDEPENDENT":
                joints = set()
                for sm in robot.submechanisms:
                    joints = joints.union([joint for joint in sm.jointnames_independent if joint in robot_joint_names])
                return list(joints)
            elif joint_desc.upper() == "ACTIVE":
                joints = set()
                for sm in robot.submechanisms:
                    joints = joints.union([joint for joint in sm.jointnames_active if joint in robot_joint_names])
                return list(joints)
            elif joint_desc.upper() == "ALL":
                return list(set(robot_joint_names))
        elif joint_desc.upper() in ["ALL", "INDEPENDENT", "ACTIVE"]:
            return list(set(robot_joint_names))
    raise Exception(joint_desc + " is no valid joint descriptor!")
