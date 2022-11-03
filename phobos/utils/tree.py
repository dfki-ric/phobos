import numpy as np


def skip_upwards_over_fixed(input_spanningtree, input_model, joint_name):
    """
    In the given jointnames spanningtree of the given input_model it goes upward in the urdf tree while skipping over
    fixed joints. Returns the next movable parent name of the given fixed joint in input_spanningtree.
    """
    j = input_model.get_joint(joint_name)
    if j.joint_type == 'fixed':
        parents = input_model.get_parent(j.parent)
        if parents is not None:
            return skip_upwards_over_fixed(input_spanningtree, input_model, parents)
        else:
            return j.name
    else:
        return j.name


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


def get_all_children(input_model, linkname):
    children = []
    for child in input_model.get_children(linkname):
        children += [child]
        joint = input_model.get_joint(child)
        children += get_all_children(input_model, joint.child)
    return children


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
        children = get_all_children(input_model, joint.child)
        if not any([x in children for x in input_spanningtree]):
            leaves += [joint.child]
    return leaves


def find_close_ancestor_links(robot, linkname):  # Todo find also siblings?
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
                pjoint = robot.get_joint(j_name)
                res, fc = go_tree_down(pjoint)
                tree += res
                far_children += fc
        elif pjoint is not None:
            far_children += [pjoint.child]
        else:
            far_children += robot.get_children(pjoint)
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
        joint = robot.get_joint(str(child))
        assert joint is not None
        joints += [joint]
        try:
            joints += get_joints_depth_first(robot, str(joint.child), independent_joints=independent_joints)
        except Exception as e:
            print(joint.to_urdf_string(), joint.child, joint._child)
            raise e
    return joints

