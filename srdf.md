SRDF
====

The Semantic Robot Description Format (SRDF) is a format complementing URDF in ROS, allowing to define "semantic" information.

## Supported Elements

###group
Groups in SRDF can contain *links*, *joints*, *chains* and other *groups* (the latter two of which have to be specified upstream. As nested groups is just a shortcut for adding links and joints to a group, it is not supported and the user will have to add all links and joints explicitly to each group.
SRDF however implicitly assumes this, so the current implementation only adds the links.

###chain
Chains are elements to simplify defining groups and are supported. The internal robot dictionary also contains a list of all elements belonging to that chain, which is discarded and not written to SRDF, however. It might be written to SMURF in the future.

###link_sphere_approximation
Tese spheres are a way to approximate the collision properties of a link by using a number of spheres, thereby simplifying collision detection for instance for purposes of motion planning. SRDF defines the convention that if no sphere is defined for a link, one large sphere is assumed. If one wants to have no sphere at all, it is necessary to define a sphere of radius 0. As one large sphere can be explicitly added by the user and should be if that is what he intends (WYSIWYG), Phobos adds a sphere of radius 0 by default if no sphere is specified.

###passive_joint
Marks a joint as passive. Simply put the custom property `joint/passive` in a bone to mark the joint as passive.

###disable_collisions
Disables collisions between pairs of links to simplify collision checking and avoid collisions of parents and children at their joints. Parents and children are added by default, the rest of the pairs are computed from the [collision bitmasks](collisions.md) of the objects.


##Currently not supported
    - <group_state>
    - <virtual_joint>

