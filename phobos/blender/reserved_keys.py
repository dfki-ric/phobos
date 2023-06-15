JOINT_KEYS = ["name", "axis", "type", "parent", "child",
              "limits/lower", "limits/upper", "limits/effort", "limits/velocity",
              "dynamics/friction", "dynamics/spring_stiffness", "dynamics/damping", "dynamics/spring_reference", "pose", "joint_relative_origin", "origin"]
LINK_KEYS = ["name", "visuals", "collisions", "inertial", "inertia", "pose", "joint_relative_origin", "origin"]
VISUAL_KEYS = COLLISION_KEYS = VISCOL_KEYS = ["name", "origin", "material", "geometry", "bitmask", "geometry/type", "link", "joint_relative_origin"]
INERTIAL_KEYS = ["inertia", "mass", "origin", "joint_relative_origin"]
MOTOR_KEYS = ["name", "type", "joint", "maxSpeed", "maxValue", "maxEffort", "minValue"]
INTERFACE_KEYS = ["name", "type", "direction", "parent", "origin"]
INTERNAL_KEYS = ["phobostype", "phobosmatrixinfo"]
ANNOTATION_KEYS = ["$name", "$category", "$include_parent", "$include_transform", "$transform"]
SUBMECHANISM_KEYS = ["type", "jointnames_spanningtree", "jointnames_active", "jointnames_independent",
                     "jointnames"]
