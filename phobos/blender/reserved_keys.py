JOINT_KEYS = ["name", "axis", "type", "parent", "child",
              "limits/lower", "limits/upper", "limits/effort", "limits/velocity",
              "dynamics/friction", "dynamics/spring_stiffness", "dynamics/damping", "dynamics/spring_reference", "pose"]
LINK_KEYS = ["name", "visuals", "collisions", "inertial", "inertia", "pose"]
VISUAL_KEYS = COLLISION_KEYS = VISCOL_KEYS = ["name", "origin", "material", "geometry", "bitmask", "geometry/type", "link"]
INERTIAL_KEYS = ["inertia", "mass", "origin"]
MOTOR_KEYS = ["name", "type", "joint", "maxSpeed", "maxValue", "maxEffort", "minValue"]
INTERFACE_KEYS = ["name", "type", "direction", "parent", "origin"]
INTERNAL_KEYS = ["phobostype", "phobosmatrixinfo"]
