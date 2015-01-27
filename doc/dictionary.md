## Phobos' Robot Representation

To facilitate simple im- and export of robot models in various formats, Phobos uses a file-format-independent Python dictionary representation to store robot data. This dictionary representation is loosely based on URDF/SDF, with some added MARS' and 'Blender's own naming conventions. It is layed out in the following in YAML-notation:

    model
    - {joints}
        - {joint_1}
            - [axis]: (d, d, d)
            - child: str
            - {limits}:
                - lower: d
                - upper: d
                - velocity: d
                - effort: d
            - parent: str
            - type: str ("hinge", "continuous", "linear", "prismatic", "revolute")
            
        - {joint_2}
            - ...
        - ...
    - {links}
        - {link_1}
            - filename: str
            - [pose]: (d, d, d, d, d, d, d)
            - {visual}:
                - {visual_1}
                    - [pose]: (d, d, d, d, d, d, d) #x, y, z, w, x, y, z
                    - {material}:
                        - name: str
                        - [diffuseColor]: (d, d, d, d)
                        - [ambientColor]: (d, d, d, d)
                        - [emissionColor]: (d, d, d, d)
                        - [specularColor]: (d, d, d, d)
                        - transparency: d
                    - {geometry}:
                        - type: str ("box" | "sphere" | "cylinder" | "plane" | "mesh")
                        - radius: d #sphere
                        - [size]: (d, d, d) #box
                        - radius, height: d, d #cylinder
                        - [size]: (d, d, d) #mesh
                        - [size]: (d, d) #plane
                        - filename: str #mesh
                - {visual_2}
                    - ...
                - ...
            - {collision}
                - {collision_1}
                    - bitmask: int
                    - {geometry}:
                        - type: str ("box" | "sphere" | "cylinder" | "plane" | "mesh")
                        - radius: d #sphere
                        - [size]: (d, d, d) #box
                        - radius, height: d, d #cylinder
                        - [size]: (d, d, d) #mesh
                        - [size]: (d, d) #plane
                        - filename #mesh
                    - [pose]: (d, d, d, d, d, d, d)
                    - max_contacts: int
                - {collision_2}
                    - ...
                - ...
            - {inertial}:
                - mass: d
                - [inertia]: (d, d, d, d, d, d) #ixx, ixy, ixz, iyx, iyy, iyz
        - {link_2}
            - ...
        - ...
    - ["sensor"]
        - link: str
        - type: str
    - ["motor"]
    - ["controller"]
    - {group}:
        - [group1]: (str, ...) #names of links
        - ...
    - {poses}:
    


int: integer
d: double
str: string
