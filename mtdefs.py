'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtdefs.py

Created on 7 Jan 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.

NOTE: This module is used to provide a number of global definitions which
are used by the other modules. If you make changes to this module,
do not include any imports which are not part of the standard python 3 libray.
This module may well be imported and used outside of the MARS Blender Tools
in the future.
'''

marstypes = [tuple(['undefined']*3),
             tuple(['body']*3),
             ('joint', 'joint', 'joint'),
             ('motor', 'motor', 'motor'),
             ('sensor', 'sensor', 'sensor'),
             tuple(['controller']*3)]

type_properties = {
    "body": ('name', 'collisionPrimitive', 'collisionBitmask', ),
    "body_default": ('new_node', '65536'),
    "joint": ('name', 'child', 'jointType'),
    "joint_default": ("new_joint", 'some_node', 'hinge'),
    "motor": ('name', 'motor_type'),
    "motor_default": ("new_motor", "1"),
    "sensor": ('name', 'sensorType'),
    "sensor_default": ("new_sensor", "RaySensor"),
    "controller": ('name'),
    "controller_default": ("controller")
    }

type_property_defaults = {
    'node': {'name': 'new_node',
             'collisionBitmask': '65536'}
    }

# definition of node types
nodeTypes = ["undefined",
             "mesh",
             "box",
             "sphere",
             "capsule",
             "cylinder",
             "plane",
             "terrain",
             "reference"
            ]

# definition of joint types
jointTypes = ["undefined",
              "hinge",
              "hinge2",
              "slider",
              "ball",
              "universal",
              "fixed",
              "istruct-spine"
             ]

# definition of sensor types
sensorTypes = ["RaySensor",
               "CameraSensor",
               "ScanningSonar",
               "JointPosition",
               "JointVelocity",
               "JointLoad",
               "JointTorque",
               "JointAVGTorque",
               "Joint6DOF",
               "NodeContact",
               "NodePosition",
               "NodeRotation",
               "NodeContactForce",
               "NodeCOM",
               "NodeVelocity",
               "NodeAngularVelocity",
               "MotorCurrent"
              ]

sensorProperties = {"RaySensor": {"width": 144},
               "CameraSensor": {"width": 640, "height": 480},
               "ScanningSonar": {},
               "JointPosition": {},
               "JointVelocity": {},
               "JointLoad": {},
               "JointTorque": {},
               "JointAVGTorque": {},
               "Joint6DOF": {},
               "NodeContact": {},
               "NodePosition": {},
               "NodeRotation": {},
               "NodeContactForce": {},
               "NodeCOM": {},
               "NodeVelocity": {},
               "NodeAngularVelocity": {},
               "MotorCurrent": {}
              }

#definitions of which elements live on which layers
layerTypes = {
              "nodes": 0,
              "joints": 1,
              "jointSpheres": 2,
              "sensors": 3,
              "decorations": 4,
              "constraints": 5,
              "names": 6
              }
