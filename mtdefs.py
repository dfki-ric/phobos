'''
Created on 7 Jan 2014

@author: kavonszadkowski
'''

marstypes = [tuple(['undefined']*3),
             tuple(['body']*3),
             ('joint', 'joint', 'joint'),
             #('motor', 'motor', 'motor'),
             ('sensor', 'sensor', 'sensor'),
             tuple(['controller']*3)]

type_properties = {
    "body": ('name', 'coll_bitmask'),
    "body_default": ('new_node', '65536'),
    "joint": ('name', 'node2'),
    "joint_default": ("new_joint", 'some_node'),
    "motor": ('name', 'motor_type'),
    "motor_default": ("new_motor", "1"),
    "sensor": ('name', 'sensor_type'),
    "sensor_default": ("new_sensor", "RaySensor")
    }

type_property_defaults = {
    'node': {'name': 'new_node',
             'coll_bitmask': '65536'}
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
