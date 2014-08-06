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
#TODO: the following definitions for enum properties in blender should be
# combined with the type definitions further below
marstypes = (('undefined',)*3,
             ('link',)*3,
             ('inertial',)*3,
             ('visual',)*3,
             ('collision',)*3,
             ('sensor',)*3,
             ('controller',)*3)

jointtypes = (('revolute',)*3,
              ('continuous',)*3,
              ('prismatic',)*3,
              ('fixed',)*3,
              ('floating',)*3,
              ('planar',)*3)

motortypes = (('servo',)*3,
               ('DC',)*3)

geometrytypes = (('box',)*3,
            ('cylinder',)*3,
            ('sphere',)*3,
            ('capsule',)*3,
            ('mesh',)*3)

type_properties = {
    "undefined": (),
    "undefined_default": (),
    "link": ('name'),
    "link_default": ('new_node'),
    "inertial": ('name'),
    "inertial_default": ('new_node'),
    "visual": ('name', 'visualType'),
    "visual_default": ('new_node', 'box'),
    "collision": ('name', 'collisionType', 'bitmask'),
    "collision_default": ('new_node', 'box', '65536'),
    "sensor": ('name', 'sensorType'),
    "sensor_default": ("new_sensor", "RaySensor"),
    "controller": ('name',),
    "controller_default": ("controller")
    }

# definition of node types
nodeTypes = ("undefined",
             "mesh",
             "box",
             "sphere",
             "capsule",
             "cylinder",
             "plane",
             "terrain",
             "reference"
            )

# definition of joint types
jointTypes = ("undefined",
              "revolute",
              "continuous",
              "prismatic",
              "hinge2",
              "slider",
              "ball",
              "universal",
              "fixed",
              "istruct-spine"
             )

# definition of sensor types
sensorTypes = ("RaySensor",
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
              )

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

#definitions of which elements live on which layers by default
layerTypes = {
              "link": 0,
              'inertial': 1,
              "visual": 2,
              "collision": 3,
              "sensor": 4,
              "decoration": 5
              }

MARSlegacydict = {'specularColor': 'specularFront',
                  'diffuseColor': 'diffuseFront'
                  }
