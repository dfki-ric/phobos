#!/usr/bin/python

"""
Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

This file is part of Phobos, a Blender Add-On to edit robot models.

Phobos is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License
as published by the Free Software Foundation, either version 3
of the License, or (at your option) any later version.

Phobos is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Phobos.  If not, see <http://www.gnu.org/licenses/>.

File defs.py

Created on 7 Jan 2014

@author: Kai von Szadkowski

NOTE: This module is used to provide a number of global definitions which
are used by the other modules. If you make changes to this module,
do not include any imports which are not part of the standard python 3 libray.
This module may well be imported and used outside of the MARS Blender Tools
in the future.
"""

import math

# TODO: the following definitions for enum properties in blender should be
# combined with the type definitions further below

version = '0.5'

phobostypes = (('undefined',) * 3,
               ('link',) * 3,
               ('inertial',) * 3,
               ('visual',) * 3,
               ('collision',) * 3,
               ('sensor',) * 3,
               ('controller',) * 3,
               ('approxsphere',) * 3)

jointtypes = (('revolute',) * 3,
              ('continuous',) * 3,
              ('prismatic',) * 3,
              ('fixed',) * 3,
              ('floating',) * 3,
              ('planar',) * 3)

motortypes = (('PID',) * 3,
              ('DC',) * 3)

geometrytypes = (('box',) * 3,
                 ('cylinder',) * 3,
                 ('sphere',) * 3,
                 ('capsule',) * 3,
                 ('mesh',) * 3)

type_properties = {"undefined": (),
                   "undefined_default": (),
                   "link": ('name',),
                   "link_default": ('new_node',),
                   "inertial": ('name',),
                   "inertial_default": ('new_node',),
                   "visual": ('name', 'visual/type'),
                   "visual_default": ('new_node', 'box'),
                   "collision": ('name', 'collision/type', 'bitmask'),
                   "collision_default": ('new_node', 'box', '65536'),
                   "approxsphere": (),
                   "approxsphere_default": (),
                   "sensor": ('name', 'sensor/type'),
                   "sensor_default": ("new_sensor", "RaySensor"),
                   "controller": ('name',),
                   "controller_default": ("controller",)
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
# jointTypes = ("undefined",
#              "revolute",
#              "continuous",
#              "prismatic",
#              "hinge2",
#              "slider",
#              "ball",
#              "universal",
#              "fixed",
#              "istruct-spine"
#             )

# definition of sensor types
sensortypes = ("RaySensor",
               "RotatingRaySensor",
               "MultiLevelLaserRangeFinder",
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
               "MotorCurrent",
               "Custom"
               )

sensorProperties = {"RaySensor": {
                        'width': 144,
                        'height': 1,
                        'opening_width': 0.5*math.pi,
                        'opening_height': 0.5*math.pi,
                        'maxDistance': 100.0,
                        'draw_rays': True
                    },
                    "RotatingRaySensor": {
                        'bands': 16,
                        'lasers': 32,
                        'maxDistance': 100.0,
                        'draw_rays': True,
                        'horizontal_resolution': (1.0/180.0)*math.pi,
                        'horizontal_offset':  0.0,
                        'vertical_offset':  0.0
                    },
                    "CameraSensor": {
                        'width': 640,
                        'height': 480,
                        'show_cam': True,
                        'opening_width': 90,
                        'opening_height': 90,
                        'hud_pos': 0,
                        'hud_width': 320,
                        'hud_height': 240,
                        'depthImage': False
                    },
                    "ScanningSonar": {
                        'width': 64,
                        'height': 512,
                        'resolution': 0.1,
                        'maxDist': 100.0,
                        'hud_pos': 0,
                        'updateRate': 10,
                        'gain': 1,
                        'show_cam': False,
                        'only_ray': False,
                        'extension': (0.010000, 0.004000, 0.004000),
                        'left_limit': math.pi,
                        'right_limit': -math.pi,
                        'ping_pong_mode': False,
                    },
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
                    "MotorCurrent": {},
                    "MultiLevelLaserRangeFinder": {
                        'numRaysVertical': 32,
                        'numRaysHorizontal': 1900,
                        'rttResolutionX':  128 * 4,
                        'rttResolutionY':  128 * 2,
                        'verticalOpeningAngle': 40 / 180.0 * math.pi,
                        'horizontalOpeningAngle': 2 * math.pi * 1899 / 1900,
                        'maxDistance':  100.0
                    }
}

#definitions of which elements live on which layers by default
layerTypes = {
    "link": 0,
    'inertial': 1,
    "visual": 2,
    "collision": 3,
    "sensor": 4,
    "decoration": 5,
    "approxsphere": 13
}

#default materials used: Phobos
defaultmaterials = {
    'phobos_joint': {'diffuse': (0, 0, 1), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
    'phobos_name': {'diffuse': (1, 1, 1), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
    'phobos_laserscanner': {'diffuse': (1.0, 0.08, 0.08), 'specular': (1, 1, 1), 'alpha': 0.3,
                            'diffuse_intensity': 1.0},
    #'phobos_camera': {'diffuse': (0.13, 0.4, 1), 'specular': (1, 1, 1), 'alpha': 0.3, 'diffuse_intensity': 0.8},
    'phobos_tof-camera': {'diffuse': (0.44, 1, 0.735), 'specular': (1, 1, 1), 'alpha': 0.3, 'diffuse_intensity': 0.7},
    'phobos_inertial': {'diffuse': (1, 0.18, 0), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
    'phobos_sensor': {'diffuse': (0.8, 0.75, 0), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
    'phobos_controller': {'diffuse': (0.518, 0.364, 0.8), 'specular': (1, 1, 1), 'alpha': 0.3,
                          'diffuse_intensity': 0.7},
    'phobos_indicator1': {'diffuse': (1, 0, 0), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
    'phobos_indicator2': {'diffuse': (0, 1, 0), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
    'phobos_indicator3': {'diffuse': (0, 0, 1), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
    'phobos_indicator4': {'diffuse': (1, 0, 1), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
    'phobos_indicator5': {'diffuse': (1, 1, 0), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
    'phobos_indicator6': {'diffuse': (0, 1, 1), 'specular': (1, 1, 1), 'alpha': 1.0, 'diffuse_intensity': 1.0},
}

MARSlegacydict = {'specularColor': 'specularFront',
                  'diffuseColor': 'diffuseFront'
}
