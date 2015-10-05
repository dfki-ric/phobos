#!/usr/bin/python

"""
..module:: phobos.defs
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

..moduleauthor:: Kai von Szadowski

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

NOTE: This module is used to provide a number of global definitions which
are used by the other modules. If you make changes to this module,
do not include any imports which are not part of the standard python 3 libray.
This module may well be imported and used outside of the MARS Blender Tools
in the future.
"""

import math, os, yaml, re, bpy
from bpy.types import AddonPreferences
from bpy.props import StringProperty, EnumProperty


# TODO: the following definitions for enum properties in blender should be
# combined with the type definitions further below

version = '0.7'

phobostypes = (('undefined',) * 3,
               ('link',) * 3,
               ('inertial',) * 3,
               ('visual',) * 3,
               ('collision',) * 3,
               ('sensor',) * 3,
               ('controller',) * 3,
               ('approxsphere',) * 3,
               ('light',) * 3)

subtypes = ("visual", "joint", "motor", "collision", "sensor", "link", "inertial", "controller", "light", "approxsphere")

jointtypes = (('revolute',) * 3,
              ('continuous',) * 3,
              ('prismatic',) * 3,
              ('fixed',) * 3,
              ('floating',) * 3,
              ('planar',) * 3)

motortypes = []

logLevels = {"NONE": 0, "ERROR": 1, "WARNING": 2, "INFO": 3}

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
                   "controller_default": ("controller",),
                   "light": ('name', 'light/directional', 'light/exponent'),
                   "light_default": ('new_light', 'true', '1.0')
}

# definition of phobos specific custom properties
reservedProperties ={
    #saves the filename of a shared mesh for visual and collision objects. It lives in the geometry category.
    "SHAREDMESH": "sharedMeshFile"
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
sensortypes = []

sensorProperties = {}

dictConstraints = {}

checkMessages = {"NoObject": []}

def generateCheckMessages(param1, param2):#FIXME: Parameter?
    """This function is just for generating a blender friendly list for an operator.

    """
    return [(x,)*3 for x in list(checkMessages.keys())]

#definitions of which elements live on which layers by default
layerTypes = {
    "link": 0,
    'inertial': 1,
    "visual": 2,
    "collision": 3,
    "sensor": 4,
    "decoration": 5,
    "light": 6,
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

def updateDefs(defsFolderPath):
    """Updates the definitions with all yml files in the given folder

    :param defsFolderPath: The path to the folder with the definitions yaml files.
    :type defsFolderPath: str

    """
    print("Parsing YAML files for updating defs")
    dicts = __parseAllYAML(defsFolderPath)
    for entry in dicts:
        if 'Sensors' in entry:
            for sens in entry['Sensors']:
                if sens not in sensortypes:
                    sensortypes.append(sens)
                    sensorProperties[sens] = entry['Sensors'][sens]
        if 'DictConstraints' in entry:
            for cons in entry['DictConstraints']:
                if cons not in dictConstraints:
                    dictConstraints[cons] = entry['DictConstraints'][cons]
        if 'Motors'in entry:
            for motor in entry['Motors']:
                if (motor,) * 3 not in motortypes:
                    motortypes.append((motor,) * 3)
    # Extending dictConstraints
    dictConstraints['sensors']['$forElem']['$selection__type'] = sensorProperties

def __parseAllYAML(path):
    """This functions reads all .yml files in the given path and loads them.
    It also evaluates the by & enclosed expressions in this file.

    :param path: The path to open all files in.
    :type path: str
    :return: dict -- The dictionary with all parsed YAML files.

    """
    #TODO: Better Exception handling!
    dicts = []
    for root, dirs, files in os.walk(path):
        for file in files:
            print("Parsing "+ file)
            if file.endswith(".yml"):
                try:
                    f = open(path+'/'+file, 'r') #TODO: Better way to handle filepath and avoid double '/'?
                    tmpString = f.read()
                    f.close()
                    try:
                        tmpYAML = yaml.load(__evaluateString(tmpString))
                        dicts.append(tmpYAML)
                    except(yaml.scanner.ScannerError):
                        log("Error while parsing YAML file", "ERROR")
                except(FileNotFoundError):
                    log("The file "+file+" was not found.", "ERROR")
    return dicts

def __evaluateString(s):
    """This functions evaluates a string by searching for mathematical expressions enclosed in &
    and evaluating the inner string as python code.

    :param s: The string to evaluate.
    :type s: str
    :return: str -- the evaluated string.
    """
    p = re.compile('&.*&')
    for ma in p.findall(s):
        try:
            s = s.replace(ma, str(eval(ma[1:-1])))
        except():
            log("The expression " + ma + " could not be evaluated. Ignoring file", "ERROR")
            return ""
    return s

class PhobosPrefs(AddonPreferences):
    bl_idname = __package__

    logFile = StringProperty(
        name = "Log file",
        subtype = "FILE_PATH",
    )

    logLevel = EnumProperty(
        name = "Loglevel",
        items = tuple(((l,)*3 for l in tuple(logLevels.keys()))),
        default = "INFO"
    )

    def draw(self, context):
        layout = self.layout
        layout.label(text="Logging Settings")
        layout.prop(self, "logFile")
        layout.prop(self, "logLevel")

def getPrefs():
    """Returns the addon preferences object for fast access.
    :return: The addon preferences object.
    """
    return bpy.context.user_preferences.addons["phobos"].preferences

def register():
    print("Registering " + __name__)
    bpy.utils.register_class(PhobosPrefs)

def unregister():
    print("Unregistering " + __name__)
    bpy.utils.unregister_class(PhobosPrefs)
