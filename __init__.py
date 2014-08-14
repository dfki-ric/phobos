'''
Phobos - a Blender Add-On to work with MARS robot models

File __init__.py

Created on 6 Jan 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
'''


bl_info = {
    "name": "Phobos",
    "description": "A set of tools to enable editing of MARS robot models in Blender.",
    "author": "Kai von Szadkowski, Malte Langosz, Stefan Rahms, Ole Schwiegert",
    "version": (0, 5),
    "blender": (2, 62, 0),
    "location": "",
    "warning": "",
    "wiki_url": "",
    "category": "3D View"
    }

if "bpy" in locals():
    import imp
    imp.reload(update)
    imp.reload(robotdictionary)
    imp.reload(controllers)
    imp.reload(export)
    imp.reload(importer)
    imp.reload(phobosgui)
    imp.reload(joints)
    imp.reload(links)
    imp.reload(misctools)
    imp.reload(sensors)
    imp.reload(utility)
    imp.reload(testing)
    imp.reload(collision)
    imp.reload(inertia)
    imp.reload(marssceneexport)
    print("Reloading Phobos.")
else:
    from . import robotupdate, links, marssceneexport, robotdictionary, controllers, exporter, importer, joints, misctools, sensors, utility, collision, inertia, phobosgui
    print("Importing Phobos modules.")

import bpy
#import sys
#sys.path.append('/usr/lib/python3/dist-packages/') #TODO: make this work on any system!!!
#import yaml

def register():
    links.register()
    controllers.register()
    exporter.register()
    phobosgui.register()
    importer.register()
    joints.register()
    misctools.register()
    sensors.register()
    utility.register()
    collision.register()
    inertia.register()
    bpy.utils.register_module(__name__)

def unregister():
    links.unregister()
    controllers.unregister()
    exporter.unregister()
    phobosgui.unregister()
    importer.unregister()
    joints.unregister()
    misctools.unregister()
    sensors.unregister()
    utility.unregister()
    collision.unregister()
    inertia.register()
    bpy.utils.unregister_module(__name__)

#if __name__ == "__main__":
#    register()
