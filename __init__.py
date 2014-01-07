'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File __init__.py

Created on 6 Jan 2014

@author: kavonszadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
'''


bl_info = {
    "name": "MARS Blender Tools",
    "description": "A set of marstools to simplify editing of MARS robot models in Blender.",
    "author": "Kai von Szadkowski",
    "version": (0, 1),
    "blender": (2, 62, 0),
    "location": "",
    "warning": "", # used for warning icon and text in addons panel
    "wiki_url": "",
    "category": "3D View"
    }

if "bpy" in locals():
    import imp
    imp.reload(mtgui)
    imp.reload(mtmisctools)
    print("Reloaded MARS Tools.")
else:
    from . import mtgui, mtmisctools
    print("Imported MARS Tools modules.")

import bpy

def register():
    #mtgui.register()
    #mtmisctools.register()
    bpy.utils.register_module(__name__)

def unregister():
    #marstools.mtgui.unregister()
    #marstools.mtmisctools.unregister()
    bpy.utils.unregister_module(__name__)

if __name__ == "__main__":
    register()
