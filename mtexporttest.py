import bpy
import sys
import os

sys.path.insert(0, os.environ['MARS_BLENDER_SCRIPT_PATH'])

import marstools.mtexport as mtexport
import marstools.mtgui as mtgui


mtgui.register()
eo = mtexport.export()
