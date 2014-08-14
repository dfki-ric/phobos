import bpy
import sys
import os

sys.path.insert(0, os.environ['MARS_BLENDER_SCRIPT_PATH'])

import phobos.exporter as exporter
import phobos.gui as gui


mtgui.register()
eo = mtexport.export()
