# -*- coding: utf-8 -*-

bl_info = {
"name": "Phobos",
"description": "A toolbox to enable editing of robot models in Blender.",
"author": "Kai von Szadkowski, Ole Schwiegert, Stefan Rahms, Malte Langosz, Simon Reichel, Henning Wiedemann",
"version": (2, 0, 0),
"blender": (2, 93, 0),
"location": "Phobos adds a number of custom tool panels.",
"warning": "",
"wiki_url": "https://github.com/dfki-ric/phobos/wiki",
"support": "COMMUNITY",
"tracker_url": "https://github.com/dfki-ric/phobos/issues",
"category": "Development",
}

__BLENDER_ACTIVE__ = False

try:
  import bpy
  __BLENDER_ACTIVE__ = True
except ImportError:
  pass


if not __BLENDER_ACTIVE__:
    pass
else:
    from .blender import *
