#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Registers the :mod:`models` and :mod:`mechanisms` submodules to Blender.
"""

from . import models, mechanisms


def register():
    """TODO Missing documentation"""
    models.register()
    mechanisms.register()


def unregister():
    """TODO Missing documentation"""
    models.unregister()
    mechanisms.unregister()
