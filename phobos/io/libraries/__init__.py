#!/usr/bin/python3.5
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2018 University of Bremen & DFKI GmbH Robotics Innovation Center

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
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
