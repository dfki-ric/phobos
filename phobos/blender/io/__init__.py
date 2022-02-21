#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""Contains all kind of input/output modules which are required to import/export different formats.

Basic model formats are in :mod:`phobos.io.entities`, while scene formats are found in
:mod:`phobos.io.scenes`.

The :mod:`phobos.io.meshes` module contains the different mesh formats and
:mod:`phobos.io.libraries` contains the functions for robot/submechanism import and export.

New import/export entities can be defined by just adding a new python file to the respective
subpackage. These are parsed automatically by Phobos and added to the Blender GUI, if they have the
proper ``entity_type_dict``. See one of the existing entities (such as
:mod:`phobos.io.entities.yaml` or :mod:`phobos.io.entities.urdf` for an example).
"""
