# TODO shebang

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
