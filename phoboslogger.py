#!/usr/bin/python

"""
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

File phoboslogger.py

Created on 05 Dec 2014

@author: Ole Schwiegert
"""


class PhobosLogger(object):

    def __init__(self):
        self.__operator = None
        self.__enabled = True

    def startLog(self, operator):
        self.__operator = operator

    def endLog(self):
        self.__operator = None

    def allowLog(self, isEnabled):
        self.__enabled = isEnabled

    def log(self, msg, logType="WARNING"):
        if self.__enabled:
            if self.__operator != None:
                self.__operator.report ({logType}, msg)
            else:
                print ("LOGGER: ", "Log without bound operator: ", msg)

logger = PhobosLogger()