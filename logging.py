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

File logging.py

Created on 05 Dec 2014

@author: Ole Schwiegert
"""

levels = {"ALL": True, "ERROR": True, "WARNING": True, "INFO": True}
operator = None

def startLog(pOperator):
    global operator
    operator = pOperator

def endLog():
    global operator
    operator = None

def adjustLevel(type, isEnabled):
    global levels
    if type in levels:
        levels[type] = isEnabled

def log(msg, logType="WARNING"):
    global levels, operator
    if levels['ALL'] or logType == "INFO":
        if operator != None and (logType in levels) and levels[logType] == True:
            operator.report ({logType}, msg)
        else:
            print ("LOGGER: ", "Log without bound operator: ", msg)