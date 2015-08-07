#!/usr/bin/python

"""
.. module:: phobos.logging
    :platform: Unix, Windows, Mac
    :synopsis: TODO: This module offers a simple way to log messages from phobos and uses blender integrated tools
    to display them.

.. moduleauthor:: Ole Schwiegert

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

"""

levels = {"ALL": True, "ERROR": True, "WARNING": True, "INFO": True}
operator = None

def startLog(pOperator):
    """This function starts logging for a specified operator.

    :param pOperator: The operator you want to log for.
    :type pOperator: bpy.types.Operator

    """
    global operator
    operator = pOperator

def endLog():
    """This function ends the logging for a former registered operator.

    """
    global operator
    operator = None

def adjustLevel(type, isEnabled):
    """This function adjusts the visibility for a certain logging level.

    :param type: The log level you want to enable or disable.
    :type type: str -- One of ALL, ERROR, WARNING or INFO.
    :param isEnabled: Sets whether logs with the specified level are visible or not.
    :type isEnabled: bool.

    """
    global levels
    if type in levels:
        levels[type] = isEnabled

def log(msg, logType="WARNING"):
    """Logs a given message and a given log level.
    If there is a registered operator its report function is used, else its printed to the console.

    :param msg: The message to log.
    :type msg: str
    :param logType: The log level you want to log the message with.
    :type logType: str -- one of ERROR, WARNING or INFO.

    """
    global levels, operator
    if levels['ALL'] or logType == "INFO":
        if operator != None and (logType in levels) and levels[logType] == True:
            operator.report ({logType}, msg)
        else:
            print ("LOGGER: ", "Log without bound operator: ", msg)