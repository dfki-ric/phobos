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

import phobos.defs as defs
from datetime import datetime


"""
This is the global operator variable storing the currently active blender operator.
"""
operator = None

def startLog(pOperater):
    """Sets the global operator variable to the given operator
    :param pOperator: The new active operator.
    """
    global operator
    operator = pOperater

def endLog():
    """Sets the global operator variable to None.
    """
    global operator
    operator = None

def log(message, level="INFO", origin=""):
    """Logs a given message to the blender console and logging file if present
    and if log level is low enough.
    :param message: The message to log.
    :param level: The log level for the message. Must be one of the logLevels specified defs
    :param origin: If set the message is prefixed with the origin.
    """
    prefs = defs.getPrefs()
    if defs.logLevels[level] <= defs.logLevels[prefs.logLevel]:
        msg = origin + "::" + message if origin != "" else message
        if operator != None:
            operator.report({level}, msg)
        if prefs.logFile != "":
            try:
                with open(prefs.logFile, "a") as lf:
                    date = datetime.now().strftime("%Y%m%d_%H:%M")
                    lf.write(date + "  " + msg + "\n")
            except IOError:
                prefs.logFile = ""
                log("Cannot write to log file! Resetting it.", "ERROR", __name__+".log")
        print(msg)

def register():
    print("Registering " + __name__)

def unregister():
    print("Unregistering " + __name__)
