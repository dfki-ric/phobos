#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.logging
    :platform: Unix, Windows, Mac
    :synopsis: TODO: This module offers a simple way to log messages from phobos and uses blender integrated tools
    to display them.

.. moduleauthor:: Ole Schwiegert, Kai von Szadkowski

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

import bpy
from datetime import datetime


# global operator storing the currently active blender operator
operator = None

# levels of detail for logging
loglevels = {"NONE": 0, "ERROR": 1, "WARNING": 2, "INFO": 3, "DEBUG": 4}

class col:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    DEBUG = '\033[35m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    BLINK = '\033[5m'
    DIM = '\033[2m'


def decorate(level):
    if level == "INFO":
        return col.BOLD+col.OKGREEN+level+col.ENDC
    if level == "WARNING":
        return col.BOLD+col.WARNING+level+col.ENDC
    if level == "ERROR":
        return col.BOLD+col.FAIL+level+col.ENDC
    if level == "DEBUG":
        return col.BOLD+col.DEBUG+level+col.ENDC
    else:
        return level


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


def log(message, level="INFO", origin="", prefix=""):
    """Logs a given message to the blender console and logging file if present
    and if log level is low enough.
    :param message: The message to log.
    :param level: Valid log level for the message as defined by 'loglevels'.
    :param origin: If set the message is prefixed with the origin.
    :param prefix: Any string that should be printed before message (e.g. "\n")
    """
    prefs = bpy.context.user_preferences.addons["phobos"].preferences
    if loglevels[level] <= loglevels[prefs.loglevel]:
        date = datetime.now().strftime("%Y%m%d_%H:%M")
        msg = "[" + date + "] " + level + " " + message + " (" + origin + ")"
        terminalmsg = prefix + "[" + date + "] " + decorate(level) + " " + message +\
                      col.DIM + " (" + origin + ")" + col.ENDC
        if prefs.logtofile:
            try:
                with open(prefs.logfile, "a") as lf:
                    lf.write(date + "  " + msg + "\n")
            except IOError:
                log("Cannot write to log file! Resetting it.", "ERROR", __name__+".log")
        # log to terminal or Blender
        if prefs.logtoterminal:
            print(terminalmsg)
        elif operator is not None:
            operator.report({level}, msg)

def register():
    print("Registering " + __name__)


def unregister():
    print("Unregistering " + __name__)
