#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.phoboslog
    :platform: Unix, Windows, Mac
    :synopsis: Offers a simple way to log messages from Phobos and uses Blender integrated tools \
            to display them.

.. moduleauthor:: Ole Schwiegert, Kai von Szadkowski, Simon Reichel

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

File phoboslog.py

Created on 05 Dec 2014

"""

import inspect
from datetime import datetime
from enum import Enum
from types import SimpleNamespace

import bpy

import phobos.display as display

#: Levels of detail for the logging information.
LOGLEVELS = ('NONE', 'ERROR', 'WARNING', 'INFO', 'DEBUG')

#: Calling functions that will never be logged to the GUI of Blender.
FUNCTION_BLACKLIST = ('register')


class Col(Enum):
    """Provides the color ids for different terminal messages."""
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
    """Simple wrapper to color the log level according to the colors from :class:`.Col`.

    If there is no decorator for this level, an undecorated string will be returned.

    :param level:: the loging level as described by :data:`LOGLEVELS`.
    :type level: str

    :return: decorated string of the specific level
    :rtype: str
    """
    if level == "INFO":
        return Col.BOLD.value + Col.OKGREEN.value + level + Col.ENDC.value
    if level == "WARNING":
        return Col.BOLD.value + Col.WARNING.value + level + Col.ENDC.value
    if level == "ERROR":
        return Col.BOLD.value + Col.FAIL.value + level + Col.ENDC.value
    if level == "DEBUG":
        return Col.BOLD.value + Col.DEBUG.value + level + Col.ENDC.value
    return level


def log(message, level="INFO", prefix="", guionly=False, end='\n'):
    """Logs a given message to the blender console/logging file and if log level is low enough.

    The origin can be defined as string or an object. The message is logged by the operator
    depending on the loglevel settings.

    :param message: message to log
    :type message: str
    :param level: valid log level for the message as defined by :data:`.LOGLEVELS`
    :type level: str
    :param prefix: any string that should be printed before the message
    :type prefix: str
    :param guionly: if True, only prints to GUI
    :type guionly: bool
    :param end: string to be used at the end of the resulting print statement
    :type end: str
    """
    frame = inspect.stack()[1][0]
    info = inspect.getframeinfo(frame)
    originname = '{0} - {1} (l{2})'.format(info.filename.split('addons/')[-1], info.function,
                                           info.lineno)

    # display only messages up to preferred log level
    prefs = bpy.context.user_preferences.addons["phobos"].preferences

    # Phobos preferences might not be initialised yet! Use a dummy namespace instead.
    if not prefs:
        prefs = SimpleNamespace()
        prefs.loglevel = 'DEBUG'
        prefs.logtofile = False
        prefs.logtoterminal = True

    if LOGLEVELS.index(level) > LOGLEVELS.index(prefs.loglevel):
        return

    date = datetime.now().strftime("%Y%m%d_%H:%M:%S")
    # end of line will add the date and level information before the message
    if end == '\n' or end == '\n\n':
        msg = date + " - " + level + " " + message + " (" + originname + ")"
        terminalmsg = '{0}[{1}] {2} {3}{4} ({5}){6}'.format(
            prefix, date, decorate(level), message, Col.DIM.value, originname, Col.ENDC.value)
    else:
        msg = message
        terminalmsg = Col.OKBLUE.value + message + Col.ENDC.value

    # log to file if activated
    if prefs.logtofile and not guionly:
        try:
            with open(prefs.logfile, "a") as logfile:
                logfile.write(msg + end)
        except (FileNotFoundError, IsADirectoryError):
            log("Invalid log file path, cannot write to log file!", 'ERROR', guionly=True)
        except (IOError, OSError):
            log("Cannot write to log file!", 'ERROR', guionly=True)

    # log to terminal or Blender
    if prefs.logtoterminal and not guionly:
        print(terminalmsg, end=end)
    # log in GUI depending on loglevel
    else:
        origin = find_calling_operator(inspect.currentframe())

        # show message in Blender status bar.
        if origin:
            # format report message to remove loging level and originname
            msg = msg.split(level)[1][1:]
            msg = msg.split(originname)[0][:-2]
            origin.report({level}, msg)
    # push message to the Phobos message history
    display.push_message(message, level.lower())


def find_calling_operator(frame):
    """Finds the calling operator of a log call from the specified frame.

    If one intermediary function name is in the :data:`FUNCTION_BLACKLIST`, the search is
    interrupted.

    If nothing is found or the search is interrupted, None will be returned.

    :param frame: call frame to begin search from
    :type frame: frame

    :return: execute function of the calling operator or None
    :rtype: function
    """
    # check for next frame up the stack
    while frame.f_back:
        frame = frame.f_back

        # if we find the execute function of an operator we return the function object
        if frame.f_code.co_name == 'execute':
            if 'self' in frame.f_locals:
                return frame.f_locals['self']

        # finding a function in the blacklist needs to interupt the search
        elif frame.f_code.co_name in FUNCTION_BLACKLIST:
            break
    else:
        return None
