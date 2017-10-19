#!/usr/bin/python
# coding=utf-8

"""
Copyright 2014-2017, University of Bremen & DFKI GmbH Robotics Innovation Center

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

File model.py

Created on 2 Jun 2017

@author: Kai von Szadkowski, Stefan Rahms, Simon Reichel
"""

import subprocess
import yaml
import bpy
from phobos.phoboslog import log


def cloneGit(name, url, destination):
    # DOCU add some docstring
    try:
        subprocess.check_output(['git', 'clone', url, name], cwd=destination, universal_newlines=True)
        log("Cloned git into " + destination + ".", "INFO")
        return True
    except subprocess.CalledProcessError:
        log("Problem cloning git repository. Destination is either not empty or remote is incorrect.",
            "ERROR")
        return False


def switchToBranch(branch, workingdir):
    # DOCU add some docstring
    if not branch or not workingdir:
        log("No branch specified.", "ERROR")
        return False
    try:
        subprocess.check_output(['git', 'checkout', branch], cwd=workingdir, universal_newlines=True)
        log("Switched to branch " + branch + ".", "INFO")
        return True
    except subprocess.CalledProcessError:
        try:  # checking out remote branch
            subprocess.check_output(['git', 'checkout', '-b', branch, 'origin/' + branch], cwd=workingdir,
                                    universal_newlines=True)
        except subprocess.CalledProcessError:
            log("Could not switch to branch " + branch + ".", "ERROR")
            return False


def checkoutCommit(commit, workingdir):
    # DOCU add some docstring
    if not commit or not workingdir:
        log("No commit specified.", "ERROR")
        return False
    try:
        subprocess.check_output(['git', 'checkout', commit], cwd=workingdir, universal_newlines=True)
        log("Checked out commit " + commit + ".", "INFO")
        return True
    except subprocess.CalledProcessError:
        log("Problem checking out " + commit, "ERROR")
        return False


def getgitbranch():
    """Checks whether working directory (of .blend file) contains a git repository.
    Returns branch if repository is found.
    """
    try:
        output = str(subprocess.check_output(['git', 'branch'], cwd=bpy.path.abspath('//'),
                                             universal_newlines=True))
        branch = [a for a in output.split('\n') if a.find('*') >= 0][0]
        return branch[branch.find('*')+2:]
    except subprocess.CalledProcessError:
        return None
    except FileNotFoundError:
        log("No git repository found.", "ERROR")
        return None


def getGitRemotes(category=''):
    """Returns a dictionary with git remotes of the shape {name: url, ...} if valid
    category is provided, else {'fetch': {name: url, ...}, 'push': {name: url, ...}}.
    """
    try:
        output = str(subprocess.check_output(['git', 'remote', '-v'], cwd=bpy.path.abspath('//'),
                                             universal_newlines=True))
        remotes = {'fetch': {}, 'push': {}}
        for line in [a for a in output.split('\n') if a != '']:
            try:
                linedata = line.split()
                if 'fetch' in linedata[-1]:
                    remotes['fetch'][linedata[0]] = linedata[1]
                else:
                    remotes['push'][linedata[0]] = linedata[1]
            except IndexError:
                log("Git return line does not fit expected output format.", "ERROR")
        log("Found the following remotes: " + yaml.dump(remotes), "DEBUG")
        try:
            return remotes[category]
        except KeyError:
            log("No valid remotes category ('fetch'/'push') provided: " + category, "DEBUG")
            return remotes
    except subprocess.CalledProcessError:
        log("CalledProcessError", "ERROR")
        return None
    except FileNotFoundError:
        log("No git repository found.", "ERROR")
        return None


def getPushRemotesList(self, context):
    remotes = getGitRemotes('push')
    remoteslist = [remotes[a] for a in remotes]
    print(remoteslist)
    return [(url,)*3 for url in remoteslist]


def getFetchRemotesList(self, context):
    remotes = getGitRemotes('fetch')
    remoteslist = [remotes[a] for a in remotes]
    print(remoteslist)
    return [(url,)*3 for url in remoteslist]
