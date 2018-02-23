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
import os
from phobos.phoboslog import log


def isGit(folder):
    """Checks whether the folder contains a git directory."""
    return os.path.exists(os.path.join(folder, '.git'))

def cloneGit(name, url, destination):
    '''Clones the git repository which is specified by its url into the folder
    with the specified name in the destination folder. If the url provides the
    https:// start, it might be necessary to enter gituser and password into
    commandline, so better use the git@git start.
    If the destination folder is a git already, it will not be cloned, but True
    will be returned.
    If the destination folder does not exist, it is created on the fly.
    '''
    # check for existing git first
    try:
        if os.path.exists(os.path.join(destination,name)):
            subprocess.check_output(['git', 'status'], cwd=os.path.join(destination, name))
            return True
    except subprocess.CalledProcessError:
        pass

    # if there is no destination folder, create it
    if not os.path.exists(destination):
        os.makedirs(destination)

    # clone git if not existing already
    try:
        subprocess.check_output(['git', 'clone', url, name], cwd=destination, universal_newlines=True)
        log("Cloned git into " + destination + ".", "INFO")
        return True
    except subprocess.CalledProcessError:
        log("Problem cloning git repository. Destination is either not empty or remote is incorrect.",
            "ERROR")
    return False


def checkoutBranch(branch, workingdir, create=False):
    if not branch or not workingdir:
        log("No branch specified.", "ERROR")
        return False
    try:
        subprocess.check_output(['git', 'checkout', branch], cwd=workingdir, universal_newlines=True)
        log("Checkout branch " + branch + " successful.", "INFO")
        return True
    except subprocess.CalledProcessError:
        if create:
            createNewBranch(branch, workingdir)
        else:
            log("Could not checkout branch " + branch + ".", "ERROR")
            return False


def createNewBranch(branch, workingdir):
    if not branch or not workingdir:
        log("No branch specified.", "ERROR")
        return False
    try:
        subprocess.check_output(['git', 'checkout', '-b', branch], cwd=workingdir, universal_newlines=True)
        log("Created branch " + branch + ".", "INFO")
        return True
    except subprocess.CalledProcessError:
        log("Could not create branch " + branch + ".", "ERROR")
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


def getGitBranch():
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


def getGitRemotes(category='', folder=None):
    """Returns a dictionary with git remotes of the shape {name: url, ...} if valid
    category is provided, else {'fetch': {name: url, ...}, 'push': {name: url, ...}}.
    """
    try:
        if not folder:
            folder = bpy.path.abspath('//')
        print('Checking git folder: ', folder)
        output = str(subprocess.check_output(['git', 'remote', '-v'], cwd=folder,
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


def getPushRemotesList(self, context, folder=None):
    remotes = getGitRemotes('push', folder=folder)
    remoteslist = [remotes[a] for a in remotes]
    print(remoteslist)
    return [(url,)*3 for url in remoteslist]


def getFetchRemotesList(self, context):
    remotes = getGitRemotes('fetch')
    remoteslist = [remotes[a] for a in remotes]
    print(remoteslist)
    return [(url,)*3 for url in remoteslist]
