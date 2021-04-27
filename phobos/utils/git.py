#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import subprocess
import json
import bpy
import os
from phobos.phoboslog import log


def isGit(folder):
    """Checks whether the folder contains a git directory.

    Args:
      folder: 

    Returns:

    """
    return os.path.exists(os.path.join(folder, '.git'))


def cloneGit(name, url, destination):
    """Clones the git repository which is specified by its url into the folder
    with the specified name in the destination folder. If the url provides the
    https:// start, it might be necessary to enter gituser and password into
    commandline, so better use the git@git start.
    If the destination folder is a git already, it will not be cloned, but True
    will be returned.
    If the destination folder does not exist, it is created on the fly.

    Args:
      name: 
      url: 
      destination: 

    Returns:

    """
    # check for existing git first
    try:
        if isGit(os.path.join(destination, name)):
            if not os.path.exists(os.path.join(destination, name, '.git', 'config')):
                log('Git folder found, but the config seems corrupted. Cloning aborted.', 'ERROR')
                return False
            # compare the git url with the existing git folder
            with open(os.path.join(destination, name, '.git', 'config'), 'r') as gitconfigfile:
                if url in gitconfigfile.read():
                    log('Git already cloned.', 'INFO')
                    return True
                log('Git folder found, but with different url. Cloning aborted.', 'ERROR')
                return False
    except subprocess.CalledProcessError:
        pass

    # if there is no destination folder, create it
    if not os.path.exists(destination):
        os.makedirs(destination)

    # clone git if not existing already
    try:
        subprocess.check_output(
            ['git', 'clone', url, name], cwd=destination, universal_newlines=True
        )
        log("Cloned git into " + destination + ".", "INFO")
        return True
    except subprocess.CalledProcessError:
        log(
            "Problem cloning git repository. Destination is either not empty or remote is incorrect.",
            "ERROR",
        )
    return False


def initGit(destination, filename=None, initialsave=False, url=None, readmetxt=''):
    """Creates a new Phobos git in the specified destination folder.
    This also creates the required folder structure.
    Optionally, the git can be pushed to the specified url (as origin) and an
    initial commit can be created.

    Args:
      destination: the destination folder on the filesystem where to
    initialise the git repository
      filename: the filename of the blender file to save
    (only useful in combination with initialsave) (Default value = None)
      initialsave: True if the blendfile shall be saved for the first
    commit already, False if not (Default value = False)
      url: the url of the origin repository (it needs to be empty!) (Default value = None)
      readmetxt: a string which will be written to the Readme.md (Default value = '')

    Returns:
      : True if the initialisation was successful, False if not

    """
    if isGit(destination):
        log('Could not initialise git: Folder already existing!', 'ERROR')
        return False
    elif os.path.exists(destination) and os.listdir(destination):
        log('Could not initialise git: Folder is not empty!', 'ERROR')
        return False
    else:
        os.makedirs(destination)

    # Initialise git locally and make first push for new repositories
    log('Initialising the new git...', 'INFO')
    # TODO add fully featured checks for the requirements and exceptions
    try:
        subprocess.check_output(['git', 'init'], cwd=destination, universal_newlines=True)
        if readmetxt != '':
            readmetxt = '# Initial commit'
        with open(os.path.join(destination, 'Readme.md'), 'w') as readme:
            readme.write(readmetxt)
        with open(os.path.join(destination, '.gitignore'), 'w') as gitignore:
            gitignore.write('*.blend1\n')

        # Add readme and gitignore
        subprocess.check_output(
            ['git', 'add', 'Readme.md'], cwd=destination, universal_newlines=True
        )
        subprocess.check_output(
            ['git', 'add', '.gitignore'], cwd=destination, universal_newlines=True
        )
        subprocess.check_output(
            ['git', 'commit', '-m', '"Initial commit"'], cwd=destination, universal_newlines=True
        )

        # TODO what if the origin repository is not empty?
        subprocess.check_output(
            ['git', 'remote', 'add', 'origin', url], cwd=destination, universal_newlines=True
        )
        subprocess.check_output(
            ['git', 'push', '-u', 'origin', 'master'], cwd=destination, universal_newlines=True
        )
    except subprocess.CalledProcessError as error:
        log('Could not initialise git folder: ' + str(error), 'ERROR')
        return False

    makeGitFolders(destination)

    # saving the initial folder state
    if initialsave and filename:
        bpy.ops.wm.save_as_mainfile(
            filepath=os.path.join(destination, 'blender', filename + '.blend')
        )
        if not commit(destination):
            return False
    log('Git initialised successfully.', 'DEBUG')

    return True


def commit(destination, message='Automated commit', ignore=[]):
    """Commits the current status in the git folder at destination.
    This can ignore the first level subfolders and files specified in the
    optional parameter.

    Args:
      destination: 
      message: (Default value = 'Automated commit')
      ignore: (Default value = [])

    Returns:
      : TODO

    """
    # Add the subfolders and files to git
    # TODO test this functionality
    for direc in os.listdir(destination):
        if direc not in ignore:
            try:
                subprocess.check_output(
                    ['git', 'add', direc], cwd=destination, universal_newlines=True
                )
            except subprocess.CalledProcessError:
                log('Could not add to git: ' + direc, 'ERROR')
                return False

    # Commit the changes
    try:
        subprocess.check_output(
            ['git', 'commit', '-m', '"{0}"'.format(message)],
            cwd=destination,
            universal_newlines=True,
        )
        subprocess.check_output(['git', 'push', 'origin'], cwd=destination, universal_newlines=True)
        log('Commit to ' + destination + ' successful.', 'DEBUG')
        return True
    except subprocess.CalledProcessError as error:
        log('Could not commit and push: ' + str(error), 'ERROR')
        return False


def makeGitFolders(destination):
    """Create the Phobos folder structure for a git at the destination.

    Args:
      destination: TODO

    Returns:
      : TODO

    """
    from os.path import exists, join

    if not exists(join(destination, 'blender')):
        os.makedirs(join(destination, 'blender'))

    if not exists(join(destination, 'urdf')):
        os.makedirs(join(destination, 'urdf'))

    if not exists(join(destination, 'smurf')):
        os.makedirs(join(destination, 'smurf'))

    if not exists(join(destination, 'meshes')):
        os.makedirs(join(destination, 'meshes'))


def checkoutBranch(branch, workingdir, create=False, pushorigin=False):
    """

    Args:
      branch: 
      workingdir: 
      create: (Default value = False)
      pushorigin: (Default value = False)

    Returns:

    """
    if not branch or not workingdir:
        log("No branch specified.", "ERROR")
        return False
    try:
        subprocess.check_output(['git', 'fetch'], cwd=workingdir, universal_newlines=True)
        subprocess.check_output(
            ['git', 'checkout', branch], cwd=workingdir, universal_newlines=True
        )
        log("Checkout branch " + branch + " successful.", "INFO")
        return True
    except subprocess.CalledProcessError:
        if create:
            return createNewBranch(branch, workingdir, pushorigin)
        log("Could not checkout branch " + branch + ".", "ERROR")
        return False


def createNewBranch(branch, workingdir, pushorigin=False):
    """

    Args:
      branch: 
      workingdir: 
      pushorigin: (Default value = False)

    Returns:

    """
    if not branch or not workingdir:
        log("No branch specified.", "ERROR")
        return False
    try:
        subprocess.check_output(
            ['git', 'checkout', '-b', branch], cwd=workingdir, universal_newlines=True
        )
        log("Created branch " + branch + ".", "INFO")
        if pushorigin:
            subprocess.check_output(
                ['git', 'push', '-u', 'origin', branch], cwd=workingdir, universal_newlines=True
            )
        return True
    except subprocess.CalledProcessError as e:
        log("Could not create branch " + branch + ": " + str(e), "ERROR")
        return False


def checkoutCommit(commit, workingdir):
    """

    Args:
      commit: 
      workingdir: 

    Returns:

    """
    # DOCU add some docstring
    if not commit or not workingdir:
        log("No commit specified.", "ERROR")
        return False
    try:
        subprocess.check_output(
            ['git', 'checkout', commit], cwd=workingdir, universal_newlines=True
        )
        log("Checked out commit " + commit + ".", "INFO")
        return True
    except subprocess.CalledProcessError:
        log("Problem checking out " + commit, "ERROR")
        return False


def getGitBranch():
    """Checks whether working directory (of .blend file) contains a git repository.
    Returns branch if repository is found.

    Args:

    Returns:

    """
    try:
        output = str(
            subprocess.check_output(
                ['git', 'branch'], cwd=bpy.path.abspath('//'), universal_newlines=True
            )
        )
        branch = [a for a in output.split('\n') if a.find('*') >= 0][0]
        return branch[branch.find('*') + 2 :]
    except subprocess.CalledProcessError:
        return None
    except FileNotFoundError:
        log("No git repository found.", "ERROR")
        return None


def getGitRemotes(category='', folder=None):
    """Returns a dictionary with git remotes of the shape {name: url, ...} if valid
    category is provided, else {'fetch': {name: url, ...}, 'push': {name: url, ...}}.

    Args:
      category: (Default value = '')
      folder: (Default value = None)

    Returns:

    """
    try:
        if not folder:
            folder = bpy.path.abspath('//')
        print('Checking git folder: ', folder)
        output = str(
            subprocess.check_output(['git', 'remote', '-v'], cwd=folder, universal_newlines=True)
        )
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
        log("Found the following remotes: " + json.dump(remotes), "DEBUG")
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
    """

    Args:
      context: 
      folder: (Default value = None)

    Returns:

    """
    remotes = getGitRemotes('push', folder=folder)
    remoteslist = [remotes[a] for a in remotes]
    print(remoteslist)
    return [(url,) * 3 for url in remoteslist]


def getFetchRemotesList(self, context):
    """

    Args:
      context: 

    Returns:

    """
    remotes = getGitRemotes('fetch')
    remoteslist = [remotes[a] for a in remotes]
    print(remoteslist)
    return [(url,) * 3 for url in remoteslist]
