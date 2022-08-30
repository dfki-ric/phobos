import os
import re
import subprocess
import numpy as np
from copy import deepcopy
from xml.dom.minidom import parseString
from xml.etree import ElementTree as ET
from ..utils.commandline_logging import get_logger
log = get_logger(__name__)


def duplicate(obj, link_obj=False):
    if link_obj:
        return obj
    return deepcopy(obj)


def to_pretty_xml_string(xml):
    xml_string = xml if type(xml) == str else ET.tostring(xml, method='xml').decode('utf-8')
    xml_string = parseString(xml_string)
    return xml_string.toprettyxml(indent='  ').replace(u'<?xml version="1.0" ?>', '').strip()


def read_angle_2_rad(config_input):
    """Converts ["rad"/"deg", value] into the rad value"""
    if type(config_input) is list:
        if config_input[0] == "deg":
            return np.pi * config_input[1] / 180.
        elif config_input[0] == "rad":
            return config_input[1]
        else:
            raise AttributeError("unit not properly defined")
    else:
        log.warning("Rad/deg not defined, taking rad!")
        return config_input


def trunc(values, decimals=0):
    return np.trunc(values*10**decimals)/(10**decimals)


def regex_replace(string, replacements, verbose=False):
    """
    In string applies all replacements defined in replacements dict.
    It can be a list of dicts or a dict of strings that shall be replaced.0
    Regular expressions can be used.
    """
    before = deepcopy(string)
    after = before
    if type(replacements) is dict:
        for old, new in replacements.items():
            after = re.sub(
                r"" + old,
                new,
                after
            )
    elif type(replacements) is list:
        for entry in replacements:
            for old, new in entry.items():
                after = re.sub(
                    r"" + old,
                    new,
                    after
                )
    else:
        raise ValueError("Given replacement must either be dict or list, but it is "+str(type(replacements)))
    if before != after and verbose:
        log.debug(f"Replacing: {before} by {after}")
    return after


def append_string(s, *args, **kwargs):
    """Replacement for print so that the printed string is put to s"""
    new = " ".join([str(a) for a in args])
    if "end" in kwargs.keys():
        new += kwargs["end"]
    else:
        new += "\n"
    if "print" in kwargs.keys() and kwargs["print"]:
        kwargs.pop("print")
        log.debug(*args, **kwargs)
    return s + new


def execute_shell_command(cmd, cwd=None, dry_run=False, silent=False):
    if cwd is None:
        cwd = os.getcwd()
    if not silent:
        log.info(("Skipping" if dry_run else "Executing") + f": {cmd} in {cwd}")
    if dry_run:
        return "", ""
    proc = subprocess.Popen([cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, cwd=cwd)
    # proc.wait()
    (out, err) = proc.communicate()
    if not silent:
        log.info(out.decode('UTF-8'))
        log.info(err.decode('UTF-8'))
        log.info(f"Subprocess returned {proc.returncode}")
    if proc.returncode != 0:
        log.error(out.decode('UTF-8'))
        log.error(err.decode('UTF-8'))
        raise Exception("Subprocess failed! Return code:"+str(proc.returncode) +
                        " Command was: "+cmd+"\nError:"+err.decode('UTF-8'))
    return out.decode('UTF-8'), err.decode('UTF-8')


def create_symlink(pipeline, target, link):
    create_dir(pipeline, os.path.dirname(link))
    if os.path.exists(link):
        log.debug(f"Deleting old link {pipeline.relpath(link) if pipeline is not None else link}")
        os.system("rm {}".format(link))
    log.debug(f"Linking {pipeline.relpath(link) if pipeline is not None else link} to {pipeline.relpath(target) if pipeline is not None else target}")
    os.symlink(target, link)


def create_dir(pipeline, directory):
    if not os.path.exists(directory):
        log.debug(f"Creating {pipeline.relpath(directory) if pipeline is not None else directory}")
        execute_shell_command("mkdir -p " + directory, silent=True)
    else:
        log.debug(f"Already exists: {pipeline.relpath(directory)}")


def remove_dir(pipeline, directory):
    if os.path.exists(directory):
        log.debug(f"Deleting {pipeline.relpath(directory) if pipeline is not None else directory}")
        os.system("rm -rf {}".format(directory))


def recreate_dir(pipeline, directory):
    remove_dir(pipeline, directory)
    create_dir(pipeline, directory)


def copy(pipeline, src, dst, silent=False):
    if not silent:
        log.debug(f"Copying {pipeline.relpath(src) if pipeline is not None else src} to {pipeline.relpath(dst) if pipeline is not None else dst}")
    execute_shell_command("mkdir -p {} || true".format(os.path.dirname(dst)), silent=True)
    os.system("cp -rP {} {}".format(src, dst))


def store_persisting_files(pipeline, repo, list_of_files, temp_dir):
    log.debug("Storing the following files:")
    for f in list_of_files:
        src = os.path.join(repo, f)
        dst = os.path.join(temp_dir, f)
        create_dir(pipeline, os.path.dirname(dst))
        log.debug(pipeline.relpath(src))
        copy(pipeline, src, dst, silent=False)


def restore_persisting_files(pipeline, repo, list_of_files, temp_dir):
    log.debug("Restoring the following files:")
    for f in list_of_files:
        src = os.path.join(temp_dir, f)
        dst = os.path.join(repo, f)
        create_dir(pipeline, os.path.dirname(dst))
        log.debug(pipeline.relpath(dst))
        copy(pipeline, src, dst, silent=False)


def edit_name_string(name, prefix=None, suffix=None, replacements=None):
    if replacements is None:
        replacements = {}
    name = regex_replace(name, replacements)
    if prefix:
        name = str(prefix) + name
    if suffix:
        name = name + str(suffix)
    return name


def list_files(startpath, ignore=["\.git"]):
    file_list = []
    for directory, _, files in os.walk(startpath, followlinks=True):
        directory = os.path.relpath(directory, startpath)
        if any([re.search(pattern, directory) is not None for pattern in ignore]):
            continue
        for file in files:
            if any([re.search(pattern, file) is not None for pattern in ignore]):
                continue
            path = os.path.join(directory, file)
            file_list += [path]
    return sorted(file_list)


def check_for_iterable(check_object):
    try:
        some_object_iterator = iter(check_object)
    except TypeError as te:
        log.error(check_object, 'is not iterable')
