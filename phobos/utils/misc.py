import os
import platform
import re
import subprocess
from copy import deepcopy
from xml.dom.minidom import parseString
from xml.etree import ElementTree as ET

import numpy as np

from ..commandline_logging import get_logger

log = get_logger(__name__)


def duplicate(obj, link_obj=False):
    if link_obj:
        return obj
    return deepcopy(obj)


def to_pretty_xml_string(xml):
    xml_string = xml if type(xml) == str else ET.tostring(xml, method='xml').decode('utf-8')
    xml_string = parseString(xml_string)
    return xml_string.toprettyxml(indent='  ').replace(u'<?xml version="1.0" ?>', '').strip()


def merge_default(input_dict, default_dict):
    if input_dict is None or len(input_dict) == 0:
        return default_dict
    for _k, _v in default_dict.items():
        if _k not in input_dict:
            input_dict[_k] = _v
    return input_dict


def sys_path(path):
    if path is None:
        return path
    if platform.system() == "Windows":
        return path.replace("/", "\\")
    else:
        return path.replace("\\", "/")


def posix_path(path):
    if path is None:
        return path
    return path.replace("\\", "/")


def deepen_dict(input_dict):
    out = {}
    for k, v in input_dict.items():
        if "/" in k:
            out[k.split("/", 1)[0]] = deepen_dict({k.split("/", 1)[1]: v})
        else:
            out[k] = v
    return out


def flatten_dict(input_dict):
    out = {}
    for k, v in input_dict.items():
        if type(v) == dict:
            for k2, v2 in flatten_dict(v).items():
                out[k+"/"+k2] = v2
        else:
            out[k] = v
    return out


def read_number_from_config(config_input):
    """Converts ["rad"/"deg", value] into the rad value, computes *+/- and pi in input string"""
    if type(config_input) is list:
        if config_input[0].lower() == "deg":
            return np.pi * eval(str(config_input[1])) / 180.
        elif config_input[0].lower() == "rad":
            return eval(str(config_input[1]))
        else:
            raise AttributeError("unit not properly defined")
    else:
        return eval(str(config_input))


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
        log.info(" ".join(args))
    return s + new


def is_binary_file(filepath):
    # adapted from https://stackoverflow.com/a/7392391/14537313
    textchars = bytearray({7, 8, 9, 10, 12, 13, 27} | set(range(0x20, 0x100)) - {0x7f})
    with open(filepath, "rb") as f:
        return bool(f.read(256).translate(None, textchars))


def execute_shell_command(cmd, cwd=None, dry_run=False, silent=False):
    if cwd is None:
        cwd = os.getcwd()
    if not silent:
        log.debug(("Skipping" if dry_run else "Executing") + f": {cmd} in {cwd}")
    if dry_run:
        return "", ""
    proc = subprocess.Popen([cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, cwd=cwd)
    # proc.wait()
    (out, err) = proc.communicate()
    if not silent:
        log.debug(out.decode('UTF-8'))
        log.debug(err.decode('UTF-8'))
        log.debug(f"Subprocess returned {proc.returncode}")
    if proc.returncode != 0:
        if not silent:
            log.error(out.decode('UTF-8'))
            log.error(err.decode('UTF-8'))
        raise Exception("Subprocess failed! Return code:"+str(proc.returncode) +
                        " Command was: "+cmd+" Executed in: "+cwd+
                        "\nError:"+err.decode('UTF-8'))
    return out.decode('UTF-8'), err.decode('UTF-8')


def make_icon(im, thumbnail_path, size=512, trim=True):
    from PIL import Image, ImageChops
    if trim:
        bg = Image.new("RGB", im.size, im.getpixel((0, 0)))
        diff = ImageChops.difference(im.convert("RGB"), bg)
        diff = ImageChops.add(diff, diff)
        bbox = diff.getbbox()
        if bbox:
            im = im.crop(bbox)
    im.thumbnail((size, size))
    bg = Image.new("RGB", (size, size), im.getpixel((0, 0)))
    bg.paste(im, (int((size-im.size[0])/2), int((size-im.size[1])/2)))
    bg.save(thumbnail_path)


def get_thumbnail(robotfile, icon_size=512):
    from PIL import Image, ImageChops

    if robotfile.lower().endswith(".smurf"):
        base_dir = os.path.dirname(robotfile)
        with open(robotfile, "r") as f:
            robotfile = [x for x in yaml.safe_load(f.read())["files"] if x.endswith("df")][0]
        robotfile = os.path.normpath(os.path.join(base_dir, robotfile))
    # use pybullet to get an image
    import pybullet as pb
    pb.connect(pb.DIRECT)
    if robotfile.lower().endswith(".sdf"):
        pb.loadSDF(robotfile)
    elif robotfile.lower().endswith(".urdf"):
        pb.loadURDF(robotfile)
    all = pb.getVisualShapeData(0)
    max_box = np.max([np.array(x[3])+np.array(x[5]) for x in all], axis=0)
    projectionMatrix = pb.computeProjectionMatrixFOV(
        fov=45.0,
        aspect=1.0,
        nearVal=0.5,
        farVal=max(max_box)*5
    )
    viewMatrix = pb.computeViewMatrix(
        cameraEyePosition=(max_box*2).tolist(),
        cameraTargetPosition=[0, 0, 0],
        cameraUpVector=[0, 0, 1]
    )
    width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
        width=icon_size*4,
        height=icon_size*4,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix
    )
    im = Image.fromarray(rgbImg)
    return im


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
        os.makedirs(directory, exist_ok=True)
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


def list_files(startpath, ignore=["\.git"], resolve_symlinks=False, abs_path=False):
    file_list = []
    for directory, _, files in os.walk(startpath, followlinks=True):
        if not abs_path:
            directory = os.path.relpath(directory, startpath)
        if any([re.search(pattern, directory) is not None for pattern in ignore]):
            continue
        for file in files:
            if any([re.search(pattern, file) is not None for pattern in ignore]):
                continue
            path = (os.path.join(directory, file))
            if resolve_symlinks:
                assert abs_path == True, "Use abs_path=True when resolving symlinks"
                path = os.path.realpath(os.path.abspath(path))
            file_list += [path]
    return sorted(file_list)


def check_for_iterable(check_object):
    try:
        some_object_iterator = iter(check_object)
    except TypeError as te:
        log.error(check_object, 'is not iterable')


def color_parser(*args, rgba=None):
    out = None
    count = len(args)
    if rgba is not None:
        out = rgba
    elif count == 4 or count == 3:
        out = args
    elif count == 1:
        out = args[0]
        if type(out) == dict:
            out = [out[k] for k in "rgba" if k in out.keys()]
    elif count == 0:
        out = None
    if out is not None:
        if type(out) != list:
            out = list(out)
        if len(out) == 3:
            out += [1.]
        if len(out) != 4:
            raise Exception(f'Invalid color argument count for argument "{out}"')
    else:
        return None
    # round everything
    out = [int(x * 255) / 255 for x in out]
    return out


def to_hex_color(color_as_list):
    return "#" + "".join([hex(int(x * 255))[2:] for x in color_as_list])
