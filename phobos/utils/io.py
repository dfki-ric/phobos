
import os
import subprocess
import bpy
from phobos import defs
from phobos.phoboslog import log


indent = '  '
xmlHeader = '<?xml version="1.0"?>\n<!-- created with Phobos ' + defs.version + ' -->\n'


def xmlline(ind, tag, names, values):
    """Generates an xml line with specified values.
    To use this function you need to know the indentation level you need for this line.
    Make sure the names and values list have the correct order.

    :param ind: Indentation level
    :type ind: int >= 0
    :param tag: xml element tag
    :type tag: String
    :param names: Names of xml element's attributes
    :type names: list (same order as for values)
    :param values: Values of xml element's attributes
    :type values: list (same order as for names)
    :return: String -- Generated xml line.

    """
    line = [indent * max(0, ind) + '<' + tag]
    for i in range(len(names)):
        line.append(' ' + names[i] + '="' + str(values[i]) + '"')
    line.append('/>\n')
    return ''.join(line)


def l2str(items, start=0, end=-1):
    """Generates string from (part of) a list.

    :param items: List from which the string is derived (elements need to implement str())
    :type items: list
    :param start: Inclusive start index for iteration
    :type start: int
    :param end: Exclusive end index for iteration
    :type end: int
    :return: str - Generated string.

    """
    start = max(start, 0)
    end = end if end >= 0 else len(items)
    return ' '.join([str(i) for i in items[start:end]])


def securepath(path):
    """Checks whether directories of a path exist and generates them if necessary.

    :param path: The path to be secured (directories only)
    :type path: str
    :return: String -- secured path as absolute path, None on error

    """
    path = os.path.abspath(path)
    if not os.path.exists(path):
        try:
            os.makedirs(path)
        except NotADirectoryError:
            log(path + " is not a valid directory", "ERROR", "securepath")
            return None
    return path
    # os.path.expanduser(path)  # this is probably not necessary


def getgitbranch():
    """Checks whether working directory (of .blend file) contains a git repository.
    Returns branch if repository is found.
    """
    try:
        output = str(subprocess.check_output(['git', 'branch'], cwd=bpy.path.abspath('//'), universal_newlines=True))
        branch = [a for a in output.split('\n') if a.find('*') >= 0][0]
        return branch[branch.find('*')+2:]
    except subprocess.CalledProcessError:
        return None
    except FileNotFoundError:
        log("No git repository found.", "ERROR", origin="utils/io/getgitbranch")
        return None
