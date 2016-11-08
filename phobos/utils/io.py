
import os
from phobos import defs
from phobos.logging import log

indent = '  '
xmlHeader = '<?xml version="1.0"?>\n<!-- created with Phobos ' + defs.version + ' -->\n'

def xmlline(ind, tag, names, values):
    """This function generates an xml line with specified values.
    To use this function you need to know the indentation level you need for this line.
    Make sure the names and values list have the correct order.

    :param ind: The level of indentation
    :type ind: int, has to be positive!
    :param tag: This is the xml lines tag
    :type tag: String
    :param names: This are the names of the xml lines attributions.
    :type names: list, check for analogue order to values.
    :param values: This are the values of the xml lines attributions.
    :type values: list, check  for analogue order to names.
    :return: String -- the generated xml line.

    """
    line = [indent * max(0, ind) + '<' + tag]
    for i in range(len(names)):
        line.append(' ' + names[i] + '="' + str(values[i]) + '"')
    line.append('/>\n')
    return ''.join(line)


def l2str(items, start=0, end=-1):
    """This function takes a list and generates a String with its element.

    :param items: The list of elements you want to generate a String from. *Make sure the elements can be cast to
    Strings with str().*
    :type items: list
    :param start: The inclusive start index to iterate the list from. If negative it defaults to 0.
    :type start: int
    :param end: The exclusive end to iterate the list to. If negative its defaults to len(items).
    :type end: int
    :return: str - the generated string.

    """
    start = max(start, 0)
    end = end if end >= 0 else len(items)
    return ' '.join([str(i) for i in items[start:end]])


def securepath(path):
    """This function checks whether a path exists or not.
    If it doesn't the functions creates the path.

    :param path: The path you want to check for existence *DIRECTORIES ONLY*
    :type path: str
    :return: String -- the path given as parameter, but secured by expanding ~ constructs.

    """
    if not os.path.exists(path):
        try:
            os.makedirs(path)
        except NotADirectoryError:
            log(path + " is not a valid directory", "ERROR", "securepath")
    return os.path.expanduser(path)
