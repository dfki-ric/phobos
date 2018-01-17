
# TODO add shebang and document introduction
import os.path
import bpy
from phobos import defs
from phobos.phoboslog import log
from phobos.utils import selection as sUtils
from phobos.utils import naming as nUtils


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
        # TODO notadirectoryerror is not recognized... where to import it from?
        except NotADirectoryError:
            log(path + " is not a valid directory", "ERROR")
            return None
    return path
    # TODO delete me?
    # os.path.expanduser(path)  # this is probably not necessary


def getExpSettings():
    # DOCU add some docstring
    return bpy.data.worlds[0].phobosexportsettings


def getExportModels():
    # DOCU add some docstring
    if getExpSettings().selectedOnly:
        roots = {root for root in sUtils.getRoots() if root.select}
    else:
        roots = sUtils.getRoots()
    return list(roots)


def getExportEntities():
    # DOCU add some docstring
    if getExpSettings().selectedOnly:
        roots = [obj for obj in sUtils.getSelectedObjects() if sUtils.isEntity(obj)]
    else:
        roots = [obj for obj in bpy.context.scene.objects if sUtils.isEntity(obj)]
    return roots


def getModelListForEnumProp(self, context):
    # DOCU add some docstring
    rootnames = set([r['modelname'] for r in getExportModels()])
    return sorted([(r,) * 3 for r in rootnames])


def getOutputMeshtype():
    # DOCU add some docstring
    return str(getExpSettings().outputMeshtype)


def getOutputMeshpath(path, meshtype=None):
    # DOCU add some docstring
    if getExpSettings().structureExport:
        return os.path.join(path, 'meshes', meshtype if meshtype else getOutputMeshtype())
    else:
        return path


def getExportPath(structure=""):
    """Returns the export path joined with the subfolder structure if structured export is enabled

    :param structure: The name of the subfolder to append to the path
    :return: The export path
    """
    path_fragment = getExpSettings().path
    if getExpSettings().structureExport and structure:
        path_fragment = os.path.join(path_fragment, structure)
    if os.path.isabs(path_fragment):
        return path_fragment
    else:
        return os.path.normpath(os.path.join(bpy.path.abspath('//'), path_fragment))


def getAbsolutePath(path):
    # DOCU add some docstring
    if os.path.isabs(path):
        return path
    else:
        return os.path.join(bpy.path.abspath('//'), path)


def importBlenderModel(filepath, namespace='', prefix=True):
    if (os.path.exists(filepath) and os.path.isfile(filepath) and
       filepath.endswith('.blend')):
        log("Importing Blender model" + filepath, "INFO")
        objects = []
        with bpy.data.libraries.load(filepath) as (data_from, data_to):
            for obj in data_from.objects:
                objects.append({'name': obj})
        bpy.ops.wm.append(directory=filepath + "/Object/", files=objects)
        bpy.ops.view3d.view_selected(use_all_regions=False)
        # allow the use of both prefixes and namespaces, thus truly merging
        # models or keeping them separate for export
        if namespace != '':
            if prefix:
                for obj in bpy.context.selected_objects:
                    # set prefix instead of namespace
                    obj.name = namespace + '__' + obj.name
                    # make sure no internal name-properties remain
                    for ptype in defs.subtypes:
                        nametag = ptype + "/name"
                        if nametag in obj:
                            del obj[nametag]
            else:
                for obj in bpy.context.selected_objects:
                    nUtils.addNamespace(obj, namespace)
        return True
    else:
        return False
