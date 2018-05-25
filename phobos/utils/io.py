
# TODO add shebang and document introduction
import os.path
import bpy
from phobos import defs
from phobos.phoboslog import log
from phobos.utils import selection as sUtils
from phobos.utils import naming as nUtils
from phobos.utils import blender as bUtils


indent = '  '
xmlHeader = '<?xml version="1.0"?>\n<!-- created with Phobos ' + defs.version + ' -->\n'


def xmlline(ind, tag, names, values):
    """Generates an xml line with specified values.
    To use this function you need to know the indentation level you need for this line.
    Make sure the names and values list have the correct order.

    Args:
      ind(int >= 0): Indentation level
      tag(String): xml element tag
      names(list (same order as for values)): Names of xml element's attributes
      values(list (same order as for names)): Values of xml element's attributes

    Returns:
      String -- Generated xml line.

    """
    line = [indent * max(0, ind) + '<' + tag]
    for i in range(len(names)):
        line.append(' ' + names[i] + '="' + str(values[i]) + '"')
    line.append('/>\n')
    return ''.join(line)


def l2str(items, start=0, end=-1):
    """Generates string from (part of) a list.

    Args:
      items(list): List from which the string is derived (elements need to implement str())
      start(int, optional): Inclusive start index for iteration (Default value = 0)
      end(int, optional): Exclusive end index for iteration (Default value = -1)

    Returns:
      str - Generated string.

    """
    start = max(start, 0)
    end = end if end >= 0 else len(items)
    return ' '.join([str(i) for i in items[start:end]])


def securepath(path):
    """Checks whether directories of a path exist and generates them if necessary.

    Args:
      path(str): The path to be secured (directories only)

    Returns:
      String -- secured path as absolute path, None on error

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
    return bpy.data.window_managers[0].phobosexportsettings


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


def getExportPath():
    # DOCU add some docstring
    if os.path.isabs(getExpSettings().path):
        return getExpSettings().path
    else:
        return os.path.normpath(os.path.join(bpy.path.abspath('//'), getExpSettings().path))


def getAbsolutePath(path):
    # DOCU add some docstring
    if os.path.isabs(path):
        return path
    else:
        return os.path.join(bpy.path.abspath('//'), path)


def importBlenderModel(filepath, namespace='', prefix=False):
    if os.path.exists(filepath) and os.path.isfile(filepath) and filepath.endswith('.blend'):
        log("Importing Blender model" + filepath, "INFO")
        objects = []
        with bpy.data.libraries.load(filepath) as (data_from, data_to):
            for objname in data_from.objects:
                objects.append({'name': objname})
        bpy.ops.wm.append(directory=filepath + "/Object/", files=objects)
        imported_objects = bpy.context.selected_objects
        resources = [obj for obj in imported_objects if obj.name.startswith('resource::')]
        new_objects = [obj for obj in imported_objects if not obj.name.startswith('resource::')]
        if resources:
            if 'resources' not in bpy.data.scenes.keys():
                bpy.data.scenes.new('resources')
            sUtils.selectObjects(resources)
            bpy.ops.object.make_links_scene(scene='resources')
            bpy.ops.object.delete(use_global=False)
            sUtils.selectObjects(new_objects)
        bpy.ops.view3d.view_selected(use_all_regions=False)
        # allow the use of both prefixes and namespaces, thus truly merging
        # models or keeping them separate for export
        if namespace != '':
            if prefix:
                for obj in bpy.context.selected_objects:
                    # set prefix instead of namespace
                    obj.name = namespace + '__' + obj.name
                    # make sure no internal name-properties remain
                    for key in obj.keys():
                        try:
                            if obj[key].endswidth("/name"):
                                del obj[key]
                        except AttributeError:  # prevent exceptions from non-string properties
                            pass
            else:
                for obj in bpy.context.selected_objects:
                    nUtils.addNamespace(obj, namespace)
        return True
    else:
        return False


def getConfigPath():
    return bpy.context.user_preferences.addons["phobos"].preferences.configfolder


def importResources(restuple, filepath=None):
    """Accepts a tuple of pairs (tuples) describing resource objects to import. For instance,
    the call reslist=(('joint', 'continuous'), ('sensor', 'camera')) would import two
    resource objects.

    Args:
      restuple: tuple of tuples of length 2
      filepath: path to file from which to load resource (Default value = None)

    Returns:

    """
    currentscene = bpy.context.scene.name
    bUtils.switchToScene('resources')
    # avoid importing the same objects multiple times
    imported_objects = [nUtils.stripNamespaceFromName(obj.name)
                        for obj in bpy.data.scenes['resources'].objects]
    requested_objects = [resource[0] + '_' + resource[1] for resource in restuple]
    new_objects = [obj for obj in requested_objects if obj not in imported_objects]

    # if no filepath is provided, use the path from the preferences
    if not filepath:
        filepath = os.path.join(getConfigPath(), 'resources', 'resources.blend')

    # import new objects from resources.blend
    if new_objects:
        with bpy.data.libraries.load(filepath) as (data_from, data_to):
            objects = [{'name': name} for name in new_objects if name in data_from.objects]
            if objects:
                bpy.ops.wm.append(directory=filepath + "/Object/", files=objects)
            else:
                log('Resource objects could not be imported.', 'ERROR')
                bUtils.switchToScene(currentscene)
                return None
    objects = bpy.context.selected_objects
    for obj in objects:
        nUtils.addNamespace(obj, 'resource')
    bUtils.switchToScene(currentscene)
    return objects


def getResource(restype, resname):
    try:
        resobjname = nUtils.addNamespaceToName(restype + '_' + resname, 'resource')
        return bpy.data.scenes['resources'].objects[resobjname]
    except KeyError:  # no resource scene or key not in scene
        newobjects = importResources(((restype, resname),))
        if newobjects:
            return newobjects[0]
        else:
            return None
