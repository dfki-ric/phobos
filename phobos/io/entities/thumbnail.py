
# TODO add shebang and intro documentation
import bpy
from phobos.phoboslog import log
from phobos.utils.blender import createPreview


def exportPreview(model, path, mesh_format=''):
    """This function exports a given robot model to a specified filepath as YAML.

    :param model: The robot model to export
    :type model: dict -- the generated robot model dictionary
    :param path:  The filepath to export the robot to. *WITH filename!*
    :type path: str

    """
    log("Phobos Thumbnail export: Creating thumbnail in " + path, "INFO")
    visuals = []
    for linkname in model['links']:
        for visualname in model['links'][linkname]['visual']:
            visuals.append(bpy.data.objects[visualname])
    createPreview(visuals, path, model['name'])


# registering export functions of types with Phobos
entity_type_dict = {'thumbnails': {'export': exportPreview,
                             'extensions': ('png', 'jpg')}
                    }
