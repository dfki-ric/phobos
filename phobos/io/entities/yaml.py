# TODO add shebang and intro documentation
import yaml
import os
from datetime import datetime
import phobos.defs as defs
from phobos.phoboslog import log


def exportYAML(model, path):
    """This function exports a given robot model to a specified filepath as YAML.

    Args:
        model (dict):  Phobos robot model dictionary
        path (str): filepath to export the robot to (*WITH* filename)
    """
    log("phobos YAML export: Writing model data to " + path, "INFO")
    with open(os.path.join(path, model['name'] + '.yaml'), 'w') as outputfile:
        outputfile.write("# YAML dump of robot model '{}', {}\n".format(
            model['name'], datetime.now().strftime("%Y%m%d_%H:%M")))
        outputfile.write("# created with Phobos {} - https://github.com/dfki-ric/phobos\n\n".format(
            defs.version))

        # write the yaml dump to the file
        outputfile.write(yaml.dump(model))


# registering export functions of types with Phobos
entity_type_dict = {'yaml': {'export': exportYAML,
                             'extensions': ('yaml', 'yml')}}
