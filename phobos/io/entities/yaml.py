
# TODO add shebang and intro documentation
import yaml
import os
from datetime import datetime
import phobos.defs as defs
from phobos.phoboslog import log


def exportYAML(model, path):
    """This function exports a given robot model to a specified filepath as YAML.

    Args:
      model(dict -- the generated robot model dictionary): The robot model to export
      path(str): The filepath to export the robot to. *WITH filename!*

    Returns:

    """
    log("phobos YAML export: Writing model data to " + path, "INFO")
    with open(os.path.join(path, model['name'] + '.yaml'), 'w') as outputfile:
        outputfile.write('# YAML dump of robot model "' + model['name'] + '", ' + datetime.now().strftime(
            "%Y%m%d_%H:%M") + "\n")
        outputfile.write("# created with Phobos" + defs.version + " - https://github.com/rock-simulation/phobos\n\n")
        # TODO delete me?
        outputfile.write(yaml.dump(
            model))  # default_flow_style=False))
        #last parameter prevents inline formatting for lists and dictionaries


# registering export functions of types with Phobos
entity_type_dict = {'yaml': {'export': exportYAML,
                             'extensions': ('yaml', 'yml')}
                    }
