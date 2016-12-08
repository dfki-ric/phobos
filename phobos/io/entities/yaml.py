def exportModelToYAML(model, filepath):
    """This function exports a given robot model to a specified filepath as YAML.

    :param model: The robot model to export
    :type model: dict -- the generated robot model dictionary
    :param filepath:  The filepath to export the robot to. *WITH filename!*
    :type filepath: str

    """
    log("phobos YAML export: Writing model data to " + filepath, "INFO", "exportModelToYAML")
    with open(filepath, 'w') as outputfile:
        outputfile.write('# YAML dump of robot model "' + model['modelname'] + '", ' + datetime.now().strftime(
            "%Y%m%d_%H:%M") + "\n")
        outputfile.write("# created with Phobos" + defs.version + " - https://github.com/rock-simulation/phobos\n\n")
        outputfile.write(yaml.dump(
            model))  # default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries
