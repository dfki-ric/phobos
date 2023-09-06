#!python3

def can_be_used():
    return True


def cant_be_used_msg():
    return ""


INFO = 'Prints a json string where to find the different model files'


def main(args):
    import argparse
    import os.path as path
    from ..core.robot import Robot
    from ..defs import dump_json
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('robot_file', type=str, help='Path to the urdf, sdf or smurf file')
    # parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
    #                     default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)
    # log = setup_logger_level(log_level=args.loglevel)

    def recurse_find_model_files(robot_file):
        out = {}
        robot = Robot(inputfile=robot_file, shallow=True)
        if robot.xmlfile.endswith("urdf"):
            out["urdf"] = robot.xmlfile
        elif robot.xmlfile.endswith("sdf"):
            out["sdf"] = robot.xmlfile
        if robot.smurffile:
            out["smurf"] = robot.smurffile
        for k, v in robot.additional_files:
            out[k] = v
        for f in robot.inputfiles:
            if "submechanisms" in f:
                if "submechanisms_file" not in out:
                    out["submechanisms_file"] = f
                else:
                    raise RuntimeError("There are multiple submechanism files in this smurf")
        if len(robot.submodel_defs) > 0:
            submodels = {}
            for name, definition in robot.submodel_defs.items():
                submodels[name] = recurse_find_model_files(path.join(path.dirname(robot.smurffile), definition["export_dir"], "smurf", definition["name"]+".smurf"))
            out["submodels"] = submodels
        return out

    out = recurse_find_model_files(args.robot_file)
    print(dump_json(out))


if __name__ == '__main__':
    import sys
    main(sys.argv)
