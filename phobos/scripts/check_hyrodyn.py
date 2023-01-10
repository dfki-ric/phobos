#!python3

def can_be_used():
    try:
        import hyrodyn
        return True
    except:
        return False


def cant_be_used_msg():
    return "Hyrodyn might not be installed!"


INFO = 'Checks whether the model can be loaded in Hyrodyn.'


def main(args):
    from phobos.utils import hyrodyn as hyrodyn_utils
    import argparse
    import os.path as path
    from ..core import Robot
    from ..commandline_logging import setup_logger_level

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('robot_file', type=str, help='Path to the urdf file')
    parser.add_argument('--submechanisms_file', type=str, help='Path to the urdf or smurf file', default=None)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default="INFO")
    args = parser.parse_args(args)

    log = setup_logger_level(log_level=args.loglevel)
    log.info("\n--> Checking Model in Hyrodyn!")
    robot = Robot(inputfile=args.robot_file, submechanisms_file=args.submechanisms_file)
    report = hyrodyn_utils.get_load_report(robot.xmlfile, robot.submechanisms_file)
    hyrodyn_utils.debug_report(report, robot.xmlfile, robot.submechanisms_file)


if __name__ == '__main__':
    import sys
    main(sys.argv)
