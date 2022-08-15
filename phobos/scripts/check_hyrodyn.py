#!python3
from ..utils.commandline_logging import setup_logger_level, get_logger
log = get_logger(__name__)

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
    log.info("\n--> Checking Model in Hyrodyn!")
    from phobos.utils import hyrodyn as hyrodyn_utils
    import argparse
    import os.path as path

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('robot_file', type=str, help='Path to the urdf file')
    parser.add_argument('submechanisms_file', type=str, help='Path to the urdf or smurf file')

    args = parser.parse_args(args)

    report = hyrodyn_utils.get_load_report(args.robot_file, args.submechanisms_file)
    hyrodyn_utils.debug_report(report, args.robot_file, args.submechanisms_file)


if __name__ == '__main__':
    import sys

    main(sys.argv)
