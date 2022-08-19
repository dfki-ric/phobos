#!python3
from ..utils.commandline_logging import get_logger

def can_be_used():
    return True


def cant_be_used_msg():
    return ""


INFO = 'Gives access to a lot of tools to edit a robot model.'


def main(args):
    import argparse
    import os.path as path
    
    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('urdf', type=str, help='Path to the urdf file')
    parser.add_argument('-o', '--output', type=str, help='Writes the result as YAML to the given file', action="store",
                        default=None)
    parser.add_argument('-a', '--all', help='Writes everything not only issues', action="store_true", default=False)
    parser.add_argument('-w', '--warn', help='Show warnings', action="store_true", default=False)
    parser.add_argument("verbose_argument", '-v', '--verbose',
                        type=str, help="allowed levels: DEBUG, INFO, WARNING, ERROR, CRITICAL", default=None)

    args = parser.parse_args(args)

    log = get_logger(__name__, verbose_argument=args.verbose_argument)

if __name__ == '__main__':
    import sys
    main(sys.argv)
