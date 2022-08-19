#!python3
from ..utils.commandline_logging import get_logger

def can_be_used():
    return True


def cant_be_used_msg():
    return "Unknown error!"


INFO = 'Test the latest model using a CI-pipeline.'


def main(args):
    import sys

    import argparse
    import os

    parser = argparse.ArgumentParser(description=INFO, prog="phobos "+os.path.basename(__file__)[:-3])
    parser.add_argument('config_file', type=str, help='Path to the test configfile', default="test_config.yml")
    parser.add_argument("verbose_argument", '-v', '--verbose',
                        type=str, help="allowed levels: DEBUG, INFO, WARNING, ERROR, CRITICAL", default=None)
    args = parser.parse_args(args)

    log = get_logger(__name__, verbose_argument=args.verbose_argument)
    if os.path.isfile(args.config_file):
        from phobos.ci.pipeline import TestingPipeline
        pipeline = TestingPipeline(root=os.getcwd(), configfile=args.config_file)

        test_failed = not pipeline.test_models()

        print("Success rate: {:.2f} %".format(pipeline.get_coverage() * 100), file=sys.stderr)
        sys.exit(test_failed)
    else:
        parser.print_help()
        raise Exception("Config file not found!")


if __name__ == '__main__':
    import sys
    main(sys.argv)
