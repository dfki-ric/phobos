#!python3


def can_be_used():
    return True


def cant_be_used_msg():
    return "Unknown error!"


INFO = 'Test the latest model using a CI-pipeline.'


def main(args):
    import sys

    import argparse
    import os

    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    parser = argparse.ArgumentParser(description=INFO, prog="phobos "+os.path.basename(__file__)[:-3])
    parser.add_argument('config_file', type=str, help='Path to the test configfile', default="test_config.yml")
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)

    log = setup_logger_level(log_level=args.loglevel)
    if os.path.isfile(args.config_file):
        from phobos.ci.pipeline import TestingPipeline
        pipeline = TestingPipeline(root=os.getcwd(), configfile=args.config_file)

        test_failed = not pipeline.test_models()

        print("Success rate: {:.2f} %".format(pipeline.get_coverage() * 100), file=sys.stderr)
        return test_failed
    else:
        parser.print_help()
        raise Exception(f"Config file '{args.config_file}' not found!")


if __name__ == '__main__':
    import sys
    main(sys.argv)
