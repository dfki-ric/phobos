#!python3

def can_be_used():
    return True


def cant_be_used_msg():
    return "Unknown error!"


INFO = 'Process simulation models automatically.'


def main(args):
    import sys

    import argparse
    import os
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    parser = argparse.ArgumentParser(description=INFO, prog="phobos "+os.path.basename(__file__)[:-3])
    parser.add_argument('config_file', type=str, help='Path to the pipeline configfile', default="pipeline.yml")
    parser.add_argument('-p', '--process', help='Process Models', action='store_true', default=False)
    parser.add_argument('-t', '--test', help='Test Models', action='store_true', default=False)
    parser.add_argument('-d', '--deploy', help='Deploy Models (This is made for usage in the phobos-CI-docker rather than local/manual usage)', action='store_true', default=False)
    parser.add_argument('-v', '--verify',
                        help='Verify Success. Lets the process finish with exit code != 0 if a job has failed',
                        action='store_true', default=False)
    parser.add_argument('--allow_na_in_verify',
                        help='Set this when you run this for a branch from which will not be deployed',
                        action='store_true', default=False)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    parser.add_argument('--logfile', type=str, help='where to write a logfile', default=None)
    args = parser.parse_args(args)
    log = setup_logger_level(log_level=args.loglevel, file_name=args.logfile)
    test_failed = False
    if any([args.process, args.test, args.deploy, args.verify]) is True:
        if os.path.isfile(args.config_file):
            from phobos.ci.pipeline import Pipeline
            pipeline = Pipeline(args.config_file, processed_model_exists=not args.process)
        else:
            raise FileNotFoundError("Config file not found!")

        phases = []
        if args.process:
            pipeline.process_models()
            phases += ["process"]

        if args.test:
            test_failed = not pipeline.test_models()
            phases += ["test"]

        if args.deploy and not test_failed:
            pipeline.deploy_models()
            phases += ["deploy"]

        if args.verify:
            phases = ["process", "deploy"]

        if len(phases) > 0:
            pipeline.print_fail_log(file=sys.stderr)
        print("Success rate: {:.2f} %".format(pipeline.get_coverage(
            phases=phases if not args.verify else ["process", "test", "deploy"],
            allow_na=args.allow_na_in_verify)*100), file=sys.stderr)
        print("Number of not successfully processed models:", pipeline.number_unfinished_models(), file=sys.stderr)
        if not args.verify:
            return 0
        else:
            return pipeline.get_coverage(phases=phases, allow_na=args.allow_na_in_verify) != 1
    else:
        parser.print_help()


if __name__ == '__main__':
    import sys
    main(sys.argv)
