#!python3
from ..utils.commandline_logging import get_logger


def can_be_used():
    from ..defs import DEIMOS_AVAILABLE
    return DEIMOS_AVAILABLE


def cant_be_used_msg():
    return "tools/cad/deimos is not available!"


INFO = 'Assemble the given model to a urdf.'


def main(args):
    import sys

    import argparse
    import os
    import pkg_resources

    from phobos.ci import XTypePipeline
    import phobos.utils.misc as misc

    try:
        from deimos.deimos import Deimos
    except ImportError:
        print("Package tools/cad/deimos is required for this tool!")
        sys.exit(1)

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + os.path.basename(__file__)[:-3])

    parser.add_argument('--model_name', type=str, help='The model name in the database', default=None, action='store')
    parser.add_argument('--model_version', type=str, help='The model version in the database', default=None,
                        action='store')

    group_lg = parser.add_mutually_exclusive_group()
    group_lg.add_argument('-g', '--generate-cfg', type=str, action='store',
                          help='Provide configuration file write to the file specified (e.g. to install this model in'
                               ' a pipeline or to edit the config', default=None)
    group_lg.add_argument('-l', '--load-cfg', type=str, help='Loads the given file and assembles the URDF accordingly',
                          action='store', default=None)

    parser.add_argument('-f', '--overwrite-existing', action='store_true',
                        help='Overwrites the existing output', default=False)
    parser.add_argument('-p', '--process', help='Process the model according to configuration', action='store_true',
                        default=False)
    parser.add_argument('-t', '--test', help='Test model the model according to configuration', action='store_true',
                        default=False)
    # parser.add_argument('-d', '--deploy', help='Deploy Model', action='store_true', default=False)

    parser.add_argument('-o', '--output', help='Output model directory', type=str, action='store', default=None)
    parser.add_argument('-k', '--keep-temp', help='Keep the temp directory', action='store_true', default=False)

    parser.add_argument("verbose_argument", '-v', '--verbose',
                        type=str, help="allowed levels: DEBUG, INFO, WARNING, ERROR, CRITICAL", default=None)
    args = parser.parse_args(args)

    log = get_logger(__name__, verbose_argument=args.verbose_argument)

    def generate_cfg(cfg_path, overwrite=False):
        if not os.path.exists(os.path.dirname(os.path.abspath(cfg_path))):
            misc.create_dir(None, os.path.dirname(cfg_path))
        if not overwrite and os.path.exists(cfg_path):
            raise FileExistsError("Config file is already there!")
        file = pkg_resources.resource_string("phobos", "data/assembly_config.yml.in")
        file = file.replace("@MODEL_NAME@", args.model_name)
        file = file.replace("@MODEL_VERSION@", args.model_version)
        with open(cfg_path, "w") as f:
            f.write(file)

    def load_cfg(cfg_path):
        if not os.path.exists(cfg_path):
            raise FileNotFoundError("Config file does not exist!")

        pipeline = XTypePipeline(
            cfg_path,
            processed_model_exists=not args.process and args.test,
            only_create=not (args.process or args.test)
        )

        phases = []
        pipeline.process_models()
        phases += ["process"]

        if args.test:
            # test_failed = not pipeline.testModels()
            phases += ["test"]

        if len(phases) > 0:
            pipeline.print_fail_log(file=sys.stderr)
            print("Success rate: {:.2f} %".format(pipeline.get_coverage(phases=phases) * 100), file=sys.stderr)
            log.info("Success rate: {:.2f} %".format(pipeline.get_coverage(phases=phases) * 100))

        if args.output is not None:
            if not args.overwrite_existing and os.path.exists(args.output):
                raise FileExistsError(args.output + " already exists and won't be overwritten")
            elif os.path.exists(args.output):
                misc.remove_dir(pipeline, args.output)
            misc.copy(pipeline, pipeline.model.exportdir, args.output, silent=True)

        if not args.keep_temp:
            misc.remove_dir(pipeline, pipeline.temp_dir)

    if args.model_name is not None and args.model_version is not None\
       and args.generate_cfg is None and args.load_cfg is None:
        path = ".temp_assemble_xtype_cfg.yml"
        generate_cfg(path, overwrite=True)
        load_cfg(path)
        os.remove(path)
    elif args.model_name is not None and args.model_version is not None and args.generate_cfg is not None:
        generate_cfg(args.generate_cfg, overwrite=args.overwrite_existing)
    elif args.load_cfg is not None:
        load_cfg(args.load_cfg)
    else:
        parser.print_help()


if __name__ == '__main__':
    import sys

    main(sys.argv)
