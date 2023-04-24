#!python3
def can_be_used():
    return True


def cant_be_used_msg():
    return "Unknown error!"


INFO = 'Loads a SMURFS/SMURFA file and exports the urdf(s)/smurf of the assembly.'


def main(args):
    import argparse
    import os

    from ..core import Arrangement
    from ..utils import resources
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=str, help="SMURFA or SMURFS file")
    parser.add_argument("output", type=str,  help="The output directory")
    parser.add_argument('-m', '--copy-meshes', help='Copies the meshes', action='store_true', default=False)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    # [ToDo v2.0.0] add setter for export_config, both key or config file
    args = parser.parse_args(args)

    log = setup_logger_level(log_level=args.loglevel)
    log.info("Unique names for all links and joints are assumed to create a valid .urdf file!")

    arrangement = Arrangement(inputfile=args.input)
    if arrangement.is_empty():
        log.error("Given file is empty!")
        return 1
    if arrangement.has_one_root():
        assembly = arrangement.assemble()
        assembly.export(outputdir=args.output, export_config=resources.get_default_export_config(), with_meshes=args.copy_meshes)
    else:
        for root in arrangement.get_root_entities():
            assembly = arrangement.assemble(root)
            assembly.export(outputdir=os.path.join(args.output, f"root-{root.name}"), export_config=resources.get_default_export_config(), with_meshes=args.copy_meshes)
    return 0


if __name__ == '__main__':
    import sys
    main(sys.argv)
