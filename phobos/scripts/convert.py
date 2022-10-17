#!python3

def can_be_used():
    return True


def cant_be_used_msg():
    return ""


INFO = 'Converts the given input robot file to SDF/URDF/PDF/SMURF.'


def main(args):
    import argparse
    import os.path as path
    from ..core.robot import Robot
    from ..defs import BASE_LOG_LEVEL
    from ..utils.commandline_logging import setup_logger_level

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('input_file', type=str, help='Path to the urdf or smurf file')
    parser.add_argument('output_file', type=str, help='Writes the result to the given file. '
                                                      'The output is determined by the file ending, '
                                                      'if the output_file has no ending like sdf, urdf or pdf and '
                                                      'is a (non-existing) directory SMURF will be exported.',
                        action="store", default=None)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)
    log = setup_logger_level(log_level=args.loglevel)

    robot = Robot(inputfile=args.input_file)

    if args.output_file.lower().endswith("sdf"):
        log.info("Converting to SDF")
        robot.export_sdf(outputfile=args.output_file)
    elif args.output_file.lower().endswith("urdf"):
        log.info("Converting to URDF")
        robot.export_urdf(outputfile=args.output_file)
    elif args.output_file.lower().endswith("pdf"):
        log.info("Converting to PDF")
        robot.export_pdf(outputfile=args.output_file)
    elif not path.exists(args.output_file) or path.isdir(args.output_file):
        log.info("Converting to SMURF")
        robot.export_smurf(outputfile=args.output_file)
    else:
        log.ERROR(f"Don't know which conversion to apply for {args.output_file}")
        sys.exit(1)


if __name__ == '__main__':
    import sys

    main(sys.argv)
