#!python3

def can_be_used():
    return True


def cant_be_used_msg():
    return ""


INFO = "Reduces the given mesh's number of vertices by the given factor, while trying to maintain the geometry as similar as possible."


def main(args):
    import argparse
    import os.path as path
    from ..geometry import io, geometry
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('input_file', type=str, help='Path to the STL file that shall be reduced')
    parser.add_argument('output_file', type=str, help='Writes the result as YAML to the given file', action="store",
                        default=None)
    parser.add_argument('--factor', type=float, help='The factor by which the number of vertices shall be reduced', default=0.8)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)
    log = setup_logger_level(log_level=args.loglevel)

    mesh = io.import_mesh(args.input_file)
    mesh = geometry.reduce_mesh(mesh, float(args.factor))
    io.export_mesh(mesh, args.output_file)


if __name__ == '__main__':
    import sys

    main(sys.argv)
