#!python3

def can_be_used():
    from ..defs import XDBI_AVAILABLE
    return XDBI_AVAILABLE


def cant_be_used_msg():
    return "tools/cad/deimos is not available!"


INFO = 'Creates the xtype database entry for the given model repository'


def main(args):
    import argparse
    import xtypes_py
    import xdbi_py
    import datetime
    import os.path as path
    from ..utils.git import get_repo_data
    from ..defs import BASE_LOG_LEVEL
    from ..utils.commandline_logging import setup_logger_level

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('model_repository', type=str, help='Path to local git repository where the model is stored'),
    parser.add_argument('db_path', type=str, help="The url of the database")
    # Todo working graph, serverless etc
    parser.add_argument('--name', type=str, help="name property of the created ComponentModel"),
    parser.add_argument('--version', type=str, help="version  property of the created ComponentModel"),
    parser.add_argument('--designedBy', type=str, help="designedBy  property of the created ComponentModel",
                        default=None),
    parser.add_argument('--projectName', type=str, help="projectName  property of the created ComponentModel",
                        default=None),
    parser.add_argument('--maturity', choices=[ "UNDEFINED", "INPROGRESS", "UNSTABLE", "TESTING", "STABLE"],
                        help="maturity  property of the created ComponentModel", default=None),

    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)
    log = setup_logger_level(log_level=args.loglevel)

    xdbi = xdbi_py.Client(xtypes_py.ProjectRegistry(), args.db_path)
    assert xdbi.isConnected()

    cm = xtypes_py.ComponentModel()
    cm.name = args.name
    cm.version = args.version
    if args.designedBy is not None:
        cm.designedBy = args.designedBy
    if args.projectName is not None:
        cm.projectName = args.projectName
    if args.maturity is not None:
        cm.maturity = args.maturity
    cm.date = datetime.date.today().isoformat()
    _, _, cm.repository = get_repo_data(args.model_repository)

    xdbi.add(cm)


if __name__ == '__main__':
    import sys

    main(sys.argv)
