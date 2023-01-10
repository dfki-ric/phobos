#!python3

def can_be_used():
    from ..defs import XDBI_AVAILABLE
    return XDBI_AVAILABLE


def cant_be_used_msg():
    return "tools/cad/deimos is not available!"


INFO = 'Creates the xtype database entry for the given model repository'


def query_var(varname, default_value=None, test=None, allow_none=False):
    _input = None
    while _input == None:
        _input = input((f"{varname}:" if default_value is None else f"{varname} [{default_value}]:"))
        if default_value is None and _input.strip() == "" and allow_none is False:
            print(f"Sorry, please define a value for {varname}")
            _input = None
        elif default_value is not None and _input.strip() == "":
            _input = default_value
        elif _input.strip() in ["", "null", "None"] and allow_none:
            _input = None
            break
        else:
            _input = _input.strip()
        if test is not None:
            if not test(_input):
                _input = None
    return _input


def test_dbi(input):
    if input.upper() not in ["CLIENT", "SERVERLESS"]:
        print("Please type either CLIENT or SERVERLESS")
        return False
    return True


def order_xtype_properties(propname):
    xtype_property_order = ["name", "version", "domain", "type", "repository", "designedBy", "projectName", "date", "maturity"]
    if propname in xtype_property_order:
        return str(xtype_property_order.index(propname)).rjust(2,"0")
    return f"{len(xtype_property_order)}_{propname}"


def main(args):
    import argparse
    import xtypes_py
    import xdbi_py
    import datetime
    import os
    import os.path as path
    from ..utils.git import get_repo_data
    from ..core import Robot
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    cm = xtypes_py.ComponentModel()

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('model_directory', type=str, help="The model repository that shall be added to the database", default=os.getcwd())
    parser.add_argument('-n', '--non-interactive', help="The version of the project.", action="store_true", default=False)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"], default=BASE_LOG_LEVEL)

    parser.add_argument('--db_interface', choices=["CLIENT", "SERVERLESS"], help="The type of the database interface", default=None)
    parser.add_argument('--db_path', type=str, help="The url of the database", default=None)
    parser.add_argument('--db_graph', type=str, help="The url of the database", default=None)
    parser.add_argument('--remote', type=str, help='Path to git remote where the model is stored', default=None)

    _default = {}
    for prop in sorted(cm.get_properties().keys(), key=order_xtype_properties):
        kwargs = {
            "default": None,
            "help": f"Value for ComponentModel property {prop}"
        }
        if len(cm.get_allowed_property_values(prop)) == 0:
            kwargs["type"] = str
        else:
            kwargs["choices"] = cm.get_allowed_property_values(prop)
        parser.add_argument(f'--{prop}', **kwargs)
        _default[prop] = cm.get_property(prop)

    args = parser.parse_args(args)
    log = setup_logger_level(log_level=args.loglevel)

    # detect default values
    _default_db_interface = "CLIENT"
    _default_db_path = "0.0.0.0:8183"
    _default_db_graph = None

    robot_file = None
    for root, dirs, files in os.walk(args.model_directory):
        for f in files:
            if f.lower().endswith("smurf"):
                robot_file = os.path.join(root, f)
                break
            if robot_file is not None:
                break
        if robot_file is not None:
            break
    if robot_file is not None:
        robot = Robot(inputfile=robot_file)
    else:
        raise AssertionError("Directory doesn't contain a smurf robot file!")

    _default["name"] = robot.name
    _default["version"] = "0.0.0"
    _default["date"] = datetime.date.today().isoformat()
    _default["projectName"] = cm.projectName
    _default["domain"] = "MECHANICS"
    _default["maturity"] = cm.maturity
    _author, _maintainer, _default["repository"] = get_repo_data(args.model_directory)
    if _default["repository"].startswith("git@"):
        _default["repository"] = _default["repository"].replace(":", "/").replace("//", "/").replace("git@", "https://")
    _default["designedBy"] = ", ".join({_author, _maintainer}).replace("[", "").replace("]", "")
    _default["designedBy"] = _default["designedBy"] if len(_default["designedBy"]) > 0 or _default["designedBy"] is not None else cm.designedBy

    # run interactive mode
    xdbi = None
    if not args.non_interactive:
        print("How to connect to the database:")
        while xdbi is None:
            if args.db_interface is None:
                args.db_interface = query_var("  db_interface", _default_db_interface, test=test_dbi).upper()
            if args.db_path is None:
                args.db_path = query_var("  db_path", _default_db_path )

            if args.db_interface.upper() == "CLIENT":
                xdbi = xdbi_py.Client(xtypes_py.ProjectRegistry(), args.db_path)
                if not xdbi.isConnected():
                    xdbi = None
            elif args.db_interface.upper() == "SERVERLESS":
                xdbi = xdbi_py.Serverless(xtypes_py.ProjectRegistry(), args.db_path)
            else:
                print(f"Can't connect to db with interface {args.db_interface.upper()}")
                xdbi = None
            if xdbi is None:
                args.db_interface = None
                args.db_path = None
        if args.db_graph is None:
            args.db_graph = query_var("  db_graph", xdbi.getWorkingGraph())
        print("Define the ComponentModel properties:")
        for k, v in sorted(_default.items(), key=lambda x: order_xtype_properties(x[0])):
            if getattr(args, k) is None:
                setattr(args, k, query_var("  "+k, v, allow_none=v is None))
    else:
        if args.db_interface.upper() == "CLIENT":
            xdbi = xdbi_py.Client(xtypes_py.ProjectRegistry(), args.db_path)
        elif args.db_interface.upper() == "SERVERLESS":
            xdbi = xdbi_py.Serverless(xtypes_py.ProjectRegistry(), args.db_path)
        else:
            raise AssertionError(f"Can't connect to db with interface {args.db_interface.upper()}")
        xdbi.setWorkingGraph(args.db_graph)
        assert xdbi.isConnected()

    # create xtype
    cm.set_properties({
        k: v if getattr(args, k) is None else getattr(args, k) for k, v in _default.items()
    })

    xdbi.add([cm])


if __name__ == '__main__':
    import sys
    main(sys.argv)
