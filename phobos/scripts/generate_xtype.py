#!python3

def can_be_used():
    from ..defs import XDBI_AVAILABLE
    return XDBI_AVAILABLE


def cant_be_used_msg():
    return "tools/cad/deimos is not available!"


INFO = 'Creates the xtype database entry for the given model repository'


def query_var(varname, default_value=None, test=None, allow_none=False, choices=None):
    _input = None
    if choices is not None and len(choices) == 1:
        _input = list(choices)[0]
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
        elif choices is not None and len(choices) > 1 and _input.strip() not in choices:
            print(f"Sorry, please define a value out of {choices}")
            _input = None
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


def order_component_model_properties(propname):
    xtype_property_order = ["name", "version", "domain", "type", "designedBy", "projectName", "date", "maturity"]
    if propname in xtype_property_order:
        return str(xtype_property_order.index(propname)).rjust(2,"0")
    return f"{len(xtype_property_order)}_{propname}"


def order_external_reference_properties(propname):
    xtype_property_order = ["name", "description", "remote_url", "vcs_type", "revision_type", "revision_name"]
    if propname in xtype_property_order:
        return str(xtype_property_order.index(propname)).rjust(2,"0")
    return f"{len(xtype_property_order)}_{propname}"


def main(args):
    import argparse
    import xtypes_py
    import xdbi_py
    import datetime
    import os
    import sys
    import os.path as path
    from ..utils import git
    from ..core import Robot
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL
    from ..utils.misc import execute_shell_command

    cm = xtypes_py.ComponentModel()
    er = xtypes_py.ExternalReference()

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('model_directory', type=str, help="The model repository that shall be added to the database", default=os.getcwd())
    parser.add_argument('-n', '--non-interactive', help="The version of the project.", action="store_true", default=False)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"], default=BASE_LOG_LEVEL)

    parser.add_argument('--db_interface', choices=["CLIENT", "SERVERLESS"], help="The type of the database interface", default=None)
    parser.add_argument('--db_path', type=str, help="The url of the database", default=None)
    parser.add_argument('--db_graph', type=str, help="The url of the database", default=None)
    parser.add_argument('--remote', type=str, help='Path to git remote where the model is stored', default=None)

    _default_cm = {}
    _default_er = {}
    for prop in sorted(cm.get_properties().keys(), key=order_component_model_properties):
        kwargs = {
            "default": None,
            "help": f"Value for ComponentModel property {prop}"
        }
        if len(cm.get_allowed_property_values(prop)) == 0:
            kwargs["type"] = str
        else:
            kwargs["choices"] = cm.get_allowed_property_values(prop)
        parser.add_argument(f'--cm_{prop}', **kwargs)
        _default_cm[prop] = cm.get_property(prop)
    for prop in sorted(er.get_properties().keys(), key=order_external_reference_properties):
        kwargs = {
            "default": None,
            "help": f"Value for External Reference property {prop}"
        }
        if len(er.get_allowed_property_values(prop)) == 0:
            kwargs["type"] = str
        else:
            kwargs["choices"] = er.get_allowed_property_values(prop)
        parser.add_argument(f'--er_{prop}', **kwargs)
        _default_er[prop] = er.get_property(prop)

    args = parser.parse_args(args)
    log = setup_logger_level(log_level=args.loglevel)

    # detect default values
    _default_db_interface = "CLIENT"
    _default_db_path = "0.0.0.0:8183"
    _default_db_graph = "hardware"

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

    current_branch = git.get_branch(args.model_directory)

    if not (current_branch.startswith("v") and "." in current_branch):
        log.error("Please create a proper branch for this version. Name style: v0.0.0")
        sys.exit(2)
    git_diff = execute_shell_command("git diff", cwd=args.model_directory, silent=True)
    if not (len(git_diff[0] + git_diff[1]) == 0):
        log.error("Please make sure all your changes are committed and pushed or the unwanted ones stashed.")
        sys.exit(3)

    _default_cm["name"] = robot.name
    _default_cm["version"] = current_branch[1:] if current_branch.startswith("v") and "." in current_branch else "0.0.1"
    _default_cm["date"] = datetime.date.today().isoformat()
    _default_cm["projectName"] = cm.projectName
    _default_cm["domain"] = "ASSEMBLY"
    _default_cm["maturity"] = cm.maturity
    _default_cm["atomic"] = True
    _author, _maintainer, _default_er["remote_url"] = git.get_repo_data(args.model_directory)
    _default_cm["designedBy"] = ", ".join({_author, _maintainer}).replace("[", "").replace("]", "")
    _default_cm["designedBy"] = _default_cm["designedBy"]\
        if len(_default_cm["designedBy"]) > 0 or _default_cm["designedBy"] is not None else cm.designedBy

    _default_er["revision_type"] = "BRANCH"
    _default_er["revision_name"] = git.get_branch(args.model_directory)
    if _default_er["remote_url"].startswith("git@"):
        _default_er["remote_url"] = _default_er["remote_url"].replace(":", "/").replace("//", "/")\
            .replace("git@", "https://")
    _default_er["name"] = "model_repository"
    _default_er["description"] = "This repository contains the SMURF (etc.) simulation and control models."
    _default_er["vcs_type"] = "GIT"

    # run interactive mode
    xdbi = None
    if not args.non_interactive:
        print("How to connect to the database:")
        while xdbi is None:
            if args.db_interface is None:
                args.db_interface = query_var("  db_interface", _default_db_interface,
                                              choices=["CLIENT", "SERVERLESS"]).upper()
            if args.db_path is None:
                args.db_path = query_var("  db_path", _default_db_path)
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
        for k, v in sorted(_default_cm.items(), key=lambda x: order_component_model_properties(x[0])):
            if getattr(args, "cm_"+k) is None:
                setattr(args, "cm_"+k, query_var("  "+k, v, allow_none=v is None,
                                                 choices=cm.get_allowed_property_values(k)))
        print("Define the ExternaleReference properties:")
        for k, v in sorted(_default_er.items(), key=lambda x: order_external_reference_properties(x[0])):
            if getattr(args, "er_"+k) is None:
                setattr(args, "er_"+k, query_var("  "+k, v, allow_none=v is None,
                                                 choices=er.get_allowed_property_values(k)))
    else:
        if args.db_interface.upper() == "CLIENT":
            xdbi = xdbi_py.Client(xtypes_py.ProjectRegistry(), args.db_path)
        elif args.db_interface.upper() == "SERVERLESS":
            xdbi = xdbi_py.Serverless(xtypes_py.ProjectRegistry(), args.db_path)
        else:
            raise AssertionError(f"Can't connect to db with interface {args.db_interface.upper()}")
        xdbi.setWorkingGraph(args.db_graph)
        if not xdbi.isConnected():
            log.error("Couldn't connect to the database!")
            sys.exit(4)

    if _default_cm["version"] != current_branch[1:]:
        log.error("Please go to the branch of this version")
        sys.exit(5)

    # create xtype
    cm.set_properties({
        k: v if getattr(args, "cm_"+k) is None else getattr(args, "cm_"+k) for k, v in _default_cm.items()
    })
    er.set_properties({
        k: v if getattr(args, "er_"+k) is None else getattr(args, "er_"+k) for k, v in _default_er.items()
    })

    cm.annotate_with(er, False)
    xdbi.add([cm])


if __name__ == '__main__':
    import sys
    main(sys.argv)
