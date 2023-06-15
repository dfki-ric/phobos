#!python3

def can_be_used():
    return True


def cant_be_used_msg():
    return ""


INFO = 'Checks whether all meshes are available.'


def main(args):
    print("\n--> Checking meshes!")
    import argparse
    import os.path as path
    from copy import deepcopy
    from ..core.robot import Robot
    from ..io import representation
    from ..defs import dump_json
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('robot_file', type=str, help='Path to the urdf or smurf file')
    parser.add_argument('-o', '--output', type=str, help='Writes the result as YAML to the given file', action="store",
                        default=None)
    parser.add_argument('-a', '--all', help='Writes everything not only issues', action="store_true", default=False)
    parser.add_argument('-w', '--warn', help='Show warnings', action="store_true", default=False)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)
    log = setup_logger_level(log_level=args.loglevel)

    robot = Robot(inputfile=args.robot_file)

    report = {}
    n_errors = 0
    n_warnings = 0

    for link in robot.links:
        link_report = {}
        link_issues = False
        for geo in link.collisions + link.visuals:
            if isinstance(geo.geometry, representation.Mesh):
                mesh_path = geo.geometry.filepath
                geo_report = {"path": mesh_path, "found": True, "error": False, "warning": False, "note": ""}
                f = open(path.realpath(mesh_path), "rb")
                if not path.isfile(path.realpath(mesh_path)):
                    geo_report["found"] = False
                    geo_report["error"] = True
                    geo_report["note"] = "Mesh not found."
                elif b'\0' not in f.read():
                    file = open(path.realpath(mesh_path), "r").read()
                    if len(file.split("\n")) == 3 and file.startswith("version"):
                        geo_report["found"] = False
                        geo_report["error"] = True
                        geo_report["note"] += "LFS not properly checked out. "
                    else:
                        geo_report["found"] = True
                        geo_report["note"] += "Mesh found. "
                f.close()
                if not mesh_path.lower().endswith("bobj"):
                    suffix_idx = mesh_path.rfind(".")
                    suffix = mesh_path[suffix_idx + 1:]
                    bobj_path = None
                    if path.isfile(mesh_path[:suffix_idx] + ".bobj"):
                        bobj_path = mesh_path[:suffix_idx] + ".bobj"
                    mesh_path_pre = path.dirname(path.dirname(mesh_path))
                    mesh_dir = path.basename(path.dirname(mesh_path))
                    mesh = path.basename(mesh_path)
                    if "mars_obj" in mesh_dir:
                        alt_mesh_path = path.join(mesh_path_pre, mesh_dir.replace("mars_obj", "bobj"),
                                                  mesh[:-len(suffix)] + "bobj")
                    else:
                        alt_mesh_path = path.join(mesh_path_pre, mesh_dir.replace(suffix, "bobj"),
                                                  mesh[:-len(suffix)] + "bobj")
                    if path.isfile(alt_mesh_path):
                        bobj_path = alt_mesh_path
                    if bobj_path is not None and b'\0' not in open(path.realpath(bobj_path), "rb").read():
                        file = open(path.realpath(mesh_path), "r").read()
                        if len(file.split("\n")) == 3 and file.startswith("version"):
                            geo_report["error"] = True
                            geo_report["note"] += "Found bobj, but LFS was not properly checked out. "
                        else:
                            geo_report["error"] = True
                            geo_report["note"] += "Found bobj, but is invalid file. "
                    elif bobj_path is not None:
                        geo_report["note"] += "bobj available."
                    else:
                        geo_report["warning"] = True
                        geo_report["note"] += "bobj not available. "
                if geo_report["error"]:
                    n_errors += 1
                if geo_report["warning"]:
                    n_warnings += 1
                link_issues |= (args.warn and geo_report["warning"]) or geo_report["error"]
                link_report[geo.name] = deepcopy(geo_report)
        report[link.name] = {"report": deepcopy(link_report), "issues": deepcopy(link_issues)}

    if args.output is not None:
        if path.exists(args.output):
            log.error(f"Won't overwrite {args.output}")
            return 1
        with open(args.output, "w") as f:
            f.write(dump_json(report))
        log.info(f"Wrote full output to {args.output}")
    else:
        for link, lr in report.items():
            if lr["issues"] or (args.all and len(lr["report"].items()) > 0):
                log.info(f"{link} :")
                for geo, gr in lr["report"].items():
                    if gr["error"] or (args.warn and gr["warning"]) or args.all:
                        note = gr["note"]
                        path_string = gr["path"]
                        log.info(f" {geo} : {note}")
                        log.info(f"    {path_string}")

    log.info(f"{n_errors} mesh errors and {n_warnings} warnings found!")
    if not args.warn and n_warnings != 0:
        log.info("To display warnings add -w option.")
    return 0


if __name__ == '__main__':
    import sys
    main(sys.argv)
