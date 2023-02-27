#!python3


def can_be_used():
    return True


def cant_be_used_msg():
    return "Unknown error!"


INFO = 'Preprocess CAD->URDF exports to use with pipeline.'


def main(args):
    import argparse
    import os
    import pkg_resources

    from phobos.defs import EULER_CONVENTION, RPY_CONVENTION
    from phobos.utils.git import get_repo_data
    from phobos.utils.misc import create_dir, copy, regex_replace
    from phobos.utils.transform import order_angles
    from phobos.defs import load_json
    from phobos.core import Robot
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    def invertJoint(urdfpath, name):
        try:
            import xml.etree.ElementTree as ET
            import numpy as np
            from scipy.spatial.transform import Rotation as Rot

            try:
                tree = ET.parse(urdfpath)
            except ET.ParseError as error:
                log.error(f"Reading of URDF file {urdfpath} failed!")
                log.error("If this is a yaml please check if you wanted to give a basefile or a derived_base!")
                raise error
            root = tree.getroot()

            done = False
            for child in root:
                if child.tag == "joint" and child.attrib["name"] == name:
                    done = True
                    for cchild in child:
                        if cchild.tag == "origin":
                            trans = np.array([float(x) for x in cchild.attrib["xyz"].split()])
                            rot = Rot.from_euler(EULER_CONVENTION,
                                                 np.array([float(x) for x in cchild.attrib["rpy"].split()])).as_matrix()
                            T = np.eye(4)
                            T[:3, 3] = trans
                            T[:3, :3] = rot
                            T = np.linalg.inv(T)
                            trans = T[:3, 3]
                            rot = order_angles(Rot.from_matrix(T[:3, :3]).as_euler(EULER_CONVENTION),
                                               EULER_CONVENTION, RPY_CONVENTION)
                            cchild.attrib["xyz"] = " ".join([str(x) for x in trans])
                            cchild.attrib["rpy"] = " ".join([str(x) for x in rot])
                        elif cchild.tag == "parent":
                            for cchild2 in child:
                                if cchild2.tag == "child":
                                    # print("Swapping", cchild2.attrib["link"], cchild.attrib["link"])
                                    temp = cchild2.attrib["link"]
                                    cchild2.attrib["link"] = cchild.attrib["link"]
                                    cchild.attrib["link"] = temp

            tree.write(urdfpath)
            if not done:
                log.warning(f"Couldn't find joint {args.invertJoint}")
        except ImportError:
            log.error("Couldn't import required modules! Can't invert joint")

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + os.path.basename(__file__)[:-3])
    parser.add_argument('input_directory', type=str, help='Path to the freshly exported CAD model')
    parser.add_argument('output_directory', type=str, help='Path to the preprocessed model')
    parser.add_argument('-m', '--no-mesh-simplification', dest="meshes", help='Do not simplify meshes with meshlab',
                        action='store_false', default=True)
    parser.add_argument('-i', '--invert-joint', dest="invertJoint", type=str,
                        help='Invert the joint with the given name', default=None)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)

    log = setup_logger_level(log_level=args.loglevel)

    if os.path.exists(args.input_directory) and args.output_directory:
        log.info("Preprocessing:")
        # create output
        create_dir(None, args.output_directory)
        create_dir(None, os.path.join(args.output_directory, "urdf"))

        # reduce meshes
        if args.meshes:
            try:
                log.info("  Reducing meshes ...")
                import meshlabxml as mlx
                in_dir = os.path.join(args.input_directory, "meshes")
                out_dir = os.path.join(args.output_directory, "meshes")
                create_dir(None, out_dir)
                for mesh in os.listdir(in_dir):
                    in_mesh = os.path.join(in_dir, mesh)
                    out_mesh = os.path.join(out_dir, mesh)
                    simplify_mesh = mlx.FilterScript(
                        file_in=in_mesh,
                        file_out=out_mesh,
                        ml_version='2018.12', )
                    simplify_mesh.run_script(
                        script_file=pkg_resources.resource_filename(__name__, "data/simplify_stl.mlx"))
                    # os.system("meshlabserver -i "+in_mesh+" -o "+out_mesh+" -s "+cfg_file)
            except Exception as e:
                log.warning(f"Failed to reduce mesh as meshxml: {e}")
                # copy meshes
                copy(None, os.path.join(args.input_directory, "meshes"), os.path.join(args.output_directory))
        else:
            # copy meshes
            copy(None, os.path.join(args.input_directory, "meshes"), os.path.join(args.output_directory))

        # Replace package
        urdf_name = [f for f in os.listdir(os.path.join(args.input_directory, "urdf")) if f.endswith("urdf")]
        urdf_path = os.path.join(args.output_directory, "urdf", "model.urdf")
        with open(os.path.join(args.input_directory, "urdf", urdf_name[0]), "r") as input_file:
            content = input_file.read()
            log.info("  Replacing meshes pathes and xml version ...")
            content = regex_replace(content, {
                "package://.*/meshes": "../meshes",
                "<\?xml version.*\?>\n": ""
            })
            if os.path.exists(os.path.join(args.input_directory, "replacements.yml")):
                log.info("  Replacing strings according to replacements.yml")
                copy(None, os.path.join(args.input_directory, "replacements.yml"),
                     os.path.join(args.output_directory, "replacements.yml"))
                replacements = load_json(open(os.path.join(args.input_directory, "replacements.yml"), "r").read())
                content = regex_replace(content, replacements)
                for file in [f for f in os.listdir(os.path.join(args.output_directory, "meshes")) if
                             os.path.isfile(os.path.join(args.output_directory, "meshes", f))]:
                    file_new = regex_replace(file, replacements)
                    if file != file_new:
                        log.info(f"Renaming {file} to {file_new}")
                        os.rename(os.path.join(args.output_directory, "meshes", file),
                                  os.path.join(args.output_directory, "meshes", file_new))
            log.info("  Writing updated URDF to model.urdf ...")
            with open(os.path.join(args.output_directory, "urdf", "model.urdf"), "w") as output:
                output.write(content)

        if args.invertJoint is not None:
            log.info(f"  Inverting {Jointargs.invertJoint} ...")
            invertJoint(urdf_path, args.invertJoint)

        if os.path.basename(os.path.dirname(__file__)) == "ci-run":
            readme_path = pkg_resources.resource_filename(__name__, "data/README-for-input-repo.md.in")
            manifest_path = pkg_resources.resource_filename(__name__, "data/manifest.xml.in")
            if not os.path.exists(os.path.join(args.output_directory, "README.md")):
                copy(None, readme_path, os.path.join(args.output_directory, "README.md"))
            if not os.path.exists(os.path.join(args.output_directory, "manifest.xml")):
                copy(None, manifest_path, os.path.join(args.output_directory, "manifest.xml"))
        #     elif "AUTOPROJ_CURRENT_ROOT" in os.environ:
        #         readme_path = os.path.join(
        #             os.environ["AUTOPROJ_CURRENT_ROOT"],
        #             "models/phobos/phobos/phobos/data/README-for-input-repo.md.in")
        #         manifest_path = os.path.join(
        #             os.environ["AUTOPROJ_CURRENT_ROOT"],
        #             "models/phobos/phobos/phobos/data/manifest-for-input-repo.xml.in")
        #         if not os.path.exists(os.path.join(args.output_directory, "README.md")):
        #             copy(readme_path, os.path.join(args.output_directory, "README.md"))
        #         if not os.path.exists(os.path.join(args.output_directory, "manifest.xml")):
        #             copy(manifest_path, os.path.join(args.output_directory, "manifest.xml"))
        else:
            log.warning("Couldn't insert default README.md file, remember updating this in your input repo!")

        if os.path.exists(os.path.join(args.output_directory, "README.md")):
            with open(os.path.join(args.output_directory, "README.md"), "r") as readme:
                content = readme.read()
                content = regex_replace(content, {
                    "\$INPUTNAME": os.path.basename(os.path.abspath(args.output_directory))
                })
                log.debug("  Replacing $INPUTNAME with" +
                      str(os.path.basename(os.path.abspath(args.output_directory))) + "in README.md ...")
            with open(os.path.join(args.output_directory, "README.md"), "w") as readme:
                readme.write(content)

        if os.path.exists(os.path.join(args.output_directory, "manifest.xml")):
            with open(os.path.join(args.output_directory, "manifest.xml"), "r") as manifest:
                content = manifest.read()
                author, maintainer, url = get_repo_data(os.path.abspath(args.output_directory))
                author = regex_replace(author, {"\<": "", "\>": ""})
                # print("Author:", author, "\nMaintainer:", maintainer, "\nURL:", url)
                content = regex_replace(content, {
                    "\$INPUTNAME": os.path.basename(os.path.abspath(args.output_directory)),
                    "\$AUTHOR": "<author>" + author + "</author>" if author != "" else "",
                    "\$MAINTAINER": "<maintainer>" + maintainer + "</maintainer>" if maintainer != "" else "",
                    "\$URL": "<url>" + url + "</url>" if url != "" else "",
                })
                log.info(f"  Inserting contents of manifest.xml in {os.path.basename(os.path.abspath(args.output_directory))} ...")
            with open(os.path.join(args.output_directory, "manifest.xml"), "w") as manifest:
                manifest.write(content)

        try:
            log.info("  Trying to check URDF using urdfdom...")
            os.system("check_urdf " + urdf_path)
        except:
            pass

        robot = Robot(inputfile=urdf_path)
        robot.export_pdf(outputfile=urdf_path[:-4]+"pdf")

        log.info("Finished!")

        try:
            print("Trying to load model in pybullet... (Press CTRL+C to interrupt)")

            import time
            time.sleep(5)

            import pybullet as pb

            # execute our now mute function
            pb.connect(pb.GUI)
            pb.loadURDF(urdf_path)
            while pb.isConnected(0):
                pass
        except:
            pass
    else:
        log.error("Input directory does not exist!")
        parser.print_help()


if __name__ == '__main__':
    import sys

    main(sys.argv)
