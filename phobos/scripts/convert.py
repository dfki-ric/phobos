#!python3


def can_be_used():
    return True


def cant_be_used_msg():
    return ""


INFO = 'Converts the given input robot file to SDF/URDF/PDF/THUMBNAIL/SMURF/SMURFA/SMURFS.'


def main(args):
    import argparse
    import os.path as path
    from ..defs import load_json
    from ..core import Robot, World
    from ..utils import misc, resources
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + path.basename(__file__)[:-3])
    parser.add_argument('input', type=str, help='Input specification: A path to a file (with extension [smurfa, smurfs, smurf, urdf, sdf]) or directory')
    parser.add_argument('output', type=str, help='Output specification: A path to a file (with extension [smurfa, smurfs, smurf, urdf, sdf]) or directory',
                        action="store", default=None)
    parser.add_argument('-c', '--copy-meshes', help='Copies the meshes', action='store_true', default=False)
    parser.add_argument('-m', '--mesh-type', choices=["stl", "obj", "mars_obj", "dae", "input_type", "glb"], help='If -c set the mesh type', default="input_type")
    parser.add_argument('-a', '--sdf-assemble', help='When converting a scene to sdf set this flag to get one assembled model', action='store_true', default=False)
    parser.add_argument('-e', '--export_config', type=str, help='Path to the a json/yaml file that stores the export config', default=None)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)
    log = setup_logger_level(log_level=args.loglevel)

    if args.input == args.output:
        return 0
    if args.output.endswith("/"):
        args.output = args.output[:-1]

    # Detect input format
    input_format = None
    input_split = args.input.lower().rsplit(".",1)
    if len(input_split) < 1:
        log.error(f"Invalid input spec {args.input}")
        return 2
    elif len(input_split) == 1:
        input_format = "DIRECTORY"
    elif input_split[-1] in ["smurfa"]:
        input_format = "SMURFA"
    elif input_split[-1] in ["smurfs"]:
        input_format = "SMURFS"
    elif input_split[-1] in ["smurf"]:
        input_format = "SMURF"
    elif input_split[-1] in ["urdf"]:
        input_format = "URDF"
    elif input_split[-1] in ["sdf"]:
        input_format = "SDF"
    else:
        log.error(f"Could not determine input format from {args.input}")
        return 3

    # Detect output format
    output_format = None
    output_split = args.output.lower().rsplit(".",1)
    if len(output_split) < 1:
        log.error(f"Invalid output spec {args.output}")
        return 4
    elif len(output_split) == 1 or path.isdir(args.output):
        output_format = "DIRECTORY"
    elif output_split[-1] in ["smurfa"]:
        output_format = "SMURFA"
    elif output_split[-1] in ["smurfs"]:
        output_format = "SMURFS"
    elif output_split[-1] in ["smurf"]:
        output_format = "SMURF"
    elif output_split[-1] in ["urdf"]:
        output_format = "URDF"
    elif output_split[-1] in ["sdf"]:
        output_format = "SDF"
    elif output_split[-1] in ["pdf"]:
        output_format = "PDF"
    elif output_split[-1] in ["jpg"]:
        output_format = "IMAGE"
    elif output_split[-1] in ["png"]:
        output_format = "IMAGE"
    else:
        log.error(f"Could not determine output format from {args.output}")
        return 5

    # Check if input/output format combination is valid and choose corresponding procedure
    if input_format == output_format:
        log.info(f"Input and output format are equal. Will do nothing :)")
        return 0

    # If input is either smurfa or smurfs we have to assemble a robot first
    world = None
    robot = None
    if input_format in ["SMURFA", "SMURFS"]:
        world = World(inputfile=args.input)
        if world.is_empty():
            log.error("World/Arrangement is empty")
            return 1
        if world.has_one_root():
            log.info(f"Assembling robot from World/Arrangement")
            robot = world.assemble()
    else:
        robot = Robot(inputfile=args.input)

    # Check results
    if not world and not robot:
        log.error("You have specified something which could not be converted to neither a robot nor a world/arrangement")
        return 6

    if args.copy_meshes and robot and not args.export_config:
        robot.export_meshes(path.join(path.dirname(args.output), "meshes"), format=args.mesh_type)
    if args.export_config:
        assert path.isfile(args.export_config)
        with open(args.export_config, "r") as f:
            export_config = load_json(f.read())
        robot.export(args.output, export_config=export_config["export_config"], with_meshes=args.copy_meshes, no_smurf=["no_smurf"])
    elif output_format in ["URDF"]:
        log.info("Converting to URDF")
        if robot:
            robot.export_urdf(outputfile=args.output, mesh_format=args.mesh_type)
        else:
            raise NotImplementedError("Cannot export an URDF from World/Arrangement which cannot be assembled")
    elif output_format in ["SDF"]:
        if args.sdf_assemble:  # world
            log.info("Converting to SDF model")
            robot.export_sdf(outputfile=args.output, mesh_format=args.mesh_type)
        else:  # world
            # FIXME: This is actually not implemented. Will raise an Exception
            log.info("Converting to SDF world")
            world.export_sdf(outputfile=args.output, mesh_format=args.mesh_type)
    elif output_format in ["PDF"]:
        log.info("Converting to PDF")
        if robot:
            robot.export_pdf(outputfile=args.output)
        else:
            raise NotImplementedError("Can't yet create PDF for world representations")
    elif output_format in ["IMAGE"]:
        log.info("Converting to thumbnail")
        if robot:
            size=512
            robotfile = robot.smurffile
            if not robotfile:
                robotfile = robot.xmlfile
            if not robotfile:
                log.error("Robot does not have a compatible specification file for thumbnail creation")
            im = misc.get_thumbnail(robotfile, icon_size=size)
            misc.make_icon(im, args.output, size=size)
        else:
            raise NotImplementedError("Can't yet export thumbnails for world representations")
    elif output_format in ["SMURF"]:
        log.info("Converting to SMURF file inside a smurf directory (the file might be named differently than specified!)")
        if robot:
            robot.export_smurf(outputfile=args.output, mesh_format=args.mesh_type)
        else:
            raise NotImplementedError("Cannot export a SMURF from World/Arrangement which cannot be assembled")
    elif output_format in ["DIRECTORY"]:
        log.info("Converting to directory (will export multiple subformats in the appropriate subdirs)")
        if robot:
            robot.export(outputdir=args.output, export_config=resources.get_default_export_config())
        else:
            for root in world.get_root_entities():
                robot = world.assemble(root)
                robot.export(outputdir=path.join(args.output, f"root-{root.name}"), export_config=getattr(world, "export_config", resources.get_default_export_config()))
    elif output_format in ["SMURFA"]:
        log.info("Converting to SMURF Arrangement")
        if robot:
            world = World()
            world.add_robot(name="robot", robot=robot, anchor="world")
        world.export_smurfa(outputfile=args.output)
    elif output_format in ["SMURFS"]:
        log.info("Converting to SMURF Scene")
        if robot:
            world = World()
            world.add_robot(name="robot", robot=robot, anchor="world")
        world.export_smurfs(outputfile=args.output)
    else:
        log.error(f"Unknown conversion operation from {args.input} {input_format} to {args.output} {output_format}")
        return 7

    log.info("Done")
    return 0

if __name__ == '__main__':
    import sys

    main(sys.argv)
