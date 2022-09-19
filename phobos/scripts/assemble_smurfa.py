#!python3
def can_be_used():
    return True


def cant_be_used_msg():
    return "Unknown error!"


INFO = 'Loads a SMURFS/SMURFA file and exports the urdf(s)/smurf of the assembly.'


def main(args):
    import argparse
    import os

    from ..scenes import Scene, Assembly
    from ..defs import BASE_LOG_LEVEL
    from ..utils.commandline_logging import setup_logger_level

    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=str, help="SMURFA or SMURFS file")
    parser.add_argument("output", type=str,  help="The output directory")
    parser.add_argument('-c', '--copy-meshes', help='Copies the meshes', action='store_true', default=False)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)

    log = setup_logger_level(log_level=args.loglevel)
    log.info("Unique names for all links and joints are assumed to create a valid .urdf file!")

    scene = Scene(args.input)
    if scene.is_empty():
        log.error("Given file is empty!")
        sys.exit(1)
    elif scene.has_one_root():
        log.info("Found assembly!")
        assembly = Assembly.from_scene(scene, output_dir=args.output)
        assembly.merge(copy_meshes=args.copy_meshes)
        assembly.robot.name = os.path.basename(args.input).split(".")[0]
        assembly.robot.full_export(output_dir=args.output)
    else:
        log.info("Found Scene!")
        for i, ents in enumerate(scene.entities):
            name = os.path.basename(args.input).split(".")[0] + "_part"+str(i)
            assembly = Assembly.from_entities(ents, output_dir=os.path.join(args.output, name))
            assembly.merge(copy_meshes=args.copy_meshes)
            assembly.robot.name = name
            assembly.robot.full_export(output_dir=args.output)


if __name__ == '__main__':
    import sys
    main(sys.argv)
