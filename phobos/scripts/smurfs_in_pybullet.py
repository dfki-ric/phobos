#!python3


from ..defs import PYBULLET_AVAILABLE


def can_be_used():
    return PYBULLET_AVAILABLE


def cant_be_used_msg():
    return "Pybullet is not available!"


INFO = 'Loads a "smurfs" file in pyBullet.'


def main(args):
    import argparse
    import os
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + os.path.basename(__file__)[:-3])
    parser.add_argument('smurfs', type=str, help='Path to the smurfs file', default="pipeline.yml")
    parser.add_argument('-g', '--add_plane', help='Add a ground plane', action='store_true', default=False)
    parser.add_argument('-i', '--pb_id', type=int,
                        help='pyBullet client id, when this should be parsed to an existing pybullet session',
                        action='store', default=None)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    arguments = parser.parse_args(args)

    log = setup_logger_level(log_level=arguments.loglevel)
    import os
    from copy import deepcopy
    import numpy as np
    import pybullet as pb
    import pybullet_data
    # noinspection SpellCheckingInspection
    from scipy.spatial.transform import Rotation as SciPyRot

    from phobos.defs import load_json

    def parse_smurf(path, basedir):
        smurf_path = os.path.join(basedir, path)
        dir_path = os.path.dirname(smurf_path)
        smurf = load_json(open(smurf_path, "r").read())
        urdf_path = os.path.join(dir_path, [f for f in smurf["files"] if f.lower().endswith("urdf")][0])
        # setting a convention here!!! If there is another URDF with a pb_ prefix we load this instead
        pb_urdf_path = os.path.join(os.path.dirname(urdf_path), "pb_" + os.path.basename(urdf_path))
        if os.path.exists(pb_urdf_path):
            log.info(f"Loading pybullet URDF for {smurf_path}")
            urdf_path = pb_urdf_path
        for f in smurf["files"]:
            if f.lower().endswith(".yml"):
                d = load_json(open(os.path.join(dir_path, f), "r").read())
                for k, v in d.items():
                    smurf[k] = v
        return urdf_path, smurf

    def parse_smurfs(client_id, path, basedir=os.getcwd(), loaded=None):
        if loaded is None:
            loaded = {}
        smurfs_path = os.path.join(basedir, path)
        dir_path = os.path.dirname(smurfs_path)
        smurfs = load_json(open(smurfs_path, "r").read())
        # physics
        if "physics" in smurfs.keys():
            if "gravity" in smurfs["physics"].keys():
                pb.setGravity(
                    smurfs["physics"]["gravity"]["x"],
                    smurfs["physics"]["gravity"]["y"],
                    smurfs["physics"]["gravity"]["z"],
                    physicsClientId=client_id
                )
            physics = {}
            if "bullet" in smurfs["physics"].keys():
                physics = smurfs["physics"]["bullet"]
            elif "ode" in smurfs["physics"].keys():
                physics = smurfs["physics"]["ode"]
            pb_physics_kwargs = {"physicsClientId": client_id}
            if "erp" in physics.keys():
                pb_physics_kwargs["erp"] = float(physics["erp"])
                pb_physics_kwargs["contactERP"] = float(physics["erp"])
                pb_physics_kwargs["frictionERP"] = float(physics["erp"])
            if "cfm" in physics.keys():
                pb_physics_kwargs["globalCFM"] = float(physics["cfm"])
            pb.setPhysicsEngineParameter(**pb_physics_kwargs)
            if "stepsize" in physics.keys():
                pb.setTimeStep(
                    timestep=physics["stepsize"],
                    physicsClientId=client_id
                )
        # entities
        entities = smurfs["smurfs"] if "smurfs" in smurfs.keys() else smurfs["entities"]
        for entity in entities:
            info = None
            if entity["file"].lower().endswith("urdf"):
                file_name = os.path.join(dir_path, entity["file"])
            elif entity["file"].lower().endswith("smurf"):
                file_name, info = parse_smurf(entity["file"], dir_path)
            elif entity["file"].lower().endswith("smurfs"):
                loaded = parse_smurfs(client_id, entity["file"], dir_path, loaded=loaded)
                continue
            elif entity["file"].lower().endswith("smurfa"):
                log.warning("PyBullet can't create a joints between entities. Therefore smurfa is not supported"
                            "if you have only an arrangement of robots use smurfs!")
                continue
            else:
                log.error(f"The entity {entity['file']} has an unknown format and can't be loaded")
                continue
            ori = entity["rotation"] if "rotation" in entity.keys() else [0, 0, 0]
            if type(ori) is not list:
                ori = [0, 0, ori]
            load_dict = {
                "fileName": file_name,
                "physicsClientId": client_id,
                "basePosition": entity["position"] if "position" in entity.keys() else [0, 0, 0],
                "baseOrientation": SciPyRot.from_euler("xyz", ori, degrees=True).as_quat(),
                "flags": pb.URDF_USE_INERTIA_FROM_FILE
            }
            # check if we should use mtl
            urdf = open(file_name, "r").read()
            included_pathes = []
            pos1 = 0
            pos2 = 0
            mtl = False
            while pos2 != -1 and pos1 != -1:
                max_pos1 = pos1
                pos1 = urdf.find("mesh filename", pos2)
                if pos1 < max_pos1:
                    break
                pos1 += len('mesh filename="')
                pos2 = urdf.find('"', pos1)
                included_pathes += [os.path.dirname(urdf[pos1: pos2])]
            included_pathes = list(set(included_pathes))
            for p in included_pathes:
                if any([str(f).lower().endswith("mtl") for f in
                        os.listdir(os.path.join(os.path.dirname(file_name), p))]):
                    mtl = True
                    break
            if mtl:
                log.info(f"Using found mtl files for {entity['file']}")
                load_dict["flags"] |= pb.URDF_USE_MATERIAL_COLORS_FROM_MTL
            # orientation
            if "parent" in entity.keys():
                if entity["parent"] != "world":
                    parent_name, parent_link = entity["parent"].split("::")
                    parent_link_id = None
                    if parent_name not in loaded.keys():
                        log.error(f"The entity {parent_name}  was not loaded before {entity['file']} ."
                                  f" Therefore the latter can't be loaded relatively")
                        continue
                    for j in pb.getJointInfo(bodyUniqueId=loaded[parent_name]["id"], physicsClientId=client_id):
                        if j[12] == parent_link:
                            parent_link_id = j[0]
                    child = np.identity(4)
                    child[0:3, 3] = load_dict["basePostion"]
                    child[0:3, 0:3] = SciPyRot.from_quat(load_dict["baseOrientation"]).as_matrix()
                    parent = np.identity(4)
                    parent[3, 0:3] = pb.getLinkState(loaded[parent_name]["id"], parent_link_id)[0]
                    parent[0:3, 0:3] = SciPyRot.from_quat(
                        pb.getLinkState(loaded[parent_name]["id"], parent_link_id)[1]).as_matrix()
                    target = parent.dot(child)
                    load_dict["basePostion"] = target[0:3, 3]
                    load_dict["baseOrientation"] = target[0:3, 0:3]
            if "anchor" in entity.keys():
                if entity["anchor"] == "world" or entity["anchor"] == "parent" and entity["parent"] == "world":
                    load_dict["useFixedBase"] = 1
                elif entity["anchor"] == "parent" and entity["parent"] != "world":
                    log.warning(f"PyBullet can't create a joint for the parent anchoring in entity {entity['file']} ")
            load_dict["id"] = pb.loadURDF(**load_dict)
            if info is not None:
                load_dict["info"] = info
            if "name" in entity.keys():
                if entity["name"] not in loaded.keys():
                    loaded[entity["name"]] = deepcopy(load_dict)
                else:
                    log.warning(
                        f"The entity name {entity['name']} of {entity['file']} is a duplicate!")
            else:
                log.warning(f"The entity {entity['file']} does not have a name!")
        return loaded

    if arguments.pb_id is None:
        pb_id = pb.connect(pb.GUI)
    else:
        pb_id = arguments.pb_id
    if arguments.add_plane:
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane = pb.loadURDF("plane.urdf", physicsClientId=pb_id)
        pb.changeDynamics(plane, -1, restitution=0.0, physicsClientId=pb_id)
    if arguments.smurfs.lower().endswith("smurfs"):
        parse_smurfs(pb_id, arguments.smurfs)
    else:
        raise argparse.ArgumentTypeError("Can only parse smurfs files!")
    if arguments.pb_id is None:
        while pb.isConnected(pb_id):
            pass


if __name__ == '__main__':
    import sys
    main(sys.argv)
