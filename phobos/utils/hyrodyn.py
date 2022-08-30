import sys
import subprocess
import os

from ..core import Robot
from ..defs import load_json, dump_json, dump_yaml
from ..utils.commandline_logging import get_logger
log = get_logger(__name__)


def get_load_report(urdf_file, submechs):
    proc = subprocess.Popen([
        "python -c 'import hyrodyn; hyrodyn.RobotModel(\"{}\", \"{}\")'".format(urdf_file, submechs)],
        restore_signals=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, cwd=os.getcwd())
    # proc.wait()
    (out, err) = proc.communicate()
    return out.decode(), err.decode(), proc.returncode


def debug_report(report, urdf_file, submech_file, raise_error_failure=False):
    log.info("Trying to load model in hyrodyn:", urdf_file,
          "(file exists)" if os.path.exists(urdf_file) else "(does not exist)", submech_file,
          "(file exists)" if os.path.exists(submech_file) else "(does not exist)",
          flush=True)
    # print(submech_file, open(submech_file, "r").read(), load_json(open(submech_file, "r").read()))
    submech_dict = load_json(open(submech_file, "r").read())
    robot = Robot(xmlfile=urdf_file)
    if report[2] > 0:
        log.info(report[0])
        log.error(report[1])
        jointnames = []
        jointnames_spanningtree = []
        for submech in submech_dict["submechanisms"]:
            jointnames += submech["jointnames"] if "jointnames" in submech.keys() else []
            if "jointnames" in submech.keys():
                for jname in submech["jointnames"]:
                    joint = robot.get_joint(jname)
                    if joint is None:
                        log.info(f"{jname} in {submech['contextual_name']} jointnames is no joint in this robot")
            for x in ["jointnames_active", "jointnames_spanningtree", "jointnames_independent"]:
                jointnames_spanningtree += submech[x]
                for jname in submech[x]:
                    joint = robot.get_joint(jname)
                    if joint is None:
                        log.warning(f'{jname} in {submech["contextual_name"]} {x} is no joint in this robot')
                    elif joint.joint_type == "fixed":
                        log.warning(f'{jname} in {submech["contextual_name"]} {x} is a fixed joint')

        if "exoskeletons" in submech_dict.keys():
            for submech in submech_dict["exoskeletons"]:
                jointnames += submech["jointnames"] if "jointnames" in submech.keys() else []
                jointnames_spanningtree += submech["jointnames_spanningtree"]
        doubles = []
        temp = []
        for j in jointnames:
            if j not in temp:
                temp += [j]
            else:
                doubles += [j]
        doubles = list(set(doubles))
        jointnames = set(jointnames)
        log.info("The following joints are not defined in the submechanisms_file:")
        log.info(sorted(set([j.name for j in robot.joints]) - jointnames))
        log.info("The following joints are defined in the submechanisms_file but are not in the robot:")
        log.info(sorted(jointnames - set([j.name for j in robot.joints])))
        log.info("The following joints are defined multiple times in the submechanisms_file:")
        log.info(sorted(doubles))
        log.info(f"There are {len(jointnames)} joints defined in the submechanisms_file and {len(robot.joints)} in the URDF.")
        log.info("The number of fixed joints defined in the URDF is",
              len([j for j in robot.joints if j.joint_type == "fixed"]))
        if raise_error_failure:
            raise RuntimeError("Hyrodyn aborted!")
        sys.exit(report[2])
