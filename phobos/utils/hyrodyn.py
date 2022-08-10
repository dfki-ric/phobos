import sys
import subprocess
import os

from ..core import Robot
from ..defs import load_json, dump_json, dump_yaml


def get_load_report(urdf_file, submechs):
    proc = subprocess.Popen([
        "python -c 'import hyrodyn; hyrodyn.RobotModel(\"{}\", \"{}\")'".format(urdf_file, submechs)],
        restore_signals=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, cwd=os.getcwd())
    # proc.wait()
    (out, err) = proc.communicate()
    return out.decode(), err.decode(), proc.returncode


def debug_report(report, urdf_file, submech_file, raise_error_failure=False):
    print("Trying to load model in hyrodyn:", urdf_file,
          "(file exists)" if os.path.exists(urdf_file) else "(does not exist)", submech_file,
          "(file exists)" if os.path.exists(submech_file) else "(does not exist)",
          flush=True)
    # print(submech_file, open(submech_file, "r").read(), load_json(open(submech_file, "r").read()))
    submech_dict = load_json(open(submech_file, "r").read())
    robot = Robot(xmlfile=urdf_file)
    if report[2] > 0:
        print(report[0], file=sys.stdout, flush=True)
        print(report[1], file=sys.stderr, flush=True)
        jointnames = []
        jointnames_spanningtree = []
        for submech in submech_dict["submechanisms"]:
            jointnames += submech["jointnames"] if "jointnames" in submech.keys() else []
            if "jointnames" in submech.keys():
                for jname in submech["jointnames"]:
                    joint = robot.get_joint(jname)
                    if joint is None:
                        print(jname, "in", submech["contextual_name"], "jointnames",
                              "is no joint in this robot", flush=True, file=sys.stderr)
            for x in ["jointnames_active", "jointnames_spanningtree", "jointnames_independent"]:
                jointnames_spanningtree += submech[x]
                for jname in submech[x]:
                    joint = robot.get_joint(jname)
                    if joint is None:
                        print(jname, "in", submech["contextual_name"], x, "is no joint in this robot",
                              flush=True, file=sys.stderr)
                    elif joint.joint_type == "fixed":
                        print(jname, "in", submech["contextual_name"], x, "is a fixed joint",
                              flush=True, file=sys.stderr)

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
        print("The following joints are not defined in the submechanisms_file:", flush=True, file=sys.stderr)
        print(sorted(set([j.name for j in robot.joints]) - jointnames), flush=True, file=sys.stderr)
        print("The following joints are defined in the submechanisms_file but are not in the robot:",
              flush=True, file=sys.stderr)
        print(sorted(jointnames - set([j.name for j in robot.joints])), flush=True, file=sys.stderr)
        print("The following joints are defined multiple times in the submechanisms_file:",
              flush=True, file=sys.stderr)
        print(sorted(doubles), flush=True, file=sys.stderr)
        print("There are", len(jointnames), "joints defined in the submechanisms_file and",
              len(robot.joints), "in the URDF.", flush=True, file=sys.stderr)
        print("The number of fixed joints defined in the URDF is",
              len([j for j in robot.joints if j.joint_type == "fixed"]), flush=True, file=sys.stderr)
        if raise_error_failure:
            raise RuntimeError("Hyrodyn aborted!")
        sys.exit(report[2])
