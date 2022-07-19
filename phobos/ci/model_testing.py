import sys
from copy import deepcopy
import numpy as np
import yaml
import subprocess

from ..defs import *
from ..core import Robot
from ..io import representation
from ..utils import urdf
from ..utils.hyrodyn import get_load_report, debug_report
from ..utils.misc import execute_shell_command, list_files


class ModelTest(object):
    """Checks the general validity of a new model possibly against an old model.
    """

    def __init__(self, new_model, old_robot=None):
        # Check for the module
        self.new = new_model
        self.old = old_robot
        self.new_hyrodyn = None
        self.old_hyrodyn = None
        self.new_hml_test = self._hyrodyn_load_test(self.new.urdf, self.new.submechanisms_file_path)
        self.new_fb_hml_test = None
        self.new_sm_hml_test = []
        if hasattr(self.new, "export_floatingbase") and self.new.floatingbase is True:
            self.new_fb_hml_test = self._hyrodyn_load_test(self.new.urdf[:-5]+"_floatingbase.urdf",
                                                           self.new.floatingbase_submechanisms_file_path)
        if hasattr(self.new, "export_submodels"):
            sm_path = os.path.join(self.new.exportdir, "submodels")
            for au in self.new.export_submodels:
                if "only_urdf" in au.keys() and au["only_urdf"] is True:
                    continue
                self.new_sm_hml_test += [{
                    "name": au["name"],
                    "urdf": os.path.join(sm_path, au["name"], "urdf", au["name"] + ".urdf"),
                    "submech": os.path.join(sm_path, au["name"], "smurf", au["name"] + "_submechanisms.yml")
                }]
                self.new_sm_hml_test[-1]["report"] = get_load_report(
                    self.new_sm_hml_test[-1]["urdf"],
                    self.new_sm_hml_test[-1]["submech"]
                )
        self.old_hml_test = None
        if self.old is not None:
            self.old_hml_test = get_load_report(self.old.xmlfile, self.old.submechanisms_path)

    def _load_old_hyrodyn_model(self):
        if self.old is None:
            return True
        if self.old_hml_test[2]:
            return False
        if self.old_hyrodyn is None and HYRODYN_AVAILABLE:
            try:
                print(
                    "Trying to load old model in hyrodyn:",
                    self.old.xmlfile,
                    "(file exists)" if os.path.exists(self.old.xmlfile) else "(does not exist)",
                    os.path.join(self.old.submechanisms_path),
                    "(file exists)" if os.path.exists(os.path.join(self.old.submechanisms_path))
                    else "(does not exist)",
                    flush=True
                )
                if self.old_hml_test[2] > 0:
                    # print(self.old_hml_test[0], file=sys.stdout)
                    # print(self.old_hml_test[1], file=sys.stderr)
                    raise Exception("Hyrodyn aborted!")
                self.old_hyrodyn = hyrodyn.RobotModel(
                    self.old.xmlfile,
                    self.old.submechanisms_path
                )
                print("Old model loaded!", flush=True)
                return True
            except Exception as e:
                print("Failed to load old model in hyrodyn:", flush=True)
                print(e, flush=True)
                return False

    # info procedures
    def info_swing_my_robot(self):
        if not (hasattr(self.new, "swing_my_robot") and self.new.swing_my_robot is True):
            return
        print("Running swing_my_robot", flush=True)
        if not HYRODYN_AVAILABLE:
            print('WARNING: Info procedure swing_my_robot not possible: Hyrodyn not present',
                  flush=True, file=sys.stderr)
            return
        submech_file = self.new.submechanisms_file_path
        cmd = "swing_my_robot.py"
        if self.new.submechanisms_path is not None and os.path.exists(self.new.submechanisms_path):
            print("Submechs:", submech_file, "exists!" if os.path.isfile(submech_file) else "does not exist!")
            cmd += " --submechanism_yml " + submech_file
        limits_file = os.path.join(self.new.modeldir, "submechanisms/joint_limits.yml")
        print("Limits:", limits_file, "exists!" if os.path.isfile(limits_file) else "does not exist!")
        if os.path.isfile(limits_file):
            cmd += " --joint_limits_yml " + limits_file
        print("URDF:", self.new.robot.xmlfile,
              "exists!" if os.path.isfile(self.new.robot.xmlfile) else "does not exist!")
        # cmd += " --export_animation"
        cmd += " --export_animation_as_mp4"
        cmd += " " + self.new.robot.xmlfile
        cmd += " --out " + os.path.join(self.new.modeldir, "swing-anim_"+self.new.modelname+".gif")
        cmd += "|| true"  # make this fail safe
        try:
            execute_shell_command(cmd, os.getcwd())
        except ImportError as e:
            print("Can't run swing_my_robot procedure because of missing library:\n", e)
            return
        except AssertionError as e:
            print("Swing_my_robot.py failed due to an assertion error:", e)
            return
        except:
            try:
                execute_shell_command("python " + os.path.join(os.environ["AUTOPROJ_CURRENT_ROOT"],
                                                               "control/hyrodyn/python/") + cmd, os.getcwd())
            except ImportError as e:
                print("Can't run swing_my_robot procedure because of missing library:\n", e)
                return
            except AssertionError as e:
                print("Swing_my_robot.py failed due to an assertion error:", e)
                return
            except Exception as e:
                print("Swing_my_robot.py failed due to an unknown error", e)

    # test procedures
    def test_process_double_check(self):
        success = True
        # have joints been removed?
        if hasattr(self.new, "remove_joints"):
            joint_names = [joint.name for joint in self.new.robot.joints]
            for joint_name in self.new.remove_joints:
                success &= joint_name not in joint_names
        # TODO check the others
        # check if meshes are there
        for link in self.new.robot.links:
            for mesh in [c.geometry.filename for c in link.collisions if hasattr(c.geometry, "filename")] +\
                        [c.geometry.filename for c in link.visuals if hasattr(c.geometry, "filename")]:
                mesh_path = urdf.read_urdf_filename(mesh, self.new.robot.xmlfile)
                if os.path.isfile(mesh_path):
                    success &= True
                else:
                    print("Mesh", mesh_path, "does not exist!")
                    success = False
        return success

    def test_compare_link_masses(self):
        success = True
        print("Masses of Links:", flush=True)
        new_link_masses = {
            link.name: (link.inertial.mass if link.inertial is not None else 0) for link in self.new.robot.links
        }
        if self.old is not None:
            old_link_masses = {
                link.name: (link.inertial.mass if link.inertial is not None else 0) for link in self.old.links
            }
        else:
            print("Old model not present! Skipping test!", flush=True)
            return "skipped (no model to compare)"
        max_length = max(*[len(n) for n in new_link_masses.keys()])
        for k in new_link_masses.keys():
            link_name = k + (max_length - len(k)) * " "
            if k not in old_link_masses.keys():
                print("%s New: %8.4e\t Old: non-existent" % (link_name, new_link_masses[k]), flush=True)
                continue
            print("%s New: %8.4e\t Old: %8.4e\t Diff: %8.4e"
                  % (link_name, new_link_masses[k], old_link_masses[k], new_link_masses[k] - old_link_masses[k]),
                  end="", flush=True)
            if abs(new_link_masses[k] - old_link_masses[k]) > self.new.tolerances["tolerance_mass"]:
                print(" too big!", flush=True)
                success = False
            else:
                print("", flush=True)
        return success

    def test_symmetry_check_masses(self, left_right_end_effectors):
        for ee in left_right_end_effectors:
            if self.new.robot.get_link_id(ee) is None:
                print("Existing links:", [link.name for link in self.new.robot.links])
                raise AssertionError(ee + " Link does not exist in the newly exported model!")

        masses = {}
        for link in self.new.robot.links:
            masses[link.name] = link.inertial.mass if hasattr(link, "inertial") and link.inertial is not None else 0
        left = self.new.robot.get_chain(self.new.robot.get_root(), left_right_end_effectors[0],
                                        links=True, joints=False, fixed=True)
        right = self.new.robot.get_chain(self.new.robot.get_root(), left_right_end_effectors[1],
                                         links=True, joints=False, fixed=True)
        assert len(left) == len(right)
        print("Symmetry Check Masses:", flush=True)
        success = True
        max_len = np.array([len(left[i] + "/" + right[i]) for i in range(len(left))]).max(initial=0)
        for i in range(len(left)):
            left_mass = masses[left[i]]
            right_mass = masses[right[i]]
            diff = left_mass - right_mass
            print("{}\t{}\t{}\t{}\t{}".format(
                left[i] + "/" + right[i] + " " * (max_len - (len(left[i] + "/" + right[i]))),
                "{:f}".format(left_mass),
                "{:f}".format(right_mass),
                "{:f}".format(diff),
                "!!!" if diff is None or np.abs(diff) > self.new.tolerances["tolerance_mass"] or np.isnan(diff) else ""
            ), flush=True)
            success &= not (np.abs(diff) > self.new.tolerances["tolerance_mass"] or np.isnan(diff))
        print("  Success?", success, flush=True)
        return success

    def test_compare_link_transformations(self):
        success = True
        print("Transformation changes of Links:", flush=True)
        root2new_links = {link.name: self.new.robot.get_transformation(link.name) for link in self.new.robot.links}
        if self.old is not None:
            root2old_links = {link.name: self.old.get_transformation(link.name) for link in self.old.links}
        else:
            print("Old model not present! Skipping test!", flush=True)
            return "skipped (no model to compare)"
        max_length = max(*[len(n) for n in root2new_links.keys()])
        for k in root2new_links.keys():
            link_name = k + (max_length - len(k)) * " "
            if k not in root2old_links.keys():
                print("%s doesn't exist in compare model" % link_name, flush=True)
                print("root2link: xyz: %s \t rpy:%s" % (repr(representation.Pose.from_matrix(root2new_links[k]).xyz.tolist()),
                                                        repr(representation.Pose.from_matrix(root2new_links[k]).rpy.tolist())),
                      flush=True)
                continue
            diff = np.linalg.inv(root2old_links[k]).dot(root2new_links[k])
            diff_o = representation.Pose.from_matrix(diff)
            print("%s Difference: xyz: %s\trpy: %s" % (link_name, repr(diff_o.xyz.tolist()), repr(diff_o.rpy.tolist())),
                  end="", flush=True)
            if np.linalg.norm(diff[0:3, 3]) > self.new.tolerances["tolerance_distance"] or \
               any(abs(diff_o.rpy) > [self.new.tolerances["tolerance_rad"]]*3):
                if np.linalg.norm(diff[0:3, 3]) > self.new.tolerances["tolerance_distance"]:
                    print(np.linalg.norm(diff[0:3, 3]), ">", self.new.tolerances["tolerance_distance"],
                          end="", flush=True)
                if any(abs(diff_o.rpy) > [self.new.tolerances["tolerance_rad"]]*3):
                    print(abs(diff_o.rpy), ">", [self.new.tolerances["tolerance_rad"]]*3, end="", flush=True)
                print(" !!!", flush=True)
                # print("Difference as Transformation Matrix from old to new:", flush=True)
                # print(diff, flush=True)
                success = False
            else:
                print("", flush=True)
        return success

    def test_topological_self_consistency(self):
        if self.old is None:
            print("Old model not present! Skipping test!", flush=True)
            return "skipped (no model to compare)"
        # The new model is allowed to have more joints/links than the previous
        print("New model contains:", flush=True)
        print("Links", sorted(
            set([link.name for link in self.new.robot.links])
            - set([link.name for link in self.old.links])
        ), flush=True)
        print("Coll", sorted(
            set([collision.name for link in self.new.robot.links for collision in link.collisions])
            - set([collision.name for link in self.old.links for collision in link.collisions])
        ), flush=True)
        print("Vis", sorted(
            set([visual.name for link in self.new.robot.links for visual in link.visuals])
            - set([visual.name for link in self.old.links for visual in link.visuals])
        ), flush=True)
        print("Joints", sorted(
            set([joint.name for joint in self.new.robot.joints])
            - set([joint.name for joint in self.old.joints])
        ), flush=True)
        changed_joint_types1 = set([joint.name + ":" + joint.joint_type for joint in self.new.robot.joints]) - \
                               set([joint.name + ":" + joint.joint_type for joint in self.old.joints])
        print("JTypes", sorted(changed_joint_types1), flush=True)
        print("but not:", flush=True)
        removed_links = set([link.name for link in self.old.links]) - set([link.name for link in self.new.robot.links])
        print("Links", sorted(removed_links), flush=True)
        print("Coll", sorted(
            set([collision.name for link in self.old.links for collision in link.collisions])
            - set([collision.name for link in self.new.robot.links for collision in link.collisions])
        ), flush=True)
        print("Vis", sorted(
            set([visual.name for link in self.old.links for visual in link.visuals])
            - set([visual.name for link in self.new.robot.links for visual in link.visuals])
        ), flush=True)
        removed_joints = set([joint.name for joint in self.old.joints]) - \
                         set([joint.name for joint in self.new.robot.joints])
        print("Joints", sorted(removed_joints), flush=True)
        changed_joint_types2 = set([joint.name + ":" + joint.joint_type for joint in self.old.joints]) - \
                               set([joint.name + ":" + joint.joint_type for joint in self.new.robot.joints])
        print("JTypes", sorted(changed_joint_types2), flush=True)
        return len(removed_links) + len(removed_joints) + len(changed_joint_types1) + len(changed_joint_types2) == 0

    def test_compare_amount_joints(self):
        if self.old is None:
            print("Old model not present! Skipping test!", flush=True)
            return "skipped (no model to compare)"
        print("New Model has", len(self.new.robot.joints), "and old model has", len(self.old.joints))
        return len(self.new.robot.joints) == len(self.old.joints)

    def test_load_in_pybullet(self):
        if not PYBULLET_AVAILABLE:
            print('Pybullet not present', flush=True)
            return True
        try:
            client = pb.connect(pb.DIRECT)
            pb.loadURDF(os.path.join(self.new.urdf), physicsClientId=client)
            pb.disconnect(client)
            return True
        except Exception as e:
            print("Failed: pyBullet check failed!\n", e, flush=True)
            return False

    def test_file_consistency(self):
        if self.old is None:
            print("Old model not present! Skipping test!", flush=True)
            return "skipped (no model to compare)"
        new_files = list_files(self.new.exportdir, ignore=["\.gv", "\.pdf", "\.git*", "README\.md", "manifest\.xml"])
        old_files = list_files(self.old.directory, ignore=["\.gv", "\.pdf", "\.git*", "README\.md", "manifest\.xml"])
        print("New model contains:", flush=True)
        print(yaml.safe_dump(sorted(list(set(new_files) - set(old_files)))), flush=True)
        print("but not:", flush=True)
        removed_files = set(old_files) - set(new_files)
        print(yaml.safe_dump(sorted(list(removed_files))), flush=True)
        return len(removed_files) == 0

    def test_hyrodyn_load_in_hyrodyn(self):
        if not HYRODYN_AVAILABLE:
            print('Hyrodyn not present', flush=True)
            return True

        out = True
        try:
            debug_report(self.new_hml_test, self.new.urdf, self.new.submechanisms_file_path, raise_error_failure=True)
            self.new_hyrodyn = hyrodyn.RobotModel(self.new.urdf, self.new.submechanisms_file_path)
            print("Model loaded!", flush=True)
            out &= True
        except RuntimeError as e:
            print("Failed to load model in HyRoDyn:", flush=True, file=sys.stderr)
            print("Failed: Loading the model in HyRoDyn not possible. Check failed!", e, flush=True, file=sys.stderr)
            out &= False
        if self.new.floatingbase is True:
            try:
                debug_report(self.new_fb_hml_test, self.new.floatingbase_urdf,
                             self.new.floatingbase_submechanisms_file_path, raise_error_failure=True)
                print("Floatingbase model loaded!", flush=True)
                out &= True
            except RuntimeError as e:
                print("Failed to load floatingbase model in HyRoDyn:", flush=True, file=sys.stderr)
                print("Failed: Loading floatingbase model in HyRoDyn not possible. Check failed!", e,
                      flush=True, file=sys.stderr)
                out &= False
        for test in self.new_sm_hml_test:
            try:
                debug_report(test["report"], test["urdf"], test["submech"], raise_error_failure=True)
                print(test["name"]+" submodel loaded!", flush=True)
                out &= True
            except RuntimeError as e:
                print("Failed to load "+test["name"]+" submodel model in HyRoDyn:", flush=True, file=sys.stderr)
                print("Failed: Loading "+test["name"]+" submodel in HyRoDyn not possible. Check failed!", e,
                      flush=True, file=sys.stderr)
                out &= False
        return out

    def test_hyrodyn_compare_masses(self):
        if not HYRODYN_AVAILABLE:
            print('Hyrodyn not present', flush=True)
            return "skipped (no HyRoDyn)"
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            print("New HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (HyRoDyn model not loaded)"
        if self.old_hyrodyn is None and self.old is None:
            print("Old HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (no model to compare)"
        elif self.old_hyrodyn is None:
            print("Old HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (no hyrodyn model to compare)"

        self.old_hyrodyn.calculate_com_properties()
        self.new_hyrodyn.calculate_com_properties()
        print("Comparing masses:", flush=True)
        print("  Total mass of old model =", self.old_hyrodyn.mass, flush=True)
        print("  Total mass of new model =", self.new_hyrodyn.mass, flush=True)
        value = self.old_hyrodyn.mass - self.new_hyrodyn.mass
        check = value < self.new.tolerances["tolerance_mass"] * self.old_hyrodyn.mass
        print("  Success?", check, "Diff:", value, flush=True)
        return check

    def test_hyrodyn_compare_com(self):
        if not HYRODYN_AVAILABLE:
            print('Hyrodyn not present', flush=True)
            return "skipped (no HyRoDyn)"
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            print("New HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (HyRoDyn model not loaded)"
        if self.old_hyrodyn is None and self.old is None:
            print("Old HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (no model to compare)"
        elif self.old_hyrodyn is None:
            print("Old HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (no hyrodyn model to compare)"

        print("Compare COM position:", flush=True)
        print("  COM of old robot =", self.old_hyrodyn.com, flush=True)
        print("  COM of new robot =", self.new_hyrodyn.com, flush=True)
        diff = self.old_hyrodyn.com - self.new_hyrodyn.com
        value = np.linalg.norm(diff)
        check = value < (self.new.tolerances["tolerance_distance"] and
                         not any([np.isnan(x) for x in diff.reshape((diff.size,))]))
        print("  Success?", check, "Diff:", value, flush=True)
        return check

    def test_hyrodyn_compare_torques(self):
        if not HYRODYN_AVAILABLE:
            print('Hyrodyn not present', flush=True)
            return "skipped (no HyRoDyn)"
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            print("New HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (HyRoDyn model not loaded)"
        if self.old_hyrodyn is None and self.old is None:
            print("Old HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (no model to compare)"
        elif self.old_hyrodyn is None:
            print("Old HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (no hyrodyn model to compare)"

        print("Compare joint torques:", flush=True)
        self.old_hyrodyn.calculate_inverse_dynamics()
        self.new_hyrodyn.calculate_inverse_dynamics()
        torques = {}
        try:
            for i in range(len(self.old_hyrodyn.jointnames_active)):
                torques.update({
                    self.old_hyrodyn.jointnames_active[i]: {"O": self.old_hyrodyn.Tau_actuated[0][i], "N": None}
                })
            for i in range(len(self.new_hyrodyn.jointnames_active)):
                if self.new_hyrodyn.jointnames_active[i] in torques.keys():
                    torques[self.new_hyrodyn.jointnames_active[i]].update({"N": self.new_hyrodyn.Tau_actuated[0][i]})
                else:
                    torques.update({
                        self.new_hyrodyn.jointnames_active[i]: {"N": self.new_hyrodyn.Tau_actuated[0][i], "O": None}
                    })
            print("Name                          \tOld        \tNew        \tDiff", flush=True)
            for k, v in torques.items():
                diff = v["O"] - v["N"] if v["O"] is not None and v["N"] is not None else None
                name = "".join([k[j] if j < len(k) else " " for j in range(30)])
                print("{}\t{}\t{}\t{}\t{}".format(
                    name,
                    "{:f}".format(v["O"]) if v["O"] is not None else "   ---   ",
                    "{:f}".format(v["N"]) if v["N"] is not None else "   ---   ",
                    "{:f}".format(diff) if v["O"] is not None and v["N"] is not None else "   ---   ",
                    "!!!" if diff is None or np.abs(diff) > self.new.tolerances["tolerance"] or np.isnan(diff) else ""
                ), flush=True)
            if self.old_hyrodyn.Tau_actuated.shape == self.new_hyrodyn.Tau_actuated.shape:
                diff = np.abs(self.old_hyrodyn.Tau_actuated - self.new_hyrodyn.Tau_actuated)
                value = np.amax(diff)
                check = value < (self.new.tolerances["tolerance"] and
                                 not any([np.isnan(x) for x in diff.reshape((diff.size,))]))
            else:
                value = None
                check = "skipped (not the same joints)"
            print("  Success?", check, "Diff:", value, flush=True)
            return check
        except Exception as e:
            print("  Failed due to error:", e)
            return False

    def test_hyrodyn_compare_link_positions(self, end_effectors):
        if not HYRODYN_AVAILABLE:
            print('Hyrodyn not present', flush=True)
            return "skipped (no HyRoDyn)"
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            print("New HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (HyRoDyn model not loaded)"
        if self.old_hyrodyn is None and self.old is None:
            print("Old HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (no model to compare)"
        elif self.old_hyrodyn is None:
            print("Old HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (no hyrodyn model to compare)"

        succ = True

        print("Compare EE positions:", flush=True)
        for ee in end_effectors:
            if ee not in [link.name for link in self.old.links]:
                print(ee, "not found in compare model. Skipping comparison", file=sys.stderr, flush=True)
                continue
            self.old_hyrodyn.calculate_forward_kinematics(ee)
            old_pose = deepcopy(self.old_hyrodyn.pose)
            self.new_hyrodyn.calculate_forward_kinematics(ee)
            new_pose = deepcopy(self.new_hyrodyn.pose)

            print(ee, " pose of old model", old_pose, flush=True)
            print(ee, " pose of new model", new_pose, flush=True)
            value = np.linalg.norm(old_pose[0, 0:3] - new_pose[0, 0:3])
            check = value < self.new.tolerances["tolerance_distance"]
            print("  Success?", check, "Diff:", value, flush=True)
            succ &= check
        return succ

    def test_hyrodyn_symmetry_check(self, left_right_end_effectors):
        if not HYRODYN_AVAILABLE:
            print('Hyrodyn not present', flush=True)
            return True
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            print("New HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (HyRoDyn model not loaded)"

        for ee in left_right_end_effectors:
            if self.new.robot.get_link_id(ee) is None:
                print("Existing links:", [link.name for link in self.new.robot.links])
                raise AssertionError(ee + " Link does not exist in the newly exported model!")

        self.new_hyrodyn.calculate_forward_kinematics(left_right_end_effectors[0])
        left_pose = deepcopy(self.new_hyrodyn.pose)
        self.new_hyrodyn.calculate_forward_kinematics(left_right_end_effectors[1])
        right_pose = deepcopy(self.new_hyrodyn.pose)

        print("Symmetry Check:", flush=True)
        print("      Right EE of new model", right_pose[0, 0:3], flush=True)
        mirrored_left_pose = np.array([left_pose[0, 0],
                                       -left_pose[0, 1],
                                       left_pose[0, 2],
                                       left_pose[0, 3],
                                       -left_pose[0, 4],
                                       left_pose[0, 5],
                                       left_pose[0, 6]]).reshape(1, 7)
        print("  (-y) Left EE of new model", mirrored_left_pose[0, 0:3], flush=True)
        value = np.linalg.norm(right_pose[0, 0:3] - mirrored_left_pose[0, 0:3])
        check = value < self.new.tolerances["tolerance_distance"]
        print("  Success?", check, "Diff:", value, flush=True)
        print("!!! Check ignores orientation !!!", flush=True)
        return check

    def test_hyrodyn_symmetry_check_torques(self, left_right_end_effectors):
        if not HYRODYN_AVAILABLE:
            print('Hyrodyn not present', flush=True)
            return True
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            print("New HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (HyRoDyn model not loaded)"

        for ee in left_right_end_effectors:
            if self.new.robot.get_link_id(ee) is None:
                print("Existing links:", [link.name for link in self.new.robot.links])
                raise AssertionError(ee + " Link does not exist in the newly exported model!")

        self.new_hyrodyn.calculate_inverse_dynamics()

        torques = {}
        for i in range(len(self.new_hyrodyn.jointnames_active)):
            torques[self.new_hyrodyn.jointnames_active[i]] = self.new_hyrodyn.Tau_actuated[0][i]
        left = self.new.robot.get_chain(self.new.robot.get_root(), left_right_end_effectors[0],
                                        links=False, joints=True, fixed=False)
        right = self.new.robot.get_chain(self.new.robot.get_root(), left_right_end_effectors[1],
                                         links=False, joints=True, fixed=False)
        assert len(left) == len(right)
        print("Symmetry Check Torques:", flush=True)
        print("Calculated inverse dynamics for pose: ", self.new_hyrodyn.y, flush=True)
        success = True
        max_len = np.array([len(left[i] + "/" + right[i]) for i in range(len(left))]).max(initial=0)
        for i in range(len(left)):
            left_torque = torques[left[i]]
            right_torque = torques[right[i]]
            diff = left_torque - right_torque
            print("{}\t{}\t{}\t{}\t{}".format(
                left[i] + "/" + right[i] + " " * (max_len - (len(left[i] + "/" + right[i]))),
                "{:f}".format(left_torque),
                "{:f}".format(right_torque),
                "{:f}".format(diff),
                "!!!" if diff is None or np.abs(diff) > self.new.tolerances["tolerance"] or np.isnan(diff) else ""
            ), flush=True)
            success &= not (np.abs(diff) > self.new.tolerances["tolerance"] or np.isnan(diff))
        print("  Success?", success, flush=True)
        return success

    def move_hyrodyn_model(self, new_joint_angles):
        if not HYRODYN_AVAILABLE:
            print('Hyrodyn not present', flush=True)
            return True
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            print("New HyRoDyn model not present! Skipping test!", flush=True)
            return "skipped (HyRoDyn model not loaded)"

        if type(new_joint_angles) is float:
            self.new_hyrodyn.y = np.full(self.new_hyrodyn.y.size, 0.0)
            if self.old_hyrodyn is not None:
                if self.new_hyrodyn.y.size != self.old_hyrodyn.y.size:
                    print("WARNING: Number of joints in old model differ from those in new model!",
                          flush=True, file=sys.stderr)
                self.old_hyrodyn.y = np.full(self.old_hyrodyn.y.size, new_joint_angles)
        elif type(new_joint_angles) is list:
            self.new_hyrodyn.y = np.array(new_joint_angles).reshape(self.new_hyrodyn.y.size)
            if self.old_hyrodyn is not None:
                if self.new_hyrodyn.y.size != self.old_hyrodyn.y.size:
                    raise ValueError("ERROR: Number of joints in old model differ from those in new model!")
                self.old_hyrodyn.y = new_joint_angles
        elif type(new_joint_angles) is dict:
            values = np.full(self.new_hyrodyn.y.size, 0.0)
            for k, v in new_joint_angles.items():
                values[self.new_hyrodyn.jointnames_active.index(k)] = v
            self.new_hyrodyn.y = values
            if self.old_hyrodyn is not None:
                if self.new_hyrodyn.y.size != self.old_hyrodyn.y.size:
                    raise ValueError("ERROR: Number of joints in old model differ from those in new model!")
                self.old_hyrodyn.y = deepcopy(self.new_hyrodyn.y)
        else:
            raise Exception("move_hyrodyn_model not properly configured!")

