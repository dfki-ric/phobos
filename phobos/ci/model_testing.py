from copy import deepcopy
import numpy as np
import os

from ..commandline_logging import get_logger
from ..defs import *
from ..io import representation
from ..io.representation import Pose
from ..utils import xml
from ..utils.hyrodyn import get_load_report, debug_report
from ..utils.misc import execute_shell_command, list_files

log = get_logger(__name__)


class ModelTest(object):
    """Checks the general validity of a new model possibly against an old model.
    """

    def __init__(self, new_model, compare_model=None):
        # Check for the model
        self.new = new_model
        self.old = compare_model
        self.new_hyrodyn = None
        self.old_hyrodyn = None
        self.new_hml_test = get_load_report(self.new.robot.xmlfile, self.new.robot.submechanisms_file)
        self.new_sm_hml_test = []
        if len([x for x in self.new.robot.submodel_defs if not x["name"].startswith("#submech#")]) > 0:
            sm_path = os.path.join(self.new.exportdir, "submodels")
            for au in self.new.robot.submodel_defs:
                if au["name"].startswith("#submech#"):
                    continue
                self.new_sm_hml_test += [{
                    "name": au["name"],
                    "urdf": os.path.join(sm_path, au["name"], "urdf", au["name"] + ".urdf"),
                    "submech": os.path.normpath(os.path.join(sm_path, au["name"], "smurf", au["name"] + "_submechanisms.yml"))
                }]
                self.new_sm_hml_test[-1]["report"] = get_load_report(
                    self.new_sm_hml_test[-1]["urdf"],
                    self.new_sm_hml_test[-1]["submech"]
                )
        self.old_hml_test = None
        if self.old is not None:
            self.old_hml_test = get_load_report(self.old.xmlfile, self.old.submechanisms_file)

    def _load_old_hyrodyn_model(self):
        if self.old is None:
            return True
        if self.old_hml_test[2]:
            return False
        if self.old_hyrodyn is None and HYRODYN_AVAILABLE:
            try:
                log.info(
                    "Trying to load old model in hyrodyn:",
                    self.old.xmlfile,
                    "(file exists)" if os.path.exists(self.old.xmlfile) else "(does not exist)",
                    self.old.submechanisms_file,
                    "(file exists)" if os.path.exists(self.old.submechanisms_file)
                    else "(does not exist)",
                    flush=True
                )
                if self.old_hml_test[2] > 0:
                    # print(self.old_hml_test[0], file=sys.stdout)
                    # print(self.old_hml_test[1], file=sys.stderr)
                    raise Exception("Hyrodyn aborted!")
                self.old_hyrodyn = hyrodyn.RobotModel(
                    self.old.xmlfile,
                    self.old.submechanisms_file
                )
                log.info("Old model loaded!")
                return True
            except Exception as e:
                log.error("Failed to load old model in hyrodyn:")
                log.error(e)
                return False

    # info procedures
    def info_swing_my_robot(self):
        if not (hasattr(self.new, "swing_my_robot") and self.new.swing_my_robot is True):
            return
        log.info("Running swing_my_robot")
        if not HYRODYN_AVAILABLE:
            log.warning('Info procedure swing_my_robot not possible: Hyrodyn not present')
            return
        submech_file = self.new.submechanisms_file
        cmd = "swing_my_robot.py"
        if submech_file is not None and os.path.exists(submech_file):
            log.info(f'Submechs: {submech_file} {"exists!" if os.path.isfile(submech_file) else "does not exist!"}')
            cmd += " --submechanism_yml " + submech_file
        limits_file = os.path.join(self.new.modeldir, "submechanisms/joint_limits.yml")
        log.info(f'Limits: {limits_file} {"exists!" if os.path.isfile(limits_file) else "does not exist!"}')
        if os.path.isfile(limits_file):
            cmd += " --joint_limits_yml " + limits_file
        log.info("URDF:", self.new.robot.xmlfile,
                 "exists!" if os.path.isfile(self.new.robot.xmlfile) else "does not exist!")
        # cmd += " --export_animation"
        cmd += " --export_animation_as_mp4"
        cmd += " " + self.new.robot.xmlfile
        cmd += " --out " + os.path.join(self.new.modeldir, "swing-anim_"+self.new.modelname+".gif")
        cmd += "|| true"  # make this fail safe
        try:
            execute_shell_command(cmd, os.getcwd())
        except ImportError as e:
            log.warning(f"Can't run swing_my_robot procedure because of missing library:\n {e}")
            return
        except AssertionError as e:
            log.error(f"Swing_my_robot.py failed due to an assertion error: {e}")
            return
        except:
            try:
                execute_shell_command("python " + os.path.join(os.environ["AUTOPROJ_CURRENT_ROOT"],
                                                               "control/hyrodyn/python/") + cmd, os.getcwd())
            except ImportError as e:
                log.warning(f"Can't run swing_my_robot procedure because of missing library:\n {e}")
                return
            except AssertionError as e:
                log.error(f"Swing_my_robot.py failed due to an assertion error: {e}")
                return
            except Exception as e:
                log.error(f"Swing_my_robot.py failed due to an unknown error {e}")

    # test procedures
    def test_process_double_check(self):
        success = True
        # have joints been removed?
        if hasattr(self.new, "remove_joints"):
            joint_names = [joint.name for joint in self.new.robot.joints]
            for joint_name in self.new.remove_joints:
                success &= joint_name not in joint_names
        # [TODO later] check the others
        # check if meshes are there
        log.debug(f"Checking meshes for {self.new.robot.xmlfile}")
        for link in self.new.robot.links:
            for mesh in [c.geometry.filepath for c in link.collisions+link.visuals if isinstance(c.geometry, representation.Mesh)]:
                mesh_path = xml.read_relative_filename(mesh, self.new.robot.xmlfile)
                if os.path.isfile(mesh_path):
                    success &= True
                else:
                    log.error(f"Mesh {mesh_path} does not exist!")
                    success = False
        return success

    def test_compare_link_masses(self):
        success = True
        log.info("Masses of Links:")
        new_link_masses = {
            link.name: (link.inertial.mass if link.inertial is not None else 0) for link in self.new.robot.links
        }
        if self.old is not None:
            old_link_masses = {
                link.name: (link.inertial.mass if link.inertial is not None else 0) for link in self.old.links
            }
        else:
            log.info("Old model not present! Skipping test!")
            return "skipped (no model to compare)"
        max_length = max(*[len(n) for n in new_link_masses.keys()])
        for k in new_link_masses.keys():
            link_name = k + (max_length - len(k)) * " "
            if k not in old_link_masses.keys():
                outmsg = "%s New: %8.4e\t Old: non-existent\n" % (link_name, new_link_masses[k])
                continue
            outmsg = "%s New: %8.4e\t Old: %8.4e\t Diff: %8.4e" % (link_name, new_link_masses[k], old_link_masses[k], new_link_masses[k] - old_link_masses[k])
            if abs(new_link_masses[k] - old_link_masses[k]) > self.new.test["tolerances"]["mass"]:
                outmsg +=" too big!"
                success = False
            log.info(outmsg)
        return success

    def test_symmetry_check_masses(self, left_right_end_effectors):
        for ee in left_right_end_effectors:
            if self.new.robot.get_link_id(ee) is None:
                log.error(f"Existing links: {[link.name for link in self.new.robot.links]}")
                raise AssertionError(ee + " Link does not exist in the newly exported model!")

        masses = {}
        for link in self.new.robot.links:
            masses[link.name] = link.inertial.mass if hasattr(link, "inertial") and link.inertial is not None else 0
        left = self.new.robot.get_chain(self.new.robot.get_root(), left_right_end_effectors[0],
                                        links=True, joints=False, fixed=True)
        right = self.new.robot.get_chain(self.new.robot.get_root(), left_right_end_effectors[1],
                                         links=True, joints=False, fixed=True)
        assert len(left) == len(right)
        log.info("Symmetry Check Masses:")
        success = True
        max_len = np.array([len(left[i] + "/" + right[i]) for i in range(len(left))]).max(initial=0)
        for i in range(len(left)):
            left_mass = masses[left[i]]
            right_mass = masses[right[i]]
            diff = left_mass - right_mass
            log.info("{}\t{}\t{}\t{}\t{}".format(
                left[i] + "/" + right[i] + " " * (max_len - (len(left[i] + "/" + right[i]))),
                "{:f}".format(left_mass),
                "{:f}".format(right_mass),
                "{:f}".format(diff),
                "!!!" if diff is None or np.abs(diff) > self.new.test["tolerances"]["mass"] or np.isnan(diff) else ""
            ))
            success &= not (np.abs(diff) > self.new.test["tolerances"]["mass"] or np.isnan(diff))
        log.info(f"  Success? {success}")
        return success

    def test_compare_link_transformations(self):
        success = True
        log.info("Transformation changes of Links:")
        root2new_links = {link.name: self.new.robot.get_transformation(link.name) for link in self.new.robot.links}
        if self.old is not None:
            root2old_links = {link.name: self.old.get_transformation(link.name) for link in self.old.links}
        else:
            log.info("Old model not present! Skipping test!")
            return "skipped (no model to compare)"
        max_length = max(*[len(n) for n in root2new_links.keys()])
        for k in root2new_links.keys():
            link_name = k + (max_length - len(k)) * " "
            if k not in root2old_links.keys():
                log.info("%s doesn't exist in compare model" % link_name)
                _temp_pose = Pose.from_matrix(root2new_links[k], relative_to=self.new.robot.get_root())
                log.info("root2link: xyz: %.5f %.5f %.5f\trpy: %.5f %.5f %.5f" % tuple(_temp_pose.xyz + _temp_pose.rpy))
                continue
            diff = np.linalg.inv(root2old_links[k]).dot(root2new_links[k])
            diff_o = representation.Pose.from_matrix(diff, relative_to=k)
            outmsg = "%s Difference: xyz: %.5f %.5f %.5f\trpy: %.5f %.5f %.5f" % tuple([link_name] + diff_o.xyz + diff_o.rpy)
            if np.linalg.norm(diff[0:3, 3]) > self.new.test["tolerances"]["distance"] or \
               any(np.abs(diff_o.rpy) > [self.new.test["tolerances"]["rad"]]*3):
                if np.linalg.norm(diff[0:3, 3]) > self.new.test["tolerances"]["distance"]:
                    outmsg += " %.6f" % (np.linalg.norm(diff[0:3, 3])) + " > " + str(self.new.test["tolerances"]["distance"])
                if any(np.abs(diff_o.rpy) > [self.new.test["tolerances"]["rad"]]*3):
                    outmsg += str(np.abs(diff_o.rpy)) + " > " + str([self.new.test["tolerances"]["rad"]]*3)
                outmsg += " !!!"
                log.error(outmsg)
                # print("Difference as Transformation Matrix from old to new:")
                # print(diff)
                success = False
            else:
                log.info(outmsg)
        return success

    def test_topological_self_consistency(self):
        if self.old is None:
            log.info("Old model not present! Skipping test!")
            return "skipped (no model to compare)"
        # The new model is allowed to have more joints/links than the previous
        log.info("New model contains:")
        log.info(f"Links {sorted(set([link.name for link in self.new.robot.links]) - set([link.name for link in self.old.links]))}")
        log.info(f"Coll {sorted(set([collision.name for link in self.new.robot.links for collision in link.collisions]) - set([collision.name for link in self.old.links for collision in link.collisions]))}")
        log.info(f"Vis {sorted(set([visual.name for link in self.new.robot.links for visual in link.visuals]) - set([visual.name for link in self.old.links for visual in link.visuals]))}")
        log.info(f"Joints {sorted(set([joint.name for joint in self.new.robot.joints]) - set([joint.name for joint in self.old.joints]))}")
        changed_joint_types1 = set([joint.name + ":" + joint.joint_type for joint in self.new.robot.joints]) - \
                               set([joint.name + ":" + joint.joint_type for joint in self.old.joints])
        log.info(f"JTypes {sorted(changed_joint_types1)}")
        log.info("but not:")
        removed_links = set([link.name for link in self.old.links]) - set([link.name for link in self.new.robot.links])
        log.info(f"Links {sorted(removed_links)}")
        log.info(f"Coll {sorted(set([collision.name for link in self.old.links for collision in link.collisions]) - set([collision.name for link in self.new.robot.links for collision in link.collisions]))}")
        log.info(f"Vis {sorted(set([visual.name for link in self.old.links for visual in link.visuals]) - set([visual.name for link in self.new.robot.links for visual in link.visuals]))}")
        removed_joints = set([joint.name for joint in self.old.joints]) - \
                         set([joint.name for joint in self.new.robot.joints])
        log.info(f"Joints {sorted(removed_joints)}")
        changed_joint_types2 = set([joint.name + ":" + joint.joint_type for joint in self.old.joints]) - \
                               set([joint.name + ":" + joint.joint_type for joint in self.new.robot.joints])
        log.info(f"JTypes {sorted(changed_joint_types2)}")
        return len(removed_links) + len(removed_joints) + len(changed_joint_types1) + len(changed_joint_types2) == 0

    def test_compare_amount_joints(self):
        if self.old is None:
            log.info("Old model not present! Skipping test!")
            return "skipped (no model to compare)"
        log.info(f"New Model has {len(self.new.robot.joints)} and old model has {len(self.old.joints)}")
        return len(self.new.robot.joints) == len(self.old.joints)

    def test_load_in_pybullet(self):
        if not PYBULLET_AVAILABLE:
            log.warning('Pybullet not present')
            return True
        try:
            client = pb.connect(pb.DIRECT)
            pb.loadURDF(os.path.join(self.new.robot.xmlfile), physicsClientId=client)
            pb.disconnect(client)
            return True
        except Exception as e:
            log.error(f"Failed: pyBullet check failed!\n {e}")
            return False

    def test_file_consistency(self):
        if self.old is None:
            log.info("Old model not present! Skipping test!")
            return "skipped (no model to compare)"
        new_files = list_files(self.new.exportdir, ignore=["\.gv", "\.pdf", "\.git*", "README\.md", "manifest\.xml"])
        old_files = list_files(self.old.directory, ignore=["\.gv", "\.pdf", "\.git*", "README\.md", "manifest\.xml"])
        log.info("New model contains:")
        log.info(dump_json(sorted(list(set(new_files) - set(old_files)))))
        log.info("but not:")
        removed_files = set(old_files) - set(new_files)
        log.info(dump_json(sorted(list(removed_files))))
        return len(removed_files) == 0

    def test_hyrodyn_load_in_hyrodyn(self):
        if not HYRODYN_AVAILABLE:
            print('Hyrodyn not present')
            return True

        out = True
        try:
            debug_report(self.new_hml_test, self.new.robot.xmlfile, self.new.robot.submechanisms_file, raise_error_failure=True)
            self.new_hyrodyn = hyrodyn.RobotModel(self.new.robot.xmlfile, self.new.robot.submechanisms_file)
            log.info("Model loaded!")
            out &= True
        except RuntimeError as e:
            log.error(f"Failed: Loading the model in HyRoDyn not possible. Check failed! {e}")
            out &= False
        for test in self.new_sm_hml_test:
            try:
                debug_report(test["report"], test["urdf"], test["submech"], raise_error_failure=True)
                log.info(test["name"]+" submodel loaded!")
                out &= True
            except RuntimeError as e:
                log.error(f"Failed: Loading {test['name']} submodel in HyRoDyn not possible. Check failed! {e}")
                out &= False
        return out

    def test_hyrodyn_compare_masses(self):
        if not HYRODYN_AVAILABLE:
            log.info('Hyrodyn not present')
            return "skipped (no HyRoDyn)"
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            log.info("New HyRoDyn model not present! Skipping test!")
            return "skipped (HyRoDyn model not loaded)"
        if self.old_hyrodyn is None and self.old is None:
            log.info("Old HyRoDyn model not present! Skipping test!")
            return "skipped (no model to compare)"
        elif self.old_hyrodyn is None:
            log.info("Old HyRoDyn model not present! Skipping test!")
            return "skipped (no hyrodyn model to compare)"

        self.old_hyrodyn.calculate_com_properties()
        self.new_hyrodyn.calculate_com_properties()
        log.info("Comparing masses:")
        log.info(f"  Total mass of old model = {self.old_hyrodyn.mass}")
        log.info(f"  Total mass of new model = {self.new_hyrodyn.mass}")
        value = self.old_hyrodyn.mass - self.new_hyrodyn.mass
        check = value < self.new.test["tolerances"]["mass"] * self.old_hyrodyn.mass
        log.info("  Success? {check} Diff: {value}")
        return check

    def test_hyrodyn_compare_com(self):
        if not HYRODYN_AVAILABLE:
            log.info('Hyrodyn not present')
            return "skipped (no HyRoDyn)"
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            log.info("New HyRoDyn model not present! Skipping test!")
            return "skipped (HyRoDyn model not loaded)"
        if self.old_hyrodyn is None and self.old is None:
            log.info("Old HyRoDyn model not present! Skipping test!")
            return "skipped (no model to compare)"
        elif self.old_hyrodyn is None:
            log.info("Old HyRoDyn model not present! Skipping test!")
            return "skipped (no hyrodyn model to compare)"

        log.info("Compare COM position:")
        log.info(f"  COM of old robot = {self.old_hyrodyn.com}")
        log.info(f"  COM of new robot = {self.new_hyrodyn.com}")
        diff = self.old_hyrodyn.com - self.new_hyrodyn.com
        value = np.linalg.norm(diff)
        check = value < (self.new.test["tolerances"]["distance"] and
                         not any([np.isnan(x) for x in diff.reshape((diff.size,))]))
        log.info(f"  Success? {check} Diff: {value}")
        return check

    def test_hyrodyn_compare_torques(self):
        if not HYRODYN_AVAILABLE:
            log.info('Hyrodyn not present')
            return "skipped (no HyRoDyn)"
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            log.info("New HyRoDyn model not present! Skipping test!")
            return "skipped (HyRoDyn model not loaded)"
        if self.old_hyrodyn is None and self.old is None:
            log.info("Old HyRoDyn model not present! Skipping test!")
            return "skipped (no model to compare)"
        elif self.old_hyrodyn is None:
            log.info("Old HyRoDyn model not present! Skipping test!")
            return "skipped (no hyrodyn model to compare)"

        log.info("Compare joint torques:")
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
            log.info("Name                          \tOld        \tNew        \tDiff")
            for k, v in torques.items():
                diff = v["O"] - v["N"] if v["O"] is not None and v["N"] is not None else None
                name = "".join([k[j] if j < len(k) else " " for j in range(30)])
                log.info("{}\t{}\t{}\t{}\t{}".format(
                    name,
                    "{:f}".format(v["O"]) if v["O"] is not None else "   ---   ",
                    "{:f}".format(v["N"]) if v["N"] is not None else "   ---   ",
                    "{:f}".format(diff) if v["O"] is not None and v["N"] is not None else "   ---   ",
                    "!!!" if diff is None or np.abs(diff) > self.new.test["tolerances"]["default"] or np.isnan(diff) else ""
                ))
            if self.old_hyrodyn.Tau_actuated.shape == self.new_hyrodyn.Tau_actuated.shape:
                diff = np.abs(self.old_hyrodyn.Tau_actuated - self.new_hyrodyn.Tau_actuated)
                value = np.amax(diff)
                check = value < (self.new.test["tolerances"]["default"] and
                                 not any([np.isnan(x) for x in diff.reshape((diff.size,))]))
            else:
                value = None
                check = "skipped (not the same joints)"
            log.info(f"  Success? {check} Diff: {value}")
            return check
        except Exception as e:
            log.error(f"  Failed due to error: {e}")
            return False

    def test_hyrodyn_compare_link_positions(self, end_effectors):
        if not HYRODYN_AVAILABLE:
            log.info('Hyrodyn not present')
            return "skipped (no HyRoDyn)"
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            log.info("New HyRoDyn model not present! Skipping test!")
            return "skipped (HyRoDyn model not loaded)"
        if self.old_hyrodyn is None and self.old is None:
            log.info("Old HyRoDyn model not present! Skipping test!")
            return "skipped (no model to compare)"
        elif self.old_hyrodyn is None:
            log.info("Old HyRoDyn model not present! Skipping test!")
            return "skipped (no hyrodyn model to compare)"

        succ = True

        log.info("Compare EE positions:")
        for ee in end_effectors:
            if ee not in [link.name for link in self.old.links]:
                log.error(f"{ee} not found in compare model. Skipping comparison")
                continue
            self.old_hyrodyn.calculate_forward_kinematics(ee)
            old_pose = deepcopy(self.old_hyrodyn.pose)
            self.new_hyrodyn.calculate_forward_kinematics(ee)
            new_pose = deepcopy(self.new_hyrodyn.pose)

            log.info(f"ee pose of old model {old_pose}")
            log.info(f"ee pose of new model {new_pose}")
            value = np.linalg.norm(old_pose[0, 0:3] - new_pose[0, 0:3])
            check = value < self.new.test["tolerances"]["distance"]
            log.info(f"  Success? {check} Diff: {value}")
            succ &= check
        return succ

    def test_hyrodyn_symmetry_check(self, left_right_end_effectors):
        if not HYRODYN_AVAILABLE:
            log.info('Hyrodyn not present')
            return True
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            log.info("New HyRoDyn model not present! Skipping test!")
            return "skipped (HyRoDyn model not loaded)"

        for ee in left_right_end_effectors:
            if self.new.robot.get_link_id(ee) is None:
                log.info(f"Existing links: {[link.name for link in self.new.robot.links]}")
                raise AssertionError(ee + " Link does not exist in the newly exported model!")

        self.new_hyrodyn.calculate_forward_kinematics(left_right_end_effectors[0])
        left_pose = deepcopy(self.new_hyrodyn.pose)
        self.new_hyrodyn.calculate_forward_kinematics(left_right_end_effectors[1])
        right_pose = deepcopy(self.new_hyrodyn.pose)

        log.info("Symmetry Check:")
        log.info(f"      Right EE of new model {right_pose[0, 0:3]}")
        mirrored_left_pose = np.array([left_pose[0, 0],
                                       -left_pose[0, 1],
                                       left_pose[0, 2],
                                       left_pose[0, 3],
                                       -left_pose[0, 4],
                                       left_pose[0, 5],
                                       left_pose[0, 6]]).reshape(1, 7)
        log.info(f"  (-y) Left EE of new model {mirrored_left_pose[0, 0:3]}")
        value = np.linalg.norm(right_pose[0, 0:3] - mirrored_left_pose[0, 0:3])
        check = value < self.new.test["tolerances"]["distance"]
        log.info(f"  Success? {check} Diff: {value}")
        log.info("!!! Check ignores orientation !!!")
        return check

    def test_hyrodyn_symmetry_check_torques(self, left_right_end_effectors):
        if not HYRODYN_AVAILABLE:
            log.info('Hyrodyn not present')
            return True
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            log.info("New HyRoDyn model not present! Skipping test!")
            return "skipped (HyRoDyn model not loaded)"

        for ee in left_right_end_effectors:
            if self.new.robot.get_link_id(ee) is None:
                log.error(f"Existing links: {[link.name for link in self.new.robot.links]}")
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
        log.info("Symmetry Check Torques:")
        log.info(f"Calculated inverse dynamics for pose: {self.new_hyrodyn.y}")
        success = True
        max_len = np.array([len(left[i] + "/" + right[i]) for i in range(len(left))]).max(initial=0)
        for i in range(len(left)):
            left_torque = torques[left[i]]
            right_torque = torques[right[i]]
            diff = left_torque - right_torque
            log.info("{}\t{}\t{}\t{}\t{}".format(
                left[i] + "/" + right[i] + " " * (max_len - (len(left[i] + "/" + right[i]))),
                "{:f}".format(left_torque),
                "{:f}".format(right_torque),
                "{:f}".format(diff),
                "!!!" if diff is None or np.abs(diff) > self.new.test["tolerances"]["default"] or np.isnan(diff) else ""
            ))
            success &= not (np.abs(diff) > self.new.test["tolerances"]["default"] or np.isnan(diff))
        log.info(f"  Success? {success}")
        return success

    def move_hyrodyn_model(self, new_joint_angles):
        if not HYRODYN_AVAILABLE:
            log.info('Hyrodyn not present')
            return True
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            log.info("New HyRoDyn model not present! Skipping test!")
            return "skipped (HyRoDyn model not loaded)"

        if type(new_joint_angles) is float:
            self.new_hyrodyn.y = np.full(self.new_hyrodyn.y.size, 0.0)
            if self.old_hyrodyn is not None:
                if self.new_hyrodyn.y.size != self.old_hyrodyn.y.size:
                    log.warning("Number of joints in old model differ from those in new model!")
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

