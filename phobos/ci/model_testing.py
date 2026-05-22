from copy import deepcopy
import numpy as np
import os

from ..commandline_logging import get_logger
from ..defs import *
from ..io import representation
from ..io.representation import Pose
from ..utils import xml
from ..utils.hyrodyn import get_load_report, debug_report
from ..utils.misc import execute_shell_command, list_files, append_string

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
        if len([x for name, x in self.new.robot.submodel_defs.items() if not name.startswith("#submech#")]) > 0:
            sm_path = os.path.join(self.new.exportdir, "submodels")
            for name, au in self.new.robot.submodel_defs.items():
                if name.startswith("#submech#"):
                    continue
                self.new_sm_hml_test += [{
                    "name": name,
                    "urdf": os.path.join(sm_path, name, "urdf", name + ".urdf"),
                    "submech": os.path.normpath(os.path.join(sm_path, name, "smurf", name + "_submechanisms.yml"))
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
            return None
        protocol = None
        protocol = append_string(protocol, "Running swing_my_robot", loglevel="info")
        if not HYRODYN_AVAILABLE:
            protocol = append_string(protocol, 'Info procedure swing_my_robot not possible: Hyrodyn not present', loglevel="warning")
            return protocol
        submech_file = self.new.submechanisms_file
        cmd = "swing_my_robot.py"
        if submech_file is not None and os.path.exists(submech_file):
            protocol = append_string(protocol, f'Submechs: {submech_file} {"exists!" if os.path.isfile(submech_file) else "does not exist!"}', loglevel="info")
            cmd += " --submechanism_yml " + submech_file
        limits_file = os.path.join(self.new.modeldir, "submechanisms/joint_limits.yml")
        protocol = append_string(protocol, f'Limits: {limits_file} {"exists!" if os.path.isfile(limits_file) else "does not exist!"}', loglevel="info")
        if os.path.isfile(limits_file):
            cmd += " --joint_limits_yml " + limits_file
        protocol = append_string(protocol, f"URDF: {self.new.robot.xmlfile}" + "exists!" if os.path.isfile(self.new.robot.xmlfile) else "does not exist!", loglevel="info")
        # cmd += " --export_animation"
        cmd += " --export_animation_as_mp4"
        cmd += " " + self.new.robot.xmlfile
        cmd += " --out " + os.path.join(self.new.modeldir, "swing-anim_"+self.new.modelname+".gif")
        cmd += "|| true"  # make this fail safe
        try:
            execute_shell_command(cmd, os.getcwd())
        except ImportError as e:
            protocol = append_string(protocol, f"Can't run swing_my_robot procedure because of missing library:\n {e}", loglevel="warning")
            return
        except AssertionError as e:
            protocol = append_string(protocol, f"Swing_my_robot.py failed due to an assertion error: {e}", loglevel="error")
            return
        except:
            try:
                execute_shell_command("python " + os.path.join(os.environ["AUTOPROJ_CURRENT_ROOT"],
                                                               "control/hyrodyn/python/") + cmd, os.getcwd())
            except ImportError as e:
                protocol = append_string(protocol, f"Can't run swing_my_robot procedure because of missing library:\n {e}", loglevel="warning")
                return
            except AssertionError as e:
                protocol = append_string(protocol, f"Swing_my_robot.py failed due to an assertion error: {e}", loglevel="error")
                return
            except Exception as e:
                protocol = append_string(protocol, f"Swing_my_robot.py failed due to an unknown error {e}", loglevel="error")
        return protocol

    # test procedures
    def test_process_double_check(self):
        success = True
        protocol = ""
        # have joints been removed?
        if hasattr(self.new, "remove_joints"):
            joint_names = [joint.name for joint in self.new.robot.joints]
            for joint_name in self.new.remove_joints:
                success &= joint_name not in joint_names
        # [TODO later] check the others
        # check if meshes are there
        protocol = append_string(protocol, f"Checking meshes for {self.new.robot.xmlfile}", loglevel="debug")
        for link in self.new.robot.links:
            for mesh in [c.geometry.filepath for c in link.collisions+link.visuals if isinstance(c.geometry, representation.Mesh)]:
                mesh_path = xml.read_relative_filename(mesh, self.new.robot.xmlfile)
                if os.path.isfile(mesh_path):
                    success &= True
                else:
                    protocol = append_string(protocol, f"Mesh {mesh_path} does not exist!", loglevel="error")
                    success = False
        return success, protocol

    def test_total_mass(self, expected_mass=None):
        protocol = ""
        if expected_mass is None:
            path = os.path.join(os.path.dirname(self.new.configfile), getattr(self.new, "test_data_file", "test_data.yml"))
            if not os.path.isfile(path):
                path = os.path.join(os.path.dirname(self.new.exportdir), getattr(self.new, "test_data_file", "test_data.yml"))
            assert os.path.isfile(path), path
            log.info("Found test_data_file: "+path)
            with open(path, "r") as f:
                expected_mass = load_json(f.read())["test_data"]["expected_mass"]
        assert expected_mass is not None and type(expected_mass) in [float, int]
        old_mass = None
        if self.old is not None:
            old_mass = self.old.compute_mass()
        current_mass = self.new.robot.compute_mass()
        mass_check = np.absolute(current_mass - expected_mass) <= self.new.test["tolerances"]["mass"]
        out_msg = f"Mass check: Current: {current_mass}; Expected: {expected_mass}; Previous: {old_mass}"
        if not mass_check:
            out_msg += " Difference too big"
        mass_check2 = True
        if old_mass:
            mass_check2 = np.absolute(old_mass - expected_mass) <= self.new.test["tolerances"]["mass"]
            out_msg += " Mass changed with respect to previous version"
        protocol = append_string(protocol, out_msg, loglevel="info" if mass_check and mass_check2 else "error")
        return mass_check and mass_check2, protocol

    def test_compare_link_masses(self):
        success = True
        protocol = ""
        protocol = append_string(protocol, "Masses of Links:", loglevel="info")
        new_link_masses = {
            link.name: (link.inertial.mass if link.inertial is not None else 0) for link in self.new.robot.links
        }
        if self.old is not None:
            old_link_masses = {
                link.name: (link.inertial.mass if link.inertial is not None else 0) for link in self.old.links
            }
        else:
            protocol = append_string(protocol, "Old model not present! Skipping test!", loglevel="info")
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
            protocol = append_string(protocol, outmsg, loglevel="info" if success else "error")
        return success, protocol

    def test_symmetry_check_masses(self, left_right_end_effectors):
        protocol = ""
        for ee in left_right_end_effectors:
            if self.new.robot.get_link_id(ee) is None:
                protocol = append_string(protocol, f"Existing links: {[link.name for link in self.new.robot.links]}", loglevel="error")
                raise AssertionError(ee + " Link does not exist in the newly exported model!")
        masses = {}
        for link in self.new.robot.links:
            masses[link.name] = link.inertial.mass if hasattr(link, "inertial") and link.inertial is not None else 0
        left = self.new.robot.get_chain(self.new.robot.get_root(), left_right_end_effectors[0],
                                        links=True, joints=False, fixed=True)
        right = self.new.robot.get_chain(self.new.robot.get_root(), left_right_end_effectors[1],
                                         links=True, joints=False, fixed=True)
        assert len(left) == len(right)
        protocol = append_string(protocol, "Symmetry Check Masses:", loglevel="info")
        success = True
        max_len = np.array([len(left[i] + "/" + right[i]) for i in range(len(left))]).max(initial=0)
        for i in range(len(left)):
            left_mass = masses[left[i]]
            right_mass = masses[right[i]]
            diff = left_mass - right_mass
            protocol = append_string(protocol, "{}\t{}\t{}\t{}\t{}".format(
                left[i] + "/" + right[i] + " " * (max_len - (len(left[i] + "/" + right[i]))),
                "{:f}".format(left_mass),
                "{:f}".format(right_mass),
                "{:f}".format(diff),
                "!!!" if diff is None or np.abs(diff) > self.new.test["tolerances"]["mass"] or np.isnan(diff) else ""
            ), loglevel="info")
            success &= not (np.abs(diff) > self.new.test["tolerances"]["mass"] or np.isnan(diff))
        protocol = append_string(protocol, f"  Success? {success}", loglevel="info")
        return success, protocol

    def test_compare_link_transformations(self):
        success = True
        protocol = ""
        log.info("Transformation changes of Links:")
        root2new_links = {link.name: self.new.robot.get_transformation(link.name) for link in self.new.robot.links}
        if self.old is not None:
            root2old_links = {link.name: self.old.get_transformation(link.name) for link in self.old.links}
        else:
            protocol = append_string(protocol, "Old model not present! Skipping test!", loglevel="info")
            return "skipped (no model to compare)", protocol
        max_length = max(*[len(n) for n in root2new_links.keys()])
        for k in root2new_links.keys():
            link_name = k + (max_length - len(k)) * " "
            if k not in root2old_links.keys():
                protocol = append_string(protocol, "%s doesn't exist in compare model" % link_name, loglevel="info")
                _temp_pose = Pose.from_matrix(root2new_links[k], relative_to=self.new.robot.get_root())
                protocol = append_string(protocol, "root2link: xyz: %.5f %.5f %.5f\trpy: %.5f %.5f %.5f" % tuple(_temp_pose.xyz + _temp_pose.rpy), loglevel="info")
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
                protocol = append_string(protocol, outmsg, loglevel="error")
                # print("Difference as Transformation Matrix from old to new:")
                # print(diff)
                success = False
            else:
                protocol = append_string(protocol, outmsg, loglevel="info")
        return success, protocol

    def test_topological_self_consistency(self):
        protocol = ""
        if self.old is None:
            log.info("Old model not present! Skipping test!")
            return "skipped (no model to compare)", protocol
        # The new model is allowed to have more joints/links than the previous
        protocol = append_string(protocol, "New model contains:", loglevel="info")
        protocol = append_string(protocol, f"Links {sorted(set([link.name for link in self.new.robot.links]) - set([link.name for link in self.old.links]))}", loglevel="info")
        protocol = append_string(protocol, f"Coll {sorted(set([collision.name for link in self.new.robot.links for collision in link.collisions]) - set([collision.name for link in self.old.links for collision in link.collisions]))}", loglevel="info")
        protocol = append_string(protocol, f"Vis {sorted(set([visual.name for link in self.new.robot.links for visual in link.visuals]) - set([visual.name for link in self.old.links for visual in link.visuals]))}", loglevel="info")
        protocol = append_string(protocol, f"Joints {sorted(set([joint.name for joint in self.new.robot.joints]) - set([joint.name for joint in self.old.joints]))}", loglevel="info")
        changed_joint_types1 = set([joint.name + ":" + joint.joint_type for joint in self.new.robot.joints]) - \
                               set([joint.name + ":" + joint.joint_type for joint in self.old.joints])
        protocol = append_string(protocol, f"JTypes {sorted(changed_joint_types1)}", loglevel="info")
        protocol = append_string(protocol, "but not:", loglevel="info")
        removed_links = set([link.name for link in self.old.links]) - set([link.name for link in self.new.robot.links])
        protocol = append_string(protocol, f"Links {sorted(removed_links)}", loglevel="info")
        protocol = append_string(protocol, f"Coll {sorted(set([collision.name for link in self.old.links for collision in link.collisions]) - set([collision.name for link in self.new.robot.links for collision in link.collisions]))}", loglevel="info")
        protocol = append_string(protocol, f"Vis {sorted(set([visual.name for link in self.old.links for visual in link.visuals]) - set([visual.name for link in self.new.robot.links for visual in link.visuals]))}", loglevel="info")
        removed_joints = set([joint.name for joint in self.old.joints]) - \
                         set([joint.name for joint in self.new.robot.joints])
        protocol = append_string(protocol, f"Joints {sorted(removed_joints)}", loglevel="info")
        changed_joint_types2 = set([joint.name + ":" + joint.joint_type for joint in self.old.joints]) - \
                               set([joint.name + ":" + joint.joint_type for joint in self.new.robot.joints])
        protocol = append_string(protocol, f"JTypes {sorted(changed_joint_types2)}", loglevel="info")
        return len(removed_links) + len(removed_joints) + len(changed_joint_types1) + len(changed_joint_types2) == 0, protocol

    def test_compare_amount_joints(self):
        protocol = ""
        if self.old is None:
            protocol = append_string(protocol, "Old model not present! Skipping test!", loglevel="info")
            return "skipped (no model to compare)", protocol
        protocol = append_string(protocol, f"New Model has {len(self.new.robot.joints)} and old model has {len(self.old.joints)}", loglevel="info")
        return len(self.new.robot.movable_joints) == len(self.old.movable_joints), protocol

    def test_load_in_pybullet(self):
        protocol = ""
        from ..defs import  check_pybullet_available
        if not check_pybullet_available():
            protocol = append_string(protocol, 'Pybullet not present', loglevel="warning")
            return True, protocol
        try:
            import pybullet as pb
            client = pb.connect(pb.DIRECT)
            pb.loadURDF(os.path.join(self.new.robot.xmlfile), physicsClientId=client)
            pb.disconnect(client)
            return True, protocol
        except Exception as e:
            protocol = append_string(protocol, f"Failed: pyBullet check failed!\n {e}", loglevel="error")
            return False, protocol

    def test_file_consistency(self):
        protocol = ""
        if self.old is None:
            protocol = append_string("Old model not present! Skipping test!", loglevel="info")
            return "skipped (no model to compare)", protocol
        ignore = ["\.gv", "\.pdf", "\.git*", "README\.md", "manifest\.xml", "meshes/*"]
        new_files = list_files(self.new.exportdir, ignore=ignore)
        new_files_full = list_files(self.new.exportdir)
        old_files = list_files(self.old.directory, ignore=ignore)
        old_files_full = list_files(self.old.directory)
        protocol = append_string(protocol, "New model contains:", loglevel="info")
        protocol = append_string(protocol, dump_json(sorted(list(set(new_files) - set(old_files)))), loglevel="info")
        protocol = append_string(protocol, "but not:", loglevel="info")
        removed_files = set(old_files) - set(new_files)
        protocol = append_string(protocol, dump_json(sorted(list(removed_files))), loglevel="info")
        if len(removed_files) != 0:
            protocol = append_string(protocol, "The removal of following files is critical: " + str(removed_files), loglevel="error")
        return len(removed_files) == 0, protocol

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
        protocol = ""
        if not HYRODYN_AVAILABLE:
            protocol = append_string(protocol, 'Hyrodyn not present', loglevel="info")
            return "skipped (no HyRoDyn)", protocol
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            protocol = append_string(protocol, "New HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (HyRoDyn model not loaded)", protocol
        if self.old_hyrodyn is None and self.old is None:
            protocol = append_string(protocol, "Old HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (no model to compare)", protocol
        elif self.old_hyrodyn is None:
            protocol = append_string(protocol, "Old HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (no hyrodyn model to compare)", protocol

        self.old_hyrodyn.calculate_com_properties()
        self.new_hyrodyn.calculate_com_properties()
        protocol = append_string(protocol, "Comparing masses:", loglevel="info")
        protocol = append_string(protocol, f"  Total mass of old model = {self.old_hyrodyn.mass}", loglevel="info")
        protocol = append_string(protocol, f"  Total mass of new model = {self.new_hyrodyn.mass}", loglevel="info")
        value = self.old_hyrodyn.mass - self.new_hyrodyn.mass
        check = value < self.new.test["tolerances"]["mass"] * self.old_hyrodyn.mass
        protocol = append_string(protocol, "  Success? {check} Diff: {value}", loglevel="info")
        return check, protocol

    def test_hyrodyn_compare_com(self):
        protocol = ""
        if not HYRODYN_AVAILABLE:
            protocol = append_string(protocol, 'Hyrodyn not present', loglevel="info")
            return "skipped (no HyRoDyn)", protocol
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            protocol = append_string(protocol, "New HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (HyRoDyn model not loaded)", protocol
        if self.old_hyrodyn is None and self.old is None:
            protocol = append_string(protocol, "Old HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (no model to compare)", protocol
        elif self.old_hyrodyn is None:
            protocol = append_string(protocol, "Old HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (no hyrodyn model to compare)", protocol

        protocol = append_string(protocol, "Compare COM position:", loglevel="info")
        protocol = append_string(protocol, f"  COM of old robot = {self.old_hyrodyn.com}", loglevel="info")
        protocol = append_string(protocol, f"  COM of new robot = {self.new_hyrodyn.com}", loglevel="info")
        diff = self.old_hyrodyn.com - self.new_hyrodyn.com
        value = np.linalg.norm(diff)
        check = value < (self.new.test["tolerances"]["distance"] and
                         not any([np.isnan(x) for x in diff.reshape((diff.size,))]))
        protocol = append_string(protocol, f"  Success? {check} Diff: {value}", loglevel="info")
        return check, protocol

    def test_hyrodyn_compare_torques(self):
        protocol = ""
        if not HYRODYN_AVAILABLE:
            protocol = append_string('Hyrodyn not present', loglevel="info")
            return "skipped (no HyRoDyn)", protocol
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            protocol = append_string("New HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (HyRoDyn model not loaded)", protocol
        if self.old_hyrodyn is None and self.old is None:
            protocol = append_string("Old HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (no model to compare)", protocol
        elif self.old_hyrodyn is None:
            protocol = append_string("Old HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (no hyrodyn model to compare)", protocol

        protocol = append_string("Compare joint torques:", loglevel="info")
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
            protocol = append_string("Name                          \tOld        \tNew        \tDiff", loglevel="info")
            for k, v in torques.items():
                diff = v["O"] - v["N"] if v["O"] is not None and v["N"] is not None else None
                name = "".join([k[j] if j < len(k) else " " for j in range(30)])
                protocol = append_string("{}\t{}\t{}\t{}\t{}".format(
                    name,
                    "{:f}".format(v["O"]) if v["O"] is not None else "   ----   ",
                    "{:f}".format(v["N"]) if v["N"] is not None else "   ----   ",
                    "{:f}".format(diff) if v["O"] is not None and v["N"] is not None else "   ----   ",
                    "!!!" if diff is None or np.abs(diff) > self.new.test["tolerances"]["default"] or np.isnan(diff) else ""
                ), loglevel="info")
            if self.old_hyrodyn.Tau_actuated.shape == self.new_hyrodyn.Tau_actuated.shape:
                diff = np.abs(self.old_hyrodyn.Tau_actuated - self.new_hyrodyn.Tau_actuated)
                value = np.amax(diff)
                check = value < (self.new.test["tolerances"]["default"] and
                                 not any([np.isnan(x) for x in diff.reshape((diff.size,))]))
            else:
                value = None
                check = "skipped (not the same joints)"
            protocol = append_string(f"  Success? {check} Diff: {value}", loglevel="info")
            return check, protocol
        except Exception as e:
            protocol = append_string(f"  Failed due to error: {e}", loglevel="error")
            return False, protocol

    def test_hyrodyn_compare_link_positions(self, end_effectors):
        protocol = ""
        if not HYRODYN_AVAILABLE:
            protocol = append_string(protocol, 'Hyrodyn not present', loglevel="info")
            return "skipped (no HyRoDyn)", protocol
        if self.old_hyrodyn is None:
            self._load_old_hyrodyn_model()
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            protocol = append_string(protocol, "New HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (HyRoDyn model not loaded)", protocol
        if self.old_hyrodyn is None and self.old is None:
            protocol = append_string(protocol, "Old HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (no model to compare)", protocol
        elif self.old_hyrodyn is None:
            protocol = append_string(protocol, "Old HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (no hyrodyn model to compare)", protocol

        succ = True

        protocol = append_string(protocol, "Compare EE positions:", loglevel="info")
        for ee in end_effectors:
            if ee not in [link.name for link in self.old.links]:
                protocol = append_string(protocol, f"{ee} not found in compare model. Skipping comparison", loglevel="error")
                continue
            self.old_hyrodyn.calculate_forward_kinematics(ee)
            old_pose = deepcopy(self.old_hyrodyn.pose)
            self.new_hyrodyn.calculate_forward_kinematics(ee)
            new_pose = deepcopy(self.new_hyrodyn.pose)

            protocol = append_string(protocol, f"ee pose of old model {old_pose}", loglevel="info")
            protocol = append_string(protocol, f"ee pose of new model {new_pose}", loglevel="info")
            value = np.linalg.norm(old_pose[0, 0:3] - new_pose[0, 0:3])
            check = value < self.new.test["tolerances"]["distance"]
            protocol = append_string(protocol, f"  Success? {check} Diff: {value}", loglevel="info")
            succ &= check
        return succ, protocol

    def test_hyrodyn_symmetry_check(self, left_right_end_effectors):
        protocol = ""
        if not HYRODYN_AVAILABLE:
            protocol = append_string('Hyrodyn not present', loglevel="info")
            return True, protocol
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            protocol = append_string("New HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (HyRoDyn model not loaded)", protocol

        for ee in left_right_end_effectors:
            if self.new.robot.get_link_id(ee) is None:
                protocol = append_string(f"Existing links: {[link.name for link in self.new.robot.links]}", loglevel="info")
                raise AssertionError(ee + " Link does not exist in the newly exported model!")

        self.new_hyrodyn.calculate_forward_kinematics(left_right_end_effectors[0])
        left_pose = deepcopy(self.new_hyrodyn.pose)
        self.new_hyrodyn.calculate_forward_kinematics(left_right_end_effectors[1])
        right_pose = deepcopy(self.new_hyrodyn.pose)

        protocol = append_string("Symmetry Check:", loglevel="info")
        protocol = append_string(f"      Right EE of new model {right_pose[0, 0:3]}", loglevel="info")
        mirrored_left_pose = np.array([left_pose[0, 0],
                                       -left_pose[0, 1],
                                       left_pose[0, 2],
                                       left_pose[0, 3],
                                       -left_pose[0, 4],
                                       left_pose[0, 5],
                                       left_pose[0, 6]]).reshape(1, 7)
        protocol = append_string(f"  (-y) Left EE of new model {mirrored_left_pose[0, 0:3]}", loglevel="info")
        value = np.linalg.norm(right_pose[0, 0:3] - mirrored_left_pose[0, 0:3])
        check = value < self.new.test["tolerances"]["distance"]
        protocol = append_string(f"  Success? {check} Diff: {value}", loglevel="info")
        protocol = append_string("!!! Check ignores orientation !!!", loglevel="info")
        return check, protocol

    def test_hyrodyn_symmetry_check_torques(self, left_right_end_effectors):
        protocol = ""
        if not HYRODYN_AVAILABLE:
            protocol = append_string('Hyrodyn not present', loglevel="info")
            return True, protocol
        if self.new_hyrodyn is None and self.new_hml_test[2] == 0:
            self.test_hyrodyn_load_in_hyrodyn()
        if self.new_hyrodyn is None:
            protocol = append_string("New HyRoDyn model not present! Skipping test!", loglevel="info")
            return "skipped (HyRoDyn model not loaded)", protocol

        for ee in left_right_end_effectors:
            if self.new.robot.get_link_id(ee) is None:
                protocol = append_string(f"Existing links: {[link.name for link in self.new.robot.links]}", loglevel="error")
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
        protocol = append_string("Symmetry Check Torques:", loglevel="info")
        protocol = append_string(f"Calculated inverse dynamics for pose: {self.new_hyrodyn.y}", loglevel="info")
        success = True
        max_len = np.array([len(left[i] + "/" + right[i]) for i in range(len(left))]).max(initial=0)
        for i in range(len(left)):
            left_torque = torques[left[i]]
            right_torque = torques[right[i]]
            diff = left_torque - right_torque
            protocol = append_string("{}\t{}\t{}\t{}\t{}".format(
                left[i] + "/" + right[i] + " " * (max_len - (len(left[i] + "/" + right[i]))),
                "{:f}".format(left_torque),
                "{:f}".format(right_torque),
                "{:f}".format(diff),
                "!!!" if diff is None or np.abs(diff) > self.new.test["tolerances"]["default"] or np.isnan(diff) else ""
            ), loglevel="info")
            success &= not (np.abs(diff) > self.new.test["tolerances"]["default"] or np.isnan(diff))
        protocol = append_string(f"  Success? {success}", loglevel="info")
        return success, protocol

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

