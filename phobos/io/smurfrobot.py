from copy import deepcopy, copy
import os


from .poses import JointPoseSet
from ..io import sensor_representations
from ..io import representation
from ..io.xmlrobot import XMLRobot
from ..io.parser import parse_xml
from .hyrodyn import Submechanism, Exoskeleton
from ..geometry import import_mesh
from ..utils import tree
from ..defs import load_json, dump_json, dump_yaml
from ..utils.transform import inv
from ..utils.commandline_logging import get_logger
log = get_logger(__name__)


class SMURFRobot(XMLRobot):
    def __init__(self, name=None, xmlfile=None, submechanisms_file=None, smurffile=None, verify_meshes_on_import=True,
                 inputfile=None, description=None, autogenerate_submechanisms=None, is_human=False):
        self.name = name
        self.smurffile = None
        self.submechanisms_file = None
        self.autogenerate_submechanisms = autogenerate_submechanisms
        self.inputfiles = []
        self.annotations = {}
        self.named_annotations = {}
        # Smurf Informations
        self.motors = []
        self.poses = []
        self.description = None
        # Hyrodyn stuff
        self.submechanisms = []
        self.exoskeletons = []
        # Modular stuff
        self.interfaces = []

        if inputfile is not None:
            if inputfile.lower().endswith(".smurf") and smurffile is None:
                smurffile = inputfile
            elif inputfile.lower().endswith(".urdf") or inputfile.lower().endswith(".sdf") and xmlfile is None:
                xmlfile = inputfile
            else:
                raise ValueError("Can't parse robot format: "+inputfile.lower().split(".")[-1])
        self.xmlfile = xmlfile
        self.smurffile = smurffile
        self.submechanisms_file = submechanisms_file

        if self.smurffile is not None:
            # Check the input file
            self.read_smurffile(self.smurffile)

        super(SMURFRobot, self).__init__(xmlfile=self.xmlfile, is_human=is_human)
        if self.xmlfile is not None:
            # Fill everything with the xml information
            base_robot = parse_xml(self.xmlfile)
            assert type(base_robot) == XMLRobot, f"{type(base_robot)}"
            for k, v in base_robot.__dict__.items():
                setattr(self, k, v)

        self.description = "" if description is None else description

        if self.submechanisms_file is not None:
            self.inputfiles.append(self.submechanisms_file)
        for f in self.inputfiles:
            self._parse_annotations(f)
        self._init_annotations()
        if is_human:
            self.annotate_as_human()
        self.link_entities()

        if verify_meshes_on_import:
            self.verify_meshes()

        if len(self.links) > 0 and len(self.joints) > 0:
            self.joints = self.get_joints_ordered_df()

        if self.name is None and self.xmlfile is not None:
            self.name, _ = os.path.splitext(self.xmlfile)

    # helper methods
    def link_entities(self, check_linkage_later=False):
        super(SMURFRobot, self).link_entities(check_linkage_later=True)
        for entity in self.submechanisms + self.exoskeletons + self.motors + self.poses + self.interfaces:
            entity.link_with_robot(self, check_linkage_later=True)
        if not check_linkage_later:
            self.check_linkage()

    def unlink_entities(self, check_linkage_later=False):
        super(SMURFRobot, self).unlink_entities(check_linkage_later=True)
        for entity in self.submechanisms + self.exoskeletons + self.motors + self.poses + self.interfaces:
            entity.unlink_from_robot(check_linkage_later=True)
        if not check_linkage_later:
            self.check_unlinkage()

    def check_linkage(self):
        super(SMURFRobot, self).check_linkage()
        for entity in self.submechanisms + self.exoskeletons + self.motors + self.poses + self.interfaces:
            entity.check_linkage()

    def check_unlinkage(self):
        super(SMURFRobot, self).check_unlinkage()
        for entity in self.submechanisms + self.exoskeletons + self.motors + self.poses + self.interfaces:
            entity.check_unlinkage()

    def read_smurffile(self, smurffile):
        if smurffile is not None:
            self.smurffile = os.path.abspath(smurffile)
            with open(smurffile, 'r') as stream:
                smurf_dict = load_json(stream)
                self.name = smurf_dict['modelname']
                self.inputfiles = smurf_dict['files']

                # Process the file to get the absolute path
                for i, f in enumerate(self.inputfiles):
                    if not os.path.isabs(f):
                        self.inputfiles[i] = os.path.join(os.path.dirname(self.smurffile), f)
        if self.inputfiles is not None:
            for f in self.inputfiles:
                # Get abs
                if f.lower().endswith('.urdf'):
                    self.xmlfile = os.path.abspath(f)
                    self.inputfiles.remove(f)

    def _parse_annotations(self, annotationfile):
        # Load the file
        with open(annotationfile, 'r') as stream:
            try:
                annotation = load_json(stream.read())
                if "submechanisms" in annotation.keys():
                    self.submechanisms_file = os.path.abspath(annotationfile)
                self.annotations.update(annotation)
            except Exception as exc:
                log.error(exc)

    def _init_annotations(self):
        if 'motors' in self.annotations:
            for motor in self.annotations['motors']:
                # Search for the joint
                joint = self.get_joint(motor['joint'])
                if joint is not None:
                    annotations = deepcopy(motor)
                    if "name" in annotations.keys():
                        annotations.pop('name')
                    self.add_aggregate(
                        'motors',
                        representation.Motor(
                            name=motor['name'] if 'name' in motor else motor['joint'] + "_motor",
                            **annotations
                        )
                    )
                else:
                    log.error(motor)
                    log.error("There is no joint to which the above motor definition relates. Skipping...")

        if 'sensors' in self.annotations:
            for sensor_def in self.annotations['sensors']:
                # Search for the joint or link
                input_args = {}
                existing = self.get_sensor(sensor_def["name"])
                sensor = getattr(sensor_representations, sensor_def["type"])(**sensor_def)
                if existing is not None and not existing.equivalent(sensor):
                    log.debug(f"Replacing existing sensor with name {sensor_def['name']}\n"
                                f"existing: {existing.to_yaml()}\n"
                                f"new: {sensor.to_yaml()}")
                    self.remove_aggregate("sensors", existing)
                    self.add_sensor(sensor)
                elif existing is None:
                    self.add_sensor(sensor)

        if 'poses' in self.annotations:
            for pose in self.annotations['poses']:
                self.add_aggregate(
                    'poses',
                    JointPoseSet(robot=self, name=pose['name'], configuration=pose['joints'])
                )

        if 'joints' in self.annotations:
            for joint in self.annotations['joints']:
                joint_instance = self.get_joint(joint['name'])
                if joint_instance is not None:
                    joint_instance.add_annotations(overwrite=False, **joint)

        if 'links' in self.annotations:
            for link in self.annotations['links']:
                link_instance = self.get_link(link['name'])
                if link_instance is not None:
                    link_instance.add_annotations(overwrite=False, **link)

        if 'materials' in self.annotations:
            for material in self.annotations['materials']:
                mat_instance = self.get_material(material['name'])
                if mat_instance is not None:
                    mat_instance.add_annotations(overwrite=False, **material)

        if 'submechanisms' in self.annotations:
            for submech in self.annotations['submechanisms']:
                self.add_aggregate(
                    'submechanisms',
                    Submechanism(**submech)
                )

        if 'exoskeletons' in self.annotations:
            for exo in self.annotations['exoskeletons']:
                self.add_aggregate(
                    'exoskeletons',
                    Exoskeleton(**exo)
                )

        if 'interfaces' in self.annotations:
            for interf in self.annotations['interfaces']:
                self.add_aggregate(
                    'interfaces',
                    representation.Interface(**interf)
                )

    def _rename(self, targettype, target, new_name, further_targettypes=None):
        other_targettypes = ["motors", "poses", "submechanisms", "exoskeletons", "interfaces"]
        if further_targettypes is not None:
            other_targettypes += further_targettypes
        return super(SMURFRobot, self)._rename(targettype, target, new_name, further_targettypes=other_targettypes)

    def remove_aggregate(self, typeName, elem):
        if type(elem) == str:
            elem = self.get_aggregate(typeName, elem)
        if elem is None:
            return
        if typeName in "motors":
            elem.joint.motor = None
            super(SMURFRobot, self).remove_aggregate(typeName, elem)
        elif typeName in "joints":
            super(SMURFRobot, self).remove_aggregate(typeName, elem)
            # motors
            self.motors = [m for m in self.motors if m.joint != str(elem)]
            # interfaces
            for interf in self.interfaces:
                if interf.parent == elem.child:
                    interf.origin = representation.Pose.from_matrix(elem.origin.to_matrix().dot(interf.origin.to_matrix()))
                    interf.parent = elem.parent
            # poses
            poses_to_remove = []
            for p in self.poses:
                p.remove_joint(str(elem))
                if len(p.configuration) == 0:
                    poses_to_remove.append(p)
            for p in poses_to_remove:
                self.poses.remove(p)
            # hyrodyn
            for sub in self.submechanisms + self.exoskeletons:
                for key in ["jointnames", "jointnames_spanningtree", "jointnames_independent", "jointnames_active", "jointnames_dependent"]:
                    if hasattr(sub, key) and getattr(sub, key) is not None and str(elem) in getattr(sub, key):
                        setattr(sub, key, [j for j in getattr(sub, key) if j != str(elem)])
            self.submechanisms = [sm for sm in self.submechanisms if not sm.is_empty()]
            self.exoskeletons = [sm for sm in self.exoskeletons if not sm.is_empty()]
        elif typeName in "links":
            super(SMURFRobot, self).remove_aggregate(typeName, elem)
            # interfaces
            self.interfaces = [m for m in self.interfaces if str(m.parent) != str(elem)]
        else:
            super(SMURFRobot, self).remove_aggregate(typeName, elem)

    # getters
    def get_motor(self, motor_name) -> [representation.Motor, list]:
        """Returns the ID (index in the motor list) of the motor(s).
        """
        if isinstance(motor_name, list):
            return [self.get_motor(motor) for motor in motor_name]

        return self.get_aggregate('motors', motor_name)

    # tools
    def verify_meshes(self):
        no_problems = True
        for link in self.links:
            for vc in link.collisions + link.visuals:
                if isinstance(vc.geometry, representation.Mesh) and \
                        import_mesh(vc.geometry.filename, urdf_path=self.xmlfile) is None:
                    log.info(f"Mesh file {vc.geometry.filename} is empty and therefore the corresponding visual/geometry removed!")
                    no_problems = False
                    link.remove_aggregate(vc)
        return no_problems

    def add_named_annotation(self, name, content):
        if name in self.named_annotations.keys():
            raise NameError("A named annotation with the name " + name +
                            " does already exist. Please merge or rename your annotations!")
        else:
            self.named_annotations[name] = content

    def add_motor(self, motor):
        """Attach a new motor to the robot. Either the joint is already defined inside the motor
        or a jointname is given. Renames the motor if already given.
        """
        if isinstance(motor, list):
            return [self.add_motor(m) for m in motor]
        if not isinstance(motor, representation.Motor):
            raise Exception(f"Please provide an instance of Motor to attach. Got {type(motor)}")
        # Check if the motor already contains joint information
        joint = self.get_joint(motor.joint)
        assert joint is not None
        motor.link_with_robot(self)
        self.add_aggregate("motors", motor)
        joint.motor = motor

    def add_pose(self, pose):
        """Add a new pose to the robot.
        """
        if not isinstance(pose, JointPoseSet):
            raise Exception("Please provide an instance of Pose to add.")
        self.add_aggregate('poses', pose)

    def set_bitmask(self, linkname, bitmask, collisionname=None, **kwargs):
        """Set the bitmask used for collisiondetection for the corresponding link. If no 'collisionname'(s) are given,
        all collisions of the corresponding link are set to the bitmask.
        """
        assert type(linkname) is str
        link = self.get_link(linkname)
        assert link is not None

        if collisionname is None:
            self.set_bitmask(linkname, bitmask, collisionname=[c.name for c in link.collisions], **kwargs)
            return

        if isinstance(collisionname, list):
            for c in collisionname:
                self.set_bitmask(linkname, bitmask, collisionname=c, **kwargs)
            return

        for c in link.collisions:
            if c.name == collisionname:
                c.add_annotations(bitmask=bitmask, overwrite=True)
                break

    # Reimplementation of Robot methods
    def get_joints_ordered_df(self, ignore_indep=False):
        """Returns the joints in depth first order"""
        indep_joints = []
        for sm in self.submechanisms:
            indep_joints += sm.jointnames_independent
        return tree.get_joints_depth_first(self, self.get_root(), independent_joints=None if ignore_indep else list(set(indep_joints)) if len(indep_joints) > 0 else None)

    # submechanism related
    def create_submechanism(self, name, definition):
        """ Create a submechanism with a give name and root from the definition ( which is a dict ).
        The dict contains the definition e.g. given as

            type: "2SPU+1U"
            name : three_revolute_joints
            jointnames_independent: ["BodyPitch", "BodyRoll"]
            jointnames_spanningtree: ["BodyPitch", "BodyRoll", "Body_B11", "Body_B12", "Body_Act1",
                                      "Body_B21", "Body_B22", "Body_Act2"]
            jointnames_active: ["Body_Act1", "Body_Act2"]
        """
        assert isinstance(definition, dict)
        # Check if the information given is enough
        needed_keys = ["type", "name", "jointnames_independent", "jointnames_spanningtree", "jointnames_active"]
        assert all(k in definition.keys() for k in needed_keys)
        # Check if all jointnames are in the spanningtree
        jointnames = definition.get('jointnames_spanningtree')
        m_joints = [j.name for j in self.joints]
        if not all(k in m_joints for k in jointnames):
            set(jointnames) - set(m_joints)
            raise ValueError("These jointnames: {} do not belong to the robot!".format(set(jointnames) - set(m_joints)))
        # Get the unique names
        unique_names = list(set(jointnames))

        sort_defs = {
            "jointnames_active": definition["jointnames_active"],
            "jointnames_spanningtree": unique_names,
            "jointnames_independent": definition["jointnames_independent"],
            "file_path": definition["file_path"] if "file_path" in definition.keys() else None,
            "contextual_name": name,
            "type": definition["type"],
            "name": definition["name"]
        }
        self.submechanisms += [Submechanism(**sort_defs)]
        return

    def load_submechanisms(self, submechanism_definition, replace_only_conflicting=False):
        """
        Loads the given submechanisms definition and creates instances of Submechanism or Exoskeleton for each entry
        :param submechanism_definition: The submechanism dict given in the file.
        :param replace_only_conflicting: Whether the loaded definition should be added to the exiting ones. (Not recommended as this might
        lead to joints that are included in multiple definitions.
        :return: None
        """
        if type(submechanism_definition) == str and os.path.isfile(submechanism_definition):
            submechanism_definition = load_json(open(submechanism_definition, "r").read())
        if not replace_only_conflicting:
            self.submechanisms = []
            self.exoskeletons = []
        _submechs = []
        _exoskels = []
        if type(submechanism_definition) == dict:
            if "submechanisms" in submechanism_definition.keys():
                _submechs = submechanism_definition["submechanisms"]
            if "exoskeletons" in submechanism_definition.keys():
                _exoskels = submechanism_definition["exoskeletons"]
        elif type(submechanism_definition) == list:
            log.warning("Loading submechanisms from list. This list is interpreted as list of nothing but "
                        "submechanisms. This means no exoskeletons will be created.")
            _submechs = submechanism_definition
        else:
            raise TypeError("submechanism_definition is neither list nor dict")

        for sm in _submechs:
            _sm = Submechanism(**sm)
            if replace_only_conflicting:
                for existing in self.submechanisms:
                    if set(existing.get_joints()) & set(_sm.get_joints()):
                        self.submechanisms.remove(existing)
            self.add_aggregate("submechanism", _sm)
        for ex in _exoskels:
            _ex = Exoskeleton(**ex)
            if replace_only_conflicting:
                for existing in self.exoskeletons:
                    if set(existing.get_joints()) & set(_ex.get_joints()):
                        self.exoskeletons.remove(existing)
            self.add_aggregate("exoskeletons", _ex)

    def sort_submechanisms(self):
        """
        Sorts the submechanisms as well as there jointname lists
        :return: None
        """
        sorted_joints = [jn.name for jn in self.get_joints_ordered_df()]
        # sorted_links = [jn.name for jn in self.get_links_ordered_df()]
        for x in self.submechanisms:
            assert len(x.get_root_joints(self)) > 0, x.name
        self.submechanisms = sorted(self.submechanisms, key=lambda submech: sorted_joints.index(submech.get_root_joints(self)[0]))
        for sm in self.submechanisms + self.exoskeletons:
            for key in ["jointnames", "jointnames_spanningtree", "jointnames_active", "jointnames_independent", "jointnames_dependent"]:
                if hasattr(sm, key) and getattr(sm, key) is not None:
                    setattr(sm, key, sorted(set(getattr(sm, key)), key=lambda jn: sorted_joints.index(jn)))

    def _get_joints_not_included_in_submechanisms(self):
        """
        Scans the joints not included in any submechanism definition
        :return: A list of joints that are not included in the submechanism definition
        """
        joints = [j.name for j in self.joints]
        sm_joints = []
        for sm in self.submechanisms + self.exoskeletons:
            sm_joints += sm.get_joints()
        return list(set(joints) - set(sm_joints))

    def _get_joints_included_twice_in_submechanisms(self):
        """
        Scans the joints not included in any submechanism definition
        :return: A list of joints that are not included in the submechanism definition
        """
        sm_joints = []
        twice = set()
        for sm in self.submechanisms + self.exoskeletons:
            twice = set(sm_joints) & set(sm.get_joints())
            sm_joints += sm.get_joints()
        return twice

    def generate_submechanisms(self):
        """
        Scans the defined submechanisms and creates the entries for missing joints
        :return: None
        """
        if len(self.submechanisms) == 0:
            return
        has_exo = len(self.exoskeletons) != 0
        for link in self.links:
            has_exo |= False if link.is_human is None else link.is_human
            if has_exo:
                break
        if has_exo and len(self.exoskeletons) == 0:
            self.exoskeletons = [Exoskeleton(
                name="exo",
                contextual_name="exo",
                around="human",
                file_path=self.xmlfile,
                auto_gen=True,
                jointnames_spanningtree=[], jointnames_dependent=[]
            )]
            log.warning("Currently it's not fully supported to create exokeleton definitions automatically, "
                        "a preliminary version has been created. Make sure to check it before usage.")
        for sm in self.submechanisms + self.exoskeletons:
            sm.regenerate(self)
        missing_joints = self._get_joints_not_included_in_submechanisms()
        sorted_joints = [jn.name for jn in self.get_joints_ordered_df()]
        if len(missing_joints) == 0:
            return
        self.sort_submechanisms()
        for jointname in missing_joints:
            joint = self.get_joint(jointname)
            if joint.joint_type != "fixed" and joint._child.is_human is not True:
                # If we have not already inserted this joint (fixed) let's create a serial mechanism for it
                self.add_aggregate("submechanisms", Submechanism(
                    name="serial",
                    contextual_name="serial",
                    type="serial",
                    jointnames_active=[] if joint.joint_type == "fixed" else [jointname],
                    jointnames_independent=[] if joint.joint_type == "fixed" else [jointname],
                    jointnames_spanningtree=[] if joint.joint_type == "fixed" else [jointname],
                    jointnames=[jointname],
                    auto_gen=True
                ), silent=True)
                # print("Created for", jointname)
        for sm in self.submechanisms + self.exoskeletons:
            sm.regenerate(self)
        self.sort_submechanisms()
        # Now we merge all serial mechanisms to reduce the number of mechanisms
        new_submechanisms = []
        for sm in self.submechanisms:
            if sm.auto_gen and len(new_submechanisms) > 0 and new_submechanisms[-1].auto_gen and self.get_parent(sm.get_root(self), targettype="joint") in new_submechanisms[-1].get_joints():
                new_submechanisms[-1].jointnames = list(set(new_submechanisms[-1].jointnames + sm.jointnames))
                new_submechanisms[-1].jointnames_active = list(
                    set(new_submechanisms[-1].jointnames_active + sm.jointnames_active))
                new_submechanisms[-1].jointnames_independent = list(
                    set(new_submechanisms[-1].jointnames_independent + sm.jointnames_independent))
                new_submechanisms[-1].jointnames_spanningtree = list(
                    set(new_submechanisms[-1].jointnames_spanningtree + sm.jointnames_spanningtree))
            else:
                new_submechanisms.append(sm)
        self.submechanisms = new_submechanisms
        self.sort_submechanisms()
        # As movable joints are now placed, we place the fixed joints
        insertion_happened = True
        while insertion_happened:
            insertion_happened = False
            for jointname in self._get_joints_not_included_in_submechanisms():
                joint = self.get_joint(jointname)
                joint_idx = sorted_joints.index(jointname)
                if joint.joint_type == "fixed" and joint._child.is_human is not True:
                    # If it's just a fixed joint we might be able to add this to an existing submechanisms
                    for sm in self.submechanisms:
                        for jn in sm.get_joints():
                            # print(jointname, jn, sorted_joints.index(jn) == joint_idx + 1, self.get_children(joint.child))
                            if (sorted_joints.index(jn) == joint_idx - 1 or self.get_parent(joint.parent) == jn) and \
                                    (len(self.get_children(joint.parent)) <= 1 or
                                     all([((self.get_joint(c).joint_type == "fixed" and len(self.get_children(self.get_joint(c).child)) == 0) or
                                           c not in self._get_joints_not_included_in_submechanisms())
                                          for c in self.get_children(joint.parent, targettype="joint")])):
                                # place after jn
                                temp = sm.jointnames
                                temp.insert(sm.jointnames.index(jn) + 1, jointname)
                                sm.jointnames = temp
                                insertion_happened = True
                                # print("Inserted", jointname)
                                break
                            elif sorted_joints.index(jn) == joint_idx + 1 and len(self.get_children(joint.parent)) > 1 and jn in self.get_children(joint.child):
                                # place before jn
                                temp = sm.jointnames
                                temp.insert(sm.jointnames.index(jn), jointname)
                                sm.jointnames = temp
                                insertion_happened = True
                                # print("Inserted", jointname)
                                break
                        # print(jointname, jn, sorted_joints.index(jn) == joint_idx - 1, self.get_parent(joint.parent), self.get_children(joint.child), insertion_happened)
                        if insertion_happened:
                            break
                        # else:
                        #     print("Couldn't insert ", jointname, "into", sm.to_yaml())
        for jointname in self._get_joints_not_included_in_submechanisms():
            joint = self.get_joint(jointname)
            if joint._child.is_human is not True:
                # We create submechanisms now for the missing fixed joints
                self.add_aggregate("submechanisms", Submechanism(
                    name="serial",
                    contextual_name="serial",
                    type="serial",
                    jointnames_active=[] if joint.joint_type == "fixed" else [jointname],
                    jointnames_independent=[] if joint.joint_type == "fixed" else [jointname],
                    jointnames_spanningtree=[] if joint.joint_type == "fixed" else [jointname],
                    jointnames=[jointname],
                    auto_gen=True
                ), silent=True)
        self.sort_submechanisms()
        counter = 0
        for sm in self.submechanisms:
           if sm.auto_gen:
               sm.name = "serial_chain"
               sm.contextual_name = "serial_chain" + str(counter)
               counter += 1