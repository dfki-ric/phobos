from copy import deepcopy, copy
import os

import yaml

from .poses import JointPoseSet
from ..io import sensor_representations
from ..io import representation
from ..io.xmlrobot import XMLRobot
from ..io.parser import parse_xml
from .hyrodyn import Submechanism, Exoskeleton
from ..geometry import import_mesh
from ..utils import tree


class SMURFRobot(XMLRobot):
    def __init__(self, name=None, xmlfile=None, submechanisms_file=None, smurffile=None, verify_meshes_on_import=True,
                 inputfile=None, description=None):
        self.name = None
        self.smurffile = None
        self.submechanisms_file = None
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

        if inputfile is not None:
            if inputfile.lower().endswith(".smurf") and smurffile is None:
                smurffile = inputfile
            elif inputfile.lower().endswith(".urdf") and xmlfile is None:
                xmlfile = inputfile
        self.xmlfile = xmlfile
        self.smurffile = smurffile
        self.submechanisms_file = submechanisms_file

        if self.smurffile is not None:
            # Check the input file
            self.read_smurffile(self.smurffile)

        super(SMURFRobot, self).__init__(xmlfile=self.xmlfile)
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

        self.link_entities()

        if verify_meshes_on_import:
            self.verify_meshes()

        if len(self.links) > 0 and len(self.joints) > 0:
            self.joints = self.get_joints_ordered_df()

        if self.name is None and self.xmlfile is not None:
            self.name, _ = os.path.splitext(self.xmlfile)

    # helper methods
    def link_entities(self):
        super(SMURFRobot, self).link_entities()
        for entity in self.submechanisms + self.exoskeletons + self.motors + self.poses:
            entity.link_with_robot(self)

    def unlink_entities(self):
        super(SMURFRobot, self).unlink_entities()
        for entity in self.submechanisms + self.exoskeletons + self.motors + self.poses:
            entity.unlink_from_robot()

    def read_smurffile(self, smurffile):
        if smurffile is not None:
            self.smurffile = os.path.abspath(smurffile)
            with open(smurffile, 'r') as stream:
                smurf_dict = yaml.safe_load(stream)
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
                annotation = yaml.safe_load(stream.read())
                if "submechanisms" in annotation.keys():
                    self.submechanisms_file = os.path.abspath(annotationfile)
                self.annotations.update(annotation)
            except yaml.YAMLError as exc:
                print(exc)

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
                            name=motor['name'] if 'name' in motor else motor['joint']+"_motor",
                            **annotations
                        )
                    )
                else:
                    print(motor)
                    print("ERROR: There is no joint to which the above motor definition relates. Skipping...")

        if 'sensors' in self.annotations:
            for sensor_def in self.annotations['sensors']:
                # Search for the joint or link
                input_args = {}
                existing = self.get_sensor(sensor_def["name"])
                sensor = getattr(sensor_representations, sensor_def["type"])(**sensor_def)
                if existing is not None and existing != sensor:
                    print("WARNING: There is already a sensor with name", sensor_def["name"])
                elif existing is None:
                    self.add_sensor(sensor)

        if 'poses' in self.annotations:
            for pose in self.annotations['poses']:
                self.add_aggregate(
                    'poses',
                    JointPoseSet(robot=self, name=pose['name'], configuration=pose['joints'])
                )

        if 'joint' in self.annotations:
            for joint in self.annotations['joint']:
                joint_instance = self.get_joint(joint['name'])
                if joint_instance is not None:
                    joint_instance.add_annotations(overwrite=False, **joint)

        if 'link' in self.annotations:
            for link in self.annotations['link']:
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

    def _rename(self, targettype, target, new_name, further_targettypes=None):
        other_targettypes = ["motors", "poses", "submechanisms", "exoskeletons"]
        if further_targettypes is not None:
            other_targettypes += further_targettypes
        return super(SMURFRobot, self)._rename(targettype, target, new_name, further_targettypes=other_targettypes)

    # getters
    def get_motor(self, motorname):
        """Returns the ID (index in the motor list) of the motor(s).
        """

        if isinstance(motorname, list):
            return [self.get_motor(motor) for motor in motorname]

        return self.get_instance('motors', motorname)

    # tools
    def verify_meshes(self):
        no_problems = True
        for link in self.links:
            for vc in link.collisions + link.visuals:
                if hasattr(vc.geometry, "filename") and \
                        import_mesh(vc.geometry.filename, urdf_path=self.xmlfile) is None:
                    print("WARNING: Mesh file", vc.geometry.filename,
                          "is empty and therefore the corresponding visual/geometry removed!")
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
        if not isinstance(motor, representation.Motor):
            raise Exception("Please provide an instance of Motor to attach.")
        # Check if the motor already contains joint information
        self.add_aggregate("motor", motor)
        return

    def add_pose(self, pose):
        """Add a new pose to the robot.
        """
        if not isinstance(pose, JointPoseSet):
            raise Exception("Please provide an instance of Pose to add.")
        self.add_aggregate('poses', pose)
        return

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

    def get_joints_ordered_df(self):
        """Returns the joints in depth first order"""
        indep_joints = []
        for sm in self.submechanisms:
            indep_joints += sm.jointnames_independent
        return tree.get_joints_depth_first(self, self.get_root(), independent_joints=list(set(indep_joints)) if len(indep_joints) > 0 else None)

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
        ToDo: check internal and close fixed joints
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

    def load_submechanisms(self, submechanism_definition, add=False):
        """
        Loads the given submechanisms definition and creates instances of Submechanism or Exoskeleton for each entry
        :param submechanism_definition: The submechanism dict given in the file.
        :param add: Whether the loaded definition should be added to the exiting ones. (Not recommended as this might
        lead to joints that are included in multiple definitions.
        :return: None
        """
        if type(submechanism_definition) == str and os.path.isfile(submechanism_definition):
            submechanism_definition = yaml.safe_load(open(submechanism_definition, "r").read())
        if not add:
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
            print("WARNING: Loading submechanisms from list. This list is interpreted as list of nothing but"
                  " submechanisms. This means no exoskeletons will be created.")
            _submechs = submechanism_definition
        else:
            raise TypeError("submechanism_definition is neither list nor dict")

        for sm in _submechs:
            self.add_aggregate("submechanism", Submechanism(**sm))
        for ex in _exoskels:
            self.add_aggregate("exoskeletons", Exoskeleton(**ex))

    def sort_submechanisms(self):
        """
        Sorts the submechanisms as well as there jointname lists
        :return: None
        """
        sorted_joints = [jn.name for jn in self.get_joints_ordered_df()]
        sorted_links = [jn.name for jn in self.get_links_ordered_df()]
        self.submechanisms = sorted(self.submechanisms, key=lambda submech: sorted_links.index(submech.get_root(self)))
        self.exoskeletons = sorted(self.exoskeletons, key=lambda submech: sorted_links.index(submech.get_root(self)))
        for sm in self.submechanisms + self.exoskeletons:
            for key in ["jointnames", "jointnames_spanningtree", "jointnames_active", "jointnames_independent", "jointnames_dependent"]:
                if hasattr(sm, key):
                    setattr(sm, key, sorted(set(getattr(sm, key)),
                                            key=lambda jn: sorted_joints.index(jn)) if getattr(sm, key) is not None else None)

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

    def fill_submechanisms(self):
        """
        Scans the defined submechanisms and creates the entries for missing joints
        :return: None
        """
        missing_joints = self._get_joints_not_included_in_submechanisms()
        sorted_joints = [jn.name for jn in self.get_joints_ordered_df()]
        for jointname in missing_joints:
            joint = self.get_joint(jointname)
            joint_idx = sorted_joints.index(jointname)
            inserted = False
            if joint.joint_type == "fixed":
                # If it's just a fixed joint we might be able to add this to an existing submechanisms
                for sm in self.submechanisms + self.exoskeletons:
                    for jn in sm.jointnames:
                        if sorted_joints.index(jn) == joint_idx + 1:
                            inserted = True
                            sm.jointnames.insert(sm.jointnames.index(jn), jointname)
                            break
                        elif sorted_joints.index(jn) == joint_idx - 1:
                            inserted = True
                            sm.jointnames.insert(sm.jointnames.index(jn) + 1, jointname)
                            break
                    if inserted:
                        break
            if not inserted:
                # If we have not already inserted this joint (fixed) let's create a serial mechanism for it
                jn_spanningtree = jn_independent = jn_active = [] if joint.joint_type == "fixed" else [jointname]
                jn = jn_spanningtree if joint.joint_type != "fixed" else [jointname]
                self.add_aggregate("submechanism", Submechanism(
                    name="serial",
                    contextual_name="serial",
                    type="serial",
                    jointnames_active=jn_active,
                    jointnames_independent=jn_independent,
                    jointnames_spanningtree=jn_spanningtree,
                    jointnames=jn,
                    auto_gen=True
                ), silent=True)
        self.sort_submechanisms()
        # Now we merge all serial mechanisms to reduce the number of mechanisms
        new_submechanisms = []
        for sm in self.submechanisms:
            if sm.auto_gen and len(new_submechanisms) > 0 and new_submechanisms[-1].auto_gen:
                new_submechanisms[-1].jointnames = list(set(new_submechanisms[-1].jointnames + sm.jointnames))
                new_submechanisms[-1].jointnames_active = list(set(new_submechanisms[-1].jointnames_active + sm.jointnames_active))
                new_submechanisms[-1].jointnames_independent = list(set(new_submechanisms[-1].jointnames_independent + sm.jointnames_independent))
                new_submechanisms[-1].jointnames_spanningtree = list(set(new_submechanisms[-1].jointnames_spanningtree + sm.jointnames_spanningtree))
            else:
                new_submechanisms.append(sm)
        self.submechanisms = new_submechanisms
        self.sort_submechanisms()
        counter = 0
        for sm in self.submechanisms:
            if sm.auto_gen:
                sm.name = "serial_chain"
                sm.contextual_name = "serial_chain" + str(counter)
                counter += 1
