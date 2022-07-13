from copy import deepcopy
import os

import yaml

from .motors import Motor
from .poses import JointPositionSet
from ..io import sensors
from ..io.xmlrobot import XMLRobot
from ..io.parser import parse_xml
from .hyrodyn import Submechanism, Exoskeleton
from ..geometry import import_mesh
from ..utils import tree


class SMURFRobot(XMLRobot):
    def __init__(self, xmlfile=None, submechanisms_file=None, smurffile=None, verify_meshes_on_import=True,
                 inputfile=None, description=None):
        if inputfile is not None:
            if inputfile.lower().endswith(".smurf") and smurffile is None:
                smurffile = inputfile
            elif inputfile.lower().endswith(".urdf") and xmlfile is None:
                xmlfile = inputfile
        self.xmlfile = xmlfile
        self.smurffile = smurffile
        self.submechanisms_file = submechanisms_file

        # Empty init for the props
        self.inputfiles = []

        # Check the input file
        self.read_smurffile(self.smurffile)

        # Fill everything with the xml information
        base_robot = parse_xml(self.xmlfile)
        assert type(base_robot) == XMLRobot, f"{type(base_robot)}"
        for k, v in base_robot.__dict__.items():
            self.__dict__[k] = v

        self.annotations = {}
        self.named_annotations = {}

        # Smurf Informations
        self.motors = []
        self.poses = []
        # Hyrodyn stuff
        self.submechanisms = []
        self.exoskeletons = []
        self.hyrodyn_transmissions = []
        self.hyrodyn_loop_constraints = []

        self.description = "" if description is None else description


        for f in self.inputfiles:
            self._parse_annotations(f)
        self._init_annotations()

        for entity in self.submechanisms + self.exoskeletons + self.motors:
            entity.link_with_robot(self)

        if verify_meshes_on_import:
            self.verify_meshes()

        self.joints = self.get_joints_ordered_df()

    # helper methods
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

    def _parse_annotations(self, annotationfile):
        # Load the file
        with open(annotationfile, 'r') as stream:
            try:
                annotation = yaml.safe_load(stream)
                if "submechanisms" in annotation.keys():
                    self.submechanisms_file = os.path.abspath(annotationfile)
            except yaml.YAMLError as exc:
                print(exc)
        self.annotations.update(annotation)

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
                        Motor(
                            robot=self,
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
                sensor = getattr(sensors, sensor_def["sensortype"])(**sensor_def)
                if existing is not None and existing != sensor:
                    print("WARNING: There is already a sensor with name", sensor_def["name"])
                elif existing is None:
                    self.add_sensor(sensor)

        if 'poses' in self.annotations:
            for pose in self.annotations['poses']:
                self.add_aggregate(
                    'poses',
                    JointPositionSet(robot=self, name=pose['name'], configuration=pose['joints'])
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
                    Submechanism(self, **submech)
                )

        if 'exoskeletons' in self.annotations:
            for exo in self.annotations['exoskeletons']:
                self.add_aggregate(
                    'exoskeletons',
                    Exoskeleton(self, **exo)
                )

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

    def add_motor(self, motor, jointname=None):
        """Attach a new motor to the robot. Either the joint is already defined inside the motor
        or a jointname is given. Renames the motor if already given.
        """
        if not isinstance(motor, Motor):
            raise Exception("Please provide an instance of Motor to attach.")
        # Check if the motor already contains joint information
        motor.robot = self
        if not jointname and motor._joint:
            if motor._joint in self.joints:
                self.add_aggregate('motors', motor)
                return
        # Check if the joint is part of the robot if a jointname is given
        elif jointname and self.get_joint_id(jointname):
            motor._joint = self.joints[self.get_joint_id(jointname)]
            self.add_aggregate('motors', motor)
            return
        else:
            raise Exception("Please provide a joint which is part of the model")
        return

    def add_pose(self, pose):
        """Add a new pose to the robot.
        """
        if not isinstance(pose, JointPositionSet):
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
                c.add_annotation(bitmask=bitmask, overwrite=True)
                break

    def add_loop_constraint(self, loop_constraint):
        self.hyrodyn_loop_constraints += [loop_constraint]

    def add_transmission(self, transmission):
        self.hyrodyn_transmissions += [transmission]

    # Reimplementation of Robot methods
    def _rename(self, targettype, target, new_name):
        # ToDo this should be obsolete
        # if targettype.startswith("link"):
        #     for obj in self.links:
        #         if obj.name == target:
        #             obj.name = new_name
        #     for obj in self.sensors + self.get_all_collisions():
        #         if obj.link == target:
        #             obj.link = new_name
        #     for obj in self.hyrodyn_loop_constraints:
        #         if obj.prepredecessor_body == target:
        #             obj.predecessor_body = new_name
        #         if obj.successor_body == target:
        #             obj.successor_body = new_name
        # elif targettype.startswith("joint"):
        #     for obj in self.smurf_joints:
        #         if obj.name == target:
        #             obj.name = new_name
        #     for obj in self.motors + self.poses:
        #         if obj.joint == target:
        #             obj.joint = new_name
        #     for obj in self.exoskeletons + self.submechanisms:
        #         for key in ["jointnames_independent", "jointnames_spanningtree", "jointnames_active", "jointnames"]:
        #             if hasattr(obj, key):
        #                 setattr(obj, key, [j if j != target else new_name for j in getattr(obj, key)])
        #     for obj in self.hyrodyn_loop_constraints:
        #         if obj.cut_joint == target:
        #             obj.cut_joint = new_name
        #     for obj in self.hyrodyn_transmissions:
        #         if obj.joint == target:
        #             obj.joint = new_name
        #         for dep in obj.joint_dependencies:
        #             if dep.joint == target:
        #                 dep.joint = new_name
        # elif targettype.startswith("material"):
        #     for obj in self.smurf_materials:
        #         if obj.name == target:
        #             obj.name = new_name
        # elif targettype.startswith("collision"):
        #     for obj in self.get_all_collisions():
        #         if obj.name == target:
        #             obj.name = new_name
        # elif targettype.startswith("visual"):
        #     pass
        # elif targettype.startswith("motor"):
        #     for obj in self.motors:
        #         if obj.name == target:
        #             obj.name = new_name
        # elif targettype.startswith("sensor"):
        #     for obj in self.sensors:
        #         if obj.name == target:
        #             obj.name = new_name
        # else:
        #     raise NotImplementedError("_rename() not implemented for targettype " + targettype)
        return {target: new_name}

    def get_joints_ordered_df(self):
        """Returns the joints in depth first order"""
        indep_joints = []
        for sm in self.submechanisms:
            indep_joints += sm.jointnames_independent
        return tree.get_joints_depth_first(self, self.get_root(), independent_joints=list(set(indep_joints)))

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
        self.submechanisms += [Submechanism(self, **sort_defs)]
        return

    def load_submechanisms(self, submechanism_definition, add=False):
        """
        Loads the given submechanisms definition and creates instances of Submechanism or Exoskeleton for each entry
        :param submechanism_definition: The submechanism dict given in the file.
        :param add: Whether the loaded definition should be added to the exiting ones. (Not recommended as this might
        lead to joints that are included in multiple definitions.
        :return: None
        """
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
            self.submechanisms += [Submechanism(self, **sm)]
        for ex in _exoskels:
            self.exoskeletons += [Exoskeleton(self, **ex)]

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
            for key in ["jointnames", "jointnames_spanningtree", "jointnames_active", "jointnames_independent"]:
                if hasattr(sm, key):
                    setattr(sm, key, sorted(getattr(sm, key), key=lambda jn: sorted_joints.index(jn)))
        for transmission in self.hyrodyn_transmissions:
            found = False
            for sm in self.submechanisms:
                if sm.contextual_name == transmission.name:
                    sm.multi_joint_dependencies += [transmission]
                    found = True
                    break
            if not found:
                print(transmission.to_yaml())
                raise AssertionError("Couldn't assign transmission")
        for loop_constraint in self.hyrodyn_loop_constraints:
            found = False
            for sm in self.submechanisms:
                links = sm.get_links(self)
                sm.loop_constraints = []
                if all([x in links for x in [loop_constraint.predecessor_body, loop_constraint.successor_body]]):
                    sm.loop_constraints += [loop_constraint]
                    found = True
                    break
            if not found:
                print(loop_constraint.to_yaml())
                raise AssertionError("Couldn't assign loop constraint")

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
                self.submechanisms += [Submechanism(
                    self,
                    name="serial",
                    type="serial",
                    jointnames_active=jn_active,
                    jointnames_independent=jn_independent,
                    jointnames_spanningtree=jn_spanningtree,
                    jointnames=jn,
                    auto_gen=True
                )]
        self.sort_submechanisms()
        # Now we merge all serial mechanisms to reduce the number of mechanisms
        new_submechanisms = []
        for sm in self.submechanisms:
            if sm.auto_gen and len(new_submechanisms) > 0 and new_submechanisms[-1].auto_gen:
                new_submechanisms[-1].jointnames += sm.jointnames
                new_submechanisms[-1].jointnames_active += sm.jointnames_active
                new_submechanisms[-1].jointnames_independent += sm.jointnames_independent
                new_submechanisms[-1].jointnames_spanningtree += sm.jointnames_spanningtree
            else:
                new_submechanisms += [deepcopy(sm)]
        self.submechanisms = new_submechanisms
        self.sort_submechanisms()
        counter = 0
        for sm in self.submechanisms:
            if sm.auto_gen:
                sm.name = "serial_chain" + str(counter)
                sm.contextual_name = "serial_chain" + str(counter)
                counter += 1
