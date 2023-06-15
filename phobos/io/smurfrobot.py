import os
from copy import deepcopy

from .hyrodyn import Submechanism, Exoskeleton
from .poses import JointPoseSet
from ..commandline_logging import get_logger
from ..defs import load_json
from ..io import representation
from ..io import sensor_representations
from ..io.parser import parse_xml
from ..io.xmlrobot import XMLRobot
from ..utils import tree, misc

log = get_logger(__name__)


class SMURFRobot(XMLRobot):
    def __init__(self, name=None, xmlfile=None, submechanisms_file=None, smurffile=None, verify_meshes_on_import=True,
                 inputfile=None, description=None, autogenerate_submechanisms=None, is_human=False):
        self.smurf_annotation_keys = [
            'motors', 'sensors', 'materials', "joints", "links", 'collisions', 'visuals', 'poses',
            "submechanisms", "exoskeletons", "interfaces"
        ]

        self.name = name
        self.smurffile = None
        self.submechanisms_file = None
        self.autogenerate_submechanisms = autogenerate_submechanisms
        self.inputfiles = []
        self.annotations = {}
        self.categorized_annotations = []
        # Smurf Informations
        self.poses = []
        self.description = None
        self.generic_annotations = []
        # Hyrodyn stuff
        self.submechanisms = []
        self.exoskeletons = []
        # Modular stuff
        self.interfaces = []

        if inputfile is None and smurffile is not None:
            inputfile = smurffile
        elif inputfile is None and xmlfile is not None:
            inputfile = xmlfile
        inputfile = misc.sys_path(inputfile)
        if inputfile is not None:
            inputfile = os.path.abspath(inputfile)
            if inputfile.lower().endswith(".smurf"):
                smurffile = inputfile
            elif inputfile.lower().endswith(".urdf") or inputfile.lower().endswith(".sdf"):
                xmlfile = inputfile
            elif os.path.isdir(inputfile):
                relevant_files = []
                content = os.listdir(inputfile)
                relevant_files += [f for f in content if f.rsplit(".", 1)[-1].lower() in ["smurf", "urdf", "sdf"]]
                if any([f for f in relevant_files if f.endswith(".smurf")]) == 1:
                    smurffile = os.path.join(inputfile, [f for f in relevant_files if f.endswith(".smurf")][0])
                    log.info(f"Found smurf file {smurffile} in the input directory!")
                elif "smurf" in content:
                    smurffiles = [f for f in os.listdir(os.path.join(inputfile, "smurf")) if f.endswith("smurf")]
                    if len(smurffiles) == 1:
                        smurffile = os.path.join(inputfile, "smurf", smurffiles[0])
                        log.info(f"Found smurf directory with {smurffile} in the input directory!")
                elif "urdf" in content:
                    urdffiles = [f for f in os.listdir(os.path.join(inputfile, "urdf")) if f.endswith("urdf")]
                    if len(urdffiles) == 1:
                        xmlfile = os.path.join(inputfile, "urdf", urdffiles[0])
                        log.info(f"Found urdf directory with {xmlfile} in the input directory!")
                elif "sdf" in content:
                    sdffiles = [f for f in os.listdir(os.path.join(inputfile, "sdf")) if f.endswith("sdf")]
                    if len(sdffiles) == 1:
                        xmlfile = os.path.join(inputfile, "sdf", sdffiles[0])
                        log.info(f"Found sdf directory with {xmlfile} in the input directory!")
                elif any([f for f in relevant_files if f.endswith(".urdf")]) == 1:
                    xmlfile = os.path.join(inputfile, [f for f in relevant_files if f.endswith("urdf")][0])
                    log.info(f"Found urdf file {xmlfile} in the input directory!")
                elif any([f for f in relevant_files if f.endswith(".sdf")]) == 1:
                    xmlfile = os.path.join(inputfile, [f for f in relevant_files if f.endswith("sdf")][0])
                    log.info(f"Found sdf file {xmlfile} in the input directory!")
                if inputfile is None:
                    raise ValueError("Couldn't find a valid input file in the given directory!")
            else:
                raise ValueError("Can't parse robot from: "+inputfile + ("(can't parse)" if os.path.exists(inputfile) else "(not found)"))

        super(SMURFRobot, self).__init__(is_human=is_human)
        self.xmlfile = misc.sys_path(xmlfile)
        self.smurffile = misc.sys_path(smurffile)
        self.submechanisms_file = misc.sys_path(submechanisms_file)

        if self.smurffile is not None:
            # Check the input file
            self.read_smurffile(self.smurffile)

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

        if len(self.links) > 0 and len(self.joints) > 0:
            self.link_entities()
            self.joints = self.get_joints_ordered_df()

        if verify_meshes_on_import:
            self.verify_meshes()

        if self.name is None and self.xmlfile is not None:
            self.name, _ = os.path.splitext(self.xmlfile)

    # helper methods
    def link_entities(self, check_linkage_later=False):
        super(SMURFRobot, self).link_entities(check_linkage_later=True)
        for entity in self.submechanisms + self.exoskeletons + self.poses + self.interfaces:
            entity.link_with_robot(self, check_linkage_later=True)
        if not check_linkage_later:
            assert self.check_linkage()

    def unlink_entities(self, check_linkage_later=False):
        super(SMURFRobot, self).unlink_entities(check_linkage_later=True)
        for entity in self.submechanisms + self.exoskeletons + self.poses + self.interfaces:
            entity.unlink_from_robot(check_linkage_later=True)
        if not check_linkage_later:
            assert self.check_unlinkage()

    def check_linkage(self):
        out = super(SMURFRobot, self).check_linkage()
        for entity in self.submechanisms + self.exoskeletons + self.poses + self.interfaces:
            out &= entity.check_linkage()
        return out

    def check_unlinkage(self):
        out = super(SMURFRobot, self).check_unlinkage()
        for entity in self.submechanisms + self.exoskeletons + self.poses + self.interfaces:
            out &= entity.check_unlinkage()
        return out

    def read_smurffile(self, smurffile):
        if smurffile is not None:
            self.smurffile = os.path.abspath(smurffile)
            with open(smurffile, 'r') as stream:
                smurf_dict = load_json(stream)
                self.name = smurf_dict['modelname']
                self.inputfiles = smurf_dict['files']

                # Process the file to get the absolute path
                for i, f in enumerate(self.inputfiles):
                    f = misc.sys_path(f)
                    if not os.path.isabs(f):
                        self.inputfiles[i] = os.path.join(os.path.dirname(self.smurffile), f)
        if self.inputfiles is not None:
            xml_found = False
            for f in self.inputfiles:
                # Get abs
                if f.lower().endswith('urdf') or f.lower().endswith('sdf'):
                    if xml_found:
                        raise AssertionError("Multiple kinematics-XML-files (SDF/URDF) given in SMURF-file")
                    xml_found = True
                    self.xmlfile = os.path.abspath(f)
                    self.inputfiles.remove(f)

    # [TODO v2.1.0] Refactor this
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

    # [TODO v2.1.0] Refactor this
    def _init_annotations(self):
        if 'motors' in self.annotations:
            for motor_def in self.annotations['motors']:
                # Search for the joint
                joint = self.get_joint(motor_def['joint'])
                if joint is None:
                    log.error(motor)
                    log.error("There is no joint to which the above motor definition relates. Skipping...")
                    continue
                existing = self.get_motor(motor_def["name"])
                annotations = deepcopy(motor_def)
                if "name" in annotations.keys():
                    annotations.pop('name')
                motor = representation.Motor(
                    name=motor_def.get('name', motor_def['joint'] + "_motor"),
                    **annotations
                )
                if existing is not None and not existing.equivalent(motor):
                    log.debug(f"Replacing existing sensor with name {motor_def['name']}\n"
                              f"existing: {existing.to_yaml()}\n"
                              f"new: {motor.to_yaml()}")
                    self.remove_aggregate("motors", existing)
                    self.add_motor(motor)
                elif existing is None:
                    self.add_motor(motor)

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
                    if "origin" in link:
                        link_instance.origin = representation.Pose(**link.pop("origin"))
                    link_instance.add_annotations(overwrite=False, **link)

        if 'materials' in self.annotations:
            for material in self.annotations['materials']:
                mat_instance = self.get_material(material['name'])
                if mat_instance is not None:
                    mat_instance.add_annotations(overwrite=False, **material)

        if 'visuals' in self.annotations:
            for visual in self.annotations['visuals']:
                vis_instance = self.get_visual_by_name(visual['name'])
                if vis_instance is not None:
                    # [TODO v2.1.0] We should prefer this over the URDF Mesh
                    if isinstance(vis_instance.geometry, representation.Mesh) and type(visual["geometry"]) == dict:
                        vis_instance.geometry.add_annotations(overwrite=True, **visual["geometry"])
                    visual.pop("geometry")
                    vis_instance.add_annotations(overwrite=False, **visual)
                else:
                    log.error(f"There is no visual with name {visual['name']} in this robot.")
                    log.debug(f"But there are: {[str(v) for v in self.visuals]}")

        if 'collisions' in self.annotations:
            for collision in self.annotations['collisions']:
                coll_instance = self.get_collision_by_name(collision['name'])
                if coll_instance is not None:
                    if "geometry" in collision:
                        # [TODO v2.1.0] We should prefer this over the URDF Mesh
                        if isinstance(coll_instance.geometry, representation.Mesh) and type(collision["geometry"]) == dict:
                            coll_instance.geometry.add_annotations(overwrite=True, **collision["geometry"])
                        collision.pop("geometry")
                    coll_instance.add_annotations(overwrite=False, **collision)
                else:
                    log.error(f"There is no collision with name {collision['name']} in this robot.")
                    log.debug(f"But there are: {[str(c) for c in self.collisions]}")

        if 'submechanisms' in self.annotations:
            for submech in self.annotations['submechanisms']:
                if submech.get("auto_gen", False):
                    continue
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

        # [TODO v2.1.0] Check how we can store which properties are defined by which literals to reload them properly from file
        pop_annotations = []
        for k, v in self.annotations.items():
            if k not in self.smurf_annotation_keys:
                pop_annotations.append(k)
                log.info(f"Adding generic annotation of category {k}")
                if type(v) == list:
                    for a in v:
                        self.add_aggregate("generic_annotations", representation.GenericAnnotation(
                            GA_category=k,
                            GA_name=k,
                            **a
                        ))
                elif type(v) == dict:
                    for an, av in v.items():
                        self.add_aggregate("generic_annotations", representation.GenericAnnotation(
                            GA_category=k,
                            GA_name=an,
                            **av
                        ))
        for k in pop_annotations:
            self.annotations.pop(k)

    def _rename(self, targettype, target, new_name, further_targettypes=None):
        other_targettypes = ["poses", "submechanisms", "exoskeletons", "interfaces"]
        if further_targettypes is not None:
            other_targettypes += further_targettypes
        return super(SMURFRobot, self)._rename(targettype, target, new_name, further_targettypes=other_targettypes)

    def remove_aggregate(self, typeName, elem):
        if type(elem) in (list, tuple):
            return [self.remove_aggregate(typeName, e) for e in elem]
        if type(elem) == str:
            elem = self.get_aggregate(typeName, elem)
        if elem is None:
            return
        if typeName in "joints":
            super(SMURFRobot, self).remove_aggregate(typeName, elem)
            # interfaces
            for interf in self.interfaces:
                if interf.parent == elem.child:
                    interf.parent = elem.parent
                    interf.origin = representation.Pose.from_matrix(elem.origin.to_matrix().dot(interf.origin.to_matrix()), relative_to=interf.parent)
                    interf.origin.link_with_robot(self)
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

    # tools
    def verify_meshes(self):
        """
        Checks whether all meshes are available.

        Returns:
            bool
        """
        no_problems = True
        for link in self.links:
            for vc in link.collisions + link.visuals:
                no_problems &= not isinstance(vc.geometry, representation.Mesh) or not vc.geometry.available()
        return no_problems

    # [ToDo v2.1.0] Support here all powers of GenericAnnotation
    def add_categorized_annotation(self, name, content):
        self.categorized_annotations.append(representation.GenericAnnotation(GA_category=name, GA_dict=content))

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
        self.submechanisms = sorted([sm for sm in self.submechanisms if not sm.is_empty()], key=lambda submech: submech.get_index(self))
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
        self.submechanisms = [sm for sm in self.submechanisms if not sm.auto_gen]
        self.exoskeletons = [ex for ex in self.exoskeletons if not ex.auto_gen]
        # All fixed joints inside the submechanisms are handled by now, next we handle remaining joints
        missing_joints = self._get_joints_not_included_in_submechanisms()
        sorted_joints = [jn.name for jn in self.get_joints_ordered_df()]
        if len(missing_joints) == 0:
            self.sort_submechanisms()
            return
        for jointname in missing_joints:
            joint = self.get_joint(jointname)
            if joint.joint_type != "fixed" and not joint._child.is_human and joint.cut_joint is False:
                # Normal moving joints are considered as serial if there is no submechanism given
                self.add_aggregate("submechanisms", Submechanism(
                    name="serial",
                    contextual_name="serial",
                    type="serial",
                    jointnames_active=[jointname],
                    jointnames_independent=[jointname],
                    jointnames_spanningtree=[jointname],
                    jointnames=[jointname],
                    auto_gen=True
                ), silent=True)
        for sm in self.submechanisms + self.exoskeletons:
            sm.regenerate(self, absorb_fixed_upwards=True)
        for sm in self.submechanisms + self.exoskeletons:
            sm.regenerate(self, absorb_fixed_downwards=True)
        # We create submechanisms now for the missing fixed joints
        for jointname in self._get_joints_not_included_in_submechanisms():
            joint = self.get_joint(jointname)
            if joint._child.is_human is not True:
                assert joint.joint_type == "fixed"
                self.add_aggregate("submechanisms", Submechanism(
                    name="fixed",
                    contextual_name="serial",
                    type="serial",
                    jointnames_active=[],
                    jointnames_independent=[],
                    jointnames_spanningtree=[],
                    jointnames=[jointname],
                    auto_gen=True
                ), silent=True)
        self.sort_submechanisms()
        # Now we merge all serial mechanisms to reduce the number of mechanisms
        # we do it twice to check whether the branches might be joined to another branch
        for _ in range(2):
            new_submechanisms = [self.submechanisms[0]]
            for sm in self.submechanisms[1:]:
                this_joint_indices = sorted([sorted_joints.index(str(j)) for j in sm.get_joints()])
                prev_joint_indices = sorted([sorted_joints.index(str(j)) for j in new_submechanisms[-1].get_joints()])
                if sm.auto_gen and new_submechanisms[-1].auto_gen and (
                        self.get_parent(sm.get_root(self), targettype="joint") in new_submechanisms[-1].get_joints() or
                        (self.get_parent(sm.get_root(self), targettype="joint") is None and
                         self.get_parent(new_submechanisms[-1].get_root(self), targettype="joint") is None)):
                    new_submechanisms[-1].jointnames = list(set(new_submechanisms[-1].jointnames + sm.jointnames))
                    new_submechanisms[-1].jointnames_active = list(
                        set(new_submechanisms[-1].jointnames_active + sm.jointnames_active))
                    new_submechanisms[-1].jointnames_independent = list(
                        set(new_submechanisms[-1].jointnames_independent + sm.jointnames_independent))
                    new_submechanisms[-1].jointnames_spanningtree = list(
                        set(new_submechanisms[-1].jointnames_spanningtree + sm.jointnames_spanningtree))
                elif sm.auto_gen and sm.is_only_fixed() and \
                        len(sm.get_children(self)) == 0 and sm.get_root(self) in new_submechanisms[-1].get_internal_links(self) and \
                        all([this_joint_indices[i+1] - this_joint_indices[i] == 1 for i in range(len(this_joint_indices)-1)]):
                    # if this is a continuous series of joint indices has no children and the parent is part of the other submechanism we can merge them
                    new_submechanisms[-1].jointnames = list(set(new_submechanisms[-1].jointnames + sm.jointnames))
                elif new_submechanisms[-1].auto_gen and new_submechanisms[-1].is_only_fixed() and\
                        len(new_submechanisms[-1].get_children(self)) == 0 and new_submechanisms[-1].get_root(self) in sm.get_internal_links(self) and \
                        all([prev_joint_indices[i+1] - prev_joint_indices[i] == 1 for i in range(len(prev_joint_indices)-1)]):
                    # same as previous but the other way around
                    new_submechanisms[-1].jointnames = list(set(new_submechanisms[-1].jointnames + sm.jointnames))
                    new_submechanisms[-1].jointnames_active = list(
                        set(new_submechanisms[-1].jointnames_active + sm.jointnames_active))
                    new_submechanisms[-1].jointnames_independent = list(
                        set(new_submechanisms[-1].jointnames_independent + sm.jointnames_independent))
                    new_submechanisms[-1].jointnames_spanningtree = list(
                        set(new_submechanisms[-1].jointnames_spanningtree + sm.jointnames_spanningtree))
                    new_submechanisms[-1].name = sm.name
                    new_submechanisms[-1].contextual_name = sm.contextual_name
                else:
                    new_submechanisms.append(sm)
            self.submechanisms = new_submechanisms
        self.sort_submechanisms()
        counter = 0
        for sm in self.submechanisms:
           if sm.auto_gen:
               sm.name = "serial_chain" if not sm.is_only_fixed() else "fixed_chain"
               sm.contextual_name = "serial_chain" + str(counter)
               counter += 1
        assert len(self._get_joints_included_twice_in_submechanisms()) == 0, self._get_joints_included_twice_in_submechanisms()
        assert len(self._get_joints_not_included_in_submechanisms()) == 0, self._get_joints_not_included_in_submechanisms()

    def get_loop_closure_joints(self):
        return [j for j in self.joints if j.cut_joint]
