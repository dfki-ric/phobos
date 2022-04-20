from copy import deepcopy
# import datetime
import os
import sys

import numpy as np
import yaml

from ..core import Robot
from .motors import Motor
from .poses import Pose
from .core import Material, Collision, Joint, Link
from .hyrodyn import Submechanism, Exoskeleton
from .sensors import Sensor, MultiSensor
from ..geometry import get_reflection_matrix
from ..utils import tree, transform
from ..utils.misc import edit_name_string


class Smurf(Robot):
    def __init__(self, name=None, xmlfile=None, submechanisms_file=None, smurffile=None, verify_meshes_on_import=True,
                 inputfile=None):
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
        self.annotations = {"link": [], "joint": []}
        self.named_annotations = {}

        # Smurf Informations
        self.motors = []
        self.sensors = []
        self.collisions = []
        self.poses = []
        self.submechanisms = []
        self.exoskeletons = []
        self._transmissions = []
        self._loop_constraints = []
        # The properties that are already included in the urdf get a smurf_ prefix
        self.smurf_links = []
        self.smurf_joints = []
        self.smurf_materials = []
        # Check the input file
        self.load_smurffile(self.smurffile)

        super().__init__(name=name, xmlfile=self.xmlfile, verify_meshes_on_import=verify_meshes_on_import)

        self._init_annotations()

    # helper methods
    def load_smurffile(self, smurffile):
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
                else:
                    self._parse_annotations(f)

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
                j_id = self.get_joint_id(motor['joint'])
                if j_id:
                    annotations = deepcopy(motor)
                    try:
                        annotations.pop('name')
                    except:
                        pass
                    try:
                        annotations.pop('joint')
                    except:
                        pass
                    self._attach_part(
                        'motors',
                        Motor(
                            robot=self,
                            name=motor['name'] if 'name' in motor else None,
                            joint=self.joints[j_id], **annotations
                        )
                    )

        if 'sensors' in self.annotations:
            for sensor in self.annotations['sensors']:
                # Search for the joint or link
                input_args = {}
                for k in sensor.keys():
                    if k == 'joint':
                        j_id = self.get_joint_id(sensor['joint'])
                        if j_id:
                            input_args.update(
                                {'joint': self.joints[j_id]}
                            )
                    if k == 'link':
                        l_id = self.get_link_id(sensor['link'])
                        if l_id:
                            input_args.update(
                                {'link': self.links[l_id]}
                            )
                    else:
                        input_args.update(
                            {k: sensor[k]}
                        )

                if input_args:
                    self._attach_part('sensors',
                                      Sensor(robot=self, **input_args)
                                      )

        if 'poses' in self.annotations:
            for pose in self.annotations['poses']:
                self._attach_part(
                    'poses',
                    Pose(robot=self, name=pose['name'], configuration=pose['joints'])
                )

        if 'joint' in self.annotations:
            for joint in self.annotations['joint']:
                self._attach_part(
                    'smurf_joints',
                    Joint(robot=self, joint=joint['name'], noDataPackage=joint["noDataPackage"],
                          reducedDataPackage=joint["reducedDataPackage"])
                )

        if 'link' in self.annotations:
            for link in self.annotations['link']:
                self._attach_part(
                    'smurf_links',
                    Link(robot=self, link=link['name'], noDataPackage=link["noDataPackage"],
                         reducedDataPackage=link["reducedDataPackage"])
                )

        if 'materials' in self.annotations:
            for material in self.annotations['materials']:
                self._attach_part(
                    'smurf_materials',
                    Material(**material)
                )

        if 'submechanisms' in self.annotations:
            for submech in self.annotations['submechanisms']:
                self._attach_part(
                    'submechanisms',
                    Submechanism(self, **submech)
                )

        if 'exoskeletons' in self.annotations:
            for exo in self.annotations['exoskeletons']:
                self._attach_part(
                    'exoskeletons',
                    Exoskeleton(self, **exo)
                )

    def get_joints_ordered_df(self):
        """Returns the joints in depth first order"""
        indep_joints = []
        for sm in self.submechanisms + self.exoskeletons:
            indep_joints += sm.jointnames_independent
        return tree.get_joints_depth_first(self, self.get_root(), independent_joints=list(set(indep_joints)))

    def copy_related_annotations(self, source, renamed_entities=None):
        """
        Copies applicable/relevant further annotations e.g. such that consider more than one joint/link/material from
        the source to this Robot instance.
        :param source: Another robot instance from which we copy the annotations
        :return: None
        """
        jointnames = [j.name for j in self.joints]
        linknames = [ln.name for ln in self.links]

        if renamed_entities is None:
            renamed_entities = {}
        for omotor in source.motors:
            jointname = omotor.joint if omotor.joint not in renamed_entities.keys() else renamed_entities[omotor.joint]
            if omotor.joint in self.joints:
                self.attach_motor(
                    deepcopy(omotor),
                    jointname
                )

        for osensor in source.sensors:
            _osensor = deepcopy(osensor)
            jointname = osensor.joint if _osensor.joint not in renamed_entities.keys() \
                else renamed_entities[_osensor.joint]
            linkname = osensor.link if _osensor.link not in renamed_entities.keys() \
                else renamed_entities[_osensor.link]
            add = True
            if isinstance(_osensor, MultiSensor):
                _ids = []
                for _id in _osensor.id:
                    _id = _id if _id not in renamed_entities.keys() else renamed_entities[_id]
                    if _id in linknames:
                        _ids += [_id]
                _osensor.id = _ids
                add &= len(_osensor.id) > 0
            else:
                if "link" in _osensor.returns:
                    add &= linkname in linknames
                if "joint" in _osensor.returns:
                    add &= jointname in jointnames
            if add:
                self.attach_sensor(_osensor)

        for onode in source.smurf_links:
            self.attach_smurf_version("link", deepcopy(onode))

        for onode in source.smurf_joints:
            self.attach_smurf_version("joint", deepcopy(onode))

        for onode in source.smurf_materials:
            self.attach_smurf_version("material", deepcopy(onode))

        for onode in source.poses:
            self.add_pose(deepcopy(onode))

        self.collisions += [deepcopy(c) for c in source.collisions]
        self.sort_submechanisms()
        for subm in source.submechanisms + source.exoskeletons:
            if all([self.get_joint(j) is not None for j in subm.jointnames_spanningtree]):
                if hasattr(subm, "loop_constraints"):
                    subm.loop_constraints = [deepcopy(lc) for lc in subm.loop_constraints]
                    self._loop_constraints += subm.loop_constraints
                if hasattr(subm, "multi_joint_dependencies"):
                    subm.multi_joint_dependencies = [deepcopy(mjd) for mjd in subm.multi_joint_dependencies]
                    self._transmissions += subm.multi_joint_dependencies
                _subm = deepcopy(subm)
                _subm.jointnames = [jn for jn in _subm.jointnames if self.get_joint(jn) is not None]
                if isinstance(_subm, Submechanism):
                    self.submechanisms += [deepcopy(_subm)]
                elif isinstance(_subm, Exoskeleton):
                    self.exoskeletons += [deepcopy(_subm)]

    # export methods
    def export_smurf(self, outputdir=None, export_visuals=True, export_collisions=True, create_pdf=False,
                     ros_pkg=False, export_with_ros_pathes=None, ros_pkg_name=None,
                     export_joint_limits=True, export_submodels=True):
        """ Export self and all annotations inside a given folder with structure
        """
        # Convert to absolute path
        outputdir = os.path.abspath(outputdir)
        if not os.path.exists(outputdir):
            os.mkdir(outputdir)

        submech_dir = os.path.join(outputdir, "submechanisms")
        if len(self.submechanisms) > 0 or len(self.exoskeletons) > 0:
            if not os.path.exists(submech_dir):
                os.makedirs(submech_dir)

        # First, export the urdf
        robotfile = os.path.join(outputdir, "urdf/{}.urdf".format(self.name))
        if not os.path.exists(os.path.dirname(robotfile)):
            os.mkdir(os.path.dirname(robotfile))
        super().full_export(output_dir=outputdir, export_visuals=export_visuals, export_collisions=export_collisions,
                            create_pdf=create_pdf, ros_pkg=ros_pkg, export_with_ros_pathes=export_with_ros_pathes,
                            ros_pkg_name=ros_pkg_name, export_joint_limits=export_joint_limits,
                            export_submodels=export_submodels)
        # Export the smurf files
        smurf_dir = os.path.join(outputdir, "smurf")
        if not os.path.exists(smurf_dir):
            os.mkdir(smurf_dir)
        # Export attr
        smurf_annotations = [
            'motors', 'sensors', 'smurf_materials', "smurf_joints", "smurf_links", 'collisions', 'poses',
            "submechanisms", "exoskeletons"
        ]
        export_files = [os.path.relpath(robotfile, outputdir + "/smurf")]
        submechanisms = {}
        self.fill_submechanisms()
        for sm in self.submechanisms + self.exoskeletons:
            if hasattr(sm, "file_path"):
                _submodel = self.define_submodel(name="#sub_mech#", start=sm.get_root(self),
                                                 stop=sm.get_leaves(self), robotname=sm.name)
                sm.file_path = "../submechanisms/" + os.path.basename(sm.file_path)
                self.export_submodel(name="#sub_mech#", output_dir=os.path.join(outputdir, "submechanisms"),
                                     filename=os.path.basename(sm.file_path), only_urdf=True)
                self.remove_submodel(name="#sub_mech#")
        for annotation in smurf_annotations:
            # Check if exists and not empty
            if hasattr(self, annotation) and getattr(self, annotation):
                annotation_dict = {annotation: []}
                if annotation == "smurf_joints":
                    annotation_dict = {"joint": []}
                if annotation == "smurf_links":
                    annotation_dict = {"link": []}
                if annotation == "smurf_materials":
                    annotation_dict = {"materials": []}
                # Collect all
                for item in getattr(self, annotation):
                    if not annotation.startswith("smurf_"):
                        annotation_dict[annotation].append(item.to_yaml())
                    elif annotation == "smurf_links" and type(item) == Link:
                        annotation_dict["link"].append(item.to_yaml())
                    elif annotation == "smurf_joints" and type(item) == Joint:
                        annotation_dict["joint"].append(item.to_yaml())
                    elif annotation == "smurf_materials":
                        annotation_dict["materials"].append(item.to_yaml())
                # Export to file
                annotation_name = annotation
                if annotation_name.startswith("smurf_"):
                    annotation_name = annotation_name[len("smurf_"):]
                if annotation == "submechanisms" or annotation == "exoskeletons":
                    submechanisms[annotation] = annotation_dict[annotation]
                else:
                    with open(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, annotation_name)), "w+") as stream:
                        yaml.safe_dump(annotation_dict, stream, default_style=False)
                        export_files.append(os.path.split(stream.name)[-1])
        if submechanisms != {}:
            self.submechanisms_file = os.path.join(smurf_dir, "{}_submechanisms.yml".format(self.name))
            with open(self.submechanisms_file, "w+") as stream:
                yaml.safe_dump(submechanisms, stream, default_style=False)
                export_files.append(os.path.split(stream.name)[-1])

        # further annotations
        for k, v in self.annotations.items():
            if k not in smurf_annotations + ["materials", "link", "joint"]:
                with open(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k)), "w+") as stream:
                    yaml.safe_dump({k: v}, stream, default_style=False)
                    export_files.append(os.path.split(stream.name)[-1])

        for k, v in self.named_annotations.items():
            if os.path.isfile(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k))):
                raise NameError("You can't overwrite the already existing SMURF-Annotation-File " +
                                os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k)) +
                                "\nPlease choose another name for you annotation")
            else:
                with open(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k)), "w") as stream:
                    yaml.safe_dump(v, stream, default_style=False)
                    export_files.append(os.path.split(stream.name)[-1])

        # Create the smurf file itsself
        annotation_dict = {
            'modelname': self.name,
            # 'date': datetime.datetime.now().strftime("%Y%m%d_%H:%M"),
            'files': sorted(export_files)
        }

        with open(os.path.join(smurf_dir, "{}.smurf".format(self.name)), "w+") as stream:
            yaml.safe_dump(annotation_dict, stream, default_style=False, sort_keys=True)
        print("SMURF written to", smurf_dir)

    def export_joint_limits(self, outputdir, file_name="joint_limits.yml", names=None):
        super(Smurf, self).export_joint_limits(outputdir=outputdir, file_name=file_name)
        jointnames_independent = []
        jointnames_active = []
        for sm in self.submechanisms + self.exoskeletons:
            if hasattr(sm, outputdir):
                jointnames_independent += sm.jointnames_independent
                jointnames_active += sm.jointnames_active
        super(Smurf, self).export_joint_limits(outputdir=os.path.join(outputdir, "../submechanisms"),
                                               file_name="joint_limits_independent.yml",
                                               names=jointnames_independent)
        super(Smurf, self).export_joint_limits(outputdir=os.path.join(outputdir, "../submechanisms"),
                                               file_name="joint_limits_active.yml",
                                               names=jointnames_active)

    def full_export(self, output_dir=None, export_visuals=True, export_collisions=True,
                    create_pdf=False, ros_pkg=False, export_with_ros_pathes=None, ros_pkg_name=None,
                    export_joint_limits=True, export_submodels=True):
        self.export_smurf(output_dir, export_visuals, export_collisions, create_pdf, ros_pkg, export_with_ros_pathes,
                          ros_pkg_name, export_joint_limits, export_submodels=export_submodels)

    # getters
    def get_motor_id(self, motorname):
        """Returns the ID (index in the motor list) of the motor(s).
        """

        if isinstance(motorname, list):
            return [self.get_motor_id(motor) for motor in motorname]

        return self.get_id('motors', motorname)

    def get_sensor_id(self, sensorname):
        """Returns the ID (index in the sensor list) of the sensor(s).
        """

        if isinstance(sensorname, list):
            return [self.get_sensor_id(sensor) for sensor in sensorname]

        return self.get_id('sensors', sensorname)

    # tools
    def remove_collisions(self):
        super(Smurf, self).remove_collisions()
        self.collisions = []

    def add_named_annotation(self, name, content):
        if name in self.named_annotations.keys():
            raise NameError("A named annotation with the name " + name +
                            " does already exist. Please merge or rename your annotations!")
        else:
            self.named_annotations[name] = content

    def get_smurf_collision_by_name(self, collisionname):
        """Return the collisions with the corresponding names.
        """
        if not isinstance(collisionname, list):
            cnames = [collisionname]
        else:
            cnames = collisionname

        returns = []

        for link in self.links:
            if link.collisions:
                for c in link.collisions:
                    if c.name in cnames:
                        returns += [c]

        return returns

    def attach_motor(self, motor, jointname=None):
        """Attach a new motor to the robot. Either the joint is already defined inside the motor
        or a jointname is given. Renames the motor if already given.
        """
        if not isinstance(motor, Motor):
            raise Exception("Please provide an instance of Motor to attach.")
        # Check if the motor already contains joint information
        motor.robot = self
        if not jointname and motor._joint:
            if motor._joint in self.joints:
                self._attach_part('motors', motor)
                return
        # Check if the joint is part of the robot if a jointname is given
        elif jointname and self.get_joint_id(jointname):
            motor._joint = self.joints[self.get_joint_id(jointname)]
            self._attach_part('motors', motor)
            return
        else:
            raise Exception("Please provide a joint which is part of the model")
        return

    def attach_smurf_version(self, node_type, node):
        """Attaches a smurf version of link, joint, material"""
        if not isinstance(node, Link) and not isinstance(node, Joint) and not isinstance(node, Material):
            raise Exception("Please provide an instance of Link/Joint/Material to attach.")
        self._attach_part('smurf_' + node_type.lower() + 's', node)
        return

    def attach_sensor(self, sensor):
        """Attach a new sensor to the robot. Either a joint or link is already defined inside the sensor
        or a jointname or linkname is given. Renames the sensor if the same name is already present.
        """
        if not isinstance(sensor, Sensor):
            raise Exception("Please provide an instance of Sensor to attach.")
        self._attach_part('sensors', sensor)
        return

    def add_pose(self, pose):
        """Add a new pose to the robot.
        """
        if not isinstance(pose, Pose):
            raise Exception("Please provide an instance of Pose to add.")
        self._attach_part('poses', pose)
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

        col = [c for c in link.collisions if c.name == collisionname][0]

        cbit = Collision(self, link, col, bitmask, **kwargs)

        self._attach_part('collisions', cbit)
        return

    def set_self_collision(self, val=False, coll_override=None, no_coll_override=None, **kwargs):
        """If True, tries to avoid self collision with bitmasks which do not intersect.
        """
        if no_coll_override is None:
            no_coll_override = {}
        if coll_override is None:
            coll_override = {}
        self.collisions = []

        if not val:
            return

        coll_names, coll_matrix = self.generate_collision_matrix(coll_override=coll_override,
                                                                 no_coll_override=no_coll_override)

        colls = {}
        link_colls = {}
        for link in self.links:
            colls.update({c.name: {"coll": c, "linkname": link.name} for c in link.collisions})
            link_colls.update({link.name: [c.name for c in link.collisions]})
        link_names = [colls[cn]["linkname"] for cn in coll_names]

        # find-bitmask-algo
        bits = [[] for _ in range(16)]  # list of list for each bit containing the collisions that lie on this bit
        X = coll_matrix
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                if i == j or coll_matrix[i, j] == 0:
                    continue
                # check if this collision is already entered
                coll_exists = False
                for b in bits:
                    if i in b and j in b:
                        coll_exists = True
                if coll_exists:
                    continue
                # check if we have i already entered somewhere and if it's possible to enter j there, too
                for b in bits:
                    if i in b and all([X[j, x] == 1 for x in b]):
                        b.append(j)
                        coll_exists = True
                    elif j in b and all([X[i, x] == 1 for x in b]):
                        b.append(i)
                        coll_exists = True
                    elif (all([X[i, x] == 1 for x in b]) and all([X[j, x] == 1 for x in b])) or len(b) == 0:
                        b.append(i)
                        b.append(j)
                        coll_exists = True
                    if coll_exists:
                        break
                if not coll_exists:
                    print("WARNING: Auto-Bitmask algorithm was unable to create the collision:", coll_names[i], "<->",
                          coll_names[j])
        for b in bits:
            for i in b:
                for j in b:
                    if X[i, j] == 0 and not i == j:
                        raise AssertionError("Unwanted Collision: " + coll_names[i] + " " + coll_names[j])
        # create the bitmasks
        bitmasks = [0 for _ in range(X.shape[0])]
        for i in range(X.shape[0]):
            for exp in range(len(bits)):
                if i in bits[exp]:
                    bitmasks[i] += 2 ** exp

        for i in range(len(coll_names)):
            self.set_bitmask(link_names[i], bitmask=bitmasks[i], collisionname=coll_names[i], **kwargs)

    def add_loop_constraint(self, loop_constraint):
        self._loop_constraints += [loop_constraint]

    def add_transmission(self, transmission):
        self._transmissions += [transmission]

    # Reimplementation of Robot methods
    def _rename(self, targettype, target, new_name):
        if targettype.startswith("link"):
            for obj in self.smurf_links:
                if obj.name == target:
                    obj.name = new_name
            for obj in self.sensors + self.collisions:
                if obj.link == target:
                    obj.link = new_name
            for obj in self._loop_constraints:
                if obj.prepredecessor_body == target:
                    obj.predecessor_body = new_name
                if obj.successor_body == target:
                    obj.successor_body = new_name
        elif targettype.startswith("joint"):
            for obj in self.smurf_joints:
                if obj.name == target:
                    obj.name = new_name
            for obj in self.motors + self.poses:
                if obj.joint == target:
                    obj.joint = new_name
            for obj in self.exoskeletons + self.submechanisms:
                for key in ["jointnames_independent", "jointnames_spanningtree", "jointnames_active", "jointnames"]:
                    if hasattr(obj, key):
                        setattr(obj, key, [j if j != target else new_name for j in getattr(obj, key)])
            for obj in self._loop_constraints:
                if obj.cut_joint == target:
                    obj.cut_joint = new_name
            for obj in self._transmissions:
                if obj.joint == target:
                    obj.joint = new_name
                for dep in obj.joint_dependencies:
                    if dep.joint == target:
                        dep.joint = new_name
        elif targettype.startswith("material"):
            for obj in self.smurf_materials:
                if obj.name == target:
                    obj.name = new_name
        elif targettype.startswith("collision"):
            for obj in self.collisions:
                if obj.name == target:
                    obj.name = new_name
        elif targettype.startswith("visual"):
            pass
        elif targettype.startswith("motor"):
            for obj in self.motors:
                if obj.name == target:
                    obj.name = new_name
        elif targettype.startswith("sensor"):
            for obj in self.sensors:
                if obj.name == target:
                    obj.name = new_name
        else:
            raise NotImplementedError("_rename() not implemented for targettype " + targettype)
        return {target: new_name}

    def add_link_by_properties(self, name, translation, rotation, parent, jointname=None, jointtype="fixed", axis=None, mass=0.0,
                               add_default_motor=True):
        """
        Adds a link with the given parameters. See core.Robot.addLink()
        """
        parent, link, joint = super(Smurf, self).add_link_by_properties(name, translation, rotation, parent, jointname=jointname,
                                                                        jointtype=jointtype, axis=axis, mass=mass)
        if joint.type in ["revolute", "prismatic"] and add_default_motor:
            self.attach_motor(Motor(
                robot=self,
                name=joint.name,
                joint=joint
            ))

    def attach(self, other, joint, do_not_rename=False, name_prefix="", name_suffix="_2"):
        """
        Attach another robot via the given joint at the link defined in the joint.
        Note this might edit other!
        :param other: the other Robot instance to attach (make sure to pass a deepcopy)
        :param joint: Joint definition used for attaching the robot
        :param do_not_rename: if true, all names of the attached robot will be edited else only duplicates
        :param name_prefix: a prefix to add to the names the have to be renamed (default: "")
        :param name_suffix: a prefix to add to the names the have to be renamed (default: "_2")
        :return: None
        """
        renamed_entities = {}

        if isinstance(other, Smurf):
            pmotors = set([e.name for e in self.motors])
            psensors = set([e.name for e in self.sensors])
            pcollisions = set([e.name for e in self.collisions])
            pposes = set([e.name for e in self.poses])
            psmfp = set([e.file_path for e in self.submechanisms + self.exoskeletons if hasattr(e, "file_path")])

            cmotors = set([e.name for e in other.motors])
            csensors = set([e.name for e in other.sensors])
            ccollisions = set([e.name for e in other.collisions])
            cposes = set([e.name for e in other.poses])
            csmfp = set([e.file_path for e in other.submechanisms + other.exoskeletons if hasattr(e, "file_path")])

            if pmotors & cmotors:
                print("Warning: Motor names are duplicates. A", name_prefix, "and a", name_suffix,
                      "will be pre-/appended!", pmotors & cmotors, file=sys.stderr)
                if not do_not_rename:
                    print("Correcting...")
                    renamed_entities.update(
                        other.rename(targettype="motor", target=list(pmotors & cmotors), prefix=name_prefix,
                                     suffix=name_suffix))
                    return self.attach(other, joint, do_not_rename=do_not_rename, name_prefix=name_prefix,
                                       name_suffix=name_suffix)
                else:
                    raise NameError("There are duplicates in motor names", repr(pmotors & cmotors))

            if psensors & csensors:
                print("Warning: Sensor names are duplicates. A", name_prefix, "and a", name_suffix,
                      "will be pre-/appended!", psensors & csensors, file=sys.stderr)
                if not do_not_rename:
                    print("Correcting...")
                    renamed_entities.update(
                        other.rename(targettype="sensor", target=list(psensors & csensors), prefix=name_prefix,
                                     suffix=name_suffix))
                    return self.attach(other, joint, do_not_rename=do_not_rename, name_prefix=name_prefix,
                                       name_suffix=name_suffix)
                else:
                    raise NameError("There are duplicates in sensor names", repr(psensors & csensors))

            if pcollisions & ccollisions:
                raise AssertionError("There are duplicates in collision names" + repr(
                    pcollisions & ccollisions) + "\nThis should have been handled via rename")

            if pposes & cposes:
                print("Warning: Pose names are duplicates. A", name_prefix, "and a", name_suffix,
                      "will be pre-/appended!", pposes & cposes, file=sys.stderr)
                if not do_not_rename:
                    print("Correcting...")
                    renamed_entities.update(
                        other.rename(targettype="pose", target=list(pposes & cposes), prefix=name_prefix,
                                     suffix=name_suffix))
                    return self.attach(other, joint, do_not_rename=do_not_rename, name_prefix=name_prefix,
                                       name_suffix=name_suffix)
                else:
                    raise NameError("There are duplicates in pose names", repr(pposes & cposes))

            if psmfp & csmfp:
                print(f"Warning: File pathes in submechanisms that are duplicates. A '{name_prefix}' and "
                      f"a '{name_suffix}' will be pre-/appended!", psmfp & csmfp, file=sys.stderr)
                print("Correcting...")
                for subm in other.submechanisms + other.exoskeletons:
                    if hasattr(subm, "file_path") and subm.file_path in list(psmfp & csmfp):
                        path, file = os.path.split(subm.file_path)
                        file, ext = file.split(".")
                        subm.file_path = path + edit_name_string(file, prefix=name_prefix, suffix=name_suffix) + ext
                return self.attach(other, joint, do_not_rename=do_not_rename, name_prefix=name_prefix,
                                   name_suffix=name_suffix)

        renamed_entities = super(Smurf, self).attach(other, joint, do_not_rename, name_prefix, name_suffix)

        # this will be done by fill_submechanisms
        # # Add the connection joint to the submechanism tree
        # self.submechanisms += [Submechanism(self, name=joint.name, type="serial", contextual_name="ConnectionJoint",
        #                                     jointnames_independent=[] if joint.type == "fixed" else [joint.name],
        #                                     jointnames_spanningtree=[] if joint.type == "fixed" else [joint.name],
        #                                     jointnames_active=[] if joint.type == "fixed" else [joint.name],
        #                                     jointnames=[joint.name])]

        self.copy_related_annotations(other, renamed_entities)

    def remove_joint(self, jointname, keep_collisions=True):
        """
        Remove the joint(s) from the mechanism and transforms all inertia, visuals and collisions
        to the corresponding parent of the joint.
        """
        if isinstance(jointname, list):
            for joint in jointname:
                self.remove_joint(joint, keep_collisions=keep_collisions)
            return

        joint = deepcopy(self.get_joint(jointname))
        super(Smurf, self).remove_joint(jointname, keep_collisions=keep_collisions)

        self.motors = [m for m in self.motors if m.joint != jointname]
        self.smurf_joints = [j for j in self.smurf_joints if j.name != joint.name]
        self.smurf_links = [ln for ln in self.smurf_links if ln.name != joint.child]

        for pose in self.poses:
            pose.remove_joint(jointname)

        new_sensors = []
        for sensor in self.sensors:
            if isinstance(sensor, MultiSensor):
                sensor.remove_target(joint.child)
                sensor.remove_target(joint.name)
                if not sensor.is_empty():
                    new_sensors += [sensor]
            else:
                if not sensor.joint == joint.name:
                    new_sensors += [sensor]
                elif sensor.link == joint.child:
                    sensor.transform(transform.origin_to_homogeneous(joint.origin))
                    new_sensors += [sensor]
        self.sensors = new_sensors

        for sub in self.submechanisms + self.exoskeletons:
            for key in ["jointnames", "jointnames_spanningtree", "jointnames_independent", "jointnames_active"]:
                if jointname in sub[key]:
                    setattr(sub, key, [j for j in getattr(sub, key) if j != jointname])
        self.submechanisms = [sm for sm in self.submechanisms if not sm.is_empty()]
        self.exoskeletons = [sm for sm in self.exoskeletons if not sm.is_empty()]

    def mirror_model(self, mirror_plane=None, maintain_order=None, exclude_meshes=None, name_replacements=None,
                     target_urdf=None, target_smurf=None, only_return=False):
        robot = super(Smurf, self).mirror_model(mirror_plane, maintain_order, exclude_meshes, name_replacements,
                                                target_urdf, only_return=True)
        # reflection matrix
        T_R = get_reflection_matrix(normal=np.array(mirror_plane))

        # transform sensor frames
        for sensor in self.sensors:
            if hasattr(sensor, "origin") and sensor.link is not None:
                T_link = self.get_transformation(sensor.link)
                T_root2link = robot.get_transformation(sensor.link)
                T = T_R.dot(T_link.dot(transform.origin_to_homogeneous(sensor.origin)))
                sensor.origin = transform.to_origin(transform.inv(T_root2link).dot(T))

        if target_smurf is not None:
            robot.load_smurffile(target_smurf)
        if only_return:
            return robot
        self.__dict__ = robot.__dict__

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
                setattr(sm, key, sorted(getattr(sm, key), key=lambda jn: sorted_joints.index(jn)))
        for transmission in self._transmissions:
            found = False
            for sm in self.submechanisms:
                if sm.contextual_name == transmission.name:
                    sm.multi_joint_dependencies += [transmission]
                    found = True
                    break
            if not found:
                print(transmission.to_yaml())
                raise AssertionError("Couldn't assign transmission")
        for loop_constraint in self._loop_constraints:
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
            if joint.type == "fixed":
                # If it's just a fixed joint we might be able to add this to an existing submechanisms
                for sm in self.submechanisms + self.exoskeletons:
                    for jn in sm.jointnames:
                        if sorted_joints.index(jn) == joint_idx + 1:
                            inserted = True
                            sm.jointnames.insert(sm.jointnames.index(jn), jointname)
                            break
                        elif sorted_joints.index(jn) == joint_idx - 1:
                            inserted = True
                            sm.jointnames.insert(sm.jointnames.index(jn)+1, jointname)
                            break
                    if inserted:
                        break
            if not inserted:
                # If we have not already inserted this joint (fixed) let's create a serial mechanism for it
                jn_spanningtree = jn_independent = jn_active = [] if joint.type == "fixed" else [jointname]
                jn = jn_spanningtree if joint.type != "fixed" else [jointname]
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
                sm.name = "serial_chain"+str(counter)
                sm.contextual_name = "serial_chain"+str(counter)
                counter += 1

    def add_floating_base(self):
        """
        Returns a copy of this Robot with a floatingbase mechanisms prpoended including the respective submechanism
        definition.
        :return: instance of robot
        """
        fb_robot = super(Smurf, self).add_floating_base()
        freeflyer = {
            "type": "3T+3R",
            "name": "free_flyer_joint",
            "contextual_name": "free_flyer_joint",
            "jointnames_independent": ["FreeFlyerX", "FreeFlyerY", "FreeFlyerZ", "FreeFlyerRX", "FreeFlyerRY",
                                       "FreeFlyerRZ"],
            "jointnames_spanningtree": ["FreeFlyerX", "FreeFlyerY", "FreeFlyerZ", "FreeFlyerRX", "FreeFlyerRY",
                                        "FreeFlyerRZ"],
            "jointnames_active": [],
            "jointnames": ["FreeFlyerX", "FreeFlyerY", "FreeFlyerZ", "FreeFlyerRX", "FreeFlyerRY", "FreeFlyerRZ"]
        }
        fb_robot.submechanisms += [Submechanism(fb_robot, **freeflyer)]
        return fb_robot
