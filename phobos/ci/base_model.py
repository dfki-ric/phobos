import os
import sys
import re
import filecmp
import pkg_resources
import yaml
import numpy as np
from copy import deepcopy, copy

from ..defs import load_json, dump_json, dump_yaml, KINEMATIC_TYPES

from ..core import Robot
from ..geometry import replace_collision, join_collisions, remove_collision, import_mesh, import_mars_mesh, \
    export_mesh, export_mars_mesh, export_bobj_mesh
from ..io.hyrodyn import ConstraintAxis
from ..utils import misc, git, xml, transform, tree
from ..io import representation, sensor_representations, poses
from ..utils.commandline_logging import get_logger
log = get_logger(__name__)

SUBMECHS_VIA_ASSEMBLIES = False


# [TODO pre_v2.0.0] simplify config files
class BaseModel(yaml.YAMLObject):
    def __init__subclass__(self, configfile, pipeline, processed_model_exists=True):
        self.processed_model_exists = processed_model_exists
        self.pipeline = pipeline

        if type(configfile) is str:
            if not os.path.isfile(configfile):
                raise Exception('{} not found!'.format(configfile))
            self.cfg = load_json(open(configfile, 'r'))
        else:
            self.cfg = configfile

        # These variables have to be defined in the config file
        self.modelname = ""
        self.robotname = ""
        self.export_config = []
        kwargs = {}
        if 'model' in self.cfg.keys():
            kwargs = self.cfg['model']
        elif 'xtype_model' in self.cfg.keys():
            kwargs = self.cfg['xtype_model']
        for (k, v) in kwargs.items():
            setattr(self, k, v)
        # check whether all necessary configurations are there
        assert hasattr(self, "modelname") and len(self.modelname) > 0
        assert hasattr(self, "robotname") and len(self.robotname) > 0
        assert hasattr(self, "export_config") and len(self.export_config) >= 1

        # get directories for this model
        self.exportdir = os.path.join(self.pipeline.temp_dir, self.modelname)
        self.targetdir = os.path.join(self.pipeline.root, self.modelname)
        self.tempdir = os.path.join(self.pipeline.temp_dir, "temp_" + self.modelname)

        # parse export_config
        self.export_meshpathes = []
        self.export_xmlfile = []
        for ec in self.export_config:
            if ec["type"] in KINEMATIC_TYPES:
                assert "mesh_format" in ec
                if "link_in_smurf" in ec and ec["link_in_smurf"] is True:
                    if self.export_xmlfile is not None:
                        raise AssertionError("Can only have one kinematics defining xml file in smurf, "
                                             "but defined multiple exports to be linked in smurf (link_in_smurf).")
                    self.export_xmlfile = self.robotname
                    if "filename_suffix" in ec:
                        self.export_xmlfile += ec["filename_suffix"]
                    ext = ec["type"].split("_")[-1].lower()
                    assert ext in ["sdf", "urdf"]
                    self.export_xmlfile += "." + ext
                    self.export_meshpathes = [self.pipeline.meshes[ec["mesh_format"]]] + self.export_meshpathes
                else:
                    self.export_meshpathes += [self.pipeline.meshes[ec["mesh_format"]]]                
                
    def __init__(self, configfile, pipeline, processed_model_exists=True):
        # These variables have to be defined in the config file
        self.input_models = {}
        self.assemble = {}
        # init
        self.__init__subclass__(configfile, pipeline, processed_model_exists)
        # check whether all necessary configurations are there
        assert hasattr(self, "input_models") and len(self.input_models) >= 1
        assert hasattr(self, "assemble") and len(self.assemble) > 0

        # list directly imported mesh pathes
        self._meshes = []
        for _, v in self.input_models.items():
            if "basefile" in v.keys():
                r = Robot(inputfile=v["basefile"], is_human=v["is_human"] if "is_human" in v else False)
                for link in r.links:
                    for g in link.visuals + link.collisions:
                        if hasattr(g.geometry, "filename"):
                            self._meshes += [xml.read_urdf_filename(g.geometry.filename[:-4], v["basefile"])]
            elif "repo" in v.keys():
                repo_path = os.path.join(self.tempdir, "repo", os.path.basename(self.input_models["repo"]["git"]))
                git.clone(
                    self,
                    self.input_models["repo"]["git"],
                    repo_path,
                    commit_id=self.input_models["repo"]["commit"],
                    recursive=True,
                    ignore_failure=True
                )
                self.basefile = os.path.join(repo_path, self.input_models["repo"]["model_in_repo"])
                r = Robot(inputfile=self.basefile)
                for link in r.links:
                    for g in link.visuals + link.collisions:
                        if isinstance(g.geometry, representation.Mesh):
                            self._meshes += [xml.read_urdf_filename(g.geometry.filename[:-4], self.basefile)]
        self.processed_meshes = []  # used to make mesh processing more efficient

        # where to find the already processed model
        self.basedir = os.path.join(self.tempdir, "combined_model")
        self.basefile = os.path.join(self.basedir, "smurf", "combined_model.smurf")

        if self.processed_model_exists:
            self._load_robot()

        log.debug(f"Finished reading config and joining models to base model {configfile}")

    def _load_robot(self):
        if not self.processed_model_exists:
            if os.path.exists(os.path.join(self.basedir, "smurf", "combined_model.smurf")):
                # may be there is already an assembly from a stopped job
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                   smurffile=os.path.join(self.basedir, "smurf", "combined_model.smurf"))
            else:
                # create a robot with the basic properties given
                self.robot = Robot(name=self.robotname if self.robotname else None)
        else:
            if os.path.exists(os.path.join(self.exportdir, "smurf", self.robotname + ".smurf")):
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                   smurffile=os.path.join(self.exportdir, "smurf", self.robotname + ".smurf"))
            else:
                raise Exception('Preprocessed file {} not found!'.format(self.basefile))

    def _join_to_basefile(self):
        # get all the models we need
        self.dep_models = {}
        for name, config in self.input_models.items():
            if "derived_base" in config.keys():
                self.dep_models.update({
                    name: BaseModel(
                        os.path.join(self.pipeline.configdir, config["derived_base"]),
                        self.pipeline, processed_model_exists=True)
                })
                # copy the mesh files to the temporary combined model directory
                for mp in self.dep_models[name].export_meshpathes:
                    misc.create_symlink(
                        self.pipeline, os.path.join(self.pipeline.temp_dir, mp), os.path.join(self.basedir, mp)
                    )
        for name, config in self.input_models.items():
            if "basefile" in config.keys():
                kwargs = {}
                kwargs["inputfile"] = config["basefile"]
                if "is_human" in config:
                    kwargs["is_human"] = config["is_human"]
                kwargs["inputfile"] = config["basefile"]
                self.dep_models.update({name: Robot(name=name, **kwargs)})
                # copy the mesh files to the temporary combined model directory
                for link in self.dep_models[name].links:
                    for v in link.visuals + link.collisions:
                        if isinstance(v.geometry, representation.Mesh):
                            misc.copy(
                                self.pipeline,
                                os.path.join(os.path.dirname(config["basefile"]), v.geometry.filename),
                                os.path.join(os.path.dirname(self.basefile), v.geometry.filename),
                                silent=True
                            )
            elif "repo" in config.keys():
                repo_path = os.path.join(self.tempdir, "repo", os.path.basename(config["repo"]["git"]))
                git.clone(
                    self.pipeline,
                    config["repo"]["git"],
                    repo_path,
                    config["repo"]["commit"],
                    recursive=True,
                    ignore_failure=True
                )
                self.dep_models.update({
                    name: Robot(name=name, inputfile=os.path.join(repo_path, config["repo"]["model_in_repo"]),
                                submechanisms_file=os.path.join(repo_path, config["repo"]["submechanisms_in_repo"])
                                                   if "submechanisms_in_repo" in config["repo"] else None,
                                is_human=config["is_human"] if "is_human" in config else False)
                })
                # copy the mesh files to the temporary combined model directory
                for link in self.dep_models[name].links:
                    for v in link.visuals + link.collisions:
                        if isinstance(v.geometry, representation.Mesh):
                            source = os.path.join(repo_path, os.path.dirname(self.dep_models[name].xmlfile),
                                                  v.geometry.filename)
                            target = os.path.join(os.path.dirname(self.basefile), v.geometry.filename)
                            if not os.path.exists(target) or (os.path.exists(target) and filecmp.cmp(source, target)):
                                misc.copy(self.pipeline, source, target)
                            else:
                                misc.copy(self.pipeline, source, target[:-4] + name + target[-4:])
                                v.geometry.filename = v.geometry.filename[:-4] + name + v.geometry.filename[-4:]

        # now we can join theses models
        # 1. get root model
        if isinstance(self.dep_models[self.assemble["model"]], Robot):
            combined_model = self.dep_models[self.assemble["model"]].duplicate()
        else:  # it must be an instance of BaseModel
            combined_model = self.dep_models[self.assemble["model"]].robot.duplicate()
        combined_model.name = self.robotname
        combined_model.autogenerate_submechanisms = False
        combined_model.smurffile = self.basefile
        combined_model.xmlfile = os.path.join(self.basedir, "urdf", "combined_model.urdf")

        if "name_editing" in self.assemble.keys():
            combined_model.edit_names(self.assemble["name_editing"])
            for c in self.assemble["children"]:
                c["joint"]["parent"] = misc.edit_name_string(
                    c["joint"]["parent"],
                    prefix=self.assemble["name_editing"]["prefix"] if "prefix" in self.assemble["name_editing"] else "",
                    suffix=self.assemble["name_editing"]["suffix"] if "suffix" in self.assemble["name_editing"] else "",
                    replacements=self.assemble["name_editing"]["replacements"] if "replacements" in self.assemble["name_editing"] else {})
                c["joint"]["parent"] = misc.edit_name_string(
                    c["joint"]["parent"],
                    prefix=self.assemble["name_editing"]["link_prefix"] if "link_prefix" in self.assemble["name_editing"] else "",
                    suffix=self.assemble["name_editing"]["link_suffix"] if "link_suffix" in self.assemble["name_editing"] else "",
                    replacements=self.assemble["name_editing"]["link_replacements"] if "link_replacements" in self.assemble["name_editing"] else {})

        if "remove_beyond" in self.assemble.keys():
            combined_model = combined_model.get_before(self.assemble["remove_beyond"])

        if "take_leaf" in self.assemble.keys():
            assert type(self.assemble["take_leaf"]) == str
            combined_model = combined_model.get_beyond(self.assemble["take_leaf"])

        if "mirror" in self.assemble.keys():
            combined_model.mirror_model(
                mirror_plane=self.assemble["mirror"]["plane"] if "plane" in self.assemble["mirror"].keys() else [0, 1, 0],
                flip_axis=self.assemble["mirror"]["flip_axis"] if "flip_axis" in self.assemble["mirror"].keys() else 1,
                exclude_meshes=self.assemble["mirror"]["exclude_meshes"] if "exclude_meshes" in self.assemble["mirror"].keys() else [],
                target_urdf=os.path.dirname(self.basefile)
            )

        # 2. go recursively through the children and attach them
        def recursive_attach(parent, children, parentname):
            for child in children:
                if isinstance(self.dep_models[parentname], Robot):
                    parent_model = self.dep_models[parentname]
                else:
                    parent_model = self.dep_models[parentname].robot

                if isinstance(self.dep_models[child["model"]], Robot):
                    att_model = self.dep_models[child["model"]].duplicate()
                else:
                    att_model = self.dep_models[child["model"]].robot.duplicate()

                att_model.relink_entities()

                if "child" not in child["joint"].keys() or child["joint"]["child"] is None:
                    child["joint"]["child"] = str(att_model.get_root())
                    if "take_leaf" in child:
                        child["joint"]["child"] = child["take_leaf"]

                if "r2r_transform" in child["joint"].keys():
                    T = np.array(child["joint"]["r2r_transform"])
                    src_T = parent_model.get_transformation(child["joint"]["parent"])
                    dst_T = att_model.get_transformation(child["joint"]["child"])
                    T = np.linalg.inv(src_T).dot(T).dot(dst_T)
                    origin = representation.Pose.from_matrix(T)
                    child["joint"]["xyz"] = origin.xyz
                    child["joint"]["rpy"] = origin.rpy

                if "name_editing" in child.keys():
                    att_model.edit_names(child["name_editing"])
                    child["joint"]["child"] = misc.edit_name_string(
                        child["joint"]["child"],
                        prefix=child["name_editing"]["prefix"] if "prefix" in child["name_editing"] else "",
                        suffix=child["name_editing"]["suffix"] if "suffix" in child["name_editing"] else "",
                        replacements=child["name_editing"]["replacements"] if "replacements" in child["name_editing"] else {})
                    child["joint"]["child"] = misc.edit_name_string(
                        child["joint"]["child"],
                        prefix=child["name_editing"]["link_prefix"] if "link_prefix" in child["name_editing"] else "",
                        suffix=child["name_editing"]["link_suffix"] if "link_suffix" in child["name_editing"] else "",
                        replacements=child["name_editing"]["link_replacements"] if "link_replacements" in child["name_editing"] else {})
                    if "children" in child:
                        for c in child["children"]:
                            c["joint"]["parent"] = misc.edit_name_string(
                                c["joint"]["parent"],
                                prefix=child["name_editing"]["prefix"] if "prefix" in child["name_editing"] else "",
                                suffix=child["name_editing"]["suffix"] if "suffix" in child["name_editing"] else "",
                                replacements=child["name_editing"]["replacements"] if "replacements" in child["name_editing"] else {})
                            c["joint"]["parent"] = misc.edit_name_string(
                                c["joint"]["parent"],
                                prefix=child["name_editing"]["link_prefix"] if "link_prefix" in child["name_editing"] else "",
                                suffix=child["name_editing"]["link_suffix"] if "link_suffix" in child["name_editing"] else "",
                                replacements=child["name_editing"]["link_replacements"] if "link_replacements" in child["name_editing"] else {})

                if "remove_beyond" in child.keys():
                    att_model = att_model.get_before(child["remove_beyond"])
                    att_model.relink_entities()

                if "take_leaf" in child.keys():
                    assert type(child["take_leaf"]) == str
                    att_model = att_model.get_beyond(child["take_leaf"])
                    assert len(att_model.keys()) == 1, "take_leaf: Please cut the tree in a way to get only one leaf"
                    att_model = list(att_model.values())[0]
                    att_model.relink_entities()

                if "mirror" in child.keys():
                    att_model.mirror_model(
                        mirror_plane=child["mirror"]["plane"] if "plane" in child["mirror"].keys() else [0, 1, 0],
                        flip_axis=child["mirror"]["flip_axis"] if "flip_axis" in child["mirror"].keys() else 1,
                        exclude_meshes=child["mirror"]["exclude_meshes"] if "exclude_meshes" in child["mirror"].keys() else [],
                        target_urdf=combined_model.xmlfile
                    )

                if "name" not in child["joint"].keys() or child["joint"]["name"] is None:
                    child["joint"]["name"] = child["joint"]["parent"] + "2" + child["joint"]["child"]
                if "type" not in child["joint"].keys() or child["joint"]["type"] is None:
                    child["joint"]["type"] = "fixed"

                if parent.get_link(child["joint"]["parent"]) is None:
                    log.error(f"Parent links: {sorted([lnk.name for lnk in parent.links])}")
                    raise AssertionError(
                        "Problem with assembling joint " + child["joint"]["parent"] + " -> " + child["joint"]["child"]
                        + ": the parent link doesn't exist!")
                elif att_model.get_link(child["joint"]["child"]) is None:
                    log.error(f"Child links: {sorted([lnk.name for lnk in att_model.links])}")
                    raise AssertionError(
                        "Problem with assembling joint " + child["joint"]["parent"] + " -> " + child["joint"]["child"]
                        + ": the child link doesn't exist!")
                assert att_model.get_joint(child["joint"]["name"]) is None and parent.get_joint(child["joint"]["name"]) is None, f'Can not join using joint name {child["joint"]["name"]} as this name already exists.'
                joint = representation.Joint(
                    name=child["joint"]["name"],
                    parent=parent.get_link(child["joint"]["parent"]).name,
                    child=att_model.get_link(child["joint"]["child"]).name,
                    joint_type=child["joint"]["type"] if "type" in child["joint"].keys() else "fixed",
                    origin=representation.Pose(
                        child["joint"]["xyz"] if "xyz" in child["joint"].keys() else [0, 0, 0],
                        child["joint"]["rpy"] if "rpy" in child["joint"].keys() else [0, 0, 0]
                    ),
                    limit=representation.JointLimit(
                        effort=child["joint"]["eff"] if "eff" in child["joint"].keys() else 0,
                        velocity=child["joint"]["vel"] if "vel" in child["joint"].keys() else 0,
                        lower=child["joint"]["min"] if "min" in child["joint"].keys() else 0,
                        upper=child["joint"]["max"] if "max" in child["joint"].keys() else 0)
                    if child["joint"]["type"] != "fixed" else None
                )

                parent.attach(att_model if isinstance(att_model, Robot) else att_model.robot, joint, do_not_rename=False)
                assert len(combined_model.links) == len(combined_model.joints) + 1
                if "remove_joint_later" in child["joint"] and child["joint"]["remove_joint_later"]:
                    if hasattr(self, "remove_joints") and type(self.remove_joints) == list:
                        self.remove_joints += [child["joint"]["name"]]
                    else:
                        self.remove_joints = [child["joint"]["name"]]
                parent.relink_entities()

                if "children" in child.keys():
                    recursive_attach(parent, child["children"], child["model"])

        if "children" in self.assemble.keys() and len(self.assemble["children"]) > 0:
            recursive_attach(combined_model, self.assemble["children"], parentname=self.assemble["model"])
            combined_model.relink_entities()

        # 3. save combined_model to the temp directory
        assert len(combined_model.links) == len(combined_model.joints) + 1
        combined_model.name = "combined_model"
        combined_model.full_export(self.basedir)

    def process(self):
        misc.recreate_dir(self.pipeline, self.tempdir)
        misc.recreate_dir(self.pipeline, self.basedir)

        self._join_to_basefile()
        self._load_robot()

        assert hasattr(self, 'robot') and hasattr(self, 'pipeline')

        log.info('Start processing robot')

        self.robot.correct_inertials()
        self.robot.clean_meshes()

        if hasattr(self, "frames"):
            _default = {} if "$default" not in self.frames else self.frames["$default"]
            _ignore_new_links = []
            for linkname, config in self.frames.items():
                self.frames[linkname] = misc.merge_default(config, _default)
                config = copy(self.frames[linkname])
                if self.robot.get_link(linkname) is None:
                    assert "transform_frame" not in config and "transform_link" not in config
                    assert "joint" in config
                    # [TODO!!! pre_v2.0.0] add provider for default values
                    # _joint_def = misc.merge_default(config.pop("joint"), default_joint)
                    _joint_def = config.pop("joint")
                    _joint = representation.Joint(
                        child=linkname,
                        parent=_joint_def.pop("parent"),
                        origin=representation.Pose(xyz=_joint_def.pop("xyz"), rpy=_joint_def.pop("rpy")),
                        **_joint_def
                    )
                    self.robot.add_link_by_properties(linkname, _joint, **config)
                    _ignore_new_links.append(linkname)
            for link in self.robot.links:
                linkname = link.name
                if linkname in _ignore_new_links:
                    continue
                config = self.frames["linkname"] if linkname in self.frames else _default
                for k, v in config.items():
                    if k in ["transform_frame", "transform_link"]:
                        transformation = transform.create_transformation(
                            xyz=v["xyz"] if "xyz" in v.keys() else [0, 0, 0],
                            rpy=v["rpy"] if "rpy" in v.keys() else [0, 0, 0]
                        )
                        # this is never really used and therefore not perfectly tested, hence commented
                        # if "transform" in v.keys() and v["transform"] == "TO":
                        #     if self.robot.getParent(k) is not None:
                        #         transformation = inv(
                        #             Homogeneous(self.robot.getJoint(self.robot.getParent(k)[0]).origin)
                        #         ).dot(transformation)
                        # else: BY
                        self.robot.transform_link_orientation(
                            linkname=k,
                            transformation=transformation,
                            only_frame=(k == "transform_frame"),
                            # transform_to="transform" in v.keys() and v["transform"] == "TO"
                        )
                    elif k == "reparent_to":
                        self.robot.move_link_in_tree(link_name=linkname, new_parent_name=v)
                    elif k == "estimate_missing_com" and v is True:
                        self.robot.set_estimated_link_com(linkname, dont_overwrite=True)
                    else:
                        link.add_annotation(k, v, overwrite=True)
            
        if hasattr(self, "joints"):
            transmissions = {}
            _default = {} if "$default" not in self.joints else self.joints["$default"]
            for jointname, config in self.joints.items():
                if self.robot.get_joint(jointname, verbose=True) is None and ("cut_joint" not in config or config["cut_joint"] is False):
                    raise NameError(f"There is no joint with name {jointname}")
                else:
                    # [TODO pre_v2.0.0] Review and Check whether this works as expected
                    # Check whether everxthing is given and calculate origin and axis (?)
                    _joint = representation.Joint(**config)
                    assert "constraint_axes" in config
                    _joint.constraint_axes = [ConstraintAxis(**ca) for ca in config["constraint_axes"]]
                    assert _joint.check_valid()
                    self.robot.add_aggregate(_joint)
            for joint in self.robot.joints:
                jointname = joint.name
                config = misc.merge_default(self.joints[jointname], _default)
                for k, v in config.items():
                    if k in ["min", "max", "eff", "vel"]:
                        if joint.limit is None:
                            joint.limit = representation.JointLimit()
                    if k == "move_joint_axis_to_intersection":
                        self.robot.move_joint_to_intersection(jointname, v)
                    elif k == "type":
                        joint.joint_type = v
                    elif k == "min":
                        joint.limit.lower = misc.read_number_from_config(v)
                    elif k == "max":
                        joint.limit.upper = misc.read_number_from_config(v)
                    elif k == "eff":
                        joint.limit.efforrt = misc.read_number_from_config(v)
                    elif k == "vel":
                        joint.limit.velocity = misc.read_number_from_config(v)
                    elif k == "movement_depends_on":
                        for jd in v:
                            joint.joint_dependencies.append(representation.JointMimic(joint=jd["joint_name"],
                                                                                      offset=jd["offset"],
                                                                                      multiplier=jd["multiplier"]))
                    elif k == "active":
                        if type(v) == dict:
                            if "name" not in v:
                                v["name"] = jointname+"_motor"
                            if "joint" not in v:
                                v["joint"] = jointname
                            else:
                                assert jointname == v["joint"]
                            _motor = representation.Motor(**v)
                            self.robot.add_motor(_motor)
                        elif v is True:
                            _motor = representation.Motor(name=jointname+"_motor", joint=jointname)
                            self.robot.add_motor(_motor)
                    else:  # axis, cut_joint
                        joint.add_annotation(k, v, overwrite=True)
                    # [TODO pre_v2.0.0] Re-add transmission support
                joint.link_with_robot(self.robot)
    
        # Check for joint definitions
        self.robot.check_joint_definitions(
            raise_error=self.typedef[
                "ensure_valid_joints"] if "ensure_valid_joints" in self.typedef.keys() else True,
            backup=self.redefine_articulation["default"] if (
                    hasattr(self, "redefine_articulation")
                    and "default" in self.redefine_articulation.keys()
                    and "backup" in self.redefine_articulation["default"].keys()
                    and self.redefine_articulation["default"]["backup"]
            ) else None,
        )


        if hasattr(self, 'replace_collisions'):
            self.edit_collisions = self.replace_collisions
        if hasattr(self, 'edit_collisions'):
            log.debug('  Replacing collisions')
            for link in self.robot.links:
                conf = deepcopy(self.edit_collisions["default"])
                exclude = self.edit_collisions["exclude"] if "exclude" in self.edit_collisions.keys() else []
                if link.name in exclude:
                    continue
                log.debug('       {}'.format(link.name))
                if link.name in self.edit_collisions.keys():
                    for k, v in self.edit_collisions[link.name].items():
                        conf[k] = v
                if "remove" in conf.keys():
                    if type(conf["remove"]) is list:
                        remove_collision(self.robot, link.name, collisionname=conf["remove"])
                    elif type(conf["remove"]) is str:
                        remove_collision(self.robot, link.name, collisionname=[c.name for c in link.collisions if
                                                                               re.fullmatch(r"" + conf["remove"],
                                                                                            c.name) is not None])
                if "join" in conf.keys() and conf["join"] is True:
                    if len(link.collisions) > 1:
                        # print("       Joining meshes of", link.name)
                        join_collisions(self.robot, link.name, name_id=self.modelname)
                if "shape" in conf.keys():
                    if conf["shape"] == "remove":
                        remove_collision(self.robot, link.name)
                    elif conf["shape"] != "mesh":
                        if not ("do_not_apply_primitives" in self.edit_collisions.keys() and
                                self.edit_collisions["do_not_apply_primitives"] is True):
                            replace_collision(
                                self.robot, link.name,
                                shape=conf["shape"],
                                oriented=conf["oriented"] if "oriented" in conf.keys() else True,
                                scale=conf["scale"] if "scale" in conf.keys() else 1.0,
                            )
                    # leads to problems in reusing identic meshes
                    # if conf["shape"] == "convex":
                    #     reduceMeshCollision(self.robot, link.name, reduction=0.3)

        if hasattr(self, "smurf"):
            log.debug('  Smurfing Collisions')
            if 'collisions' in self.smurf.keys():
                if "auto_bitmask" in self.smurf["collisions"].keys() and \
                        self.smurf["collisions"]["auto_bitmask"] is True:
                    log.debug("         Setting auto bitmask")
                    kwargs = self.smurf["collisions"]["default"] if "default" in self.smurf[
                        "collisions"].keys() else {}
                    self.robot.set_self_collision(
                        1,
                        coll_override=self.smurf["collisions"]["collision_between"]
                        if "collision_between" in self.smurf["collisions"].keys() else {},
                        no_coll_override=self.smurf["collisions"]["no_collision_between"]
                        if "no_collision_between" in self.smurf["collisions"].keys() else {},
                        **kwargs
                    )
                    for coll in self.robot.get_all_collisions():
                        if coll.name in self.smurf["collisions"].keys():
                            for k, v in self.smurf["collisions"][coll.name].items():
                                setattr(coll, k, v)
                else:
                    for coll_name in self.smurf["collisions"].keys():
                        conf = self.smurf["collisions"][coll_name]
                        coll = self.robot.get_collision_by_name(coll_name)
                        if coll is not None:
                            for key in ["name", "link", "geometry", "origin"]:
                                if key in conf:
                                    conf.pop(key)
                            coll.add_annotations(**conf)

        log.info('  Re-exporting meshes')
        misc.create_symlink(self.pipeline,
                            os.path.join(self.pipeline.temp_dir, self.typedef["meshespath"]),
                            os.path.join(self.exportdir, self.typedef["meshespath"])
                            )
        if self.typedef["output_mesh_format"].lower() != "bobj" and \
                "also_export_bobj" in self.typedef.keys() and self.typedef["also_export_bobj"]:
            misc.create_symlink(self.pipeline,
                                os.path.join(self.pipeline.temp_dir, self.pipeline.meshes["bobj"]),
                                os.path.join(self.exportdir, self.pipeline.meshes["bobj"])
                                )
        assert self.processed_meshes == []
        for link in self.robot.links:
            v_c = 0
            for v in link.visuals + link.collisions:
                if isinstance(v.geometry, representation.Mesh) and v.geometry.filename not in self.processed_meshes:
                    v_c += 1
                    if "mars_obj" in v.geometry.filename.lower() or (
                            "input_meshes_are_mars_obj" in self.typedef.keys() and
                            self.typedef["input_meshes_are_mars_obj"]):
                        mesh = v.geometry.load_mesh(urdf_path=os.path.dirname(self.basefile), mars_mesh=True)
                    elif v.geometry.filename.lower().endswith("bobj"):
                        raise NotImplementedError("Can't load bobj meshes!")
                    else:
                        if v.geometry.filename.lower().endswith("obj"):
                            log.warning("Loading obj mesh, where the orientation convention might be unknown")
                        mesh = v.geometry.load_mesh(urdf_path=os.path.dirname(self.basefile))
                    meshexport = os.path.join(self.exportdir, self.typedef["meshespath"],
                                              os.path.basename(v.geometry.filename)[:-3])
                    b_meshexport = os.path.join(self.exportdir, self.pipeline.meshes["bobj"],
                                                os.path.basename(v.geometry.filename)[:-3])
                    v.geometry.filename = v.geometry.filename.replace(" ", "_")
                    v.geometry.filename = meshexport
                    if self.typedef["output_mesh_format"].lower() == "mars_obj":
                        export_mars_mesh(mesh, meshexport + "obj", urdf_path=self.exporturdf)
                        v.geometry.filename += "obj"
                    elif self.typedef["output_mesh_format"].lower() == "bobj":
                        export_bobj_mesh(mesh, meshexport + "bobj", urdf_path=self.exporturdf)
                        v.geometry.filename += "bobj"
                    elif self.typedef["output_mesh_format"].lower() == "obj":
                        export_mesh(mesh, meshexport + "obj", urdf_path=self.exporturdf)
                        v.geometry.filename += "obj"
                    elif self.typedef["output_mesh_format"].lower() == "stl":
                        # and (not v.geometry.filename.lower.endswith("stl")) -> copy
                        export_mesh(mesh, meshexport + "stl", urdf_path=self.exporturdf)
                        v.geometry.filename += "stl"
                    elif self.typedef["output_mesh_format"].lower() == "dae":
                        color = None
                        for m in self.robot.materials:
                            if str(m) != str(v.material):
                                continue
                            else:
                                color = m.color.rgba
                        export_mesh(mesh, meshexport + "dae", urdf_path=self.exporturdf, dae_mesh_color=color)
                        v.geometry.filename += "dae"
                    else:
                        raise ValueError("Unknown mesh type" + self.typedef["output_mesh_format"].lower())
                    self.processed_meshes.append(os.path.realpath(v.geometry._filename))
                    if self.typedef["output_mesh_format"].lower() != "bobj" and \
                            "also_export_bobj" in self.typedef.keys() and self.typedef["also_export_bobj"]:
                        _filepath = export_bobj_mesh(mesh, b_meshexport + "bobj", urdf_path=self.exporturdf)
                        self.processed_meshes.append(os.path.realpath(_filepath))
            # print('       {} with {} meshes as {}'.format(link.name, v_c, self.typedef["output_mesh_format"]))

        if hasattr(self, "name_editing_after") and self.name_editing_after is not None:
            self.robot.edit_names(self.name_editing_after)

        if hasattr(self, "exoskeletons") or hasattr(self, "submechanisms"):
            if hasattr(self, "exoskeletons"):
                self.robot.load_submechanisms({"exoskeletons": deepcopy(self.exoskeletons)},
                                              replace_only_conflicting=True)
            if hasattr(self, "submechanisms"):
                self.robot.load_submechanisms({"submechanisms": deepcopy(self.submechanisms)},
                                              replace_only_conflicting=True)
        elif hasattr(self, "submechanisms_file"):
            self.robot.autogenerate_submechanisms = False
            self.robot.load_submechanisms(deepcopy(self.submechanisms_file))
        # if hasattr(self, "export_total_submechanisms"):
        #     # add all to one urdf
        #     spanningtree = []
        #     for sm in self.robot.submechanisms:
        #         spanningtree += sm.jointnames_spanningtree
        #     spanningtree = list(set(spanningtree))
        #     root = tree.find_common_root(input_spanningtree=spanningtree, input_model=self.robot)
        #     self.robot.define_submodel(name=self.export_total_submechanisms, start=root,
        #                                stop=tree.find_leaves(self.robot, spanningtree),
        #                                only_urdf=True)

        if hasattr(self, "additional_urdfs") and not hasattr(self, "export_submodels"):
            setattr(self, "export_submodels", self.additional_urdfs)
        if hasattr(self, "export_submodels"):
            for au in self.export_submodels:
                self.robot.define_submodel(name=au["name"], start=au["start"],
                                           stop=au["stop"] if "stop" in au else None,
                                           only_urdf=au["only_urdf"] if "only_urdf" in au.keys() else None)

        # motors
        if hasattr(self, "smurf"):
            assert self.robot.motors == []
            for joint in self.robot.joints:
                if hasattr(self, "smurf"):
                    conf = self.smurf["motors"]["default"] if "motors" in self.smurf.keys() and "default" in \
                                                              self.smurf["motors"].keys() else {}
                else:
                    conf = {}
                if "name" in conf.keys():  # we dont ẃant that someone overwrites the same name for all motors
                    conf.pop("name")
                if hasattr(self, "smurf") and "motors" in self.smurf.keys() and joint.name in self.smurf[
                    "motors"].keys():
                    for k, v in self.smurf["motors"][joint.name].items():
                        conf[k] = v
                if joint.joint_type == "fixed":
                    continue
                motor = representation.Motor(
                    joint=joint,
                    name=conf["name"] if "name" in conf.keys() else joint.name + "_motor",
                    p=conf["p"] if "p" in conf.keys() else 20.0,
                    i=conf["i"] if "i" in conf.keys() else 0.0,
                    d=conf["d"] if "d" in conf.keys() else 0.1,
                    maxEffort=joint.limit.effort if joint.limit is not None and joint.limit.effort > 0.0 else 400,
                    reducedDataPackage=conf["reducedDataPackage"] if "reducedDataPackage" in conf.keys() else False,
                    noDataPackage=conf["noDataPackage"] if "noDataPackage" in conf.keys() else False,
                )
                self.robot.add_motor(motor)

        if hasattr(self, "smurf"):
            log.debug('  Smurfing poses, sensors, links, materials, etc.')
            if 'poses' in self.smurf.keys():
                for (cn, config) in self.smurf["poses"].items():
                    pose = poses.JointPoseSet(robot=self.robot, name=cn, configuration=config)
                    self.robot.add_pose(pose)
                    log.debug('      Added pose {}'.format(cn))

            if 'sensors' in self.smurf.keys():
                multi_sensors = [x for x in dir(sensor_representations) if
                                 not x.startswith("__") and x not in sensor_representations.__IMPORTS__ and
                                 issubclass(getattr(sensor_representations, x), sensor_representations.MultiSensor)]
                single_sensors = [x for x in dir(sensor_representations) if
                                  not x.startswith(
                                      "__") and x not in sensor_representations.__IMPORTS__ and issubclass(
                                      getattr(sensor_representations, x),
                                      sensor_representations.Sensor) and x not in multi_sensors]
                moveable_joints = [j for j in self.robot.joints if j.joint_type != 'fixed']

                for s in self.smurf["sensors"]:
                    sensor_ = None
                    if s["type"] in single_sensors:
                        kwargs = {k: v for k, v in s.items() if k != "type"}
                        sensor_ = getattr(sensor_representations, s["type"])(**kwargs)

                    if s["type"] in multi_sensors:
                        if "targets" not in s:
                            pass
                        elif type(s['targets']) is str and s['targets'].upper() == "ALL":
                            s['targets'] = moveable_joints
                        elif type(s['targets']) is list:
                            pass
                        else:
                            raise ValueError('Targets can only be a list or "All"!')
                        kwargs = {k: v for k, v in s.items() if k != "type"}
                        sensor_ = getattr(sensor_representations, s["type"])(**kwargs)

                    if sensor_ is not None:
                        self.robot.add_sensor(sensor_)
                        log.debug('      Attached {} {}'.format(s["type"], s['name']))

            if 'links' in self.smurf.keys():
                for link in self.robot.links:
                    name = link.name if link.name in self.smurf["links"].keys() else "default"
                    if name in self.smurf["links"].keys():
                        link_instance = self.robot.get_link(link.name)
                        link_instance.noDataPackage = self.smurf["links"][name][
                            "noDataPackage"] if "noDataPackage" in self.smurf["links"][name].keys() else False
                        link_instance.reducedDataPackage = self.smurf["links"][name][
                            "reducedDataPackage"] if "reducedDataPackage" in self.smurf["links"][
                            name].keys() else False
                        log.debug('      Defined Link {}'.format(link.name))

            if "materials" in self.smurf.keys():
                for m in self.smurf["materials"]:
                    material_instance = self.robot.get_material(m["name"])
                    material_instance.add_annotations(**m)
                    log.debug('      Defined Material {}'.format(m["name"]))

            if "further_annotations" in self.smurf.keys():
                for k, v in self.smurf["further_annotations"]:
                    if k in self.robot.annotations.keys():
                        self.robot.annotations[k] += v
                    else:
                        self.robot.annotations[k] = v

            if "named_annotations" in self.smurf.keys():
                for k, v in self.smurf["named_annotations"].items():
                    self.robot.add_named_annotation(k, v)

        log.info('Finished processing')

        return True