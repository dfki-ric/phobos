import os
import re
from copy import deepcopy, copy

import numpy as np
import yaml
import json

from ..commandline_logging import get_logger
from ..core import Robot
from ..defs import load_json, dump_json, dump_yaml, KINEMATIC_TYPES
from ..geometry import replace_collision, join_collisions, remove_collision, remove_visual
from ..io import representation, sensor_representations, poses
from ..io.hyrodyn import ConstraintAxis
from ..utils import misc, git, xml, transform, resources

log = get_logger(__name__)

SUBMECHS_VIA_ASSEMBLIES = False


class BaseModel(yaml.YAMLObject):
    def __init__subclass__(self, configfile, pipeline, processed_model_exists=True, configkey=None):
        self.processed_model_exists = processed_model_exists
        self.pipeline = pipeline

        self.configkey = configkey
        self.configfile = None
        if type(configfile) is str:
            if not os.path.isfile(configfile):
                raise Exception('{} not found!'.format(configfile))
            self.cfg = load_json(open(configfile, 'r'))
            self.configfile = configfile
        else:
            self.cfg = configfile

        # These variables have to be defined in the config file
        self.modelname = ""
        self.robotname = ""
        self.test = {}
        kwargs = {}
        if 'model' in self.cfg.keys():
            kwargs = self.cfg['model']
            if "defaults" in kwargs:
                definition_cascade = [kwargs]
                prev_defaults_file = None
                base_dir = os.path.dirname(self.configfile)
                while "defaults" in definition_cascade[-1]:
                    defaults_file = misc.abspath(definition_cascade[-1]["defaults"], base_dir)
                    assert defaults_file != prev_defaults_file, "File references itself as default"
                    prev_defaults_file = defaults_file
                    assert os.path.isfile(defaults_file), defaults_file
                    base_dir = os.path.dirname(defaults_file)
                    with open(defaults_file, "r") as f:
                        definition_cascade.append(load_json(f.read())["default_model"])
                # go through the cascade
                parent = definition_cascade[-1]
                child = definition_cascade[-2]
                for i in range(len(definition_cascade)-1):
                    child = misc.patch_dict(parent, child)
                    parent = child
                    if i <= len(definition_cascade)-3:
                        child = definition_cascade[-i-3]
                # make it possible to use the things defined in kwargs as variables
                defaults_string = dump_json(parent) # to make sure we don't mess with yaml whitespaces
                defaults_string = misc.regex_replace(defaults_string, replacements={"@"+k+"@": json.dumps(v)[1:-1] for k, v in parent.items()})
                kwargs = load_json(defaults_string)
                kwargs.pop("defaults")
        for (k, v) in kwargs.items():
            setattr(self, k, v)
        # check whether all necessary configurations are there
        assert hasattr(self, "input_models") and len(self.modelname) > 0 and type(self.input_models) == dict
        assert hasattr(self, "modelname") and len(self.modelname) > 0
        assert hasattr(self, "robotname") and len(self.robotname) > 0
        assert hasattr(self, "test") and len(self.test) >= 1
        self.test = misc.merge_default(
            self.test,
            self.pipeline.default_test if hasattr(pipeline, "default_test") else resources.get_default_ci_test_definition()
        )
        if "test_data_file" in self.test:
            self.test_data_file = self.test["test_data_file"]
        self.deployment = misc.merge_default(
            self.deployment if hasattr(self, "deployment") else {},
            self.pipeline.default_deployment if hasattr(pipeline, "default_deployment") else resources.get_default_ci_deploy_definition()
        )

        self.extended_test_protocol = ""

        # get directories for this model
        self.exportdir = os.path.join(self.pipeline.temp_dir, self.modelname)
        self.targetdir = os.path.join(self.pipeline.root, self.modelname)
        self.tempdir = os.path.join(self.pipeline.temp_dir, "temp_" + self.modelname)
        self.extended_test_protocol_path = os.path.join(self.tempdir, "extended_test_protocol.log")

        if not os.path.isdir(self.targetdir):
            log.warning(self.targetdir + "does not exist. Trying to check it our from: " + self.pipeline.remote_base + self.modelname)
            git.clone(self.pipeline.remote_base + self.modelname, self.targetdir, branch=git.GIT_PRODUCTION_BRANCH, pipeline=self.pipeline, all_branches=True, shallow=0)

        # parse export_config
        self.export_meshes = {}
        self.export_xmlfile = None
        if not hasattr(self, "export_config"):
            self.export_config = self.pipeline.default_export_config
        if hasattr(self, "append_export_config"):
            self.export_config += self.append_export_config
        for ec in self.export_config:
            if ec["type"] in KINEMATIC_TYPES:
                assert "mesh_format" in ec
                if ec["mesh_format"] == "input_type":
                    log.warning("Due to the mesh handling on the git repos for CI-pipelines using input_type as mesh format is not yet fully tested and supported.")
                if "additional_meshes" in ec and type(ec["additional_meshes"]) == str:
                    ec["additional_meshes"] = [ec["additional_meshes"]]
                elif not ("additional_meshes" in ec and type(ec["additional_meshes"]) == list):
                    ec["additional_meshes"] = []
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
                self.export_meshes = {
                    mt.lower(): self.pipeline.meshes[mt] for mt in [ec["mesh_format"]] + ec["additional_meshes"]
                }
            elif ec["type"] == "kccd":
                self.export_meshes["iv"] = self.pipeline.meshes["iv"]

    def __init__(self, configfile, pipeline, processed_model_exists=True, configkey=None):
        # These variables have to be defined in the config file
        self.input_models = {}
        self.assemble = {}
        # init
        self.__init__subclass__(configfile, pipeline, processed_model_exists, configkey=configkey)
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
                        if isinstance(g.geometry, representation.Mesh):
                            self._meshes += [xml.read_relative_filename(g.geometry.filepath[:-4], v["basefile"])]
            elif "repo" in v.keys():
                repo_path = os.path.join(self.tempdir, "repo", os.path.basename(self.input_models["repo"]["git"]))
                git.clone(
                    pipeline = self,
                    repo=self.input_models["repo"]["git"],
                    target=repo_path,
                    commit_id=self.input_models["repo"]["commit"],
                    recursive=True,
                    ignore_failure=True,
                    recreate=True
                )
                self.basefile = os.path.join(repo_path, self.input_models["repo"]["model_in_repo"])
                r = Robot(inputfile=self.basefile)
                for link in r.links:
                    for g in link.visuals + link.collisions:
                        if isinstance(g.geometry, representation.Mesh):
                            self._meshes += [xml.read_relative_filename(g.geometry.filename[:-4], self.basefile)]
        self.processed_meshes = set()  # used to make mesh processing more efficient

        # where to find the already processed model
        self.basedir = os.path.join(self.tempdir, "combined_model")
        self.basefile = os.path.join(self.basedir, "smurf", "combined_model.smurf")

        if self.processed_model_exists:
            self._load_robot()

        log.debug(f"Finished reading config and joining models to base model {configfile}")

    @staticmethod
    def get_exported_model_path(pipeline, configfile):
        cfg = load_json(open(configfile, 'r'))
        assert "model" in cfg
        cfg = cfg["model"]
        return os.path.join(pipeline.temp_dir, cfg["modelname"], "smurf", cfg["robotname"] + ".smurf")

    def _load_robot(self):
        if not self.processed_model_exists:
            if os.path.exists(os.path.join(self.basedir, "smurf", "combined_model.smurf")):
                # may be there is already an assembly from a stopped job
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                   inputfile=os.path.join(self.basedir, "smurf", "combined_model.smurf"))
            else:
                # create a robot with the basic properties given
                self.robot = Robot(name=self.robotname if self.robotname else None)
        else:
            load_file = os.path.join(self.exportdir, "smurf", getattr(self, "filename", self.robotname) + ".smurf")
            if os.path.exists(load_file):
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                   inputfile=load_file)
            else:
                raise Exception('Preprocessed file {} not found!'.format(load_file))

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
                if self.pipeline.central_meshes:
                    # copy the mesh files to the temporary combined model diretory
                    for mp in self.dep_models[name].export_meshes.values():
                        misc.create_symlink(
                            self.pipeline, os.path.join(self.pipeline.temp_dir, str(mp)), os.path.join(self.basedir, str(mp))
                        )
                else:
                    for mp in self.dep_models[name].export_meshes.values():
                        misc.create_dir(
                            self.pipeline, os.path.join(self.basedir, str(mp))
                        )
        for name, config in self.input_models.items():
            if "basefile" in config.keys():
                kwargs = {}
                kwargs["inputfile"] = config["basefile"]
                if "is_human" in config:
                    kwargs["is_human"] = config["is_human"]
                kwargs["inputfile"] = config["basefile"]
                self.dep_models.update({name: Robot(name=name, **kwargs)})
            elif "repo" in config.keys():
                repo_path = os.path.join(self.tempdir, "repo", os.path.basename(config["repo"]["git"]))
                git.clone(
                    pipeline=self.pipeline,
                    repo=config["repo"]["git"],
                    target=repo_path,
                    branch=config["repo"]["commit"],
                    recursive=True,
                    ignore_failure=True,
                    recreate=True
                )
                self.dep_models.update({
                    name: Robot(name=name, inputfile=os.path.join(repo_path, config["repo"]["model_in_repo"]),
                                submechanisms_file=os.path.join(repo_path, config["repo"]["submechanisms_in_repo"])
                                                   if "submechanisms_in_repo" in config["repo"] else None,
                                is_human=config["is_human"] if "is_human" in config else False)
                })
            self.dep_models[name].clean_meshes(check_enough_vertices=False)
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
            assert self.assemble["take_leaf"] in combined_model, f"{combined_model}"
            combined_model = combined_model[self.assemble["take_leaf"]]

        if "mirror" in self.assemble.keys():
            combined_model.clean_meshes()
            combined_model.mirror_model(
                mirror_plane=self.assemble["mirror"]["plane"] if "plane" in self.assemble["mirror"].keys() else [0, 1, 0],
                flip_axis=self.assemble["mirror"]["flip_axis"] if "flip_axis" in self.assemble["mirror"].keys() else 1,
                exclude_meshes=self.assemble["mirror"]["exclude_meshes"] if "exclude_meshes" in self.assemble["mirror"].keys() else [],
                target_urdf=os.path.dirname(self.basefile),
                name_replacements=self.assemble["mirror"].get("name_replacements", {})
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

                att_model.link_entities()

                if "child" not in child["joint"].keys() or child["joint"]["child"] is None:
                    child["joint"]["child"] = str(att_model.get_root())
                    if "take_leaf" in child:
                        child["joint"]["child"] = child["take_leaf"]

                if "r2r_transform" in child["joint"].keys():
                    T = np.array(child["joint"]["r2r_transform"])
                    src_T = parent_model.get_transformation(child["joint"]["parent"])
                    dst_T = att_model.get_transformation(child["joint"]["child"])
                    T = np.linalg.inv(src_T).dot(T).dot(dst_T)
                    _temp = representation.Pose.from_matrix(T, relative_to=child["joint"]["parent"])
                    child["joint"]["xyz"] = _temp.xyz
                    child["joint"]["rpy"] = _temp.rpy

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

                att_model.unlink_entities()

                if "remove_beyond" in child.keys():
                    att_model = att_model.get_before(child["remove_beyond"])

                if "take_leaf" in child.keys():
                    assert type(child["take_leaf"]) == str
                    att_model = att_model.get_beyond(child["take_leaf"])
                    assert len(att_model.keys()) == 1, "take_leaf: Please cut the tree in a way to get only one leaf"
                    att_model = list(att_model.values())[0]

                if "mirror" in child.keys():
                    att_model.mirror_model(
                        mirror_plane=child["mirror"]["plane"] if "plane" in child["mirror"].keys() else [0, 1, 0],
                        flip_axis=child["mirror"]["flip_axis"] if "flip_axis" in child["mirror"].keys() else 1,
                        exclude_meshes=child["mirror"]["exclude_meshes"] if "exclude_meshes" in child["mirror"].keys() else [],
                        target_urdf=combined_model.xmlfile,
                        final_linking_optional=True,
                        name_replacements=child["mirror"].get("name_replacements", {})
                    )

                if "name" not in child["joint"].keys() or child["joint"]["name"] is None:
                    child["joint"]["name"] = child["joint"]["parent"] + "2" + child["joint"]["child"]
                if "type" not in child["joint"].keys() or child["joint"]["type"] is None:
                    child["joint"]["type"] = "fixed"

                if parent.get_link(child["joint"]["parent"]) is None:
                    log.error(f"Parent links: {sorted([lnk.name for lnk in parent.links])}")
                    raise AssertionError(
                        "Problem with assembling joint " + child["joint"]["parent"] + " -> " + child["joint"]["child"]
                        + ": the parent link doesn't exist! (Further info above)")
                elif att_model.get_link(child["joint"]["child"]) is None:
                    log.error(f"Child links: {sorted([lnk.name for lnk in att_model.links])}")
                    raise AssertionError(
                        "Problem with assembling joint " + child["joint"]["parent"] + " -> " + child["joint"]["child"]
                        + ": the child link doesn't exist! (Further info above)")
                assert att_model.get_joint(child["joint"]["name"]) is None and parent.get_joint(child["joint"]["name"]) is None,\
                    f'Can not join using joint name {child["joint"]["name"]} as this name already exists.'
                joint = representation.Joint(
                    name=child["joint"]["name"],
                    parent=parent.get_link(child["joint"]["parent"]).name,
                    child=att_model.get_link(child["joint"]["child"]).name,
                    joint_type=child["joint"]["type"] if "type" in child["joint"].keys() else "fixed",
                    origin=representation.Pose(
                        child["joint"]["xyz"] if "xyz" in child["joint"].keys() else [0, 0, 0],
                        child["joint"]["rpy"] if "rpy" in child["joint"].keys() else [0, 0, 0],
                        relative_to=parent.get_parent(child["joint"]["parent"])
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
                parent.unlink_entities()

                if "children" in child.keys():
                    recursive_attach(parent, child["children"], child["model"])

        if "children" in self.assemble.keys() and len(self.assemble["children"]) > 0:
            recursive_attach(combined_model, self.assemble["children"], parentname=self.assemble["model"])
            combined_model.link_entities()

        # 3. save combined_model to the temp directory
        assert len(combined_model.links) == len(combined_model.joints) + 1
        combined_model.name = "combined_model"
        combined_model.export(outputdir=self.basedir, export_config=resources.get_default_export_config("minimal"),
                              check_submechs=False)

    def recreate_sym_links(self):
        if self.pipeline.central_meshes:
            for mt, mp in self.export_meshes.items():
                log.info('  Re-creating central meshes symlinks')
                misc.create_symlink(
                    self.pipeline, os.path.join(self.pipeline.temp_dir, str(mp)), os.path.join(self.exportdir, str(mp))
                )
        else:
            for mt, mp in self.export_meshes.items():
                log.info('  Ensuring mesh directories')
                misc.create_dir(
                    self.pipeline, os.path.join(self.exportdir, str(mp))
                )

    def process(self):
        misc.recreate_dir(self.pipeline, self.tempdir)
        misc.recreate_dir(self.pipeline, self.exportdir)

        # Make sure the mesh symlinks are set correctly
        if self.pipeline.central_meshes:
            for mt, mp in self.export_meshes.items():
                log.info('  Creating mesh symlinks')
                misc.create_symlink(
                    self.pipeline, os.path.join(self.pipeline.temp_dir, str(mp)), os.path.join(self.exportdir, str(mp))
                )
        else:
            for mt, mp in self.export_meshes.items():
                log.info('  Creating mesh directories')
                misc.create_dir(
                    self.pipeline, os.path.join(self.exportdir, str(mp))
                )

        self._join_to_basefile()
        self._load_robot()

        assert hasattr(self, 'robot') and hasattr(self, 'pipeline')

        log.info('Start processing robot')

        self.robot.correct_inertials()

        if hasattr(self, "materials"):
            for m_name, m_def in self.materials.items():
                material_instance = self.robot.get_material(m_name)
                if material_instance is not None:
                    material_instance.add_annotations(**m_def)
                    log.debug('Added annotation to Material {}'.format(m_name))
                else:
                    material_instance = representation.Material(name=m_name, **m_def)
                    self.robot.add_aggregate("material", material_instance)
                    log.debug('Defined Material {}'.format(m_name))

        if hasattr(self, "links") and not hasattr(self, "frames"):
            self.frames = self.links
        if hasattr(self, "frames"):
            _default = {} if "$default" not in self.frames else self.frames["$default"]
            _ignore_new_links = []
            make_root = []
            for linkname, config in self.frames.items():
                if linkname.startswith("$"):
                    continue
                if config.get("make_root", False):
                    make_root.append(linkname)
                self.frames[linkname] = misc.merge_default(config, _default)
                config = copy(self.frames[linkname])
                if self.robot.get_link(linkname) is None:
                    assert "transform_frame" not in config and "transform_link" not in config, "You can't transform a frame your are about to create, just give it's transformation via the joint."
                    if "joint" not in config:
                        raise KeyError(f"Frame {linkname} can't be defined without a joint definiton. Links that are already in the robot:\n" + str([str(l) for l in self.robot.links]))
                    _joint_def = config.pop("joint")
                    if "type" not in _joint_def:
                        log.debug("No joint type for {linkname} specified assuming fixed.")
                        _joint_def["type"] = "fixed"
                    _joint_def = misc.merge_default(_joint_def, resources.get_default_joint(_joint_def["type"]))
                    parent_link = _joint_def.pop("parent")
                    parent_joint = self.robot.get_parent(parent_link)
                    _joint = representation.Joint(
                        child=linkname,
                        parent=parent_link,
                        origin=representation.Pose(xyz=_joint_def.pop("xyz"), rpy=_joint_def.pop("rpy"),
                                                   relative_to=parent_joint),
                        **_joint_def
                    )
                    self.robot.add_link_by_properties(linkname, _joint, **config)
                    _ignore_new_links.append(linkname)
            if len(make_root) == 1:
                self.robot.exchange_root(make_root[0])
            elif len(make_root) > 1:
                raise AssertionError("You can make only one link to the new root not "+str(make_root))
            assert self.robot.get_root()
            for link in self.robot.links:
                linkname = link.name
                if linkname in _ignore_new_links:
                    continue
                config = self.frames[linkname] if linkname in self.frames else _default
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
                            linkname=linkname,
                            transformation=transformation,
                            only_frame=(k == "transform_frame"),
                            # transform_to="transform" in v.keys() and v["transform"] == "TO"
                        )
                    elif k == "reparent_to" and v != link.name:
                        self.robot.move_link_in_tree(link_name=linkname, new_parent_name=v)
                    elif k == "estimate_missing_com" and v is True:
                        self.robot.set_estimated_link_com(linkname, dont_overwrite=True)
                    elif k == "material" or k == "materials":
                        link.materials = v
                    else:
                        link.add_annotation(k, v, overwrite=True)
            for link in self.robot.links:
                linkname = link.name
                if linkname in _ignore_new_links:
                    continue
                config = self.frames[linkname] if linkname in self.frames else _default
                for k, v in config.items():
                    if k == "name_editing":
                        link.name = misc.edit_name_string(link.name, **v, correspondance=link)
            if "$name_editing" in self.frames:
                self.robot.rename("links", self.robot.links, **self.frames.get("$name_editing", {}))

        if hasattr(self, "joints"):
            if '$replace_joint_types' in self.joints:
                for joint in self.robot.joints:
                    for old, new in self.joints["$replace_joint_types"].items():
                        if joint.joint_type == old:
                            joint.joint_type = new
            transmissions = {}
            _default = {} if "$default" not in self.joints else self.joints["$default"]
            faulty_joint_defs = []
            for jointname, config in self.joints.items():
                if jointname.startswith("$"):
                    continue
                if self.robot.get_joint(jointname) is None and ("cut_joint" not in config or config["cut_joint"] is False):
                    faulty_joint_defs += [(jointname, [str(j) for j in self.robot.joints if jointname in str(j) or str(j) in jointname])]
                elif self.robot.get_joint(jointname) is None and ("cut_joint" in config and config["cut_joint"] is True):  # cut_joint
                    # [TODO v2.2.0] Review and Check whether this works as expected
                    # Check whether everything is given and calculate origin and axis (?)
                    _joint = representation.Joint(**config)
                    assert "constraint_axes" in config
                    _joint.constraint_axes = [ConstraintAxis(**ca) for ca in config["constraint_axes"]]
                    assert _joint.check_valid()
                    self.robot.add_aggregate("joint", _joint)
            if len(faulty_joint_defs) > 0:
                log.error("The following joint changes are defined but the joint does not exist:")
                for fjd in faulty_joint_defs:
                    log.error(f"- {fjd[0]} "+(f"Did you mean: {fjd[1]}" if len(fjd[1]) > 0 else ""))
                log.info("However, The following joins DO exist:")
                for joint in self.robot.joints:
                    log.info(f"- {joint}")
            remove_joints = []
            for joint in self.robot.joints:
                jointname = joint.name
                if jointname in self.joints or self.joints.get("$all", False):
                    config = misc.merge_default(self.joints[jointname] if jointname in self.joints else {}, _default)
                    for k, v in config.items():
                        if k == "remove" and v is True:
                            remove_joints.append(jointname)
                            break
                        if k in ["min", "max", "eff", "vel"]:
                            if joint.limit is None:
                                joint.limit = representation.JointLimit()
                        if k in ["friction", "damping", "spring_stiffness", "spring_reference"]:
                            if joint.dynamics is None:
                                joint.dynamics = representation.JointDynamics()
                        if k == "move_joint_axis_to_intersection":
                            self.robot.move_joint_to_intersection(jointname, v)
                        elif k == "type":
                            joint.joint_type = v
                        elif k == "min":
                            joint.limit.lower = misc.read_number_from_config(v)
                        elif k == "max":
                            joint.limit.upper = misc.read_number_from_config(v)
                        elif k == "eff":
                            joint.limit.effort = misc.read_number_from_config(v)
                        elif k == "vel":
                            joint.limit.velocity = misc.read_number_from_config(v)
                        elif k == "friction":
                            joint.dynamics.friction = misc.read_number_from_config(v)
                        elif k == "damping":
                            joint.dynamics.damping = misc.read_number_from_config(v)
                        elif k == "spring_stiffness":
                            joint.dynamics.spring_stiffness = misc.read_number_from_config(v)
                        elif k == "spring_reference":
                            joint.dynamics.spring_reference = misc.read_number_from_config(v)
                        elif k == "movement_depends_on":
                            for jd in v:
                                joint.joint_dependencies.append(representation.JointMimic(joint=jd["joint_name"],
                                                                                          offset=jd["offset"],
                                                                                          multiplier=jd["multiplier"]))
                        elif k == "mimic":
                            if "joint_name" in v:
                                v["joint"] = v.pop("joint_name")
                            joint.mimic = representation.JointMimic(**v)
                        elif k == "active" and v is True:
                            _motor = representation.Motor(name=jointname+"_motor", joint=jointname)
                            self.robot.add_motor(_motor)
                        elif k == "motor":
                            assert type(v) == dict
                            v = misc.merge_default(v, resources.get_default_motor())
                            if "name" not in v:
                                v["name"] = jointname+"_motor"
                            if "joint" not in v:
                                v["joint"] = jointname
                            else:
                                assert jointname == v["joint"]
                            _motor = representation.Motor(**v)
                            self.robot.add_motor(_motor)
                        else:  # axis
                            joint.add_annotation(k, v, overwrite=True)
                # elif "$default" in self.joints:
                #     config = _default
                #     for k, v in config.items():
                #         if k not in ["min", "max", "eff", "vel", "movement_depends_on", "mimic", "active", "name_editing", "move_joint_axis_to_intersection"] + representation.Joint._class_variables:
                #             joint.add_annotation(k, v, overwrite=True)
                # [TODO v2.1.0] Re-add transmission support
                joint.link_with_robot(self.robot)
            for joint in remove_joints:
                self.robot.remove_joint(joint)
            for joint in self.robot.joints:
                jointname = joint.name
                if jointname in self.joints or self.joints.get("$all", False):
                    config = misc.merge_default(self.joints[jointname] if jointname in self.joints else {}, _default)
                    joint.name = misc.edit_name_string(joint.name, **config.get("name_editing", {}), correspondance=joint)
            if "$name_editing" in self.joints:
                self.robot.rename("joints", self.robot.joints, **self.joints.get("$name_editing", {}))
        # as we mess manually with some names, we need to make sure the tree maps are up to date
        self.robot.regenerate_tree_maps()

        # Check for joint definitions
        self.robot.check_joint_definitions(
            raise_error=True,
            backup=self.joints["$backup"] if (
                    hasattr(self, "joints")
                    and "$backup" in self.joints.keys()
            ) else None
        )

        self.robot.clean_meshes()

        if hasattr(self, 'collisions'):
            if "$name_editing" in self.collisions.keys():
                self.robot.rename("collision", self.robot.collisions, **self.collisions["$name_editing"])
            for link in self.robot.links:
                conf = deepcopy(self.collisions["$default"])
                exclude = self.collisions["$exclude"] if "$exclude" in self.collisions.keys() else []
                if link.name in exclude:
                    continue
                if link.name in self.collisions.keys():
                    conf = misc.merge_default(self.collisions[link.name], conf)
                if "remove" in conf.keys():
                    if type(conf["remove"]) is list:
                        remove_collision(self.robot, link.name, collisionname=conf["remove"])
                    elif type(conf["remove"]) is str:
                        remove_collision(self.robot, link.name, collisionname=[c.name for c in link.collisions if re.fullmatch(r"" + conf["remove"], c.name) is not None])
                if "join" in conf.keys() and conf["join"] is True:
                    if len(link.collisions) > 1:
                        # print("       Joining meshes of", link.name)
                        join_collisions(self.robot, link.name, name_id=self.modelname)
                if "shape" in conf.keys():
                    if conf["shape"] == "remove":
                        remove_collision(self.robot, link.name)
                    elif conf["shape"] != "mesh":
                        if not (conf["shape"] == "convex" and "join" in conf.keys() and conf["join"] is True): # beacuse join_collisions already make convex hulls
                            replace_collision(
                                self.robot, link.name,
                                shape=conf["shape"],
                                oriented=conf["oriented"] if "oriented" in conf.keys() else True,
                                scale=conf["scale"] if "scale" in conf.keys() else 1.0,
                                apply_primitives=self.collisions.get("apply_primitives", False)
                            )
                    # leads to problems in reusing identic meshes
                    # if conf["shape"] == "convex":
                    #     reduceMeshCollision(self.robot, link.name, reduction=0.3)

            if "auto_bitmask" in self.collisions.keys() and \
                    self.collisions["auto_bitmask"] is True:
                log.debug("         Setting auto bitmask")
                kwargs = self.collisions["default"] if "default" in self.collisions.keys() else {}
                self.robot.set_self_collision(
                    1,
                    coll_override=self.collisions["collision_between"]
                    if "collision_between" in self.collisions.keys() else {},
                    no_coll_override=self.collisions["no_collision_between"]
                    if "no_collision_between" in self.collisions.keys() else {},
                    **kwargs
                )
                for coll in self.robot.get_all_collisions():
                    if coll.name in self.collisions.keys():
                        for k, v in self.collisions[coll.name].items():
                            setattr(coll, k, v)
            else:
                for coll_name in self.collisions.keys():
                    conf = self.collisions[coll_name]
                    coll = self.robot.get_collision_by_name(coll_name)
                    if coll is not None:
                        for key in ["name", "link", "geometry", "origin"]:
                            if key in conf:
                                conf.pop(key)
                        coll.add_annotations(**conf)

        if hasattr(self, 'visuals'):
            if "$name_editing" in self.visuals.keys():
                self.robot.rename("visuals", self.robot.visuals, **self.visuals["$name_editing"])
            for link in self.robot.links:
                conf = deepcopy(self.visuals["$default"])
                exclude = self.visuals["$exclude"] if "$exclude" in self.visuals.keys() else []
                if link.name in exclude:
                    continue
                if link.name in self.visuals.keys():
                    conf = misc.merge_default(self.visuals[link.name], conf)
                if "remove" in conf.keys():
                    if type(conf["remove"]) is list:
                        remove_visual(self.robot, link.name, visualname=conf["remove"])
                    elif type(conf["remove"]) is str:
                        remove_visual(self.robot, link.name, visualname=[c.name for c in link.visuals if re.fullmatch(r"" + conf["remove"], c.name) is not None])

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

        if hasattr(self, "sensors"):
            multi_sensors = [x for x in dir(sensor_representations) if
                             not x.startswith("__") and x not in sensor_representations.__IMPORTS__ and
                             issubclass(getattr(sensor_representations, x), sensor_representations.MultiSensor)]
            single_sensors = [x for x in dir(sensor_representations) if
                              not x.startswith(
                                  "__") and x not in sensor_representations.__IMPORTS__ and issubclass(
                                  getattr(sensor_representations, x),
                                  sensor_representations.Sensor) and x not in multi_sensors]
            moveable_joints = [j for j in self.robot.joints if j.joint_type != 'fixed']

            for s in self.sensors:
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

        if hasattr(self, "poses"):
            if "$default" in self.poses:
                assert type(self.poses["$default"]) in [float, int]
                default_config = {str(j): float(self.poses["$default"]) for j in self.robot.joints if j.joint_type != "fixed"}
            for (cn, config) in self.poses.items():
                if cn == "$default":
                    continue
                if "$default" in self.poses:
                    misc.merge_default(config, default_config)
                pose = poses.JointPoseSet(robot=self.robot, name=cn, configuration=config)
                self.robot.add_pose(pose)

        if hasattr(self, "annotations"):
            log.debug('  Adding further annotations.')

            if "general_annotations" in self.annotations.keys():
                for k, v in self.annotations["general_annotations"]:
                    if k in self.robot.annotations.keys():
                        self.robot.annotations[k] += v
                    else:
                        self.robot.annotations[k] = v

            if "named_annotations" in self.annotations.keys():
                for k, v in self.annotations["named_annotations"].items():
                    self.robot.add_categorized_annotation(k, v)

        log.info('Finished processing')
        return True

    def export(self):
        self.robot.link_entities()
        self.robot.submodel_defs = {} # we define these here per model
        ros_pkg_name = self.robot.export(outputdir=self.exportdir, export_config=self.export_config,
                                         rel_mesh_paths=self.pipeline.meshes, ros_pkg_later=True,
                                         reduce_meshes=getattr(self, "reduce_meshes", None),
                                         apply_scale=getattr(self, "apply_scale", None),
                                         ros_pkg_name=getattr(self, "ros_pkg_name", None),
                                         filename=getattr(self, "filename", None),
                                         mark_as_autogenerated=getattr(self, "mark_as_autogenerated", False)
                                         )
        for vc in self.robot.collisions + self.robot.visuals:
            if isinstance(vc.geometry, representation.Mesh):
                self.processed_meshes = self.processed_meshes.union([os.path.realpath(f["filepath"]) for f in vc.geometry._exported.values()])
                try:
                    self.processed_meshes.add(os.path.realpath(vc.geometry.abs_filepath))
                except IOError:
                    # This mesh has been edited and has no source mesh any more that could provide a file path
                    pass

        if "keep_files" in self.deployment:
            git.reset(self.targetdir, git.GIT_REMOTE_NAME, git.GIT_PRODUCTION_BRANCH, hard=False)
            misc.store_persisting_files(self.pipeline, self.targetdir, self.deployment["keep_files"], self.exportdir)

        log.info('Finished export of the new model')
        self.processed_model_exists = True

        if hasattr(self, "post_processing"):
            for script in self.post_processing:
                os.environ["PHOBOS_CI_EXPORTDIR"] = self.exportdir
                os.environ["PHOBOS_CI_TARGETDIR"] = self.targetdir
                os.environ["PHOBOS_CI_DEFINITIONDIR"] = self.pipeline.configdir
                if "cwd" not in script.keys():
                    script["cwd"] = self.exportdir
                else:
                    script["cwd"] = misc.abspath(script["cwd"])
                misc.execute_shell_command(script["cmd"], script["cwd"], verbose=True, executable=script.get("executable", None))
            log.info('Finished post_processing of the new model')

        if ros_pkg_name is not None:
            self.robot.export_ros_package_files(
                self.exportdir, ros_pkg_name,
                author=self.pipeline.relpath(self.configfile),
                maintainer="Phobos CI Pipeline",
                url=self.pipeline.remote_base + self.modelname,
                version=getattr(self, "version", self.pipeline.git_rev[10]),
                license=getattr(self, "license", None)
            )

    def deploy(self, mesh_commit, failed_model=False, uses_lfs=False):
        feature_branch = None
        if os.getenv("CI_COMMIT_BRANCH", "").startswith("feature/"):
            feature_branch = os.getenv("CI_COMMIT_BRANCH", None)
        if "do_not_deploy" in self.deployment and self.deployment["do_not_deploy"] is True:
            return "deployment suppressed in cfg"
        if not os.path.exists(self.targetdir):
            raise Exception("The result directory " + self.targetdir + " doesn't exist:\n" +
                            "  This might happen if you haven't added the result" +
                            " repo to the manifest.xml or spelled it wrong.")
        repo = self.targetdir
        # Clear local changes
        git.reset(self.targetdir, git.GIT_REMOTE_NAME, git.GIT_PRODUCTION_BRANCH)
        if "keep_files" in self.deployment:
            misc.store_persisting_files(self.pipeline, repo, self.deployment["keep_files"], os.path.join(self.tempdir, "_sustain"))
        # Fetch the latest changes from the target branch for a clean merge
        git.update(repo, update_target_branch=feature_branch if feature_branch else (git.GIT_FAILURE_BRANCH if failed_model else git.GIT_SUCCESS_BRANCH))
        # Remove everything so that we can implement the produced version instead
        git.clear_repo(repo)
        git.ignore(repo, "*\_history.log")
        if uses_lfs:
            track = ["*.obj", "*.stl", "*.dae", "*.bobj", "*.iv", "*.mtl", "*.OBJ", "*.STL", "*.DAE", "*.BOBJ", "*.IV", "*.MTL"]
            for file in getattr(self, "add_lfs_track", getattr(self.pipeline, "add_lfs_track", [])):
                if not file.startswith("*"):
                    if not file.startswith("."):
                        file = "."+file
                    file = "*"+file
                track.append(file)
                track.append(file.upper())
                track.append(file.lower())
            track = list(set(track))
            git.install_lfs(repo, track=track)
        # add manifest.xml if necessary
        manifest_path = os.path.join(repo, "manifest.xml")
        if getattr(self.deployment, "add_manifest", getattr(self.pipeline, "add_manifest", False)) and not os.path.isfile(manifest_path):
            misc.copy(self.pipeline, resources.get_resources_path("manifest.xml.in"), manifest_path)
            with open(manifest_path, "r") as manifest:
                content = manifest.read()
                url = self.pipeline.remote_base + self.modelname
                content = misc.regex_replace(content, {
                    "\$INPUTNAME": self.modelname,
                    "\$AUTHOR": "<author>" + os.path.join(self.pipeline.configdir,
                                                          self.modelname + ".yml") + "</author>",
                    "\$MAINTAINER": "<maintainer>https://git.hb.dfki.de/phobos/ci-run/-/wikis/home</maintainer>",
                    "\$URL": "<url>" + url + "</url>",
                })
            with open(manifest_path, "w") as manifest:
                manifest.write(content)
        readme_path = os.path.join(repo, "README.md")
        if not os.path.isfile(readme_path):
            misc.copy(self.pipeline, resources.get_resources_path("README.md.in"), readme_path)
            _url =  misc.regex_replace(os.getenv("CI_PROJECT_URL", "PipelineRepo://"), {"https:\/\/.*@":"https://"})
            if git.RUNNING_FOR_AZURE:
                config_link = os.path.join(_url+"?path=", os.path.relpath(self.configfile, self.pipeline.configdir)) + "&version=GB" + os.getenv("CI_COMMIT_BRANCH", "master").replace("/", "%2F")
            else:
                config_link = os.path.join(_url+"-/blob", os.getenv("CI_COMMIT_BRANCH", "master"), os.path.relpath(self.configfile, self.pipeline.configdir))
            with open(readme_path, "r") as readme:
                content = readme.read()
                content = misc.regex_replace(content, {
                    "\$MODELNAME": self.modelname,
                    "\$USERS": getattr(self.deployment, "mr_mention", getattr(self.pipeline, "mr_mention", "none")),
                    "\$DEFINITION_FILE": config_link
                })
            with open(readme_path, "w") as readme:
                readme.write(content)
        if uses_lfs:
            readme_content = open(readme_path, "r").read()
            if "Git LFS" not in readme_content:
                with open(readme_path, "a") as readme:
                    readme.write(open(resources.get_resources_path("GitLFS_README.md.in")).read())
        # update additional submodules
        if "submodules" in self.deployment:
            for subm in self.deployment["submodules"]:
                git.add_submodule(
                    repo,
                    subm["repo"],
                    subm["target"],
                    commit=subm["commit"] if "commit" in subm.keys() else None,
                    branch=subm["branch"] if "branch" in subm.keys() else git.GIT_SUCCESS_BRANCH
                )
        # update the mesh submodule
        if self.pipeline.central_meshes:
            for mt, mp in self.export_meshes.items():
                git.add_submodule(
                    repo,
                    os.path.relpath(
                        os.path.join(self.pipeline.root, str(mp)), repo
                    ),
                    mp,
                    commit=mesh_commit[mt]["commit"],
                    branch=os.environ["CI_MESH_UPDATE_TARGET_BRANCH"]
                    if "CI_MESH_UPDATE_TARGET_BRANCH" in os.environ.keys() else git.GIT_SUCCESS_BRANCH
                )

        # now we move back to the push repo
        misc.copy(self.pipeline, self.exportdir + "/*", self.targetdir, silent=False)
        if "keep_files" in self.deployment:
            misc.restore_persisting_files(self.pipeline, repo, self.deployment["keep_files"], os.path.join(self.tempdir, "_sustain"))
        commit_hash = git.commit(repo, origin_repo=self.pipeline.configdir)
        log.info("Created commit: "+commit_hash)
        git.add_remote(repo, self.pipeline.remote_base + self.modelname)
        if not feature_branch:
            mr = git.MergeRequest()
            mr.title = self.deployment["mr_title"]
            mr.description = self.deployment["mr_description"]
            if os.path.isfile(self.pipeline.test_protocol):
                log.info("Appending test_protocol to MR description")
                with open(self.pipeline.test_protocol, "r") as f:
                    protocol = load_json(f.read())
                    mr.description = misc.append_string(mr.description, "\n" + str(protocol["all"]))
                    mr.description = misc.append_string(mr.description, str(protocol[self.modelname]))
                if not bool(int(os.getenv("RUNNING_FOR_AZURE", 0))):
                    log.info("Appending extended test_protocol to MR description")
                    with open(self.extended_test_protocol_path, "r") as f:
                        protocol = f.read()
                        mr.description = misc.append_string(mr.description, "\n\n\n# Extended Protocol\n" + protocol)
                elif os.getenv("CI_EXTENDED_TEST_PROTOCOL_URL", None) is not None:
                    log.info("Appending link to extended test_protocol to MR description")
                    mr.description = misc.append_string(mr.description, "\n\n The extended test protocol can be found here: " + os.getenv("CI_EXTENDED_TEST_PROTOCOL_URL", "URL not available"))
            else:
                log.warning(f"Did not find test_protocol file at: {self.pipeline.test_protocol}")
            if "mr_mention" in self.deployment:
                mr.mention = self.deployment["mr_mention"]
            elif hasattr(self.pipeline, "mr_mention"):
                mr.mention = self.pipeline.mr_mention
            if failed_model:
                mr.target = self.deployment.get("mr_target_branch", git.GIT_SUCCESS_BRANCH)
                return_msg = "pushed to " + git.push(repo, branch=git.GIT_FAILURE_BRANCH,
                                                    merge_request=mr)
            else:
                mr.target = self.deployment.get("mr_target_branch", git.GIT_PRODUCTION_BRANCH)
                return_msg = "pushed to " + git.push(repo, branch=git.GIT_SUCCESS_BRANCH,
                                                    merge_request=mr if git.GIT_PRODUCTION_BRANCH != git.GIT_SUCCESS_BRANCH else None)
            # deploy to mirror
            if "mirror" in self.deployment:
                log.info(f"Deploying to mirror:\n {dump_yaml(self.deployment['mirror'], default_flow_style=False)}")
                mirror_dir = os.path.join(self.tempdir, "deploy_mirror")
                git.clone(
                    pipeline=self.pipeline,
                    repo=self.deployment["mirror"]["repo"],
                    target=mirror_dir,
                    branch=git.GIT_SUCCESS_BRANCH,
                    shallow=False,
                    recreate=True
                )
                git.update(mirror_dir, update_remote=git.GIT_REMOTE_NAME, update_target_branch=self.deployment["mirror"]["branch"])
                git.clear_repo(mirror_dir)
                submodule_dict = {}
                if "submodules" in self.deployment["mirror"].keys() and ".gitmodules" in os.listdir(repo):
                    submodule_file = open(os.path.join(repo, ".gitmodules"), "r").read()
                    current_key = None
                    for line in submodule_file.split("\n"):
                        if line.startswith("["):
                            current_key = line.split()[1][1:-2]
                            if current_key not in submodule_dict.keys():
                                submodule_dict[current_key] = {}
                        elif " = " in line:
                            k, v = line.strip().split(" = ")
                            submodule_dict[current_key][k] = v
                    if self.deployment["mirror"]["submodules"]:
                        for _, sm in submodule_dict.items():
                            git.add_submodule(mirror_dir, sm["url"], sm["path"],
                                            branch=sm["branch"] if "branch" in sm.keys() else git.GIT_SUCCESS_BRANCH)
                misc.copy(self.pipeline, repo + "/*", mirror_dir)
                if "submodules" in self.deployment["mirror"].keys() and ".gitmodules" in os.listdir(repo):
                    for _, sm in submodule_dict.items():
                        # git.clone(self.pipeline, os.path.join(self.deployment["mirror"]["repo"], sm["url"]), sm["path"],
                        #           sm["branch"] if "branch" in sm.keys() else git.GIT_SUCCESS_BRANCH, cwd=mirror_dir)
                        misc.execute_shell_command("rm -rf " + os.path.join(sm["path"], ".git*"), mirror_dir)
                git.commit(mirror_dir, origin_repo=self.pipeline.configdir)
                if "merge_request" in self.deployment["mirror"].keys() and self.deployment["mirror"]["merge_request"]:
                    if "mr_mention" in self.deployment["mirror"].keys():
                        mr.mention = self.deployment["mirror"]["mr_mention"]
                    if "mr_title" in self.deployment["mirror"].keys():
                        mr.title = self.deployment["mirror"]["mr_title"]
                    if "mr_target" in self.deployment["mirror"].keys():
                        mr.target = self.deployment["mirror"]["mr_target"]
                    git.push(mirror_dir, remote=git.GIT_REMOTE_NAME, merge_request=mr, branch=self.deployment["mirror"]["branch"])
                else:
                    git.push(mirror_dir, remote=git.GIT_REMOTE_NAME, branch=self.deployment["mirror"]["branch"])
                return_msg += "& pushed to mirror"
        else:
            return_msg = "pushed to " + git.push(repo, branch=feature_branch)
        #git.checkout(repo, branch=git.GIT_SUCCESS_BRANCH)
        return return_msg

    def get_input_meshes(self):
        return self._meshes
