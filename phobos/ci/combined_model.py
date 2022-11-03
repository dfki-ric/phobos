import os
from copy import deepcopy

import filecmp
import numpy as np

from ..core import Robot
from ..utils.misc import regex_replace
from ..io import representation

from ..ci.base_model import BaseModel
from ..utils import git
from ..utils import misc

from ..utils.commandline_logging import get_logger
log = get_logger(__name__)


class CombinedModel(BaseModel):
    def __init__(self, configfile, pipeline, processed_model_exists=True, super_call=False):
        super().__init__(configfile, pipeline, processed_model_exists)
        if not super_call:
            assert "combined_model" in self.cfg.keys()

            self.basedir = os.path.join(self.tempdir, "combined_model")
            self.basefile = os.path.join(self.basedir, "smurf", "combined_model.smurf")

            if self.processed_model_exists:
                self._load_robot()

            log.debug(f"Finished reading config and joining models to base model  {configfile}")

    def process(self):
        misc.recreate_dir(self.pipeline, self.tempdir)
        misc.recreate_dir(self.pipeline, self.basedir)

        self._join_to_basefile()
        self._load_robot()

        if not hasattr(self, 'robot') or not hasattr(self, 'pipeline'):
            raise Exception('Model is not initialized!')

        super()._process()

    def _join_to_basefile(self):
        if not (hasattr(self, "join") and hasattr(self, "depends_on")):
            raise Exception("This model definition is not a valid definition of a combined model!\n"
                            "It lacks the 'join' and the 'depends_on' definition")
        # get all the models we need
        self.dep_models = {}
        for name, config in self.depends_on.items():
            if "derived_base" in config.keys():
                self.dep_models.update({
                    name: BaseModel(
                        os.path.join(self.pipeline.configdir, config["derived_base"]),
                        self.pipeline, processed_model_exists=True)
                })
                # copy the mesh files to the temporary combined model directory
                misc.create_symlink(
                    self.pipeline,
                    os.path.join(self.pipeline.temp_dir, self.dep_models[name].typedef["meshespath"]),
                    os.path.join(self.basedir, self.dep_models[name].typedef["meshespath"])
                )
        for name, config in self.depends_on.items():
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
                                submechanisms_file=os.path.join(repo_path, config["repo"]["submechanisms_in_repo"]) if "submechanisms_in_repo" in config["repo"] else None,
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
        if isinstance(self.dep_models[self.join["model"]], Robot):
            combined_model = self.dep_models[self.join["model"]].duplicate()
        else:  # it must be an instance of BaseModel
            combined_model = self.dep_models[self.join["model"]].robot.duplicate()
        combined_model.name = self.robotname
        combined_model.autogenerate_submechanisms = False
        combined_model.smurffile = self.basefile
        combined_model.xmlfile = os.path.join(self.basedir, "urdf", "combined_model.urdf")

        if "name_prefix" in self.join.keys() or "name_replacements" in self.join.keys():
            if "name_prefix" not in self.join.keys():
                self.join["name_prefix"] = ""
            if "name_replacements" not in self.join.keys():
                self.join["name_replacements"] = {}
            if "children" in self.join.keys():
                for c in self.join["children"]:
                    c["joint"]["parent"] = regex_replace(c["joint"]["parent"], self.join["name_replacements"])
                    if not c["joint"]["parent"].startswith(self.join["name_prefix"]):
                        c["joint"]["parent"] = self.join["name_prefix"] + c["joint"]["parent"]
            for lnk in combined_model.links:
                combined_model.rename(targettype="link", target=lnk.name, prefix=self.join["name_prefix"],
                                      replacements=self.join["name_replacements"])
                for coll in lnk.collisions:
                    combined_model.rename(targettype="collision", target=coll.name,
                                          prefix=self.join["name_prefix"],
                                          replacements=self.join["name_replacements"])
                    # att_model.rename(targettype="collision", target=coll.name,
                    #                  prefix=child["collision_prefix"], suffix=child["collision_suffix"],
                    #                  replacements=child["collision_replacements"])
                for vis in lnk.visuals:
                    combined_model.rename(targettype="visual", target=vis.name,
                                          prefix=self.join["name_prefix"],
                                          replacements=self.join["name_replacements"])
                    # att_model.rename(targettype="visual", target=coll.name,
                    #                  prefix=child["visual_prefix"], suffix=child["visual_suffix"],
                    #                  replacements=child["visual_replacements"])
            for jnt in combined_model.joints:
                combined_model.rename(targettype="joint", target=jnt.name, prefix=self.join["name_prefix"],
                                      replacements=self.join["name_replacements"])

        if "remove_beyond" in self.join.keys():
            combined_model = combined_model.get_before(self.join["remove_beyond"])

        if "take_leaf" in self.join.keys():
            assert type(self.join["take_leaf"]) == str
            combined_model = combined_model.get_beyond(self.join["take_leaf"])

        if "mirror" in self.join.keys():
            combined_model.mirror_model(
                mirror_plane=self.join["mirror"]["plane"] if "plane" in self.join["mirror"].keys() else [0, 1, 0],
                maintain_order=self.join["mirror"]["maintain_order"] if "maintain_order" in self.join[
                    "mirror"].keys() else [0, 2, 1],
                exclude_meshes=self.join["mirror"]["exclude_meshes"] if "exclude_meshes" in self.join[
                    "mirror"].keys() else [],
                name_replacements=self.join["mirror"]["name_replacements"] if "name_replacements" in self.join[
                    "mirror"].keys() else {},
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

                if child["joint"]["parent"] is None:
                    child["joint"]["parent"] = str(parent_model.get_root())
                if "child" not in child["joint"].keys() or child["joint"]["child"] is None:
                    child["joint"]["child"] = str(att_model.get_root())
                    if "take_leaf" in child:
                        child["joint"]["child"] = child["take_leaf"]
                if "name" not in child["joint"].keys() or child["joint"]["name"] is None:
                    child["joint"]["name"] = child["joint"]["parent"] + "2" + child["joint"]["child"]
                if "type" not in child["joint"].keys() or child["joint"]["type"] is None:
                    child["joint"]["type"] = "fixed"
                if "r2r_transform" in child["joint"].keys():
                    T = np.array(child["joint"]["r2r_transform"])
                    src_T = parent_model.get_transformation(child["joint"]["parent"])
                    dst_T = att_model.get_transformation(child["joint"]["child"])
                    T = np.linalg.inv(src_T).dot(T).dot(dst_T)
                    origin = representation.Pose.from_matrix(T)
                    child["joint"]["xyz"] = origin.xyz
                    child["joint"]["rpy"] = origin.rpy

                if "name_prefix" in child.keys() or "name_replacements" in child.keys():
                    if "name_prefix" not in child.keys():
                        child["name_prefix"] = ""
                    if "name_replacements" not in child.keys():
                        child["name_replacements"] = {}
                    child["joint"]["child"] = regex_replace(child["joint"]["child"], child["name_replacements"])
                    if not child["joint"]["child"].startswith(child["name_prefix"]):
                        child["joint"]["child"] = child["name_prefix"] + child["joint"]["child"]
                    if not child["joint"]["name"].startswith(child["name_prefix"]):
                        child["joint"]["name"] = child["name_prefix"] + child["joint"]["name"]

                    if "children" in child.keys():
                        for cchild in child["children"]:
                            cchild["joint"]["parent"] = regex_replace(cchild["joint"]["parent"],
                                                                      child["name_replacements"])
                            if not cchild["joint"]["parent"].startswith(child["name_prefix"]):
                                cchild["joint"]["parent"] = child["name_prefix"] + cchild["joint"]["parent"]

                    for lnk in att_model.links:
                        att_model.rename(targettype="link", target=lnk.name, prefix=child["name_prefix"],
                                      replacements=child["name_replacements"])
                        for coll in lnk.collisions:
                            att_model.rename(targettype="collision", target=coll.name,
                                          prefix=child["name_prefix"],
                                          replacements=child["name_replacements"])
                            # att_model.rename(targettype="collision", target=coll.name,
                            #                  prefix=child["collision_prefix"], suffix=child["collision_suffix"],
                            #                  replacements=child["collision_replacements"])
                        for vis in lnk.visuals:
                            att_model.rename(targettype="visual", target=vis.name,
                                          prefix=child["name_prefix"],
                                          replacements=child["name_replacements"])
                            # att_model.rename(targettype="visual", target=coll.name,
                            #                  prefix=child["visual_prefix"], suffix=child["visual_suffix"],
                            #                  replacements=child["visual_replacements"])
                    for jnt in att_model.joints:
                        att_model.rename(targettype="joint", target=jnt.name, prefix=child["name_prefix"],
                                      replacements=child["name_replacements"])

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
                        maintain_order=child["mirror"]["maintain_order"] if "maintain_order" in child[
                            "mirror"].keys() else [0, 2, 1],
                        exclude_meshes=child["mirror"]["exclude_meshes"] if "exclude_meshes" in child[
                            "mirror"].keys() else [],
                        name_replacements=child["mirror"]["name_replacements"] if "name_replacements" in child[
                            "mirror"].keys() else {},
                        target_urdf=combined_model.xmlfile
                    )

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
                assert att_model.get_joint(child["joint"]["name"]) is None and parent_model.get_joint(child["joint"]["name"]) is None, f'Can not join using joint name {child["joint"]["name"]} as this name already exists.'
                joint = representation.Joint(
                    name=child["joint"]["name"],
                    parent=parent.get_link(child["joint"]["parent"]).name,
                    child=att_model.get_link(child["joint"]["child"]).name,
                    joint_type=child["joint"]["type"],
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

        if "children" in self.join.keys() and len(self.join["children"]) > 0:
            recursive_attach(combined_model, self.join["children"], parentname=self.join["model"])
            combined_model.relink_entities()

        # 3. save combined_model to the temp directory
        assert len(combined_model.links) == len(combined_model.joints) + 1
        combined_model.name = "combined_model"
        combined_model.full_export(self.basedir)

    def _load_robot(self):
        if not self.processed_model_exists:
            if os.path.exists(os.path.join(self.basedir, "smurf", "combined_model.smurf")):
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                   smurffile=os.path.join(self.basedir, "smurf", "combined_model.smurf"))
            else:
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                   xmlfile=self.basefile)
            return
        else:
            if not os.path.isfile(self.exporturdf):
                raise Exception('Preprocessed file {} not found!'.format(self.exporturdf))
            if os.path.exists(os.path.join(self.exportdir, "smurf", self.robotname + ".smurf")):
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                   smurffile=os.path.join(self.exportdir, "smurf", self.robotname + ".smurf"))
            else:
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                   xmlfile=self.exporturdf)
