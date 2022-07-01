import os
import sys
import re
import pkg_resources
import yaml
from copy import deepcopy

from .. import smurf as ps
from ..geometry import replace_collision, join_collisions, remove_collision, import_mesh, import_mars_mesh, \
    export_mesh, export_mars_mesh, export_bobj_mesh
from ..smurf import Smurf as Robot
from ..smurf.hyrodyn import JointDependency, MultiJointDependency
from ..utils import misc, git, urdf, transform, tree
from ..io import representation

SUBMECHS_VIA_ASSEMBLIES = False


class BaseModel(yaml.YAMLObject):
    def __init__(self, configfile, pipeline, processed_model_exists=True):
        if type(configfile) is str:
            if not os.path.isfile(configfile):
                raise Exception('{} not found!'.format(configfile))
            self.cfg = yaml.safe_load(open(configfile, 'r'))
        else:
            self.cfg = configfile

        kwargs = {}
        if 'combined_model' in self.cfg.keys():
            kwargs = self.cfg['combined_model']
        elif 'model' in self.cfg.keys():
            kwargs = self.cfg['model']
        elif 'xtype_model' in self.cfg.keys():
            kwargs = self.cfg['xtype_model']
        for (k, v) in kwargs.items():
            setattr(self, k, v)

        self.processed_model_exists = processed_model_exists
        self.pipeline = pipeline

        # self.tempfile = None
        self.submech2urdf = []
        self.exportdir = os.path.join(self.pipeline.temp_dir, self.modelname)
        self.submechanisms_path = os.path.join(self.exportdir, "submechanisms")
        self.exporturdf = os.path.join(self.exportdir, "urdf", self.robotname + ".urdf")
        self.exportsmurf = os.path.join(self.exportdir, "smurf", self.robotname + ".smurf")
        self.exportsubmechs = os.path.join(self.exportdir, "smurf", self.robotname + "_submechanisms.yml")
        self.tempdir = os.path.join(self.pipeline.temp_dir, "temp_" + self.modelname)
        # self.tempfile = os.path.join(self.tempdir,)
        self.targetdir = os.path.join(self.pipeline.root, self.modelname)

        # list directly imported mesh pathes
        self._meshes = []
        if hasattr(self, "basefile"):
            r = Robot(xmlfile=self.basefile)
            for link in r.links:
                for g in link.visuals + link.collisions:
                    if hasattr(g.geometry, "filename"):
                        self._meshes += [urdf.read_urdf_filename(g.geometry.filename, self.basefile)]
        elif hasattr(self, "depends_on"):
            for _, v in self.depends_on.items():
                if "basefile" in v.keys():
                    r = Robot(xmlfile=v["basefile"])
                    for link in r.links:
                        for g in link.visuals + link.collisions:
                            if hasattr(g.geometry, "filename"):
                                self._meshes += [urdf.read_urdf_filename(g.geometry.filename[:-4], v["basefile"])]
        elif hasattr(self, "repo"):
            repo_path = os.path.join(self.tempdir, "repo", os.path.basename(self.repo["git"]))
            git.clone(
                self,
                self.repo["git"],
                repo_path,
                commit_id=self.repo["commit"],
                recursive=True,
                ignore_failure=True
            )
            self.basefile = os.path.join(repo_path, self.repo["model_in_repo"])
            r = Robot(inputfile=self.basefile)
            for link in r.links:
                for g in link.visuals + link.collisions:
                    if hasattr(g.geometry, "filename"):
                        self._meshes += [urdf.read_urdf_filename(g.geometry.filename[:-4], self.basefile)]

        if self.modeltype in pipeline.modeltypes.keys():
            self.typedef = pipeline.modeltypes[self.modeltype]
        else:
            print('Model type {} not known using default!'.format(self.modeltype), flush=True)
            self.typedef = pipeline.modeltypes["default"]
        self.ros_pkg_name = self.modelname if "ros_package" in self.typedef.keys() and self.typedef[
            "ros_package"] else None
        if hasattr(self, "ros_package_name") and "ros_package" in self.typedef.keys() and self.typedef["ros_package"]:
            self.ros_pkg_name = self.ros_package_name

        if self.processed_model_exists or (hasattr(self, "basefile") and os.path.isfile(self.basefile)):
            self._load_robot()

    @staticmethod
    def get_exported_model_path(pipeline, configfile):
        cfg = yaml.safe_load(open(configfile, 'r'))
        if "model" in cfg.keys():
            cfg = cfg['model']
        elif "combined_model" in cfg.keys():
            cfg = cfg['combined_model']
        return os.path.join(pipeline.temp_dir, cfg["modelname"], "urdf", cfg["robotname"] + ".urdf")

    @property
    def tolerances(self):
        """Convenience getter related to class TestModel"""
        return self.typedef['compare_model']

    @property
    def urdf(self):
        """Convenience getter related to class TestModel"""
        return self.exporturdf

    @property
    def floatingbase_urdf(self):
        """Convenience getter related to class TestModel"""
        return self.urdf[:-5]+"_floatingbase.urdf"

    @property
    def modeldir(self):
        """Convenience getter related to class TestModel"""
        return self.exportdir

    @property
    def floatingbase(self):
        return hasattr(self, "export_floatingbase") and self.export_floatingbase is True

    @property
    def submechanisms_file_path(self):
        return self.exportsubmechs

    @property
    def floatingbase_submechanisms_file_path(self):
        return self.exportsubmechs.replace("_submechanisms", "_floatingbase_submechanisms")

    def _load_robot(self):
        if not self.processed_model_exists and hasattr(self, "basefile"):
            if not os.path.isfile(self.basefile):
                raise Exception('{} not found!'.format(self.basefile))
            if self.basefile.endswith("smurf"):
                self.robot = ps.Smurf(name=self.robotname if self.robotname else None,
                                      smurffile=self.basefile)
            else:
                self.robot = ps.Smurf(name=self.robotname if self.robotname else None,
                                      xmlfile=self.basefile)
        else:
            if not os.path.isfile(self.exporturdf):
                raise Exception('Preprocessed file {} not found!'.format(self.exporturdf))
            if os.path.exists(self.exportsmurf):
                self.robot = ps.Smurf(name=self.robotname if self.robotname else None,
                                      smurffile=self.exportsmurf)
            else:
                self.robot = ps.Smurf(name=self.robotname if self.robotname else None,
                                      xmlfile=self.exporturdf)

    def recreate_sym_links(self):
        misc.create_symlink(self.pipeline,
                            os.path.join(self.pipeline.temp_dir, self.typedef["meshespath"]),
                            os.path.join(self.exportdir, self.typedef["meshespath"])
                            )
        if "also_export_bobj" in self.typedef.keys() and self.typedef["also_export_bobj"]:
            misc.create_symlink(self.pipeline,
                                os.path.join(self.pipeline.temp_dir, self.pipeline.meshes["bobj"]),
                                os.path.join(self.exportdir, self.pipeline.meshes["bobj"])
                                )
        if hasattr(self, "export_kccd"):
            misc.create_symlink(self.pipeline,
                                os.path.join(self.pipeline.temp_dir, self.pipeline.meshes["iv"]),
                                os.path.join(self.exportdir, self.pipeline.meshes["iv"]))

    def _process(self):
        print('Start processing robot', flush=True)

        self._load_robot()

        self.robot.correct_inertials()
        # self.robot.correct_axes()

        name_editing_dict = {}
        if hasattr(self, "name_editing") and self.name_editing is not None:
            name_editing_dict = self.name_editing
        if hasattr(self, "name_editing_before") and self.name_editing_before is not None:
            name_editing_dict = self.name_editing_before
        if hasattr(self, "append_link_suffix"):
            name_editing_dict["append_link_suffix"] = self.append_link_suffix
        if name_editing_dict != {}:
            self.robot.edit_names(name_editing_dict)

        if hasattr(self, "take_leaf"):
            self.robot.remove_before(self.take_leaf)

        if hasattr(self, "remove_beyond"):
            if type(self.remove_beyond) is list:
                for x in self.join["remove_beyond"]:
                    self.robot.remove_beyond(x)
            else:
                self.robot.remove_beyond(self.remove_beyond)

        if hasattr(self, "transform_links"):
            for k, v in self.transform_links.items():
                transformation = transform.create_transformation(
                    xyz=v["xyz"] if "xyz" in v.keys() else [0, 0, 0],
                    rpy=v["rpy"] if "rpy" in v.keys() else [0, 0, 0]
                )
                # if "transform" in v.keys() and v["transform"] == "TO":
                #     if self.robot.getParent(k) is not None:
                #         transformation = inv(Homogeneous(self.robot.getJoint(self.robot.getParent(k)[0]).origin)
                #                              ).dot(transformation)
                # else: BY
                self.robot.transform_link_orientation(linkname=k, transformation=transformation,
                                                      only_frame=v["only_frame"] if "only_frame" in v.keys() else True,
                                                      transform_to="transform" in v.keys() and v["transform"] == "TO")
                print('       {}'.format(k), flush=True)

        if hasattr(self, "move_link_in_tree"):
            for link in self.move_link_in_tree:
                ln = link["linkName"] if "linkName" in link.keys() else link["link_name"]
                pn = link["newParentName"] if "newParentName" in link.keys() else link["new_parent_name"]
                self.robot.move_link_in_tree(link_name=ln, new_parent_name=pn)

        if hasattr(self, 'remove_joints'):
            for j in self.remove_joints:
                self.robot.remove_joint(j, keep_collisions=True)

        if hasattr(self, 'add_frames'):
            for j in self.add_frames:
                self.robot.add_link_by_properties(
                    j["name"], j["xyz"], j["rpy"], j["parent"],
                    jointname=j["jointname"] if "jointname" in j.keys() else None,
                    jointtype=j["type"] if "type" in j.keys() else "fixed",
                    axis=j["axis"] if "axis" in j.keys() else [0, 0, 1],
                    mass=j["mass"] if "mass" in j.keys() else 0.0
                )

        if hasattr(self, "move_joints_to_intersection"):
            for mj_name, tjj_names in self.move_joints_to_intersection.items():
                self.robot.move_joint_to_intersection(mj_name, tjj_names)

        if hasattr(self, 'replace_joint_types'):
            for joint in self.robot.joints:
                for old, new in self.replace_joint_types.items():
                    if joint.type == old:
                        joint.type = new

        if hasattr(self, 'redefine_articulation'):
            transmissions = {}
            for joint in self.robot.joints:
                if joint.type == "fixed":
                    continue
                elif joint.name in self.redefine_articulation.keys():
                    # print("Overriding joint definition for", joint.name, flush=True)
                    if joint.limit is None:
                        joint.limit = representation.JointLimit()
                    if "type" in self.redefine_articulation[joint.name].keys():
                        joint.type = self.redefine_articulation[joint.name]["type"]
                    if "axis" in self.redefine_articulation[joint.name].keys():
                        joint.axis = self.redefine_articulation[joint.name]["axis"]
                    if "min" in self.redefine_articulation[joint.name].keys():
                        joint.limit.lower = misc.read_angle_2_rad(self.redefine_articulation[joint.name]["min"])
                    if "max" in self.redefine_articulation[joint.name].keys():
                        joint.limit.upper = misc.read_angle_2_rad(self.redefine_articulation[joint.name]["max"])
                    if "vel" in self.redefine_articulation[joint.name].keys():
                        joint.limit.velocity = self.redefine_articulation[joint.name]["vel"]
                    if "eff" in self.redefine_articulation[joint.name].keys():
                        joint.limit.effort = self.redefine_articulation[joint.name]["eff"]
                    if "mimic" in self.redefine_articulation[joint.name].keys():
                        mimic = self.redefine_articulation[joint.name]["mimic"]
                        joint.mimic = representation.JointMimic(joint=mimic["joint_name"], offset=mimic["offset"],
                                                          multiplier=mimic["multiplier"])
                    if "transmission" in self.redefine_articulation[joint.name].keys():
                        tm = self.redefine_articulation[joint.name]["transmission"]
                        jds = [JointDependency(joint_name=mimic["joint_name"],
                                               offset=mimic["offset"],
                                               multiplier=mimic["multiplier"])
                               for mimic in tm["mimics"]]
                        mjd = MultiJointDependency(name=tm["name"], joint=joint.name, joint_dependencies=jds)
                        self.robot.add_transmission(mjd)

                    if "smurf" in self.modeltype \
                            and ("reducedDataPackage" in self.redefine_articulation[joint.name].keys()
                                 or "noDataPackage" in self.redefine_articulation[joint.name].keys()):
                        self.robot.smurf_joints += [ps.Joint(
                            robot=self.robot,
                            joint=joint.name,
                            reducedDataPackage=self.redefine_articulation[joint.name]["reducedDataPackage"]
                            if "reducedDataPackage" in self.redefine_articulation[joint.name].keys() else False,
                            noDataPackage=self.redefine_articulation[joint.name]["noDataPackage"]
                            if "noDataPackage" in self.redefine_articulation[joint.name].keys() else False,
                            damping_const_constraint_axis1=self.redefine_articulation[joint.name][
                                "damping_const_constraint_axis1"]
                            if "damping_const_constraint_axis1" in self.redefine_articulation[
                                joint.name].keys() else False,
                            springDamping=self.redefine_articulation[joint.name]["springDamping"]
                            if "springDamping" in self.redefine_articulation[joint.name].keys() else False,
                            springStiffness=self.redefine_articulation[joint.name]["springStiffness"]
                            if "springStiffness" in self.redefine_articulation[joint.name].keys() else False,
                            spring_const_constraint_axis1=self.redefine_articulation[joint.name][
                                "spring_const_constraint_axis1"]
                            if "spring_const_constraint_axis1" in self.redefine_articulation[
                                joint.name].keys() else False
                        )]
                elif "default" in self.redefine_articulation.keys():
                    if joint.limit is None:
                        joint.limit = representation.JointLimit()
                    if "min" in self.redefine_articulation["default"].keys() and (
                            joint.limit is None or joint.limit.lower is None):
                        joint.limit.lower = misc.read_angle_2_rad(self.redefine_articulation["default"]["min"])
                    if "max" in self.redefine_articulation["default"].keys() and (
                            joint.limit is None or joint.limit.upper is None):
                        joint.limit.upper = misc.read_angle_2_rad(self.redefine_articulation["default"]["max"])
                    if "vel" in self.redefine_articulation["default"].keys() and (
                            joint.limit is None or joint.limit.velocity is None):
                        joint.limit.velocity = self.redefine_articulation["default"]["vel"]
                    if "eff" in self.redefine_articulation["default"].keys() and (
                            joint.limit is None or joint.limit.effort is None):
                        joint.limit.effort = self.redefine_articulation["default"]["eff"]
                    if "smurf" in self.modeltype \
                            and ("reducedDataPackage" in self.redefine_articulation["default"].keys()
                                 or "noDataPackage" in self.redefine_articulation["default"].keys()):
                        self.robot.smurf_joints += [ps.Joint(
                            robot=self.robot,
                            joint=joint.name,
                            reducedDataPackage=self.redefine_articulation["default"]["reducedDataPackage"]
                            if "reducedDataPackage" in self.redefine_articulation["default"].keys() else False,
                            noDataPackage=self.redefine_articulation["default"]["noDataPackage"]
                            if "noDataPackage" in self.redefine_articulation["default"].keys() else False,
                            damping_const_constraint_axis1=self.redefine_articulation["default"][
                                "damping_const_constraint_axis1"]
                            if "damping_const_constraint_axis1" in self.redefine_articulation[
                                "default"].keys() else False,
                            springDamping=self.redefine_articulation["default"]["springDamping"]
                            if "springDamping" in self.redefine_articulation["default"].keys() else False,
                            springStiffness=self.redefine_articulation["default"]["springStiffness"]
                            if "springStiffness" in self.redefine_articulation["default"].keys() else False,
                            spring_const_constraint_axis1=self.redefine_articulation["default"][
                                "spring_const_constraint_axis1"]
                            if "spring_const_constraint_axis1" in self.redefine_articulation[
                                "default"].keys() else False
                        )]
                # else:
                # print("    Leaving joint", joint.name, "(", joint.type, ") untouched", flush=True)
            for _, transmission in transmissions.items():
                self.robot.add_aggregate("transmission", transmission)

        # Check for joint definitions
        self.robot.check_joint_definitions(
            raise_error=self.typedef["ensure_valid_joints"] if "ensure_valid_joints" in self.typedef.keys() else True,
            backup=self.redefine_articulation["default"] if (
                    hasattr(self, "redefine_articulation")
                    and "default" in self.redefine_articulation.keys()
                    and "backup" in self.redefine_articulation["default"].keys()
                    and self.redefine_articulation["default"]["backup"]
            ) else None,
        )

        self.robot.clean_meshes()

        if hasattr(self, 'estimate_missing_coms') and self.estimate_missing_coms:
            self.robot.set_estimated_link_com(self.robot.links, dont_overwrite=True)

        if hasattr(self, 'replace_collisions'):
            self.edit_collisions = self.replace_collisions
        if hasattr(self, 'edit_collisions'):
            print('  Replacing collisions', flush=True)
            for link in self.robot.links:
                conf = deepcopy(self.edit_collisions["default"])
                exclude = self.edit_collisions["exclude"] if "exclude" in self.edit_collisions.keys() else []
                if link.name in exclude:
                    continue
                print('       {}'.format(link.name), flush=True)
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

        if "smurf" in self.modeltype and hasattr(self, "smurf"):
            print('  Smurfing Collisions', flush=True)

            if 'collisions' in self.smurf.keys():
                if "auto_bitmask" in self.smurf["collisions"].keys() and \
                        self.smurf["collisions"]["auto_bitmask"] is True:
                    print("         Setting auto bitmask", flush=True)
                    kwargs = self.smurf["collisions"]["default"] if "default" in self.smurf["collisions"].keys() else {}
                    self.robot.set_self_collision(
                        1,
                        coll_override=self.smurf["collisions"]["collision_between"]
                        if "collision_between" in self.smurf["collisions"].keys() else {},
                        no_coll_override=self.smurf["collisions"]["no_collision_between"]
                        if "no_collision_between" in self.smurf["collisions"].keys() else {},
                        **kwargs
                    )
                    for coll in self.robot.collisions:
                        if coll.name in self.smurf["collisions"].keys():
                            for k, v in self.smurf["collisions"][coll.name].items():
                                setattr(coll, k, v)
                else:
                    smurf_collisions = []
                    for link in self.robot.links:
                        for coll in link.collisions:
                            conf = self.smurf["collisions"]["default"] if "default" in self.smurf[
                                "collisions"].keys() else {}
                            if coll.name in self.smurf["collisions"].keys():
                                conf = self.smurf["collisions"][coll.name]
                            smurf_collisions += [
                                ps.Collision(
                                    robot=self.robot,
                                    link=link,
                                    collision=coll,
                                    **conf
                                )
                            ]
                    self.robot.collisions = smurf_collisions

        print('  Re-exporting meshes', flush=True)
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
        processed_meshes = []
        for link in self.robot.links:
            v_c = 0
            for v in link.visuals + link.collisions:
                if isinstance(v.geometry, representation.Mesh) and v.geometry.filename not in processed_meshes:
                    v_c += 1
                    if "mars_obj" in v.geometry.filename.lower() or (
                            "input_meshes_are_mars_obj" in self.typedef.keys() and
                            self.typedef["input_meshes_are_mars_obj"]):
                        mesh = import_mars_mesh(v.geometry.filename, urdf_path=os.path.dirname(self.basefile))
                    elif v.geometry.filename.lower().endswith("bobj"):
                        raise NotImplementedError("Can't load bobj meshes!")
                    else:
                        if v.geometry.filename.lower().endswith("obj"):
                            print("WARNING: Loading obj mesh, where the orientation convention might be unknown")
                        mesh = import_mesh(v.geometry.filename, urdf_path=os.path.dirname(self.basefile))
                    meshexport = os.path.join(self.exportdir, self.typedef["meshespath"],
                                              os.path.basename(v.geometry.filename)[:-3])
                    b_meshexport = os.path.join(self.exportdir, self.pipeline.meshes["bobj"],
                                                os.path.basename(v.geometry.filename)[:-3])
                    v.geometry.filename = v.geometry.filename.replace(" ", "_")
                    v.geometry.filename = os.path.relpath(meshexport, os.path.join(self.exportdir, "urdf"))
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
                            if m.name != v.material.name:
                                continue
                            else:
                                color = m.color.rgba
                        export_mesh(mesh, meshexport + "dae", urdf_path=self.exporturdf, dae_mesh_color=color)
                        v.geometry.filename += "dae"
                    else:
                        raise ValueError("Unknown mesh type" + self.typedef["output_mesh_format"].lower())
                    if self.typedef["output_mesh_format"].lower() != "bobj" and \
                            "also_export_bobj" in self.typedef.keys() and self.typedef["also_export_bobj"]:
                        export_bobj_mesh(mesh, b_meshexport + "bobj", urdf_path=self.exporturdf)
                    processed_meshes.append(v.geometry.filename)
            print('       {} with {} meshes as {}'.format(link.name, v_c, self.typedef["output_mesh_format"]),
                  flush=True)

        if hasattr(self, "name_editing_after") and self.name_editing_after is not None:
            self.robot.edit_names(self.name_editing_after)

        if hasattr(self, "submechanisms_file"):
            self.robot.load_submechanisms(deepcopy(self.submechanisms_file))
            if hasattr(self, "export_total_submechanisms"):
                # add all to one urdf
                spanningtree = []
                for sm in self.submechanisms_file["submechanisms"]:
                    spanningtree += sm["jointnames_spanningtree"]
                spanningtree = list(set(spanningtree))
                root = tree.find_common_root(
                    input_spanningtree=spanningtree, input_model=self.robot)
                self.robot.define_submodel(name=self.export_total_submechanisms, start=root,
                                           stop=tree.find_leaves(self.robot, spanningtree),
                                           only_urdf=True)

        if hasattr(self, "additional_urdfs") and not hasattr(self, "export_submodels"):
            setattr(self, "export_submodels", self.additional_urdfs)
        if hasattr(self, "export_submodels"):
            for au in self.export_submodels:
                self.robot.define_submodel(name=au["name"], start=au["start"],
                                           stop=au["stop"] if "stop" in au else None,
                                           only_urdf=au["only_urdf"] if "only_urdf" in au.keys() else None)

        if "smurf" in self.modeltype and hasattr(self, "smurf"):
            print('  Smurfing poses, sensors, links, materials, etc.', flush=True)

            if 'poses' in self.smurf.keys():
                for (cn, config) in self.smurf["poses"].items():
                    pose = ps.Pose(robot=self.robot, name=cn, configuration=config)
                    self.robot.add_pose(pose)
                    print('      Added pose {}'.format(cn), flush=True)

            if 'sensors' in self.smurf.keys():
                multi_sensors = [x for x in dir(ps.sensors) if
                                 not x.startswith("__") and x not in ps.sensors.__IMPORTS__ and
                                 issubclass(getattr(ps.sensors, x), ps.sensors.MultiSensor)]
                single_sensors = [x for x in dir(ps.sensors) if
                                  not x.startswith("__") and x not in ps.sensors.__IMPORTS__ and issubclass(
                                      getattr(ps.sensors, x), ps.sensors.Sensor) and x not in multi_sensors]
                moveable_joints = [j for j in self.robot.joints if j.type != 'fixed']

                for s in self.smurf["sensors"]:
                    sensor_ = None
                    if s["type"] in single_sensors:
                        kwargs = {k: v for k, v in s.items() if k != "type"}
                        sensor_ = getattr(ps.sensors, s["type"])(robot=self.robot, **kwargs)

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
                        sensor_ = getattr(ps.sensors, s["type"])(robot=self.robot, **kwargs)

                    if sensor_ is not None:
                        self.robot.attach_sensor(sensor_)
                        print('      Attached {} {}'.format(s["type"], s['name']), flush=True)

            if 'links' in self.smurf.keys():
                for link in self.robot.links:
                    name = link.name if link.name in self.smurf["links"].keys() else "default"
                    if name in self.smurf["links"].keys():
                        self.robot.smurf_links += [ps.Link(
                            robot=self.robot,
                            link=link.name,
                            noDataPackage=self.smurf["links"][name]["noDataPackage"]
                            if "noDataPackage" in self.smurf["links"][name].keys() else False,
                            reducedDataPackage=self.smurf["links"][name]["reducedDataPackage"]
                            if "reducedDataPackage" in self.smurf["links"][name].keys() else False
                        )]
                        print('      Defined Link {}'.format(link.name), flush=True)

            if "materials" in self.smurf.keys():
                for m in self.smurf["materials"]:
                    self.robot.smurf_materials += [ps.Material(**m)]
                    print('      Defined Material {}'.format(m["name"]), flush=True)

            # motors
            self.robot.motors = []
            for joint in self.robot.joints:
                conf = self.smurf["motors"]["default"] if "motors" in self.smurf.keys() and "default" in self.smurf[
                    "motors"].keys() else {}
                if "name" in conf.keys():  # we dont ẃant that someone overwrites the same name for all motors
                    conf.pop("name")
                if "motors" in self.smurf.keys() and joint.name in self.smurf["motors"].keys():
                    for k, v in self.smurf["motors"][joint.name].items():
                        conf[k] = v
                if joint.type == "fixed":
                    continue
                if hasattr(joint, "mimic") and joint.mimic is not None:
                    motor = ps.MimicMotor(
                        joint=joint,
                        name=conf["name"] if "name" in conf.keys() else joint.name,  # + "_motor",
                        robot=self.robot,
                        p=conf["p"] if "p" in conf.keys() else 20.0,
                        i=conf["i"] if "i" in conf.keys() else 0.0,
                        d=conf["d"] if "d" in conf.keys() else 0.1,
                        mimic_motor=joint.mimic.joint,  # + "_motor",
                        mimic_multiplier=joint.mimic.multiplier,
                        mimic_offset=joint.mimic.offset,
                        maxEffort=joint.limit.effort if joint.limit is not None and joint.limit.effort > 0.0 else 400,
                        reducedDataPackage=conf["reducedDataPackage"] if "reducedDataPackage" in conf.keys() else False,
                        noDataPackage=conf["noDataPackage"] if "noDataPackage" in conf.keys() else False,
                    )
                    self.robot.attach_motor(motor=motor)
                else:
                    motor = ps.Motor(
                        joint=joint,
                        name=conf["name"] if "name" in conf.keys() else joint.name,  # + "_motor",
                        robot=self.robot,
                        p=conf["p"] if "p" in conf.keys() else 20.0,
                        i=conf["i"] if "i" in conf.keys() else 0.0,
                        d=conf["d"] if "d" in conf.keys() else 0.1,
                        reducedDataPackage=conf["reducedDataPackage"] if "reducedDataPackage" in conf.keys() else False,
                        noDataPackage=conf["noDataPackage"] if "noDataPackage" in conf.keys() else False,
                    )
                    self.robot.attach_motor(motor=motor)

            if "further_annotations" in self.smurf.keys():
                for k, v in self.smurf["further_annotations"]:
                    if k in self.robot.annotations.keys():
                        self.robot.annotations[k] += v
                    else:
                        self.robot.annotations[k] = v

            if "named_annotations" in self.smurf.keys():
                for k, v in self.smurf["named_annotations"].items():
                    self.robot.add_named_annotation(k, v)

        print('Finished processing', flush=True)

        return True

    def export(self):
        self.robot.enforce_zero()
        if "smurf" in self.modeltype:
            self.robot.export_smurf(
                self.exportdir, create_pdf=self.typedef["export_pdf"],
                ros_pkg=self.typedef["ros_package"],
                ros_pkg_name=self.ros_pkg_name,
                export_with_ros_pathes=self.export_ros_pathes if hasattr(self, "export_ros_pathes") else None
            )
        else:
            misc.create_dir(self.pipeline, os.path.dirname(self.exporturdf))
            self.robot.full_export(self.exportdir, create_pdf=self.typedef["export_pdf"],
                                   ros_pkg=self.typedef["ros_package"],
                                   export_with_ros_pathes=self.export_ros_pathes
                                   if hasattr(self, "export_ros_pathes") else None,
                                   ros_pkg_name=self.ros_pkg_name
                                   if self.ros_pkg_name is not None or (
                                               hasattr(self, "export_ros_pathes") and self.export_ros_pathes)
                                   else None)

        if hasattr(self, "export_joint_limits"):
            def limits_file_export(file_path, output_dict):
                print("Exporting joint_limits file", os.path.join(self.exportdir, file_path), flush=True)
                if not os.path.isdir(os.path.dirname(os.path.join(self.exportdir, file_path))):
                    os.makedirs(os.path.dirname(os.path.join(self.exportdir, file_path)))
                with open(os.path.join(self.exportdir, file_path), "w") as jl_file:
                    jl_file.write("limits:\n")
                    jl_file.write("  names: " + yaml.safe_dump(output_dict["names"], default_flow_style=True) + "\n")
                    jl_file.write("  elements: " + yaml.safe_dump(output_dict["elements"], default_flow_style=True))

            def get_joints(robot, joint_desc):
                robot_joint_names = [jnt.name for jnt in robot.joints]
                if type(joint_desc) is list:
                    joints = joint_desc
                elif hasattr(self, "submechanisms_file") and type(joint_desc) is str and joint_desc.upper() != "ALL":
                    if joint_desc.upper() == "ABSTRACT" or joint_desc.upper() == "INDEPENDENT":
                        joints = []
                        for sm in self.submechanisms_file["submechanisms"]:
                            joints += sm["jointnames_independent"]
                        joints = [joint for joint in joints if joint in robot_joint_names]
                    elif joint_desc.upper() == "ACTIVE" or joint_desc.upper() == "ACTUATED":
                        joints = []
                        for sm in self.submechanisms_file["submechanisms"]:
                            joints += sm["jointnames_active"]
                        joints = [joint for joint in joints if joint in robot_joint_names]
                    else:
                        raise Exception(joint_desc + " is no valid joint descriptor!")
                elif joint_desc.upper() == "ALL" or (
                        not hasattr(self, "submechanisms_file") and joint_desc in ["ABSTRACT", "INDEPENDENT", "ACTIVE",
                                                                                   "ACTUATED"]):
                    joints = robot_joint_names
                else:
                    raise Exception(joint_desc + " is no valid joint descriptor!")
                return list(set(joints))

            for file in self.export_joint_limits:
                if file["type"].upper() == "URDF":
                    temp_model = ps.Smurf(name="temp", xmlfile=os.path.join(self.exportdir, file["in"]))
                    if "joints" in file.keys():
                        out = urdf.get_joint_info_dict(temp_model, get_joints(temp_model, file["joints"]))
                    else:
                        out = urdf.get_joint_info_dict(temp_model, get_joints(temp_model, "ALL"))
                elif file["type"].upper() == "JOINTS":
                    out = urdf.get_joint_info_dict(self.robot, get_joints(self.robot, file["joints"]))
                else:
                    raise Exception("Invalid export joint limits type:" + file["type"])
                limits_file_export(file["out"], out)

        if hasattr(self, "export_kccd") and self.export_kccd is not False:
            if not type(self.export_kccd) is dict:
                self.export_kccd = {
                    "join_before_convexhull": True,
                    "keep_stls": False,
                    "keep_urdf": False,
                    "dirname": "kccd",
                    "reduce_meshes": 0
                }
            print("Creating kccd model with config:\n", yaml.safe_dump(self.export_kccd, default_flow_style=False),
                  flush=True)
            misc.create_symlink(self.pipeline,
                                os.path.join(self.pipeline.temp_dir, self.pipeline.meshes["iv"]),
                                os.path.join(self.exportdir, self.pipeline.meshes["iv"]))
            self.robot.export_kccd(self.exportdir, self.pipeline.meshes["iv"], self.typedef["output_mesh_format"],
                                   edit_collisions=self.edit_collisions, **self.export_kccd)

        if hasattr(self, "export_floatingbase") and self.export_floatingbase is True:
            print("Creating floatingbase model...", flush=True)
            self.robot.export_floatingbase(
                self.exportdir,
                ros_pkg_name=self.ros_pkg_name
                if hasattr(self, "export_ros_pathes") and self.export_ros_pathes else None,
                export_with_ros_pathes=self.export_ros_pathes if hasattr(self, "export_ros_pathes") else False,
                create_pdf=self.typedef["export_pdf"]
            )

        if hasattr(self, "keep_files"):
            git.reset(self.targetdir, "autobuild", "master")
            misc.store_persisting_files(self.pipeline, self.targetdir, self.keep_files, self.exportdir)

        print('Finished export of the new model', flush=True)
        self.processed_model_exists = True

        if hasattr(self, "post_processing"):
            for script in self.post_processing:
                if "cwd" not in script.keys():
                    script["cwd"] = self.exportdir
                else:
                    script["cwd"] = os.path.abspath(script["cwd"])
                misc.execute_shell_command(script["cmd"], script["cwd"])
            print('Finished post_processing of the new model', flush=True)

        if self.ros_pkg_name is not None:
            # ROS CMakeLists.txt
            ros_cmake = os.path.join(self.exportdir, "CMakeLists.txt")
            directories = [p for p in os.listdir(self.exportdir) if os.path.isdir(os.path.join(self.exportdir, p))]
            for d in directories:
                directories += [os.path.join(d, p) for p in os.listdir(os.path.join(self.exportdir, d)) if
                                os.path.isdir(os.path.join(self.exportdir, d, p))]
            if hasattr(self, "submodules"):
                for subm in self.submodules:
                    directories += [subm["target"]]
            if hasattr(self, "keep_files"):
                for k in self.keep_files:
                    if k.endswith("/"):
                        directories += [k]
                    elif "/" in k:
                        directories += [os.path.dirname(k)]
            if not os.path.isfile(ros_cmake):
                misc.copy(self.pipeline, pkg_resources.resource_filename("phobos", "data/ROSCMakeLists.txt.in"),
                          ros_cmake)
                with open(ros_cmake, "r") as cmake:
                    content = cmake.read()
                    content = misc.regex_replace(content, {
                        "@PACKAGE_NAME@": self.ros_pkg_name,
                        "@DIRECTORIES@": " ".join(directories),
                    })
                with open(ros_cmake, "w") as cmake:
                    cmake.write(content)
            # ROS package.xml
            packagexml_path = os.path.join(self.exportdir, "package.xml")
            if not os.path.isfile(packagexml_path):
                misc.copy(self.pipeline, pkg_resources.resource_filename("phobos", "data/ROSpackage.xml.in"),
                          packagexml_path)
                with open(packagexml_path, "r") as packagexml:
                    content = packagexml.read()
                    url = self.pipeline.remote_base + "/" + self.modelname
                    content = misc.regex_replace(content, {
                        "\$INPUTNAME": self.ros_pkg_name,
                        "\$AUTHOR": "<author>" + os.path.join(self.pipeline.configdir,
                                                              self.modelname + ".yml") + "</author>",
                        "\$MAINTAINER": "<maintainer>https://git.hb.dfki.de/phobos/ci-run/-/wikis/home</maintainer>",
                        "\$URL": "<url>" + url + "</url>",
                        "\$VERSION": "def:" + self.pipeline.git_rev[10],
                    })
                with open(packagexml_path, "w") as packagexml:
                    packagexml.write(content)

        # REVIEW are the following lines here obsolete?
        if hasattr(self, "keep_files"):
            git.reset(self.targetdir, "autobuild", "master")
            misc.store_persisting_files(self.pipeline, self.targetdir, self.keep_files, self.exportdir)

    def deploy(self, mesh_commit, failed_model=False, uses_lfs=False):
        if hasattr(self, "do_not_deploy") and self.do_not_deploy is True:
            return "deployment suppressed in cfg"
        if not os.path.exists(self.targetdir):
            raise Exception("The result directory " + self.targetdir + " doesn't exist:\n" +
                            "  This might happen if you haven't added the result" +
                            " repo to the manifest.xml or spelled it wrong.")
        repo = self.targetdir
        git.reset(self.targetdir, "autobuild", "master")
        if hasattr(self, "keep_files"):
            misc.store_persisting_files(self.pipeline, repo, self.keep_files, os.path.join(self.tempdir, "sustain"))
        git.update(repo, update_target_branch="$CI_UPDATE_TARGET_BRANCH" if failed_model else "master")
        git.clear_repo(repo)
        # add manifest.xml if necessary
        manifest_path = os.path.join(repo, "manifest.xml")
        if not os.path.isfile(manifest_path):
            misc.copy(self.pipeline, pkg_resources.resource_filename("phobos", "data/manifest.xml.in"), manifest_path)
            with open(manifest_path, "r") as manifest:
                content = manifest.read()
                url = self.pipeline.remote_base + "/" + self.modelname
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
            misc.copy(self.pipeline, pkg_resources.resource_filename("phobos", "data/README.md.in"), readme_path)
            with open(readme_path, "r") as readme:
                content = readme.read()
                content = misc.regex_replace(content, {
                    "\$MODELNAME": self.modelname,
                    "\$USERS": self.pipeline.mr_mention if not hasattr(self, "mr_mention") else self.mr_mention,
                    "\$DEFINITION_FILE": "https://git.hb.dfki.de/" + os.path.join(
                        os.environ["CI_ROOT_PATH"],
                        os.environ["CI_MODEL_DEFINITIONS_PATH"],
                        self.modelname + ".yml").replace("models/robots", "models-robots"),
                })
            with open(readme_path, "w") as readme:
                readme.write(content)
        if uses_lfs:
            readme_content = open(readme_path, "r").read()
            if "# Git LFS for mesh repositories" not in readme_content:
                with open(readme_path, "a") as readme:
                    readme.write(open(pkg_resources.resource_filename("phobos", "data/GitLFS_README.md.in")).read())
        # update additional submodules
        if hasattr(self, "submodules"):
            for subm in self.submodules:
                git.add_submodule(
                    repo,
                    subm["repo"],
                    subm["target"],
                    commit=subm["commit"] if "commit" in subm.keys() else None,
                    branch=subm["branch"] if "branch" in subm.keys() else "master"
                )
        # update the mesh submodule
        git.add_submodule(
            repo,
            os.path.relpath(
                os.path.join(self.pipeline.root, self.typedef["meshespath"]), repo
            ),
            self.typedef["meshespath"],
            commit=mesh_commit[self.typedef["output_mesh_format"]]["commit"],
            branch=os.environ[
                "CI_MESH_UPDATE_TARGET_BRANCH"] if "CI_MESH_UPDATE_TARGET_BRANCH" in os.environ.keys() else "master"
        )
        if "also_export_bobj" in self.typedef.keys() and self.typedef["also_export_bobj"]:
            git.add_submodule(
                repo,
                os.path.relpath(
                    os.path.join(self.pipeline.root, self.pipeline.meshes["bobj"]), repo
                ),
                self.pipeline.meshes["bobj"],
                commit=mesh_commit["bobj"]["commit"],
                branch=os.environ[
                    "CI_MESH_UPDATE_TARGET_BRANCH"] if "CI_MESH_UPDATE_TARGET_BRANCH" in os.environ.keys() else "master"
            )
        if hasattr(self, "export_kccd") and self.export_kccd is not False:
            git.add_submodule(
                repo,
                os.path.relpath(
                    os.path.join(self.pipeline.root, self.pipeline.meshes["iv"]), repo
                ),
                self.pipeline.meshes["iv"],
                commit=mesh_commit["iv"]["commit"],
                branch=os.environ[
                    "CI_MESH_UPDATE_TARGET_BRANCH"] if "CI_MESH_UPDATE_TARGET_BRANCH" in os.environ.keys() else "master"
            )
        if "export_bobj" in self.typedef.keys() and self.typedef["also_export_bobj"] is False:
            git.add_submodule(
                repo,
                os.path.relpath(
                    os.path.join(self.pipeline.root, self.pipeline.meshes["iv"]), repo
                ),
                self.pipeline.meshes["iv"],
                commit=mesh_commit["iv"]["commit"],
                branch=os.environ[
                    "CI_MESH_UPDATE_TARGET_BRANCH"] if "CI_MESH_UPDATE_TARGET_BRANCH" in os.environ.keys() else "master"
            )
        # now we move back to the push repo
        misc.copy(self.pipeline, self.exportdir + "/*", self.targetdir)
        if hasattr(self, "keep_files"):
            misc.restore_persisting_files(self.pipeline, repo, self.keep_files, os.path.join(self.tempdir, "sustain"))
        git.commit(repo, origin_repo=os.path.abspath(self.pipeline.configdir))
        git.add_remote(repo, self.pipeline.remote_base + "/" + self.modelname)
        mr = git.MergeRequest()
        mr.target = self.pipeline.mr_target_branch if not hasattr(self, "mr_target_branch") else self.mr_target_branch
        mr.title = self.pipeline.mr_title if not hasattr(self, "mr_title") else self.mr_title
        mr.description = self.pipeline.mr_description
        if os.path.isfile(self.pipeline.test_protocol):
            print("INFO: Appending test_protocol to MR description")
            with open(self.pipeline.test_protocol, "r") as f:
                protocol = yaml.safe_load(f.read())
                mr.description = misc.append_string(mr.description, "\n" + str(protocol["all"]))
                mr.description = misc.append_string(mr.description, str(protocol[self.modelname]))
        else:
            print("INFO: Did not find test_protocol file at:", self.pipeline.test_protocol, flush=True, file=sys.stderr)
        if failed_model:
            if hasattr(self.pipeline, "mr_mention") or hasattr(self, "mr_mention"):
                mr.mention = self.pipeline.mr_mention if not hasattr(self, "mr_mention") else self.mr_mention
            return_msg = "pushed to " + git.push(repo, merge_request=mr)
        else:
            return_msg = "pushed to " + git.push(repo, branch="master")
            # deploy to mirror
        if hasattr(self, "deploy_to_mirror"):
            print("Deploying to mirror:\n", yaml.safe_dump(self.deploy_to_mirror, default_flow_style=False), flush=True,
                  file=sys.stdout)
            print("Deploying to mirror", flush=True, file=sys.stderr)
            mirror_dir = os.path.join(self.tempdir, "deploy_mirror")
            git.clone(self.pipeline, self.deploy_to_mirror["repo"], mirror_dir, branch="master", shallow=False)
            git.update(mirror_dir, update_remote="origin", update_target_branch=self.deploy_to_mirror["branch"])
            git.clear_repo(mirror_dir)
            submodule_dict = {}
            if "submodules" in self.deploy_to_mirror.keys() and ".gitmodules" in os.listdir(repo):
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
                if self.deploy_to_mirror["submodules"]:
                    for _, sm in submodule_dict.items():
                        git.add_submodule(mirror_dir, sm["url"], sm["path"],
                                          branch=sm["branch"] if "branch" in sm.keys() else "master")
            misc.copy(self.pipeline, repo + "/*", mirror_dir)
            if "submodules" in self.deploy_to_mirror.keys() and ".gitmodules" in os.listdir(repo):
                for _, sm in submodule_dict.items():
                    # git.clone(self.pipeline, os.path.join(self.deploy_to_mirror["repo"], sm["url"]), sm["path"],
                    #           sm["branch"] if "branch" in sm.keys() else "master", cwd=mirror_dir)
                    misc.execute_shell_command("rm -rf " + os.path.join(sm["path"], ".git*"), mirror_dir)
            git.commit(mirror_dir, origin_repo=self.pipeline.configdir)
            if "merge_request" in self.deploy_to_mirror.keys() and self.deploy_to_mirror["merge_request"]:
                if "mr_mention" in self.deploy_to_mirror.keys():
                    mr.mention = self.deploy_to_mirror["mr_mention"]
                if "mr_title" in self.deploy_to_mirror.keys():
                    mr.title = self.deploy_to_mirror["mr_title"]
                if "mr_target" in self.deploy_to_mirror.keys():
                    mr.target = self.deploy_to_mirror["mr_target"]
                git.push(mirror_dir, remote="origin", merge_request=mr, branch=self.deploy_to_mirror["branch"])
            else:
                git.push(mirror_dir, remote="origin", branch=self.deploy_to_mirror["branch"])
            return_msg += "& pushed to mirror"
        git.checkout("master", repo)
        return return_msg

    def get_imported_meshes(self):
        return self._meshes
