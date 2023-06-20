import os
from copy import deepcopy

import numpy as np
import pkg_resources
import pydot
import traceback

from .. import geometry as pgu, utils
from ..commandline_logging import get_logger
from ..defs import load_json, dump_json, KINEMATIC_TYPES
from ..geometry import get_reflection_matrix
from ..io import representation, sensor_representations
from ..io.hyrodyn import Submechanism
from ..io.poses import JointPoseSet
from ..io.smurfrobot import SMURFRobot
from ..utils import transform, misc, git, resources
from ..utils.misc import read_number_from_config, regex_replace, create_dir, edit_name_string, execute_shell_command
from ..utils.transform import create_transformation, inv, get_adjoint, round_array
from ..utils.tree import find_close_ancestor_links, get_joints
from ..utils.xml import transform_object, get_joint_info_dict

log = get_logger(__name__)


class Robot(SMURFRobot):
    def __init__(self, name=None, xmlfile=None, submechanisms_file=None, smurffile=None, verify_meshes_on_import=True,
                 inputfile=None, description=None, is_human=False, autogenerate_submechanisms=None,
                 assert_validity=True):
        """ The basic robot class to represent a urdf.
        """
        try:
            super().__init__(xmlfile=xmlfile, submechanisms_file=submechanisms_file, smurffile=smurffile,
                             verify_meshes_on_import=verify_meshes_on_import, inputfile=inputfile, description=description,
                             autogenerate_submechanisms=autogenerate_submechanisms, is_human=is_human)
        except Exception as e:
            log.error(f"Failed loading:\n  input: {inputfile}\n  xml: {xmlfile}\n  submechanims: {submechanisms_file}\n  smurf: {smurffile}\n"
                      f"because of:\n"+''.join(traceback.format_exception(None, e, e.__traceback__)))
            raise e

        if name is not None:
            self.name = name
        self.submodel_defs = {}
        if assert_validity and self.links:
            self.assert_validity()

    # export methods
    def export_meshes(self, mesh_output_dir, format=None, use_existing=False, apply_scale=False):
        """
        Will go through all visuals and collisions and export the meshes of all mesh geometries to in the given format to the outputdir
        Args:
            mesh_output_dir: The directory where to put the meshes
            format: a mesh format as in phobos.defs.MESH_TYPES

        Returns:
            None
        """
        for vc in self.visuals + self.collisions:
            if isinstance(vc.geometry, representation.Mesh):
                vc.geometry.provide_mesh_file(targetpath=os.path.abspath(mesh_output_dir), format=format,
                                              use_existing=use_existing, apply_scale=apply_scale)

    def to_x3d_string(self, float_fmt_dict=None, reduce_meshes=0):
        export_instance = self.duplicate()
        for vis in export_instance.visuals:
            if isinstance(vis.geometry, representation.Mesh):
                vis.geometry.apply_scale()

        if reduce_meshes > 0:
            for vis in export_instance.visuals:
                if isinstance(vis.geometry, representation.Mesh):
                    n_vertices = len(vis.geometry.x3d_vertices)/3
                    if n_vertices > reduce_meshes:
                        vis.geometry.reduce_mesh(reduce_meshes/n_vertices)

        return super(Robot, export_instance).to_x3d_string(float_fmt_dict=float_fmt_dict)

    def export_x3d(self, outputfile, float_fmt_dict=None, reduce_meshes=1000):  #, ros_pkg=False, copy_with_other_pathes=None, ros_pkg_name=None, mesh_format=None):
        """Export the mechanism to the given output file.
        If export_visuals is set to True, all visuals will be exported. Otherwise no visuals get exported.
        If export_collisions is set to to True, all collisions will be exported. Otherwise no collision get exported.
        """
        if float_fmt_dict is None:
            float_fmt_dict = {}

        outputfile = os.path.abspath(outputfile)

        xml_string = '<?xml version="1.0" encoding="UTF-8"?>\n'+ \
                     '<!DOCTYPE X3D PUBLIC "ISO//Web3D//DTD X3D 3.3//EN" "http://www.web3d.org/specifications/x3d-3.3.dtd">\n'+ \
                     "<X3D profile='Interchange' version='3.3' xmlns:xsd='http://www.w3.org/2001/XMLSchema-instance' xsd:noNamespaceSchemaLocation='http://www.web3d.org/specifications/x3d-3.3.xsd'>"+ \
                     '<head><!-- All "meta" from this section you will found in <Scene> node as MetadataString nodes. --></head>\n'+ \
                     '<Scene>\n'+self.to_x3d_string(float_fmt_dict=float_fmt_dict, reduce_meshes=reduce_meshes)+'</Scene>\n</X3D>\n'

        if not os.path.exists(os.path.dirname(os.path.abspath(outputfile))):
            os.makedirs(os.path.dirname(os.path.abspath(outputfile)))
        with open(outputfile, "w") as f:
            f.write(xml_string)
            f.close()

        log.info("X3D written to {}".format(outputfile))
        return

    def export_urdf(self, outputfile, float_fmt_dict=None, ros_pkg=False, copy_with_other_pathes=False, ros_pkg_name=None, mesh_format=None):
        """Export the mechanism to the given output file.
        If export_visuals is set to True, all visuals will be exported. Otherwise no visuals get exported.
        If export_collisions is set to to True, all collisions will be exported. Otherwise no collision get exported.
        """
        if any([j.joint_type in j.ADVANCED_TYPES for j in self.joints]):
            raise AssertionError(f"Can't export joints with a type in {representation.Joint.ADVANCED_TYPES} to a URDF")

        if float_fmt_dict is None:
            float_fmt_dict = {}
        self.joints = self.get_joints_ordered_df()

        outputfile = os.path.abspath(outputfile)

        export_robot = self.duplicate()
        # [ToDo v2.0.0] When converting an SDF to an URDF the relative_to information of the Poses will get lost.
        # Therefore we need to check whether the origin have the by URDF expected frames and transform them when necessary

        if mesh_format is not None:
            export_robot.mesh_format = mesh_format
        export_robot.xmlfile = outputfile

        xml_string = export_robot.to_urdf_string(float_fmt_dict=float_fmt_dict)

        if ros_pkg is True:
            xml_string = regex_replace(xml_string, {'filename="../': 'filename="package://' if ros_pkg_name is None else f'filename="package://{ros_pkg_name}/'})

        if not os.path.exists(os.path.dirname(os.path.abspath(outputfile))):
            os.makedirs(os.path.dirname(os.path.abspath(outputfile)))
        with open(outputfile, "w") as f:
            f.write(xml_string)
            f.close()

        if copy_with_other_pathes and not ros_pkg:
            xml_string = regex_replace(xml_string, {'filename="../': 'filename="package://'})
            f = open(outputfile[:-5] + "_ros.urdf", "w")
            f.write(xml_string)
            f.close()
        elif copy_with_other_pathes and ros_pkg:
            xml_string = regex_replace(xml_string, {'filename="package://': 'filename="../'})
            f = open(outputfile[:-5] + "_relpath.urdf", "w")
            f.write(xml_string)
            f.close()

        log.info("URDF written to {}".format(outputfile))
        return

    def export_sdf(self, outputfile, float_fmt_dict=None, ros_pkg=False, copy_with_other_pathes=None, ros_pkg_name=None, mesh_format=None):
        """Export the mechanism to the given output file.
        If export_visuals is set to True, all visuals will be exported. Otherwise no visuals get exported.
        If export_collisions is set to to True, all collisions will be exported. Otherwise no collision get exported.
        """
        # [Todo v2.1.0] Create the model.config file for gazebo
        if float_fmt_dict is None:
            float_fmt_dict = {}
        self.joints = self.get_joints_ordered_df()

        outputfile = os.path.abspath(outputfile)

        export_robot = self.duplicate()
        if mesh_format is not None:
            export_robot.mesh_format = mesh_format
        export_robot.xmlfile = outputfile

        xml_string = '<sdf version="1.9">\n'+export_robot.to_sdf_string(float_fmt_dict=float_fmt_dict)+"\n</sdf>"

        if ros_pkg is True:
            xml_string = regex_replace(xml_string, {'<uri>../': '<uri>package://' if ros_pkg_name is None else f'<uri>package://{ros_pkg_name}/'})

        if not os.path.exists(os.path.dirname(os.path.abspath(outputfile))):
            os.makedirs(os.path.dirname(os.path.abspath(outputfile)))
        with open(outputfile, "w") as f:
            f.write(xml_string)
            f.close()

        if copy_with_other_pathes and not ros_pkg:
            xml_string = regex_replace(xml_string, {'<uri>../': '<uri>package://'})
            f = open(outputfile[:-4] + "_ros.sdf", "w")
            f.write(xml_string)
            f.close()
        elif copy_with_other_pathes and ros_pkg:
            xml_string = regex_replace(xml_string, {'<uri>package://': '<uri>../'})
            f = open(outputfile[:-4] + "_relpath.sdf", "w")
            f.write(xml_string)
            f.close()

        log.info("SDF written to {}".format(outputfile))
        return

    def export_xml(self, outputdir=None, format="urdf", filename=None, float_fmt_dict=None, no_format_dir=False,
                   ros_pkg=False, copy_with_other_pathes=None, ros_pkg_name=None,
                   with_meshes=True, use_existing_meshes=False, mesh_format=None, additional_meshes=None, rel_mesh_pathes=None,
                   enforce_zero=False, correct_inertials=False):
        """ Exports all model information stored inside this instance.
        """
        outputdir = os.path.abspath(outputdir)
        if rel_mesh_pathes is None:
            rel_mesh_pathes = resources.get_default_rel_mesh_pathes()
        assert self.get_root()
        format = format.lower()
        assert format in KINEMATIC_TYPES, format

        export_robot = self.duplicate()

        # Main model
        if no_format_dir:
            model_file = os.path.join(outputdir, f"{self.name if filename is None else filename}")
        else:
            model_file = os.path.join(outputdir, f"{format}/{self.name if filename is None else filename}")
        if not model_file.lower().endswith(format):
            model_file += "." + format
        if not os.path.exists(os.path.dirname(os.path.abspath(model_file))):
            os.makedirs(os.path.dirname(os.path.abspath(model_file)))
        if ros_pkg_name is None:
            ros_pkg_name = os.path.basename(outputdir)
        export_robot.link_entities()
        export_robot.xmlfile = model_file

        # meshes
        if with_meshes and not use_existing_meshes:
            _mesh_format = mesh_format
            if _mesh_format is None:
                _mesh_format = self.mesh_format
            assert _mesh_format is not None
            if additional_meshes is None:
                additional_meshes = []
            meshes = [_mesh_format] + additional_meshes
            for mf in [f.lower() for f in meshes]:
                export_robot.export_meshes(mesh_output_dir=os.path.join(outputdir, rel_mesh_pathes[mf]), format=mf)
        # xml
        _export_robot = self.duplicate()
        if use_existing_meshes:
            _export_robot.export_meshes(mesh_output_dir=os.path.join(outputdir, rel_mesh_pathes[mesh_format]), format=mesh_format, use_existing=True)
        if enforce_zero:
            _export_robot.enforce_zero()
        if correct_inertials:
            _export_robot.correct_inertials()
        assert len(self.links) == len(self.joints) + 1
        if format == "urdf":
            _export_robot.export_urdf(
                outputfile=model_file,
                ros_pkg=ros_pkg, copy_with_other_pathes=copy_with_other_pathes, ros_pkg_name=ros_pkg_name,
                float_fmt_dict=float_fmt_dict, mesh_format=mesh_format
            )
        elif format == "sdf":
            _export_robot.export_sdf(
                outputfile=model_file,
                ros_pkg=ros_pkg, copy_with_other_pathes=copy_with_other_pathes, ros_pkg_name=ros_pkg_name,
                float_fmt_dict=float_fmt_dict, mesh_format=mesh_format
            )
        else:
            raise IOError("Unknown export format:" + format)

        return model_file

    def export_smurf(self, outputdir=None, outputfile=None, robotfile=None,
                     check_submechs=True, with_submodel_defs=False,
                     with_meshes=True, mesh_format=None, additional_meshes=None, rel_mesh_pathes=None):
        """ Export self and all annotations inside a given folder with structure
        """
        assert self.check_linkage()
        # Convert to absolute path
        if outputfile is not None:
            assert outputdir is None
            outputdir = os.path.dirname(outputfile)
            if outputdir.lower().endswith("smurf"):
                outputdir = os.path.dirname(outputdir)
            else:
                raise AssertionError("Smurf files have to be placed in a smurf subfolder")
        outputdir = os.path.abspath(outputdir)
        if not os.path.exists(outputdir):
            os.mkdir(outputdir)

        submech_dir = os.path.join(outputdir, "submechanisms")
        if len(self.submechanisms) > 0 or len(self.exoskeletons) > 0:
            if not os.path.exists(os.path.abspath(submech_dir)):
                os.makedirs(os.path.abspath(submech_dir))
        self.link_entities()
        # meshes
        if with_meshes:
            _mesh_format = mesh_format
            if _mesh_format is None:
                _mesh_format = self.mesh_format
            assert _mesh_format is not None
            if additional_meshes is None:
                additional_meshes = []
            meshes = [_mesh_format] + additional_meshes
            for mf in [f.lower() for f in meshes]:
                self.export_meshes(mesh_output_dir=os.path.join(outputdir, rel_mesh_pathes[mf]), format=mf)
        # Export the smurf files
        smurf_dir = os.path.join(outputdir, "smurf")
        self.smurffile = os.path.join(smurf_dir, "{}.smurf".format(self.name))
        if not os.path.exists(smurf_dir):
            os.mkdir(smurf_dir)
        # Export attr
        export_files = [os.path.relpath(robotfile, outputdir + "/smurf")] if robotfile is not None else []
        submechanisms = {}
        if self.autogenerate_submechanisms is None or self.autogenerate_submechanisms is True:
            self.generate_submechanisms()
        if check_submechs and (self.submechanisms is not None and len(self.submechanisms)) > 0 or (self.exoskeletons is not None and len(self.exoskeletons)):
            missing_joints = [(j, self.get_joint(j).joint_type) for j in self._get_joints_not_included_in_submechanisms()]
            if len(missing_joints) != 0:
                log.warning(f"{self.smurffile}: Not all joints defined in the submechanisms definition! "+("(only fixed joints)" if all([j[1] == "fixed" for j in missing_joints]) else ""))
                log.debug(f"Lacking definitions for:\n{missing_joints}")
            double_joints = self._get_joints_included_twice_in_submechanisms()
            if len(double_joints) != 0:
                log.error({dj: [sm.to_yaml() for sm in self.submechanisms if dj in sm.get_joints()] for dj in double_joints})
                raise AssertionError(f"The following joints are multiply defined in the submechanisms definition: \n{double_joints}")
        for sm in self.submechanisms + self.exoskeletons:
            if hasattr(sm, "file_path"):
                _submodel = self.instantiate_submodel(
                    name=str(sm), start=sm.get_root(self), stop=sm.get_leaves(self, include_dependent=True), robotname=str(sm),
                    no_submechanisms=True, include_unstopped_branches=False
                )
                sm.file_path = f"../submechanisms/{str(sm)}.urdf"
                if not os.path.isfile(sm.file_path):
                    _submodel.export_urdf(outputfile=os.path.normpath(os.path.join(outputdir, "submechanisms", f"{str(sm)}.urdf")))
                    _submodel.export_joint_limits(
                        outputdir=os.path.normpath(os.path.join(outputdir, "submechanisms")),
                        file_name=f"joint_limits_{str(sm)}.yml"
                    )
                else:
                    log.warning(f"File {sm.file_path} does already exist. Not exported submechanism urdf.")
        for annotation in self.smurf_annotation_keys:
            # Check if exists and not empty
            if hasattr(self, annotation) and getattr(self, annotation):
                annotation_dict = {annotation: []}
                # Collect all
                for item in getattr(self, annotation):
                    annotation_dict[annotation].append(item.to_yaml())
                # Export to file
                annotation_name = annotation
                if annotation == "submechanisms" or annotation == "exoskeletons":
                    submechanisms[annotation] = annotation_dict[annotation]
                else:
                    with open(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, annotation_name)), "w+") as stream:
                        stream.write(dump_json(annotation_dict, default_style=False))
                        export_files.append(os.path.split(stream.name)[-1])
        if submechanisms != {}:
            self.submechanisms_file = os.path.join(smurf_dir, "{}_submechanisms.yml".format(self.name))
            with open(self.submechanisms_file, "w+") as stream:
                stream.write(dump_json(submechanisms, default_style=False))
                export_files.append(os.path.split(stream.name)[-1])

        # further annotations
        for k, v in self.annotations.items():
            if k not in self.smurf_annotation_keys:
                with open(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k)), "w+") as stream:
                    stream.write(dump_json({k: v}, default_style=False))
                    export_files.append(os.path.split(stream.name)[-1])

        # submodel list
        if with_submodel_defs and len(self.submodel_defs) > 0:
            out_submodel_defs = {}
            for name, definition in self.submodel_defs.items():
                out_submodel_defs[name] = deepcopy(definition)
                out_submodel_defs[name]["export_dir"] = os.path.relpath(out_submodel_defs[name]["export_dir"], smurf_dir)
            with open(os.path.join(smurf_dir, "{}_submodels.yml".format(self.name)), "w+") as stream:
                stream.write(dump_json({"submodels": out_submodel_defs}, default_style=False))
                export_files.append(os.path.split(stream.name)[-1])

        # generic annotations
        temp_generic_annotations = {}
        for ga in self.categorized_annotations:
            if ga.GA_category not in temp_generic_annotations:
                temp_generic_annotations[ga.GA_category] = []
            temp_generic_annotations[ga.GA_category].append({ga.GA_name: ga.to_yaml()} if ga.GA_name is not None else ga.to_yaml())
        # clean-up the temporary lists
        for k, v in temp_generic_annotations.items():
            if len(v) == 1:
                temp_generic_annotations[k] = v[0]
            elif len(v) > 1 and all(type(x) == dict and len(x.keys()) == 1 for x in v):
                for sub_dict in v:
                    temp_generic_annotations[k].update(sub_dict)
        for k, v in temp_generic_annotations.items():
            # deal with existing names
            if os.path.isfile(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k))):
                k = "generic_annotation_"+k
            new_k = k
            i = 0
            while os.path.isfile(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, new_k))):
                i += 1
                new_k = f"{k}_{i}"
            k = new_k
            # write
            if len(v) > 0 and k not in self.smurf_annotation_keys:
                with open(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k)), "w") as stream:
                    stream.write(dump_json({k: v}, default_style=False))
                    export_files.append(os.path.split(stream.name)[-1])

        # Create the smurf file itsself
        annotation_dict = {
            'modelname': self.name,
            # 'date': datetime.datetime.now().strftime("%Y%m%d_%H:%M"),
            'files': sorted(export_files),
            'description': self.description
        }
        if self.version is not None:
            annotation_dict['version'] = self.version

        with open(self.smurffile, "w+") as stream:
            stream.write(dump_json(annotation_dict, default_style=False, sort_keys=True))
        log.info(f"SMURF written to {smurf_dir}")

    def export_joint_limits(self, outputdir, file_name="joint_limits.yml", joint_desc=None):
        output_dict = get_joint_info_dict(self, get_joints(self, joint_desc))
        log.info(f"Exporting joint_limits file {os.path.join(outputdir, file_name)}")
        if not os.path.isdir(os.path.dirname(os.path.abspath(os.path.join(outputdir, file_name)))):
            os.makedirs(os.path.dirname(os.path.abspath(os.path.join(outputdir, file_name))))
        output_dict = {"limits": output_dict}
        with open(os.path.join(outputdir, file_name), "w") as jl_file:
            jl_file.write(dump_json(output_dict))

    def export_kccd(self, outputdir, rel_iv_meshes_path, output_mesh_format, join_before_convexhull=True,
                    keep_stls=True, keep_urdf=True, dirname="kccd",
                    reduce_meshes=0, edit_collisions=None, **kwargs):
        if edit_collisions is None:
            edit_collisions = {}
        kccd_meshes = os.path.join(outputdir, rel_iv_meshes_path)
        kccd_path = os.path.join(outputdir, dirname)
        kccd_urdf = os.path.join(kccd_path, self.name + ".urdf")
        create_dir(None, kccd_path)
        kccd_robot = self.duplicate()
        kccd_robot.xmlfile = kccd_urdf
        if "remove_joints" in kwargs.keys():
            for j in kwargs["remove_joints"]:
                # print("Removing joint for kccd:", j)
                kccd_robot.remove_joint(j)
                assert str(j) not in [str(jnt) for jnt in kccd_robot.joints]
        # generate collision model
        pgu.generate_kccd_optimizer_ready_collision(
            kccd_robot, [link.name for link in kccd_robot.links],
            outputdir=kccd_meshes,
            join_first=join_before_convexhull,
            merge_additionally=kwargs["merge_additionally"] if "merge_additionally" in kwargs.keys() else None,
            mars_meshes=output_mesh_format.lower() == "mars_obj",
            reduce_meshes=reduce_meshes
        )

        # urdf2kccd generates the model out of the visuals therefore we have to remove all visuals and make the
        # collisions visuals
        for link in kccd_robot.links:
            pgu.remove_visual(kccd_robot, link.name)
            assert (link.visuals is None or len(link.visuals) == 0)
            if len(link.collisions) == 0 or link.collisions is None:
                continue
            assert (len(link.collisions) == 1)
            link.add_aggregate("visual", representation.Visual(
                name=link.collisions[0].name,
                origin=link.collisions[0].origin,
                geometry=link.collisions[0].geometry,
            ))
        kccd_robot.export_urdf(outputfile=kccd_urdf)
        execute_shell_command("urdf2kccd -b " + self.name + ".urdf", cwd=kccd_path)
        kccd_kinematics_file = open(kccd_urdf[:-5] + "Kinematics.cfg", "r").read().split("\n\n")
        kccd_kinematics = {}
        for block in kccd_kinematics_file:
            if len(block.strip()) == 0:
                continue
            first_line = block.split("\n")[0]
            if first_line.startswith("FRAME"):
                first_line = first_line.split(" ")
                kccd_kinematics[first_line[1][2:]] = {
                    "type": "FRAME",
                    "name": block[1],
                    "moveswith": block[3]
                }
            else:
                first_line = first_line.split(" AXIS ")
                first_line = first_line[0].split(" ") + first_line[1][:first_line[1].rfind("]") + 1].split("LIMITS") \
                             + first_line[1][first_line[1].rfind("]") + 1:].strip().split(" ")
                if "JOINT" in first_line[0]:
                    kccd_kinematics[first_line[1]] = {
                        "type": first_line[0],
                        "name": first_line[1],
                        "frame": first_line[2],
                        "axis": load_json(first_line[3]),
                        "limits": load_json(first_line[4]),
                        "jointIdx": int(first_line[5]),
                        "order": int(first_line[6]),
                    }
        # now update the kccd.cfg
        kccd_cfg = open(kccd_urdf[:-5] + ".cfg", "r").read().split("\n\n")
        new_kccd_cfg = []
        kccd_dict = {}
        dontchecks = []
        for block in kccd_cfg:
            if "TITLE" in block:
                if "safety_distance" in kwargs.keys():
                    block += "\nSAFETYDISTANCE " + str(kwargs["safety_distance"])
                if "report_up_to" in kwargs.keys():
                    block += "\nREPORTUPTO " + str(kwargs["report_up_to"])
                if "computation_budget" in kwargs.keys():
                    block += "\nCOMPUTATIONBUDGET " + str(kwargs["computation_budget"])
                if "max_approximation_order" in kwargs.keys():
                    block += "\nMAXAPPROXIMATIONORDER " + str(kwargs["max_approximation_order"])
                block += "\n"
            elif "# BRAKINGMODEL" in block and "braking_model" in kwargs.keys():
                block = block.replace("# BRAKINGMODEL", "BRAKINGMODEL")
                block = block.replace("<linearFactor>", str(kwargs["braking_model"]["linearFactor"]))
                block = block.replace("<invDeceleration>", str(kwargs["braking_model"]["invDeceleration"]))
                block = block.replace("UNCERTAINTY <uncertainty>",
                                      "UNCERTAINTY " + str(kwargs["braking_model"]["uncertainty"]))
                block = block.replace("LATENCY <latency>",
                                      "LATENCY " + str(kwargs["braking_model"]["latency"]))
            elif "# OVERWRITE" in block and "simplify_swept_from" in kwargs.keys() and len(
                    kwargs["simplify_swept_from"]) > 0:
                for link in kwargs["simplify_swept_from"]:
                    assert kccd_robot.get_link(link) is not None
                    subtree = kccd_robot.instantiate_submodel(name="kccd_subtree", start=link, stop=None)
                    order0 = [joint.name for joint in subtree.joints if joint.name in kccd_kinematics.keys()]
                    if len(order0) > 0:
                        block += "\nOVERWRITE " + " ".join(order0) + " WITH ORDER 0 END"
            elif block.startswith("BODY "):
                entry = block.split()
                temp = {"body": entry[1],
                        "link": entry[1][2:],
                        "joint": entry[3],
                        "mesh": entry[8]}
                coll_name = os.path.basename(temp["mesh"][:temp["mesh"].rfind("_")])
                if coll_name in kccd_dict.keys():
                    log.info("{temp['mesh']} ->  {coll_name} already in kccd_dict keys!")
                    raise AssertionError
                if "DONTCHECK" in block:
                    dontchecks += [block[block.find("DONTCHECK") + 9:block.find("END")].strip().split(" WITH ")]
                    block = block.replace(block[block.find("DONTCHECK"):block.find("END") + 3], "")
                # generate cover volume
                distance = len(
                    kccd_robot.get_chain(self.get_root(), temp["link"], links=False, joints=True, fixed=False))
                n_points = int((25 - 3) * 0.4 ** (distance / 3) + 3)  # [TODO v2.1.0] elaborate this
                if temp["link"] in edit_collisions.keys():
                    if "n_points" in edit_collisions[temp["link"]].keys():
                        n_points = edit_collisions[temp["link"]]["n_points"]
                    elif "shape" in edit_collisions[temp["link"]].keys() and \
                            edit_collisions[temp["link"]]["shape"].lower() != "convex":
                        if edit_collisions[temp["link"]]["shape"].lower() == "cylinder":
                            n_points = 2
                        elif edit_collisions[temp["link"]]["shape"].lower() == "box":
                            n_points = 8
                        elif edit_collisions[temp["link"]]["shape"].lower() == "sphere":
                            n_points = 1
                log.info(f"Covering {temp['mesh']} of {temp['link']} with {n_points}")
                out, _ = execute_shell_command(
                    "kccdcoveriv " + temp["mesh"] + " " + str(n_points) + " " + temp["body"], cwd=kccd_path,
                    silent=True)
                block_lines = block.split("\n")
                new_block = []
                for line in block_lines:
                    if not (line.startswith("BODYRADIUS") or line.startswith("BODYPOINT")):
                        new_block += [line]
                new_block = "\n".join(new_block).strip()
                new_block += "\n" + out[out.find("# Fitted volume"):].strip()
                block = new_block
                kccd_dict[coll_name] = deepcopy(temp)
            new_kccd_cfg += [block.strip() + "\n\n"]
        no_coll_override = {} if "no_collision_between" not in kwargs.keys() else kwargs[
            "no_collision_between"]
        for dc in dontchecks:
            if dc[0] not in no_coll_override.keys():
                no_coll_override[dc[0]] = [dc[1]]
            else:
                no_coll_override[dc[0]] += [dc[1]]
        coll_names, coll_matrix = kccd_robot.generate_collision_matrix(no_coll_override=no_coll_override)
        for i in range(len(coll_names)):
            for j in range(i):
                if coll_matrix[i, j] == 0:
                    new_line = "DONTCHECK " + kccd_dict[coll_names[i]]["body"] \
                               + " WITH " + kccd_dict[coll_names[j]]["body"] + " END\n"
                    new_kccd_cfg += [new_line]
        with open(kccd_urdf[:-5] + ".cfg", "w") as f:
            f.write("\n")
            for line in new_kccd_cfg:
                f.write(line.replace("../../", "../"))

        # remove the stls and urdf
        if not keep_stls:
            os.system("rm -rf {}".format(os.path.join(kccd_meshes, "*.stl")))
        if not keep_urdf:
            os.system("rm -rf {}".format(kccd_urdf))

    def export_pdf(self, outputfile):
        SUBMECH_COLORS = ["cyan", "darkslateblue", "steelblue", "indigo", "darkblue", "royalblue", "lightskyblue",
                          "teal", "blue", "dodgerblue", "paleturquoise", "lightcyan", "mediumslateblue"]
        EXOSKEL_COLORS = ["lawngreen", "green", "darkgreen", "seagreen", "lightseagreen", "mediumspringgreen",
                          "palegreen", "olive"]

        def add_joint(joint):
            _out = f"\"{joint.parent}\" -> \"{joint.name}\" [label="
            _out += f"\"xyz: {joint.origin.xyz[0]} {joint.origin.xyz[1]} {joint.origin.xyz[2]} "
            _out += f"\\nrpy: {joint.origin.rpy[0]} {joint.origin.rpy[1]} {joint.origin.rpy[2]} "
            # _out += f"\\nindex: {self.get_joints_ordered_df().index(joint)} "
            if joint.axis is not None:
                _out += f"\\naxis: {joint.axis[0]} {joint.axis[1]} {joint.axis[2]} "
            _out += f"\\ntype: {joint.joint_type} "
            if len(joint.joint_dependencies) > 0:
                _out += f"\\ndepends on: "
                for jd in joint.joint_dependencies:
                    _out += f"\\n- {jd.joint} factor: {jd.multiplier} offset: {jd.offset}"
            if joint.motor is not None:
                _out += f"\\nmotor: {str(joint.motor)}"
            _out += f"\"] \"{joint.name}\" -> \"{joint.child}\"\n"
            return _out

        out = "digraph G {\n"
        out += "esep=10;\n"
        out += "sep=10;\n"
        out += "nodesep=0.5;\n"
        out += "node [shape=box];\n"
        for link in self.get_links_ordered_df():
            out += f"\"{str(link)}\" [label=\"{str(link)}\"];\n"

        printed_joints = []
        if hasattr(self, "submechanisms") and self.submechanisms is not None and len(self.submechanisms) > 0:
            for i, sm in enumerate(self.submechanisms):
                # [TODO later] pretty setting of the submechanism legend
                out += f"node [shape=box, color={SUBMECH_COLORS[i%len(SUBMECH_COLORS)]}, fontcolor=black];\n"
                out += f"\"submech_{str(sm)}\" [label="
                out += f"\"Submechanism\\ntype: {sm.type} "
                out += f"\\nname: {sm.name} "
                out += f"\\ncontextual_name: {sm.contextual_name} "
                out += "\"];\n"
            for i, sm in enumerate(self.submechanisms):
                out += f"node [shape=ellipse, color={SUBMECH_COLORS[i%len(SUBMECH_COLORS)]}, fontcolor=black];\n"
                for joint in sorted(sm.get_joints()):
                    joint = self.get_joint(joint)
                    assert str(joint) not in printed_joints, str(joint)+" "+str(printed_joints)
                    printed_joints.append(str(joint))
                    out += add_joint(joint)
                    ## includes the submechanisms deeper by drawing an error to them
                    # out += f"\"{joint.name}\" -- \"submech_{str(sm)}\" [color=gray]\n"
        if hasattr(self, "exoskeletons") and self.exoskeletons is not None and len(self.exoskeletons) > 0:
            for i, exo in enumerate(self.exoskeletons):
                out += f"node [shape=septagon, color={EXOSKEL_COLORS[i%len(SUBMECH_COLORS)]}, fontcolor=black];\n"
                for joint in sorted(exo.get_joints()):
                    joint = self.get_joint(joint)
                    if joint.is_human:
                        printed_joints.append(str(joint))
                        out += add_joint(joint)
        if len(printed_joints) < len(self.joints):
            out += f"node [shape=ellipse, color=orange, fontcolor=black];\n"
            for joint in self.get_joints_ordered_df():
                if str(joint) not in printed_joints:
                    out += add_joint(self.get_joint(joint))

        out += "}\n"

        if not os.path.isdir(os.path.dirname(os.path.abspath(outputfile))):
            os.makedirs(os.path.dirname(os.path.abspath(outputfile)), exist_ok=True)
        with open(outputfile + ".gv", "w") as f:
            f.write(out)

        graph = pydot.graph_from_dot_data(out)
        graph[0].write_pdf(outputfile)

    def export(self, outputdir, export_config, rel_mesh_pathes=None, ros_pkg_name=None, no_smurf=False,
               ros_pkg_later=False, check_submechs=True, with_meshes=True, use_existing_meshes=False, apply_scale=False):
        assert self.check_linkage()
        outputdir = os.path.abspath(outputdir)
        if rel_mesh_pathes is None:
            rel_mesh_pathes = resources.get_default_rel_mesh_pathes()
        xml_file_in_smurf = None
        ros_pkg = False
        if ros_pkg_name is None:
            ros_pkg_name = os.path.basename(outputdir)
        # submechanism generation if necessary
        if self.autogenerate_submechanisms is None or self.autogenerate_submechanisms is True:
            self.generate_submechanisms()
        # export meshes
        if with_meshes and not use_existing_meshes:
            mesh_formats = set()
            for ex in export_config:
                mesh_formats = mesh_formats.union([f.lower() for f in ex.get("additional_meshes", [])])
                if "mesh_format" in ex:
                    mesh_formats.add(ex["mesh_format"].lower())
            for mf in mesh_formats:
                self.export_meshes(mesh_output_dir=os.path.join(outputdir, rel_mesh_pathes[mf]), format=mf, apply_scale=apply_scale)
        # export everything else
        for export in export_config:
            if export["type"] in KINEMATIC_TYPES:
                if export.get("link_in_smurf", False):
                    export_robot_instance = self.duplicate()
                else:
                    export_robot_instance = self
                xml_file = export_robot_instance.export_xml(
                    outputdir=outputdir,
                    format=export["type"],
                    ros_pkg=export["ros_pathes"] if "ros_pathes" in export else None,
                    copy_with_other_pathes=export["copy_with_other_pathes"] if "copy_with_other_pathes" in export else None,
                    ros_pkg_name=ros_pkg_name,
                    float_fmt_dict=export.get("float_fmt_dict", None),
                    filename=export["filename"] if "filename" in export else None,
                    with_meshes=False, # this has already been done above
                    mesh_format=export["mesh_format"],
                    additional_meshes=export["additional_meshes"] if "additional_meshes" in export else None,
                    rel_mesh_pathes=rel_mesh_pathes,
                    enforce_zero=export.get("enforce_zero", False),
                    correct_inertials=export.get("correct_inertials", False),
                    use_existing_meshes=use_existing_meshes
                )
                ros_pkg |= export["ros_pathes"] if "ros_pathes" in export else None
                if export.get("link_in_smurf", False):
                    assert xml_file_in_smurf is None, "Only one xml file can be linked in the SMURF"
                    xml_file_in_smurf = xml_file
            elif export["type"] == "submodel":
                log.debug(f"Exporting submodel {export['name']}")
                if export["name"] not in self.submodel_defs:
                    export_robot_instance = self.define_submodel(
                        name=export["name"],
                        start=export.get("start", None),
                        stop=export.get("stop", None),
                        include_unstopped_branches=export.get("include_unstopped_branches", None),
                        no_submechanisms=export.get("no_submechanisms", False),
                        abstract_model=export.get("abstract_model", False),
                        include_human_in_abstract=export.get("include_human_in_abstract", False),
                        only_urdf=export.get("only_urdf", False),
                        remove_joints=export.get("remove_joints", None),
                        move_joint_axis_to_intersection=export.get("move_joint_axis_to_intersection", None)
                    )
                else:
                    assert export.get("start", None) == self.submodel_defs[export["name"]]["start"]
                    assert export.get("stop", None) == self.submodel_defs[export["name"]]["stop"]
                    assert export.get("include_unstopped_branches", False) == self.submodel_defs[export["name"]]["include_unstopped_branches"]
                    assert export.get("no_submechanisms", False) == self.submodel_defs[export["name"]]["no_submechanisms"]
                    export_robot_instance = self.instantiate_submodel(**export)
                if "add_floating_base" in export and export["add_floating_base"]:
                    export_robot_instance = export_robot_instance.add_floating_base()
                _export_config = None
                if "export_config" in export:
                    _export_config = export["export_config"]
                else:
                    _export_config = [ec for ec in export_config if ec["type"] != "submodel"]
                self.submodel_defs[export["name"]]["export_dir"] = os.path.join(outputdir, "submodels", export["name"])
                export_robot_instance.export(
                    outputdir=self.submodel_defs[export["name"]]["export_dir"],
                    export_config=_export_config,
                    rel_mesh_pathes={k: os.path.join("..", "..", v) for k, v in rel_mesh_pathes.items()},
                    with_meshes=False,
                    use_existing_meshes=use_existing_meshes,
                    no_smurf=no_smurf
                )
            elif export["type"] == "pdf":
                self.export_pdf(outputfile=os.path.join(outputdir, self.name + ".pdf"))
            elif export["type"] == "kccd":
                export_robot_instance = self.duplicate()
                export_robot_instance.export_kccd(
                    outputdir=outputdir,
                    rel_iv_meshes_path=rel_mesh_pathes["iv"],
                    output_mesh_format="stl",
                    **export
                )
            elif export["type"] == "joint_limits":
                kwargs = {}
                if "file_name" in export:
                    kwargs["file_name"] = export["file_name"]
                if "joints" in export:
                    kwargs["joint_desc"] = export["joints"]
                self.export_joint_limits(
                    outputdir=outputdir,
                    **kwargs
                )
            elif export["type"] == "smurf":
                # will be exported by default
                pass
            else:
                log.error(f"Can't export according to following export configuration:\n{export}")
        # export smurf
        if not no_smurf:
            self.export_smurf(
                outputdir=outputdir,
                robotfile=xml_file_in_smurf,
                check_submechs=check_submechs,
                with_submodel_defs=True,
                with_meshes=False  # has been done before
            )
        # export ros package files
        if ros_pkg and not ros_pkg_later:
            self.export_ros_package_files(outputdir, ros_pkg_name)
        elif ros_pkg and ros_pkg_later:
            return ros_pkg_name

    def export_ros_package_files(self, outputdir, ros_pkg_name, author=None, maintainer=None, url=None, version=None):
        # ROS CMakeLists.txt
        ros_cmake = os.path.join(outputdir, "CMakeLists.txt")
        directories = [os.path.relpath(dir, outputdir) for dir, _, _ in os.walk(outputdir)]
        if not os.path.isfile(ros_cmake):
            misc.copy(None, pkg_resources.resource_filename("phobos", "data/ROSCMakeLists.txt.in"),
                      ros_cmake)
            with open(ros_cmake, "r") as cmake:
                content = cmake.read()
                content = misc.regex_replace(content, {
                    "@PACKAGE_NAME@": ros_pkg_name,
                    "@DIRECTORIES@": " ".join(directories),
                })
            with open(ros_cmake, "w") as cmake:
                cmake.write(content)
        # ROS package.xml
        packagexml_path = os.path.join(outputdir, "package.xml")
        if not os.path.isfile(packagexml_path):
            misc.copy(None, pkg_resources.resource_filename("phobos", "data/ROSpackage.xml.in"),
                      packagexml_path)
            with open(packagexml_path, "r") as packagexml:
                content = packagexml.read()
                if any([x is None for x in [author, maintainer, url]]):
                    _author, _maintainer, _url = git.get_repo_data(outputdir)
                content = misc.regex_replace(content, {
                    "\$INPUTNAME": ros_pkg_name,
                    "\$AUTHOR": f"<author>{author if author is not None else _author}</author>",
                    "\$MAINTAINER": f"<maintainer>{maintainer if maintainer is not None else _maintainer}</maintainer>",
                    "\$URL": f"<url>{url if url is not None else _url}</url>",
                    "\$VERSION": f"def: {version if version is not None else self.version}"
                })
            with open(packagexml_path, "w") as packagexml:
                packagexml.write(content)

    # getters
    def get_submodel(self, name):
        """ Return the submodel with the given name.
        """
        if name in self.submodel_defs.keys():
            return self.submodel_defs[name]
        else:
            log.warning("No submodel named {}".format(name))
        return

    # tools
    def exchange_root(self, new_root):
        """
        This function adapts all joints in this robot in that way, that the given link becomes the new root.
        Args:
            link_name: The name of the link that shall become the new root

        Returns:
            None, this method works in place
        """
        if str(self.get_root()) == str(new_root):
            return
        if type(new_root) == str:
            new_root = self.get_link(new_root)
        flip_joints = self.get_chain(self.get_root(), new_root, links=False)
        old_robot = self.duplicate()
        or2x = old_robot.get_transformation
        nr2x = lambda x: old_robot.get_transformation(x, start=str(new_root))
        # 0. make sure all link parts follow the convention that they have to be relative_to that link
        for link in self.links:
            for vc in self.visuals + self.collisions:
                if str(vc.origin.relative_to) != str(link):
                    vc.origin = representation.Pose.from_matrix(
                        inv(or2x(link)).dot(or2x(vc.origin.relative_to).dot(vc.origin.to_matrix())),
                        relative_to=link
                    )
                    vc.origin.link_with_robot(self)
                if link.inertial is not None:
                    link.inertial.origin = representation.Pose.from_matrix(
                        inv(or2x(link)).dot(or2x(vc.origin.relative_to).dot(link.inertial.origin.to_matrix())),
                        relative_to=link
                    )
                    link.inertial.origin.link_with_robot(self)
        # 1. get the tree structure correct
        for jointname in flip_joints:
            joint = self.get_joint(jointname)
            _temp = joint.parent
            joint.parent = joint.child
            joint.child = _temp
        # 2. go through all joints and links and set the origins according to new root
        for joint in self.joints:
            joint.origin = representation.Pose.from_matrix(
                nr2x(joint.name),
                relative_to=str(new_root)
            )
            joint.origin.link_with_robot(self)
        for link in self.links:
            link.origin = representation.Pose.from_matrix(
                nr2x(link.name),
                relative_to=str(new_root)
            )
            link.origin.link_with_robot(self)
        new_root.origin = None
        # 3. regenerate tree
        self.regenerate_tree_maps()
        # 4. make all origins follow the convention
        for jl in self.links + self.joints:
            if jl.origin is None:
                continue
            jl.origin = jl.joint_relative_origin
            jl.origin.link_with_robot(self)

    def remove_visuals(self):
        """
        Removes all visuals from this robot.
        :return: None
        """
        for link in self.links:
            for vis in link.visuals:
                link.remove_aggregate(vis)

    def remove_collisions(self):
        """
        Removes all visuals from this robot.
        :return:
        """
        for link in self.links:
            for coll in link.collisions:
                link.remove_aggregate(coll)

    # not used
    # def reparent_link(self, link_name, parent, inertia=True, visual=True, collision=True):
    #     """
    #     Reparent all xml-children ( inertia, visual and collision ) of the given link onto the new parent.
    #     :param link_name: the link we apply this to
    #     :param parent: the new parent
    #     :param inertia: whether we do this for the inertias
    #     :param visual: whether we do this for the visuals
    #     :param collision: whether we do this for the collisions
    #     :return: None
    #     """
    #     if isinstance(link_name, list):
    #         if isinstance(parent, list):
    #             assert len(link_name) == len(parent)
    #             for link_, parent_ in zip(link_name, parent):
    #                 self.reparent_link(link_, parent_, inertia=inertia, visual=visual, collision=collision)
    #             return
    #         for link_ in link_name:
    #             self.reparent_link(link_, parent, inertia=inertia, visual=visual, collision=collision)
    #         return
    #
    #     link = self.get_link(link_name)
    #     parent = self.get_link(parent)
    #
    #     if not link or not parent:
    #         log.warning("Link or new parent not found!")
    #         return
    #
    #     # Get the transformation
    #     # root to link
    #     L_T_R = self.get_transformation(link.name)
    #     R_T_P = inv(self.get_transformation(parent.name))
    #
    #     L_T_P = R_T_P.dot(L_T_R)
    #
    #     if inertia and link.inertial:
    #         inertia_L = link.inertial
    #         if parent.inertial:
    #             # Merge the inertials
    #             # Old one
    #             I_L = inertia_L.to_mass_matrix()
    #             IP_T_IL = parent.inertial.origin.to_matrix().dot(
    #                 L_T_P.dot(inertia_L.origin.to_matrix()))
    #             Ad = get_adjoint(IP_T_IL)
    #             # Transform into parent
    #             I_NL = Ad.dot(I_L.dot(Ad.T)) + parent.inertial.to_mass_matrix()
    #             parent.inertial = representation.Inertial.from_mass_matrix(I_NL, parent.inertial.origin)
    #
    #         else:
    #             # Set inertial to new parent
    #             new_origin = L_T_P.dot(inertia_L.origin.to_matrix())
    #             parent.inertial = link.inertial
    #             parent.inertial.origin = representation.Pose.from_matrix(new_origin)
    #
    #         # Set link to near zero
    #         link.inertial = representation.Inertial.from_mass_matrix(1e-5 * np.ones((6, 6)), link.inertial.origin)
    #
    #     if visual and link.visuals:
    #         for vis in link.visuals:
    #             VL_T_L = vis.origin.to_matrix()
    #             new_origin = L_T_P.dot(VL_T_L)
    #             vis.origin = representation.Pose.from_matrix(new_origin)
    #             parent.add_aggregate('visual', vis.duplicate())
    #             link.remove_aggregate(vis)
    #
    #     if collision and link.collisions:
    #         for col in link.collisions:
    #             CL_T_L = col.origin.to_matrix()
    #             new_origin = L_T_P.dot(CL_T_L)
    #             col.origin = representation.Pose.from_matrix(new_origin)
    #             parent.add_aggregate('collision', col.duplicate())
    #             link.remove_aggregate(col)
    #
    #     # Reinit the link
    #     # link.to_xml()
    #     # parent.to_xml()
    #     self.relink_entities()
    #     return

    def move_link_in_tree(self, link_name, new_parent_name):
        """
        Moves the given link to a new parent
        :param link_name: the link to move
        :param new_parent_name: the link where to attach to
        :return: None
        """
        if isinstance(link_name, list):
            for ln in link_name:
                self.move_link_in_tree(ln, new_parent_name)

        jointname = self.get_parent(link_name)
        if jointname is None:
            raise AssertionError("Can't move the root link.")
        joint = self.get_joint(jointname)

        T0_old = self.get_transformation(link_name)
        T0_newp = self.get_transformation(new_parent_name)
        joint.origin = representation.Pose.from_matrix(inv(T0_newp).dot(T0_old), relative_to=new_parent_name)
        joint.origin.link_with_robot(self)
        joint.parent = new_parent_name

    def define_submodel(self, name, start=None, stop=None, robotname=None, only_urdf=False, abstract_model=False,
                        remove_joints=None, no_submechanisms=False, include_unstopped_branches=False,
                        move_joint_axis_to_intersection=None, include_human_in_abstract=False,
                        overwrite=False):
        """Defines a submodel from a given starting link.
        If stop is provided than the chain from start to stop is used.
        """
        if remove_joints is None:
            remove_joints = []
        assert stop is None or type(stop) == list
        definition = {
            "name": name,
            "robotname": robotname,
            "start": str(start) if start is not None else None,
            "stop": [str(s) for s in stop] if stop is not None else None,
            "only_urdf": only_urdf,
            "abstract_model": abstract_model,
            "remove_joints": remove_joints,
            "move_joint_axis_to_intersection": move_joint_axis_to_intersection,
            "include_unstopped_branches": include_unstopped_branches,
            "no_submechanisms": no_submechanisms,
            "include_human_in_abstract": include_human_in_abstract
        }
        if name in self.submodel_defs.keys() and not overwrite:
            raise NameError("A submodel with the given name is already defined")
        else:
            self.submodel_defs[name] = definition
        return self.instantiate_submodel(**definition)

    def get_links_and_joints_in_subtree(self, start, stop=None, include_unstopped_branches=False):
        assert self.get_link(start, verbose=True) is not None, f"Link {start} does not exist"
        linknames = set()
        _stop = list(set(stop)) if stop is not None else None
        if include_unstopped_branches or stop is None:
            _stop = self.get_leaves(start, stop=_stop)
        for leave in _stop:
            linknames = linknames.union(self.get_chain(start, leave, joints=False))
        linknames = list(linknames)

        jointnames = [str(j) for j in self.get_joint(self.get_parent(linknames)) if j is not None and j.parent in linknames]
        assert len(linknames) == len(jointnames) + 1, f"n_links={len(linknames)} - 1 != n_joints={len(jointnames)}\n{start}\t{stop}\n{linknames}\n{jointnames}"
        #print(f"n_links={len(linknames)} - 1 != n_joints={len(jointnames)}\n{start}\t{stop}\n{linknames}\n{jointnames}")

        return linknames, jointnames

    def instantiate_submodel(self, name=None, start=None, stop=None, robotname=None, include_unstopped_branches=None,
                             no_submechanisms=False, abstract_model=False, remove_joints=None,
                             move_joint_axis_to_intersection=None, include_human_in_abstract=False,
                             **ignored_kwargs):
        """
        Instantiates a submodel by it's definition. Takes either name or definition. If both are given, the submodel
        definition with the given name will be updated including renaming it
        See define_submodel for the params. Call by robot.instantiate_submodel(**robot.submodel_defs[name])
        :return: the submodel
        """
        if remove_joints is None:
            remove_joints = []
        if start is None:
            start = self.get_root()

        if robotname is None and name is not None:
            robotname = name
        # elif name is None and robotname is not None:  # name is not used here otherwise
        #     name = robotname
        elif name is None and robotname is None:
            raise ValueError("Neither name nor robotname given")

        submodel = type(self)(name=robotname, assert_validity=False)

        if stop is None:
            include_unstopped_branches = True

        linknames, jointnames = self.get_links_and_joints_in_subtree(
            start=start, stop=stop, include_unstopped_branches=include_unstopped_branches
        )

        # [ToDo] In the following we have to go through all origins and check whether their relative_to is in the submodel as well and if not they have to be transformed
        links = self.get_link(linknames)
        materials = set()
        for link in links:
            for visual in link.visuals:
                if visual.material is not None:
                    _mat = self.get_material(visual.material)
                    materials.add(_mat)
        submodel.add_aggregate("materials", self.get_aggregate("material", list(materials)))
        submodel.add_aggregate("links", [ln.duplicate() for ln in links])

        _joints = self.get_joint(jointnames)
        # remove mimic relation if the mimiced joint is not in here
        joints = []
        for joint in _joints:
            if any([jd.joint not in jointnames for jd in joint.joint_dependencies]):
                _joint = joint.duplicate()
                log.debug(f"Removing mimic relation in submodel {robotname} for {_joint.name} (mimiced "
                          f"{_joint.mimic.joint})!")
                _joint.joint_dependencies = [jd for jd in _joint.joint_dependencies if jd.joint in jointnames]
                joints.append(_joint)
            else:
                joints.append(joint.duplicate())

        submodel.add_aggregate("joints", joints)
        assert all([j.mimic is None or str(j.mimic.joint) in jointnames for j in submodel.joints])

        motors = []
        for joint in _joints:
            if joint.motor is not None:
                motors.append(joint._motor.duplicate() if type(joint._motor) != str else self.get_motor(joint._motor))
        submodel.add_aggregate("motor", motors)

        sensors = []
        for sensor in self.sensors:
            if sensor.is_related_to(links + joints):
                if isinstance(sensor, sensor_representations.MultiSensor):
                    _sensor = sensor.duplicate()
                    _sensor.reduce_to_match(links + joints)
                    sensors.append(_sensor)
                else:
                    sensors.append(sensor.duplicate())
        submodel.add_aggregate("sensors", sensors)
        submechanisms = []
        exoskeletons = []
        if not no_submechanisms:
            for subm in self.submechanisms:
                subm.unlink_from_robot()
                _subm = deepcopy(subm)
                subm.link_with_robot(self)
                _subm._jointnames = None
                assert _subm.jointnames is None
                if _subm.is_related_to(joints, pure=True):
                    submechanisms.append(_subm)
            for exo in self.exoskeletons:
                exo.unlink_from_robot()
                _exo = deepcopy(exo)
                exo.link_with_robot(self)
                _exo.reduce_to_match(joints)
                _exo._jointnames = None
                if not _exo.is_empty():
                    exoskeletons.append(_exo)
        interfaces = []
        for interf in self.interfaces:
            if interf.is_related_to(joints):
                interfaces.append(interf.duplicate())
        if not no_submechanisms:
            submodel.add_aggregate("submechanisms", submechanisms)
            submodel.add_aggregate("exoskeletons", exoskeletons)
        submodel.add_aggregate("interfaces", interfaces)

        # copy all annotations we do not have yet
        for k, v in self.__dict__.items():
            if k not in submodel.__dict__.keys() or submodel.__dict__[k] is None:
                submodel.__dict__[k] = v

        submodel.get_root().origin = None
        submodel.link_entities()

        if abstract_model and len(self.submechanisms) > 0:
            abstract_joints = [str(j) for j in self.joints
                               if j.joint_type == "fixed"]
            for sm in self.submechanisms:
                abstract_joints += [str(j) for j in sm.jointnames_independent]
            if not include_human_in_abstract:
                for sm in self.exoskeletons:
                    abstract_joints += [str(j) for j in sm.jointnames_dependent]

            remove_joints += [str(j) for j in self.joints
                              if str(j) not in abstract_joints]
            submodel.autogenerate_submechanisms = True
            submodel.submechanisms = []
            submodel.exoskeletons = []

        submodel.remove_joint(remove_joints)

        if move_joint_axis_to_intersection is not None:
            for jointname, intersecting in move_joint_axis_to_intersection.items():
                submodel.move_joint_to_intersection(jointname, intersecting)

        submodel.link_entities()

        return submodel

    def remove_submodel(self, name):
        """Remove the submodel with the given name"""
        self.submodel_defs.pop(name)

    def intersection(self, other, name=None, submodel=True, useother=False, keep=None):
        """ Gives the intersection of the spanning tree of two robots. Uses information stored in this robot, if not
        use other.
        Removes all joints and links from the robot, which are not in the union and returns a new robot.

        If submodel is True, the resulting robot is stored as submodel inside the robot.
        """
        if keep is None:
            keep = []
        driver_joints = set([j.name for j in self.joints])
        other_joints = set([j.name for j in other.joints])

        if keep:
            other_joints.update(set([j.name for j in self.joints if j.name in keep]))

        difference = list(driver_joints.difference(other_joints))

        new_robot = type(self)()
        new_robot.__dict__.update(self.__dict__)

        if useother:
            new_robot.__dict__.update(other.__dict__)

        for j in difference:
            new_robot.remove_joint(j)

        if name is not None:
            new_robot.name = name

        if submodel:
            self.submodel_defs.update(
                {new_robot.name: new_robot}
            )
        return new_robot

    # # [TODO later] this method is not fully implemented
    # def difference(self, other, name=None, submodel=True, useother=False, exclude=None):
    #     """
    #     Compute the difference between two models and returns a new robot model with this as the spanning
    #     """
    #     raise NotImplementedError
    #     # if exclude is None:
    #     #     exclude = []
    #     # # Collect all joints
    #     # driver_joints = set([j.name for j in self.joints])
    #     # other_joints = set([j.name for j in other.joints])
    #     #
    #     # if exclude:
    #     #     driver_joints.update(driver_joints.difference(set(exclude)))
    #     #
    #     # # Get the difference
    #     # difference = list(driver_joints.difference(other_joints))
    #     #
    #     # return difference

    def correct_inertials(self, limit=1e-5):
        """
        Correct all inertials of the robot.
        """
        for link in self.links:
            # [TODO v2.1.0] check if the I is basically zero and then recreate the inertial using the collision
            if link.inertial:
                M = self.get_inertial(link.name).to_mass_matrix()
                origin = link.inertial.origin
                if origin.relative_to is None:
                    origin.relative_to = link
            else:
                M = np.zeros((6, 6))
                origin = representation.Pose.from_matrix(np.eye(4), relative_to=link)
                origin.link_with_robot(self)
            m = M[0, 0]
            if m <= limit:
                M[:3, :3] = np.eye(3) * limit
                log.info(" Corrected mass for link {}".format(link.name))

            I = M[3:, 3:]

            if any([x < limit for x in np.linalg.eigvals(I)]):
                E, V = np.linalg.eig(I)
                diff = np.abs(np.min(E) - limit)

                E = np.diag([e + diff for e in E])
                I = np.matmul(V, np.matmul(E, np.linalg.inv(V)))

                M[3:, 3:] = I

            link.inertial = representation.Inertial.from_mass_matrix(M, origin, link)
            log.debug(" Corrected inertia for link {}".format(link.name))

    def compute_mass(self):
        """
        Compute the overall mass of the robot.
        """
        m = 0.0
        for link in self.links:
            m += link.inertial.mass if link.inertial else 0.0
        log.info("{} has a total mass of {} kg.".format(self.name, m))
        return m

    def compute_com(self):
        com = np.array([0.0, 0.0, 0.0])
        mass = 0
        for link in self.links:
            T = self.get_transformation(link.name)
            if link.inertial is not None:
                T = T.dot(link.inertial.origin.to_matrix())
                m = link.inertial.mass
                mass += m
                com += T[0:3, 3] * m
        com /= mass
        return com

    def transform_link_orientation(self, linkname, transformation, only_frame=True, transform_to=False):
        """
        Rotate the given link in such a way, that inertials, visuals and collisions rest at there place.
        Transformation is given in the linkname frame.
        :param linkname: the name of the link we aply this to
        :param transformation: the transformation matrix
        :param only_frame: if true, onyl the frame will be moved all other representations remain at their place
        :param transform_to: if true the transformation will be the new transformation between parent and this
        otherwise the frame/link will be transformed by the given transformation
        :return:
        """
        if self.get_link_id(linkname) is None:
            raise Exception(
                "Provide valid link: '" + linkname + "' is not in " + str([ln.name for ln in self.links]))

        # Get the parent joint
        pjoint = self.get_joint(self.get_parent(linkname))
        # Get the child joint
        cjoint = self.get_joint(self.get_children(linkname))

        T = transformation

        Tinv = inv(T)

        link = self.get_link(linkname)

        if pjoint is not None:
            if transform_to:
                Tinv = inv(inv(pjoint.origin.to_matrix()).dot(T))
                pjoint.origin = representation.Pose.from_matrix(T, relative_to=pjoint.origin.relative_to)
            else:
                pjoint.origin = representation.Pose.from_matrix(pjoint.origin.to_matrix().dot(T), relative_to=pjoint.origin.relative_to)
        if only_frame:
            for joint in cjoint:
                joint.origin = representation.Pose.from_matrix(Tinv.dot(joint.origin.to_matrix()), relative_to=joint.origin.relative_to)

            for ent in link.collisions + link.visuals:
                ent.origin = representation.Pose.from_matrix(Tinv.dot(ent.origin.to_matrix()), relative_to=ent.origin.relative_to)

            self.transform_inertial(linkname, transformation=Tinv)

    def transform_link(self, linkname, translation=None, rotation=None, transformation=None, maintain_children=True):
        """Rotate the given link in such a way, that inertials, visuals and collisions stay the same.
        """
        if self.get_link_id(linkname) is None:
            raise Exception(
                "Provide valid link to attach to. '" + linkname + "' is not in " + str([ln.name for ln in self.links]))

        log.info("Transform {}...".format(linkname))
        # Get the parent joint
        pjoint = self.get_joint(self.get_parent(linkname))

        # Get the child joint
        cjoint = self.get_joint(self.get_children(linkname))
        # Process the rotation
        if transformation is not None:
            T = transformation
        else:
            T = create_transformation(translation, rotation)
        Tinv = inv(T)

        # If we have just one parent and children, we rotate the parent and inverse rotate all children
        log.info(" Transforming Joints")
        if pjoint is not None:
            # Transform the parent
            assert transform_object(pjoint, T)
            if maintain_children:
                # Get all child origin
                assert transform_object(cjoint, Tinv)

            return

        else:
            # Else we have to rotate everything inside the joint (if it is root)
            # Get the inertial, visual and collision
            assert transform_object(cjoint, T)
            assert self.transform_inertial(linkname, translation, rotation)
            assert self.transform_visual(linkname, translation, rotation)
            assert self.transform_collision(linkname, translation, rotation)
            return

    def transform_inertial(self, linkname, translation=None, rotation=None, transformation=None):
        """ Transform the inertial of a link given a translation and rotation.
        """
        inertial = self.get_inertial(linkname)
        link_id = self.get_link_id(linkname)

        if inertial is None:
            return True

        # Process the rotation
        if transformation is not None:
            T = transformation
        else:
            T = create_transformation(xyz=translation, rpy=rotation)
        # Transform the origin
        inertial.origin = inertial.origin.transformed_by(T, relative_to=inertial.origin.relative_to)
        rot_part = create_transformation(rpy=inertial.origin.rpy, xyz=[0, 0, 0])
        inertial.origin.rotation = [0, 0, 0]

        # T = rot_part.dot(T)
        Ad = get_adjoint(rot_part)
        # Tinv = inv(T)
        # AdInv = Adjoint(Tinv)

        # Get the Inertia Tensor
        I = inertial.to_mass_matrix()

        # Rotate the inertial tensor
        I = np.matmul(np.transpose(Ad), np.matmul(I, Ad))

        inertial = representation.Inertial.from_mass_matrix(I, inertial.origin)

        self.links[link_id].inertial = inertial
        return True

    def transform_visual(self, linkname, translation, rotation):
        """Transform the visual(s) of the given link.
        """
        # Get the visual
        visual = self.get_visual_by_link(linkname)
        log.info(" Transform Visuals")
        # Transform
        T = create_transformation(translation, rotation)
        visual.origin = collisvisualvisualon.origin.transformed_by(T, relative_to=visual.object.relative_to)
        return True

    def transform_collision(self, linkname, translation, rotation):
        """Transform the collision(s) of the given link.
        """
        # Get the collision
        collision = self.get_collision_by_link(linkname)
        log.info(" Transform Collisions")
        # Transform
        T = create_transformation(translation, rotation)
        collision.origin = collision.origin.transformed_by(T, relative_to=collision.object.relative_to)
        return True

    def enforce_zero(self, xyz_tolerance=1E-5, rad_tolerance=1E-6, mass_tolerance=1E-4, i_tolerance=1E-12):
        """
        Values belwo the respective tolerances will be rounded to zero.
        :param xyz_tolerance: tolerance for all length values (translation)
        :param rad_tolerance: tolerance for all rad angles (rotation)
        :param mass_tolerance: tolerance for masses
        :param i_tolerance: tolerance for inertia values (ixx, ... izz)
        :return: None
        """
        for link in self.links:
            if link.inertial is not None:
                link.inertial.mass = 0.0 if np.abs(link.inertial.mass) < mass_tolerance else link.inertial.mass
                if link.inertial.inertia is not None:
                    link.inertial.inertia.ixx = 0.0 if np.abs(
                        link.inertial.inertia.ixx) < i_tolerance else link.inertial.inertia.ixx
                    link.inertial.inertia.ixy = 0.0 if np.abs(
                        link.inertial.inertia.ixy) < i_tolerance else link.inertial.inertia.ixy
                    link.inertial.inertia.ixz = 0.0 if np.abs(
                        link.inertial.inertia.ixz) < i_tolerance else link.inertial.inertia.ixz
                    link.inertial.inertia.iyy = 0.0 if np.abs(
                        link.inertial.inertia.iyy) < i_tolerance else link.inertial.inertia.iyy
                    link.inertial.inertia.iyz = 0.0 if np.abs(
                        link.inertial.inertia.iyz) < i_tolerance else link.inertial.inertia.iyz
                    link.inertial.inertia.izz = 0.0 if np.abs(
                        link.inertial.inertia.izz) < i_tolerance else link.inertial.inertia.izz
                for i in range(3):
                    link.inertial.origin.xyz[i] = 0.0 if np.abs(link.inertial.origin.xyz[i]) < xyz_tolerance else \
                        link.inertial.origin.xyz[i]
                    link.inertial.origin.rpy[i] = 0.0 if np.abs(link.inertial.origin.rpy[i]) < rad_tolerance else \
                        link.inertial.origin.rpy[i]
                    for g in link.visuals + link.collisions:
                        g.origin.xyz[i] = 0.0 if np.abs(g.origin.xyz[i]) < xyz_tolerance else g.origin.xyz[i]
                        g.origin.rpy[i] = 0.0 if np.abs(g.origin.rpy[i]) < rad_tolerance else g.origin.rpy[i]
            for joint in self.joints:
                for i in range(3):
                    joint.origin.xyz[i] = 0.0 if np.abs(joint.origin.xyz[i]) < xyz_tolerance else joint.origin.xyz[i]
                    joint.origin.rpy[i] = 0.0 if np.abs(joint.origin.rpy[i]) < rad_tolerance else joint.origin.rpy[i]
                    if joint.axis is not None:
                        joint.axis[i] = 0.0 if np.abs(joint.axis[i]) < xyz_tolerance else joint.axis[i]
                if hasattr(joint, "limit") and joint.limit is not None:
                    joint.limit.lower = 0.0 if joint.limit.lower is not None and np.abs(joint.limit.lower) < rad_tolerance else joint.limit.lower
                    joint.limit.upper = 0.0 if joint.limit.upper is not None and np.abs(joint.limit.upper) < rad_tolerance else joint.limit.upper
                    joint.limit.velocity = 0.0 if joint.limit.velocity is not None and np.abs(joint.limit.velocity) < rad_tolerance else joint.limit.velocity
                    joint.limit.effort = 0.0 if joint.limit.effort is not None and np.abs(joint.limit.effort) < rad_tolerance else joint.limit.effort

    def set_estimated_link_com(self, link, dont_overwrite=True):
        """
        Estimate the links com from its collision convex hull.
        :param link: name of the link for which to estimate the com
        :param dont_overwrite: if set to false the previous inertial origin will be overwritten,
        otherwise it will be only written if the inertial has a xyz=0,0,0 transform
        :return: None
        """
        if type(link) is list:
            for lk in link:
                self.set_estimated_link_com(lk, dont_overwrite=dont_overwrite)
            return
        elif type(link) is str:
            link = self.get_link(link)
        assert (link is not None and type(link) is representation.Link)

        if dont_overwrite and any([v != 0 for v in link.inertial.origin.xyz]):
            log.warning(f"Not overwriting com of link: {link.name}")
            return

        volume = 0.0
        com = np.array([0.0, 0.0, 0.0])
        if len(link.collisions) == 0:
            return
        for coll in link.collisions:
            T = coll.origin.to_matrix()
            T_com = np.identity(4)
            if isinstance(coll.geometry, representation.Mesh):
                vol, T_com[0:3, 3] = coll.geometry.approx_volume_and_com()
            elif isinstance(coll.geometry, representation.Box):
                vol = coll.geometry.size[0] * coll.geometry.size[1] * coll.geometry.size[2]
            elif isinstance(coll.geometry, representation.Sphere):
                vol = 4 / 3 * np.pi * pow(coll.geometry.radius, 3)
            elif isinstance(coll.geometry, representation.Cylinder):
                vol = np.pi * coll.geometry.length * pow(coll.geometry.radius, 2)
            else:
                raise TypeError("Geometry type not known!")
            # print(link.name, type(coll.geometry),  com, volume)
            volume += vol
            com = com + T.dot(T_com)[0:3, 3] * vol

        com = com / volume
        # print(com)
        self.transform_inertial(link.name, translation=com, rotation=[0.0, 0.0, 0.0])

    def move_joint_to_intersection(self, joint, other_joints):
        """
        Transforms the joint in such way that it's axis intersects with the two other joints
        :param joint: the joint to transform
        :param other_joints: the joints with which the axis of "joint" intersects
        :return: None
        """
        assert len(other_joints) == 2
        mj_name = joint
        tjj_names = other_joints

        mj = self.get_joint(mj_name)
        tjj = [self.get_joint(tjj_names[0]), self.get_joint(tjj_names[1])]
        T = [
            self.get_transformation(mj.child),
            self.get_transformation(tjj[0].child),
            self.get_transformation(tjj[1].child)
        ]
        axis = [
            np.matmul(T[0][:3, :3], np.array(mj.axis)),
            np.matmul(T[2][:3, :3], np.array(tjj[1].axis)),
            np.matmul(T[1][:3, :3], np.array(tjj[0].axis))
        ]
        planes_d = [axis[i].dot(T[i][:3, 3]) for i in range(3)]
        intersection = np.linalg.solve(axis, planes_d)
        transl = inv(T[0])[:3, :3].dot(intersection - T[0][:3, 3])
        T_ = create_transformation(xyz=transl, rpy=[0, 0, 0])
        self.transform_link_orientation(mj.child, T_, only_frame=True)

    def check_joint_definitions(self, raise_error=False, backup=None, default_axis=None):
        """
        Checks all joint limits whether they are valid. Prints the results. and returns the related code
        :backup: If joint limits restrict motion, we take the default value if given
        :raise_error: if set to true it will raise an error
        :return: 0000: everything is fine
                 0001: not all joint limits defined
                 0010: not everywhere are joint limits completely defined
                 0100: not all joint axis defined
                 1000: some joint limits might restrict motion
        """
        if default_axis is None:
            default_axis = [0.0, 0.0, 1.0]
        result = 0
        for joint in self.joints:
            if joint.joint_type == "fixed":
                joint.axis = None
                joint.limit = None
                joint.mimic = None
                continue
            if not hasattr(joint, "limit") or joint.limit is None:
                result &= 1
                if backup is not None:
                    joint.limit = representation.JointLimit(
                        effort=backup["eff"] if "eff" in backup.keys() else None,
                        velocity=backup["vel"] if "vel" in backup.keys() else None,
                        lower=backup["min"] if "min" in backup.keys() else None,
                        upper=backup["max"] if "max" in backup.keys() else None)
                    log.warfning(f"Joint limits for {joint.name} not defined taking default values!")
                elif raise_error:
                    log.error(joint.to_urdf_string())
                    raise ValueError(f"ERROR: Joint limits for {joint.name} not defined!")
            else:
                if any([not hasattr(joint.limit, x) for x in ["lower", "upper", "effort", "velocity"]]) or \
                        any([getattr(joint.limit, x) is None for x in ["lower", "upper", "effort", "velocity"]]):
                    log.error(f"{joint.name} {joint.limit.lower} {joint.limit.upper} {joint.limit.effort} "
                              f"{joint.limit.velocity}")
                    result &= 2
                    if raise_error:
                        raise ValueError("ERROR: Not all joint limits for " + joint.name + " defined!")
                if (joint.joint_type == "revolute" or joint.joint_type == "prismatic") and \
                        not hasattr(joint, "axis") or joint.axis is None:
                    log.warning(f"Joint axis for joint {joint.name} not defined. Setting to [0 0 1]")
                    joint.axis = default_axis
                    result &= 4
                if hasattr(joint, "limit") and joint.limit is not None and (
                        joint.limit.lower == joint.limit.upper or joint.limit.velocity == 0):
                    log.warning(f"The joint limits of joint {joint.name} might restrict motion: "
                                f"min: {joint.limit.lower} max: {joint.limit.upper} vel: {joint.limit.velocity} "
                                f"eff: {joint.limit.effort}")
                    result &= 8
                    if backup is not None:
                        limit_temp = [joint.limit.lower, joint.limit.upper]
                        if "min" in backup.keys() and limit_temp[0] == limit_temp[1]:
                            joint.limit.lower = read_number_from_config(backup["min"])
                        if "max" in backup.keys() and limit_temp[0] == limit_temp[1]:
                            joint.limit.upper = read_number_from_config(backup["max"])
                        if "vel" in backup.keys() and joint.limit.velocity == 0:
                            joint.limit.velocity = backup["vel"]
                        if "eff" in backup.keys() and joint.limit.effort == 0:
                            joint.limit.effort = backup["eff"]
                        log.info(f"Therefore we take the backup/default values for the joint limits:\n"
                                    f"min: {joint.limit.lower} max {joint.limit.upper} vel {joint.limit.velocity} "
                                    f"eff {joint.limit.effort}")

    def generate_collision_matrix(self, coll_override=None, no_coll_override=None):
        """
        Generates a matrix which collisions are allowed to happen/should be checked for and which not.

        Returns a list of the collision names, and th collision_matrix. The names list resolves the indices of the
        matrix.

        In theory everything should collide, but we are sure that the following collisions must not be computed:
            a) Collisions in the same link
            b) Collisions with the Collision in the parent frame
            c) Collisions with collisions in the parent's parent and above as long as the transformation between those
               links is only rotational
            d) Collisions where the bounding_boxes touch
        Further (not yet implemented) :
            e) Collisions where the bounding_boxes are closer than a threshold
            f) Collisions that are impossible to collide

        By this criteria we create the collision matrix
        """
        if no_coll_override is None:
            no_coll_override = {}
        if coll_override is None:
            coll_override = {}
        colls = {}
        link_colls = {}
        for link in self.links:
            colls.update({c.name: {"coll": c, "linkname": link.name} for c in link.collisions})
            link_colls.update({link.name: [c.name for c in link.collisions]})
        coll_names = list(colls.keys())
        link_names = [colls[cn]["linkname"] for cn in coll_names]

        # resolve link names to their corresponding collision names in coll_override und no_coll_override
        for k, v in coll_override.items():
            if k in link_names:
                for c in link_colls[k]:
                    if c in coll_override.keys():
                        coll_override[c] = list(set(coll_override[c] + v))
                    else:
                        coll_override[c] = list(set(v))
                coll_override.pop(k)
            else:
                for name in v:
                    if name in link_names:
                        temp = deepcopy(v)
                        temp.remove(name)
                        temp += link_colls[name]
                        coll_override[k] = temp

        for k, v in no_coll_override.items():
            if k in link_names:
                for c in link_colls[k]:
                    if c in no_coll_override.keys():
                        no_coll_override[c] = list(set(no_coll_override[c] + v))
                    else:
                        no_coll_override[c] = list(set(v))
                no_coll_override.pop(k)
            else:
                for name in v:
                    if name in link_names:
                        temp = deepcopy(v)
                        temp.remove(name)
                        temp += link_colls[name]
                        no_coll_override[k] = temp

        n_colls = len(coll_names)
        coll_matrix = np.ones((n_colls, n_colls))

        def set_coll(i_, j_, val):
            coll_matrix[i_, j_] = val
            coll_matrix[j_, i_] = val

        # get zero pose collisions d)
        zero_pose_colls = pgu.find_zero_pose_collisions(self)
        if zero_pose_colls is not None:  # d) All collisions that exist in the zero pose should not happen
            for c1, c2 in zero_pose_colls:
                # print(c1, "and", c2, "would collide in zero pose, therefore the shall not collide.")
                i = coll_names.index(c1)
                j = coll_names.index(c2)
                set_coll(i, j, 0)

        # create coll_matrix
        for i in range(n_colls):
            for j in range(i):
                # adam and eve are the distant parents of the only rotational tree given in ._ancestors
                j_ancestors, adam, _ = find_close_ancestor_links(self, link_names[j])
                i_ancestors, eve, _ = find_close_ancestor_links(self, link_names[i])
                j_ancestors = [adam] + j_ancestors
                i_ancestors = [eve] + i_ancestors
                if (
                        i == j or
                        link_names[i] == link_names[j] or  # a)
                        link_names[i] in j_ancestors or  # b) and c)
                        link_names[j] in i_ancestors  # b) and c)
                ):
                    set_coll(i, j, 0)
                # f)
                # i_prior_to_j = link_names[i] in self.get_chain(self.get_root(), link_names[j], joints=False)
                # j_prior_to_i = link_names[j] in self.get_chain(self.get_root(), link_names[i], joints=False)
                # if i_prior_to_j and adam == i_ancestors[-1]:
                #     jn = self.getParent(adam)
                #     joint = self.getJoint(jn[0])
                #     if np.linalg.norm(joint.origin.xyz) > 0.3:
                #         for iln in i_ancestors[1:]:
                #             for jln in j_ancestors[:-1]:
                #                 print("No collision between", iln, jln)
                #                 setColl(link_names.index(iln), link_names.index(jln), 0)
                # elif j_prior_to_i and eve == j_ancestors[-1]:
                #     jn = self.getParent(eve)
                #     joint = self.getJoint(jn[0])
                #     if np.linalg.norm(joint.origin.xyz) > 0.3:
                #         for jln in j_ancestors[1:]:
                #             for iln in i_ancestors[:-1]:
                #                 print("No collision between", iln, jln)
                #                 setColl(link_names.index(iln), link_names.index(jln), 0)

                # override with user input
                if ((coll_names[i] in no_coll_override.keys() and coll_names[j] in no_coll_override[coll_names[i]]) or
                        (coll_names[j] in no_coll_override.keys() and coll_names[i] in no_coll_override[
                            coll_names[j]])):
                    set_coll(i, j, 0)
                elif ((coll_names[i] in coll_override.keys() and coll_names[j] in coll_override[coll_names[i]]) or
                      (coll_names[j] in coll_override.keys() and coll_names[i] in coll_override[coll_names[j]])):
                    set_coll(i, j, 1)
        # print(coll_matrix)
        return coll_names, coll_matrix

    def set_self_collision(self, val=False, coll_override=None, no_coll_override=None, **kwargs):
        """If True, tries to avoid self collision with bitmasks which do not intersect.
        """
        if no_coll_override is None:
            no_coll_override = {}
        if coll_override is None:
            coll_override = {}

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
        not_possible = []
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
                    not_possible.append((coll_names[i], coll_names[j]))
        if len(not_possible) > 0:
            log.warning(f"Auto-Bitmask algorithm was unable to create bitmasks that are able to represent all collisions!")
            for np in not_possible:
                log.debug(f"  {np[0]} <-> {np[1]}")
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

    def rename_all(self, prefix=None, suffix=None, replacements=None, do_not_double=True):
        for targettype in ["links", "joints", "collisions", "visuals", "sensors", "motors", "submechanisms",
                           "exoskeletons", "annotations"]:
            self.rename(targettype, getattr(self, targettype), prefix=prefix, suffix=suffix, replacements=replacements,
                        do_not_double=do_not_double)

    def rename(self, targettype, target, prefix=None, suffix=None, replacements=None, do_not_double=True):
        """
        Renames the target with the given args
        Note: Override this in subclasses and call super at the beginning
        :param targettype: type of the target
        :param target: the name or list of names to rename
        :param prefix: a prefix ot add
        :param suffix: a suffix to add
        :param replacements: a dict of replacements
        :param do_not_double: make sure prefixes and suffixes are not added multiple times
        :return: the entities that have been renamed
        """
        renamed_entities = {}
        if type(target) is list:
            for t in target:
                renamed_entities.update(self.rename(targettype, t, prefix=prefix, suffix=suffix,
                                                    replacements=replacements, do_not_double=do_not_double))
            return renamed_entities
        elif type(target) is not str:
            target = str(target)
        if replacements is None:
            replacements = {}
        if do_not_double:
            if prefix is not None and target.startswith(prefix):
                prefix = None
            if suffix is not None and target.endswith(suffix):
                suffix = None

        if not prefix and not suffix and replacements == {}:
            return renamed_entities

        new_name = edit_name_string(target, prefix=prefix, suffix=suffix, replacements=replacements)
        renamed_entities.update(self._rename(targettype, target, new_name))

        if targettype in ['link', "links"]:
            for k, v in self.submodel_defs.items():
                if target == v["start"]:
                    self.submodel_defs[k]["start"] = new_name
                if target in v["stop"]:
                    self.submodel_defs[k]["stop"] = [link if link != target else new_name for link in v]

        return renamed_entities

    def edit_names(self, cfg):
        """
        Renames everything according to the given dict.
        Dict has to be of structure:
        {
            joint_equals_link_name: whether the joint shall be named like the link without the "_link" suffix
            append_link_suffix: ALWAYS or NAME_DUPLICATES or False
            replacements: dict or list of dicts, applied on everything
            collision_replacements: dict or list of dicts, applied on collisionnames
            collision_prefix: str
            collision_suffix: str
            visual_replacements: dict or list of dicts, applied on visualnames
            visual_prefix: str
            visual_suffix: str
            submechanism_replacements: dict or list of dicts, applied on submechanism contextual names
            submechanism_prefix: str
            submechanism_suffix: str
        }
        :param cfg:
        :return:
        """
        prefix = cfg["prefix"] if "prefix" in cfg.keys() else ""
        suffix = cfg["suffix"] if "suffix" in cfg.keys() else ""
        replacements = cfg["replacements"] if "replacements" in cfg.keys() else {}
        vis_replacements = cfg["visual_replacements"] if "visual_replacements" in cfg.keys() else {}
        vis_suffix = cfg["visual_suffix"] if "visual_suffix" in cfg.keys() else ""
        vis_prefix = cfg["visual_prefix"] if "visual_prefix" in cfg.keys() else ""
        coll_replacements = cfg["collision_replacements"] if "collision_replacements" in cfg.keys() else {}
        coll_suffix = cfg["collision_suffix"] if "collision_suffix" in cfg.keys() else ""
        coll_prefix = cfg["collision_prefix"] if "collision_prefix" in cfg.keys() else ""
        link_replacements = cfg["link_replacements"] if "link_replacements" in cfg.keys() else {}
        link_suffix = cfg["link_suffix"] if "link_suffix" in cfg.keys() else ""
        link_prefix = cfg["link_prefix"] if "link_prefix" in cfg.keys() else ""
        joint_replacements = cfg["joint_replacements"] if "joint_replacements" in cfg.keys() else {}
        joint_suffix = cfg["joint_suffix"] if "joint_suffix" in cfg.keys() else ""
        joint_prefix = cfg["joint_prefix"] if "joint_prefix" in cfg.keys() else ""
        submech_replacements = cfg["submechanisms_replacements"] if "submechanisms_replacements" in cfg.keys() else {}
        submech_suffix = cfg["submechanisms_suffix"] if "submechanisms_suffix" in cfg.keys() else ""
        submech_prefix = cfg["submechanisms_prefix"] if "submechanisms_prefix" in cfg.keys() else ""
        for link in self.links:
            self.rename("link", link.name, replacements=replacements, prefix=prefix, suffix=suffix)
            self.rename("link", link.name, replacements=link_replacements, prefix=link_prefix, suffix=link_suffix)
            for collision in link.collisions:
                self.rename("collision", collision.name, replacements=replacements, prefix=prefix, suffix=suffix)
                self.rename("collision", collision.name, prefix=coll_prefix, suffix=coll_suffix,
                            replacements=coll_replacements)
            for visual in link.visuals:
                self.rename("visual", visual.name, replacements=replacements, prefix=prefix, suffix=suffix)
                self.rename("visual", visual.name, prefix=vis_prefix, suffix=vis_suffix, replacements=vis_replacements)
        for joint in self.joints:
            if "joint_equals_link_name" in cfg.keys() and cfg["joint_equals_link_name"]:
                self.rename(targettype="joint", target=joint.name, prefix=prefix, suffix=suffix, replacements={
                    joint.name: joint.child if not joint.child.upper().endswith("_LINK") else joint.child[:-5]
                })
            else:
                self.rename(targettype="joint", target=joint.name, prefix=prefix, suffix=suffix, replacements=replacements)
                self.rename(targettype="joint", target=joint.name, prefix=joint_prefix, suffix=joint_suffix, replacements=joint_replacements)
        for submech in self.submechanisms:
                self.rename(targettype="submechanism", target=submech.name, prefix=prefix, suffix=suffix, replacements=replacements)
                self.rename(targettype="submechanism", target=submech.name, prefix=submech_prefix, suffix=submech_suffix, replacements=submech_replacements)
        if "append_link_suffix" in cfg.keys() and cfg["append_link_suffix"] is not False:
            for link in self.links:
                if not link.name[-4:].upper() == "LINK":
                    if cfg["append_link_suffix"].upper() == "ALWAYS":
                        self.rename(targettype="link", target=link.name, suffix="_Link")
                    elif cfg["append_link_suffix"].upper() == "NAME_DUPLICATES":
                        pjoint = self.get_parent(link.name)
                        if pjoint is not None and pjoint == link.name:
                            self.rename(targettype="link", target=link.name, suffix="_Link")

    def set_collision_scale(self, linkname, scale):
        """
        Scales the size of the collisions of the given link
        :param linkname: the link to edit
        :param scale: the wanted scale
        :return:
        """
        if isinstance(linkname, list):
            for lName, lScale in zip(linkname, scale):
                self.set_collision_scale(lName, lScale)
            return
        else:
            c_link = self.get_link(linkname)
            for col in c_link.collisions:
                if hasattr(col, 'geometry'):
                    if hasattr(col.geometry, 'scale'):
                        col.geometry.scale = scale
            return

    def clean_meshes(self):
        """
        Checks all meshes if they have sufficient vertices. If not they will be removed
        Note: override this in subclasses if they have meshes dont forget the super call
        :return: None
        """
        for link in self.links:
            for vc in link.collisions:
                if isinstance(vc.geometry, representation.Mesh) and not vc.geometry.is_valid():
                    log.warning(f"Mesh-Geometry {vc.name} {vc.geometry.input_file} is empty/to small; removing the corresponding {repr(type(vc)).split('.')[-1][:-2]}!")
                    link.remove_aggregate(vc)

    def attach(self, other, joint, do_not_rename=False, name_prefix="", name_suffix="_2", link_other=False):
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
        # Check if other is robot
        if not isinstance(other, Robot):
            raise Exception("Can only attach robot to robot.")

        if not link_other:
            other = other.duplicate()
        elif not do_not_rename:
            log.warning(f"Robot::attach(): The arguments you chose may result in the renaming of parts of the robot {other.name}")

        other.link_entities()

        if self.get_link_id(joint.parent) is None:
            raise Exception("Provide valid link to attach to. '" + joint.parent + "' is not in " + str(
                [ln.name for ln in self.links]))

        if not isinstance(joint, representation.Joint):
            raise Exception("Provide valid joint type.")

        # Check for naming and rename if necessary
        plink = set([str(link) for link in self.links])
        pjoints = set([str(j) for j in self.joints])
        pcollisions = set([str(c) for c in self.get_all_collisions()])
        pvisuals = set([str(v) for v in self.get_all_visuals()])
        pmaterials = set([str(m) for m in self.materials])
        psensors = set([str(m) for m in self.sensors])
        ptransmissions = set([str(m) for m in self.transmissions])
        pmotors = set([m.name for m in self.motors])
        psubmechanisms = set([str(m) for m in self.submechanisms])
        pexoskeletons = set([str(m) for m in self.exoskeletons])
        pinterfaces = set([str(m) for m in self.interfaces])
        pposes = set([m.name for m in self.poses])

        clink = set([str(link) for link in other.links])
        cjoints = set([str(j) for j in other.joints])
        ccollisions = set([str(c) for c in other.get_all_collisions()])
        cvisuals = set([str(v) for v in other.get_all_visuals()])
        cmaterials = set([str(m) for m in other.materials])
        csensors = set([str(m) for m in other.sensors])
        ctransmissions = set([str(m) for m in other.transmissions])
        cmotors = set([m.name for m in other.motors])
        csubmechanisms = set([str(m) for m in other.submechanisms])
        cexoskeletons = set([str(m) for m in other.exoskeletons])
        cinterfaces = set([str(m) for m in other.interfaces])
        cposes = set([m.name for m in other.poses])

        renamed_entities = {}
        joint.unlink_from_robot()

        if plink & clink:
            if not do_not_rename:
                log.warning(f"Link names are duplicates. A {name_prefix} and a {name_suffix}"
                            f" will be pre-/appended! {plink & clink}")
                renamed_entities.update(
                    other.rename(targettype="link", target=list(plink & clink), prefix=name_prefix, suffix=name_suffix))
                if joint.child in list(plink & clink):
                    joint.child = name_prefix + joint.child + name_suffix
            else:
                raise NameError("There are duplicates in link names", repr(plink & clink))

        if pjoints & cjoints:
            if not do_not_rename:
                log.warning(f"Joint names are duplicates a {name_suffix} will be appended! {pjoints & cjoints}")
                renamed_entities.update(other.rename(targettype="joint", target=list(pjoints & cjoints), prefix=name_prefix, suffix=name_suffix))
                if joint.name in list(pjoints & cjoints):
                    joint.name = joint.name + "_2"
            else:
                raise NameError("There are duplicates in joint names", repr(pjoints & cjoints))

        if pcollisions & ccollisions:
            if not do_not_rename:
                log.warning(f"Collision names are duplicates a {name_suffix} will be appended! {pcollisions & ccollisions}")
                renamed_entities.update(
                    other.rename(targettype="collision", target=list(pcollisions & ccollisions), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in collision names", repr(pcollisions & ccollisions))

        if pvisuals & cvisuals:
            if not do_not_rename:
                log.warning(f"Visual names are duplicates a {name_suffix} will be appended! {pvisuals & cvisuals}")
                renamed_entities.update(
                    other.rename(targettype="visual", target=list(pvisuals & cvisuals), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in visual names", repr(pvisuals & cvisuals))

        for mat_name in pmaterials & cmaterials:
            if self.get_material(mat_name).equivalent(other.get_material(mat_name)):
                cmaterials.remove(mat_name)
        if pmaterials & cmaterials:
            if not do_not_rename:
                log.warning(f"Material names are duplicates a {name_suffix} will be appended! {pmaterials & cmaterials}")
                renamed_entities.update(
                    other.rename(targettype="material", target=list(pmaterials & cmaterials), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in material names", repr(pmaterials & cmaterials))

        # check equivalence
        conflicting_sensors = psensors & csensors
        for sname in psensors & csensors:
            if self.get_sensor(sname).equivalent(other.get_sensor(sname)):
                self.get_sensor(sname).merge(other.get_sensor(sname))
                conflicting_sensors.remove(sname)
        if conflicting_sensors:
            if not do_not_rename:
                log.warning(f"Sensor names are duplicates a {name_suffix} will be appended! {conflicting_sensors}")
                renamed_entities.update(
                    other.rename(targettype="sensor", target=list(conflicting_sensors), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in sensor names", repr(conflicting_sensors))

        if ptransmissions & ctransmissions:
            if not do_not_rename:
                log.warning(f"Transmission names are duplicates a {name_suffix} will be appended! {ptransmissions & ctransmissions}")
                renamed_entities.update(
                    other.rename(targettype="transmission", target=list(ptransmissions & ctransmissions), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in transmission names", repr(ptransmissions & ctransmissions))

        if pmotors & cmotors:
            if not do_not_rename:
                log.warning(f"Motor names are duplicates a {name_suffix} will be appended! {pmotors & cmotors}")
                renamed_entities.update(
                    other.rename(targettype="motor", target=list(pmotors & cmotors), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in motor names", repr(pmotors & cmotors))

        if pexoskeletons & cexoskeletons:
            if not do_not_rename:
                log.warning(f"Exoskeleton names are duplicates a {name_suffix} will be appended! {pexoskeletons & cexoskeletons}")
                renamed_entities.update(
                    other.rename(targettype="exoskeleton", target=list(pexoskeletons & cexoskeletons), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in exoskeleton names", repr(pexoskeletons & cexoskeletons))

        if psubmechanisms & csubmechanisms:
            if not do_not_rename:
                log.warning(f"Submechanism names are duplicates a {name_suffix} will be appended! {psubmechanisms & csubmechanisms}")
                renamed_entities.update(
                    other.rename(targettype="submechanism", target=list(psubmechanisms & csubmechanisms), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in submechanism names", repr(psubmechanisms & csubmechanisms))

        if pinterfaces & cinterfaces:
            if not do_not_rename:
                log.warning(f"Interface names are duplicates a {name_suffix} will be appended! {pinterfaces & cinterfaces}")
                renamed_entities.update(
                    other.rename(targettype="interface", target=list(pinterfaces & cinterfaces), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in interface names", repr(pinterfaces & cinterfaces))

        new_poses = [p for p in pposes if p.name not in pposes & cposes] + [p for p in cposes if p.name not in pposes & cposes]
        conflicting_poses = []
        for pose_name in pposes & cposes:
            ppose = [pp for pp in self.poses if pp.name == pose_name][0]
            cpose = [cp for cp in other.poses if cp.name == pose_name][0]
            if not ppose.conflicts_with(cpose):
                new_poses.append(JointPoseSet.merge(ppose, cpose))
            else:
                conflicting_poses.append(pose_name)
        if len(conflicting_poses) > 0:
            if not do_not_rename:
                log.warning(f"Pose names are duplicates a {name_suffix} will be appended! {conflicting_poses}")
                renamed_entities.update(
                    other.rename(targettype="pose", target=list(conflicting_poses), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in pose names", repr(conflicting_poses))

        if renamed_entities != {}:
            return self.attach(other, joint, do_not_rename=do_not_rename,
                               link_other=True)  # this has been copied here already if link_other false

        n_joints = len(self.joints)
        n_links = len(self.links)
        # Add all joints
        self.add_aggregate('joint', other.joints)
        self.add_aggregate('link', other.links)

        for cMaterial in other.materials:
            if not any([cMaterial.equivalent(sMat) and sMat.name == cMaterial.name for sMat in self.materials]):
                self.add_aggregate('material', cMaterial)

        self.add_aggregate('transmission', other.transmissions)
        self.add_aggregate('sensor', other.sensors)
        self.add_motor(other.motors)
        self.add_aggregate('submechanism', other.submechanisms)
        self.add_aggregate('exoskeleton', other.exoskeletons)
        self.add_aggregate('interface', other.interfaces)

        self.poses = []
        for cPose in new_poses:
            self.add_aggregate('pose', cPose)

        self.add_aggregate('joint', joint)
        self.link_entities()
        assert joint.check_valid()
        assert str(other.get_root()) == str(joint.child)
        assert len(set([str(l) for l in self.links])) == len(self.links)
        assert len(self.joints) - n_joints == len(other.joints) + 1
        assert len(set([j.child for j in self.joints])) == len([j.child for j in self.joints])
        assert len(self.joints) == len(self.links) - 1
        assert len(self.joints) == len(self.get_joints_ordered_df()), f"{sorted([str(x) for x in self.joints])}\n{sorted([str(x) for x in self.get_joints_ordered_df()])}"
        assert self.get_root()

        return renamed_entities

    def add_link_by_properties(self, name, joint, mass=0.0, add_default_motor=True, **kwargs):
        """
        Adds a link with the given parameters.
        This method has to be overridden in subclasses.
        :param name: the name of the link
        :param joint: the representation.Joint which connects the link to the robot
        :param mass: the point mass we should add to this link
        :return: parent link and the created link and joint
        """
        if name in [link.name for link in self.links]:
            raise NameError("You can't add '" + name + "' as the model already contains a link with this name!")
        if joint.name in [j.name for j in self.joints]:
            raise NameError("You can't add '" + joint.name + "' as the model already contains a joint with this name!")
        if self.get_link(joint.parent, verbose=True) is None:
            raise NameError("The parent '" + joint.name + "' of the given joint does not exist!")
        if mass > 0.0:
            inertial = representation.Inertial(
                mass=mass, inertia=representation.Inertia(
                    ixx=1e-6,
                    iyy=1e-6,
                    izz=1e-6,
                ),
                origin=representation.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0], relative_to=name)
            )
        else:
            inertial = None
        link = representation.Link(name, inertial=inertial,**kwargs)

        assert joint.child == name
        self.add_aggregate("link", link)
        self.add_aggregate("joint", joint)
        link.link_with_robot(self)
        joint.link_with_robot(self)
        if joint.joint_type in ["revolute", "prismatic"] and add_default_motor:
            motor = representation.Motor(
                name=joint.name,
                joint=joint
            )
            self.add_motor(motor)

    # has to be overridden in smurf and hyrodyn?
    def mirror_model(self, mirror_plane=None, flip_axis=1, exclude_meshes=None, name_replacements=None,
                     target_urdf=None, target_smurf=None, only_return=False, final_linking_optional=False):
        """
        Mirrors the robot model.
        :param mirror_plane: The normal of the mirror plane. Default y-plane
        :param flip_axis: The axis which will be flipped for maintaining a right handed coordinate frame while mirroring (default y-axis)
        :param exclude_meshes: List of meshes that will not be mirrored. Use this for already symmetric meshes
        :param name_replacements: dict, of name replacements for all names
        :param only_return: if set to true this instance remains untouched and the mirrored model will only be returned
        :return: None or mirrored model depending on only_return
        """
        self.unlink_entities()  # so we dont mess up everything when deepcopying
        # Create a new robot
        if name_replacements is None:
            name_replacements = {}
        if exclude_meshes is None:
            exclude_meshes = []
        if mirror_plane is None:
            mirror_plane = [0, 1, 0]
        robot = type(self)(self.name, assert_validity=False)
        # add everything on which mirroring has no influence
        for mat in self.materials:
            robot.add_aggregate("material", mat)
        for mat in self.transmissions:
            robot.add_aggregate("transmission", mat)
        for mat in self.exoskeletons:
            robot.add_aggregate("exoskeleton", mat)
        for mat in self.submechanisms:
            robot.add_aggregate("submechanism", mat)
        for mat in self.motors:
            robot.add_motor(mat)
        for mat in self.poses:
            robot.add_aggregate("pose", mat)
        for mat in self.sensors:
            robot.add_aggregate("sensor", mat)
        if not target_urdf:
            target_urdf = self.xmlfile
        robot.xmlfile = target_urdf
        robot.unlink_entities()

        # reflection matrix
        T_R = pgu.get_reflection_matrix(normal=np.array(mirror_plane))

        # copy kinematic
        def recursive_link_transform(link_, T_root_to_link):
            link_ = self.get_link(link_)
            T_link = self.get_transformation(link_.name)
            new_link = link_.duplicate()

            T_flip = np.eye(4)
            # All frames are mirrored using least-maintain-axis as flip axis
            T_flip[flip_axis, flip_axis] *= -1

            # transform link information to root and mirror

            if new_link.inertial is not None:
                T = T_R.dot(T_link.dot(link_.inertial.origin.to_matrix()))
                new_origin = representation.Pose.from_matrix(inv(T_root_to_link).dot(T), relative_to=new_link)
                new_origin.rotation = [0, 0, 0]

                # Process the rotation
                T = inv(T_root_to_link.dot(link_.inertial.origin.to_matrix())).dot(T)
                Ad = get_adjoint(T)
                # Get the Inertia Tensor
                I = link_.inertial.to_mass_matrix()
                # Rotate the inertial tensor
                I = np.matmul(np.transpose(Ad), np.matmul(I, Ad))

                new_link.inertial = representation.Inertial.from_mass_matrix(I, new_origin)

            for vis in new_link.visuals:
                T = T_R.dot(T_link.dot(vis.origin.to_matrix()))
                vis.origin = representation.Pose.from_matrix(inv(T_root_to_link).dot(T.dot(T_flip)), relative_to=new_link)
                if isinstance(vis.geometry, representation.Mesh) and not (
                        (vis.geometry.original_mesh_name in exclude_meshes or "ALL" in exclude_meshes)):
                    vis.geometry.mirror(mirror_transform=inv(T_root_to_link.dot(vis.origin.to_matrix())).dot(T),
                                        name_replacements=name_replacements)

            for col in new_link.collisions:
                T = T_R.dot(T_link.dot(col.origin.to_matrix()))
                col.origin = representation.Pose.from_matrix(inv(T_root_to_link).dot(T.dot(T_flip)), relative_to=new_link)
                if isinstance(col.geometry, representation.Mesh) and not (
                        (col.geometry.original_mesh_name in exclude_meshes or "ALL" in exclude_meshes)):
                    col.geometry.mirror(mirror_transform=inv(T_root_to_link.dot(col.origin.to_matrix())).dot(T),
                                        name_replacements=name_replacements)

            robot.add_aggregate("link", new_link)

            # joints:
            for jointname in self.get_children(link_.name):
                joint_ = self.get_joint(jointname)
                new_joint = joint_.duplicate()

                # now we transform the local coordinate system using t_flip to make it right handed
                new_root_to_joint = T_R.dot(T_link.dot(joint_.origin.to_matrix().dot(T_flip)))
                relative_to = self.get_parent(link_.name)
                if relative_to is None:
                    assert new_link.origin is None or new_link.is_zero()
                    relative_to = new_link
                assert relative_to is not None
                new_joint.origin = representation.Pose.from_matrix(inv(T_root_to_link).dot(new_root_to_joint), relative_to=str(relative_to))
                # the joint axes are mirrored so that there movement happens symmetrically
                if new_joint.joint_type != "fixed":
                    assert new_joint.axis is not None and np.linalg.norm(new_joint.axis) > 0.0
                    old_axis_point = T_link.dot(joint_.origin.to_matrix().dot(create_transformation(xyz=new_joint.axis)))
                    new_axis_point = T_R.dot(old_axis_point)
                    new_axis_point_in_joint_frame = inv(new_root_to_joint).dot(new_axis_point)
                    new_joint.axis = new_axis_point_in_joint_frame[0:3, 3]/np.linalg.norm(new_axis_point_in_joint_frame[0:3, 3])
                    new_joint.axis = list(round_array(new_joint.axis, dec=7))

                new_T_root_to_link = T_root_to_link.dot(new_joint.origin.to_matrix())

                robot.add_aggregate("joint", new_joint)
                recursive_link_transform(joint_.child, new_T_root_to_link)

        recursive_link_transform(self.get_root(), np.eye(4))

        robot.link_entities()

        robot.rename("link", robot.links, replacements=name_replacements)
        robot.rename("joint", robot.joints, replacements=name_replacements)
        robot.rename("collision", robot.get_all_collisions(), replacements=name_replacements)
        robot.rename("visual", robot.get_all_visuals(), replacements=name_replacements)

        # copy all annotations we not yet have
        for k, v in self.__dict__.items():
            if k not in robot.__dict__.keys() or robot.__dict__[k] is None:
                robot.__dict__[k] = v

        # reflection matrix
        T_R = get_reflection_matrix(normal=np.array(mirror_plane))

        # transform sensor frames
        for sensor in robot.sensors:
            if hasattr(sensor, "origin") and sensor.link is not None:
                T_link = self.get_transformation(sensor.link)
                T_root2link = robot.get_transformation(sensor.link)
                T = T_R.dot(T_link.dot(sensor.origin.to_matrix()))
                sensor.origin = representation.Pose.from_matrix(transform.inv(T_root2link).dot(T), relative_to=sensor.link)

        if target_smurf is not None:
            robot.smurffile = target_smurf

        if not final_linking_optional:
            robot.link_entities()
        robot.assert_validity()

        if only_return:
            return robot

        for k, v in robot.__dict__.items():
            setattr(self, k, v)

    def split_robot(self, link_to_cut):
        """
        Seperates the current robot model at the given link.
        :param link_to_cut: str or list of strings
        :return: before, beyond
            before: is the model before the given link(s) including the link_to_cut
            beyond: dict of models beyond resp. link_to_cut including link_to_cut
        """
        if type(link_to_cut) is str:
            links_to_cut = [link_to_cut]
        else:
            links_to_cut = [str(l) for l in link_to_cut]

        # Create new robot
        before = self.instantiate_submodel(
            name=self.name,
            start=self.get_root(),
            stop=links_to_cut,
            include_unstopped_branches=True
        )
        beyond = {
            new_root: self.instantiate_submodel(
                name=self.name,
                start=new_root,
                include_unstopped_branches=True
            ) for new_root in links_to_cut}
        return before, beyond

    def get_before(self, link_to_cut):
        """
        Wrapper for split_robot(). Removes everything beyond the given link
        :param link_to_cut: str or list, the link(s) where to cut. this link(s) will be included
        :return: None
        """
        before, _ = self.split_robot(link_to_cut)
        return before

    def get_beyond(self, link_to_cut):
        """
        Wrapper for split_robot(). Returns the beyond model
        :param link_to_cut: str or list, the link where to cut, will be included in the model
        :return: model beyond the given link including it
        """
        _, beyond = self.split_robot(link_to_cut)
        return beyond

    def remove_joint(self, jointname):
        """Remove the joint(s) from the mechanism and transforms all inertia, visuals and collisions
        to the corresponding parent of the joint.
        """
        joint = self.get_joint(jointname)
        if joint is None:
            log.warning(f"remove_joint(): Skipping removal of {jointname} as robot contains no joint with that name!")
        else:
            self.remove_aggregate("joints", self.get_joint(jointname))

    def add_floating_base(self):
        """
        Returns a copy of this robot with a floatingbase mechanisms prepended
        :return: instance of robot
        """
        floatingbase = type(self)(xmlfile=pkg_resources.resource_filename("phobos", "data/floatingbase.urdf"))
        connector = representation.Joint(
            name="FreeFlyerRZ",
            parent="FreeFlyerRY_Link",
            child=self.get_root(),
            joint_type='revolute',
            axis=[0, 0, 1],
            limit=representation.JointLimit(
                effort=0,
                velocity=0,
                lower=-1.57,
                upper=1.57
            )
        )
        fb_robot = self.duplicate()
        floatingbase.attach(fb_robot, connector)
        floatingbase.name = fb_robot.name + "_floatingbase"
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
        floatingbase.submechanisms += [Submechanism(**freeflyer)]
        floatingbase.autogenerate_submechanisms = self.autogenerate_submechanisms
        floatingbase.link_entities()
        return floatingbase

    def scale_link(self, linkname, scale_x, scale_y, scale_z, new_mass=None, geometry_for_inertia=None):
        """
        Scales the link with the given scale
        Args:
            linkname: The name of the link to scale
            scale_x: scale along x axis of the link
            scale_y: scale along y axis of the link
            scale_z: scale along z axis of the link
            new_mass: The new mass for the changed link
            geometry_for_inertia: an geometry object from which's shape the inertia will be calculated

        Returns:
            None
        """
        link = self.get_link(linkname)
        assert link is not None
        scale = np.array([scale_x, scale_y, scale_z])
        assert all([s >0 for s in scale])
        for geo in link.visuals + link.collisions:
            _geo_scale = inv(geo.origin.to_matrix())[:3, :3].dot(scale)
            geo.geometry.multiply_scale(np.abs(_geo_scale))
            # geo.geometry.apply_scale()
            geo.origin.xyz = [v*s for v, s in zip(geo.origin.xyz, scale)]
        joints = self.get_children(link.name)
        for jointname in joints:
            joint = self.get_joint(jointname)
            assert joint is not None
            joint.origin.xyz = [v*s for v, s in zip(joint.origin.xyz, scale)]
        link.inertial.mass = new_mass
        if geometry_for_inertia is None:
            raise AssertionError("No geometry for inertia calculation specified!")
        if isinstance(geometry_for_inertia, representation.Box):
            inertia_list = utils.inertia.calculateBoxInertia(new_mass, geometry_for_inertia.size)
        elif isinstance(geometry_for_inertia, representation.Cylinder):
            inertia_list = utils.inertia.calculateCylinderInertia(new_mass, geometry_for_inertia.radius, geometry_for_inertia.length)
        elif isinstance(geometry_for_inertia, representation.Sphere):
            inertia_list = utils.inertia.calculateSphereInertia(new_mass, geometry_for_inertia.radius)
        elif isinstance(geometry_for_inertia, representation.Mesh):
            inertia_list = utils.inertia.calculateMeshInertia(new_mass, geometry_for_inertia.load_mesh(self.xmlfile), geometry_for_inertia.scale)
        else:
            raise TypeError("geometry_for_inertia holds invalid type "+type(geometry_for_inertia))
        link.inertial.inertia = representation.Inertia(*inertia_list)
        link.inertial.inertia.link_with_robot(self)
