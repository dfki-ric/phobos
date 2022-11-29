import datetime
import sys
import os
import traceback
from typing import List, Any

import pkg_resources
import pydot
from copy import deepcopy, copy
from scipy.spatial.transform import Rotation as scipy_rot
import numpy as np

from ..defs import load_json, dump_json
from .. import geometry as pgu, utils
from ..geometry import get_reflection_matrix
from ..io import representation, sensor_representations
from ..io.hyrodyn import Submechanism, Exoskeleton
from ..io.xmlrobot import XMLRobot
from ..io.smurfrobot import SMURFRobot
from ..utils import transform
from ..utils.misc import read_angle_2_rad, regex_replace, create_dir, edit_name_string, execute_shell_command, \
    duplicate, color_parser
from ..utils.transform import create_transformation, inv, get_adjoint, round_array
from ..utils.tree import find_close_ancestor_links
from ..utils.xml import read_urdf_filename, transform_object, get_joint_info_dict

from ..utils.commandline_logging import get_logger
log = get_logger(__name__)


class Robot(SMURFRobot):
    def __init__(self, name=None, xmlfile=None, submechanisms_file=None, smurffile=None, verify_meshes_on_import=True,
                 inputfile=None, description=None, is_human=False, autogenerate_submechanisms=None):
        """ The basic robot class to represent a urdf.
        """
        try:
            super().__init__(xmlfile=xmlfile, submechanisms_file=submechanisms_file, smurffile=smurffile,
                             verify_meshes_on_import=verify_meshes_on_import, inputfile=inputfile, description=description,
                             autogenerate_submechanisms=autogenerate_submechanisms, is_human=is_human)
        except Exception as e:
            log.error(f"Failed loading:\n  input: {inputfile}\n  xml: {xmlfile}\n  submechanims: {submechanisms_file}\n  smurf: {smurffile}")
            raise e

        if name is not None:
            self.name = name
        self._submodels = {}

    def get_blender_model_dictionary(self):
        from phobos.blender import defs
        from phobos.blender.utils import naming as nUtils
        import mathutils
        model = {
            'links': {},
            'joints': {},
            'sensors': {},
            'motors': {},
            'controllers': {},
            'materials': {},
            'interfaces': {},
            'meshes': {},
            'lights': {},
            'groups': {},
            'chains': {},
            'date': datetime.datetime.now().strftime("%Y%m%d_%H:%M"),
            'name': self.name,
            'version': self.version,
            'description': self.description,
        }
        for material in self.materials:
            model["materials"][material.name] = {
                "name": material.name,
            }
            for key, value in material.to_yaml().items():
                if key.endswith("Color"):
                    model["materials"][material.name][key[:-5]] = color_parser(value)

        for link in self.links:
            model["links"][link.name] = {
                "name": link.name,
                "children": self.get_children(link.name, targettype="link"),
                "parent": self.get_parent(link.name, targettype="link"),
                "visual": {},
                "collision": {}
            }
            if link.inertial is not None:
                model["links"][link.name]["inertial"] = {
                    "pose": {'translation': link.inertial.origin.xyz,
                             'rotation_euler': link.inertial.origin.rpy,
                            },
                    "mass": link.inertial.mass,
                    "inertia": link.inertial.inertia.to_list(),
                    "name": f"inertial_{link.name}"
                }

            for obj in link.visuals + link.collisions:
                geometry_entry = {}
                if type(obj.geometry) == representation.Mesh:
                    geometry_entry["type"] = "mesh"
                    geometry_entry["filename"] = obj.geometry.filepath
                    geometry_entry["filepath"] = obj.geometry.filepath
                    geometry_entry["scale"] = obj.geometry.scale
                elif type(obj.geometry) == representation.Sphere:
                    geometry_entry["type"] = "sphere"
                    geometry_entry["radius"] = obj.geometry.radius
                elif type(obj.geometry) == representation.Cylinder:
                    geometry_entry["type"] = "cylinder"
                    geometry_entry["radius"] = obj.geometry.radius
                    geometry_entry["length"] = obj.geometry.length
                elif type(obj.geometry) == representation.Box:
                    geometry_entry["type"] = "box"
                    geometry_entry["size"] = obj.geometry.size
                model["links"][link.name]["visual" if type(obj) == representation.Visual else "collision"][obj.name] = {
                    "name": obj.name,
                    "pose": {'translation': obj.origin.xyz,
                             'rotation_euler': obj.origin.rpy,
                            },
                    "geometry": geometry_entry
                }
                if type(obj) == representation.Visual and obj.material is not None:
                    model["links"][link.name]["visual"][obj.name]["material"] = obj.material

        for joint in self.joints:
            model["joints"][joint.name] = {"name": joint.name,
                                           "type": joint.joint_type,
                                           "parent": joint.parent,
                                           "child": joint.child,
                                           }
            if joint.axis is not None and joint.joint_type != "fixed":
                model["joints"][joint.name]["axis"] = joint.axis
            if joint.limit is not None:
                model["joints"][joint.name]["limits"] = {
                    "lower": joint.limit.lower,
                    "upper": joint.limit.upper,
                    "effort": joint.limit.effort,
                    "velocity": joint.limit.velocity
                }
            if joint.mimic is not None:
                for k, v in joint.mimic.to_yaml().items():
                    model["joints"][joint.name]["mimic_"+k] = v
            # [TODO pre_v2.0.0] dynamics, etc.
            model["links"][joint.child]["pose"] = {'translation': joint.origin.xyz,
                                                   'rotation_euler': joint.origin.rpy,
                                                  }

        for link in model["links"].keys():
            if 'pose' not in model["links"][link]:
                model["links"][link]['pose'] = {'translation': [0, 0, 0], 'rotation_euler': [0, 0, 0]}

        for sensor in self.sensors:
            model["sensors"][sensor.name] = {
                "name": sensor.name,
                "props": sensor.to_yaml(),
                "type": sensor.type,
                "material": None,
                "shape": None,
                "size": None,
            }
            if hasattr(sensor, "origin"):
                location = mathutils.Matrix.Translation(tuple(sensor.origin.xyz) if sensor.origin.xyz is not None else (0,0,0))
                rotation = (
                    mathutils.Euler(tuple(sensor.origin.rpy) if sensor.origin.rpy is not None else (0,0,0), 'XYZ').to_matrix().to_4x4()
                )
                model["sensors"][sensor.name]["origin"] = location @ rotation
            if 'material' in defs.def_settings['sensors'][sensor.blender_type]:
                model["sensors"][sensor.name]["material"] = defs.def_settings['sensors'][sensor.blender_type]['material']
            if 'shape' in defs.def_settings['sensors'][sensor.blender_type]:
                model["sensors"][sensor.name]["shape"] = defs.def_settings['sensors'][sensor.blender_type]['shape']
            if 'size' in defs.def_settings['sensors'][sensor.blender_type]:
                model["sensors"][sensor.name]["size"] = defs.def_settings['sensors'][sensor.blender_type]['size']
            if "parent" in model["sensors"][sensor.name]["props"]:
                model["sensors"][sensor.name]["parent"] = model["sensors"][sensor.name]["props"]["parent"]
            else:
                if "link" in model["sensors"][sensor.name]["props"]:
                    model["sensors"][sensor.name]["parent"] = model["sensors"][sensor.name]["props"]["link"]
                elif "joint" in model["sensors"][sensor.name]["props"]:
                    model["sensors"][sensor.name]["parent"] = nUtils.getObjectName(str(self.get_joint(model["sensors"][sensor.name]["props"]["joint"]).child))
                else:
                    model["sensors"][sensor.name]["parent"] = nUtils.getObjectName(str(self.get_root()))

        # print(defs.def_settings['motors'])
        for motor in self.motors:
            model["motors"][motor.name] = {
                "name": motor.name,
                "props": motor.to_yaml(),
                "material": None,
                "shape": "resource://dc",
                "size": 0.25,
                "joint": str(motor.joint),
                "parent": self.get_joint(motor.joint).child
            }

        for interface in self.interfaces:
            model["interfaces"][interface.name] = interface.to_yaml()
            if hasattr(interface, "origin"):
                location = mathutils.Matrix.Translation(
                    tuple(interface.origin.xyz) if interface.origin.xyz is not None else (0, 0, 0))
                rotation = (
                    mathutils.Euler(tuple(interface.origin.rpy) if interface.origin.rpy is not None else (0, 0, 0),
                                    'XYZ').to_matrix().to_4x4()
                )
                model["interfaces"][interface.name]["origin"] = location @ rotation

        model['lights'] = self.annotations.get('lights')
        model['groups'] = self.annotations.get('groups')
        model['chains'] = self.annotations.get('chains')

        return model

    @classmethod
    def get_robot_from_blender_dict(cls, name='', objectlist=[], blender_model=None, xmlfile=None):
        """
        Uses blender workflow to access internal dictionary to call robot
        representation. Idea is to use cli methods and formats for imports and
        exports
        """
        import bpy
        import phobos.blender.utils.selection as sUtils
        from phobos.blender.model.models import deriveModelDictionary

        if blender_model is None:
            root = sUtils.getRoot(bpy.context.selected_objects[0])
            blender_model = deriveModelDictionary(root, name, objectlist)
            if blender_model is None:
                log.warning("Warning name your model and assign a version, otherwise blender-dictionary is None")
        try:
            cli_joints = []
            for key, values in blender_model['joints'].items():
                if not values['type'] == 'fixed' and values.get("limits") is not None:
                    cli_limit = representation.JointLimit(effort=values['limits'].get('effort'),
                                                          velocity=values['limits'].get('velocity'),
                                                          lower=values['limits'].get('lower'),
                                                          upper=values['limits'].get('upper'))
                else:
                    cli_limit = None
                mimic_dict = {}
                for k, v in values.items():  # [TODO pre_v2.0.0] this doesn't work, it seems phobos input dictionary is differently handled than the output dict
                    if k.startswith("mimic_"):
                        mimic_dict[k[len("mimic_"):]] = v
                cli_joints.append(representation.Joint(
                    name=values['name'],
                    parent=values['parent'],
                    child=values['child'],
                    joint_type=values['type'],
                    axis=values.get('axis'),
                    origin=representation.Pose.from_matrix(np.array(values['pose']['rawmatrix'])),
                    limit=cli_limit,
                    dynamics=None,
                    safety_controller=None,
                    calibration=None,
                    mimic=representation.JointMimic(**mimic_dict) if len(mimic_dict) > 0 else None
                ))

            cli_links = []
            for key, values in blender_model['links'].items():
                inert_entry = values.get('inertial')
                pose_entry = inert_entry.get('pose')
                inertia_val = inert_entry.get('inertia')
                if inertia_val is not None:
                    inert = representation.Inertial(mass=inert_entry['mass'],
                                                    inertia=representation.Inertia(*inertia_val),
                                                    origin=representation.Pose(
                                                        xyz=pose_entry['translation'],
                                                        rpy=pose_entry['rotation_euler']
                                                    ))
                else:
                    inert = None
                colls = []
                for key2, entry in values["collision"].items():
                    colls.append(representation.Collision(
                        geometry=representation.GeometryFactory.create(**entry["geometry"]),
                        origin=representation.Pose.from_matrix(np.array(entry["pose"]['rawmatrix'])),
                        name=entry["name"]))
                vis = []
                for key2, entry in values["visual"].items():
                    vis.append(representation.Visual(geometry=representation.GeometryFactory.create(**entry["geometry"]),
                                                     material=entry.get("material"),
                                                     origin=representation.Pose.from_matrix(np.array(entry["pose"]['rawmatrix'])),
                                                     name=entry["name"]))

                cli_links.append(representation.Link(
                    name=values['name'],
                    visuals=vis,
                    inertial=inert,
                    collisions=colls
                ))
            mats = []
            for key, value in blender_model['materials'].items():
                mats.append(representation.Material(name=value.pop('name'),
                                                    texture=None,
                                                    **value))
            if blender_model['version'] != '1.0':
                log.info(f"Versionscheck übersprungen. Version ist : {blender_model['version']}")
            cli_robot = XMLRobot(
                name=blender_model['name'],
                version=None,
                links=cli_links,
                joints=cli_joints,
                materials=mats,
                xmlfile=xmlfile
            )
            new_robot = Robot()
            new_robot.__dict__.update(cli_robot.__dict__)
            new_robot.description = blender_model["description"]
            new_robot.relink_entities()

            if "sensors" in blender_model:
                for key, values in blender_model['sensors'].items():
                    # [TODO pre_v2.0.0] "type" Abfragen an die verschiedenen User-Präferenzen angleichen
                    if values.get('id') is not None:
                        values['targets'] = [
                            x for x in values['id'] if (
                                    new_robot.get_joint(x, verbose=False) is not None or
                                    new_robot.get_link(x, verbose=False) is not None or
                                    new_robot.get_collision_by_name(x) is not None or
                                    new_robot.get_visual_by_name(x) is not None
                            )
                        ]
                        values.pop('id')
                    if values["type"].upper() in ["CAMERASENSOR", "CAMERA"]:
                        new_robot.add_sensor(
                            sensor_representations.CameraSensor(
                                         hud_height=240 if values.get('hud_height') is None else values.pop('hud_height'),
                                         hud_width=0 if values.get('hud_width') is None else values.pop('hud_width'),
                                         origin=None if values.get('origin') is None else representation.Pose(**values.pop("origin")),
                                         **values))
                    else:
                        new_robot.add_sensor(getattr(sensor_representations, values["type"])(**values))

            if "motors" in blender_model:
                motors = blender_model["motors"]
                for key, value in motors.items():
                    name = value.pop('name')
                    joint = value.pop('joint')
                    new_robot.add_motor(representation.Motor(name=name, joint=new_robot.get_joint(joint), **value))

            if "interfaces" in blender_model:
                interfaces = blender_model["interfaces"]
                for key, value in interfaces.items():
                    parent = value.pop('parent')
                    pose = value.pop('pose')
                    new_robot.add_aggregate("interfaces", representation.Interface(
                        parent=new_robot.get_link(parent),
                        origin=representation.Pose.from_matrix(np.array(pose['rawmatrix'])),
                        **value
                    ))

            additional_info = {'lights': blender_model.get('lights'),
                               'groups': blender_model.get('groups'),
                               'chains': blender_model.get('chains'),
                               # 'date': blender_model.get('date')
                               }

            for key, value in additional_info.items():
                if value is not None and key not in new_robot.named_annotations.keys():
                    new_robot.add_named_annotation(key, additional_info[key])
            new_robot.relink_entities()
        except Exception as e:
            print(blender_model)
            raise e
        return new_robot

    # export methods
    def export_urdf(self, outputfile=None, export_visuals=True, export_collisions=True, create_pdf=False,
                    ros_pkg=False, export_with_ros_pathes=None, float_fmt_dict=None):
        """Export the mechanism to the given output file.
        If export_visuals is set to True, all visuals will be exported. Otherwise no visuals get exported.
        If export_collisions is set to to True, all collisions will be exported. Otherwise no collision get exported.
        """
        if float_fmt_dict is None:
            float_fmt_dict = {}
        self.joints = self.get_joints_ordered_df()
        if not outputfile:
            outputfile = self.name

        outputfile = os.path.abspath(outputfile)
        if not export_visuals or not export_collisions:
            export_robot = self.duplicate()
            if not export_visuals:
                export_robot.remove_visuals()
            if not export_collisions:
                export_robot.remove_collisions()
        else:
            export_robot = self

        xml_string = export_robot.to_urdf_string(float_fmt_dict=float_fmt_dict)

        if ros_pkg is True:
            xml_string = regex_replace(xml_string, {'filename="../': 'filename="package://'})

        if not os.path.exists(os.path.dirname(os.path.abspath(outputfile))):
            os.makedirs(os.path.dirname(outputfile))
        with open(outputfile, "w") as f:
            f.write(xml_string)
            f.close()

        if export_with_ros_pathes is not None:
            if export_with_ros_pathes and not ros_pkg:
                xml_string = regex_replace(xml_string, {'filename="../': 'filename="package://'})
                f = open(outputfile[:-5] + "_ros.urdf", "w")
                f.write(xml_string)
                f.close()
            elif not export_with_ros_pathes and ros_pkg:
                xml_string = regex_replace(xml_string, {'filename="package://': 'filename="../'})
                f = open(outputfile[:-5] + "_relpath.urdf", "w")
                f.write(xml_string)
                f.close()

        if create_pdf:
            self.export_pdf(outputfile[:-5]+".pdf")
        log.info("URDF written to {}".format(outputfile))
        return

    def export_sdf(self, outputfile=None, export_visuals=True, export_collisions=True, create_pdf=False,
                   ros_pkg=False, export_with_ros_pathes=None, float_fmt_dict=None):
        """Export the mechanism to the given output file.
        If export_visuals is set to True, all visuals will be exported. Otherwise no visuals get exported.
        If export_collisions is set to to True, all collisions will be exported. Otherwise no collision get exported.
        """
        if float_fmt_dict is None:
            float_fmt_dict = {}
        self.joints = self.get_joints_ordered_df()
        if not outputfile:
            outputfile = self.name

        outputfile = os.path.abspath(outputfile)

        export_robot = self.duplicate()

        if not export_visuals:
            export_robot.remove_visuals()
        if not export_collisions:
            export_robot.remove_collisions()

        xml_string = "<sdf>\n"+export_robot.to_sdf_string(float_fmt_dict=float_fmt_dict)+"\n</sdf>"

        if ros_pkg is True:
            xml_string = regex_replace(xml_string, {'<uri>"../': '<uri>"package://'})

        if not os.path.exists(os.path.dirname(os.path.abspath(outputfile))):
            os.makedirs(os.path.dirname(outputfile))
        with open(outputfile, "w") as f:
            f.write(xml_string)
            f.close()

        if export_with_ros_pathes is not None:
            if export_with_ros_pathes and not ros_pkg:
                xml_string = regex_replace(xml_string, {'<uri>"../': '<uri>"package://'})
                f = open(outputfile[:-5] + "_ros.urdf", "w")
                f.write(xml_string)
                f.close()
            elif not export_with_ros_pathes and ros_pkg:
                xml_string = regex_replace(xml_string, {'<uri>"package://': '<uri>"../'})
                f = open(outputfile[:-5] + "_relpath.urdf", "w")
                f.write(xml_string)
                f.close()

        if create_pdf:
            self.export_pdf(outputfile[:-5]+".pdf")
        log.info("SDF written to {}".format(outputfile))
        return

    def export_xml(self, output_dir=None, export_visuals=True, export_collisions=True,
                   create_pdf=False, ros_pkg=False, export_with_ros_pathes=None, ros_pkg_name=None,
                   export_joint_limits=True, export_submodels=True, formats=["urdf"], filename=None,
                   float_fmt_dict=None, no_format_dir=False):
        """ Exports all model information stored inside this instance.
        """
        assert self.get_root()
        for format in formats:
            format = format.lower()
            # Main model
            if no_format_dir:
                model_file = os.path.join(output_dir, f"{self.name if filename is None else filename}")
            else:
                model_file = os.path.join(output_dir, f"{format}/{self.name if filename is None else filename}")
            if not model_file.lower().endswith(format):
                model_file += "." + format
            if not os.path.exists(os.path.dirname(model_file)):
                os.makedirs(os.path.dirname(model_file))
            if ros_pkg_name is None and (export_with_ros_pathes or ros_pkg):
                ros_pkg_name = os.path.basename(output_dir)
            self.relink_entities()
            self.xmlfile = model_file

            assert len(self.links) == len(self.joints) + 1
            if format == "urdf":
                self.export_urdf(outputfile=model_file, export_visuals=export_visuals,
                                 export_collisions=export_collisions, create_pdf=create_pdf,
                                 ros_pkg=ros_pkg, export_with_ros_pathes=export_with_ros_pathes, float_fmt_dict=float_fmt_dict)
            elif format == "sdf":
                self.export_sdf(outputfile=model_file, export_visuals=export_visuals,
                                export_collisions=export_collisions, create_pdf=create_pdf,
                                ros_pkg=ros_pkg, export_with_ros_pathes=export_with_ros_pathes, float_fmt_dict=float_fmt_dict)
            else:
                raise IOError("Unknown export format:" + format)

            if export_joint_limits:
                self.export_joint_limits(output_dir if no_format_dir else os.path.join(output_dir, format))

            if self._submodels and export_submodels:
                submodel_folder = os.path.join(output_dir, "submodels")
                if not os.path.exists(submodel_folder):
                    os.mkdir(submodel_folder)
                for sub_mod in self._submodels.keys():
                    if sub_mod.startswith("#sub_mech#"):
                        continue
                    self.export_submodel(sub_mod,
                                         output_dir=submodel_folder,
                                         export_visuals=export_visuals,
                                         export_collisions=export_collisions,
                                         create_pdf=create_pdf,
                                         ros_pkg=ros_pkg, export_with_ros_pathes=export_with_ros_pathes,
                                         ros_pkg_name=ros_pkg_name, formats=formats)

    def export_xml_with_meshes(self, output_dir=None, export_visuals=True, export_collisions=True,
                   create_pdf=False, ros_pkg=False, export_with_ros_pathes=None, ros_pkg_name=None,
                   export_joint_limits=True, export_submodels=True, formats=["urdf"], filename=None, float_fmt_dict=None):
        # [TODO pre_v2.0.0] export meshes
        self.export_xml(output_dir=output_dir, export_visuals=export_visuals, export_collisions=export_collisions,
                   create_pdf=create_pdf, ros_pkg=ros_pkg, export_with_ros_pathes=export_with_ros_pathes, ros_pkg_name=ros_pkg_name,
                   export_joint_limits=export_joint_limits, export_submodels=export_submodels, formats=formats, filename=filename, float_fmt_dict=float_fmt_dict)

    def export_smurf(self, outputdir=None, outputfile=None, export_visuals=True, export_collisions=True, create_pdf=False,
                     ros_pkg=False, export_with_ros_pathes=None, ros_pkg_name=None,
                     export_joint_limits=True, export_submodels=True, formats=["urdf"], filename=None, float_fmt_dict=None):
        """ Export self and all annotations inside a given folder with structure
        """
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
            if not os.path.exists(submech_dir):
                os.makedirs(submech_dir)

        # First, export the urdf
        robotfile = os.path.join(outputdir, "urdf/{}.urdf".format(self.name))
        if not os.path.exists(os.path.dirname(robotfile)):
            os.mkdir(os.path.dirname(robotfile))
        self.export_xml_with_meshes(output_dir=outputdir, export_visuals=export_visuals, export_collisions=export_collisions,
                                    create_pdf=create_pdf, ros_pkg=ros_pkg, export_with_ros_pathes=export_with_ros_pathes,
                                    ros_pkg_name=ros_pkg_name, export_joint_limits=export_joint_limits,
                                    export_submodels=export_submodels, formats=formats, filename=filename, float_fmt_dict=float_fmt_dict)
        # Export the smurf files
        smurf_dir = os.path.join(outputdir, "smurf")
        if not os.path.exists(smurf_dir):
            os.mkdir(smurf_dir)
        # Export attr
        smurf_annotations = [
            'motors', 'sensors', 'materials', "joints", "links", 'collisions', 'poses',
            "submechanisms", "exoskeletons", "interfaces"
        ]
        export_files = [os.path.relpath(robotfile, outputdir + "/smurf")]
        submechanisms = {}
        if self.autogenerate_submechanisms is None or self.autogenerate_submechanisms is True:
            self.generate_submechanisms()
        if (self.submechanisms is not None and len(self.submechanisms)) > 0 or (self.exoskeletons is not None and len(self.exoskeletons)):
            missing_joints = self._get_joints_not_included_in_submechanisms()
            if len(missing_joints) != 0:
                log.warning(f"Not all joints defined in the submechanisms definition! Lacking definition for:\n{missing_joints}")
            double_joints = self._get_joints_included_twice_in_submechanisms()
            if len(double_joints) != 0:
                log.error(f"The following joints are multiply defined in the submechanisms definition: \n{double_joints}")
                raise AssertionError
        for sm in self.submechanisms + self.exoskeletons:
            if hasattr(sm, "file_path"):
                _submodel = self.define_submodel(name="#sub_mech#", start=sm.get_root(self),
                                                 stop=sm.get_leaves(self), robotname=str(sm),
                                                 no_submechanisms=True, include_unstopped_branches=False)
                sm.file_path = "../submechanisms/" + os.path.basename(sm.file_path)
                if not os.path.isfile(sm.file_path):
                    self.export_submodel(name="#sub_mech#", output_dir=os.path.join(outputdir, "submechanisms"),
                                         filename=os.path.basename(sm.file_path), only_urdf=True, no_format_dir=True)
                else:
                    log.warning(f"File {sm.file_path} does already exist. Not exported submechanism urdf.")
                self.remove_submodel(name="#sub_mech#")
        for annotation in smurf_annotations:
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
            if k not in smurf_annotations:
                with open(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k)), "w+") as stream:
                    stream.write(dump_json({k: v}, default_style=False))
                    export_files.append(os.path.split(stream.name)[-1])

        for k, v in self.named_annotations.items():
            # if os.path.isfile(os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k))):
            #     raise NameError("You can't overwrite the already existing SMURF-Annotation-File " +
            #                     os.path.join(smurf_dir, "{}_{}.yml".format(self.name, k)) +
            #                     "\nPlease choose another name for you annotation")
            # else:
            if len(v) > 0:
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

        with open(os.path.join(smurf_dir, "{}.smurf".format(self.name)), "w+") as stream:
            stream.write(dump_json(annotation_dict, default_style=False, sort_keys=True))
        log.info(f"SMURF written to {smurf_dir}")

    def export_joint_limits(self, outputdir, file_name="joint_limits.yml", names=None):
        if names is None:
            names = [j.name for j in self.joints if j.joint_type != "fixed"]
        out = get_joint_info_dict(self, list(set(names)))
        if not os.path.exists(outputdir):
            os.makedirs(outputdir)
        out = {"limits": out}
        with open(os.path.join(outputdir, file_name), "w") as f:
            f.write(dump_json(out))
            #f.write("limits:\n")
            #f.write("  names: " + dump_yaml(out["names"], default_flow_style=True) + "\n")
            #f.write("  elements: " + dump_yaml(out["elements"], default_flow_style=True))

        jointnames_independent = []
        jointnames_active = []
        for sm in self.submechanisms + self.exoskeletons:
            if hasattr(sm, outputdir):
                jointnames_independent += sm.jointnames_independent
                jointnames_active += sm.jointnames_active

    def export_kccd(self, robot_export_dir, rel_iv_meshes_path, output_mesh_format, join_before_convexhull=True,
                    keep_stls=False, keep_urdf=False, dirname="kccd",
                    reduce_meshes=0, edit_collisions=None, **kwargs):
        if edit_collisions is None:
            edit_collisions = {}
        kccd_meshes = os.path.join(robot_export_dir, rel_iv_meshes_path)
        kccd_path = os.path.join(robot_export_dir, dirname)
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
        pgu.generate_kccd_optimizer_ready_collision(kccd_robot, [link.name for link in kccd_robot.links],
                                                    outputdir=kccd_meshes,
                                                    join_first=join_before_convexhull,
                                                    merge_additionally=kwargs["merge_additionally"]
                                                    if "merge_additionally" in kwargs.keys() else None,
                                                    mars_meshes=output_mesh_format.lower() == "mars_obj",
                                                    reduce_meshes=reduce_meshes)

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
        kccd_robot.export_urdf(outputfile=kccd_urdf, create_pdf=False)
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
                    subtree = kccd_robot.define_submodel(name="kccd_subtree", start=link, stop=None, only_return=True)
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

    def export_floatingbase(self, outputdir, ros_pkg_name=None, export_with_ros_pathes=False,
                            create_pdf=False, formats=["urdf"], float_fmt_dict=None):
        floatingbase = self.add_floating_base()
        floatingbase.full_export(outputdir, create_pdf=create_pdf, export_with_ros_pathes=export_with_ros_pathes,
                                 ros_pkg_name=ros_pkg_name, export_joint_limits=False, formats=formats, float_fmt_dict=float_fmt_dict)

    def export_submodel(self, name, output_dir=None, filename=None, export_visuals=True, export_collisions=True,
                        robotname=None, create_pdf=False, ros_pkg=False, export_with_ros_pathes=None,
                        ros_pkg_name=None, export_joint_limits=True, only_urdf=None, formats=["urdf"],
                        float_fmt_dict=None, no_format_dir=True):
        """
        Export the submodel to the given output.
        :param filename: filename of the urdf if only_urdf is true and name relates to a single submodel and is not a
        list
        :param export_with_ros_pathes:
        :param ros_pkg:
        :param name: name of the submodel we want to export
        :param output_dir: Path to the directory where all the submodels will be safed, each submodel gets a subfolder
        with its name
        :param export_visuals: whether we want to export visuals
        :param export_collisions: whether we want to export collisons
        :param robotname: the name of the exported robot in urdf
        :param create_pdf: whether we want to create a pdf to this model
        :param only_urdf: whether we only export the urdf
        :return: None
        """
        os.makedirs(output_dir, exist_ok=True)
        if isinstance(name, list):
            for n in name:
                self.export_submodel(n, output_dir=output_dir, export_visuals=export_visuals,
                                     export_collisions=export_collisions, create_pdf=create_pdf,
                                     only_urdf=only_urdf, ros_pkg=ros_pkg, ros_pkg_name=ros_pkg_name,
                                     export_with_ros_pathes=export_with_ros_pathes,
                                     export_joint_limits=export_joint_limits, formats=formats,
                                     float_fmt_dict=float_fmt_dict, no_format_dir=no_format_dir)
            return
        for format in formats:
            format = format.lower()
            if only_urdf is None and "only_urdf" in self._submodels[name].keys():
                only_urdf = self._submodels[name]["only_urdf"]
            elif only_urdf is None:
                only_urdf = True if name.startswith("#sub_mech#") else False

            if name in self._submodels.keys():
                _submodel = self.instantiate_submodel(name)
                assert _submodel.autogenerate_submechanisms == self.autogenerate_submechanisms
                _sm_xmlfile = filename if filename is not None else (f"{name}.{format}")
                submodel_dir = os.path.join(output_dir, name)
                if robotname is not None:
                    _submodel.name = robotname
                if only_urdf:
                    _submodel.export_xml(output_dir=output_dir, filename=_sm_xmlfile, export_visuals=export_visuals,
                                         export_collisions=export_collisions, create_pdf=create_pdf, ros_pkg=ros_pkg,
                                         export_with_ros_pathes=export_with_ros_pathes, formats=formats,
                                         float_fmt_dict=float_fmt_dict, no_format_dir=no_format_dir)

                else:
                    os.makedirs(submodel_dir, exist_ok=True)
                    _submodel.full_export(output_dir=submodel_dir, filename=_sm_xmlfile, export_visuals=export_visuals,
                                          export_collisions=export_collisions, create_pdf=create_pdf,
                                          ros_pkg=ros_pkg, export_with_ros_pathes=export_with_ros_pathes,
                                          ros_pkg_name=ros_pkg_name, export_joint_limits=export_joint_limits,
                                          export_submodels=False, formats=formats, float_fmt_dict=float_fmt_dict)
            else:
                log.warning(f"No submodel named {name}")

    def full_export(self, output_dir=None, export_visuals=True, export_collisions=True,
                    create_pdf=False, ros_pkg=False, export_with_ros_pathes=None, ros_pkg_name=None,
                    export_joint_limits=True, export_submodels=True, formats=["urdf"], filename=None, float_fmt_dict=None):
        self.export_smurf(outputdir=output_dir, export_visuals=export_visuals, export_collisions=export_collisions, create_pdf=create_pdf, ros_pkg=ros_pkg, export_with_ros_pathes=export_with_ros_pathes,
                          ros_pkg_name=ros_pkg_name, export_joint_limits=export_joint_limits, export_submodels=export_submodels, formats=formats,
                          filename=filename, float_fmt_dict=float_fmt_dict)

    def export_pdf(self, outputfile):
        SUBMECH_COLORS = ["cyan", "darkslateblue", "steelblue", "indigo", "darkblue", "royalblue", "lightskyblue",
                          "teal", "blue", "dodgerblue", "paleturquoise", "lightcyan", "mediumslateblue"]
        EXOSKEL_COLORS = ["lawngreen", "green", "darkgreen", "seagreen", "lightseagreen", "mediumspringgreen",
                          "palegreen", "olive"]

        def add_joint(joint):
            _out = f"\"{joint.parent}\" -> \"{joint.name}\" [label="
            _out += f"\"xyz: {joint.origin.xyz[0]} {joint.origin.xyz[1]} {joint.origin.xyz[2]} "
            _out += f"\\nrpy: {joint.origin.rpy[0]} {joint.origin.rpy[1]} {joint.origin.rpy[2]} "
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
                assert len(sm.get_joints()) == len(set(sm.get_joints()))
                for joint in sorted(sm.get_joints()):
                    joint = self.get_joint(joint)
                    assert str(joint) not in printed_joints, str(printed_joints)
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

        with open(outputfile + ".gv", "w") as f:
            f.write(out)

        graph = pydot.graph_from_dot_data(out)
        graph[0].write_pdf(outputfile)

    # getters
    def get_submodel(self, name):
        """ Return the submodel with the given name.
        """
        if name in self._submodels.keys():
            return self._submodels[name]
        else:
            log.warning("No submodel named {}".format(name))
        return

    # tools
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

    def reparent_link(self, link_name, parent, inertia=True, visual=True, collision=True):
        """
        Reparent all xml-children ( inertia, visual and collision ) of the given link onto the new parent.
        :param link_name: the link we apply this to
        :param parent: the new parent
        :param inertia: whether we do this for the inertias
        :param visual: whether we do this for the visuals
        :param collision: whether we do this for the collisions
        :return: None
        """
        if isinstance(link_name, list):
            if isinstance(parent, list):
                assert len(link_name) == len(parent)
                for link_, parent_ in zip(link_name, parent):
                    self.reparent_link(link_, parent_, inertia=inertia, visual=visual, collision=collision)
                return
            for link_ in link_name:
                self.reparent_link(link_, parent, inertia=inertia, visual=visual, collision=collision)
            return

        link = self.get_link(link_name)
        parent = self.get_link(parent)

        if not link or not parent:
            log.warning("Link or new parent not found!")
            return

        # Get the transformation
        # root to link
        L_T_R = self.get_transformation(link.name)
        R_T_P = inv(self.get_transformation(parent.name))

        L_T_P = R_T_P.dot(L_T_R)

        if inertia and link.inertial:
            inertia_L = link.inertial
            if parent.inertial:
                # Merge the inertials
                # Old one
                I_L = inertia_L.to_mass_matrix()
                IP_T_IL = parent.inertial.origin.to_matrix().dot(
                    L_T_P.dot(inertia_L.origin.to_matrix()))
                Ad = get_adjoint(IP_T_IL)
                # Transform into parent
                I_NL = Ad.dot(I_L.dot(Ad.T)) + parent.inertial.to_mass_matrix()
                parent.inertial = representation.Inertial.from_mass_matrix(I_NL, parent.inertial.origin)

            else:
                # Set inertial to new parent
                new_origin = L_T_P.dot(inertia_L.origin.to_matrix())
                parent.inertial = link.inertial
                parent.inertial.origin = representation.Pose.from_matrix(new_origin)

            # Set link to near zero
            link.inertial = representation.Inertial.from_mass_matrix(1e-5 * np.ones((6, 6)), link.inertial.origin)

        if visual and link.visuals:
            for vis in link.visuals:
                VL_T_L = vis.origin.to_matrix()
                new_origin = L_T_P.dot(VL_T_L)
                vis.origin = representation.Pose.from_matrix(new_origin)
                parent.add_aggregate('visual', vis.duplicate())
                link.remove_aggregate(vis)

        if collision and link.collisions:
            for col in link.collisions:
                CL_T_L = col.origin.to_matrix()
                new_origin = L_T_P.dot(CL_T_L)
                col.origin = representation.Pose.from_matrix(new_origin)
                parent.add_aggregate('collision', col.duplicate())
                link.remove_aggregate(col)

        # Reinit the link
        # link.to_xml()
        # parent.to_xml()
        self.relink_entities()
        return

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
        joint.origin = representation.Pose.from_matrix(inv(T0_newp).dot(T0_old))
        joint.parent = new_parent_name

    def define_submodel(self, name, start, stop=None, robotname=None, only_urdf=False, only_return=False,
                        overwrite=False, no_submechanisms=False, include_unstopped_branches=True):
        """Defines a submodel from a given starting link.
        If stop is provided than the chain from start to stop is used.
        """
        assert stop is None or type(stop) == list
        assert type(start) == str
        definition = {
            "name": name,
            "robotname": robotname,
            "start": start,
            "stop": stop,
            "only_urdf": only_urdf,
            "include_unstopped_branches": include_unstopped_branches
        }
        if only_return:
            return self.instantiate_submodel(definition=definition, no_submechanisms=no_submechanisms,
                                             include_unstopped_branches=include_unstopped_branches)
        if name in self._submodels.keys() and not overwrite:
            raise NameError("A submodel with the given name is already defined")
        else:
            self._submodels[name] = definition
        return self.instantiate_submodel(name, no_submechanisms=no_submechanisms,
                                         include_unstopped_branches=include_unstopped_branches)

    def get_links_and_joints_in_subtree(self, start, stop=None, include_unstopped_branches=True, extend_by_single_fixed=False):
        assert self.get_link(start) is not None
        if stop is None:
            # Collect all links on the way to the leaves
            parents, children = self._get_children_lists([start], [])
            parentset = set(parents)
            childrenset = set(children)
            linknames = list(parentset.union(childrenset))
        else:
            linknames = set()
            _stop = set(stop)
            if include_unstopped_branches:
                _stop = _stop | set(self.get_leaves(start))
            for leave in _stop:
                # print(start, leave, self.get_chain(start, leave, joints=False))
                linknames = linknames | set(self.get_chain(start, leave, joints=False))
            # linknames = set()
            # try:
            #     chains = [[str(link) for link in self.get_chain(self.get_root(), leave, joints=False)] for leave
            #                         in self.get_leaves()]
            #     chains = [chain for chain in chains if str(start) in chain]
            #     for chain in chains:
            #         begin = chain.index(str(start))
            #         end = None
            #         for leave in stop:
            #             if str(leave) in chain:
            #                 assert end is None, f"The leave {chain[end]} and {str(leave)} are on the same branch."
            #                 end = chain.index(str(leave))
            #                 while extend_by_single_fixed and end+1 < len(chain) and\
            #                     self.get_joint(self.get_parent(chain[end+1])).joint_type == "fixed" and \
            #                     len(self.get_children(chain[end])) == 1:
            #                     end += 1
            #         if end is not None:
            #             linknames.update(chain[begin:end+1])
            #         elif include_unstopped_branches:
            #             linknames.update(chain[begin:])
            # except Exception as e:
            #     log.info(self.get_root())
            #     log.info(self.parent_map.keys())
            #     log.info([link.name for link in self.links])
            #     log.info(f"Start {start} Stop {stop}")
            #     raise e
            linknames = list(linknames)

        jointnames = [str(j) for j in self.get_joint(self.get_parent(linknames)) if j is not None and j.parent in linknames]
        assert len(linknames) == len(jointnames) + 1, f"n_links={len(linknames)} - 1 != n_joints={len(jointnames)}\n{start}\t{stop}\n{linknames}\n{jointnames}"
        #print(f"n_links={len(linknames)} - 1 != n_joints={len(jointnames)}\n{start}\t{stop}\n{linknames}\n{jointnames}")

        return linknames, jointnames

    def instantiate_submodel(self, name=None, definition=None,
                             include_unstopped_branches=None, extend_by_single_fixed=False,
                             no_submechanisms=False):
        """
        Instantiates a submodel by it's definition. Takes either name or definition. If both are given, the submodel
        definition with the given name will be updated including renaming it
        :param name: name of the already defined submodel
        :param definition: definition of a submodel
        :return: the submodel
        """
        assert name is not None or definition is not None
        if name is not None and definition is None:
            definition = self._submodels[name]
            if include_unstopped_branches is None:
                include_unstopped_branches = definition["include_unstopped_branches"]
        elif name is not None and definition is not None:
            assert definition["name"] not in self._submodels.keys()
            if name != definition["name"]:
                self.remove_submodel(name)
            self.define_submodel(**definition)
        elif name is None and definition is None:
            raise AssertionError("No args given!")

        if "stop" not in definition.keys():
            definition["stop"] = None
        assert all([x in definition.keys() for x in ["name", "start", "stop"]])
        if "robotname" not in definition.keys() or definition["robotname"] is None:
            definition["robotname"] = definition["name"]

        submodel = type(self)(name=definition["robotname"])
        if include_unstopped_branches is None:
            include_unstopped_branches=True

        linknames, jointnames = self.get_links_and_joints_in_subtree(
            start=definition["start"], stop=definition["stop"],
            include_unstopped_branches=include_unstopped_branches, extend_by_single_fixed=extend_by_single_fixed)

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
                log.warning(f"Removing mimic relation in submodel {definition['robotname']} for {_joint.name} (mimiced "
                            f"{_joint.mimic.joint})!")
                _joint.joint_dependencies = [jd for jd in _joint.joint_dependencies if jd.joint in jointnames]
                joints.append(_joint)
            else:
                joints.append(joint.duplicate())

        submodel.add_aggregate("joints", joints)
        assert all([j.mimic is None or str(j.mimic.joint) in jointnames for j in submodel.joints])

        motors = []
        for joint in joints:
            if joint.motor is not None:
                motors.append(joint._motor.duplicate() if type(joint._motor) != str else self.get_motor(joint._motor))
        submodel.add_motor(motors)

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
                _exo._jointnames = None
                if _exo.is_related_to(joints, pure=True):
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

        submodel.relink_entities()

        return submodel

    def remove_submodel(self, name):
        """Remove the submodel with the given name"""
        self._submodels.pop(name)

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
            self._submodels.update(
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
            # [TODO pre_v2.0.0] check if the I is basically zero and then recreate the inertial using the collision
            if link.inertial:
                M = self.get_inertial(link.name).to_mass_matrix()
                origin = link.inertial.origin
            else:
                M = np.zeros((6, 6))
                origin = representation.Pose.from_matrix(np.eye(4))
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

            link.inertial = representation.Inertial.from_mass_matrix(M, origin)

            log.info(" Corrected inertia for link {}".format(link.name))

    def correct_axes(self, joints=None, tol=1E-3):
        """
        Turns all joints where the axis are not unit. Not yet entirely tested
        """
        if joints is None:
            joints = [j for j in self.joints if j.joint_type != "fixed" and hasattr(j, "axis") and j.axis is not None]
        elif not isinstance(joints, list):
            joints = [joints]

        for joint in joints:
            joint.axis = joint.axis / np.linalg.norm(joint.axis)
            axis_correction = np.eye(4)
            if 1 - (np.amax(np.abs(joint.axis))) > tol:
                log.warning(f"Axis of joint {joint.name} is not even close to unit! No changes made Axis:{joint.axis}")
            elif joint.type != "fixed" and len(np.where(np.array(joint.axis) == 0.0)[0]) != 2:
                log.warning(f"Joint axis is not x, y or z unit vector:\n {joint.__dict__}")
                v = [np.abs(a) for a in joint.axis]
                new_axis = [0 if i != np.argmax(v) else 1 for i in range(3)]
                if joint.axis[np.argmax(v)] < 0:
                    new_axis *= -1
                R = scipy_rot.align_vectors([new_axis], [joint.axis])
                # joint.axis = new_axis
                axis_correction[:3, :3] = R[0].as_matrix()
                log.info(f"Rotating joint by \n {axis_correction}")
                log.info(f"New axis is: {new_axis}")

                joint.origin = representation.Pose.from_matrix(axis_correction.dot(joint.originto_matrix()))
                joint.axis = new_axis

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
                pjoint.origin = representation.Pose.from_matrix(T)
            else:
                pjoint.origin = representation.Pose.from_matrix(pjoint.origin.to_matrix().dot(T))
        if only_frame:
            for joint in cjoint:
                joint.origin = representation.Pose.from_matrix(Tinv.dot(joint.origin.to_matrix()))

            for ent in link.collisions + link.visuals:
                ent.origin = representation.Pose.from_matrix(Tinv.dot(ent.origin.to_matrix()))

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
        assert transform_object(inertial, T)
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
        visual = self.get_visual(linkname)
        log.info(" Transform Visuals")
        # Transform
        T = create_transformation(translation, rotation)
        assert transform_object(visual, T)
        return True

    def transform_collision(self, linkname, translation, rotation):
        """Transform the collision(s) of the given link.
        """
        # Get the collision
        collision = self.get_collision(linkname)
        log.info(" Transform Collisions")
        # Transform
        T = create_transformation(translation, rotation)
        assert transform_object(collision, T)
        return True

    def enforce_zero(self, xyz_tolerance=1E-4, rad_tolerance=1E-6, mass_tolerance=1E-4, i_tolerance=1E-12):
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
                    joint.limit.lower = 0.0 if np.abs(joint.limit.lower) < rad_tolerance else joint.limit.lower
                    joint.limit.upper = 0.0 if np.abs(joint.limit.upper) < rad_tolerance else joint.limit.upper
                    joint.limit.velocity = 0.0 if np.abs(joint.limit.velocity) < rad_tolerance else joint.limit.velocity
                    joint.limit.effort = 0.0 if np.abs(joint.limit.effort) < rad_tolerance else joint.limit.effort

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
                mesh = pgu.import_mesh(coll.geometry.filename, self.xmlfile)
                if mesh.is_volume:
                    vol = mesh.volume
                    T_com[0:3, 3] = mesh.center_mass
                else:
                    vol = mesh.convex_hull.volume
                    T_com[0:3, 3] = mesh.convex_hull.center_mass
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
                            joint.limit.lower = read_angle_2_rad(backup["min"])
                        if "max" in backup.keys() and limit_temp[0] == limit_temp[1]:
                            joint.limit.upper = read_angle_2_rad(backup["max"])
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
                    log.warning(f"Auto-Bitmask algorithm was unable to create the collision: {coll_names[i]} <-> {coll_names[j]}")
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
            for k, v in self._submodels.items():
                if target == v["start"]:
                    self._submodels[k]["start"] = new_name
                if target in v["stop"]:
                    self._submodels[k]["stop"] = [link if link != target else new_name for link in v]

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
            for coll in link.collisions:
                if not pgu.has_enough_vertices(coll, self.xmlfile):
                    log.warning(f"Mesh {coll.name} has not enough vertices. Removing geometry!")
                    pgu.remove_collision(self, link.name, collisionname=coll.name)
            for vis in link.visuals:
                if not pgu.has_enough_vertices(vis, self.xmlfile):
                    log.warning(f"Mesh {vis.name} has not enough vertices. Removing geometry!")
                    pgu.remove_visual(self, link.name, visualname=vis.name)

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

        other.relink_entities()

        if self.get_link_id(joint.parent) is None:
            raise Exception("Provide valid link to attach to. '" + joint.parent + "' is not in " + str(
                [ln.name for ln in self.links]))

        if not isinstance(joint, representation.Joint):
            raise Exception("Provide valid joint type.")

        # Check for naming and rename if necessary
        # [TODO pre_v2.0.0] attach the rest
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
        # pposes = set([m.name for m in self.poses])

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
        # cposes = set([m.name for m in other.poses])

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
                log.warning(f"Joint names are duplicates a _2 will be appended! {pjoints & cjoints}")
                renamed_entities.update(other.rename(targettype="joint", target=list(pjoints & cjoints), prefix=name_prefix, suffix=name_suffix))
                if joint.name in list(pjoints & cjoints):
                    joint.name = joint.name + "_2"
            else:
                raise NameError("There are duplicates in joint names", repr(pjoints & cjoints))

        if pcollisions & ccollisions:
            if not do_not_rename:
                log.warning(f"Collision names are duplicates a _2 will be appended! {pcollisions & ccollisions}")
                renamed_entities.update(
                    other.rename(targettype="collision", target=list(pcollisions & ccollisions), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in collision names", repr(pcollisions & ccollisions))

        if pvisuals & cvisuals:
            if not do_not_rename:
                log.warning(f"Visual names are duplicates a _2 will be appended! {pvisuals & cvisuals}")
                renamed_entities.update(
                    other.rename(targettype="visual", target=list(pvisuals & cvisuals), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in visual names", repr(pvisuals & cvisuals))

        for mat_name in pmaterials & cmaterials:
            if self.get_material(mat_name).equivalent(other.get_material(mat_name)):
                cmaterials.remove(mat_name)
        if pmaterials & cmaterials:
            if not do_not_rename:
                log.warning(f"Material names are duplicates a _2 will be appended! {pmaterials & cmaterials}")
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
                log.warning(f"Sensor names are duplicates a _2 will be appended! {conflicting_sensors}")
                renamed_entities.update(
                    other.rename(targettype="sensor", target=list(conflicting_sensors), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in sensor names", repr(conflicting_sensors))
        
        if ptransmissions & ctransmissions:
            if not do_not_rename:
                log.warning(f"Transmission names are duplicates a _2 will be appended! {ptransmissions & ctransmissions}")
                renamed_entities.update(
                    other.rename(targettype="transmission", target=list(ptransmissions & ctransmissions), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in transmission names", repr(ptransmissions & ctransmissions))
            
        if pmotors & cmotors:
            if not do_not_rename:
                log.warning(f"Motor names are duplicates a _2 will be appended! {pmotors & cmotors}")
                renamed_entities.update(
                    other.rename(targettype="motor", target=list(pmotors & cmotors), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in motor names", repr(pmotors & cmotors))
            
        if pexoskeletons & cexoskeletons:
            if not do_not_rename:
                log.warning(f"Exoskeleton names are duplicates a _2 will be appended! {pexoskeletons & cexoskeletons}")
                renamed_entities.update(
                    other.rename(targettype="exoskeleton", target=list(pexoskeletons & cexoskeletons), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in exoskeleton names", repr(pexoskeletons & cexoskeletons))
            
        if psubmechanisms & csubmechanisms:
            if not do_not_rename:
                log.warning(f"Submechanism names are duplicates a _2 will be appended! {psubmechanisms & csubmechanisms}")
                renamed_entities.update(
                    other.rename(targettype="submechanism", target=list(psubmechanisms & csubmechanisms), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in submechanism names", repr(psubmechanisms & csubmechanisms))

        if pinterfaces & cinterfaces:
            if not do_not_rename:
                log.warning(f"Interface names are duplicates a _2 will be appended! {pinterfaces & cinterfaces}")
                renamed_entities.update(
                    other.rename(targettype="interface", target=list(pinterfaces & cinterfaces), prefix=name_prefix, suffix=name_suffix))
            else:
                raise NameError("There are duplicates in interface names", repr(pinterfaces & cinterfaces))

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

        # [TODO pre_v2.0.0] check on poses
        # for cPose in other.poses:
        #     self.add_aggregate('pose', cPose)

        joint.link_with_robot(self)
        self.add_aggregate('joint', joint)
        assert joint.check_valid()
        assert str(other.get_root()) == str(joint.child)
        assert len(set([str(l) for l in self.links])) == len(self.links)
        assert len(self.joints) - n_joints == len(other.joints) + 1
        assert len(set([j.child for j in self.joints])) == len([j.child for j in self.joints])
        assert len(self.joints) == len(self.links) - 1
        assert len(self.joints) == len(self.get_joints_ordered_df()), f"{sorted([str(x) for x in self.joints])}\n{sorted([str(x) for x in self.get_joints_ordered_df()])}"
        assert self.get_root()

        # this will be done by fill_submechanisms we are delaying this to the export to give the user the opportunity to define something thereselves
        # # Add the connection joint to the submechanism tree
        # self.submechanisms += [Submechanism(self, name=joint.name, type="serial", contextual_name="ConnectionJoint",
        #                                     jointnames_independent=[] if joint.type == "fixed" else [joint.name],
        #                                     jointnames_spanningtree=[] if joint.type == "fixed" else [joint.name],
        #                                     jointnames_active=[] if joint.type == "fixed" else [joint.name],
        #                                     jointnames=[joint.name])]

        self.relink_entities()
        return renamed_entities

    def add_link_by_properties(self, name, translation, rotation, parent, jointname=None, jointtype="fixed", axis=None,
                               mass=0.0, add_default_motor=True, is_human=False):
        """
        Adds a link with the given parameters.
        This method has to be overridden in subclasses.
        :param name: the name of the link
        :param translation: the translation of the joint
        :param rotation: the rotation of the joint
        :param parent: the parent link to which this shall be attached
        :param jointname: the name for the joint to create (default is link name)
        :param jointtype: the joint type (default: fixed)
        :param axis: the axis if the joint is not fixed
        :param mass: the point mass we should add to this link
        :return: parent link and the created link and joint
        """
        if name in [link.name for link in self.links]:
            raise NameError("You can't add '" + name + "' as the model already contains a link with this name!")
        if jointname in [j.name for j in self.joints]:
            raise NameError("You can't add '" + jointname + "' as the model already contains a joint with this name!")
        if type(parent) is str:
            parent = self.get_link(parent)
        else:
            assert (isinstance(parent, representation.Link))
        if mass > 0.0:
            inertial = representation.Inertial(
                mass=mass, inertia=representation.Inertia(
                    ixx=1e-6,
                    iyy=1e-6,
                    izz=1e-6,
                ),
                origin=representation.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0])
            )
        else:
            inertial = None
        link = representation.Link(name, inertial=inertial, is_human=is_human)
        joint = representation.Joint(name=jointname if jointname is not None else name, parent=parent.name,
                                     child=link.name, joint_type=jointtype,
                                     origin=representation.Pose(translation, rotation), axis=axis)
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
                     target_urdf=None, target_smurf=None, only_return=False):
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
        robot = type(self)(self.name)
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

        # reflection matrix
        T_R = pgu.get_reflection_matrix(normal=np.array(mirror_plane))

        # copy kinematic
        def recursive_link_transform(link_, T_root_to_link):
            link_ = self.get_link(link_)
            T_link = self.get_transformation(link_.name)
            new_link = link_.duplicate()

            # transform link information to root and mirror

            if new_link.inertial is not None:
                T = T_R.dot(T_link.dot(link_.inertial.origin.to_matrix()))
                new_origin = representation.Pose.from_matrix(inv(T_root_to_link).dot(T))
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
                vis.origin = representation.Pose.from_matrix(inv(T_root_to_link).dot(T))
                if isinstance(vis.geometry, representation.Mesh) and not (
                        (os.path.basename(vis.geometry.filename).split(".")[0] in exclude_meshes or
                         "ALL" in exclude_meshes)):
                    pgu.mirror_geometry(vis, urdf_path=target_urdf,
                                        transform=inv(T_root_to_link.dot(vis.origin.to_matrix())).dot(T),
                                        name_replacements=name_replacements)

            for col in new_link.collisions:
                T = T_R.dot(T_link.dot(col.origin.to_matrix()))
                col.origin = representation.Pose.from_matrix(inv(T_root_to_link).dot(T))
                if isinstance(col.geometry, representation.Mesh) and not (
                        (os.path.basename(col.geometry.filename).split(".")[0] in exclude_meshes or
                         "ALL" in exclude_meshes)):
                    pgu.mirror_geometry(col, target_urdf,
                                        transform=inv(T_root_to_link.dot(col.origin.to_matrix())).dot(T),
                                        name_replacements=name_replacements)

            robot.add_aggregate("link", new_link)

            # joints:
            for jointname in self.get_children(link_.name):
                joint_ = self.get_joint(jointname)
                new_joint = joint_.duplicate()

                T_flip = np.eye(4)

                # All frames are mirrored using least-maintain-axis as flip axis
                T_flip[flip_axis, flip_axis] *= -1
                # now we transform the local coordinate system using t_flip to make it right handed
                new_root_to_joint = T_R.dot(T_link.dot(joint_.origin.to_matrix().dot(T_flip)))
                new_joint.origin = representation.Pose.from_matrix(inv(T_root_to_link).dot(new_root_to_joint))
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
        self.link_entities()

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
                sensor.origin = representation.Pose.from_matrix(transform.inv(T_root2link).dot(T))

        if target_smurf is not None:
            robot.smurffile = target_smurf

        if only_return:
            return robot
        for k, v in robot.__dict__.items():
            setattr(self, k, v)
        self.relink_entities()

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
        before = self.instantiate_submodel(definition={
            "name": self.name,
            "start": self.get_root(),
            "stop": links_to_cut
        })
        beyond = {new_root: self.instantiate_submodel(definition={
            "name": self.name,
            "start": new_root
        }) for new_root in links_to_cut}
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
        for geo in link.visuals + link.collisions:
            _geo_scale = inv(geo.origin.to_matrix())[:3, :3].dot(scale)
            geo.geometry.scale_geometry(x=_geo_scale[0], y=_geo_scale[1], z=_geo_scale[2])
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


