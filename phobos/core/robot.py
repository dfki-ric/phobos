import datetime
import sys
import os
import xml
import xml.etree.ElementTree as ET

import pkg_resources
import yaml
from copy import deepcopy
from scipy.spatial.transform import Rotation as scipy_rot
import numpy as np

from .. import geometry as pgu
from ..io import representation
from ..io.parser import parse_xml
from ..utils.all import *
from ..utils import misc, urdf, tree, transform


class Robot(representation.Robot):
    def __init__(self, name=None, xmlfile=None, verify_meshes_on_import=True):
        """ The basic robot class to represent a urdf.
        """
        super().__init__(name=name)

        self.xmlfile = xmlfile
        self._submodels = {}

        if xmlfile is not None:
            # Adding name tags for visuals and collisions and rename robot
            try:
                etree = ET.parse(xmlfile)
            except ET.ParseError:
                print("Reading of URDF/SDF file", xmlfile, "failed!")
                raise
            root = etree.getroot()
            if name is not None:
                root.attrib["name"] = name

            def go_through_children(node):
                """Recursive function to name children of link class
                """
                for child in node:
                    if child.tag == "link":
                        go_through_children(child)
                    elif child.tag in ["visual", "collision"]:
                        if "name" not in child.attrib.keys() or child.attrib["name"] == "":
                            child.attrib.update({'name': child.tag + "_" + node.attrib["name"]})

            go_through_children(root)
            # Overwrite dictionary with urdf dict
            xmlfile = os.path.abspath(xmlfile)
            if os.path.splitext(xmlfile)[-1] in ['.urdf']:
                # print("Loading URDF", xmlfile, flush=True)
                robot = parse_xml(root)
                self.__dict__.update(robot.__dict__)
                self.xmlfile = xmlfile
            elif os.path.splitext(xmlfile)[-1] in ['.yml']:
                raise NotImplementedError("Import from YAML not supported!")
            elif os.path.splitext(xmlfile)[-1] in ['.sdf']:
                # print("Loading SDF", xmlfile, flush=True)
                robot = parse_xml(root)
                self.__dict__.update(robot.__dict__)
                self.xmlfile = xmlfile
            else:
                raise NotImplementedError("The format of ", xmlfile, " is not supported!")

        if verify_meshes_on_import:
            self.verify_meshes()

        if self.xmlfile is not None:
            self.joints = self.get_joints_ordered_df()

    # helper methods
    @classmethod
    def get_robot_from_dict(name='', objectlist=[]):
        """
        Uses blender workflow to access internal dictionary to call robot
        representation. Idea is to use cli methods and formats for imports and
        exports
        """
        import bpy
        import phobos.blender.utils.blender as bUtils
        import phobos.blender.utils.selection as sUtils
        import phobos.blender.utils.naming as nUtils
        import phobos.blender.utils.io as ioUtils

        root = sUtils.getRoot(bpy.context.selected_objects[0])
        blender_model = derive_model_dictionary(root, name, objectlist)
        cli_joints = []
        for key, values in blender_model['joints'].items():
            cli_axis = None
            cli_limit = None
            if not values['type'] == 'fixed':
                cli_axis = values['axis']
                cli_limit = values['limits']
            cli_joints.append(representation.Joint(
                name=values['name'],
                parent=values['parent'],
                child=values['child'],
                type=values['type'],
                axis=cli_axis,
                origin=transform.to_origin(np.array(values['pose']['rawmatrix'])),
                limit=cli_limit,
                dynamics=None,  # optional
                safety_controller=None,  # optional
                calibration=None,  # optional
                mimic=None))  # rudimentär

        cli_links = []
        for key, values in blender_model['links'].items():
            # print(key)
            # print(values)
            cli_links.append(representation.Link(
                name=values['name'],
                visuals=None,
                inertial=None,
                collisions=values['collision']))

        cli_robot = representation.Robot(
            name=blender_model['name'],
            version=blender_model['version'],
            links=cli_links,
            joints=cli_joints,
            materials=blender_model['materials'])

        print("success")
        new_robot = Robot()
        new_robot.__dict__.update(cli_robot.__dict__)
        # so geht das auch mit der SmURF klasse.
        return new_robot

    def get_joints_ordered_df(self):
        """Returns the joints in depth first order"""
        return tree.get_joints_depth_first(self, self.get_root())

    def get_links_ordered_df(self):
        """Returns the joints in depth first order"""
        joints = self.get_joints_ordered_df()
        out = [self.get_root()] + [jn.child for jn in joints]
        return [self.get_link(ln) for ln in out]

    def get_joint_level(self, jointname):
        joint = self.get_joint(jointname)
        return self.get_link_level(joint.parent)

    def get_link_level(self, linkname):
        parent = self.get_parent(linkname)
        level = 0
        while parent is not None:
            joint = self.get_joint(parent[0])
            parent = self.get_parent(joint.parent)
            level += 1
        return level

    def _get_children_lists(self, parentlist, childrenlist, targettype='link'):
        """
        Used recursively. Returns a list of all links that can be considered parents/children for the given parent list
        :param parentlist:
        :param childrenlist:
        :param targettype:
        :return:
        """
        childs = []
        for parent in parentlist:
            childs += self.get_children(parent, targettype=targettype)

        if len(childs) > 0:
            newparents, newchildren = self._get_children_lists(childs, [])
            return parentlist + newparents, childrenlist + childs + newchildren
        else:
            return [], []

    def _rename(self, targettype, target, new_name):
        """
        This method will be called when an object is renamed and should be overridden in subclasses
        """
        renamed_entities = {}
        return renamed_entities

    def _attach_part(self, targettype, target):
        """
        Safely attach a new object to the robot. Checks if the name is already given.
        :param targettype: type of the part ot attach
        :param target: what to attach
        :return:
        """
        # Collect all instances which have the same name
        instances = []
        # Original list
        objects = getattr(self, targettype)
        for obj in objects:
            if target.name in obj.name:
                instances.append(obj.name)
        if instances:
            target.name += "_{}".format(len(instances))
            print("Renamed object to {}".format(target.name))
        objects += [target]
        setattr(self, targettype, objects)
        return

    def copy_related_annotations(self, source):
        """
        Copies aplicable/relevant further annotations e.g. such that consider more than one joint/link/material from
        the source to this Robot instance.
        :param source:
        :return:
        """
        pass

    # export methods
    def export_urdf(self, outputfile=None, export_visuals=True, export_collisions=True, create_pdf=False,
                    ros_pkg=False, export_with_ros_pathes=None):
        """Export the mechanism to the given output file.
        If export_visuals is set to True, all visuals will be exported. Otherwise no visuals get exported.
        If export_collisions is set to to True, all collisions will be exported. Otherwise no collision get exported.
        """
        self.joints = self.get_joints_ordered_df()
        if not outputfile:
            outputfile = self.name

        outputfile = os.path.abspath(outputfile)

        export_robot = deepcopy(self)
        if not export_visuals:
            export_robot.remove_visuals()
        if not export_collisions:
            export_robot.remove_collisions()

        xml_string = self.to_urdf_string()

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
            create_pdf_from_urdf(outputfile)
        print("Robot written to {}".format(outputfile))
        return

    def full_export(self, output_dir=None, export_visuals=True, export_collisions=True,
                    create_pdf=False, ros_pkg=False, export_with_ros_pathes=None, ros_pkg_name=None,
                    export_joint_limits=True, export_submodels=True):
        """ Exports all model information stored inside this instance.
        """

        # Main model
        model_file = os.path.join(output_dir, "urdf/{}.urdf".format(self.name))
        if not os.path.exists(os.path.dirname(model_file)):
            os.makedirs(os.path.dirname(model_file))
        if ros_pkg_name is None and (export_with_ros_pathes or ros_pkg):
            ros_pkg_name = os.path.basename(output_dir)
        self.xmlfile = model_file
        self.export_urdf(outputfile=model_file, create_pdf=create_pdf, ros_pkg=ros_pkg,
                         export_with_ros_pathes=export_with_ros_pathes)

        if export_joint_limits:
            self.export_joint_limits(os.path.join(output_dir, "urdf"))

        # ToDo ensure that this is done elsewhere
        # elif generate_serial_submechs:
        #     if not os.path.exists(os.path.join(output_dir, "submechanisms")):
        #         os.mkdir(os.path.join(output_dir, "submechanisms"))
        #     submechanisms_general = self.generate_serial_submechanisms()
        #     with open(os.path.join(output_dir, "submechanisms", submechanisms_file), "w") as f:
        #         f.write(yaml.safe_dump(submechanisms_general))
        #         print("Submechanisms written to", os.path.join(output_dir, "submechanisms", submechanisms_file))

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
                                     ros_pkg_name=ros_pkg_name)

    def export_joint_limits(self, outputdir, file_name="joint_limits.yml", names=None):
        if names is None:
            names = [j.name for j in self.joints if j.type != "fixed"]
        out = get_joint_info_dict(self, list(set(names)))
        if not os.path.exists(outputdir):
            os.makedirs(outputdir)
        with open(os.path.join(outputdir, file_name), "w") as f:
            f.write("limits:\n")
            f.write("  names: " + yaml.safe_dump(out["names"], default_flow_style=True) + "\n")
            f.write("  elements: " + yaml.safe_dump(out["elements"], default_flow_style=True))

    def export_yaml(self, output_dir=None):
        """ Export the robot data into yaml file in such a way that it can be used to check the model.
        WARNING: export_yaml is currently not maintained! (todo)
        """
        print("WARNING: export_yaml is currently not maintained!")
        # Model statistics
        vis_no = 0
        col_no = 0
        rev_no = 0
        pris_no = 0
        fix_no = 0
        other_no = 0

        if not output_dir:
            output_dir = os.path.join(os.getcwd(), "yaml")
            if not os.path.exists(output_dir):
                os.mkdir(output_dir)

        output_data = {"links": {}, "joints": {}, "parent_map": {}, "child_map": {}}

        print("Dumping model information of {0} to {1}".format(self.name, output_dir))

        print(" Parsing links...")
        for link in self.links:
            # Get global position and orientation
            global_transform = self.global_origin(link.name).to_yaml()

            # Append to link data in output data
            current_data = link.to_yaml()
            current_data["global_transformations"] = global_transform
            output_data["links"].update(
                {link.name: current_data}
            )

            vis_no += len(link.visuals)
            col_no += len(link.collisions)

        print(" Parsing joints...")
        for j in self.joints:
            if j.type == 'revolute':
                rev_no += 1
            elif j.type == 'prismatic':
                pris_no += 1
            elif j.type == 'fixed':
                fix_no += 1
            else:
                other_no += 1

            output_data["joints"].update(
                {j.name: j.to_yaml()}
            )

        print(" Parse child map...")
        for k, v in self.child_map.items():
            current_children = []
            for vi in v:
                current_children += [{'joint': vi[0], 'link': vi[1]}]

            output_data["child_map"].update(
                {k: current_children}
            )

        print(" Parse parent map...")
        for k, v in self.parent_map.items():
            output_data["parent_map"].update(
                {k: {'joint': v[0], 'link': v[1]}}
            )

        noalias_dumper = yaml.dumper.SafeDumper
        noalias_dumper.ignore_aliases = lambda inst, data: True

        export_files = []
        for k, v in output_data.items():
            outputfile = os.path.join(output_dir, self.name + "_" + k + ".yml")
            print(" Writing {0} into {1}".format(k, outputfile))
            export_files += [outputfile]
            with open(outputfile, 'w') as outfile:
                yaml.dump(v, outfile, default_flow_style=False, Dumper=noalias_dumper)

        # Write mother yml file
        outputfile = os.path.join(output_dir, self.name + ".yml")

        annotation_dict = {
            'modelname': self.name,
            'date': datetime.datetime.now().strftime("%Y%m%d_%H:%M"),
            'files': export_files,
            'modelinformation': {
                'directory': os.path.dirname(self.xmlfile),
                'root': self.get_root(),
                'mass': self.compute_mass(),
                'joints': {'total': len(self.joints), 'revolute': rev_no, 'prismatic': pris_no, 'fixed': fix_no,
                           'other': other_no},
                'links': len(self.links),
                'visuals': vis_no,
                'collisions': col_no,
                'submodels': self._submodels,
                'submechanisms': self._submechanisms,
            }
        }

        with open(outputfile, 'w') as outfile:
            yaml.dump(annotation_dict, outfile, default_flow_style=False, Dumper=noalias_dumper)

        print("Finished")
        return

    def export_kccd(self, robot_export_dir, rel_iv_meshes_path, output_mesh_format, join_before_convexhull=True,
                    keep_stls=False, keep_urdf=False, dirname="kccd",
                    reduce_meshes=0, edit_collisions=None, **kwargs):
        if edit_collisions is None:
            edit_collisions = {}
        kccd_meshes = os.path.join(robot_export_dir, rel_iv_meshes_path)
        kccd_path = os.path.join(robot_export_dir, dirname)
        kccd_urdf = os.path.join(kccd_path, self.name + ".urdf")
        misc.create_dir(None, kccd_path)
        kccd_robot = deepcopy(self)
        kccd_robot.xmlfile = kccd_urdf
        if "remove_joints" in kwargs.keys():
            for j in kwargs["remove_joints"]:
                # print("Removing joint for kccd:", j)
                kccd_robot.remove_joint(j)
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
        misc.execute_shell_command("urdf2kccd -b " + self.name + ".urdf", cwd=kccd_path)
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
                        "axis": yaml.safe_load(first_line[3]),
                        "limits": yaml.safe_load(first_line[4]),
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
                    print(temp["mesh"], "->", coll_name, "already in kccd_dict keys!")
                    raise AssertionError
                if "DONTCHECK" in block:
                    dontchecks += [block[block.find("DONTCHECK") + 9:block.find("END")].strip().split(" WITH ")]
                    block = block.replace(block[block.find("DONTCHECK"):block.find("END") + 3], "")
                # generate cover volume
                distance = len(
                    kccd_robot.get_chain(self.get_root(), temp["link"], links=False, joints=True, fixed=False))
                n_points = int((25 - 3) * 0.4 ** (distance / 3) + 3)  # Todo elaborate this
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
                print("Covering", temp["mesh"], "of", temp["link"], "with", n_points)
                out, _ = misc.execute_shell_command(
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

    # has to be overridden in SMURF
    def export_floatingbase(self, outputdir, ros_pkg_name=None, export_with_ros_pathes=False,
                            create_pdf=False):
        floatingbase = self.add_floating_base()
        floatingbase.full_export(outputdir, create_pdf=create_pdf, export_with_ros_pathes=export_with_ros_pathes,
                                 ros_pkg_name=ros_pkg_name, export_joint_limits=False)

    def export_submodel(self, name, output_dir=None, filename=None, export_visuals=True, export_collisions=True,
                        robotname=None, create_pdf=False, ros_pkg=False, export_with_ros_pathes=None,
                        ros_pkg_name=None, export_joint_limits=True, only_urdf=None):
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
                                     export_joint_limits=export_joint_limits, )
            return

        if only_urdf is None and "only_urdf" in self._submodels[name].keys():
            only_urdf = self._submodels[name]["only_urdf"]
        elif only_urdf is None:
            only_urdf = True if name.startswith("#sub_mech#") else False

        if name in self._submodels.keys():
            _submodel = self.instantiate_submodel(name)
            _sm_urdffile = None
            submodel_dir = os.path.join(output_dir, name)
            if only_urdf:
                _sm_urdffile = os.path.join(output_dir, filename if filename is not None else (name + ".urdf"))
            else:
                _sm_urdffile = os.path.join(submodel_dir, "urdf", name + ".urdf")
            for link in _submodel.links:
                for obj in link.collisions + link.visuals:
                    if hasattr(obj.geometry, "filename"):
                        obj.geometry.filename = os.path.relpath(read_urdf_filename(obj.geometry.filename,
                                                                                   self.xmlfile), start=_sm_urdffile)
            if robotname is not None:
                _submodel.name = robotname
            if only_urdf:
                _submodel.export_urdf(outputfile=_sm_urdffile, export_visuals=export_visuals,
                                      export_collisions=export_collisions, create_pdf=create_pdf, ros_pkg=ros_pkg,
                                      export_with_ros_pathes=export_with_ros_pathes)
            else:
                os.makedirs(submodel_dir, exist_ok=True)
                _submodel.full_export(output_dir=submodel_dir, export_visuals=export_visuals,
                                      export_collisions=export_collisions, create_pdf=create_pdf,
                                      ros_pkg=ros_pkg, export_with_ros_pathes=export_with_ros_pathes,
                                      ros_pkg_name=ros_pkg_name, export_joint_limits=export_joint_limits,
                                      export_submodels=False)
        else:
            print("No submodel named {}".format(name))

    # getters
    def get_submodel(self, name):
        """ Return the submodel with the given name.
        """
        if name in self._submodels.keys():
            return self._submodels[name]
        else:
            print("No submodel named {}".format(name))
        return

    def get_id(self, targettype, target):
        """
        Returns the id of the given instance
        :param targettype: the tye of the searched instance
        :param target: the name of the searched instance
        :return: the id or None if not found
        """
        for i, obj in enumerate(getattr(self, targettype)):
            if obj.name == target:
                return i
        return None

    def get_joint_id(self, joint_name):
        """
        Returns the ID (index in the joint list) of the joint.
        :param joint_name: the name of the joint to search
        :return: the Id if found else None
        """
        if isinstance(joint_name, list):
            return [self.get_id('joints', name) for name in joint_name]

        return self.get_id('joints', joint_name)

    def get_link_id(self, link_name):
        """
        Returns the ID (index in the link list) of the link.
        :param link_name: the name of the link to search
        :return: the Id if found else None
        """
        if isinstance(link_name, list):
            return [self.get_id('links', name) for name in link_name]

        return self.get_id('links', link_name)

    def get_link(self, link_name):
        """
        Returns the link(s) corresponding to the link name(s).
        :param link_name: the name of the joint to get
        :return: the link instance, None if not found
        """
        if isinstance(link_name, list):
            return [self.get_link(lname) for lname in link_name]

        l_id = self.get_link_id(link_name)

        if l_id is not None:
            return self.links[l_id]
        else:
            print("WARN: Link", link_name, "does not exist!")
            print("These are the existing links:", [ln.name for ln in self.links])
            return None

    def get_joint(self, joint_name):
        """
        Returns the joint(s) corresponding to the joint name(s).
        :param joint_name: the name of the joint to get
        :return: the joint instance, None if not found
        """
        if isinstance(joint_name, list):
            return [self.get_joint(jname) for jname in joint_name]

        j_id = self.get_joint_id(joint_name)

        if j_id is not None:
            return self.joints[j_id]
        else:
            # print(joint_name, "not found in robot with joints", [jn.name for jn in self.joints])
            return None

    def get_parent(self, name, targettype='joint'):
        """
        Get the parent of targettype for the given link name.
        :param name: the name of the joint or link to get the parent for
        :param targettype: the next parent joint or the next parent link (default: 'joint')
        :return: List with one element (todo)
        """
        # Check if the name is present
        if name in self.parent_map.keys():
            parents = self.parent_map[name]
        elif name in self.child_map.keys():
            return None
        else:
            print("Parent map keys: ", self.parent_map.keys())
            raise AssertionError("Nothing with name " + name + " in this robot")

        # Parentmap contains links.
        # If we want joints, then collect the children of these
        assert parents is not None
        if targettype == "link":
            return [parents[1]]
        if targettype == 'joint':
            return [parents[0]]

        return parents

    def get_inertial(self, link_name):
        """
        Returns the inertial of the given link.
        :param link_name: the name of the respective link
        :return: the inertial of the given link
        """
        link_id = self.get_link_id(link_name)

        if link_id is None:
            return None

        return self.links[link_id].inertial

    def get_visual(self, link_name):
        """
        Return all visuals of the given link if it exists.
        :param link_name: the name of the respective link
        :return: list of visuals of the given link, if there are else none
        """
        link_id = self.get_link_id(link_name)

        if link_id is None:
            return None

        if self.links[link_id].visuals:
            return self.links[link_id].visuals
        else:
            return None

    def get_all_visuals(self):
        visuals = []
        for link in self.links:
            visuals += link.visuals
        return visuals

    def get_visual_by_name(self, visual_name):
        """
        Returns the visual with the given name if it exists
        :param visual_name: name of the searched visual
        :return: the found visual or none
        """
        for link in self.links:
            for vis in link.visuals:
                if vis.name == visual_name:
                    return vis

        return None

    def get_collision(self, link_name):
        """
        Return all collisions of the given link if it exists.
        :param link_name: the name of the respective link
        :return: list of collisions of the given link, if there are else none
        """
        link_id = self.get_link_id(link_name)

        if link_id is None:
            return None

        if self.links[link_id].collisions:
            return self.links[link_id].collisions
        else:
            return None

    def get_all_collisions(self):
        collisions = []
        for link in self.links:
            collisions += link.collisions
        return collisions

    def get_collision_by_name(self, collision_name):
        """
        Returns the collision with the given name if it exists
        :param collision_name: name of the searched collision
        :return: the found collision or none
        """
        for link in self.links:
            for coll in link.collisions:
                if coll.name == collision_name:
                    return coll

        return None

    def get_material_by_name(self, material_name):
        """
        Returns the collision with the given name if it exists
        :param collision_name: name of the searched collision
        :return: the found collision or none
        """
        for mat in self.materials:
            if mat.name == material_name:
                return mat

        return None

    def get_children(self, name, targettype='joint'):
        """
        Get the children of type targettype for the given link name.
        :param name: the name of the parent link
        :param targettype: whether we want the next joint children or link childrne
        :return: list of children of the given link
        """
        if isinstance(name, list):
            children = []
            for n in name:
                new_children = self.get_children(n, targettype=targettype)
                children += new_children if new_children else []
            return children

        children = []

        if name in self.child_map.keys():
            children = self.child_map[name]

        if children:
            if targettype == 'joint':
                children = [i[0] for i in children]
            elif targettype == 'link':
                children = [i[1] for i in children]

        return children

    def get_leaves(self, start=None):
        """
        Get all leaves of the given start link.
        If start is provided, returns leaves of the sub
        :param start: the root link for which to get the leaves
        :return:
        """
        if not start:
            parentset = set(self.parent_map.keys())
            childset = set(self.child_map.keys())
            return list(parentset - childset)

        if start not in self.parent_map.keys():
            return None

        parents = [start]
        parents, children = self._get_children_lists(parents, [])

        parentset = set(parents)
        childrenset = set(children)

        return parentset, childrenset

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

    def global_origin(self, stop):
        """ Get the global pose of the link.
        """
        return to_origin(self.get_transformation(stop))

    def get_transformation(self, end, start=None):
        """
        Returns the transformation from start to end
        :param end: the end link of the transformation
        :param start: the start link of the transformation (default is root)
        :return: the transformation matrix
        """
        if start is None:
            start = self.get_root()
        transformation = Homogeneous((0, 0, 0), (0, 0, 0))

        assert type(end) is str
        assert type(start) is str

        link = end
        while link != start:
            parent = self.get_parent(link)
            if parent is not None and len(parent) > 1:
                print("Multiple parents:", parent, flush=True)
            elif parent is None:
                raise Exception(link, "has no parent, but is different from start", start)
            pjoint = self.get_joint(parent[0])
            transformation = Homogeneous(pjoint.origin.xyz, pjoint.origin.rpy).dot(transformation)
            link = pjoint.parent

        return transformation

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
            print("Link or new parent not found!")
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
                I_L = inertial_to_tensor(inertia_L)
                IP_T_IL = origin_to_homogeneous(parent.inertial.origin).dot(
                    L_T_P.dot(origin_to_homogeneous(inertia_L.origin)))
                Ad = Adjoint(IP_T_IL)
                # Transform into parent
                I_NL = Ad.dot(I_L.dot(Ad.T)) + inertial_to_tensor(parent.inertial)
                parent.inertial = create_inertial(I_NL, parent.inertial.origin)

            else:
                # Set inertial to new parent
                new_origin = L_T_P.dot(origin_to_homogeneous(inertia_L.origin))
                parent.inertial = link.inertial
                parent.inertial.origin = to_origin(new_origin)

            # Set link to near zero
            link.inertial = create_inertial(1e-5 * np.ones((6, 6)), link.inertial.origin)

        if visual and link.visuals:
            for vis in link.visuals:
                VL_T_L = origin_to_homogeneous(vis.origin)
                new_origin = L_T_P.dot(VL_T_L)
                vis.origin = to_origin(new_origin)
                parent.add_aggregate('visual', deepcopy(vis))
                link.remove_aggregate(vis)

        if collision and link.collisions:
            for col in link.collisions:
                CL_T_L = origin_to_homogeneous(col.origin)
                new_origin = L_T_P.dot(CL_T_L)
                col.origin = to_origin(new_origin)
                parent.add_aggregate('collision', deepcopy(col))
                link.remove_aggregate(col)

        # Reinit the link
        link.to_xml()
        parent.to_xml()
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
        jointname = jointname[0]
        joint = self.get_joint(jointname)

        T0_old = self.get_transformation(link_name)
        T0_newp = self.get_transformation(new_parent_name)
        joint.origin = to_origin(inv(T0_newp).dot(T0_old))
        joint.parent = new_parent_name

    def define_submodel(self, name, start, stop=None, robotname=None, only_urdf=False, only_return=False,
                        overwrite=False):
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
            "only_urdf": only_urdf
        }
        if only_return:
            return self.instantiate_submodel(definition=definition)
        if name in self._submodels.keys() and not overwrite:
            raise NameError("A submodel with the given name is already defined")
        else:
            self._submodels[name] = definition
        return self.instantiate_submodel(name)

    def instantiate_submodel(self, name=None, definition=None):
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
        elif name is not None and definition is not None:
            assert definition["name"] not in self._submodels.keys()
            if name != definition["name"]:
                self.remove_submodel(name)
            self.define_submodel(**definition)
        elif name is None and definition is None:
            raise AssertionError("No args given!")

        assert all([x in definition.keys() for x in ["name", "start", "stop"]])

        if definition["stop"] is None:
            # Collect all links on the way to the leaves
            parents, children = self._get_children_lists([definition["start"]], [])
            parentset = set(parents)
            childrenset = set(children)
            linknames = list(parentset.union(childrenset))
        else:
            linknames = set()
            try:
                for leave in definition["stop"]:
                    linknames.update(set(self.get_chain(definition["start"], leave, joints=False)))
            except Exception as e:
                print(self.get_root())
                print(self.parent_map.keys())
                print([link.name for link in self.links])
                print("Start", definition["start"], "Stop", definition["stop"])
                raise e
            linknames = list(linknames)

        jointnames = self.get_children(linknames)
        link_ids = self.get_link_id(linknames)
        joint_ids = self.get_joint_id(jointnames)

        if "robotname" not in definition.keys() or definition["robotname"] is None:
            definition["robotname"] = name

        submodel = type(self)(name=definition["robotname"])
        for mat in self.materials:
            submodel.add_aggregate("material", deepcopy(mat))
        for i in link_ids:
            submodel.add_aggregate('link', deepcopy(self.links[i]))
        for j in joint_ids:
            if self.joints[j].child in linknames:
                submodel.add_aggregate('joint', deepcopy(self.joints[j]))
        submodel.copy_related_annotations(self)

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

    # # TODO this method is not fully implemented
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

    def verify_meshes(self):
        no_problems = True
        for link in self.links:
            for vc in link.collisions + link.visuals:
                if hasattr(vc.geometry, "filename") and \
                        pgu.import_mesh(vc.geometry.filename, urdf_path=self.xmlfile) is None:
                    print("WARNING: Mesh file", vc.geometry.filename,
                          "is empty and therefore the corresponding visual/geometry removed!")
                    no_problems = False
                    link.remove_aggregate(vc)
        return no_problems

    def correct_inertials(self, limit=1e-5):
        """
        Correct all inertials of the robot.
        """
        for link in self.links:
            # Todo check if the I is bascically zero and then recreate the inertial using the collision
            if link.inertial:
                M = inertial_to_tensor(self.get_inertial(link.name))
                origin = link.inertial.origin
            else:
                M = np.zeros((6, 6))
                origin = to_origin(np.eye(4))
            m = M[0, 0]
            if m <= limit:
                M[:3, :3] = np.eye(3) * limit
                print(" Corrected mass for link {}".format(link.name))

            I = M[3:, 3:]

            if any([x < limit for x in np.linalg.eigvals(I)]):
                E, V = np.linalg.eig(I)
                diff = np.abs(np.min(E) - limit)

                E = np.diag([e + diff for e in E])
                I = np.matmul(V, np.matmul(E, np.linalg.inv(V)))

                M[3:, 3:] = I

            link.inertial = create_inertial(M, origin)

            print(" Corrected inertia for link {}".format(link.name))

    def correct_axes(self, joints=None, tol=1E-3):
        """
        Turns all joints where the axis are not unit. Not yet entirely tested
        """
        if joints is None:
            joints = [j for j in self.joints if j.type != "fixed" and hasattr(j, "axis") and j.axis is not None]
        elif not isinstance(joints, list):
            joints = [joints]

        for joint in joints:
            joint.axis = joint.axis / np.linalg.norm(joint.axis)
            axis_correction = np.eye(4)
            if 1 - (np.amax(np.abs(joint.axis))) > tol:
                print(
                    "WARNING: Axis of joint " + joint.name + " is not even close to unit! No changes made Axis:" + str(
                        joint.axis))
            elif joint.type != "fixed" and len(np.where(np.array(joint.axis) == 0.0)[0]) != 2:
                print("WARNING: joint axis is not x, y or z unit vector:\n", joint.__dict__)
                v = [np.abs(a) for a in joint.axis]
                new_axis = [0 if i != np.argmax(v) else 1 for i in range(3)]
                if joint.axis[np.argmax(v)] < 0:
                    new_axis *= -1
                R = scipy_rot.align_vectors([new_axis], [joint.axis])
                # joint.axis = new_axis
                axis_correction[:3, :3] = R[0].as_matrix()
                print("Rotating joint by \n", axis_correction)
                print("New axis is:", new_axis)

                joint.origin = to_origin(axis_correction.dot(origin_to_homogeneous(joint.origin)))
                joint.axis = new_axis

    def compute_mass(self):
        """
        Compute the overall mass of the robot.
        """
        m = 0.0
        for link in self.links:
            m += link.inertial.mass if link.inertial else 0.0
        print("{} has a total mass of {} kg.".format(self.name, m))
        return m

    def compute_com(self):
        com = np.array([0.0, 0.0, 0.0])
        mass = 0
        for link in self.links:
            T = self.get_transformation(link.name)
            if link.inertial is not None:
                T = T.dot(origin_to_homogeneous(link.inertial.origin))
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
                Tinv = inv(inv(origin_to_homogeneous(pjoint[0].origin)).dot(T))
                pjoint[0].origin = to_origin(T)
            else:
                pjoint[0].origin = to_origin(origin_to_homogeneous(pjoint[0].origin).dot(T))
        if only_frame:
            for joint in cjoint:
                joint.origin = to_origin(Tinv.dot(origin_to_homogeneous(joint.origin)))

            for ent in link.collisions + link.visuals:
                ent.origin = to_origin(Tinv.dot(origin_to_homogeneous(ent.origin)))

            self.transform_inertial(linkname, transformation=Tinv)

    def transform_link(self, linkname, translation=None, rotation=None, transformation=None, maintain_children=True):
        """Rotate the given link in such a way, that inertials, visuals and collisions stay the same.
        """
        if self.get_link_id(linkname) is None:
            raise Exception(
                "Provide valid link to attach to. '" + linkname + "' is not in " + str([ln.name for ln in self.links]))

        print("Transform {}...".format(linkname))
        # Get the parent joint
        pjoint = self.get_joint(self.get_parent(linkname))

        # Get the child joint
        cjoint = self.get_joint(self.get_children(linkname))
        # Process the rotation
        if transformation is not None:
            T = transformation
        else:
            T = Homogeneous(translation, rotation)
        Tinv = inv(T)

        # If we have just one parent and children, we rotate the parent and inverse rotate all children
        print(" Transforming Joints")
        if pjoint is not None:
            # Transform the parent
            assert urdf.transform_object(pjoint, T)
            if maintain_children:
                # Get all child origin
                assert urdf.transform_object(cjoint, Tinv)

            return

        else:
            # Else we have to rotate everything inside the joint (if it is root)
            # Get the inertial, visual and collision
            assert urdf.transform_object(cjoint, T)
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

        print(" Transforming Inertials")
        # Process the rotation
        if transformation is not None:
            T = transformation
        else:
            T = Homogeneous(xyz=translation, rpy=rotation)
        # Transform the origin
        assert urdf.transform_object(inertial, T)
        rot_part = Homogeneous(rpy=inertial.origin.rpy, xyz=[0, 0, 0])
        inertial.origin.rpy = [0, 0, 0]

        # T = rot_part.dot(T)
        Ad = Adjoint(rot_part)
        # Tinv = inv(T)
        # AdInv = Adjoint(Tinv)

        # Get the Inertia Tensor
        I = inertial_to_tensor(inertial)

        # Rotate the inertial tensor
        I = np.matmul(np.transpose(Ad), np.matmul(I, Ad))

        inertial = create_inertial(I, inertial.origin)

        self.links[link_id].inertial = inertial
        return True

    def transform_visual(self, linkname, translation, rotation):
        """Transform the visual(s) of the given link.
        """
        # Get the visual
        visual = self.get_visual(linkname)
        print(" Transform Visuals")
        # Transform
        T = Homogeneous(translation, rotation)
        assert urdf.transform_object(visual, T)
        return True

    def transform_collision(self, linkname, translation, rotation):
        """Transform the collision(s) of the given link.
        """
        # Get the collision
        collision = self.get_collision(linkname)
        print(" Transform Collisions")
        # Transform
        T = Homogeneous(translation, rotation)
        assert urdf.transform_object(collision, T)
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
            print("Not overwriting com of link: ", link.name)
            return

        volume = 0.0
        com = np.array([0.0, 0.0, 0.0])
        if len(link.collisions) == 0:
            return
        for coll in link.collisions:
            T = origin_to_homogeneous(coll.origin)
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
        T_ = Homogeneous(xyz=transl, rpy=[0, 0, 0])
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
            if joint.type == "fixed":
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
                    print("Warning: Joint limits for ", joint.name, "not defined taking default values!")
                elif raise_error:
                    raise ValueError("ERROR: Joint limits for ", joint.name, "not defined!")
            else:
                if any([not hasattr(joint.limit, x) for x in ["lower", "upper", "effort", "velocity"]]) or \
                        any([getattr(joint.limit, x) is None for x in ["lower", "upper", "effort", "velocity"]]):
                    print(joint.name, joint.limit.lower, joint.limit.upper, joint.limit.effort, joint.limit.velocity)
                    result &= 2
                    if raise_error:
                        raise ValueError("ERROR: Not all joint limits for " + joint.name + " defined!")
                if (joint.type == "revolute" or joint.type == "prismatic") and \
                        not hasattr(joint, "axis") or joint.axis is None:
                    print("WARNING: Joint axis for joint", joint.name, "not defined. Setting to [0 0 1]", flush=True,
                          file=sys.stderr)
                    joint.axis = default_axis
                    result &= 4
                if hasattr(joint, "limit") and joint.limit is not None and (
                        joint.limit.lower == joint.limit.upper or joint.limit.velocity == 0):
                    print("WARNING: The joint limits of joint", joint.name, "might restrict motion:\n",
                          "min:", joint.limit.lower, "max", joint.limit.upper, "vel", joint.limit.velocity, "eff",
                          joint.limit.effort,
                          flush=True, file=sys.stderr)
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
                        print(" Therefore we take the backup/default values for the joint limits:\n",
                              "min:", joint.limit.lower, "max", joint.limit.upper, "vel", joint.limit.velocity, "eff",
                              joint.limit.effort,
                              flush=True, file=sys.stderr
                              )

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
            target = target.name
        if replacements is None:
            replacements = {}
        if do_not_double:
            if prefix is not None and target.startswith(prefix):
                prefix = None
            if suffix is not None and target.endswith(suffix):
                suffix = None

        if not prefix and not suffix and replacements == {}:
            return renamed_entities

        new_name = misc.edit_name_string(target, prefix=prefix, suffix=suffix, replacements=replacements)

        # Rename all instances in child and parent map as keys
        index = None  # gives the index of joint or link for parent/child maps
        if targettype.startswith('link'):
            index = 1
            new_link_map = {}
            for k, v in self.link_map.items():
                if k == target:
                    new_link_map[new_name] = v
                else:
                    new_link_map[k] = v
            self.link_map = new_link_map
            for j in self.joints:
                if j.parent == target:
                    j.parent = new_name
                if j.child == target:
                    j.child = new_name
        elif targettype.startswith('joint'):
            index = 0
            new_joint_map = {}
            for k, v in self.joint_map.items():
                if k == target:
                    new_joint_map[new_name] = v
                else:
                    new_joint_map[k] = v
            self.joint_map = new_joint_map
            for j in self.joints:
                if hasattr(j, "mimic") and j.mimic is not None and j.mimic.joint == target:
                    j.mimic.joint = new_name

        if targettype.startswith("link") or targettype.startswith("joint"):
            # Iterate over the values of parent child map and rename the values
            new_child_map = {}
            for k, v in self.child_map.items():
                for i, vi in enumerate(v):
                    if target == vi[index]:
                        v[i] = tuple([new_name if i == index else n for i, n in enumerate(vi)])
                if k == target:
                    new_child_map[new_name] = v
                else:
                    new_child_map[k] = v
            self.child_map = new_child_map

            new_parent_map = {}
            for k, v in self.parent_map.items():
                if target == v[index]:
                    v = tuple([new_name if i == index else n for i, n in enumerate(v)])
                if k == target:
                    new_parent_map[new_name] = v
                else:
                    new_parent_map[k] = v
            self.parent_map = new_parent_map

        renamed_entities[target] = new_name

        if targettype == 'link':
            self.get_link(target).name = new_name
            for k, v in self._submodels.items():
                if target == v["start"]:
                    self._submodels[k]["start"] = new_name
                if target in v["stop"]:
                    self._submodels[k]["stop"] = [link if link != target else new_name for link in v]
        elif targettype == 'joint':
            self.get_joint(target).name = new_name
        elif targettype == 'collision':
            obj = self.get_collision_by_name(target)
            if obj is not None:
                obj.name = new_name
        elif targettype == 'visual':
            obj = self.get_visual_by_name(target)
            if obj is not None:
                obj.name = new_name
        elif targettype == 'material':
            obj = self.get_material_by_name(target)
            if obj is not None:
                obj.name = new_name
        renamed_entities.update(self._rename(targettype, target, new_name=new_name))
        return renamed_entities

    def edit_names(self, cfg):
        """
        Renames everything according to the given dict.
        Dict has to be of structure:
        {
            joint_equals_link_name: whether the joint shall be named like the link without the "_link" suffix
            append_link_suffix: ALWAYS or NAME_DUPLICATES or False
            name_replacements: dict or list of dicts, applied on everything
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
        vis_replacements = cfg["visual_replacements"] if "visual_replacements" in cfg.keys() else {}
        vis_suffix = cfg["visual_suffix"] if "visual_suffix" in cfg.keys() else ""
        vis_prefix = cfg["visual_prefix"] if "visual_prefix" in cfg.keys() else ""
        coll_replacements = cfg["collision_replacements"] if "collision_replacements" in cfg.keys() else {}
        coll_suffix = cfg["collision_suffix"] if "collision_suffix" in cfg.keys() else ""
        coll_prefix = cfg["collision_prefix"] if "collision_prefix" in cfg.keys() else ""
        name_replacements = cfg["name_replacements"] if "name_replacements" in cfg.keys() else {}
        for link in self.links:
            self.rename("link", link.name, replacements=name_replacements)
            for collision in link.collisions:
                self.rename("collision", collision.name, replacements=name_replacements)
                self.rename("collision", collision.name, prefix=coll_prefix, suffix=coll_suffix,
                            replacements=coll_replacements)
            for visual in link.visuals:
                self.rename("visual", visual.name, replacements=name_replacements)
                self.rename("visual", visual.name, prefix=vis_prefix, suffix=vis_suffix, replacements=vis_replacements)
        for joint in self.joints:
            if "joint_equals_link_name" in cfg.keys() and cfg["joint_equals_link_name"]:
                self.rename(targettype="joint", target=joint.name, replacements={
                    joint.name: joint.child if not joint.child.upper().endswith("_LINK") else joint.child[:-5]
                })
            else:
                self.rename(targettype="joint", target=joint.name, replacements=name_replacements)
        if "append_link_suffix" in cfg.keys() and cfg["append_link_suffix"] is not False:
            for link in self.links:
                if not link.name[-4:].upper() == "LINK":
                    if cfg["append_link_suffix"].upper() == "ALWAYS":
                        self.rename(targettype="link", target=link.name, suffix="_Link")
                    elif cfg["append_link_suffix"].upper() == "NAME_DUPLICATES":
                        pjoint = self.get_parent(link.name)
                        if pjoint is not None and pjoint[0] == link.name:
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
                    print("WARNING: Mesh", coll.name, "has not enough vertices. Removing geometry!", flush=True,
                          file=sys.stderr)
                    pgu.remove_collision(self, link.name, collisionname=coll.name)
            for vis in link.visuals:
                if not pgu.has_enough_vertices(vis, self.xmlfile):
                    print("WARNING: Mesh", vis.name, "has not enough vertices. Removing geometry!", flush=True,
                          file=sys.stderr)
                    pgu.remove_visual(self, link.name, visualname=vis.name)

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
        # Check if other is robot
        if not isinstance(other, Robot):
            raise Exception("Can only attach robot to robot.")

        if self.get_link_id(joint.parent) is None:
            raise Exception("Provide valid link to attach to. '" + joint.parent + "' is not in " + str(
                [ln.name for ln in self.links]))

        if not isinstance(joint, representation.Joint):
            raise Exception("Provide valid joint type.")

        # Check for naming and rename if necessary
        plink = set([link.name for link in self.links])
        pjoints = set([j.name for j in self.joints])
        pcollisions = set([c.name for c in self.get_all_collisions()])
        pvisuals = set([v.name for v in self.get_all_visuals()])
        pmaterials = set([m.name for m in self.materials])

        clink = set([link.name for link in other.links])
        cjoints = set([j.name for j in other.joints])
        ccollisions = set([c.name for c in other.get_all_collisions()])
        cvisuals = set([v.name for v in other.get_all_visuals()])
        cmaterials = set([m.name for m in other.materials])

        renamed_entities = {}

        if plink & clink:
            if not do_not_rename:
                print("Warning : Link names are duplicates. A", name_prefix, "and a", name_suffix,
                      "will be pre-/appended!", plink & clink, file=sys.stderr)
                renamed_entities.update(
                    other.rename(targettype="link", target=list(plink & clink), prefix=name_prefix, suffix=name_suffix))
                if joint.child in list(plink & clink):
                    joint.child = name_prefix + joint.child + name_suffix
                return self.attach(other, joint, do_not_rename=do_not_rename, name_prefix=name_prefix,
                                   name_suffix=name_suffix)
            else:
                raise NameError("There are duplicates in link names", repr(plink & clink))

        if pjoints & cjoints:
            if not do_not_rename:
                print("Warning : Joint names are duplicates a _2 will be appended!", pjoints & cjoints, file=sys.stderr)
                renamed_entities.update(other.rename(targettype="joint", target=list(pjoints & cjoints), suffix="_2"))
                if joint.name in list(pjoints & cjoints):
                    joint.name = joint.name + "_2"
                return self.attach(other, joint, do_not_rename=do_not_rename)
            else:
                raise NameError("There are duplicates in joint names", repr(pjoints & cjoints))

        if pcollisions & ccollisions:
            if not do_not_rename:
                print("Warning : Collision names are duplicates a _2 will be appended!", pcollisions & ccollisions,
                      file=sys.stderr)
                renamed_entities.update(
                    other.rename(targettype="collision", target=list(pcollisions & ccollisions), suffix="_2"))
                return self.attach(other, joint, do_not_rename=do_not_rename)
            else:
                raise NameError("There are duplicates in collision names", repr(pcollisions & ccollisions))

        if pvisuals & cvisuals:
            if not do_not_rename:
                print("Warning : Visual names are duplicates a _2 will be appended!", pvisuals & cvisuals,
                      file=sys.stderr)
                renamed_entities.update(
                    other.rename(targettype="visual", target=list(pvisuals & cvisuals), suffix="_2"))
                return self.attach(other, joint, do_not_rename=do_not_rename)
            else:
                raise NameError("There are duplicates in visual names", repr(pvisuals & cvisuals))

        for mat_name in pmaterials & cmaterials:
            if self.get_material_by_name(mat_name).to_xml_string() == \
                    other.get_material_by_name(mat_name).to_xml_string():
                cmaterials.remove(mat_name)
        if pmaterials & cmaterials:
            if not do_not_rename:
                print("Warning : Material names are duplicates a _2 will be appended!", pmaterials & cmaterials,
                      file=sys.stderr)
                renamed_entities.update(
                    other.rename(targettype="material", target=list(pmaterials & cmaterials), suffix="_2"))
                return self.attach(other, joint, do_not_rename=do_not_rename)
            else:
                raise NameError("There are duplicates in material names", repr(pmaterials & cmaterials))

        # Add all joints
        for cJoint in other.joints:
            self.add_aggregate('joint', cJoint)

        for cLink in other.links:
            self.add_aggregate('link', cLink)

        for cMaterial in other.materials:
            if any([((oMat.color is not None and
                      cMaterial.color is not None and
                      oMat.color.rgba == cMaterial.color.rgba)
                     or oMat.color == cMaterial.color)
                    and ((oMat.texture is not None and
                          cMaterial.texture is not None and
                          oMat.texture.filename == cMaterial.texture.filename)
                         or oMat.texture == cMaterial.texture)
                    for oMat in other.materials]):
                continue
            else:
                self.add_aggregate('material', cMaterial)

        for cTransmission in other.transmissions:
            self.add_aggregate('transmission', cTransmission)

        self.add_aggregate('joint', joint)

        return renamed_entities

    def add_link_by_properties(self, name, translation, rotation, parent, jointname=None, jointtype="fixed", axis=None,
                               mass=0.0):
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
                    origin=representation.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0])
                )
            )
        else:
            inertial = None
        link = representation.Link(name, inertial=inertial)
        joint = representation.Joint(name=jointname if jointname is not None else name, parent=parent.name,
                                     child=link.name,
                                     type=jointtype, origin=representation.Pose(translation, rotation), axis=axis)
        self.add_aggregate("link", link)
        self.add_aggregate("joint", joint)
        return parent, link, joint

    # has to be overridden in smurf and hyrodyn?
    def mirror_model(self, mirror_plane=None, maintain_order=None, exclude_meshes=None, name_replacements=None,
                     target_urdf=None, only_return=False):
        """
        Mirrors the robot model.
        :param mirror_plane: The normal of the mirror plane. Default y-plane
        :param maintain_order: The priority in which the frame axes will be preferably conserved
        :param exclude_meshes: List of meshes that will not be mirrored. Use this for already symmetric meshes
        :param name_replacements: dict, of name replacements for all names
        :param only_return: if set to true this instance remains untouched and the mirrored model will only be returned
        :return: None or mirrored model depending on only_return
        """
        # Create a new robot
        if name_replacements is None:
            name_replacements = {}
        if exclude_meshes is None:
            exclude_meshes = []
        if maintain_order is None:
            maintain_order = [0, 2, 1]
        if mirror_plane is None:
            mirror_plane = [0, 1, 0]
        robot = type(self)(self.name)
        for mat in self.materials:
            robot.add_aggregate("material", mat)
        if not target_urdf:
            target_urdf = self.xmlfile

        # reflection matrix
        T_R = pgu.get_reflection_matrix(normal=np.array(mirror_plane))

        # copy kinematic
        def recursive_link_transform(link_, T_root_to_link):
            link_ = self.get_link(link_)
            T_link = self.get_transformation(link_.name)
            new_link = deepcopy(link_)

            # transform link information to root and mirror

            if new_link.inertial is not None:
                T = T_R.dot(T_link.dot(origin_to_homogeneous(link_.inertial.origin)))
                new_origin = to_origin(inv(T_root_to_link).dot(T))
                new_origin.rpy = [0, 0, 0]

                # Process the rotation
                T = inv(T_root_to_link.dot(origin_to_homogeneous(link_.inertial.origin))).dot(T)
                Ad = Adjoint(T)
                # Get the Inertia Tensor
                I = inertial_to_tensor(link_.inertial)
                # Rotate the inertial tensor
                I = np.matmul(np.transpose(Ad), np.matmul(I, Ad))

                new_link.inertial = create_inertial(I, new_origin)

            for vis in new_link.visuals:
                T = T_R.dot(T_link.dot(origin_to_homogeneous(vis.origin)))
                vis.origin = to_origin(inv(T_root_to_link).dot(T))
                if hasattr(vis.geometry, "filename") and not (
                        (os.path.basename(vis.geometry.filename).split(".")[0] in exclude_meshes or
                         "ALL" in exclude_meshes)):
                    pgu.mirror_geometry(vis, urdf_path=target_urdf,
                                        transform=inv(T_root_to_link.dot(origin_to_homogeneous(vis.origin))).dot(T),
                                        name_replacements=name_replacements)

            for col in new_link.collisions:
                T = T_R.dot(T_link.dot(origin_to_homogeneous(col.origin)))
                col.origin = to_origin(inv(T_root_to_link).dot(T))
                if hasattr(col.geometry, "filename") and not (
                        (os.path.basename(col.geometry.filename).split(".")[0] in exclude_meshes or
                         "ALL" in exclude_meshes)):
                    pgu.mirror_geometry(col, target_urdf,
                                        transform=inv(T_root_to_link.dot(origin_to_homogeneous(col.origin))).dot(T),
                                        name_replacements=name_replacements)

            robot.add_aggregate("link", new_link)

            # joints:
            for jointname in self.get_children(link_.name):
                joint_ = self.get_joint(jointname)
                new_joint = deepcopy(joint_)

                T_flip = np.eye(4)
                flip_axis = None

                axis_correction = np.eye(4)
                if new_joint.type != "fixed":
                    new_joint.axis = new_joint.axis / np.linalg.norm(new_joint.axis)
                    if len(np.where(np.array(new_joint.axis) == 0.0)[0]) != 2:
                        print("WARNING: joint axis is not x, y or z unit vector:\n", new_joint.__dict__)
                        vec = [np.abs(a) for a in new_joint.axis]
                        new_axis = [0 if i != np.argmax(vec) else 1 for i in range(3)]
                        if new_joint.axis[np.argmax(vec)] < 0:
                            new_axis *= -1
                        rot = scipy_rot.align_vectors([new_axis], [new_joint.axis])
                        new_joint.axis = new_axis
                        axis_correction[:3, :3] = rot[0].as_matrix()
                        print("Rotating joint by \n", axis_correction)
                        print("New axis is:", new_axis)

                if new_joint.type == "prismatic":
                    """
                    For prismatic joints we want to keep the direction of this joint while mirroring the axis.
                    Therefore this axis is maintained while the other two are flipped.
                    """
                    maintain_mirrored_axis = [np.where(np.array(new_joint.axis) != 0)[0][0]]
                    for i in range(3):
                        if maintain_order[i] not in maintain_mirrored_axis and len(maintain_mirrored_axis) <= 1:
                            maintain_mirrored_axis.append(maintain_order[i])
                        elif maintain_order[i] not in maintain_mirrored_axis and len(maintain_mirrored_axis) == 2:
                            flip_axis = maintain_order[i]
                    assert flip_axis is not None
                elif new_joint.type == "revolute":
                    """
                    For revolute joints we want to flip the axis of the joint so that the joint acts symmetrically
                    """
                    flip_axis = np.where(np.array(new_joint.axis) != 0)[0][0]
                else:
                    """
                    For all other joints (fixed) we take the least maintain axis
                    """
                    flip_axis = maintain_order[-1]

                T_flip[flip_axis, flip_axis] *= -1
                # now we transform the local coordinate system using t_flip to make it right handed
                new_joint.origin = to_origin(
                    inv(T_root_to_link).dot(
                        T_R.dot(T_link.dot(axis_correction.dot(origin_to_homogeneous(new_joint.origin)).dot(T_flip)))
                    )
                )
                new_T_root_to_link = T_root_to_link.dot(origin_to_homogeneous(new_joint.origin))

                robot.add_aggregate("joint", new_joint)
                recursive_link_transform(joint_.child, new_T_root_to_link)

        recursive_link_transform(self.get_root(), np.eye(4))
        robot.rename("link", robot.links, replacements=name_replacements)
        robot.rename("joint", robot.joints, replacements=name_replacements)
        robot.rename("collision", robot.get_all_collisions(), replacements=name_replacements)
        robot.rename("visual", self.get_all_visuals(), replacements=name_replacements)
        for k, v in self.__dict__.items():
            if k not in robot.__dict__.keys() or robot.__dict__[k] is None:
                robot.__dict__[k] = v
        if not only_return:
            self.__dict__ = robot.__dict__
        return robot

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
            links_to_cut = deepcopy(link_to_cut)

        # Create new robot
        before = type(self)(self.name)
        for mat in self.materials:
            before.add_aggregate("material", mat)
        beyond = {new_root: type(self)(self.name) for new_root in links_to_cut}
        for _, v in beyond.items():
            for mat in self.materials:
                v.add_aggregate("material", mat)
        # add link recursively from root
        root = self.get_root()

        def recursive_add(link, robot2add2, all_links=False):
            if link not in links_to_cut or all_links:
                for jointname in self.get_children(link):
                    joint = self.get_joint(jointname)
                    if joint is None:
                        raise ValueError("Joint with name " + jointname + " not found!")
                    robot2add2.add_aggregate("link", self.get_link(joint.child))
                    robot2add2.add_aggregate("joint", joint)
                    recursive_add(joint.child, robot2add2)

        before.add_aggregate("link", self.get_link(self.get_root()))
        recursive_add(root, before)
        before.copy_related_annotations(self)

        for root, robot in beyond.items():
            robot.add_aggregate("link", self.get_link(root))
            recursive_add(root, robot, all_links=True)
            robot.copy_related_annotations(self)
            setattr(robot, 'xmlfile', self.xmlfile)

        # finalize the robot
        setattr(before, 'xmlfile', self.xmlfile)
        return before, beyond

    def remove_beyond(self, link_to_cut):
        """
        Wrapper for split_robot(). Removes everything beyond the given link
        :param link_to_cut: str or list, the link(s) where to cut. this link(s) will be included
        :return: None
        """
        before, _ = self.split_robot(link_to_cut)
        self.__dict__ = before.__dict__

    def get_beyond(self, link_to_cut):
        """
        Wrapper for split_robot(). Returns the beyond model
        :param link_to_cut: str or list, the link where to cut, will be included in the model
        :return: model beyond the given link including it
        """
        _, beyond = self.split_robot(link_to_cut)
        return beyond

    def remove_before(self, link_to_cut):
        """
        Wrapper for split_robot(). Removes everything before this link.
        :param link_to_cut: str
        :return: None
        """
        assert type(link_to_cut) is str
        _, beyond = self.split_robot(link_to_cut)
        self.__dict__ = beyond[link_to_cut].__dict__

    def remove_joint(self, jointname, keep_collisions=True):
        """Remove the joint(s) from the mechanism and transforms all inertia, visuals and collisions
        to the corresponding parent of the joint.
        """

        if isinstance(jointname, list):
            for joint in jointname:
                self.remove_joint(joint)
            return

        j_id = self.get_joint_id(jointname)

        if j_id is None:
            print("Joint {} not in model.".format(jointname))
            return

        # Collect the parent and the child
        joint = self.joints[j_id]

        # Create a new robot
        robot = type(self)(self.name)
        for mat in self.materials:
            robot.add_aggregate("material", mat)

        parent_id = self.get_link_id(joint.parent)
        child_id = self.get_link_id(joint.child)

        parent = self.links[parent_id]
        child = self.links[child_id]

        if child.name in self.child_map.keys():
            next_joints = [names[0] for names in self.child_map[child.name]]
        else:
            next_joints = []

        # Get the transformation
        C_T_P = self.get_transformation(start=parent.name, end=child.name)

        for link in self.links:
            if link.name == parent.name:
                # Correct inertial if child inertial is found
                if child.inertial:
                    IC_T_P = C_T_P.dot(origin_to_homogeneous(child.inertial.origin))
                    M_c = inertial_to_tensor(child.inertial)
                    if parent.inertial:
                        COM_C = np.identity(4)
                        COM_C[0:3, 3] = np.array(child.inertial.origin.xyz)
                        COM_Cp = C_T_P.dot(COM_C)
                        new_origin = (
                                             np.array(parent.inertial.origin.xyz) * parent.inertial.mass +
                                             COM_Cp[0:3, 3] * child.inertial.mass
                                     ) / (parent.inertial.mass + child.inertial.mass)
                        new_origin = representation.Pose(xyz=new_origin, rpy=[0, 0, 0])
                        IC_T_IP = inv(origin_to_homogeneous(parent.inertial.origin)).dot(IC_T_P)
                        M_p = inertial_to_tensor(parent.inertial)
                        A = Adjoint(origin_to_homogeneous(new_origin))
                        M_p = np.dot(np.transpose(A), np.dot(M_p, A))
                    else:
                        IC_T_IP = IC_T_P
                        M_p = np.zeros((6, 6))
                        new_origin = to_origin(inv(IC_T_P))

                    A = Adjoint(IC_T_IP.dot(origin_to_homogeneous(new_origin)))
                    M = np.dot(np.transpose(A), np.dot(M_c, A)) + M_p
                    link.inertial = create_inertial(M, new_origin)

                # Correct visuals
                for vis in child.visuals:
                    VC_T_P = C_T_P.dot(origin_to_homogeneous(vis.origin))
                    vis.origin = to_origin(VC_T_P)
                    link.add_aggregate('visual', vis)

                if keep_collisions:
                    for col in child.collisions:
                        CC_T_P = C_T_P.dot(origin_to_homogeneous(col.origin))
                        col.origin = to_origin(CC_T_P)
                        link.add_aggregate('collision', col)

            if link.name != child.name:
                robot.add_aggregate('link', link)

        for j in self.joints:
            if not j.name == jointname:
                if j.name in next_joints:
                    j.origin = to_origin(C_T_P.dot(origin_to_homogeneous(j.origin)))
                    j.parent = parent.name
                robot.add_aggregate('joint', j)

        setattr(robot, 'xmlfile', self.xmlfile)

        self.__dict__.update(robot.__dict__)

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
            type='revolute',
            axis=[0, 0, 1],
            limit=representation.JointLimit(
                effort=0,
                velocity=0,
                lower=-1.57,
                upper=1.57
            )
        )
        fb_robot = deepcopy(self)
        floatingbase.attach(fb_robot, connector)
        floatingbase.name = fb_robot.name + "_floatingbase"
        return floatingbase


def derive_model_dictionary(root, name='', objectlist=[]):
    """Returns a dictionary representation of a Phobos model.

    If name is not specified, it overrides the modelname in the root. If the modelname is not
    defined at all, 'unnamed' will be used instead.

    Args:
      root(bpy_types.Object): root object of the model
      name(str, optional): name for the derived model (Default value = '')
      objectlist(list: bpy_types.Object): objects to derive the model from
      objectlist: (Default value = [])

    Returns:

    """
    import phobos.blender.utils.blender as bUtils
    import phobos.blender.utils.selection as sUtils
    import phobos.blender.utils.naming as nUtils
    import phobos.blender.utils.io as ioUtils
    from ..blender.model.models import (deriveLink, deriveMaterial, deriveJoint,
                                        deriveLight, deriveGroupEntry, deriveChainEntry, collectMaterials,
                                        deriveDictEntry)

    if root.phobostype not in ['link', 'submodel']:
        # log(root.name + " is no valid 'link' or 'submodel' object.", "ERROR")
        return None

    # define model name
    if name:
        modelname = name
    elif 'model/name' in root:
        modelname = root['model/name']
    else:
        modelname = 'unnamed'

    # define model version
    if 'model/version' in root:
        modelversion = root['model/version']
    else:
        modelversion = 'undefined'

    modeldescription = bUtils.readTextFile('README.md')

    model = {
        'links': {},
        'joints': {},
        'sensors': {},
        'motors': {},
        'controllers': {},
        'materials': {},
        'meshes': {},
        'lights': {},
        'groups': {},
        'chains': {},
        'date': datetime.datetime.now().strftime("%Y%m%d_%H:%M"),
        'name': modelname,
        'version': modelversion,
        'description': modeldescription,
    }

    # log(
    #     "Creating dictionary for model '" + modelname + "' with root '" + root.name + "'.",
    #     'INFO',
    #     prefix="\n",
    # )

    # create tuples of objects belonging to model
    if not objectlist:
        objectlist = sUtils.getChildren(
            root, selected_only=ioUtils.getExpSettings().selectedOnly, include_hidden=False
        )
    linklist = [link for link in objectlist if link.phobostype == 'link']

    # digest all the links to derive link and joint information
    # log("Parsing links, joints and motors... " + (str(len(linklist))) + " total.", "INFO")
    for link in linklist:
        # parse link information (including inertia)
        model['links'][nUtils.getObjectName(link, 'link')] = deriveLink(
            link, logging=True, objectlist=objectlist
        )

        # parse joint and motor information
        if sUtils.getEffectiveParent(link):
            # joint may be None if link is a root
            # to prevent confusion links are always defining also joints
            jointdict = deriveJoint(link, logging=True, adjust=True)
            # log("  Setting joint type '{}' for link.".format(jointdict['type']), 'DEBUG')
            # first check if we have motor information in the joint properties
            # if so they can be extended/overwritten by motor objects later on
            if '$motor' in jointdict:
                motordict = jointdict['$motor']
                # at least we need a type property
                if 'type' in motordict:
                    # if no name is given derive it from the joint
                    if not 'name' in motordict:
                        motordict["name"] = jointdict['name']
                    model['motors'][motordict['name']] = motordict
                    # link the joint by name:
                    motordict['joint'] = jointdict['name']
                del jointdict['$motor']

            model['joints'][jointdict['name']] = jointdict

            for mot in [child for child in link.children if child.phobostype == 'motor']:
                motordict = motormodel.deriveMotor(mot, jointdict)
                # motor may be None if no motor is attached
                if motordict:
                    # log("  Added motor {} to link.".format(motordict['name']), 'DEBUG')
                    if motordict['name'] in model["motors"]:
                        model['motors'][motordict['name']].update(motordict)
                    else:
                        model['motors'][motordict['name']] = motordict

    # parse sensors and controllers
    sencons = [obj for obj in objectlist if obj.phobostype in ['sensor', 'controller']]
    # log("Parsing sensors and controllers... {} total.".format(len(sencons)), 'INFO')
    for obj in sencons:
        props = deriveDictEntry(obj, names=True, objectlist=objectlist)
        model[obj.phobostype + 's'][nUtils.getObjectName(obj)] = props

    # parse materials
    # log("Parsing materials...", 'INFO')
    model['materials'] = collectMaterials(objectlist)
    for obj in objectlist:
        if obj.phobostype == 'visual':
            mat = obj.active_material
            if mat:
                if mat.name not in model['materials']:
                    model['materials'][mat.name] = deriveMaterial(mat)
                    linkname = nUtils.getObjectName(
                        sUtils.getEffectiveParent(obj, ignore_selection=bool(objectlist))
                    )
                    model['links'][linkname]['visual'][nUtils.getObjectName(obj)][
                        'material'
                    ] = mat.name

    # identify unique meshes
    # log("Parsing meshes...", "INFO")
    for obj in objectlist:
        try:
            if (
                    (obj.phobostype == 'visual' or obj.phobostype == 'collision')
                    and (obj['geometry/type'] == 'mesh')
                    and (obj.data.name not in model['meshes'])
            ):
                model['meshes'][obj.data.name] = obj
                # todo2.9: for lod in obj.lod_levels:
                #     if lod.object.data.name not in model['meshes']:
                #         model['meshes'][lod.object.data.name] = lod.object
        except KeyError:
            pass  # log("Undefined geometry type in object " + obj.name, "ERROR")

    # gather information on groups of objects
    # log("Parsing groups...", 'INFO')
    # todo2.9: TODO: get rid of the "data" part and check for relation to robot
    # for group in bpy.data.groups:
    #     # skip empty groups
    #     if not group.objects:
    #         continue

    #     # handle submodel groups separately from other groups
    #     if 'submodeltype' in group.keys():
    #         continue
    #         # TODO create code to derive Submodels
    #         # model['submodels'] = deriveSubmodel(group)
    #     elif nUtils.getObjectName(group, 'group') != "RigidBodyWorld":
    #         model['groups'][nUtils.getObjectName(group, 'group')] = deriveGroupEntry(group)

    # gather information on chains of objects
    # log("Parsing chains...", "INFO")
    chains = []
    for obj in objectlist:
        if obj.phobostype == 'link' and 'endChain' in obj:
            chains.extend(deriveChainEntry(obj))
    for chain in chains:
        model['chains'][chain['name']] = chain

    # gather information on lights
    # log("Parsing lights...", "INFO")
    for obj in objectlist:
        if obj.phobostype == 'light':
            model['lights'][nUtils.getObjectName(obj)] = deriveLight(obj)

    # gather submechanism information from links
    # log("Parsing submechanisms...", "INFO")

    return model
