import os.path
from copy import deepcopy

from .smurfscene import SMURFScene
from ..defs import dump_json
from ..io import representation
from ..utils import xml


# [Todo v2.1.0] Test and fix this
class Assembly(SMURFScene):
    def __init__(self, smurfassembly, copy_construct=False, output_dir=None, **kwargs):
        super().__init__(smurfassembly, copy_construct=copy_construct, output_dir=output_dir, **kwargs)
        if len(self.entities) > 1:
            raise AssertionError("Assembly has more than one root")
        if self.output_dir is None:
            raise AssertionError("Assembly needs either the smurfassembly or the output_dir arg set.")
        self._robot = None

    def merge(self, copy_meshes=False):
        self._robot = self.entities[0].robot.duplicate()
        mesh_dir = None
        # [ToDo] Review mesh handling
        if copy_meshes:
            mesh_dir = os.path.join(self.output_dir, "meshes")
        xml.adapt_mesh_pathes(self._robot, os.path.join(self.output_dir, "urdf"), copy_to=mesh_dir)

        def go_through_parts(children):
            for child in children:
                joint = representation.Joint(
                    name=child.name,
                    parent=child.parent_link if child.parent_link is not None else self.entities_by_name[
                        child.parent_entity].robot.get_root(),
                    child=child.robot.get_root(),
                    joint_type="fixed",
                    origin=representation.Pose.from_matrix(child.transformation)
                )
                crobot = child.robot.duplicate()
                xml.adapt_mesh_pathes(crobot, os.path.join(self.output_dir, "urdf"), copy_to=mesh_dir)

                self._robot.attach(crobot, joint)

        go_through_parts(self.entities[0].children)

        return self._robot

    @property
    def robot(self):
        if self._robot is None:
            raise AssertionError("You have to merge the assembly before you can access the robot element!")
        return self._robot

    @staticmethod
    def from_scene(scene, output_dir=None):
        assert len(scene.entities) == 1

        assembly = Assembly(None, copy_construct=True, output_dir=output_dir)
        assembly.scenefile = deepcopy(scene.scenefile)
        assembly.scene_name = deepcopy(scene.scene_name)
        assembly.scenedir = deepcopy(scene.scenedir)
        assembly.filedict = deepcopy(scene.filedict)
        assembly.entities = [e.duplicate() for e in scene.entities]
        assembly.entities_by_name = {}

        def go_through_parts(children):
            for child in children:
                assembly.entities_by_name[child.name] = child
                go_through_parts(child.children)

        go_through_parts(assembly.entities)

        return assembly

    @staticmethod
    def from_entities(entities, output_dir=None):
        assembly = Assembly(None, copy_construct=True, output_dir=output_dir)
        assembly.scenefile = None
        assembly.scene_name = None
        assembly.scenedir = None
        assembly.filedict = None
        assembly.entities = [e.duplicate() for e in entities]
        assembly.entities_by_name = {}

        def go_through_parts(children):
            for child in children:
                assembly.entities_by_name[child.name] = child
                go_through_parts(child.children)

        go_through_parts(assembly.entities)

        return assembly

    def export(self, outputfile=None):
        out = {"smurfa": []}
        for entity in self.entities_by_name.values():
            out["smurfa"].append(entity.to_yaml())
        with open(outputfile, "w") as f:
            f.write(dump_json(out, default_flow_style=False))
