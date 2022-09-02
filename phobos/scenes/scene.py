import numpy as np
import os
from .entity import WORLD, PARENT, Entity, BaseEntity
from ..defs import load_json, dump_json, dump_yaml


class Scene(object):
    """ A simple object to store all information about the scene, the smurfs and corresponding urdf
    """

    def __init__(self, smurfscene, copy_construct=False, output_dir=None, **kwargs):
        # Init the mother scene
        self.scenefile = os.path.abspath(smurfscene) if smurfscene is not None else None
        self.scene_name, extension = os.path.splitext(os.path.basename(self.scenefile)) \
            if self.scenefile is not None else None, None
        self.scenedir = os.path.dirname(self.scenefile) if self.scenefile is not None else None
        self.filedict = load_json(open(self.scenefile, "r").read()) if not copy_construct else None
        self.entities = []
        self.entities_by_name = {}
        if not copy_construct:
            self.recursive_parse(self.scenefile)
        self.output_dir = os.path.abspath(output_dir) if output_dir is not None else None
        if self.output_dir is None:
            self.output_dir = self.scenedir

    def recursive_parse(self, scenefile, transformation_offset=np.identity(4), connect_root_to=False):
        root_path = os.path.dirname(scenefile)
        smurf_dict = load_json(open(scenefile, "r").read())
        if "entities" not in smurf_dict.keys():
            smurf_dict["entities"] = []
        if "smurfs" not in smurf_dict.keys():
            smurf_dict["smurfs"] = []
        if "smurfa" not in smurf_dict.keys():
            smurf_dict["smurfa"] = []

        for entity_entry in smurf_dict["entities"] + smurf_dict["smurfs"] + smurf_dict["smurfa"]:
            _type = Entity.get_type(entity_entry)
            entity = BaseEntity(**entity_entry)

            def go_through_entities_for_transformation(children, T_=np.identity(4)):
                for child in children:
                    if entity.parent_entity == child.name:
                        if entity.parent_link is not None:
                            return T_.dot(child.transformation).dot(child.robot.get_transformation(entity.parent_link))
                        else:
                            return T_.dot(child.transformation)
                    elif len(child.children) != 0:
                        return go_through_entities_for_transformation(child.children, T_.dot(child.transformation))
                    else:
                        continue
                raise AssertionError("Parent " + entity.parent_entity + " of " + entity.name + "not yet loaded!")

            if entity.parent_entity not in self.entities_by_name.keys() and entity.parent_entity != WORLD:
                raise AssertionError(
                    "You have to sort your entities in a way that the parent is defined before the child.")
            if _type in ["urdf", "smurf"]:
                entity = Entity.from_yaml(entity_entry, root_path)
                if entity.anchor == WORLD or entity.anchor is None:
                    if entity.parent_entity == WORLD:
                        entity.transformation = transformation_offset.dot(entity.transformation)
                    else:
                        T = go_through_entities_for_transformation(self.entities, np.identity(4))
                        entity.transformation = T.dot(entity.transformation)
                    self.entities += [entity]
                    self.entities_by_name[entity.name] = entity
                elif entity.anchor == PARENT:
                    if connect_root_to is not None and entity.root is True:
                        entity.set_parent(connect_root_to)

                    def go_through_entities(children):
                        for child in children:
                            if entity.parent_entity == child.name:
                                child.children += [entity]
                                return
                            elif len(child.children) != 0:
                                return go_through_entities(child.children)
                            else:
                                continue
                        log.error(self.entities_by_name.keys())
                        raise AssertionError(
                            "Parent " + entity.parent_entity + " of " + entity.name + " not yet loaded!")

                    go_through_entities(self.entities)
                    self.entities_by_name[entity.name] = entity
                else:
                    raise NotImplementedError
            elif _type in ["smurfs", "smurfa"]:
                if entity.anchor == WORLD or entity.anchor is None:
                    self.recursive_parse(os.path.join(root_path, entity_entry["file"]),
                                         transformation_offset.dot(entity.transformation))
                elif entity.anchor == PARENT:
                    self.recursive_parse(os.path.join(root_path, entity_entry["file"]), connect_root_to=entity.parent)

    def has_one_root(self):
        return len(self.entities) == 1

    def is_empty(self):
        return len(self.entities) == 0

    def export(self, outputfile=None):
        out = {"smurfs": []}
        for entity in self.entities_by_name.values():
            out["smurfs"].append(entity.to_yaml())
        with open(outputfile, "w") as f:
            f.write(dump_json(out, default_flow_style=False))
