import os.path

import numpy as np

from phobos.io.smurf_reflection import SmurfBase
from phobos.smurf import Smurf
from phobos.utils.transform import rpy_to_matrix, quaternion_to_matrix

WORLD = "world"
PARENT = "parent"


class BaseEntity(SmurfBase):
    def __init__(self, file, parent=None, anchor=None, transformation=np.identity(4), root=False, **kwargs):
        if "name" not in kwargs.keys() or kwargs["name"] is None:
            name = os.path.basename(file).split(".")[0]
        else:
            name = kwargs["name"].strip()
        super(BaseEntity, self).__init__(file=file, **kwargs)
        self.root = root
        self.parent = parent.strip() if parent is not None else WORLD
        self.parent_entity = parent if parent is not None else WORLD
        self.parent_link = None
        self.set_parent(parent)
        self.anchor = anchor
        self.transformation = transformation
        if "position" in kwargs.keys() or "rotation" in kwargs.keys():
            self.transformation = BaseEntity.parse_transformation(
                position=kwargs["position"] if "position" in kwargs.keys() else None,
                rotation=kwargs["rotation"] if "rotation" in kwargs.keys() else None,
            )
        self.annotations = kwargs
        self.children = []
        self.excludes += ["parent_entity", "parent_link", "transformation", "annotations", "children"]
        if anchor is None:
            self.excludes += ["anchor"]
        if parent is None:
            self.excludes += ["parent"]

    def set_parent(self, parent):
        self.parent = parent.strip() if parent is not None else WORLD
        self.parent_entity = self.parent.strip()
        self.parent_link = None
        if "::" in self.parent:
            self.parent_entity = self.parent.split("::")[0]
            self.parent_link = self.parent.split("::")[1]

    @staticmethod
    def get_type(entity_entry):
        if "type" in entity_entry.keys():
            _type = entity_entry["type"]
        else:
            _type = os.path.basename(entity_entry["file"]).split(".")[1]
        return _type.lower()

    @staticmethod
    def parse_transformation(position=None, rotation=None):
        transform = np.identity(4)
        if type(rotation) is list and len(rotation) == 4:
            transform[0:3, 0:3] = quaternion_to_matrix(rotation)
        elif type(rotation) is list and len(rotation) == 3:
            transform[0:3, 0:3] = rpy_to_matrix(rotation)
        elif rotation is not None:
            transform[0:3, 0:3] = rpy_to_matrix([0, 0, rotation])
        if position is not None:
            transform[0:3, 3] = position
        return transform


class Entity(BaseEntity):
    def __init__(self, name, robot, parent=None, anchor=None, transformation=np.identity(4), root=False, **kwargs):
        if "file" not in kwargs.keys():
            if isinstance(robot, Smurf):
                kwargs["file"] = robot.smurffile
            else:
                kwargs["file"] = robot.xmlfile
        kwargs["name"] = name
        super(Entity, self).__init__(parent=parent, anchor=anchor, transformation=transformation, root=root, **kwargs)
        self.robot = robot
        self.excludes += ["robot"]

    @staticmethod
    def from_yaml(entity_entry, root_path):
        if "name" not in entity_entry.keys():
            entity_entry["name"] = os.path.basename(entity_entry["file"]).split(".")[0]

        abs_file = os.path.realpath(os.path.join(root_path, entity_entry["file"]))

        _type = Entity.get_type(entity_entry)
        if _type in ["urdf", "smurf"]:
            robot = Smurf(inputfile=abs_file)
        elif _type in ["smurfs", "smurfa"]:
            raise AssertionError("You have to resolve smurfs/smurfa before loading entity")
        else:
            raise AssertionError("Unknown entity type!")
        return Entity(robot=robot, **entity_entry)
