import os

from . import Robot
from ..defs import dump_json, load_json
from ..io import representation, scenes
from ..io.base import Representation
from ..io.smurf_reflection import SmurfBase
from ..io.xml_factory import plural as _plural, singular as _singular

__IMPORTS__ = [x for x in dir() if not x.startswith("__")]


class Entity(Representation, SmurfBase):
    _class_variables = ["origin"]

    def __init__(self, name=None, world=None, model=None, file=None, type=None, origin=None, frames=None, anchor=None,
                 **kwargs):
        Representation.__init__(self)
        self.model = _singular(model)
        self.origin = _singular(origin) if origin is not None else representation.Pose()
        self._file = file
        if model is None and file is not None:
            self.model = Robot(inputfile=file)
        assert self.model is not None
        if name is None:
            name = os.path.basename(file.rsplit(".", 1)[0])
        if world is not None:
            self.link_with_world(world)
        self._frames = []
        for frame in _plural(frames):
            self._frames.append(frame)
        self.anchor = anchor
        SmurfBase.__init__(self, name=name, returns=["name", "type", "parent", "position", "rotation", "anchor", "root", "file"], **kwargs)
        self.excludes += ["origin", "model"]

    def link_with_world(self, world, check_linkage_later=False):
        self.model._related_world_instance = world
        self.model._related_entity_instance = self
        self.model.link_entities()
        # [Todo v2.1.0]
        # go through all frame entities and check whether there are frames explicitly defined that are already there by
        # another entity
        if not check_linkage_later:
            self.check_linkage()

    def unlink_from_world(self, check_linkage_later=False):
        self.model._related_world_instance = None
        self.model._related_entity_instance = None
        self.model.unlink_entities()
        # [Todo v2.1.0]
        #  Go through all linkables (poses should be sufficient) and check whether there are still references to other
        #  entities and create frames accordingly to repplace them
        if not check_linkage_later:
            self.check_unlinkage()

    def check_linkage(self, attribute=None):
        return self.model._related_world_instance is not None and self.model._related_entity_instance is not None \
               and self.model.check_linkage()

    def check_unlinkage(self, attribute=None):
        return self.model._related_world_instance is None and self.model._related_entity_instance is None \
               and self.model.check_unlinkage()

    @property
    def file(self):
        return self.model.smurffile if self.model.smurffile is not None else self.model.xmlfile

    @property
    def type(self):
        return self.file.rsplit(".",1)[-1]

    @type.setter
    def type(self, value):
        pass

    @property
    def frames(self):
        out = self._frames
        for f_obj in self.model.links + self.model.joints:
            out.append(scenes.Frame(
                name=str(self)+"::"+str(f_obj),
                attached_to=str(self)+"::"+str(f_obj),
                origin=representation.Pose(relative_to=str(self)+"::"+str(f_obj))
            ))
        return out

    @property
    def position(self):
        if self.origin is None:
            return None
        pos = self.origin.position if self.origin.position is not None else [0.0, 0.0, 0.0]
        return {"x": pos[0], "y": pos[1], "z": pos[2]}

    @position.setter
    def position(self, val):
        if self.origin is None:
            self.origin = representation.Pose()
        self.origin.xyz = [val["x"], val["y"], val["z"]]

    @property
    def rotation(self):
        if self.origin is None:
            return None
        return self.origin.quaternion_dict

    @rotation.setter
    def rotation(self, val):
        if self.origin is None:
            self.origin = representation.Pose()
        self.origin.rotation = val

    @property
    def parent(self):
        if self.origin is not None:
            return self.origin.relative_to
        return None

    @parent.setter
    def parent(self, val):
        if self.origin is None:
            self.origin = representation.Pose()
        self.origin.relative_to = val

    @property
    def placement_frame(self):
        # [ToDo v2.1.0] SDF can do here more
        return str(self.model.get_root())

    @property
    def anchor(self):
        return self._anchor if self._anchor not in ["NONE", "PARENT", "WORLD"] else self._anchor.lower()

    @anchor.setter
    def anchor(self, value):
        if value is None:
            self._anchor = "NONE"
        elif value.lower() in ["none", "parent", "world"]:
            self._anchor = value.upper()
        else:
            self._anchor = value

    @property
    def root(self):
        if self.model._related_world_instance is not None and self in self.model._related_world_instance.get_root_entities():
            return True
        return None

    @root.setter
    def root(self, val):
        pass


class Arrangement(Representation, SmurfBase):
    def __init__(self, inputfile=None, entities=None, frames=None):
        super(Arrangement, self).__init__()
        self.entities = _plural(entities)
        self.inputfile = inputfile
        self._frames = _plural(frames)
        if self.inputfile is not None:
            ext = self.inputfile.lower().rsplit(".", 1)[-1]
            if ext == "sdf":
                # [Todo v2.1.0]
                raise NotImplementedError
            elif ext in ["smurfa", "smurfs"]:
                with open(self.inputfile, "r") as f:
                    file_dict = load_json(f.read())
                entity_defs = file_dict.get("entities", file_dict.get("smurfa", file_dict.get("smurfs", [])))
                for e_def in entity_defs:
                    self.add_entity(Entity(world=self, **e_def))
            else:
                raise IOError(f"The given file has an extension ({ext}) that cannot be parsed as Arrangement.")
        self.excludes += ["inputfile"]
        self.link_entities()

    def add_robot(self, name, robot, origin=None, anchor=None):
        self.add_entity(Entity(
            name=name,
            model=robot,
            origin=origin,
            anchor=anchor
        ))

    def add_entity(self, entity):
        assert isinstance(entity, Entity)
        assert self.get_aggregate("entities", str(entity)) is None, f"There is already an entity with the name {str(entity)}"
        self.entities.append(entity)

    def get_aggregate(self, typeName, elem):
        elem = str(elem)
        if typeName.startswith("frame"):
            if elem == "WORLD":
                return scenes.Frame(name="WORLD")
            if "::" in elem:
                entity, link = elem.split("::")
                assert self.get_aggregate("entities", entity) is not None
                assert entity.get_aggregate("links", link) is not None
                return scenes.Frame(
                    name=elem,
                    attached_to=elem,
                    origin=representation.Pose(relative_to=elem)
                )
            else:
                for e in self._frames:
                    if str(e) == elem:
                        return e
        elif len(getattr(self, typeName, None)) == list:
            for e in getattr(self, typeName):
                if str(e) == elem:
                    return e
        elif "::" in elem:
            entity, elem = elem.split("::")
            return self.get_aggregate("entities", entity).get_aggregate(typeName, elem)
        elif not hasattr(self, typeName):
            raise TypeError(f"World has no {typeName}")
        else:
            return None

    def link_entities(self):
        for e in self.entities:
            e.link_with_world(self, check_linkage_later=False)

    def unlink_entities(self):
        for e in self.entities:
            e.unlink_from_world(self, check_linkage_later=False)

    def check_linkage(self, attribute=None):
        out = True
        for e in self.entities:
            out &= e.check_linkage()
        return out

    def check_unlinkage(self, attribute=None):
        out = True
        for e in self.entities:
            out &= e.check_unlinkage()
        return out

    def is_empty(self):
        return len(self.entities) == 0

    def has_one_root(self):
        return len(self.get_root_entities()) == 1

    def get_root_entities(self):
        return [e for e in self.entities if e._anchor in ["NONE", "WORLD"]]

    def assemble(self, root_entity=None):
        if root_entity is None:
            root_entities = self.get_root_entities()
            assert len(root_entities) == 1, "assemble() is determined for Assmeblies with only one root"
            root_entity = root_entities[0]
        elif type(root_entity) == str:
            root_entity = self.get_aggregate("entities", root_entity)
            assert root_entity is not None, f"No entity with name {root_entity} found."

        assembly = root_entity.model.duplicate()
        assembly.rename_all(prefix=root_entity.name + "_")
        entities_in_tree = [root_entity]

        attached = 1
        while attached > 0:
            attached = 0
            for entity in self.entities:
                if entity._anchor in ["NONE, WORLD"] or entity in entities_in_tree:
                    continue
                if entity._anchor == "PARENT":
                    parent_entity, parent_link = entity.parent.split("::", 1)
                else:
                    parent_entity, parent_link = entity._anchor.split("::", 1)
                if parent_entity in [str(e) for e in entities_in_tree]:
                    parent_link = assembly.get_link(parent_entity+"_"+parent_link)
                    assert parent_link is not None
                    attach_model = entity.model.duplicate()
                    attach_model.rename_all(prefix=entity.name + "_")
                    entity.origin.relative_to = entity.origin.relative_to.replace("::", "_")
                    assembly.attach(
                        other=attach_model,
                        joint=representation.Joint(
                            name=str(parent_entity)+"2"+str(entity),
                            parent=parent_link,
                            child=attach_model.get_root(),
                            type="fixed",
                            origin=entity.origin
                        )
                    )
                    entities_in_tree.append(entity)
                    attached += 1

        assembly.relink_entities()
        return assembly

    def export_sdf(self, use_includes=True):
        # we need to find a solution of exporting entities either as include or as full model
        raise NotImplementedError

    def export_smurfa(self, outputfile):
        assert self.check_linkage()
        assert outputfile.endswith("smurfa")
        out = self.to_yaml()
        out["entities"] = [e.to_yaml() for e in self.entities]
        if not os.path.exists(os.path.dirname(os.path.abspath(outputfile))):
            os.makedirs(os.path.dirname(os.path.abspath(outputfile)), exist_ok=True)
        with open(outputfile, "w") as f:
            f.write(dump_json(out))


class World(Arrangement):
    def __init__(self, inputfile=None, entities=None, frames=None, physics=None, environment=None):
        super(World, self).__init__(inputfile=inputfile, entities=entities, frames=frames)
        self.physics = _singular(physics)
        self.environment = _singular(environment)

    def export_sdf(self, use_includes=True):
        # we need to find a solution of exporting entities either as include or as full model
        raise NotImplementedError

    def expot_smurfs(self, outputfile):
        assert self.check_linkage()
        assert outputfile.endswith("smurfs")
        out = {
            "smurfs": {
                e.to_yaml() for e in self.entities
            }
        }
        if self.environment:
            out["environment"] = self.environment.to_yaml()
        if self.physics:
            out["physics"] = self.physics.to_yaml()
        if not os.path.exists(os.path.dirname(outputfile)):
            os.makedirs(os.path.dirname(outputfile), exist_ok=True)
        with open(outputfile, "w") as f:
            f.write(dump_json(out))