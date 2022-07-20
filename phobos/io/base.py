import xml.etree.ElementTree as ET
from copy import deepcopy


class Linkable(object):
    _type_dict = {
        "link": "link",
        "joint": "joint",
        "frame": "frame",
        "motor": "motor",
        "material": "material",
        "relative_to": "link"
    }
    _related_robot_instance = None
    _class_variables = []

    def __init__(self):
        self._class_attributes = [var for var in self._class_variables if var in self.type_dict.keys() and not var.startswith("_")]

    def _converter(self, varname, new_value):
        if self._related_robot_instance is None or isinstance(new_value, Representation):
            return new_value
        vtype = self.type_dict[varname].lower()
        converted = getattr(self._related_robot_instance,
                       "get_" + vtype + ("_by_name" if vtype in ["collision", "visual"] else ""))(new_value)
        if converted is None and new_value is not None:
            print(f"WARNING: There is no {vtype} with name {new_value} in {self._related_robot_instance.name}; setting {varname} to None")
            print(f"Available are: {repr([x.name for x in getattr(self._related_robot_instance, vtype+'s')])}")
            raise RuntimeError
        return converted

    def _attr_get_name(self, attribute):
        if getattr(self, "_" + attribute) is None:
            return None
        if type(getattr(self, "_" + attribute)) == list:
            return [str(x) for x in getattr(self, "_" + attribute)]
        if type(getattr(self, "_" + attribute)) == str:
            return getattr(self, "_" + attribute)
        return getattr(self, "_" + attribute).name

    def _attr_set_name(self, attribute, new_value, no_check=False):
        if type(new_value) == list and all([type(v) == str or isinstance(v, Linkable) for v in new_value]):
            setattr(self, "_" + attribute, [self._converter(attribute, str(v)) for v in new_value])
        elif type(new_value) == str:
            setattr(self, "_" + attribute, self._converter(attribute, new_value))
        elif isinstance(new_value, Linkable):
            setattr(self, "_" + attribute, new_value)
        elif new_value is not None:
            raise RuntimeError(f"Can't deal with value of type {repr(type(new_value))} during link_with_robot()! Value: {new_value}")
        if self._related_robot_instance is not None and not no_check:
            self.check_linkage(attribute=attribute)

    def link_with_robot(self, robot, check_linkage_later=False):
        self._related_robot_instance = robot
        for attribute in self._class_attributes:
            self._attr_set_name(attribute, getattr(self, "_" + attribute), no_check=True)
        if not check_linkage_later:
            self.check_linkage()

    def unlink_from_robot(self):
        self._related_robot_instance = None
        for attribute in self._class_attributes:
            self._attr_set_name(attribute, self._attr_get_name(attribute))

    def check_linkage(self, attribute=None):
        _class_attributes = self._class_attributes
        if attribute is not None:
            _class_attributes = [var for var in self._class_attributes if var == attribute]
        for attribute in _class_attributes:
            assert getattr(self, "_" + attribute) is None or isinstance(getattr(self, "_" + attribute), Linkable) or \
            (isinstance(getattr(self, "_" + attribute), list) and all([isinstance(x, Linkable) for x in getattr(self, "_" + attribute)])),\
                f"{attribute} is not an instance of Linkable. type: {type(getattr(self, '_' + attribute))}"

    def is_related_to(self, entity, pure=False):
        if type(entity) not in [list, tuple, set]:
            entity = [entity]
        entity = [str(e) for e in entity]
        out = []
        for attribute in self._class_attributes:
            value = self._attr_get_name(attribute)
            if type(value) == list:
                out += [str(v) in entity for v in value]
            else:
                out.append(str(value) in entity)
        if pure:
            return all(out)
        else:
            return any(out)

    def duplicate(self):
        _robot = self._related_robot_instance
        self.unlink_from_robot()
        out = deepcopy(self)
        self.link_with_robot(_robot)
        return out

    def equivalent(self, other):
        return id(self) == id(other)


class Representation(Linkable):
    factory = {}

    def __init__(self):
        super(Representation, self).__init__()

    @classmethod
    def create(cls, *args, **kwargs):
        try:
            return cls(*args, **kwargs)
        except TypeError as e:
            print(cls.__name__)
            raise e

    @classmethod
    def from_xml(cls, xml: ET.Element, dialect, **kwargs):
        return cls.factory[dialect].from_xml(cls, xml, **kwargs)

    @classmethod
    def from_xml_string(cls, xml: str, dialect):
        return cls.factory[dialect].from_xml_string(cls, xml)

    def to_xml(self, dialect) -> ET.Element:
        return self.factory[dialect].to_xml(self)

    def to_xml_string(self, dialect) -> ET.Element:
        return self.factory[dialect].to_xml_string(self)

    def sort_string(self) -> str:
        if hasattr(self, "name"):
            return type(self).__name__ + ":" + self.name
        else:
            return type(self).__name__
