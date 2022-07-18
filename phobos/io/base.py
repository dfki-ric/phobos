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

    def __init__(self, robot=None):
        self._class_attributes = [var for var in self._class_variables if var in self.type_dict.keys() and not var.startswith("_")]
        if robot is not None:
            self.link_with_robot(robot)

    def _converter(self, varname, new_value):
        if self._related_robot_instance is None or isinstance(new_value, Representation):
            return new_value
        vtype = self.type_dict[varname].lower()
        converted = getattr(self._related_robot_instance,
                       "get_" + vtype + ("_by_name" if vtype in ["collision", "visual"] else ""))(new_value)
        if converted is None and new_value is not None:
            print(f"WARNING: There is no {vtype} with name {new_value} in {self._related_robot_instance.name} setting {varname} to None")
        return converted

    def _attr_get_name(self, attribute):
        if getattr(self, "_" + attribute) is None:
            return None
        if type(getattr(self, "_" + attribute)) == list:
            return [x.name for x in getattr(self, "_" + attribute)]
        if type(getattr(self, "_" + attribute)) == str:
            return getattr(self, "_" + attribute)
        return getattr(self, "_" + attribute).name

    def _attr_set_name(self, attribute, new_value):
        if type(new_value) == list and all([type(v) == str for v in new_value]):
            setattr(self, "_" + attribute, [self._converter(attribute, v) for v in new_value])
        elif type(new_value) == str:
            setattr(self, "_" + attribute, self._converter(attribute, new_value))
        elif isinstance(new_value, Linkable):
            setattr(self, "_" + attribute, new_value)
        elif new_value is not None:
            raise RuntimeError("Can't deal with value of type " + repr(type(new_value)) + " during link_with_robot()!")
        if self._related_robot_instance is not None:
            self.check_linkage(attribute=attribute)

    def link_with_robot(self, robot, check_linkage_later=False):
        self._related_robot_instance = robot
        for attribute in self._class_attributes:
            self._attr_set_name(attribute, getattr(self, "_" + attribute))
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
            assert getattr(self, "_" + attribute) is None or isinstance(getattr(self, "_" + attribute), Linkable),\
                f"{attribute} is not an instance of Linkable. type: {type(getattr(self, '_' + attribute))}"

    def is_related_to(self, entity):
        if isinstance(entity, Linkable):
            _entity = entity.name
        elif type(entity) == list:
            return any([self.is_related_to(e) for e in entity])
        else:
            _entity = entity
        for attribute in self._class_attributes:
            value = self._attr_get_name(attribute)
            if (type(value) == list and _entity in value) or _entity == value:
                return True
        return False

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

    def __init__(self, robot=None):
        super(Representation, self).__init__(robot=robot)

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
