import xml.etree.ElementTree as ET


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

    def __init__(self, robot=None, target_type=None):
        if target_type is not None:
            self._type_dict["targets"] = target_type

        if robot is not None:
            self.link_with_robot(robot)

    def _converter(self, varname, new_value):
        if self._related_robot_instance is None or isinstance(new_value, Representation):
            return new_value
        vtype = self._type_dict[varname].lower()
        return getattr(self._related_robot_instance,
                       "get_" + vtype + ("_by_name" if vtype in ["collision", "visual"] else ""))(new_value)

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

    def link_with_robot(self, robot):
        self._related_robot_instance = robot
        class_attributes = [var for var in dir(self.__class__) if var in self.__class__._type_dict.keys() and not var.startswith("_") and not callable(getattr(self.__class__, var))]
        for attribute in class_attributes:
            self._attr_set_name(attribute, getattr(self, "_" + attribute))
            assert isinstance(getattr(self, "_" + attribute), Linkable),\
                f"{attribute} is not an instance of Linkable. type: {type(getattr(self, '_' + attribute))}"


class Representation(Linkable):
    factory = {}

    def __init__(self, robot=None, target_type=None):
        super(Representation, self).__init__(robot=robot, target_type=target_type)

    @classmethod
    def from_xml(cls, xml: ET.Element, dialect):
        return cls.factory[dialect].from_xml(cls, xml)

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
