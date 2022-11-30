import xml.etree.ElementTree as ET
from copy import deepcopy
from ..utils.commandline_logging import get_logger
log = get_logger(__name__)


class Linkable(object):
    _type_dict = {
        "link": "links",
        "joint": "joints",
        "frame": "links",
        "material": "materials",
        "relative_to": "links",
    }
    _related_robot_instance = None
    _class_variables = []

    def __init__(self):
        self._class_linkables = [var for var in self._class_variables if var in self.type_dict.keys() and not var.startswith("_")]

    def set_unique_name(self, value):
        raise NotImplementedError("Not implemented for "+str(type(self)))

    def __str__(self):
        """
        Get's the unique name.
        By using __str__ we can call this on the Linkable instance and
        when the variable already holds the unique string.
        """
        raise NotImplementedError("__str__ not implemented for "+str(type(self)))

    def _converter(self, varname, new_value):
        if self._related_robot_instance is None or isinstance(new_value, Representation):
            return new_value
        vtype = self.type_dict[varname].lower()
        converted = self._related_robot_instance.get_aggregate(f"{vtype}", new_value)
        if converted is None and new_value is not None:
            log.warning(f"There is no {vtype} with name {new_value} in {self._related_robot_instance.name}; setting {varname} to None")
            log.warning(f"Available are: {repr([str(x) for x in getattr(self._related_robot_instance, vtype)])}")
            raise AssertionError(f"{str(type(self))}, can not convert {new_value} to value type {vtype}")
        return converted

    def _attr_get_name(self, attribute):
        if getattr(self, "_" + attribute) is None:
            return None
        if type(getattr(self, "_" + attribute)) == list:
            return [str(x) for x in getattr(self, "_" + attribute)]
        return str(getattr(self, "_" + attribute))

    def _attr_set_name(self, attribute, new_value, no_check=False):
        if type(new_value) == list and all([type(v) == str or isinstance(v, Linkable) for v in new_value]):
            setattr(self, "_" + attribute, [self._converter(attribute, str(v)) for v in new_value])
        elif type(new_value) == str:
            setattr(self, "_" + attribute, self._converter(attribute, new_value))
        elif isinstance(new_value, Linkable):
            if self._related_robot_instance is not None:
                existing = self._related_robot_instance.get_aggregate(self.type_dict[attribute].lower(), str(new_value))
                if existing is None:
                    self._related_robot_instance.add_aggregate(self.type_dict[attribute].lower(), new_value)
                    new_value.link_with_robot(self._related_robot_instance, check_linkage_later=True)
                    setattr(self, "_" + attribute, new_value)
                    log.debug(f"Added {type(new_value)} {str(new_value)} to robot")
                elif id(new_value) != id(existing):
                    if (hasattr(new_value, "is_delegate") and new_value.is_delegate()) or new_value.equivalent(existing):
                        setattr(self, "_" + attribute, existing)
                    else:
                        new_mat_name = new_value.name
                        index = 1
                        while self._related_robot_instance.get_material(new_mat_name) is not None:
                            new_mat_name = new_value.name + "_" + str(index)
                            index += 1
                        log.warning(f"Ambiguous {type(new_value)} in {str(self)} renamed {new_value.name} to {new_mat_name}")
                        new_value.name = new_mat_name
                    new_value.link_with_robot(self._related_robot_instance, check_linkage_later=True)
                    setattr(self, "_" + attribute, new_value)
            else:
                setattr(self, "_" + attribute, new_value)
        elif new_value is not None:
            raise RuntimeError(f"Can't deal with value of type {repr(type(new_value))} during link_with_robot()! Value: {new_value}")
        if self._related_robot_instance is not None and not no_check:
            self.check_linkage(attribute=attribute)

    def link_with_robot(self, robot, check_linkage_later=False):
        # if self._related_robot_instance is None:
        assert robot is not None
        self._related_robot_instance = robot
        for attribute in self._class_linkables:
            self._attr_set_name(attribute, getattr(self, "_" + attribute), no_check=True)
            # if getattr(self, "_" + attribute) is not None:
            #     if isinstance(getattr(self, "_" + attribute), Linkable):
            #         getattr(self, "_" + attribute).link_with_robot(robot, check_linkage_later=True)
            #     elif isinstance(getattr(self, "_" + attribute), list):
            #         for v in getattr(self, "_" + attribute):
            #             if isinstance(v, Linkable):
            #                 v.link_with_robot(robot, check_linkage_later=True)
        for var in self._class_variables:
            if isinstance(getattr(self, var), Linkable):
                getattr(self, var).link_with_robot(robot, check_linkage_later=True)
            elif isinstance(getattr(self, var), list):
                for v in getattr(self, var):
                    if isinstance(v, Linkable):
                        v.link_with_robot(robot, check_linkage_later=True)
        if not check_linkage_later:
            assert self.check_linkage()

    def unlink_from_robot(self, check_linkage_later=False):
        # if self._related_robot_instance is not None:
        self._related_robot_instance = None
        for attribute in self._class_linkables:
            # if getattr(self, "_" + attribute) is not None:
            #     if isinstance(getattr(self, "_" + attribute), Linkable):
            #         getattr(self, "_" + attribute).unlink_from_robot(check_linkage_later=True)
            #     elif isinstance(getattr(self, "_" + attribute), list):
            #         for v in getattr(self, "_" + attribute):
            #             if isinstance(v, Linkable):
            #                 v.unlink_from_robot(check_linkage_later=True)
            self._attr_set_name(attribute, self._attr_get_name(attribute))
            assert type(getattr(self, "_"+attribute)) in [str, list, type(None)], attribute+" "+str(getattr(self, "_"+attribute))+str(type(getattr(self, "_"+attribute)))
        for var in self._class_variables:
            if isinstance(getattr(self, var), Linkable):
                getattr(self, var).unlink_from_robot(check_linkage_later=True)
            elif isinstance(getattr(self, var), list):
                for v in getattr(self, var):
                    if isinstance(v, Linkable):
                        v.unlink_from_robot(check_linkage_later=True)
        if not check_linkage_later:
            assert self.check_unlinkage()

    def check_linkage(self, attribute=None):
        linked = self._related_robot_instance is not None
        assert linked, type(self)
        _class_attributes = self._class_linkables
        if attribute is not None:
            _class_attributes = [var for var in self._class_linkables if var == attribute]
        else:
            for var in self._class_variables:
                if isinstance(getattr(self, var), Linkable):
                    linked &= getattr(self, var).check_linkage()
                elif isinstance(getattr(self, var), list):
                    for v in getattr(self, var):
                        if isinstance(v, Linkable):
                            linked &= v.check_linkage()
        for attribute in _class_attributes:
            linked &= (
                getattr(self, "_" + attribute) is None or
                (isinstance(getattr(self, "_" + attribute), Linkable) and getattr(self, "_" + attribute)._related_robot_instance is not None) or
                (isinstance(getattr(self, "_" + attribute), list) and
                 all([(isinstance(x, Linkable) and x._related_robot_instance is not None) or x is None for x in getattr(self, "_" + attribute)]))
            )
            # if not linked:
            #     print(
            #         getattr(self, "_" + attribute) is None,
            #         isinstance(getattr(self, "_" + attribute), Linkable),
            #         isinstance(getattr(self, "_" + attribute), Linkable) and getattr(self, "_" + attribute)._related_robot_instance is not None,
            #         isinstance(getattr(self, "_" + attribute), list),
            #         getattr(self, "_" + attribute),
            #         [(isinstance(x, Linkable) and x._related_robot_instance is not None) for x in getattr(self, "_" + attribute)] if isinstance(getattr(self, "_" + attribute), list) else None
            #     )
            assert linked, f"Attribute {attribute} of {type(self)} {str(self) if self.stringable() else ''} is not linked. type: {type(getattr(self, '_' + attribute))}"
        return linked

    def check_unlinkage(self, attribute=None):
        unlinked = self._related_robot_instance is None
        assert unlinked, type(self)
        _class_attributes = self._class_linkables
        if attribute is not None:
            _class_attributes = [var for var in self._class_linkables if var == attribute]
        else:
            for var in self._class_variables:
                if isinstance(getattr(self, var), Linkable):
                    unlinked &= getattr(self, var).check_unlinkage()
                elif isinstance(getattr(self, var), list):
                    for v in getattr(self, var):
                        if isinstance(v, Linkable):
                            unlinked &= v.check_unlinkage()
        for attribute in _class_attributes:
            unlinked &= (
                getattr(self, "_" + attribute) is None or
                type(getattr(self, "_" + attribute)) == str or
                (isinstance(getattr(self, "_" + attribute), list) and
                 all([type(x) == str or x is None for x in getattr(self, "_" + attribute)]))
            )
            assert unlinked, f"Attribute {attribute} of {type(self)} {str(self) if self.stringable() else ''} is still linked. type: {type(getattr(self, '_' + attribute))}"
        return unlinked

    def is_related_to(self, entity, pure=False):
        if type(entity) not in [list, tuple, set]:
            entity = [entity]
        entity = [str(e) for e in entity]
        out = []
        for attribute in self._class_linkables:
            value = self._attr_get_name(attribute)
            if type(value) == list:
                out += [str(v) in entity for v in value]
            elif value is None:
                continue
            else:
                out.append(str(value) in entity)
        if pure:
            return all(out)
        else:
            return any(out)

    def duplicate(self, to_robot=None):
        _robot = self._related_robot_instance
        self.unlink_from_robot()
        out = deepcopy(self)
        if _robot is not None:
            self.link_with_robot(_robot)
        if to_robot is not None:
            out.link_with_robot(to_robot)
        return out

    def equivalent(self, other):
        return id(self) == id(other)

    def stringable(self):
        return True

    def is_empty(self):
        return False


class Representation(Linkable):
    factory = {}

    def __init__(self):
        super(Representation, self).__init__()

    @classmethod
    def create(cls, *args, **kwargs):
        try:
            return cls(*args, **kwargs)
        except TypeError as e:
            log.error(cls.__name__)
            raise e

    @classmethod
    def from_xml(cls, xml: ET.Element, dialect, **kwargs):
        return cls.factory[dialect].from_xml(cls, xml, **kwargs)

    @classmethod
    def from_xml_string(cls, xml: str, dialect):
        return cls.factory[dialect].from_xml_string(cls, xml)

    def to_xml(self, dialect, **kwargs) -> ET.Element:
        return self.factory[dialect].to_xml(self, float_fmt_dict=kwargs["float_fmt_dict"] if "float_fmt_dict" in kwargs else None)

    def to_xml_string(self, dialect, **kwargs) -> ET.Element:
        return self.factory[dialect].to_xml_string(self, float_fmt_dict=kwargs["float_fmt_dict"] if "float_fmt_dict" in kwargs else None)

    def sort_string(self, dialect=None) -> str:
        prefix = type(self).__name__ if dialect is None else self.to_xml(dialect).tag
        if hasattr(self, "name"):
            return prefix + ":" + str(self)
        else:
            return prefix
