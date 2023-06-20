import xml.etree.ElementTree as ET
from copy import deepcopy

from ..commandline_logging import get_logger

log = get_logger(__name__)


class Linkable(object):
    _type_dict = {  # map from variable name to the robot's instance list
        "link": "links",
        "joint": "joints",
        "frame": "links",
        "material": "materials",
        "relative_to": ["links", "joints"],
    }
    _related_robot_instance = None
    # _related_robot_instances need to have a _related_world_instance and a _related_entity_instance (where the latter has be set if the first is set)

    # _class_variables contains those properties which have to be scanned for linkables
    _class_variables = []
    _handle_ambiguous = True

    def __init__(self):
        self._class_linkables = [var for var in self._class_variables if var in self.type_dict.keys() and not var.startswith("_")]

    def set_unique_name(self, value):
        """
        Set's the unique name to the given value.
        If this class can be referenced by unique name (see stringable) this needs to be implemented
        """
        raise NotImplementedError("Not implemented for "+str(type(self)))

    def __str__(self):
        """
        Get's the unique name.
        By using __str__ we can call this on the Linkable instance and
        when the variable already holds the unique string.
        If this class can be referenced by unique name (see stringable) this needs to be implemented.
        """
        raise NotImplementedError("__str__ not implemented for "+str(type(self)))

    def stringable(self):
        """
        Whether this class has a unique_name and can be referenced by it when linking it to a robot class
        """
        return True

    def _converter(self, varname, new_value, allow_not_found=False, value_type=None):
        """
        Converts the string reference to a python-reference
        Args:
            varname: The variable name to be set
            new_value: the new value either a string or an object

        Returns:
            The corresponding object
        """
        if self._related_robot_instance is None:
            return new_value, None
        if isinstance(new_value, Representation):
            assert new_value._related_robot_instance is None or new_value._related_robot_instance == self._related_robot_instance
            # this way we ensure linking to the correct robot instance
            # if new_value._related_robot_instance is None:
            #     new_value.link_with_robot(self._related_robot_instance, check_linkage_later=True)
            # else:
            #     assert new_value._related_robot_instance == self._related_robot_instance
            return new_value, None
        vtypes = self.type_dict[varname] if value_type is None else value_type
        if type(vtypes) == str:
            vtypes = [vtypes]
        converted = None
        vtype = None
        for _vtype in vtypes:
            vtype = _vtype.lower()
            if self._related_robot_instance._related_world_instance is not None and "::" in new_value:
                if self._related_robot_instance._related_entity_instance+"::" in new_value:
                    converted = self._related_robot_instance.get_aggregate(f"{vtype}", new_value.rsplit("::", 1)[-1])
                else:
                    converted = self._related_robot_instance._related_world_instance.get_aggregate(f"{vtype}", new_value)
            else:
                converted = self._related_robot_instance.get_aggregate(f"{vtype}", new_value)
            if converted is not None:
                break
        if not allow_not_found and converted is None and new_value is not None:
            log.warning(f"There is no {vtype} with name {new_value} in {self._related_robot_instance.name}; setting {varname} to None")
            log.warning(f"Available are: {repr([str(x) for x in getattr(self._related_robot_instance, vtype)])}")
            raise AssertionError(f"{str(type(self))}, can not convert {new_value} to value type {vtype} for variable {varname}")
        return converted, vtype

    def _attr_get_name(self, attribute):
        if getattr(self, "_" + attribute) is None:
            return None
        if type(getattr(self, "_" + attribute)) == list:
            return [str(x) for x in getattr(self, "_" + attribute)]
        return str(getattr(self, "_" + attribute))

    def _attr_set_name(self, attribute, new_value, no_check=False):
        if type(new_value) == list and all([type(v) == str or isinstance(v, Linkable) for v in new_value]):
            setattr(self, "_" + attribute, [self._converter(attribute, v)[0] for v in new_value])
        elif type(new_value) == str:
            setattr(self, "_" + attribute, self._converter(attribute, new_value)[0])
        elif isinstance(new_value, Linkable):
            if self._related_robot_instance is not None:
                existing, vtype = self._converter(attribute, str(new_value), allow_not_found=True, value_type=new_value.__class__.__name__.lower())
                if existing is None:
                    self._related_robot_instance.add_aggregate(vtype, new_value)
                    new_value.link_with_robot(self._related_robot_instance, check_linkage_later=True)
                    setattr(self, "_" + attribute, new_value)
                    log.debug(f"Added {type(new_value)} {str(new_value)} to robot")
                elif id(new_value) != id(existing):
                    if (hasattr(new_value, "is_delegate") and new_value.is_delegate()) or new_value.equivalent(existing):
                        setattr(self, "_" + attribute, existing)
                    else:
                        new_name = new_value.name
                        index = 1
                        while self._related_robot_instance.get_aggregate(vtype, new_name) is not None:
                            new_name = str(new_value) + "_" + str(index)
                            index += 1
                        try:
                            this_name = str(self)
                        except NotImplementedError:
                            this_name = str(type(self))
                        log.warning(f"Ambiguous {type(new_value)} in {this_name} renamed {new_value.name} to {new_name}")
                        log.debug(f"Existing: {existing.__dict__}\nOld: {new_value.__dict__}")
                        assert self._handle_ambiguous
                        new_value.name = new_name
                    new_value.link_with_robot(self._related_robot_instance, check_linkage_later=True)
                    setattr(self, "_" + attribute, new_value)
                else:
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
        # log.debug(f"Linking {self.__class__} {id(self)}")
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
                # log.debug(f"Linking {var} of class {self.__class__} with id {id(getattr(self, var))}")
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
        """
        Checks whether all string-references have been replaced by python-references
        Args:
            attribute: If not None will check the linkage only for this attribute

        Returns:
            True if all references are python-references
        """
        linked = self._related_robot_instance is not None
        assert linked, self.__class__
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
                assert linked, f"Variable {var} of {type(self)} {str(self) if self.stringable() else repr(self)} is not linked."
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
        """
        Checks whether all references have been replaced by string-references
        Args:
            attribute: If not None will check the unlinkage only for this attribute

        Returns:
            True if all references are string-references
        """
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
        """
        Checks whether this instance holds a reference to any of the entities given.
        Args:
            entity: One or a list of many instances
            pure: if true, considers this as related to other if it all it's internal references are included in the entity list given
                  if false, a single relation is sufficient

        Returns:
            boolean
        """
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
        """
        Duplicates the current instance, like a deepcopy, but takes care of the linkage
        Args:
            to_robot: if not None the duplicate will be linked to that robot, if None the instance will be unlinked.

        Returns:
            A duplicate of this instance, unlinked if to_robot is None else linked to the specified robot
        """
        _robot = self._related_robot_instance
        self.unlink_from_robot()
        out = deepcopy(self)
        if _robot is not None:
            self.link_with_robot(_robot)
        if to_robot is not None:
            out.link_with_robot(to_robot)
        return out

    def equivalent(self, other):
        """
        Checks whether this instance is equivalent to the other instance.
        Should be reimplemented matching to the specific class.
        Args:
            other: The other instance

        Returns:
            boolean
        """
        return id(self) == id(other)

    def is_empty(self):
        """
        Should be reimplented for each class. And check whether it is an unfilled instance
        Returns:
            boolean
        """
        return False


class Representation(Linkable):
    """
    Base class for all entity representations which makes them Linkable and provides basic access to the XML-serialization.
    The detailed access is then done by class_factory() See phobos.io documentation for conceptual details.
    """

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
        try:
            return cls.factory[dialect].from_xml(cls, xml, **kwargs)
        except KeyError:
            raise LookupError(f"Class {cls.__name__} has no xml format defined for dialect '{dialect}'")

    @classmethod
    def from_xml_string(cls, xml: str, dialect):
        try:
            return cls.factory[dialect].from_xml_string(cls, xml)
        except KeyError:
            raise LookupError(f"Class {cls.__name__} has no xml format defined for dialect '{dialect}'")

    def to_xml(self, dialect, **kwargs) -> ET.Element:
        try:
            return self.factory[dialect].to_xml(self, **kwargs)
        except KeyError:
            raise LookupError(f"Class {self.__class__.__name__} has no xml format defined for dialect '{dialect}'")

    def to_xml_string(self, dialect, **kwargs) -> ET.Element:
        try:
            return self.factory[dialect].to_xml_string(self, **kwargs)
        except KeyError:
            raise LookupError(f"Class {self.__class__.__name__} has no xml format defined for dialect '{dialect}'")

    def sort_string(self, dialect=None) -> str:
        prefix = type(self).__name__ if dialect is None else self.to_xml(dialect).tag
        if hasattr(self, "name"):
            return prefix + ":" + str(self)
        else:
            return prefix
