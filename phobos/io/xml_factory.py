try:
    from lxml import etree as ET
except ImportError:
    from xml.etree import ElementTree as ET

import json
from typing import List

import numpy as np
import pkg_resources

from .base import Representation, Linkable
from ..utils.misc import to_pretty_xml_string

FORMATS = json.load(open(pkg_resources.resource_filename("phobos", "data/xml_formats.json"), "r"))


def is_int(val):
    if type(val) == int:
        return True
    try:
        int(val)
        return True
    except ValueError:
        return False


def is_float(val):
    if type(val) == float:
        return True
    try:
        float(val)
        return True
    except ValueError:
        return False


def get_class(classname):
    from . import representation
    from . import sensor_representations
    if hasattr(representation, classname) and classname not in representation.__IMPORTS__:
        cls = getattr(representation, classname)
    elif hasattr(sensor_representations, classname) and classname not in sensor_representations.__IMPORTS__:
        cls = getattr(sensor_representations, classname)
    else:
        raise AssertionError(f"The class {classname} is not None to the XML-Factory")
    assert callable(cls) or issubclass(cls,
                                       Representation), f"The class {classname} is not a valid Representation instance"
    return cls


def get_var(object, varname_string):
    if "." in varname_string:
        var = object
        for var_part in varname_string.split("."):
            var = getattr(var, var_part)
    else:
        var = getattr(object, varname_string)
    return var


class XMLDefinition(object):
    def __init__(self, dialect, tag, value=None, attributes=None, children=None, value_children=None,
                 attribute_children=None, nested_children=None):
        self.dialect = dialect
        self.xml_tag = tag
        self.xml_value = value
        self.xml_attributes = attributes if attributes else {}
        self.xml_children = children if children else {}
        for _, child_def in self.xml_children.items():
            child_def["class"] = get_class(child_def["classname"])
        self.xml_value_children = value_children if value_children else {}
        self.xml_attribute_children = attribute_children if attribute_children else {}
        self.xml_nested_children = nested_children if nested_children else {}
        for tag, nest in self.xml_nested_children.items():
            if "tag" not in nest.keys():
                nest["tag"] = tag
            self.xml_nested_children[tag] = XMLDefinition(dialect, **nest)

    def to_xml(self, object, float_fmt_dict=None, **kwargs):
        """
        Serializes the given object ot xml
        Args:
            object: the object ot serialize to xml
            float_fmt_dict: A dictionary of tag/attribute name to float format string (Style: '%.6f')
               For simple attributes the attributename is taken, for all others (attribute_children, value_children, ...) the tag

        Returns:
            An XML object
        """
        # attributes
        if float_fmt_dict is None:
            float_fmt_dict = {}
        attrib = {}
        for attname, varname in self.xml_attributes.items():
            val = get_var(object, varname)
            if val is not None:
                attrib[attname] = self._serialize(val, float_fmt=float_fmt_dict.get(attname, float_fmt_dict.get("default", None)))
        out = ET.Element(self.xml_tag, attrib=attrib)
        # value
        if self.xml_value is not None:
            assert all([x == {} for x in [self.xml_children, self.xml_value_children, self.xml_attribute_children,
                                          self.xml_nested_children]])
            val = get_var(object, self.xml_value)
            out.text = self._serialize(val, float_fmt=float_fmt_dict.get(self.xml_tag, float_fmt_dict.get("default", None)))
            if val is not None:
                return out
            else:
                return
        # normal children
        children = []
        for _, var in self.xml_children.items():
            obj = get_var(object, var["varname"])
            if type(obj) == list:
                children += [o for o in obj if o is not None and (not isinstance(o, Representation) or not o.is_empty())]
            elif isinstance(obj, var["class"]):
                children += [obj]
            elif hasattr(object, "_"+var["varname"]):
                _obj = getattr(object, "_"+var["varname"])
                if _obj is not None and (not isinstance(_obj, Representation) or not _obj.is_empty()):
                    children += [_obj]
        for child in sorted(children, key=lambda x: x.sort_string()):
            try:
                e = child.to_xml(self.dialect)
                if e is not None:
                    out.append(e)
            except KeyError as error:
                if self.dialect not in child.factory.keys():
                    pass
                else:
                    raise error
        # children that are created from a simple property and have only attributes
        for tag, attribute_map in self.xml_attribute_children.items():
            _attrib = {attname: self._serialize(get_var(object, varname), float_fmt=float_fmt_dict.get(tag, float_fmt_dict.get("default", None)))
                       for attname, varname in attribute_map.items() if get_var(object, varname) is not None}
            if len(_attrib) == 0:
                continue
            e = ET.Element(tag, attrib=_attrib)
            out.append(e)
        # children that have the a value as text
        for tag, varname in self.xml_value_children.items():
            val = get_var(object, varname)
            if val is not None:
                if type(val) == list and all([not is_int(v) and not is_float(v) and type(v) == str for v in val]):
                    for v in val:
                        e = ET.Element(tag)
                        _t = self._serialize(v, float_fmt=float_fmt_dict.get(tag, float_fmt_dict.get("default", None)))
                        try:
                            e.text = _t
                        except TypeError as error:
                            print(_t, type(_t))
                            raise error
                        out.append(e)
                else:
                    e = ET.Element(tag)
                    _t = self._serialize(val, float_fmt=float_fmt_dict.get(tag, float_fmt_dict.get("default", None)))
                    try:
                        e.text = _t
                    except TypeError as error:
                        print(_t, type(_t))
                        raise error
                    out.append(e)
        # children that are nested in another element
        if self.xml_nested_children != {}:
            for _, nest in self.xml_nested_children.items():
                out.append(nest.to_xml(object))
        return out

    def kwargs_from_xml(self, xml: ET.Element, **kwargs):
        _xmlfile = kwargs.get("_xmlfile", None)
        _smurffile = kwargs.get("_smurffile", None)
        # value
        if self.xml_value is not None and xml.text is not None:
            kwargs[self.xml_value] = self._deserialize(xml.text, key=xml.tag)
        # attributes
        for attname, varname in self.xml_attributes.items():
            if attname in xml.attrib:
                kwargs[varname] = self._deserialize(xml.attrib[attname], key=attname)
        for child in xml:
            if self.xml_value is not None:
                # value
                kwargs[self.xml_value] = self._deserialize(
                    child.text, key=child.tag
                )
            if child.tag in self.xml_children.keys():
                # normal children
                if self.xml_children[child.tag]["varname"] not in kwargs:
                    kwargs[self.xml_children[child.tag]["varname"]] = []
                kwargs[self.xml_children[child.tag]["varname"]] += [
                    self.xml_children[child.tag]["class"].from_xml(
                        child, self.dialect, _parent_xml=xml, _xmlfile=_xmlfile, _smurffile=_smurffile)]
            if child.tag in self.xml_attribute_children.keys():
                # children that are created from a simple property and have only attributes
                for attname, varname in self.xml_attribute_children[child.tag].items():
                    if attname in child.attrib.keys():
                        kwargs[varname] = self._deserialize(child.attrib[attname], key=attname)
            if child.tag in self.xml_value_children.keys():
                # children that have the a value as text
                kwargs[self.xml_value_children[child.tag]] = self._deserialize(child.text, key=child.tag)
            if child.tag in self.xml_nested_children.keys():
                # children that are nested in another element
                _kwargs = self.xml_nested_children[child.tag].kwargs_from_xml(child,
                                                                              _xmlfile=_xmlfile,
                                                                              _smurffile=_smurffile)
                for k, v in _kwargs.items():
                    if k in kwargs.keys() and v != kwargs[k]:
                        raise IndexError(
                            f"Key {k} of nested xml node {child.tag} already defined in conflict ({v}<>{kwargs[k]}) by superior node with keys: {str(kwargs.keys())}")
                    else:
                        kwargs[k] = v
        return kwargs

    def _serialize(self, entry, float_fmt=None) -> str:
        """

        Args:
            entry: The entry to serialize
            precision_dict: the float format in the style "%.6f"

        Returns:

        """
        assert entry is not None
        if hasattr(entry, "tolist"):
            entry = entry.tolist()
        if type(entry) in [list, tuple, np.array]:
            if any([type(v) in [int, np.int64] for v in entry]):
                entry = [str(v) for v in entry]
            elif any([type(v) in [float, np.float64] for v in entry]):
                entry = [float_fmt % v if type(v) in [float, np.float64] and float_fmt is not None else str(v) for v in entry]
            return " ".join(entry)
        elif type(entry) in [int, bool]:
            return str(int(entry))
        elif type(entry) in [float, np.float64]:
            return float_fmt % entry if float_fmt is not None else str(entry)
        else:
            return entry

    def _deserialize(self, string: str, key=None):
        string = string.strip()
        if " " in string and not key in ["uri", "url", "file", "filepath", "filename"]:
            _list = string.split()
            if all([is_int(v) for v in _list]):
                return [int(v) for v in _list]
            elif all([is_float(v) for v in _list]):
                return [float(v) for v in _list]
            else:
                return _list
        elif is_int(string):
            return int(string)
        elif is_float(string):
            return float(string)
        else:
            return string


class XMLFactory(XMLDefinition):
    def __init__(self, dialect, classname):
        self.available_in_dialect = classname in FORMATS[dialect].keys()
        if self.available_in_dialect:
            super(XMLFactory, self).__init__(
                dialect=dialect,
                tag=FORMATS[dialect][classname]["tag"],
                value=FORMATS[dialect][classname]["value"] if "value" in FORMATS[dialect][classname] else None,
                attributes=FORMATS[dialect][classname]["attributes"] if "attributes" in FORMATS[dialect][classname] else None,
                children=FORMATS[dialect][classname]["children"] if "children" in FORMATS[dialect][classname] else None,
                value_children=FORMATS[dialect][classname]["value_children"] if "value_children" in FORMATS[dialect][classname] else {},
                attribute_children=FORMATS[dialect][classname]["attribute_children"] if "attribute_children" in FORMATS[dialect][classname] else None,
                nested_children=FORMATS[dialect][classname]["nested_children"] if "nested_children" in FORMATS[dialect][classname] else None
            )

    def from_xml(self, classtype, xml: ET.Element, **kwargs):
        if self.available_in_dialect:
            if "Factory" in classtype.__name__:
                return classtype.create(**super(XMLFactory, self).kwargs_from_xml(xml, **kwargs), _xml=xml)
            return classtype.create(**super(XMLFactory, self).kwargs_from_xml(xml, **kwargs))
        return None

    def to_xml(self, object, float_fmt_dict=None, **kwargs):
        if float_fmt_dict is None:
            float_fmt_dict = {}
        if self.available_in_dialect:
            return super(XMLFactory, self).to_xml(object, float_fmt_dict=float_fmt_dict)
        else:
            return None

    def to_xml_string(self, object, float_fmt_dict=None, **kwargs):
        if self.available_in_dialect:
            return to_pretty_xml_string(self.to_xml(object, float_fmt_dict=float_fmt_dict, **kwargs))
        return None

    def from_xml_string(self, classtype, xml_string: str):
        if self.available_in_dialect:
            return self.from_xml(classtype, ET.fromstring(xml_string))
        return None


XML_REFLECTIONS = {
    "urdf": {
        "read": True,
        "write": True
    },
    "sdf": {
        "read": True,
        "write": True
    },
    "x3d": {
        "read": False,
        "write": True
    }
}


def class_factory(cls, only=None):
    reflections = [refl for refl in XML_REFLECTIONS.keys() if cls.__name__ in FORMATS[refl].keys()]
    setattr(cls, "factory", {refl: XMLFactory(refl, cls.__name__) for refl in reflections})
    for refl in reflections:
        if only is not None and refl not in only:
            continue

        if XML_REFLECTIONS[refl]["read"]:
            def _from_xml(c, xml, _dialect=refl, **kwargs):
                return c.from_xml(xml, dialect=_dialect, **kwargs)

            def _from_string(c, xml, _dialect=refl):
                return c.from_xml_string(xml, dialect=_dialect)

            setattr(cls, f"from_{refl}", classmethod(_from_xml))
            setattr(cls, f"from_{refl}_string", classmethod(_from_string))
            del _from_xml
            del _from_string

        if XML_REFLECTIONS[refl]["write"]:
            def _to_xml(obj, _dialect=refl, **kwargs):
                return obj.to_xml(dialect=_dialect, **kwargs)

            def _to_string(obj, _dialect=refl, **kwargs):
                return obj.to_xml_string(dialect=_dialect, **kwargs)

            setattr(cls, f"to_{refl}", _to_xml)
            setattr(cls, f"to_{refl}_string", _to_string)
            del _to_xml
            del _to_string

    if cls.__class__ == type and issubclass(cls, Linkable):
        # creates the setters and getters for all linked attributes
        if hasattr(cls, "type_dict"):
            for k, v in cls._type_dict.items():
                cls.type_dict[k] = v
        else:
            setattr(cls, "type_dict", cls._type_dict)
        class_vars = [var for var in cls._class_variables if var in cls.type_dict.keys() and not var.startswith("_")]
        for var in class_vars:
            setattr(cls, "_" + var, None)

            def _getter(cls, _var=var) -> [str, List[str]]:
                return cls._attr_get_name(_var)

            def _setter(cls, new_value: [str, List[str]], _var=var):
                cls._attr_set_name(_var, new_value)

            setattr(cls, var, property(_getter, _setter))

            del _getter
            del _setter


def singular(prop):
    """
    Makes sure the prop is single and not a list
    Args:
        prop: The property to check

    Returns:
        prop as a single element
    Raises:
        Assertion error if prop is a list with more than one element
    """
    if type(prop) == list:
        if len(prop) == 1:
            return prop[0]
        elif len(prop) == 0:
            return None
        else:
            raise AssertionError(f"Got list with content {prop} where only a single element is allowed.")
    else:
        return prop


def plural(prop):
    """
    Makes sure prop is a list
    Args:
        prop: The property to check

    Returns:
        prop as list
    """
    if type(prop) == list:
        return prop
    elif prop is None:
        return []
    else:
        return [prop]
