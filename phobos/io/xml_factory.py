from xml.etree import ElementTree as ET
import json

import numpy as np
import pkg_resources

from .base import Representation
from . import representation
from ..utils.misc import to_pretty_xml_string

FORMATS = json.load(open(pkg_resources.resource_filename("phobos", "data/xml_formats.json"), "r"))


def is_int(val: str):
    try:
        int(val)
        return True
    except ValueError:
        return False


def is_float(val: str):
    try:
        float(val)
        return True
    except ValueError:
        return False


def get_class(classname):
    if hasattr(representation, classname) and classname not in representation.__IMPORTS__:
        cls = getattr(representation, classname)
    else:
        raise AssertionError(f"The class {classname} is not None to the XML-Factory")
    assert isinstance(cls, Representation), f"The class {classname} is no valid Representation instance"
    return cls


class XMLDefinition(object):
    def __init__(self, dialect, tag, value=None, attributes=None, children=None, value_children=None, attribute_children=None, nested_children=None):
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

    def to_xml(self, object):
        # attributes
        attrib = {}
        for attname, varname in self.xml_attributes.items():
            val = getattr(object, varname)
            if val is not None:
                attrib[attname] = self._serialize(val)
        out = ET.Element(self.xml_tag, attrib=attrib)
        # value
        if self.xml_value is not None:
            assert all([x == {} for x in [self.xml_children, self.xml_value_children, self.xml_attribute_children, self.xml_nested_children]])
            val = getattr(object, self.xml_value)
            out.text = self._serialize(val)
            if val is not None:
                return out
            else:
                return
        # normal children
        children = []
        for tag, var in self.xml_children.items():
            obj = getattr(object, var["varname"])
            if type(obj) == list:
                children += [o for o in obj if isinstance(o, var["class"])]
            elif isinstance(obj, var["class"]):
                children += [obj]
        for child in sorted(children, key=lambda x: x.sort_string()):
            out.append(child.to_xml(self.dialect))
        # children that are created from a simple property and have only attributes
        for tag, attribute_map in self.xml_attribute_children.items():
            _attrib = {attname: self._serialize(getattr(object, varname))
                                        for attname, varname in attribute_map.items() if getattr(object, varname) is not None}
            if len(_attrib) == 0:
                continue
            e = ET.Element(tag, attrib=_attrib)
            out.append(e)
        # children that have the a value as text
        for tag, varname in self.xml_value_children.items():
            e = ET.Element(tag)
            val = getattr(object, varname)
            if val is not None:
                e.text = self._serialize(getattr(object, varname))
                out.append(e)
        # children that are nested in another element
        if self.xml_nested_children != {}:
            for _, nest in self.xml_nested_children.items():
                out.append(nest.to_xml(object))
        return out

    def kwargs_from_xml(self, xml: ET.Element):
        kwargs = {}
        # attributes
        for attname, varname in self.xml_attributes.items():
            if attname in xml.attrib:
                kwargs[varname] = self._deserialize(xml.attrib[attname])
        for child in xml:
            if self.xml_value is not None:
                # value
                kwargs[self.xml_value] = self._deserialize(child.text)
            elif child.tag in self.xml_children.keys():
                # normal children
                if self.xml_children[child.tag]["varname"] not in kwargs:
                    kwargs[self.xml_children[child.tag]["varname"]] = []
                kwargs[self.xml_children[child.tag]["varname"]] += [self.xml_children[child.tag]["class"].from_xml(child, self.dialect)]
            elif child.tag in self.xml_attribute_children.keys():
                # children that are created from a simple property and have only attributes
                for attname, varname in self.xml_attribute_children[child.tag].items():
                    if attname in child.attrib.keys():
                        kwargs[varname] = self._deserialize(child.attrib[attname])
            elif child.tag in self.xml_value_children.keys():
                # children that have the a value as text
                kwargs[self.xml_value_children[child.tag]] = self._deserialize(child.text)
            elif child.tag in self.xml_nested_children.keys():
                # children that are nested in another element
                _kwargs = self.xml_nested_children[child.tag].kwargs_from_xml(child)
                for k, v in _kwargs.items():
                    if k in kwargs.keys():
                        raise IndexError(f"Key {k} of nested xml node {child.tag} already defined by superior node with keys: {str(kwargs.keys())}")
                    else:
                        kwargs[k] = v
        return kwargs

    def _serialize(self, entry) -> str:
        assert entry is not None
        if hasattr(entry, "tolist"):
            entry = entry.tolist()
        if type(entry) == list:
            if any([type(v) in [int, np.int64] for v in entry]):
                entry = [str(v) for v in entry]
            elif any([type(v) in [float, np.float64] for v in entry]):
                entry = ["%.6f" % v if type(v) in [float, np.float64] else str(v) for v in entry]
            return " ".join(entry)
        elif type(entry) == int:
            return str(entry)
        elif type(entry) in [float, np.float64]:
            return "%.6f" % entry
        else:
            return entry

    def _deserialize(self, string: str):
        string = string.strip()
        if " " in string:
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

    def from_xml(self, classtype, xml: ET.Element):
        return classtype(**super(XMLFactory, self).kwargs_from_xml(xml))

    def to_xml_string(self, object):
        return to_pretty_xml_string(self.to_xml(object))

    def from_xml_string(self, classtype, xml_string: str):
        return self.from_xml(classtype, ET.fromstring(xml_string))


XML_REFLECTIONS = ["urdf", "sdf"]


def class_factory(cls):
    setattr(cls, "factory", {refl: XMLFactory(refl, cls.__name__) for refl in XML_REFLECTIONS})
    for refl in XML_REFLECTIONS:
        def _from_xml(c, xml, _dialect=refl):
            return c.from_xml(xml, dialect=_dialect)

        def _from_string(c, xml, _dialect=refl):
            return c.from_xml_string(xml, dialect=_dialect)

        def _to_xml(obj, _dialect=refl):
            return obj.to_xml(dialect=_dialect)

        def _to_string(obj, _dialect=refl):
            return obj.to_xml_string(dialect=_dialect)

        setattr(cls, f"from_{refl}", classmethod(_from_xml))
        setattr(cls, f"from_{refl}_string", classmethod(_from_string))
        setattr(cls, f"to_{refl}", _to_xml)
        setattr(cls, f"to_{refl}_string", _to_string)

        del _from_xml
        del _from_string
        del _to_xml
        del _to_string


def singular(prop):
    if type(prop) == list:
        assert len(prop) == 1
        return prop[0]
    else:
        return prop
