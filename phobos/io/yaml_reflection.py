from xml.etree import ElementTree as ET

from .base import Linkable
from ..utils.misc import to_pretty_xml_string


def to_yaml(obj):
    """ Simplify yaml representation for pretty printing """
    # Is there a better way to do this by adding a representation with
    # yaml.Dumper?
    # Ordered dict: http://pyyaml.org/ticket/29#comment:11
    if obj is None or isinstance(obj, str):
        out = str(obj)
    elif isinstance(obj, Linkable) and obj.stringable():
        return str(obj)
    elif type(obj) in [int, float, bool]:
        return obj
    elif hasattr(obj, 'to_yaml'):
        out = obj.to_yaml()
    elif isinstance(obj, type(ET.Element)):
        out = to_pretty_xml_string(obj)
    elif type(obj) == dict:
        out = {}
        for (var, value) in obj.items():
            out[str(var)] = to_yaml(value)
    elif hasattr(obj, 'tolist'):
        # For numpy objects
        out = to_yaml(obj.tolist())
    elif type(obj) in [set, list, tuple]:
        out = [to_yaml(item) for item in obj]
    else:
        out = str(obj)
    return out


class SelectiveReflection(Linkable):
    def __init__(self):
        super(SelectiveReflection, self).__init__()

    def get_refl_vars(self):
        return [v for v in vars(self).keys() if not v.startswith("_")]


class YamlReflection(SelectiveReflection):
    def __init__(self):
        super(YamlReflection, self).__init__()

    def to_yaml(self):
        raw = dict((var, getattr(self, var)) for var in self.get_refl_vars())
        return to_yaml(raw)
