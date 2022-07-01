import json
import collections
from xml.etree import ElementTree as ET

from ..utils.misc import to_pretty_xml_string


def to_yaml(obj):
    """ Simplify yaml representation for pretty printing """
    # Is there a better way to do this by adding a representation with
    # yaml.Dumper?
    # Ordered dict: http://pyyaml.org/ticket/29#comment:11
    if obj is None or isinstance(obj, str):
        out = str(obj)
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
    elif isinstance(obj, collections.Iterable):
        out = [to_yaml(item) for item in obj]
    else:
        out = str(obj)
    return out


class SelectiveReflection(object):
    def get_refl_vars(self):
        return list(vars(self).keys())


class YamlReflection(SelectiveReflection):
    def to_yaml(self):
        raw = dict((var, getattr(self, var)) for var in self.get_refl_vars())
        return to_yaml(raw)

    def __str__(self):
        # Good idea? Will it remove other important things?
        return json.dumps(self.to_yaml()).rstrip()
