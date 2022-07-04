from . import representation
from . import base
from . import xml_factory
from . import yaml_reflection

for clsname in dir(representation):
    if not clsname.startswith("_") and clsname not in representation.__IMPORTS__:
        xml_factory.class_factory(getattr(representation, clsname))

