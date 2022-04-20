from . import representation
from . import base
from . import xml_factory

for clsname in dir(representation):
    if clsname != "Representation" and not clsname.startswith("_"):
        xml_factory.class_factory(getattr(representation, clsname))

