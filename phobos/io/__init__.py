from . import representation
from . import sensors
from . import hyrodyn

from . import xml_factory
from . import yaml_reflection

for clsname in dir(representation):
    if not clsname.startswith("_") and clsname not in representation.__IMPORTS__:
        xml_factory.class_factory(getattr(representation, clsname))
for clsname in dir(sensors):
    if not clsname.startswith("_") and clsname not in sensors.__IMPORTS__:
        xml_factory.class_factory(getattr(sensors, clsname))
for clsname in dir(hyrodyn):
    if not clsname.startswith("_") and clsname not in hyrodyn.__IMPORTS__:
        xml_factory.class_factory(getattr(hyrodyn, clsname))

del xml_factory
