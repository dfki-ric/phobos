from . import representation
from . import xml_factory

for clsname in dir(representation):
    if not clsname.startswith("_") and clsname not in representation.__IMPORTS__:
        xml_factory.class_factory(getattr(representation, clsname))

from . import yaml_reflection
from . import sensor_representations
for clsname in dir(sensor_representations):
    if not clsname.startswith("_") and clsname not in sensor_representations.__IMPORTS__:
        xml_factory.class_factory(getattr(sensor_representations, clsname))

from . import hyrodyn
for clsname in dir(hyrodyn):
    if not clsname.startswith("_") and clsname not in hyrodyn.__IMPORTS__:
        xml_factory.class_factory(getattr(hyrodyn, clsname))

del xml_factory
