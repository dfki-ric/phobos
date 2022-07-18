from . import representation
from . import sensor_representations
from . import hyrodyn

from . import xml_factory
from . import yaml_reflection

for clsname in dir(representation):
    if not clsname.startswith("_") and clsname not in representation.__IMPORTS__:
        xml_factory.class_factory(getattr(representation, clsname))
for clsname in dir(sensor_representations):
    if not clsname.startswith("_") and clsname not in sensor_representations.__IMPORTS__:
        xml_factory.class_factory(getattr(sensor_representations, clsname))
for clsname in dir(hyrodyn):
    if not clsname.startswith("_") and clsname not in hyrodyn.__IMPORTS__:
        xml_factory.class_factory(getattr(hyrodyn, clsname))

del xml_factory
