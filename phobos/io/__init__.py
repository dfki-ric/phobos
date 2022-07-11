from . import representation
from . import sensors


from . import xml_factory
from . import yaml_reflection

for clsname in dir(representation):
    if not clsname.startswith("_") and not clsname in representation.__IMPORTS__:
        xml_factory.class_factory(getattr(representation, clsname))
for clsname in dir(sensors):
    if not clsname.startswith("_") and not clsname in sensors.__IMPORTS__:
        xml_factory.class_factory(getattr(sensors, clsname))

del xml_factory
