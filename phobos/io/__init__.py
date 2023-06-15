from . import representation
from .xml_factory import REGISTER_MODULE_TO_XML_FACTORY

REGISTER_MODULE_TO_XML_FACTORY(representation)

from . import yaml_reflection
from . import sensor_representations
REGISTER_MODULE_TO_XML_FACTORY(sensor_representations)

from . import poses
REGISTER_MODULE_TO_XML_FACTORY(poses)

from . import hyrodyn
REGISTER_MODULE_TO_XML_FACTORY(hyrodyn)

from . import scenes
REGISTER_MODULE_TO_XML_FACTORY(scenes)

from . import xmlrobot
REGISTER_MODULE_TO_XML_FACTORY(xmlrobot)

del REGISTER_MODULE_TO_XML_FACTORY
