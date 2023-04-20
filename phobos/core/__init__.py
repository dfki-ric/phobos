from .robot import Robot
from .multiple import Entity, Arrangement, World

from ..io.xml_factory import REGISTER_MODULE_TO_XML_FACTORY
from . import multiple
REGISTER_MODULE_TO_XML_FACTORY(multiple)
del multiple
del REGISTER_MODULE_TO_XML_FACTORY