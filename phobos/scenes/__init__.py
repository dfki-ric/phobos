from ..io import xml_factory

from .scene import Scene
from .assembly import Assembly
from .entity import BaseEntity, Entity

xml_factory.class_factory(Scene)
xml_factory.class_factory(Assembly)
xml_factory.class_factory(BaseEntity)
xml_factory.class_factory(Entity)

del xml_factory
