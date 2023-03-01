from .assembly import Assembly
from .entity import BaseEntity, Entity
from .scene import Scene
from ..io import xml_factory

xml_factory.class_factory(Scene)
xml_factory.class_factory(Assembly)
xml_factory.class_factory(BaseEntity)
xml_factory.class_factory(Entity)

del xml_factory
