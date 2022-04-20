import xml.etree.ElementTree as ET


class Representation(object):
    factory = {}

    @classmethod
    def from_xml(cls, xml: ET.Element, dialect):
        return cls.factory[dialect].from_xml(cls, xml)

    @classmethod
    def from_xml_string(cls, xml: str, dialect):
        return cls.factory[dialect].from_xml_string(cls, xml)

    def to_xml(self, dialect) -> ET.Element:
        return self.factory[dialect].to_xml(self)

    def to_xml_string(self, dialect) -> ET.Element:
        return self.factory[dialect].to_xml_string(self)

    def sort_string(self) -> str:
        if hasattr(self, "name"):
            return type(self).__name__+":"+self.name
        else:
            return type(self).__name__
