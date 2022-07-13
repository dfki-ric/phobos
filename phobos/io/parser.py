import os
from xml.etree import ElementTree as ET

from .xmlrobot import XMLRobot


def parse_xml(xml):
    xml_root = None
    file_type = None
    if type(xml) == str:
        if os.path.isfile(xml):
            xml_string = open(xml, "r").read()
            if xml.upper().endswith("SDF"):
                file_type = "sdf"
            elif xml.upper().endswith("URDF"):
                file_type = "urdf"
        else:
            xml_string = xml
        xml_root = ET.fromstring(xml_string)
    elif type(xml) == ET.ElementTree or type(xml) == ET.Element:
        if type(xml) is ET.ElementTree:
            xml_root = xml.getroot()
        else:
            xml_root = xml
    if file_type is None:
        if xml_root.tag == "sdf":
            file_type = "sdf"
            if len(xml_root.findall("./model")) > 1:
                print("WARNING: Multiple robots detected in this sdf!")
                return [XMLRobot.from_xml(x, dialect=file_type) for x in xml_root.findall("./model")]
            else:
                xml_root = xml_root.findall("./model")[0]
        elif xml_root.tag == "model":
            file_type = "sdf"
        elif xml_root.tag == "robot":
            file_type = "urdf"
    return XMLRobot.from_xml(xml_root, dialect=file_type)
