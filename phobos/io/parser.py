import os
from xml.etree import ElementTree as ET

from .xmlrobot import XMLRobot


def parse_xml(xml):
    xml_root = None
    file_type = None
    xml_file = None
    print("Parsing", xml)
    if type(xml) == str:
        if os.path.isfile(xml):
            xml_file = xml
            xml_string = open(xml, "r").read()
            if xml.upper().endswith("SDF"):
                file_type = "sdf"
            elif xml.upper().endswith("URDF"):
                file_type = "urdf"
        else:
            xml_string = xml
        xml_root = ET.fromstring(xml_string)
    elif type(xml) == ET.ElementTree:
        xml_root = xml.getroot()
    elif type(xml) == ET.Element:
        xml_root = xml
    else:
        raise ValueError("Couldn't parse xml value of type" + repr(type(xml)))
    if file_type is None:
        if xml_root.tag == "sdf":
            file_type = "sdf"
            if len(xml_root.findall("./model")) > 1:
                print("WARNING: Multiple robots detected in this sdf!")
                return [XMLRobot.from_xml(x, dialect=file_type, xmlfile=xml_file) for x in xml_root.findall("./model")]
            else:
                xml_root = xml_root.findall("./model")[0]
        elif xml_root.tag == "model":
            file_type = "sdf"
        elif xml_root.tag == "robot":
            file_type = "urdf"
    return XMLRobot.from_xml(xml_root, dialect=file_type, xmlfile=xml_file)


