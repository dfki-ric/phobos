import os
from xml.etree import ElementTree as ET

from .xmlrobot import XMLRobot
from ..commandline_logging import get_logger

log = get_logger(__name__)


def parse_xml(xml):
    xml_root = None
    file_type = None
    xml_file = None
    log.debug(f"Parsing {xml}")
    if type(xml) == str:
        if os.path.isfile(xml):
            xml_file = xml
            with open(xml, "r") as f:
                xml_string = f.read()
            if xml.upper().endswith("SDF"):
                file_type = "sdf"
            elif xml.upper().endswith("URDF"):
                file_type = "urdf"
        else:
            xml_string = xml
        try:
            xml_root = ET.fromstring(xml_string)
        except ET.ParseError as e:
            log.error(f"Tried to parse:\n  {xml}")
            raise IOError("Could not parse xml. See above for more info what was tried to parse! Error:" + e.msg)
    elif type(xml) == ET.ElementTree:
        xml_root = xml.getroot()
    elif type(xml) == ET.Element:
        xml_root = xml
    else:
        raise ValueError("Couldn't parse xml value of type" + repr(type(xml)))
    if file_type is None or file_type == "sdf":
        if xml_root.tag == "sdf":
            file_type = "sdf"
            if len(xml_root.findall("./model")) > 1:
                log.warning("Multiple robots detected in this sdf!")
                return [XMLRobot.from_xml(x, dialect=file_type, _xmlfile=xml_file) for x in xml_root.findall("./model")]
            else:
                xml_root = xml_root.findall("./model")[0]
        elif xml_root.tag == "model":
            file_type = "sdf"
        elif xml_root.tag == "robot":
            file_type = "urdf"
    return XMLRobot.from_xml(xml_root, dialect=file_type, _xmlfile=xml_file)


