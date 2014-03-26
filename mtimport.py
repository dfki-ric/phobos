'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtimport.py

Created on 28 Feb 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
import mathutils
import os
from collections import namedtuple
import xml.etree.ElementTree as ET
import marstools.mtutility as mtutility
import marstools.mtdefs as mtdefs
import marstools.mtmaterials as mtmaterials

#This is a really nice pythonic approach to creating a list of constants
Defaults = namedtuple('Defaults', ['mass', 'idtransform'])
defaults = Defaults(0.001, #mass
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #idtransform
                    )

def register():
    print("Registering mtimport...")


def is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def parse_number(s):
    if is_int(s):
        return int(s)
    elif is_float(s):
        return float(s)
    else:
        return s

def only_contains_int(stringlist):
    for num in stringlist:
        if not is_int(num):
            return False
    return True

def only_contains_float(stringlist):
    for num in stringlist:
        if not is_float(num):
            return False
    return True

def find_in_list(alist, prop, value):
    n = -1

    for i in range(len(alist)):
        if alist[i][prop] == value:
            n = i
            break
    return n

def retrieve_from_list(alist, prop, value):
    n = -1

    for i in range(len(alist)):
        if alist[i][prop] == value:
            n = i
            break
    if n >= 0:
        return alist[n][prop]
    else:
        return "None"


def parse_text(s):
    numstrings = s.split()
    if len(numstrings) > 0:
        if only_contains_int(numstrings):
            nums = [int(num) for num in numstrings]
            return nums
        elif only_contains_float(numstrings):
            nums = [float(num) for num in numstrings]
            return nums
        else:
            return s
    else:
        return parse_number(s)


class RobotModelParser():
    """Base class for a robot model file parser of a specific type"""

    def __init__(self, filepath):
        self.filepath = filepath
        self.path, self.filename = os.path.split(self.filepath)
        self.robot = {}

    def createBlenderModel(self):
        """Creates the blender object representation of the imported model."""
        print("Creating Blender model...")
        linkslist = {}
        for l in self.robot['body']:
            link = self.robot['body'][l]
            #position = link['pose'][0:3]
            #rotation = link[link]['pose'][3:]
            #print(link['visual']['geometry'])
            geomtype = link['visual']['geometry']['type']
            if geomtype == 'mesh':
                # import the .obj file (after importing a .obj file the newly
                # added objects are selected by Blender)
                filetype = link['filename'].split('.')[-1]
                if filetype == 'obj' or filetype == 'OBJ':
                    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, link['filename']))
                elif filetype == 'stl' or filetype == 'STL':
                    bpy.ops.import_mesh.stl(filepath=os.path.join(self.path, link['filename']))
            elif geomtype == 'box':
                mtutility.createPrimitive(link['name'],
                                          geomtype,
                                          tuple(link['visual']['geometry']['size']),
                                          0,
                                          link['visual']['material']['name'], #TODO: this does not yet work for locally re-defined materials
                                          (0, 0, 0)
                                          )
            elif geomtype == 'cylinder':
                mtutility.createPrimitive(link['name'],
                                          geomtype,
                                          tuple(link['visual']['geometry']['radius'], link['visual']['geometry']['length']),
                                          0,
                                          link['visual']['material']['name'], #TODO: this does not yet work for locally re-defined materials
                                          (0, 0, 0)
                                          )
            elif geomtype == 'sphere':
                mtutility.createPrimitive(link['name'],
                                          geomtype,
                                          [link['visual']['geometry']['radius']], #tuple would cause problem here
                                          0,
                                          link['visual']['material']['name'], #TODO: this does not yet work for locally re-defined materials
                                          (0, 0, 0)
                                          )
            #print(bpy.context.object)
            #print(bpy.context.scene.objects.active)
            newlink = bpy.context.selected_objects[0] #TODO: this is a total hack!!!
            newlink.name = link['name']
            if newlink.name != link['name']:
                print("Warning, name conflict!")
            linkslist[newlink.name] = link['name']
            #reset scale
            if 'visual' in link and 'geometry' in link['visual'] and 'scale' in link['visual']['geometry']:
                print("Rescaling object", link['name'])
                newlink.scale = link['visual']['geometry']['scale']
            #correct translation & rotation
            print("Correcting pose of", l)
            newlink = bpy.data.objects[l]
            #print(linkslist[l])
            #print(self.robot['body'][linkslist[l]])
            #print(self.robot['body'][linkslist[l]]['parent'])
            try:
                parent = self.robot['body'][self.robot['body'][linkslist[l]]['parent']] #TODO: here we still have a problem with changed names
            except KeyError:
                print("Did not find parent node for", l, "- root node?")
                continue
            parent_location = mathutils.Vector(tuple(parent['pose'][0:3]))
            parent_rotation = mathutils.Euler(tuple(parent['pose'][3:]), 'XYZ')
            parent_rotationQ = parent_rotation.to_quaternion()
            newlink.location = parent_location + parent_rotationQ * (parent_location + mathutils.Vector(link['pose'][0:3]))
            print(newlink.rotation_quaternion)
            newlink_rotation = mathutils.Euler(tuple(link['pose'][3:]), 'XYZ')
            newlink.rotation_mode = "QUATERNION"
            newlink.rotation_quaternion = parent_rotationQ * newlink_rotation.to_quaternion()
            print(newlink.rotation_quaternion, '\n')
            bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)

            # now that we've applied the rotation of the joint, let's add the visual part (which is applied, too)
            newlink.rotation_quaternion = mathutils.Euler(tuple(link['visual']['pose'][3:]), 'XYZ').to_quaternion()
            #bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
            #newlink.dimensions = [a*b for a,b in zip(newlink.dimensions, link['visual']['geometry']['scale'])]

        #build tree
        print("\n\n Build tree...")
        for l in linkslist:
            link = self.robot['body'][linkslist[l]]
            if 'parent' in link:
                print(l + '..', end="")
                parentLink = bpy.data.objects[link['parent']]
                childLink = bpy.data.objects[link['name']]
                bpy.ops.object.select_all(action="DESELECT") #bpy.context.selected_objects = []
                childLink.select = True
                parentLink.select = True
                bpy.context.scene.objects.active = parentLink
                bpy.ops.object.parent_set()
                #childLink.parent = parentLink # TODO: this circumvents the parent inverse calculation!!!
        print("done.")


class MARSModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a MARS scene"""

    def __init__(self, filepath):
        RobotModelParser.__init__(filepath)

    def parseModel(self):
        print("Parsing MARS scene...")


class URDFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a URDF model"""

    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)

    def parseModel(self):
        novisual = True
        print("Parsing URDF model from", self.filepath)
        self.tree = ET.parse(self.filepath)
        self.root = self.tree.getroot()#[0]
        self.robot["name"] = self.root.attrib["name"]
        if 'version' in self.root.attrib:
            self.robot["version"] = self.root.attrib['version'] #TODO: implement version functionality (time code)

        #write links to dictionary
        links = {}
        for link in self.root.iter('link'):
            print("Parsing link", link.attrib['name'])
            newlink = {a: link.attrib[a] for a in link.attrib}

            #parse 'inertial'
            newlink['inertial'] = {}
            inertial = link.find('inertial')
            if inertial is not None: # !!! 'if Element' yields none if the Element contains no children, thus this notation !!!
                origin = inertial.find('origin')
                if origin is not None:
                    newlink['inertial']['pose'] = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
                else:
                    newlink['inertial']['pose'] = defaults.idtransform
                mass = inertial.find('mass')
                if mass is not None:
                    newlink['inertial']['mass'] = float(mass.attrib['value'])
                inertia = inertial.find('inertia')
                if inertia is not None:
                    values = []
                    newlink['inertial']['inertia'] = values.append(inertia.attrib[a] for a in inertia.attrib)
            else:
                newlink['inertial']['mass'] = defaults.mass

            #parse 'visual'
            visual = link.find('visual')
            if visual:
                newlink['visual'] = {a: visual.attrib[a] for a in visual.attrib}
                origin = visual.find('origin')
                if origin is not None:
                    newlink['visual']['pose'] = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
                else:
                    newlink['visual']['pose'] = defaults.idtransform
                geometry = visual.find('geometry')
                if geometry is not None:
                    newlink['visual']['geometry'] = {a: parse_text(geometry[0].attrib[a]) for a in geometry[0].attrib}
                    newlink['visual']['geometry']['type'] = geometry[0].tag
                    novisual = False
                    if geometry[0].tag == 'mesh':
                        newlink['filename'] = geometry[0].attrib['filename'] #TODO: remove this, also from export, as it is double
                else: #if geometry is None
                    print("### ERROR: No geometry information for visual element, trying to parse from collision data.")
                material = visual.find('material')
                if material is not None:
                    newlink['visual']['material'] = {'name': material.attrib['name']}
                    color = material.find('color')
                    if color is not None:
                        newlink['visual']['material']['color'] = parse_text(color.attrib['rgba'])
            else:
                newlink['visual'] = {}

            #parse 'collision' #TODO: support multiple collision bodies via union of the geometry
            newlink['collision'] = {}
            collision = link.find('collision')
            if collision is not None:
                origin = collision.find('origin')
                if origin is not None:
                    newlink['collision']['pose'] = origin.attrib['xyz'].split(' ') + origin.attrib['rpy'].split(' ')
                else:
                    newlink['collision']['pose'] = defaults.idtransform
                geometry = collision.find('geometry')
                if geometry is not None:
                    newlink['collision']['geometry'] = {a: parse_text(geometry[0].attrib[a]) for a in geometry[0].attrib}
                    newlink['collision']['geometry']['type'] = geometry[0].tag
                    if geometry[0].tag == 'mesh':
                        newlink['filename'] = geometry[0].attrib['filename']
                    if novisual:
                        newlink['visual']['geometry'] = newlink['collision']['geometry']
                else:
                    if novisual:
                        print("### ERROR: No geometry information provided for link", newlink['name'], "- assuming box of size 1.0.")
                        newlink['collision']['geometry'] = {}
                        newlink['collision']['geometry']['size'] = 1.0
                        newlink['collision']['geometry']['type'] = 'box'
                    else:
                        print("### ERROR: No geometry information provided for collision element, parsing from visual data.")
                        newlink['collision']['geometry']['type'] = newlink['visual']['geometry']['type']
            #write link to list
            links[newlink['name']] = newlink
            #print(newlink)
        self.robot["body"] = links

        #write joints to dictionary
        joints = {}
        for joint in self.root.iter('joint'):
            print("Parsing joint", joint.attrib['name'])
            newjoint = {a: joint.attrib[a] for a in joint.attrib}
            origin = joint.find('origin')
            newjoint['parent'] = joint.find('parent').attrib['link']
            newjoint['child'] = joint.find('child').attrib['link']
            self.robot['body'][newjoint['child']]['pose'] = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
            #axis
            #calibration
            #dynamics
            #limit
            #mimic
            #safety_controller
            joints[newjoint['name']] = newjoint
            #print(newjoint)
        self.robot["joint"] = joints

        #find any links that still have no pose (most likely because they had no parent)
        for link in links:
            if not 'pose' in links[link]:
                links[link]['pose'] = defaults.idtransform
            print(link, links[link]['pose'])

        #write parent-child information to nodes
        for j in self.robot['joint']:
            joint = self.robot['joint'][j]
            self.robot['body'][joint['child']]['parent'] = joint['parent']
            print("Writing joint parent...", joint['parent'])

        materials = []
        for material in self.root.iter('material'):
            print("Parsing material", material.attrib['name'])
            newmaterial = {a: material.attrib[a] for a in material.attrib}
            color = material.find('color')
            if color is not None:
                newmaterial['color'] = parse_text(color.attrib['rgba'])
                materials.append(newmaterial)
        for m in materials:
            mtmaterials.makeMaterial(m['name'], tuple(m['color'][0:3]), (1, 1, 1), m['color'][-1]) #TODO: handle duplicate names? urdf_robotname_xxx?

    def parseLink(self, linkinfo):
        print("Parsing link", )
        link = {}
        return link

    def parseJoint(self, jointinfo):
        print("Parsing joint", )
        joint = {}
        return joint

class SMURFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a SMURF model"""

    def __init__(self, filepath):
        RobotModelParser.__init__(filepath)

    def parseModel(self):
        print("Parsing SMURF model...")


class RobotModelImporter(bpy.types.Operator):
    """Importer for MARS-compatible model or scene files"""
    bl_idname = "obj.import_robot_model"
    bl_label = "Import robot model file from various formats"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'

    # creating property for storing the path to the .scn file
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    # set a filter to only consider .scn files (only used internally)
    #filter_glob = bpy.props.StringProperty(default="*.*",options={'HIDDEN'})

    @classmethod
    def poll(cls, context):
        return context is not None

    def execute(self, context):
        # get the chosen file path
        #directory, filename = os.path.split(self.filepath)
        modeltype = self.filepath.split('.')[-1]

        if modeltype == 'scene':
            importer = MARSModelParser(self.filepath)
        elif modeltype == 'urdf':
            importer = URDFModelParser(self.filepath)
        elif modeltype == 'smurf' or modeltype == 'yml' or modeltype == 'yaml':
            importer = SMURFModelParser(self.filepath)
        else:
            print("Unknown model format, aborting import...")

        importer.parseModel()
        importer.createBlenderModel()

        return {'FINISHED'}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}

# Register and add to the file selector
bpy.utils.register_class(RobotModelImporter)


def main():
    # call the newly registered operator
    bpy.ops.import_robot_model('INVOKE_DEFAULT')

if __name__ == '__main__':
    main()