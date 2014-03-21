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
import os
import xml.etree.ElementTree as ET
import marstools.mtutility as mtutility
import marstools.mtdefs as mtdefs

def register():
    print("Registering mtimport...")


class RobotModelParser():
    """Base class for a robot model file parser of a specific type"""

    def __init__(self, filepath):
        self.filepath = filepath
        self.path, self.filename = os.path.split(self.filepath)
        self.robot = {}

    def createModel(self):
        """Creates the blender object representation of the imported model."""
        for l in self.robot['body']:
            link = self.robot['body'][l]
            #position = link['pose'][0:3]
            #rotation = link[link]['pose'][3:]
            #print(link['visual']['geometry'])
            if link['visual']['geometry'] == 'mesh':
                # import the .obj file (after importing a .obj file the newly
                # added objects are selected by Blender)
                print(os.path.join(self.path, link['filename']))
                bpy.ops.import_scene.obj(filepath=os.path.join(self.path, link['filename']))
                # apply the rotation to the imported object (is needed in
                # order to get the right orientation of the imported object)
                bpy.ops.object.transform_apply(rotation=True)

            #mtutility.createPrimitive(self.robot['body'][link]['name'], "box", (0.01, 0.01, 0.01), mtdefs.layerTypes['nodes'],
            #                          'indicator1', tuple(position), protation=tuple(rotation))



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
        print("Parsing URDF model from", self.filepath)
        self.tree = ET.parse(self.filepath)
        self.root = self.tree.getroot()[0]
        #self.robot["name"] = self.root.attrib["name"]
        #self.robot["version"] = self.robot["version"]#TODO: implement version functionality (time code)

        #write links to dictionary
        links = {}
        for link in self.root.iter('link'):
            newlink = {}
            for a in link.attrib:
                newlink[a] = link.attrib[a]
            #get all subnodes
            inertial = link.find('inertial')
            visual = link.find('visual')
            geometry = visual.find('geometry')
            material = visual.find('material')
            collision = link.find('collision')

            #compose dictionary information
            newlink['inertial'] = {}
            newlink['inertial']['mass'] = float(inertial.find('mass').attrib['value'])

            newlink['collision'] = {}
            newlink['collision']['collisionPrimitive'] = collision[0].tag
            for a in collision.attrib:
                newlink['collision'][a] = collision.attrib[a]

            newlink['visual'] = {}
            newlink['visual']['geometry'] = geometry[0].tag #TODO: this should be mesh/box/etc., but it's ugly that it's hard-coded
            if geometry[0].tag == 'mesh':
                newlink['filename'] = geometry[0].attrib['filename'] #TODO: what about scale?
            for a in visual.attrib:
                newlink['visual'][a] = visual.attrib[a]
            newlink['visual']['material'] = {'name': material.attrib['name'], 'color': material.find('color').attrib['rgba'].split(' ')}
            links[newlink['name']] = newlink
        self.robot["body"] = links

        #write joints to dictionary
        joints = {}
        for joint in self.root.iter('joint'):
            newjoint = {}
            for a in joint.attrib:
                newjoint[a] = joint.attrib[a]
            origin = joint.find('origin')
            pose = [float(num) for num in origin.attrib['xyz'].split(' ')]
            pose.extend([float(num) for num in origin.attrib['rpy'].split(' ')])
            newjoint['parent'] = joint.find('parent').attrib['link']
            newjoint['child'] = joint.find('child').attrib['link']
            self.robot['body'][newjoint['child']]['pose'] = pose
            joints[newjoint['name']] = newjoint
        self.robot["joint"] = joints

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
        importer.createModel()

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