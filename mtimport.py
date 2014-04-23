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

def cleanUpScene():
    # select all objects
    bpy.ops.object.select_all(action="SELECT")

    # and delete them
    bpy.ops.object.delete()

    # after that we have to clean up all loaded meshes (unfortunately
    # this is not done automatically)
    for mesh in bpy.data.meshes:
        bpy.data.meshes.remove(mesh)

    # and all materials
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)

    # and all lights (aka lamps)
    for lamp in bpy.data.lamps:
        bpy.data.lamps.remove(lamp)

    # clear global parameters (needed when importing this file and trying
    # to load multiple .scn files)
    #nodeList.clear()
    #jointList.clear()
    #sensorList.clear()
    #materialList.clear()
    #controllerList.clear()
    #lightList.clear()
    #objFileMap.clear()
    #unusedNodeList.clear()


class RobotModelParser():
    """Base class for a robot model file parser of a specific type"""

    def __init__(self, filepath):
        self.filepath = filepath
        self.path, self.filename = os.path.split(self.filepath)
        self.robot = {}

    def createRecursiveBlenderModel(self): #TODO: solve problem with duplicated links (linklist...namespaced via robotname?)
        """Creates the blender object representation of the imported model."""
        print("\n\nCreating Blender model...")
        for l in self.robot['link']:
            print(l + ', ', end='')
            link = self.robot['link'][l]

            #determine which visual representation to use
            geomtype = 'None'
            geomsrc = 'visual'
            if 'visual' in link:
                geomtype = link['visual']['geometry']['type']
            elif 'collision' in link:
                geomtype = link['collision']['geometry']['type']
                geomsrc = 'collision'
            else: #only inertial, create dummy link
                mtutility.createPrimitive(link['name'], 'sphere', [0.001], 0, 'None', (0, 0, 0))
                #bpy.ops.object.empty_add(type='ARROWS')
                #obj = bpy.context.object
                #obj.name = link['name']

            # create the Blender object
            if geomtype == 'mesh':
                filetype = link['filename'].split('.')[-1]
                if filetype == 'obj' or filetype == 'OBJ':
                    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, link['filename'])) #FIXME: we don't use the geomsrc variable here (problem with local and global filename)
                elif filetype == 'stl' or filetype == 'STL':
                    bpy.ops.import_mesh.stl(filepath=os.path.join(self.path, link['filename']))
            elif geomtype == 'box':
                mtutility.createPrimitive(link['name'],
                                          geomtype,
                                          tuple(link['visual']['geometry']['size']),
                                          0,
                                          link[geomsrc]['material']['name'], #TODO: this does not yet work for locally re-defined materials
                                          (0, 0, 0)
                                          )
            elif geomtype == 'cylinder':
                mtutility.createPrimitive(link['name'],
                                          geomtype,
                                          tuple(link['visual']['geometry']['radius'], link['visual']['geometry']['length']),
                                          0,
                                          link[geomsrc]['material']['name'], #TODO: this does not yet work for locally re-defined materials
                                          (0, 0, 0)
                                          )
            elif geomtype == 'sphere':
                mtutility.createPrimitive(link['name'],
                                          geomtype,
                                          link['visual']['geometry']['radius'], #tuple would cause problem here
                                          0,
                                          link[geomsrc]['material']['name'], #TODO: this does not yet work for locally re-defined materials
                                          (0, 0, 0)
                                          )
            else:
                print("### ERROR: Could not determine geometry type of link", l + '. Placing empty coordinate system.')
            #print(bpy.context.object)
            #print(bpy.context.scene.objects.active)
            newlink = bpy.context.selected_objects[0] #TODO: this is a total hack!!!
            newlink.name = link['name']
            if newlink.name != link['name']:
                print("Warning, name conflict!")
            #reset scale
            if 'visual' in link and 'geometry' in link['visual'] and 'scale' in link['visual']['geometry']:
                newlink.scale = link['visual']['geometry']['scale']
                bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

        def place_children_of(parent):
            print(parent['name']+ ', ', end='')
            children = []
            for l in self.robot['link']:
                if 'parent' in self.robot['link'][l] and self.robot['link'][l]['parent'] == parent['name']:
                    children.append(self.robot['link'][l])
            for child in children:
                # 1: set parent relationship (this makes the parent inverse the inverse of the parents world transform)
                parentLink = bpy.data.objects[parent['name']]
                childLink = bpy.data.objects[child['name']]
                bpy.ops.object.select_all(action="DESELECT") #bpy.context.selected_objects = []
                childLink.select = True
                parentLink.select = True
                bpy.context.scene.objects.active = parentLink
                bpy.ops.object.parent_set()
                # 2: move to parents origin by setting the world matrix to the parents world matrix
                childLink.matrix_world = parentLink.matrix_world #HACK: this does not work if the parent link has transformed visual
                # 3: apply local transform as saved in urdf (change matrix_local from identity to urdf)
                # 3.1: build a transformation matrix from urdf joint pose (this can be used for creating new joint spheres)
                urdf_loc = mathutils.Matrix.Translation(child['pose'][0:3]) #TODO: this is actually only the position of the joint sphere, we need to add visual/inertial to this
                urdf_rot = mathutils.Euler(tuple(child['pose'][3:]), 'XYZ').to_matrix().to_4x4()
                # 3.2: make sure to take into account visual information #TODO: also take into account inertial and joint axis (for joint sphere) and collision (bounding box)
                if 'visual' in child and 'pose' in child['visual']:
                    urdf_visual_loc = mathutils.Matrix.Translation(child['visual']['pose'][0:3])
                    urdf_visual_rot = mathutils.Euler(tuple(child['visual']['pose'][3:]), 'XYZ').to_matrix().to_4x4()
                else:
                    urdf_visual_loc = mathutils.Matrix.Identity(4)
                    urdf_visual_rot = mathutils.Matrix.Identity(4)
                #urdf_sca = #TODO: solve problem with scale
                urdfmatrix = urdf_loc * urdf_rot * urdf_visual_loc * urdf_visual_rot #*urdf_sca
                # 3.3: set local transform to matrix computed from urdf #TODO: keep bounding boxes on a different layer in blender or use inbuilt bounding box functionality?
                childLink.matrix_local = urdfmatrix
                # 4: be happy, as world and basis are now the same and local is the transform to be exported to urdf
                # 5: take care of the rest of the tree
                place_children_of(child)

        #build tree recursively and correct translation & rotation on the fly
        for l in self.robot['link']:
            if not 'parent' in self.robot['link'][l]:
                root = self.robot['link'][l]
        print("\n\nPlacing links...")
        place_children_of(root)


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
        print("\nParsing URDF model from", self.filepath)
        self.tree = ET.parse(self.filepath)
        self.root = self.tree.getroot()#[0]
        self.robot["name"] = self.root.attrib["name"]
        if 'version' in self.root.attrib:
            self.robot["version"] = self.root.attrib['version'] #TODO: implement version functionality (time code)

        #write links to dictionary
        links = {}
        print("\n\nParsing links..")
        for link in self.root.iter('link'):
            novisual = True
            print(link.attrib['name'] + ', ', end='')
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
                newlink['inertial'] = {'mass': defaults.mass}

            #parse 'visual'
            visual = link.find('visual')
            if visual is not None:
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
                    print(newlink['name'], geometry[0].tag)
                    novisual = False
                    if geometry[0].tag == 'mesh':
                        newlink['filename'] = geometry[0].attrib['filename'] #TODO: remove this, also from export, as it is double
                else: #if geometry is None
                    print("\n### WARNING: No geometry information for visual element, trying to parse from collision data.")
                material = visual.find('material')
                if material is not None:
                    newlink['visual']['material'] = {'name': material.attrib['name']}
                    color = material.find('color')
                    if color is not None:
                        newlink['visual']['material']['color'] = parse_text(color.attrib['rgba'])
                else:
                    newlink['visual']['material'] = {'name':'None'} #TODO: this is a hack!
                    print("\n### Warning: No material provided for link", newlink['name'])
            else:
                print("\n### WARNING: No visual information provided for link", newlink['name'])

            #parse 'collision' #TODO: support multiple collision bodies via union of the geometry
            collision = link.find('collision')
            if collision is not None:
                newlink['collision'] = {a: collision.attrib[a] for a in collision.attrib}
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
                    #if novisual:
                    #    newlink['visual']['geometry'] = newlink['collision']['geometry']
                else:
                    print("\n### WARNING: No collision geometry information provided for link", newlink['name'] + '.')
                    if novisual:
                        print("\n### WARNING:", newlink['name'], "is empty.")
            else:
                print("\n### WARNING: No collision information provided for link", newlink['name'] + '.')
                if novisual:
                    print("\n### WARNING:", newlink['name'], "is empty.")
            #write link to list
            links[newlink['name']] = newlink
            #print(newlink)
        self.robot["link"] = links

        #write joints to dictionary
        joints = {}
        print("\n\nParsing joints..")
        for joint in self.root.iter('joint'):
            print(joint.attrib['name']+', ', end='')
            newjoint = {a: joint.attrib[a] for a in joint.attrib}
            origin = joint.find('origin')
            newjoint['parent'] = joint.find('parent').attrib['link']
            newjoint['child'] = joint.find('child').attrib['link']
            self.robot['link'][newjoint['child']]['pose'] = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
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
            #print(link, links[link]['pose'])

        #write parent-child information to nodes
        print("\n\nWriting parent-child information to nodes..")
        for j in self.robot['joint']:
            joint = self.robot['joint'][j]
            self.robot['link'][joint['child']]['parent'] = joint['parent']
            print(joint['parent'] + ', ', end='')

        materials = []
        print("\n\nParsing materials..")
        for material in self.root.iter('material'):
            newmaterial = {a: material.attrib[a] for a in material.attrib}
            color = material.find('color')
            if color is not None:
                print(material.attrib['name'] + ', ', end='')
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
        importer.createRecursiveBlenderModel()

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