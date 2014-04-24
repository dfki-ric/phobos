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
from marstools.mtutility import *
import marstools.mtdefs as mtdefs
import marstools.mtmaterials as mtmaterials

#This is a really nice pythonic approach to creating a list of constants
Defaults = namedtuple('Defaults', ['mass', 'idtransform'])
defaults = Defaults(0.001, #mass
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #idtransform
                    )

def register():
    print("Registering mtimport...")


# def is_float(s):
#     try:
#         float(s)
#         return True
#     except ValueError:
#         return False
#
# def is_int(s):
#     try:
#         int(s)
#         return True
#     except ValueError:
#         return False
#
# def parse_number(s):
#     if is_int(s):
#         return int(s)
#     elif is_float(s):
#         return float(s)
#     else:
#         return s
#
# def only_contains_int(stringlist):
#     for num in stringlist:
#         if not is_int(num):
#             return False
#     return True
#
# def only_contains_float(stringlist):
#     for num in stringlist:
#         if not is_float(num):
#             return False
#     return True
#
# def find_in_list(alist, prop, value):
#     n = -1
#
#     for i in range(len(alist)):
#         if alist[i][prop] == value:
#             n = i
#             break
#     return n
#
# def retrieve_from_list(alist, prop, value):
#     n = -1
#
#     for i in range(len(alist)):
#         if alist[i][prop] == value:
#             n = i
#             break
#     if n >= 0:
#         return alist[n][prop]
#     else:
#         return "None"
#
#
# def parse_text(s):
#     numstrings = s.split()
#     if len(numstrings) > 0:
#         if only_contains_int(numstrings):
#             nums = [int(num) for num in numstrings]
#             return nums
#         elif only_contains_float(numstrings):
#             nums = [float(num) for num in numstrings]
#             return nums
#         else:
#             return s
#     else:
#         return parse_number(s)

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

    def placeChildLinks(self, parent):
        print(parent['name']+ ', ', end='')
        children = []
        for l in self.robot['links']:
            if 'parent' in self.robot['links'][l] and self.robot['links'][l]['parent'] == parent['name']:
                children.append(self.robot['links'][l])
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
            childLink.matrix_world = parentLink.matrix_world
            # 3: apply local transform as saved in urdf (change matrix_local from identity to urdf)
            urdf_loc = mathutils.Matrix.Translation(child['pose'][0:3])
            urdf_rot = mathutils.Euler(tuple(child['pose'][3:]), 'XYZ').to_matrix().to_4x4()
            urdfmatrix = urdf_loc * urdf_rot
            childLink.matrix_local = urdfmatrix
            # 4: be happy, as world and basis are now the same and local is the transform to be exported to urdf
            # 5: take care of the rest of the tree
            self.placeChildLinks(child)

    def placeLinkSubelements(self, link):
        #urdf_sca = #TODO: solve problem with scale
        # 3.2: make sure to take into account visual information #TODO: also take into account inertial and joint axis (for joint sphere) and collision (bounding box)
        #* urdf_visual_loc * urdf_visual_rot #*urdf_sca
        parentLink = bpy.data.objects[link['name']]
        for geomsrc in ['visual', 'collision']: #TODO: add inertial (name issue)
            if geomsrc in link:
                if 'pose' in link[geomsrc]:
                    print(link['name'], geomsrc, link[geomsrc]['pose'][0:3])
                    urdf_geom_loc = mathutils.Matrix.Translation(link[geomsrc]['pose'][0:3])
                    urdf_geom_rot = mathutils.Euler(tuple(link[geomsrc]['pose'][3:]), 'XYZ').to_matrix().to_4x4()
                else:
                    urdf_geom_loc = mathutils.Matrix.Identity(4)
                    urdf_geom_rot = mathutils.Matrix.Identity(4)
                if 'scale' in link[geomsrc]['geometry']:
                    urdf_geom_scale = link[geomsrc]['geometry']['scale']
                else:
                    urdf_geom_scale = [1.0, 1.0, 1.0]
                    #bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
                geom = bpy.data.objects[link[geomsrc]['name']]
                bpy.ops.object.select_all(action="DESELECT")
                geom.select = True
                parentLink.select = True
                bpy.context.scene.objects.active = parentLink
                bpy.ops.object.parent_set()
                geom.matrix_world = parentLink.matrix_world
                geom.matrix_local = urdf_geom_loc * urdf_geom_rot
                geom.scale = urdf_geom_scale

    def createGeometry(self, link, geomsrc):
        newgeom = None
        if link[geomsrc]['geometry'] is not {}:
            geomtype = link[geomsrc]['geometry']['geometryType']
            if 'name' in link[geomsrc]:
                name = link[geomsrc]['name']
            else:
                name = geomsrc+'_'+link['name']
                link[geomsrc]['name'] = name # TODO: Not sure if this should really be done here!
            # create the Blender object
            if geomtype == 'mesh':
                filetype = link['filename'].split('.')[-1]
                if filetype == 'obj' or filetype == 'OBJ':
                    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, link['filename'])) #FIXME: we don't use the geomsrc variable here (problem with local and global filename)
                    newgeom = bpy.context.object
                    newgeom.name = name
                elif filetype == 'stl' or filetype == 'STL':
                    bpy.ops.import_mesh.stl(filepath=os.path.join(self.path, link['filename']))
                    newgeom = bpy.context.object
                    newgeom.name = name
            elif geomtype == 'box':
                newgeom = createPrimitive(name,
                                          geomtype,
                                          link[geomsrc]['geometry']['size'],
                                          mtdefs.layerTypes[geomsrc]
                                          )
            elif geomtype == 'cylinder':
                newgeom = createPrimitive(name,
                                          geomtype,
                                          (link[geomsrc]['geometry']['radius'], link[geomsrc]['geometry']['length']),
                                          mtdefs.layerTypes[geomsrc]
                                          )
            elif geomtype == 'sphere':
                newgeom = createPrimitive(name,
                                          geomtype,
                                          link[geomsrc]['geometry']['radius'], #tuple would cause problem here
                                          mtdefs.layerTypes[geomsrc]
                                          )
            else:
                print("### ERROR: Could not determine geometry type of link", link['name'] + '. Placing empty coordinate system.')
            return newgeom

    def createInertial(self, name, inertial):
        inert = createPrimitive('inertial_'+name, 'sphere', [0.01], 0, 'None', (0, 0, 0))
        #obj = bpy.context.object
        #obj.name = name
        for prop in inertial:
            if prop not in ['pose'] and inertial[prop] is not None:
                inert[prop] = inertial[prop]

    def createLink(self, link):
        print("Creating link", link['name'])
        #create base object ( =armature)
        bpy.ops.view3d.snap_cursor_to_center()
        bpy.ops.object.armature_add(layers=defLayers([0]))
        newlink = bpy.context.active_object #print(bpy.context.object) #print(bpy.context.scene.objects.active) #bpy.context.selected_objects[0]
        newlink.name = link['name']
        if newlink.name != link['name']:
            print("Warning, name conflict!")
        #place inertial
        if 'inertial' in link:
            self.createInertial(link['name'], link['inertial'])
        # place visual
        if 'visual' in link:
            if 'geometry' in link['visual']:
                visual = self.createGeometry(link, 'visual')
        # place collision
        if 'collision' in link:
            if 'geometry' in link['collision']:
                collision = self.createGeometry(link, 'collision')

    def createBlenderModel(self): #TODO: solve problem with duplicated links (linklist...namespaced via robotname?)
        """Creates the blender object representation of the imported model."""
        print("\n\nCreating Blender model...")
        for l in self.robot['links']:
            print(l + ', ', end='')
            link = self.robot['links'][l]
            self.createLink(link)

        #build tree recursively and correct translation & rotation on the fly
        for l in self.robot['links']:
            if not 'parent' in self.robot['links'][l]:
                root = self.robot['links'][l]
        print("\n\nPlacing links...")
        self.placeChildLinks(root)
        for link in self.robot['links']:
            self.placeLinkSubelements(self.robot['links'][link])



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
            newlink = self.parseLink(link)
            #write link to list
            links[newlink['name']] = newlink
            #print(newlink)
        self.robot['links'] = links

        #write joints to dictionary
        joints = {}
        print("\n\nParsing joints..")
        for joint in self.root.iter('joint'):
            newjoint, pose = self.parseJoint(joint)
            self.robot['links'][newjoint['child']]['pose'] = pose
            joints[newjoint['name']] = newjoint
            #print(newjoint)
        self.robot['joints'] = joints

        #find any links that still have no pose (most likely because they had no parent)
        for link in links:
            if not 'pose' in links[link]:
                links[link]['pose'] = defaults.idtransform
            #print(link, links[link]['pose'])

        #write parent-child information to nodes
        print("\n\nWriting parent-child information to nodes..")
        for j in self.robot['joints']:
            joint = self.robot['joints'][j]
            self.robot['links'][joint['child']]['parent'] = joint['parent']
            print(joint['parent'] + ', ', end='')

        materials = [] #TODO: build dictionary entry for materials
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

    def parseLink(self, link):
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
                newlink['visual']['geometry']['geometryType'] = geometry[0].tag
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
                newlink['collision']['pose'] = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
            else:
                newlink['collision']['pose'] = defaults.idtransform
            geometry = collision.find('geometry')
            if geometry is not None:
                newlink['collision']['geometry'] = {a: parse_text(geometry[0].attrib[a]) for a in geometry[0].attrib}
                newlink['collision']['geometry']['geometryType'] = geometry[0].tag
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
        return newlink

    def parseJoint(self, joint):
        print(joint.attrib['name']+', ', end='')
        newjoint = {a: joint.attrib[a] for a in joint.attrib}
        origin = joint.find('origin')
        newjoint['parent'] = joint.find('parent').attrib['link']
        newjoint['child'] = joint.find('child').attrib['link']
        pose = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
        #axis
        #calibration
        #dynamics
        #limit
        #mimic
        #safety_controller
        return newjoint, pose

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
    cleanUpScene()
    bpy.ops.import_robot_model('INVOKE_DEFAULT')

if __name__ == '__main__':
    main()