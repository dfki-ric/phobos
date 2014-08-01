'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtinertia.py

Created on 13 Feb 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
import mathutils
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, FloatVectorProperty, EnumProperty, FloatProperty
import marstools.mtdefs as mtdefs
import marstools.mtutility as mtutility


def register():
    print("Registering mtinertia...")


def unregister():
    print("Unregistering mtinertia...")


def getDummyInertia():
    return calculateBoxInertia(0.001, (0.01, 0.01, 0.01,))


def calculateInertia(mass, geometry):
    inertia = None
    gt = geometry['geometryType']
    if gt == 'box':
        inertia = calculateBoxInertia(mass, geometry['size'])
    elif gt == 'cylinder':
        inertia = calculateCylinderInertia(mass, geometry["radius"], geometry["height"])
    elif gt == 'sphere':
        inertia = calculateSphereInertia(mass, geometry['radius'])
    elif gt == 'mesh':
        inertia = calculateEllipsoidInertia(mass, geometry['size'])
    return inertia


def calculateBoxInertia(mass, size):
    '''Returns upper diagonal of inertia tensor of a box as tuple.'''
    i = mass / 12
    ixx = i*(size[1]**2 + size[2]**2)
    ixy = 0
    ixz = 0
    iyy = i*(size[0]**2 + size[2]**2)
    iyz = 0
    izz = i*(size[0]**2 + size[1]**2)
    return [ixx, ixy, ixz, iyy, iyz, izz]


def calculateCylinderInertia(mass, r, h):
    '''Returns upper diagonal of inertia tensor of a cylinder as tuple.'''
    i = mass / 12 * (3 * r**2 + h**2)
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = 0.5 * mass * r**2
    return [ixx, ixy, ixz, iyy, iyz, izz]


def calculateSphereInertia(mass, r):
    '''Returns upper diagonal of inertia tensor of a sphere as tuple.'''
    i = 0.4 * mass * r**2
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = i
    return [ixx, ixy, ixz, iyy, iyz, izz]


def calculateEllipsoidInertia(mass, size):
    '''Returns upper diagonal of inertia tensor of an ellisoid as tuple.'''
    i = mass / 5
    ixx = i*(size[1]**2 + size[2]**2)
    ixy = 0
    ixz = 0
    iyy = i*(size[0]**2 + size[2]**2)
    iyz = 0
    izz = i*(size[0]**2 + size[1]**2)
    return [ixx, ixy, ixz, iyy, iyz, izz]


def inertiaListToMatrix(il):
    inertia = [[il[0], il[1], il[2]],
               [0.0, il[3], il[4]],
               [0.0, 0.0, il[5]]]
    return mathutils.Matrix(inertia)


def getInertiaRelevantObjects(link):
    '''Returns a list of visual and collision objects of a link. If pairs of visual and collision
    objects are detected, the visual objects are omitted to prevent redundant information. The function
    does not check whether mass information within pairs is consistent.'''
    tmpobjects = mtutility.getImmediateChildren(link, ['visual', 'collision'])
    objlist = [obj.name for obj in tmpobjects]
    inertiaobjects = []
    for obj in tmpobjects:
        if 'mass' in obj:
            if obj.MARStype == 'collision':
                inertiaobjects.append(obj)
            elif obj.MARStype == 'visual':
                collision = obj.name.replace('visual_', 'collision_')
                if not collision in objlist:
                    inertiaobjects.append(obj)
    return inertiaobjects


def calculateLinkInertiaData(inertials):
    '''Returns mass, center of mass and inertia of a link as a whole, taking a list of inertials.
    *mass* is of type double'''
    objects = []
    for o in inertials:
        try:
            objdict = {'name': o.name,
                       'mass': o['mass'],
                       'com': o.matrix_local.to_translation(),
                       'inertia': o['inertia']
                       }
        except KeyError:
            print('Inertial object ' + o.name + ' is missing data.')
        objects.append(objdict)
    mass, com, inertia = compound_inertia_analysis_3x3(objects)
    return mass, com, inertia#


def createInertial(obj):
    size = (0.05, 0.05, 0.05)
    rotation = (0, 0, 0)
    center = (0, 0, 0)
    if obj.MARStype == 'link':
        name = obj.name
    else:
        name = obj.name.replace(obj.MARStype+'_', '')
    inertial = mtutility.createPrimitive('inertial_' + name, 'box', size,
                                   mtdefs.layerTypes["inertial"], 'inertial', center, rotation)
    inertial.MARStype = 'inertial'
    bpy.ops.object.select_all(action="DESELECT")
    inertial.select = True
    bpy.context.scene.objects.active = inertial
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.select = True
    if obj.MARStyp == 'link':
        bpy.context.scene.objects.active = obj
        #TODO: sync masses (invoke operator?)
        #TODO: set inertia (invoke operator?)
    else:
        bpy.context.scene.objects.active = obj.parent
        #TODO: invoke setmass operator if mass not present
    inertial.matrix_world = obj.matrix_world
    bpy.ops.object.parent_set()
    return inertial

################################################################################
# From here on we have code modified from Berti's implementation

#import  numpy

def combine_cog_3x3(objects) :
    '''
    combine center the COG of a list of bodies given with their masses and their centers of gravity
    '''
    if objects == []:
        return 0.0, mathutils.Vector((0.0,)*3)
    combined_com = mathutils.Vector((0.0,)*3)
    combined_mass = 0
    for obj in objects:
        combined_com = combined_com + obj.matrix_local.to_translation() * obj['mass']
        combined_mass = obj['mass']
    combined_com = combined_com / combined_mass
    return (combined_mass, combined_com)


def _shift_cog_inertia_3x3(mass, cog, inertia_cog, ref_point=mathutils.Vector((0.0,)*3)) :
    '''
    shift inertia matrix, steiner theorem / parallel axis theorem, private method

    - without changing the orientation  -

    see SCISIC B.12 or featherstone 2.63, but not Selig (sign swap, not COG)

        c   = COG - O
        I_O = I_COG + m · c× (c× )T

        changed the formula to (Wikipedia):
        \mathbf{J} = \mathbf{I} + m \left[\left(\mathbf{R} \cdot \mathbf{R}\right) \mathbf{E}_{3} - \mathbf{R} \otimes \mathbf{R} \right],

        This was necessary as previous calculations founded on math libraries of cad2sim.
    '''
    # diff_vec
    c               = cog - ref_point
    c_outer         = mtutility.outerProduct(c)

    inertia_ref    = inertia_cog + mass * (c.dot(c) * mathutils.Matrix.Identity(3) - c_outer)

    return inertia_ref, c, c_outer


def shift_cog_inertia_3x3(mass, cog, inertia_cog, ref_point=mathutils.Vector((0.0,)*3)):
    '''
    shift inertia matrix, Steiner theorem / parallel axis theorem

        see SCISIC B.12 or Featherstone 2.63, but not Selig (sign swap, not COG)

    shift inertia matrix, proxy to _shift_inertia_3x3
    '''
    inertia_ref, c, c_outer = _shift_cog_inertia_3x3(mass, cog, inertia_cog, ref_point)

    return inertia_ref


# def spin_inertia_3x3(inertia_3x3, rotmat, interpretation="passive" ) :
#     '''
#     rotate the inertia matrix
#
#     active and passive interpretation
#
#         "passive"   -- the object stands still but the inertia is expressed with respect to a rotated reference frame
#         "active"    -- the object moves and therefore its inertia
#
#     consistent with 6x6 method :
#
#          active     -- consistent with   N'  =  (H^T)^{-1}  *  N  *  H^{-1}
#          passive    -- consistent with   N'  =  (H^T)       *  N  *  H
#
#     WHERE IS a COMBINED METHOD of shifted and rotated inertia ? does it exist ?
#     '''
#     R   = rotmat
#     R_T = rotmat.T
#     I   = inertia_3x3
#
#     if interpretation == "passive" :
#         # the object stands still but the inertia is expressed with respect to a rotated reference frame
#         rotated_inertia = (R_T.dot(I)).dot(R)
#
#     elif interpretation == "active" :
#         # the object moves and therefore its inertia
#         rotated_inertia = ((R.dot(I)).dot(R_T))
#
#     return rotated_inertia


def _compound_inertia_analysis_3x3(masses, cogs, inertias):
    '''
    '''
    total_mass, common_cog = combine_cog_3x3(masses, cogs)

    shifted_inertias = list()
    for mass_i, cog_i, inert_i in zip(masses, cogs, inertias):
        inert_i_at_common_cog = shift_cog_inertia_3x3(mass_i, cog_i, inert_i, common_cog)
        shifted_inertias.append(inert_i_at_common_cog)

    total_inertia_at_common_cog = sum(shifted_inertias)

    return total_mass, common_cog, total_inertia_at_common_cog


def compound_inertia_analysis_3x3(objects):
    '''
    Computes total mass, common center of mass and inertia matrix at CCOM
    '''

    masses          = [o['mass'] for o in objects]      # float
    cogs            = [o['com'] for o in objects]      # origin vector of collision object
    inertias        = [o['inertia'] for o in objects]   # 3x3 inertia matrix of collision object

    total_mass, common_cog, total_inertia_at_common_cog = _compound_inertia_analysis_3x3(masses, cogs, inertias)

    return total_mass, common_cog, total_inertia_at_common_cog

