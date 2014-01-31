'''
Created on 7 Jan 2014

@author: kavonszadkowski
'''

import bpy

def makeMaterial(name, diffuse, specular, alpha, diffuse_intensity=1.0):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = diffuse
    mat.diffuse_shader = 'LAMBERT'
    mat.diffuse_intensity = diffuse_intensity
    mat.specular_color = specular
    mat.specular_shader = 'COOKTORR'
    mat.specular_intensity = 0.5
    mat.alpha = alpha
    mat.ambient = 1
    return mat

def createMARSMaterials():
    materials = bpy.data.materials.keys()
    if not 'joint_sphere' in materials:
        makeMaterial('joint_sphere', (0, 1, 0), (1, 1, 1), 1)
    if not 'joint' in materials:
        makeMaterial('joint', (0, 0, 1), (1, 1, 1), 1)
    if not 'name' in materials:
        makeMaterial('name', (1, 1, 1), (1, 1, 1), 1)
    if not 'laserscanner' in materials:
        makeMaterial('laserscanner', (1.0, 0.08, 0.08), (1, 1, 1), 0.3)
    if not 'camera' in materials:
        makeMaterial('camera', (0.13, 0.4, 1), (1, 1, 1), 0.3, 0.8)
    if not 'tof-camera' in materials:
        makeMaterial('tof-camera', (0.44, 1, 0.735), (1, 1, 1), 0.3, 0.7)
    if not 'sensor' in materials:
        makeMaterial('sensor', (0.8, 0.3, 0), (1, 1, 1), 0.3, 0.7)
    if not 'controller' in materials:
        makeMaterial('controller', (0.518, 0.364, 0.8), (1, 1, 1), 0.3, 0.7)
    if not 'indicator1' in materials:
        makeMaterial('indicator1', (1, 0, 0), (1, 1, 1), 1)
    if not 'indicator2' in materials:
        makeMaterial('indicator2', (0, 1, 0), (1, 1, 1), 1)
    if not 'indicator3' in materials:
        makeMaterial('indicator3', (0, 0, 1), (1, 1, 1), 1)
    if not 'indicator4' in materials:
        makeMaterial('indicator4', (1, 0, 1), (1, 1, 1), 1)
    if not 'indicator5' in materials:
        makeMaterial('indicator5', (1, 1, 0), (1, 1, 1), 1)
    if not 'indicator6' in materials:
        makeMaterial('indicator6', (0, 1, 1), (1, 1, 1), 1)
