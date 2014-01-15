'''
Created on 7 Jan 2014

@author: kavonszadkowski
'''

import bpy

materials = {}

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
    if 'joint_sphere' not in materials:
        materials['joint_sphere'] = makeMaterial('joint_sphere', (0, 1, 0), (1, 1, 1), 1)
    if not 'joint' in materials:
        materials['joint'] = makeMaterial('joint', (0, 0, 1), (1, 1, 1), 1)
    if not 'name' in materials:
        materials['name'] = makeMaterial('name', (1, 1, 1), (1, 1, 1), 1)
    if not 'laserscanner' in materials:
        materials['laserscanner'] = makeMaterial('laserscanner', (1.0, 0.08, 0.08), (1, 1, 1), 0.3)
    if not 'camera' in materials:
        materials['camera'] = makeMaterial('camera', (0.13, 0.4, 1), (1, 1, 1), 0.3, 0.8)
    if not 'tof-camera' in materials:
        materials['tof-camera'] = makeMaterial('tof-camera', (0.44, 1, 0.735), (1, 1, 1), 0.3, 0.7)
    if not 'sensor' in materials:
        materials['sensor'] = makeMaterial('sensor', (0.8, 0.3, 0), (1, 1, 1), 0.3, 0.7)
    if not 'indicator1' in materials:
        materials['sensor'] = makeMaterial('indicator1', (1, 0, 0), (1, 1, 1), 1)
    if not 'indicator2' in materials:
        materials['sensor'] = makeMaterial('indicator2', (0, 1, 0), (1, 1, 1), 1)
    if not 'indicator3' in materials:
        materials['sensor'] = makeMaterial('indicator3', (0, 0, 1), (1, 1, 1), 1)
    if not 'indicator4' in materials:
        materials['sensor'] = makeMaterial('indicator4', (1, 0, 1), (1, 1, 1), 1)
    if not 'indicator5' in materials:
        materials['sensor'] = makeMaterial('indicator5', (1, 1, 0), (1, 1, 1), 1)
    if not 'indicator6' in materials:
        materials['sensor'] = makeMaterial('indicator6', (0, 1, 1), (1, 1, 1), 1)