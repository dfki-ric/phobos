from copy import deepcopy

import bpy
import mathutils
import numpy as np

from .. import defs
from .. import reserved_keys
from ..model import joints as jointmodel
from ..model.materials import assignMaterial
from ..phoboslog import log, ErrorMessageWithBox
from ..utils import blender as bUtils
from ..utils import editing as eUtils
from ..utils import io as ioUtils
from ..utils import naming as nUtils
from ..utils import selection as sUtils

from ... import core
from ...io import representation, sensor_representations
from ...utils import misc
from ...utils.resources import get_default_joint

"""
Factory functions for creating blender Instances from phobos.io instance
"""


def createMaterial(material: representation.Material):
    newmat = bpy.data.materials.new(material.name)
    newmat.use_nodes = True
    material_output = newmat.node_tree.nodes.get('Material Output')
    material_output.location = (400, 0)

    # Exchange the default Material
    principled_bsdf = newmat.node_tree.nodes.get('Principled BSDF')
    if principled_bsdf is not None:
        newmat.node_tree.nodes.remove(principled_bsdf)
    shader_node = newmat.node_tree.nodes.new('ShaderNodeEeveeSpecular')
    shader_node.location = (0, 0)
    newmat.node_tree.links.new(shader_node.outputs[0], material_output.inputs[0])

    if material.diffuseTexture is not None:
        diffuse_tex_node = newmat.node_tree.nodes.new('ShaderNodeTexImage')
        diffuse_tex_node.image = material.diffuseTexture.load_image()
        diffuse_tex_node.location = (-400, 0)
        newmat.node_tree.links.new(diffuse_tex_node.outputs[0], shader_node.inputs[0])
    shader_node.inputs['Base Color'].default_value = material.diffuse

    if material.normalTexture is not None:
        normal_tex_node = newmat.node_tree.nodes.new('ShaderNodeTexImage')
        normal_tex_node.image = material.normalTexture.load_image()
        normal_tex_node.location = (-600, -400)
        normal_map_node = newmat.node_tree.nodes.new('ShaderNodeNormalMap')
        normal_map_node.location = (-300, -400)
        newmat.node_tree.links.new(normal_tex_node.outputs[0], normal_map_node.inputs[1])
        newmat.node_tree.links.new(normal_map_node.outputs[0], shader_node.inputs[5])

    if material.specular is not None:
        shader_node.inputs['Specular'].default_value = material.specular
        newmat.specular_intensity = 1.

    if material.emissive is not None:
        shader_node.inputs['Emissive Color'].default_value = material.emissive

    if material.shininess is not None:
        shader_node.inputs['Roughness'].default_value = 1-(material.shininess/100 if material.shininess > 1 else material.shininess)

    shader_node.inputs['Transparency'].default_value = 1-(material.diffuse[-1] if len(material.diffuse) == 4 else 1)

    return newmat


def createGeometry(viscol, geomsrc, linkobj=None):
    bpy.ops.object.select_all(action='DESELECT')
    geometry = viscol.geometry
    geometry_type = None
    if isinstance(geometry, representation.Mesh):
        geometry_type = "mesh"
        bpy.ops.object.add(type='MESH')
        newgeom = bpy.context.active_object
        newgeom.name = viscol.name
        newgeom.data = geometry.load_mesh()
        newgeom.scale = geometry.scale
    elif isinstance(geometry, representation.Box) or isinstance(geometry, representation.Cylinder) or isinstance(geometry, representation.Sphere):
        dimensions = None
        if isinstance(geometry, representation.Box):
            dimensions = geometry.size
            geometry_type = "box"
        elif isinstance(geometry, representation.Cylinder):
            dimensions = (geometry.radius, geometry.length)
            geometry_type = "cylinder"
        elif isinstance(geometry, representation.Sphere):
            dimensions = geometry.radius
            geometry_type = "sphere"
        log("Creating primitive for {0}: {1}".format(type(geometry), viscol.name), 'INFO')
        newgeom = bUtils.createPrimitive(viscol.name, geometry_type, dimensions, phobostype=geomsrc, pmaterial="phobos_collision")
        newgeom.select_set(True)
    else:
        log(
            "Unknown geometry type of "
            + type(geometry)
            + viscol.name
            + '. Placing empty coordinate system.',
            "ERROR",
        )
        return None

    newgeom.phobostype = geomsrc

    # from here it's the same for both meshes and primitives
    newgeom['geometry/type'] = geometry_type
    if geomsrc == 'visual':
        if hasattr(viscol, "material") and viscol.material is not None:
            assignMaterial(newgeom, viscol.material)
        else:
            log('No material for visual {}.'.format(viscol.name), 'WARNING')
    elif newgeom.data.users == 1:
        assignMaterial(newgeom, "phobos_collision")

    # write generic custom properties
    for prop, value in viscol.to_yaml().items():
        if prop not in reserved_keys.VISCOL_KEYS:
            if type(value) == dict:
                for k, v in value.items():
                    newgeom[f"{prop}/{k}"] = v
            else:
                newgeom[prop] = value

    # place geometric object relative to its parent link
    if linkobj:
        eUtils.parentObjectsTo(newgeom, linkobj)
        newgeom.matrix_local = mathutils.Matrix(
            viscol.joint_relative_origin.to_matrix()
            if viscol._related_robot_instance is not None and viscol.relative_to != linkobj.name else viscol.origin.to_matrix()
        )

    bUtils.sortObjectToCollection(newgeom, cname=geomsrc)
    # # make object smooth
    # eUtils.smoothen_surface(newgeom)

    return newgeom


def createInertial(inertial: representation.Inertial, newlink: bpy.types.Object, size=0.03, errors=None, adjust=False, logging=False):
    if errors is not None and (len(errors) > 0 or type(errors) != list) and not adjust:
        log(f'Can not create inertial object for {newlink.name}.', 'ERROR')
        log('Erros were:', 'ERROR')
        for e in errors:
            log("  "+e.message, 'ERROR')
            log("   "+str(e.information), 'ERROR')

    assert newlink is not None

    # create new inertial object
    name = nUtils.getUniqueName('inertial_' + newlink.name, bpy.data.objects)
    inertialobject = bUtils.createPrimitive(
        name,
        'box',
        (size,) * 3,
        defs.layerTypes["inertial"],
        pmaterial='phobos_inertial',
        phobostype='inertial',
    )
    eUtils.parentObjectsTo(inertialobject, newlink)
    inertialobject.matrix_local = mathutils.Matrix(inertial.joint_relative_origin.to_matrix())
    sUtils.selectObjects((inertialobject,), clear=True, active=0)

    inertialobject["inertia"] = inertial.inertia.to_list()
    inertialobject["mass"] = inertial.mass

    # write generic custom properties
    for prop, value in inertial.to_yaml().items():
        if prop not in reserved_keys.INERTIAL_KEYS:
            if type(value) == dict:
                for k, v in value.items():
                    inertialobject[f"{prop}/{k}"] = v
            else:
                inertialobject[f"{prop}"] = value

    return inertialobject


def createLink(link):
    log("Creating link object '{}'...".format(link.name), 'DEBUG', prefix='\n')
    # create armature/bone
    bUtils.toggleLayer('link', True)
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.armature_add()
    newlink = bpy.context.active_object

    # give it a proper name
    newlink.phobostype = 'link'
    if link.name in bpy.data.objects.keys():
        log('Object with name of new link already exists: ' + link.name, 'WARNING')
    nUtils.safelyName(newlink, link.name)

    # create geometric elements
    log(
        "Creating visual and collision objects for link '{0}':\n{1}".format(
            link.name, '    \n'.join([elem.name for elem in link.visuals + link.collisions])
        ),
        'DEBUG',
    )
    bound_box = (0, 0, 0)
    geometries = []
    for viscol in link.visuals:
        geom = createGeometry(viscol, 'visual')
        bound_box = (
            max(bound_box[0], max([c[0] for c in geom.bound_box])),
            max(bound_box[1], max([c[1] for c in geom.bound_box])),
            max(bound_box[2], max([c[2] for c in geom.bound_box])),
        )
        geometries.append((geom, viscol))
    for viscol in link.collisions:
        geom = createGeometry(viscol, "collision")
        bound_box = (
            max(bound_box[0], max([c[0] for c in geom.bound_box])),
            max(bound_box[1], max([c[1] for c in geom.bound_box])),
            max(bound_box[2], max([c[2] for c in geom.bound_box])),
        )
        geometries.append((geom, viscol))

    # set the size of the link
    scale = max(bound_box)
    if scale == 0.:
        scale = 0.2

    # use scaling factor provided by user
    if hasattr(link, "scale") and link.scale is not None:
        scale *= link.scale
    newlink.scale = (scale, scale, scale)
    bpy.ops.object.select_all(action='DESELECT')
    newlink.select_set(True)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True, properties=False)

    # parent geometries
    for newgeom, viscol in geometries:
        eUtils.parentObjectsTo(newgeom, newlink)
        _scale = deepcopy(newgeom.scale)
        newgeom.matrix_local = mathutils.Matrix(viscol.joint_relative_origin.to_matrix())
        newgeom.scale = _scale

    # create inertial
    if link.inertial is not None:
        createInertial(link.inertial, newlink)

    # write generic custom properties
    for prop, value in link.to_yaml().items():
        if prop not in reserved_keys.LINK_KEYS:
            if type(value) == dict:
                for k, v in value.items():
                    newlink[f"link/{prop}/{k}"] = v
            else:
                newlink[f"link/{prop}"] = value

    bUtils.sortObjectToCollection(newlink, 'link')
    return newlink


def createJoint(joint: representation.Joint, linkobj=None):
    # try deriving link object from joint['child']
    if not linkobj:
        linkobj = sUtils.getObjectByName(joint.child)
        if isinstance(linkobj, list):
            log(
                "Could not identify object to define joint '{0}'.".format(joint.name),
                'ERROR',
            )
            return

    # make sure the proper joint name is kept
    if joint.name != linkobj.name:
        linkobj['joint/name'] = joint.name

    # select the link object
    bUtils.activateObjectCollection(linkobj)
    sUtils.selectObjects([linkobj], clear=True, active=0)

    # set axis
    if joint.axis is not None:
        if mathutils.Vector(tuple(joint.axis)).length == 0.:
            log('Axis of joint {0} is of zero length: '.format(joint.name), 'ERROR')
        joint.axis = (np.array(joint.axis) / np.linalg.norm(joint.axis)).tolist()
        if np.linalg.norm(joint.axis) != 0:
            bpy.ops.object.mode_set(mode='EDIT')
            editbone = linkobj.data.edit_bones[0]
            length = editbone.length
            editbone.tail = editbone.head + mathutils.Vector(tuple(joint.axis)).normalized() * length
            bpy.ops.object.mode_set(mode='OBJECT')

    # add constraints to the joint
    jointmodel.setJointConstraints(
        joint=linkobj,
        jointtype=joint.joint_type,
        lower=getattr(joint.limit, "lower", None),
        upper=getattr(joint.limit, "upper", None),
        velocity=getattr(joint.limit, "velocity", None),
        effort=getattr(joint.limit, "effort", None),
        spring=getattr(joint.dynamics, "spring", None),
        damping=getattr(joint.dynamics, "damping", None),
        axis=joint.axis
    )

    # write generic custom properties
    for prop, value in joint.to_yaml().items():
        if prop not in reserved_keys.JOINT_KEYS:
            if type(value) == dict:
                for k, v in value.items():
                    linkobj[f"joint/{prop}/{k}"] = v
            else:
                linkobj[f"joint/{prop}"] = value

    log("Assigned joint information to {}.".format(linkobj.name), 'DEBUG')


def createInterface(interface: representation.Interface, parent, scale=None):
    bUtils.toggleLayer('interface', value=True)

    if not parent:
        parent = sUtils.getObjectByName(interface.parent)
    assert parent.phobostype == "link"
    templateobj = ioUtils.getResource(('interface', 'default', interface.direction))
    scale = parent.data.bones[0].length if scale is None else scale
    ifobj = bUtils.createPrimitive(
        interface.name,
        'box',
        (scale,) * 3,
        player=defs.layerTypes['interface'],
        phobostype='interface',
    )
    nUtils.safelyName(ifobj, interface.name, 'interface')
    ifobj.data = templateobj.data
    ifobj.phobostype = "interface"
    ifobj['type'] = interface.type
    ifobj['direction'] = interface.direction
    eUtils.parentObjectsTo(ifobj, parent)
    ifobj.matrix_local = interface.origin.to_matrix()
    ifobj.scale = (scale,) * 3

    # write generic custom properties
    for prop, value in interface.to_yaml().items():
        if prop not in reserved_keys.INTERFACE_KEYS:
            if type(value) == dict:
                for k, v in value.items():
                    ifobj[f"{prop}/{k}"] = v
            else:
                ifobj[f"{prop}"] = value

    # select the new interface
    sUtils.selectObjects([ifobj], clear=True, active=0)
    return ifobj


def createSensor(sensor: sensor_representations.Sensor, linkobj=None):
    bUtils.toggleLayer('sensor', value=True)

    newsensor = bUtils.createPrimitive(
        sensor.name,
        'box',
        [1, 1, 1],
        None,
        phobostype='sensor'
    )

    # use resource name provided as: "resource:whatever_name"
    resource_obj = ioUtils.getResource(['sensor'] + defs.def_settings['sensors'][sensor.blender_type]['shape'].split('://')[1].split('_'))
    if resource_obj:
        log("Assigned resource mesh and materials to new sensor object.", 'DEBUG')
        newsensor.data = resource_obj.data
        newsensor.scale = (defs.def_settings['sensors'][sensor.blender_type]['size'],) * 3
    else:
        log("Could not use resource mesh for sensor. Default cube used instead.", 'WARNING')

    # assign the parent if available
    if linkobj is not None:
        eUtils.parentObjectsTo(newsensor, nUtils.getObjectName(linkobj) if type(linkobj) == str else linkobj)
        # newsensor.matrix_local = sensor.origin.to_matrix() #TODO

    # set sensor properties
    newsensor.phobostype = 'sensor'
    newsensor.name = sensor.name
    newsensor['type'] = sensor.blender_type

    # write generic custom properties
    for prop, value in sensor.to_yaml().items():
        if type(value) == dict:
            for k, v in value.items():
                newsensor[f"{prop}/{k}"] = v
        else:
            newsensor[f"{prop}"] = value

    # throw warning if type is not known
    # TODO we need to link this error to the sensor type specifications
    if sensor.blender_type not in [key.lower() for key in defs.def_settings['sensors']]:
        log(
            "Sensor " + sensor.name + " is of unknown/custom type: " + sensor.blender_type + ".",
            'WARNING',
        )

    # select the new sensor
    sUtils.selectObjects([newsensor], clear=True, active=0)
    return newsensor

def createSubmechanism(submechanism, linkobj=None):
    bUtils.toggleLayer('submechanism', value=True)

    newsubm = bUtils.createPrimitive(
        submechanism.contextual_name,
        'box',
        [0.05, 0.05, 0.05],
        None,
        phobostype='submechanism'
    )

    # TODO: Create submechanism objects
    # use resource name provided as: "resource:whatever_name"
    resource_obj = ioUtils.getResource(['submechanism','default'])
    if resource_obj:
        log("Assigned resource mesh and materials to new sensor object.", 'DEBUG')
        newsubm.data = resource_obj.data
        newsubm.scale = (1, 1, 1)
    else:
        log("Could not use resource mesh for submechanism. Default cube used instead.", 'WARNING')

    # assign the parent if available
    if linkobj is not None:
        eUtils.parentObjectsTo(newsubm, nUtils.getObjectName(linkobj) if type(linkobj) == str else linkobj)
        # newsensor.matrix_local = sensor.origin.to_matrix() #TODO

    # set sensor properties
    newsubm.phobostype = 'submechanism'

    # Handle submechanism keys
    for prop in reserved_keys.SUBMECHANISM_KEYS:
        attr = getattr(submechanism, prop)
        if type(attr) == str:
            value = attr
        elif type(attr) == dict:
            value = {}
            for key, joint in attr.items():
                value[key] = sUtils.getObjectByName(str(joint))
        elif attr is not None: # Is a list
            value = [sUtils.getObjectByName(str(j)) for j in attr]
        else:
            log(f"Unknown property type {type(attr)}", "WARNING")
            continue
        newsubm[f"{prop}"] = value

    # write generic custom properties
    for prop, value in submechanism.to_yaml().items():
        if prop not in reserved_keys.SUBMECHANISM_KEYS:
            if type(value) == dict:
                for k, v in value.items():
                    newsubm[f"{prop}/{k}"] = v
            else:
                newsubm[f"{prop}"] = value

    # throw warning if type is not known
    # TODO check if type is known
    # TODO we need to link this error to the type specifications


    # select the new submechanism
    #sUtils.selectObjects([newsubm], clear=True, active=0)
    return newsubm


def createMotor(motor: representation.Motor, linkobj: bpy.types.Object):
    # write generic custom properties
    for prop, value in motor.__dict__.items():
        if prop in motor.get_refl_vars():
            if type(value) == dict:
                for k, v in value.items():
                    linkobj[f"motor/{prop}/{k}"] = v
            else:
                linkobj[f"motor/{prop}"] = value


def createAnnotation(ga: representation.GenericAnnotation, parent=None, size=0.1):
    if not parent and ga.GA_parent is not None:
        parent = sUtils.getObjectByName(ga.GA_parent)

    annot_obj = bUtils.createPrimitive(
        f"{ga.GA_category}:{ga.GA_name if ga.GA_name is not None else 'unnamed'}",
        'box',
        [1, 1, 1],
        player=defs.layerTypes['annotation'],
        plocation=None if parent is None else parent.matrix_world.to_translation(),
        phobostype='annotation'
    )
    annot_obj.scale = (size,) * 3
    resource = ioUtils.getResource(['annotation', ga.GA_category])
    if resource:
        annot_obj.data = resource.data
    else:
        annot_obj.data = ioUtils.getResource(['annotation', 'default']).data

    if parent is not None:
        # make sure all layers are enabled for parenting
        originallayers = {}
        for name, coll in bpy.context.window.view_layer.layer_collection.children.items():
            originallayers[name] = coll.exclude
            coll.exclude = False
        # parent annotation object
        eUtils.parentObjectsTo(annot_obj, parent)
        # Restore original layers
        for key, value in originallayers.items():
            bpy.context.window.view_layer.layer_collection.children[key].exclude = value

        annot_obj["$include_parent"] = True
    else:
        annot_obj["$include_parent"] = False

    annot_obj["$include_transform"] = ga.GA_transform is None

    props = ga.to_yaml()

    for k, v in misc.flatten_dict(props).items():
        annot_obj[k] = v

    return annot_obj


def createRobot(robot: core.Robot):
    log("Creating Blender model...", 'INFO', prefix='\n' + '-' * 25 + '\n')

    log("  Initializing materials... ({} total)".format(len(robot.materials)), 'INFO')
    for mat in robot.materials:
        createMaterial(mat)

    newobjects = []
    newlinks = {}
    log("  Creating links... ({} total)".format(len(robot.links)), 'INFO')
    for link in robot.links:
        newlink = createLink(link)
        parent_joint = robot.get_parent(link)
        if parent_joint is not None:
            newlink.matrix_world = mathutils.Matrix(robot.get_transformation(parent_joint))
        else:
            newlink.matrix_world = mathutils.Matrix(np.identity(4))
        newlinks[link.name] = newlink
        newobjects.append(newlink)

    log("Creating joints... ({} total)".format(len(robot.joints)), 'INFO', prefix='\n')
    bUtils.toggleLayer('link', True)
    for joint in robot.joints:
        parent = newlinks[joint.parent]
        child = newlinks[joint.child]
        child.matrix_world = parent.matrix_world
        eUtils.parentObjectsTo(child, parent)
        child.matrix_local = mathutils.Matrix(joint.joint_relative_origin.to_matrix())
        createJoint(joint, child)

    log("Creating {} motors...".format(len(robot.motors)), 'INFO')
    for motor in robot.motors:
        createMotor(motor, newlinks[robot.get_joint(motor.joint).child])

    log("Assigning model name: {}".format(robot.name), 'INFO')
    rootlink = newlinks[str(robot.get_root())]
    rootlink['model/name'] = robot.name
    rootlink.pose.bones[0].custom_shape = ioUtils.getResource(('link', 'root'))
    rootlink.location = (0, 0, 0)

    log("Creating {} sensors...".format(len(robot.sensors)), 'INFO')
    for sensor in robot.sensors:
        newobjects.append(createSensor(sensor, linkobj=None if sensor.frame is None else newlinks[sensor.frame]))

    log("Creating interfaces...", 'INFO')
    for interface in robot.interfaces:
        newobjects.append(createInterface(interface, newlinks[interface.parent]))

    # [TODO v2.1.0] Re-Add SRDF support
    # log("Creating groups...", 'INFO')
    # if 'groups' in model and model['groups']:
    #     for group in model['groups']:
    #         createGroup(model['groups'][group])
    # else:
    #     log("  No kinematic groups in model.", 'INFO')
    #
    # log("Creating chains...", 'INFO')
    # if 'chains' in model and model['chains']:
    #     for ch in model['chains']:
    #         createChain(model['chains'][ch])
    # else:
    #     log("  No kinematic chains in model.", 'INFO')

    # [TODO v2.1.0] Re-add light support
    # log("Creating lights...", 'INFO')
    # if 'lights' in model and model['lights']:
    #     for light in model['lights']:
    #         lightmodel.createLight(model['lights'][light])
    # else:
    #     log("  No lights in model.", 'INFO')

    # display new objects after import
    sUtils.selectObjects(newobjects, clear=True, active=0)
    eUtils.sortObjectsToLayers(newobjects)
    for obj in newobjects:
        bUtils.setObjectLayersActive(obj, extendlayers=True)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.view3d.view_selected()

    # update the scene
    bUtils.update()
