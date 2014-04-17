'''
File mtcheckdict.py

Created on 17 Apr 2014
'''

def check_geometry(geometry, owner_type, owner_key, link_key):
    '''
    '''
    notifications = ''
    if not 'type' in geometry:
        note = "CheckModel: Error, geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'type'."
        notifications += note + "\n"
        print(note)
    elif geometry['type'] == 'box' or geometry['type'] == 'plane':
        if not 'size' in geometry:
            note = "CheckModel: Error, box / plane type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'size'."
            notifications += note + "\n"
            print(note)
    elif geometry['type'] == 'sphere':
        if not 'radius' in geometry:
            note = "CheckModel: Error, sphere type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'radius'."
            notifications += note + "\n"
            print(note)
    elif geometry['type'] == 'cylinder':
        if not 'radius' in geometry:
            note = "CheckModel: Error, cylinder type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'radius'."
            notifications += note + "\n"
            print(note)
        if not 'length' in geometry:
            note = "CheckModel: Error, cylinder type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'length'."
            notifications += note + "\n"
            print(note)
    elif geometry['type'] == 'mesh':
        if not 'size' in geometry:
            note = "CheckModel: Error, mesh type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'size'."
            notifications += note + "\n"
            print(note)
        if not 'filename' in geometry:
            note = "CheckModel: Error, mesh type geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has no attribute 'filename'."
            notifications += note + "\n"
            print(note)
    else:
        note = "CheckModel: Error, geometry of " + owner_type + " '" + owner_key + "' of link '" + link_key + "' has invalid value for attribute 'type': '" + geometry['type'] + "'."
        notifications += note + "\n"
        print(note)
    return notifications

def check_visuals(visuals, link_key):
    '''
    '''
    notifications = ''
    for visual_key in visuals.keys():
        visual = visuals[visual_key]
        if not 'pose' in visual:
            note = "CheckModel: Error, visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'pose'."
            notifications += note + "\n"
            print(note)
        if not 'material' in visual:
            note = "CheckModel: Warning, visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'material'."
            notifications += note + "\n"
            print(note)
        else:
            material = visual[material_key]
            if not 'name' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'name'."
                notifications += note + "\n"
                print(note)
            if not 'diffuseColor' in material:
                note = "CheckModel: Error, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'diffuseColor'."
                notifications += note + "\n"
                print(note)
            if not 'ambientColor' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'ambientColor'."
                notifications += note + "\n"
                print(note)
            if not 'emissionColor' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'emissionColor'."
                notifications += note + "\n"
                print(note)
            if not 'specularColor' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'specularColor'."
                notifications += note + "\n"
                print(note)
            if not 'transparency' in material:
                note = "CheckModel: Warning, material of visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'transparency'."
                notifications += note + "\n"
                print(note)
        if not 'geometry' in visual:
            note = "CheckModel: Error, visual '" + visual_key + "' of link '" + link_key + "' has no attribute 'geometry'."
            notifications += note + "\n"
            print(note)
        else:
            notifications += check_geometry(visual['geometry'], 'visual', visual_key, link_key)
    return notifications
            
            
def check_collisions(collisions, link_key):
    '''
    '''
    notifications = ''
    for collision_key in collisions.keys():
        collision = collisions[collision_key]
        if not 'bitmask' in collision:
            note = "CheckModel: Warning, collision '" + collision_key + "' has no attribute 'bitmask'."
            notifications += note + "\n"
            print(note)
        if not 'geometry' in collision:
            note = "CheckModel: Error, collision '" + collision_key + "' of link '" + link_key + "' has no attribute 'geometry'."
            notifications += note + "\n"
            print(note)
        else:
            notifications += check_geometry(collision['geometry'], 'collision', collision_key, link_key)
        if not 'pose' in collision:
            note = "CheckModel: Error, collision '" + collision_key + "' has no attribute 'pose'."
            notifications += note + "\n"
            print(note)
        if not 'max_contacts' in collision:
            note = "CheckModel: Warning, collision '" + collision_key + "' has no attribute 'max_contacts'."
            notifications += note + "\n"
            print(note)
        elif collision['max_contacts'] < 1:
            note = "CheckModel: Note, attribute 'max_contacts' in collision '" + collision_key + "' should not be zero or negative."
            notifications += note + "\n"
            print(note)
    return notifications

def check_links(links):
    '''
    '''
    notifications = ''
    for link_key in links.keys():
        link = links[link_key]
        if not 'filename' in link:
            note = "CheckModel: Error, link '" + link_key + "' has no attribute 'filename'."
            notifications += note + "\n"
            print(note)
        if not 'pose' in link:
            note = "CheckModel: Error, link '" + link_key + "' has no attribute 'pose'."
            notifications += note + "\n"
            print(note)
        if not 'visual' in link:
            note = "CheckModel: Warning, link '" + link_key + "' has no attribute 'visual'."
            notifications += note + "\n"
            print(note)
        else:
            notifications += check_visuals(link['collision'], link_key)
        if not 'collision' in link:
            note = "CheckModel: Warning, link '" + link_key + "' has no attribute 'collision'."
            notifications += note + "\n"
            print(note)
        else:
            notifications += check_collisions(link['collision'], link_key)
        if not 'inertial' in link:
            note = "CheckModel: Warning, link '" + link_key + "' has no attribute 'inertial'."
            notifications += note + "\n"
            print(note)
        else:
            inertial = link['inertial']
            if not 'mass' in inertial:
                note = "CheckModel: Error, inertial in link '" + link_key + "' has no attribute 'mass'."
                notifications += note + "\n"
                print(note)
            if not 'inertia' in inertial:
                note = "CheckModel: Error, inertial in link '" + link_key + "' has no attribute 'inertia'."
                notifications += note + "\n"
                print(note)
    return notifications
                

def check_joints(joints):
    '''
    '''
    notifications = ''
    for joint_key in joints.keys():
        joint = joints[joint_key]
        if not 'parent' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'parent'."
            notifications += note + "\n"
            print(note)
        if not 'child' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'child'."
            notifications += note + "\n"
            print(note)
        if not 'jointType' in joint:
            note = "CheckModel: Error, joint '" + joint_key + "' has no attribute 'type'."
            notifications += note + "\n"
            print(note)
        elif joint['jointType'] not in ['hinge', 'continuous', 'linear']:
            note = "CheckModel: Error, joint '" + joint_key + "' has invalid value for attribute 'jointType': '" + joint['jointType'] + "'."
            notifications +=  note + '\n'
            print(note)
    return notifications
        
        

def check_dict(model):
    '''
    '''
    notifications = ''
    notifications += check_link(model['link'])
    notifications += check_joint(model['joint'])
    return notifications
    
