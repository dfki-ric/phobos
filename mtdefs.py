'''
Created on 7 Jan 2014

@author: kavonszadkowski
'''

marstypes = [tuple(['node']*3),
             ('joint', 'joint', 'joint'),
             ('motor', 'motor', 'motor'),
             ('sensor', 'sensor', 'sensor')]

type_properties = {
    "node": ('name', 'coll_bitmask'),
    "node_default": ('new_node', '65536'),
    "joint": ('name', 'node2'),
    "joint_default": ("new_joint", 'some_node'),
    "motor": ('name', 'motor_type'),
    "motor_default": ("new_motor", "1"),
    "sensor": ('name', 'sensor_type'),
    "sensor_default": ("new_sensor", "RaySensor")
    }

type_property_defaults = {
    'node': {'name': 'new_node',
             'coll_bitmask': '65536'}
    }
