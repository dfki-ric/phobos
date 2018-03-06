bl_info = {
    'version': (1, 0),
    'blender': (2, 79, 0),
    'author': 'sambler',
    'name': 'Custom Object Factory',
    'location': 'blender',
    'description': 'Create multiple custom operators that create objects' ,
    'warning': 'only for testing',
    'category': 'test',
}

import bpy
import os

yaml_categ = [
    'none',
    'ray',
    'camera',
    'environment'
]

yaml_sensors = {
    'ray': {'laser': {'energy': 20}, 'deathray': {'power': 40, 'death': 'short', 'doom': 0.5, 'abort': False}},
    'camera': {'spycam': {'tinyness': 'extreme', 'battery': 2.5}, 'omnieye': {'roentgen': True}},
    'environment': {'thermo': {'min_temp': 20., 'max_temp': 40.}}
}

# this holds the custom operators so we can cleanup when turned off
custom_items = []

sensor_operators = []

def clearCustomItems():
    for c in custom_items:
        bpy.utils.unregister_class(c)


class TestStuff(bpy.types.PropertyGroup):
    name = bpy.props.StringProperty()
    intProp = bpy.props.IntProperty()
    boolProp = bpy.props.BoolProperty()
    stringProp = bpy.props.StringProperty()
    floatProp = bpy.props.FloatProperty()


def registerSensorOperator(category, name):
    try:
        bpy.utils.unregister_class(TemporarySensorOperator)
    except UnboundLocalError:
        pass
    operatorIdName = 'object.add_sensor_' + name
    print(operatorIdName)
    class TemporarySensorOperator(bpy.types.Operator):
        """Temporary sensor operator"""
        bl_idname = operatorIdName
        bl_label = 'Add a ' + name + ' sensor'
        bl_description ='Adds a ' + name + ' sensor to the scene.'
        bl_options = {'REGISTER'}

        sensor_data = bpy.props.CollectionProperty(type=TestStuff)
        categ = category
        sensor = name
        
        def draw(self, context):
            layout = self.layout
            if 'sensor_data' in dir(self):
                for i in range(len(self.sensor_data)):
                    name = self.sensor_data[i].name[2:]
                    if self.sensor_data[i].name[0] == 'i':
                        layout.prop(self.sensor_data[i], 'intProp', text=name)
                    elif self.sensor_data[i].name[0] == 'b':
                        layout.prop(self.sensor_data[i], 'boolProp', text=name)
                    elif self.sensor_data[i].name[0] == 's':
                        layout.prop(self.sensor_data[i], 'stringProp', text=name)
                    elif self.sensor_data[i].name[0] == 'f':
                        layout.prop(self.sensor_data[i], 'floatProp', text=name)
                    
        def invoke(self, context, event):
            data = yaml_sensors[self.categ][self.sensor]
            for dat in data.keys():
                piece = self.sensor_data.add()
                prefix = ''
                print(type(data[dat]))
                if type(data[dat]) is int:
                    piece.intProp = data[dat]
                    prefix = 'i'
                elif type(data[dat]) is bool:
                    piece.boolProp = data[dat]
                    prefix = 'b'
                elif type(data[dat]) is str:
                    piece.stringProp = data[dat]
                    prefix = 's'
                elif type(data[dat]) is float:
                    piece.floatProp = data[dat]
                    prefix = 'f'
                piece.name = prefix + '_' + dat
                print(piece.name)
            return context.window_manager.invoke_props_dialog(self)
            
        def execute(self, context):
            print(self.sensor_data)

            return {'FINISHED'}
    bpy.utils.register_class(TemporarySensorOperator)
    return operatorIdName


def sensorlist(self, context):
    if self.categ == 'none':
        return [('none',) * 3]
    items = []
    for sensor in yaml_sensors[self.categ]:
        items.append((sensor,) * 3)
    return items


class AddingOperator(bpy.types.Operator):
    """Base for adding a custom object"""
    bl_idname = 'object.adding_operator'
    bl_label = 'Add Item'
    bl_description ='Base operator for adding custom objects'
    bl_options = {'REGISTER', 'UNDO'}

    categ = bpy.props.EnumProperty(items=tuple((cat,) * 3 for cat in yaml_categ))
    sensor = bpy.props.EnumProperty(items=sensorlist)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, 'categ', expand=True)
        
        #layout.template_list('whatever', '', self, 'test', active_dataptr)
        if self.categ != 'none':
            layout.prop(self, 'sensor')

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)
    
    def check(self, context):
        return True

    def execute(self, context):
        operatorName = registerSensorOperator(self.categ, self.sensor)
        operator = eval('bpy.ops.' + operatorName + "('INVOKE_DEFAULT')")
        print(operator)
        return {'FINISHED'}


class TestPanel(bpy.types.Panel):
    """Contains all general phobos tools in the Phobos viewport toolbar"""
    bl_idname = "TOOLS_PT_TEST_TOOLS"                                                                                                                                                  
    bl_label = "General Tools"                                                                                                                                                           
    bl_space_type = 'VIEW_3D'                                                                                                                                                            
    bl_region_type = 'TOOLS'                                                                                                                                                             
    bl_category = 'Tools'
    
    def draw(self, context):
        layout = self.layout
        layout.operator('object.adding_operator')
        layout.operator('object.add_sensor_deathray')
        layout.operator('object.add_sensor_ray')
        layout.operator('object.add_sensor_spycam')


def register():
    bpy.utils.register_class(TestStuff)
    bpy.utils.register_class(AddingOperator)
    bpy.utils.register_class(TestPanel)

def unregister():
    bpy.utils.register_class(TestStuff)
    clearCustomItems()
    bpy.utils.unregister_class(CustomObjectBase)
    bpy.utils.register_class(AddingOperator)
    bpy.utils.unregister_class(TestPanel)

if __name__ == '__main__':
    register()
    
    
#import bpy

#def update_enum(self, context):
#    # self = current scene in an EnumProperty callback!
#    print(self.my_enum)
#    eval('bpy.ops.%s()' % self.my_enum)


#class LayoutDemoPanel(bpy.types.Panel):
#    bl_label = "Test"
#    bl_space_type = 'VIEW_3D'
#    bl_region_type = 'UI'

#    def draw(self, context):
#        layout = self.layout

#        layout.label(text="Enum prop")

#        scene = context.scene
#        layout.prop(scene, "my_enum", expand=True)


#def register():
#    bpy.utils.register_class(LayoutDemoPanel)

#    bpy.types.Scene.my_enum = bpy.props.EnumProperty(
#            name = "My enum",
#            description = "My enum description",
#            items = [
#                ("mesh.primitive_cube_add", "Cube", "Create a cube"),
#                ("mesh.primitive_uv_sphere_add", "Sphere", "Create a Sphere"),              
#            ],
#            update=update_enum
#        )

#def unregister():
#    bpy.utils.unregister_class(LayoutDemoPanel)

#    bpy.types.Scene.my_enum


#if __name__ == "__main__":
#    register()