bl_info = {
    'version': (1, 0),
    'blender': (2, 79, 0),
    'author': 'Mr. Anderson',
    'name': 'Temporary Operator Showcase',
    'location': 'blender',
    'description': 'Creates operators when executing another operator, based on the contents of a dictionary.' ,
    'warning': 'This is a test only!',
    'category': 'test',
}

import bpy


custom_dict = {
    'spoon': {'bent': True, 'dynamic': False},
    'smiths': {'amount': 55, 'angry': True},
    'matrix': {'greenness': 0.5, 'speed': 6.5}
}


class PropertyEntry(bpy.types.PropertyGroup):
    name = bpy.props.StringProperty()
    intProp = bpy.props.IntProperty()
    boolProp = bpy.props.BoolProperty()
    stringProp = bpy.props.StringProperty()
    floatProp = bpy.props.FloatProperty()


def registerTempOperator(name):
    # check for previous temporary Operator and unregister it
    try:
        bpy.utils.unregister_class(TempOperator)
    except UnboundLocalError:
        pass
    blender_id_name = 'object.add_' + name

    # create the temporary operator class
    class TempOperator(bpy.types.Operator):
        """Temporary operator"""
        bl_idname = blender_id_name
        bl_label = 'Add a ' + name + ' object.'
        bl_description ='Adds a ' + name + ' object to the scene.'
        bl_options = {'REGISTER'}

        operator_data = bpy.props.CollectionProperty(type=PropertyEntry)
        operator_name = name
        
        def draw(self, context):
            layout = self.layout

            # expose all properties in the collection to the user
            for i in range(len(self.operator_data)):
                propname = self.operator_data[i].name[2:]
                # choose the right Property type depending on name identifier
                if self.operator_data[i].name[0] == 'i':
                    layout.prop(self.operator_data[i], 'intProp', text=propname)
                elif self.operator_data[i].name[0] == 'b':
                    layout.prop(self.operator_data[i], 'boolProp', text=propname)
                elif self.operator_data[i].name[0] == 's':
                    layout.prop(self.operator_data[i], 'stringProp', text=propname)
                elif self.operator_data[i].name[0] == 'f':
                    layout.prop(self.operator_data[i], 'floatProp', text=propname)
                    
        def invoke(self, context, event):
            data = custom_dict[self.operator_name]
            for key in data.keys():
                item = self.operator_data.add()
                prefix = ''
                if type(data[key]) is int:
                    item.intProp = data[key]
                    prefix = 'i'
                elif type(data[key]) is bool:
                    item.boolProp = data[key]
                    prefix = 'b'
                elif type(data[key]) is str:
                    item.stringProp = data[key]
                    prefix = 's'
                elif type(data[key]) is float:
                    item.floatProp = data[key]
                    prefix = 'f'
                
                # add the identifier to specify the data type
                item.name = prefix + '_' + key
            
            # show operator as popup window
            return context.window_manager.invoke_props_dialog(self)
            
        def execute(self, context):
            # TODO write the contents of operator_data somewhere here
            # or do something awesome
            
            print('Data is in here:')
            print(self.operator_data)
            print([i.name for i in self.operator_data])
            return {'FINISHED'}
    
    # register the temporary class and return the operatorBlId
    bpy.utils.register_class(TempOperator)
    return blender_id_name


def operatorList(self, context):
    items = []
    for item in custom_dict:
        items.append((item,) * 3)
    return items


class CallingOperator(bpy.types.Operator):
    """Operator which calls the temporary operator"""
    bl_idname = 'object.calling_operator'
    bl_label = 'Call operator'
    bl_description ='Call the temporary operators specified in custom_dict'
    bl_options = {'REGISTER'}

    op_list = bpy.props.EnumProperty(items=operatorList)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, 'op_list', expand=True)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)
    
    def execute(self, context):
        operator_name = registerTempOperator(self.op_list)
        
        # invoke the other operator, based on its bl_idname
        operator = eval('bpy.ops.' + operator_name + "('INVOKE_DEFAULT')")
        return {'FINISHED'}


# just a basic panel to add the test operator to the 3D toolbar
class TestPanel(bpy.types.Panel):
    """Contains the operator to test."""
    bl_idname = "TOOLS_PT_TEST_TOOLS"                                                                                                                                                  
    bl_label = "TEST"                                                                                                                                                           
    bl_space_type = 'VIEW_3D'                                                                                                                                                            
    bl_region_type = 'TOOLS'                                                                                                                                                             
    bl_category = 'Tools'
    
    def draw(self, context):
        layout = self.layout
        layout.operator('object.calling_operator')


def register():
    bpy.utils.register_class(PropertyEntry)
    bpy.utils.register_class(CallingOperator)
    bpy.utils.register_class(TestPanel)

def unregister():
    bpy.utils.register_class(PropertyEntry)
    bpy.utils.register_class(CallingOperator)
    bpy.utils.unregister_class(TestPanel)

if __name__ == '__main__':
    register()