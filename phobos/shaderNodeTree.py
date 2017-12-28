import bpy
from bpy.types import NodeTree, Node, NodeSocket
import mathutils
from mathutils import Matrix
import nodeitems_utils
from nodeitems_utils import NodeCategory, NodeItem

shader_data_types = [
    ("INT", "Integer", "Integer value"),
    ("FLOAT", "Float", "Float value"),
    ("VEC2", "Vec2", "Vector2 value"),
    ("VEC3", "Vec3", "Vector3 value"),
    ("VEC4", "Vec4", "Vector4 value"),
    ("MAT4", "Mat4", "Matrix4 value"),
    ("SAMPLER2D", "Sampler2D", "Sampler2D value"),
]


class VertexShaderTree(NodeTree):
    """
    The Node tree for vertex shader
    """
    bl_idname = 'VertexShaderTree'
    bl_label = 'Vertex Shader'
    bl_icon = 'NODETREE'


class FragmentShaderTree(NodeTree):
    """
    The Node tree for fragment shader
    """
    bl_idname = 'FragmentShaderTree'
    bl_label = 'Fragment Shader'
    bl_icon = 'NODETREE'


class SocketSampler2D(NodeSocket):
    """
    Socket class for Sampler2D types
    """
    bl_idname = 'SocketSampler2D'
    bl_label = 'Sampler 2D'

    def draw(self, context, layout, node, text):
        layout.label(text)

    # Socket color
    def draw_color(self, context, node):
        return 1.0, 1.0, 1.0, 1.0


class SocketVector2(NodeSocket):
    """
    Socket class for Vector2 types
    """
    bl_idname = 'SocketVector2'
    bl_label = 'Vector2'

    values = bpy.props.FloatVectorProperty(name="Vector2",
                                           description="The vec2",
                                           size=2)

    def draw(self, context, layout, node, text):
        if self.is_output or self.is_linked:
            layout.label(text)
        else:
            layout.prop(self, "values")

    # Socket color
    def draw_color(self, context, node):
        return 0.0, 0.0, 1.0, 0.5


class SocketVector3(NodeSocket):
    """
    Socket class for Vector3 types
    """
    bl_idname = 'SocketVector3'
    bl_label = 'Vector3'

    values = bpy.props.FloatVectorProperty(name="Vector3",
                                           description="The vec3",
                                           size=3)

    def draw(self, context, layout, node, text):
        if self.is_output or self.is_linked:
            layout.label(text)
        else:
            split = layout.split()
            column = split.column()
            column.prop(self, "values")

    # Socket color
    def draw_color(self, context, node):
        return 0.0, 0.0, 1.0, 0.5


class SocketVector4(NodeSocket):
    """
    Socket class for Vector4 types
    """
    bl_idname = 'SocketVector4'
    bl_label = 'Vector4'

    values = bpy.props.FloatVectorProperty(name="Vector4",
                                           description="The vec4",
                                           size=4)

    def draw(self, context, layout, node, text):
        if self.is_output or self.is_linked:
            layout.label(text)
        else:
            split = layout.split()
            column = split.column()
            column.prop(self, "values")

    # Socket color
    def draw_color(self, context, node):
        return 0.0, 0.0, 1.0, 0.5


class SocketMat4(NodeSocket):
    """
        Socket class for Vector4 types
        """
    bl_idname = 'SocketMat4'
    bl_label = 'Matrix4'

    col_1 = bpy.props.FloatVectorProperty(name="Matrix4_col1",
                                          description="The mat4",
                                          size=4)

    col_2 = bpy.props.FloatVectorProperty(name="Matrix4_col2",
                                          description="The mat4",
                                          size=4)

    col_3 = bpy.props.FloatVectorProperty(name="Matrix4_col3",
                                          description="The mat4",
                                          size=4)

    col_4 = bpy.props.FloatVectorProperty(name="Matrix4_col4",
                                          description="The mat4",
                                          size=4)

    def draw(self, context, layout, node, text):
        if self.is_output or self.is_linked:
            layout.label(text)
        else:
            split = layout.split()
            column1 = split.column()
            column2 = split.column()
            column1.prop(self, "col_1")
            column1.prop(self, "col_2")
            column2.prop(self, "col_3")
            column2.prop(self, "col_4")

    # Socket color
    def draw_color(self, context, node):
        return 0.0, 0.5, 0.5, 0.5


class VertexNode:
    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == 'VertexShaderTree'


class FragmentNode:
    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == 'FragmentShaderTree'


class VertexFragmentNode:
    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == 'VertexShaderTree' or ntree.bl_idname == 'FragmentShaderTree'


class UniformNode(Node, VertexFragmentNode):
    """
    A Node representing a Uniform
    """
    bl_idname = 'UniformNodeType'
    bl_label = 'Uniform Node'
    bl_icon = 'SOUND'

    def update_type(self, context):
        self.outputs.remove(self.outputs[0])
        if self.uniform_type == "INT":
            self.outputs.new("NodeSocketInt", "output")
        elif self.uniform_type == "FLOAT":
            self.outputs.new("NodeSocketFloat", "output")
        elif self.uniform_type == "VEC2":
            self.outputs.new("SocketVector2", "output")
        elif self.uniform_type == "VEC3":
            self.outputs.new("SocketVector3", "output")
        elif self.uniform_type == "VEC4":
            self.outputs.new("SocketVector4", "output")
        elif self.uniform_type == "MAT4":
            self.outputs.new("SocketMat4", "output")
        elif self.uniform_type == "SAMPLER2D":
            self.outputs.new("SocketSampler2D", "output")

    uniform_type = bpy.props.EnumProperty(name="Type",
                                          description="Data type of the uniform",
                                          items=shader_data_types,
                                          default="INT",
                                          update=update_type)

    uniform_name = bpy.props.StringProperty(name="Name", description="Name of the Uniform", default="uniform")

    def init(self, context):
        self.outputs.new("NodeSocketInt", "output")

    def draw_buttons(self, context, layout):
        layout.prop(self, "uniform_name")
        layout.prop(self, "uniform_type")


class VaryingVertexNode(Node, VertexNode):
    """
    A Node representing a Varying in the Vertex Shader
    """
    bl_idname = 'VaryingVertexNodeType'
    bl_label = 'Varying Node'
    bl_icon = 'SOUND'

    def update_type(self, context):
        self.inputs.remove(self.inputs[0])
        if self.varying_type == "INT":
            self.inputs.new("NodeSocketInt", "input")
        elif self.varying_type == "FLOAT":
            self.inputs.new("NodeSocketFloat", "input")
        elif self.varying_type == "VEC2":
            self.inputs.new("SocketVector2", "input")
        elif self.varying_type == "VEC3":
            self.inputs.new("SocketVector3", "input")
        elif self.varying_type == "VEC4":
            self.inputs.new("SocketVector4", "input")
        elif self.varying_type == "MAT4":
            self.inputs.new("SocketMat4", "input")
        elif self.varying_type == "SAMPLER2D":
            self.inputs.new("SocketSampler2D", "input")

    varying_type = bpy.props.EnumProperty(name="Type",
                                          description="Data type of the varying",
                                          items=shader_data_types,
                                          default="INT",
                                          update=update_type)

    varying_name = bpy.props.StringProperty(name="Name", description="Name of the varying", default="varying")

    def init(self, context):
        self.inputs.new("NodeSocketInt", "input")

    def draw_buttons(self, context, layout):
        layout.prop(self, "varying_name")
        layout.prop(self, "varying_type")


class VaryingFragmentNode(Node, FragmentNode):
    """
    A Node representing a Varying in the Fragment Shader
    """
    bl_idname = 'VaryingFragmentNodeType'
    bl_label = 'Varying Node'
    bl_icon = 'SOUND'

    def update_type(self, context):
        self.outputs.remove(self.outputs[0])
        if self.varying_type == "INT":
            self.outputs.new("NodeSocketInt", "input")
        elif self.varying_type == "FLOAT":
            self.outputs.new("NodeSocketFloat", "input")
        elif self.varying_type == "VEC2":
            self.outputs.new("SocketVector2", "input")
        elif self.varying_type == "VEC3":
            self.outputs.new("SocketVector3", "input")
        elif self.varying_type == "VEC4":
            self.outputs.new("SocketVector4", "input")
        elif self.varying_type == "MAT4":
            self.outputs.new("SocketMat4", "input")
        elif self.varying_type == "SAMPLER2D":
            self.outputs.new("SocketSampler2D", "input")

    varying_type = bpy.props.EnumProperty(name="Type",
                                          description="Data type of the varying",
                                          items=shader_data_types,
                                          default="INT",
                                          update=update_type)

    varying_name = bpy.props.StringProperty(name="Name", description="Name of the varying", default="varying")

    def init(self, context):
        self.outputs.new("NodeSocketInt", "input")

    def draw_buttons(self, context, layout):
        layout.prop(self, "varying_name")
        layout.prop(self, "varying_type")


class VertexNodeCategory(NodeCategory):
    @classmethod
    def poll(clscls, context):
        return context.space_data.tree_type == 'VertexShaderTree'


class FragmentNodeCategory(NodeCategory):
    @classmethod
    def poll(clscls, context):
        return context.space_data.tree_type == 'FragmentShaderTree'


node_categories = [
    VertexNodeCategory("INPUT", "Input", items=[
        NodeItem("UniformNodeType")
    ]),
    FragmentNodeCategory("INPUT", "Input", items=[
        NodeItem("UniformNodeType"),
        NodeItem("VaryingFragmentNodeType")
    ]),
    VertexNodeCategory("OUTPUT", "Output", items=[
        NodeItem("VaryingVertexNodeType")
    ])
]


def draw_func_shader_graphs(self, context):
    layout = self.layout
    ob = context.object
    layout.prop(ob.active_material, "export_shaders")
    layout.prop(ob.active_material, "vertex_shader")
    layout.prop(ob.active_material, "fragment_shader")


def register():
    print("Registering Shader Node Tree")
    bpy.utils.register_class(VertexShaderTree)
    bpy.utils.register_class(FragmentShaderTree)
    bpy.utils.register_class(SocketVector2)
    bpy.utils.register_class(SocketVector3)
    bpy.utils.register_class(SocketVector4)
    bpy.utils.register_class(SocketMat4)
    bpy.utils.register_class(SocketSampler2D)
    bpy.utils.register_class(UniformNode)
    bpy.utils.register_class(VaryingVertexNode)
    bpy.utils.register_class(VaryingFragmentNode)

    nodeitems_utils.register_node_categories("CUSTOM_NODES", node_categories)

    bpy.types.Material.vertex_shader = bpy.props.PointerProperty(type=NodeTree, name="Vertex Shader")
    bpy.types.Material.fragment_shader = bpy.props.PointerProperty(type=NodeTree, name="Fragment Shader")
    bpy.types.Material.export_shaders = bpy.props.BoolProperty(name="Export Shaders",
                                                               description="Toogle shader export for material")
    bpy.types.MATERIAL_PT_context_material.append(draw_func_shader_graphs)


def unregister():
    print("Unregistering Shader Node Tree")
    bpy.utils.unregister_class(VertexShaderTree)
    bpy.utils.unregister_class(FragmentShaderTree)
    bpy.utils.unregister_class(SocketVector2)
    bpy.utils.unregister_class(SocketVector3)
    bpy.utils.unregister_class(SocketVector4)
    bpy.utils.unregister_class(SocketMat4)
    bpy.utils.unregister_class(SocketSampler2D)
    bpy.utils.unregister_class(UniformNode)
    bpy.utils.unregister_class(VaryingVertexNode)
    bpy.utils.unregister_class(VaryingFragmentNode)

    nodeitems_utils.unregister_node_categories("CUSTOM_NODES")

    bpy.types.MATERIAL_PT_context_material.remove(draw_func_shader_graphs)
    del bpy.types.Material.vertex_shader
