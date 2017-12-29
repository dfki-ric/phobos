import bpy
from bpy.types import NodeSocket


class SocketSampler2D(NodeSocket):
    """
    Socket class for Sampler2D types
    """
    bl_idname = "SocketSampler2D"
    bl_label = "Sampler 2D"

    def draw(self, context, layout, node, text):
        layout.label(text)

    # Socket color
    def draw_color(self, context, node):
        return 1.0, 1.0, 1.0, 1.0


class SocketVector2(NodeSocket):
    """
    Socket class for Vector2 types
    """
    bl_idname = "SocketVector2"
    bl_label = "Vector2"

    values = bpy.props.FloatVectorProperty(name="Vector2",
                                           description="The vec2",
                                           size=2)

    def draw(self, context, layout, node, text):
        if self.is_output or self.is_linked:
            layout.label(text)
        else:
            layout.label(text)
            layout.prop(self, "values")

    # Socket color
    def draw_color(self, context, node):
        return 0.0, 0.0, 1.0, 0.5


class SocketVector3(NodeSocket):
    """
    Socket class for Vector3 types
    """
    bl_idname = "SocketVector3"
    bl_label = "Vector3"

    values = bpy.props.FloatVectorProperty(name="Vector3",
                                           description="The vec3",
                                           size=3)

    def draw(self, context, layout, node, text):
        if self.is_output or self.is_linked:
            layout.label(text)
        else:
            layout.label(text)
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
    bl_idname = "SocketVector4"
    bl_label = "Vector4"

    values = bpy.props.FloatVectorProperty(name="Vector4",
                                           description="The vec4",
                                           size=4)

    def draw(self, context, layout, node, text):
        if self.is_output or self.is_linked:
            layout.label(text)
        else:
            layout.label(text)
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
    bl_idname = "SocketMat4"
    bl_label = "Matrix4"

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
            layout.label(text)
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


def register():
    print("Registering shader socket types")
    bpy.utils.register_class(SocketVector2)
    bpy.utils.register_class(SocketVector3)
    bpy.utils.register_class(SocketVector4)
    bpy.utils.register_class(SocketMat4)
    bpy.utils.register_class(SocketSampler2D)


def unregister():
    print("Unregistering shader socket types")
    bpy.utils.unregister_class(SocketVector2)
    bpy.utils.unregister_class(SocketVector3)
    bpy.utils.unregister_class(SocketVector4)
    bpy.utils.unregister_class(SocketMat4)
    bpy.utils.unregister_class(SocketSampler2D)
