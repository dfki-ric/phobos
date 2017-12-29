import bpy
from bpy.types import NodeTree


class VertexShaderTree(NodeTree):
    """
    The Node tree for vertex shader
    """
    bl_idname = "VertexShaderTree"
    bl_label = "Vertex Shader"
    bl_icon = "NODETREE"


class FragmentShaderTree(NodeTree):
    """
    The Node tree for fragment shader
    """
    bl_idname = "FragmentShaderTree"
    bl_label = "Fragment Shader"
    bl_icon = "NODETREE"


def register():
    print("Registering shader tree types")
    bpy.utils.register_class(VertexShaderTree)
    bpy.utils.register_class(FragmentShaderTree)


def unregister():
    print("Unregistering shader tree types")
    bpy.utils.unregister_class(VertexShaderTree)
    bpy.utils.unregister_class(FragmentShaderTree)