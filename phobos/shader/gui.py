import nodeitems_utils
from nodeitems_utils import NodeCategory, NodeItem
import bpy
from bpy.types import NodeTree


class VertexNodeCategory(NodeCategory):
    @classmethod
    def poll(clscls, context):
        return context.space_data.tree_type == 'VertexShaderTree'


class FragmentNodeCategory(NodeCategory):
    @classmethod
    def poll(clscls, context):
        return context.space_data.tree_type == 'FragmentShaderTree'


class VertexFragmentNodeCategory(NodeCategory):
    @classmethod
    def poll(clscls, context):
        return context.space_data.tree_type == "FragmentShaderTree" \
               or context.space_data.tree_type == "VertexShaderTree"


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
    ]),
    VertexFragmentNodeCategory("SHARED_NODES", "Shared Nodes", items=[
        NodeItem("CustomNodeType")
    ])
]


def draw_func_shader_graphs(self, context):
    layout = self.layout
    ob = context.object
    layout.prop(ob.active_material, "export_shaders")
    layout.prop(ob.active_material, "vertex_shader")
    layout.prop(ob.active_material, "fragment_shader")


def register():
    print("Registering shader gui components")
    nodeitems_utils.register_node_categories("CUSTOM_NODES", node_categories)

    bpy.types.Material.vertex_shader = bpy.props.PointerProperty(type=NodeTree, name="Vertex Shader")
    bpy.types.Material.fragment_shader = bpy.props.PointerProperty(type=NodeTree, name="Fragment Shader")
    bpy.types.Material.export_shaders = bpy.props.BoolProperty(name="Export Shaders",
                                                               description="Toogle shader export for material")
    bpy.types.MATERIAL_PT_context_material.append(draw_func_shader_graphs)


def unregister():
    print("Unregistering shader node components")

    nodeitems_utils.unregister_node_categories("CUSTOM_NODES")

    bpy.types.MATERIAL_PT_context_material.remove(draw_func_shader_graphs)
    del bpy.types.Material.vertex_shader
