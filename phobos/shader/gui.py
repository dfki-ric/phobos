import nodeitems_utils
from nodeitems_utils import NodeCategory, NodeItem
import bpy
from bpy.types import NodeTree
from bpy.types import Operator
import phobos.shader.export as export
from phobos.phoboslog import log


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
    VertexNodeCategory("VERTEX_INPUT", "Input", items=[
        NodeItem("UniformNode"),
        NodeItem("VertInfoNode")
    ]),
    FragmentNodeCategory("FRAGMENT_INPUT", "Input", items=[
        NodeItem("UniformNode"),
        NodeItem("VaryingFragmentNode"),
        NodeItem("FragInfoNode")
    ]),
    VertexNodeCategory("VERTEX_OUTPUT", "Output", items=[
        NodeItem("VaryingVertexNode"),
        NodeItem("VertOutNode")
    ]),
    FragmentNodeCategory("FRAGMENT_OUTPUT", "Output", items=[
        NodeItem("FragOutNode")
    ]),
    VertexFragmentNodeCategory("SHARED_NODES", "Shared Nodes", items=[
        NodeItem("CustomNode"),
        NodeItem("BackfaceNormalNode"),
        NodeItem("ComposeVectorNode"),
        NodeItem("DecomposeVectorNode"),
        NodeItem("VectorMathNode")
    ])
]


class ValidateGraph(Operator):
    """Validates the active graph"""
    bl_idname = "phobos.validate_graph"
    bl_label = "Validate graph"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        active_tree = context.space_data.edit_tree
        errors = export.validate_shader(active_tree)
        for error in errors:
            log("Type mismatch between " + error["node"] + "-" + error["socket"] + " and " + error["from_node"] + "-" +
                error["from_socket"], "WARNING", "ValidateGraph")
        if len(errors) == 0:
            log("No type mismatches found!", "INFO", "ValidateGraph")
        return {'FINISHED'}


class FixGraph(Operator):
    """Validates and tries to fix the active graph"""
    bl_idname = "phobos.fix_graph"
    bl_label = "Fix graph"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        active_tree = context.space_data.edit_tree
        errors = export.validate_shader(active_tree)
        remains = export.solve_errors(active_tree, errors)
        if len(remains) > 0:
            log("The following errors could not be solved:")
            for error in remains:
                log("Type mismatch between " + error["node"] + "-" + error["socket"] + " and " + error[
                    "from_node"] + "-" +
                    error["from_socket"], "WARNING", "FixGraph")
        return {'FINISHED'}


class PhobosNodePanel(bpy.types.Panel):
    bl_idname = "NODE_EDITOR_PT_PHOBOS_TOOLS"
    bl_label = "Phobos Node Editor Tools"
    bl_space_type = "NODE_EDITOR"
    bl_region_type = "TOOLS"
    bl_category = "Phobos Tools"

    @classmethod
    def poll(cls, context):
        return context.space_data.tree_type == 'FragmentShaderTree' \
               or context.space_data.tree_type == 'VertexShaderTree'

    def draw(self, context):
        self.layout.operator('phobos.validate_graph', text='Validate Graph')
        self.layout.operator('phobos.fix_graph', text='Fix Graph')


def draw_func_shader_graphs(self, context):
    layout = self.layout
    ob = context.object
    layout.prop(ob.active_material, "export_shaders")
    layout.prop(ob.active_material, "print_shaders")
    layout.prop(ob.active_material, "vertex_shader")
    layout.prop(ob.active_material, "fragment_shader")


def register():
    print("Registering shader gui components")
    nodeitems_utils.register_node_categories("CUSTOM_NODES", node_categories)

    bpy.types.Material.vertex_shader = bpy.props.PointerProperty(type=NodeTree, name="Vertex Shader")
    bpy.types.Material.fragment_shader = bpy.props.PointerProperty(type=NodeTree, name="Fragment Shader")
    bpy.types.Material.export_shaders = bpy.props.BoolProperty(name="Export Shaders",
                                                               description="Toogle shader export for material")
    bpy.types.Material.print_shaders = bpy.props.BoolProperty(name="Print Shader",
                                                              description="Toogle shader debug print for material")
    bpy.types.MATERIAL_PT_context_material.append(draw_func_shader_graphs)
    bpy.utils.register_class(PhobosNodePanel)


def unregister():
    print("Unregistering shader node components")

    bpy.utils.unregister_class(PhobosNodePanel)

    nodeitems_utils.unregister_node_categories("CUSTOM_NODES")

    bpy.types.MATERIAL_PT_context_material.remove(draw_func_shader_graphs)
    del bpy.types.Material.vertex_shader
    del bpy.types.Material.fragment_shader
    del bpy.types.Material.export_shaders
