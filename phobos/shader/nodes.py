import bpy
from bpy.types import Node
import os
import yaml

import phobos.defs as defs


class VertexNode:
    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == "VertexShaderTree"


class FragmentNode:
    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == "FragmentShaderTree"


class VertexFragmentNode:
    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == "VertexShaderTree" or ntree.bl_idname == "FragmentShaderTree"


class UniformNode(Node, VertexFragmentNode):
    """
    A Node representing a Uniform
    """
    bl_idname = "UniformNode"
    bl_label = "Uniform Node"
    bl_icon = "SOUND"

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
                                          items=defs.shader_data_types,
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
    bl_idname = "VaryingVertexNodeType"
    bl_label = "Varying Node"
    bl_icon = "SOUND"

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
                                          items=defs.shader_data_types,
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
    bl_idname = "VaryingFragmentNodeType"
    bl_label = "Varying Node"
    bl_icon = "SOUND"

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
                                          items=defs.shader_data_types,
                                          default="INT",
                                          update=update_type)

    varying_name = bpy.props.StringProperty(name="Name", description="Name of the varying", default="varying")

    def init(self, context):
        self.outputs.new("NodeSocketInt", "input")

    def draw_buttons(self, context, layout):
        layout.prop(self, "varying_name")
        layout.prop(self, "varying_type")


class CustomNode(Node, VertexFragmentNode):
    """
    A custom node for vertex shader
    """
    bl_idname = "CustomNodeType"
    bl_label = "Custom Node"
    bl_icon = "SOUND"

    node_types = []
    input_sets = {}
    output_sets = {}

    def update_type(self, context):
        for input_sock in self.inputs:
            self.inputs.remove(input_sock)
        for output_sock in self.outputs:
            self.outputs.remove(output_sock)
        for input_sock in CustomNode.input_sets[self.node_type]:
            if input_sock["type"] == "INT":
                self.inputs.new("NodeSocketInt", input_sock["name"])
            elif input_sock["type"] == "FLOAT":
                self.inputs.new("NodeSocketFloat", input_sock["name"])
            elif input_sock["type"] == "VEC2":
                self.inputs.new("SocketVector2", input_sock["name"])
            elif input_sock["type"] == "VEC3":
                self.inputs.new("SocketVector3", input_sock["name"])
            elif input_sock["type"] == "VEC4":
                self.inputs.new("SocketVector4", input_sock["name"])
            elif input_sock["type"] == "MAT4":
                self.inputs.new("SocketMat4", input_sock["name"])
            elif input_sock["type"] == "SAMPLER2D":
                self.inputs.new("SocketSampler2D", input_sock["name"])
        for output_sock in CustomNode.output_sets[self.node_type]:
            if output_sock["type"] == "INT":
                self.outputs.new("NodeSocketInt", output_sock["name"])
            elif output_sock["type"] == "FLOAT":
                self.outputs.new("NodeSocketFloat", output_sock["name"])
            elif output_sock["type"] == "VEC2":
                self.outputs.new("SocketVector2", output_sock["name"])
            elif output_sock["type"] == "VEC3":
                self.outputs.new("SocketVector3", output_sock["name"])
            elif output_sock["type"] == "VEC4":
                self.outputs.new("SocketVector4", output_sock["name"])
            elif output_sock["type"] == "MAT4":
                self.outputs.new("SocketMat4", output_sock["name"])
            elif output_sock["type"] == "SAMPLER2D":
                self.outputs.new("SocketSampler2D", output_sock["name"])

    node_type = bpy.props.EnumProperty(name="Type",
                                       description="Function type of the node",
                                       items=node_types,
                                       update=update_type)

    def init(self, context):
        pass

    def draw_buttons(self, context, layout):
        layout.prop(self, "node_type")


def load_node_def(definition):
    print("Loading custom node definition for: ", definition["name"])
    if definition["type"] == "BOTH":
        CustomNode.node_types.append((definition["name"], definition["name"], definition["name"]))
        if "inputs" in definition:
            CustomNode.input_sets[definition["name"]] = definition["inputs"]
        else:
            CustomNode.input_sets[definition["name"]] = []
        if "outputs" in definition:
            CustomNode.output_sets[definition["name"]] = definition["outputs"]
        else:
            CustomNode.output_sets[definition["name"]] = []


def load_node_defs():
    def_path = os.path.join(os.path.dirname(__file__), "..", "shader_nodes")
    for root, dirs, files in os.walk(def_path):
        for node_def in files:
            if node_def.endswith(".yml"):
                with open(os.path.join(def_path, node_def)) as f:
                    node_def = yaml.load(f.read())
                    load_node_def(node_def)


def register():
    print("Registering shader node types")
    load_node_defs()
    bpy.utils.register_class(UniformNode)
    bpy.utils.register_class(VaryingVertexNode)
    bpy.utils.register_class(VaryingFragmentNode)
    bpy.utils.register_class(CustomNode)


def unregister():
    print("Unregistering shader node types")
    bpy.utils.unregister_class(UniformNode)
    bpy.utils.unregister_class(VaryingVertexNode)
    bpy.utils.unregister_class(VaryingFragmentNode)
    bpy.utils.unregister_class(CustomNode)
