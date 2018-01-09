import bpy
from bpy.types import Node
import os
import yaml

import phobos.defs as defs


class ShaderNode:
    def get_clean_name(self):
        """
        Replaces spaces and dots in the name with underscores
        :return: The cleaned up name
        """
        return (self.name.replace(" ", "_")).replace(".", "_")

    def export(self):
        node = dict(outgoing={}, incoming={}, name=self.get_clean_name(), type="ShaderNode")
        for output_socket in self.outputs:
            node["outgoing"][output_socket.name] = self.get_clean_name() + "_" + output_socket.name
        for input_socket in self.inputs:
            if input_socket.is_linked:
                link = input_socket.links[0]
                node["incoming"][input_socket.name] = link.from_node.get_clean_name() + "_" + link.from_socket.name
            else:
                node["incoming"][input_socket.name] = input_socket.get_default_value()
        return node


class VertexNode(ShaderNode):
    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == "VertexShaderTree"


class FragmentNode(ShaderNode):
    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == "FragmentShaderTree"


class VertexFragmentNode(ShaderNode):
    @classmethod
    def poll(cls, ntree):
        return ntree.bl_idname == "VertexShaderTree" or ntree.bl_idname == "FragmentShaderTree"


class FragInfoNode(Node, FragmentNode):
    """
    A node for providing information for the fragment shader
    """
    bl_idname = "FragInfoNode"
    bl_label = "Fragment Info Node"
    bl_icon = "SOUND"

    def init(self, contex):
        self.outputs.new("SocketVector4", "ambient")
        self.outputs.new("SocketVector4", "diffuse")
        self.outputs.new("SocketVector4", "specular")
        self.outputs.new("SocketVector4", "emission")
        self.outputs.new("SocketVector2", "texCoord")

    def export(self):
        result = super().export()
        result["type"] = "fragInfo"
        return result


class FragOutNode(Node, FragmentNode):
    """
    A node for the fragment shader output
    """
    bl_idname = "FragOutNode"
    bl_label = "Fragment Output Node"
    bl_icon = "SOUND"

    def init(self, contex):
        self.inputs.new("SocketVector4", "color")

    def export(self):
        result = super().export()
        result["type"] = "fragOut"
        return result


class VertOutNode(Node, VertexNode):
    """
    A node for the vertex shader output
    """
    bl_idname = "VertOutNode"
    bl_label = "Vertex Output Node"
    bl_icon = "SOUND"

    def init(self, contex):
        self.inputs.new("SocketVector4", "viewPos")
        self.inputs.new("SocketVector4", "modelPos")
        self.inputs.new("SocketVector3", "normalVarying")

    def export(self):
        result = super().export()
        result["type"] = "vertexOut"
        return result


class VertInfoNode(Node, VertexNode):
    """
    A node for providing information for the vertex shader
    """
    bl_idname = "VertInfoNode"
    bl_label = "Vertex Info Node"
    bl_icon = "SOUND"

    def init(self, contex):
        self.outputs.new("SocketVector3", "normal")

    def export(self):
        result = super().export()
        result["type"] = "vertexInfo"
        return result


class BackfaceNormalNode(Node, VertexFragmentNode):
    """
    A node for backface normal operation
    """
    bl_idname = "BackfaceNormalNode"
    bl_label = "Backface Normal Node"
    bl_icon = "SOUND"

    def init(self, context):
        self.inputs.new("SocketVector3", "n")
        self.outputs.new("SocketVector3", "n")

    def export(self):
        result = super().export()
        result["type"] = "backfaceNormal"
        return result


class VectorMathNode(Node, VertexFragmentNode):
    """
    A node for a vector math operation of the form (vec_a+vec_b)*scalar
    """
    bl_idname = "VectorMathNode"
    bl_label = "Vector Math Node"
    bl_icon = "SOUND"

    vector_types = [("VEC2", "Vec2", "Vector2 value"),
                    ("VEC3", "Vec3", "Vector3 value"),
                    ("VEC4", "Vec4", "Vector4 value")
                    ]

    def update_type(self, context):
        for input_sock in self.inputs:
            self.inputs.remove(input_sock)
        for output_sock in self.outputs:
            self.outputs.remove(output_sock)
        if self.vector_type == "VEC2":
            self.inputs.new("SocketVector2", "vector_a")
            self.inputs.new("SocketVector2", "vector_b")
            self.inputs.new("SocketFloat", "scalar").default_value = 1.0
            self.outputs.new("SocketVector2", "vector")
        elif self.vector_type == "VEC3":
            self.inputs.new("SocketVector3", "vector_a")
            self.inputs.new("SocketVector3", "vector_b")
            self.inputs.new("SocketFloat", "scalar").default_value = 1.0
            self.outputs.new("SocketVector3", "vector")
        elif self.vector_type == "VEC4":
            self.inputs.new("SocketVector4", "vector_a")
            self.inputs.new("SocketVector4", "vector_b")
            self.inputs.new("SocketFloat", "scalar").default_value = 1.0
            self.outputs.new("SocketVector4", "vector")

    vector_type = bpy.props.EnumProperty(name="Type",
                                         description="Data type of the vector",
                                         items=vector_types,
                                         default="VEC2",
                                         update=update_type)

    def init(self, context):
        self.inputs.new("SocketVector2", "vector_a")
        self.inputs.new("SocketVector2", "vector_b")
        self.inputs.new("SocketFloat", "scalar").default_value = 1.0
        self.outputs.new("SocketVector2", "vector")

    def draw_buttons(self, context, layout):
        layout.prop(self, "vector_type")

    def export(self):
        result = super().export()
        if self.vector_type == "VEC2":
            result["type"] = "math_vec2"
        elif self.vector_type == "VEC3":
            result["type"] = "math_vec3"
        elif self.vector_type == "VEC4":
            result["type"] = "math_vec4"
        return result


class DecomposeVectorNode(Node, VertexFragmentNode):
    """
    A node for a vector decomposing operation
    """
    bl_idname = "DecomposeVectorNode"
    bl_label = "Decompose Vector Node"
    bl_icon = "SOUND"

    vector_types = [("VEC2", "Vec2", "Vector2 value"),
                    ("VEC3", "Vec3", "Vector3 value"),
                    ("VEC4", "Vec4", "Vector4 value")
                    ]

    def update_type(self, context):
        for input_sock in self.inputs:
            self.inputs.remove(input_sock)
        for output_sock in self.outputs:
            self.outputs.remove(output_sock)
        if self.vector_type == "VEC2":
            self.inputs.new("SocketVector2", "vector")
            self.outputs.new("SocketFloat", "x")
            self.outputs.new("SocketFloat", "y")
        elif self.vector_type == "VEC3":
            self.inputs.new("SocketVector3", "vector")
            self.outputs.new("SocketFloat", "x")
            self.outputs.new("SocketFloat", "y")
            self.outputs.new("SocketFloat", "z")
        elif self.vector_type == "VEC4":
            self.inputs.new("SocketVector4", "vector")
            self.outputs.new("SocketFloat", "x")
            self.outputs.new("SocketFloat", "y")
            self.outputs.new("SocketFloat", "z")
            self.outputs.new("SocketFloat", "w")

    vector_type = bpy.props.EnumProperty(name="Type",
                                         description="Data type of the vector",
                                         items=vector_types,
                                         default="VEC2",
                                         update=update_type)

    def init(self, context):
        self.inputs.new("SocketVector2", "vector")
        self.outputs.new("SocketFloat", "x")
        self.outputs.new("SocketFloat", "y")

    def draw_buttons(self, context, layout):
        layout.prop(self, "vector_type")

    def export(self):
        result = super().export()
        if self.vector_type == "VEC2":
            result["type"] = "decompose_vec2"
        elif self.vector_type == "VEC3":
            result["type"] = "decompose_vec3"
        elif self.vector_type == "VEC4":
            result["type"] = "decompose_vec4"
        return result


class ComposeVectorNode(Node, VertexFragmentNode):
    """
    A node for a vector composing operation
    """
    bl_idname = "ComposeVectorNode"
    bl_label = "Compose Vector Node"
    bl_icon = "SOUND"

    vector_types = [("VEC2", "Vec2", "Vector2 value"),
                    ("VEC3", "Vec3", "Vector3 value"),
                    ("VEC4", "Vec4", "Vector4 value")
                    ]

    def update_type(self, context):
        for input_sock in self.inputs:
            self.inputs.remove(input_sock)
        for output_sock in self.outputs:
            self.outputs.remove(output_sock)
        if self.vector_type == "VEC2":
            self.outputs.new("SocketVector2", "vector")
            self.inputs.new("SocketFloat", "x")
            self.inputs.new("SocketFloat", "y")
        elif self.vector_type == "VEC3":
            self.outputs.new("SocketVector3", "vector")
            self.inputs.new("SocketFloat", "x")
            self.inputs.new("SocketFloat", "y")
            self.inputs.new("SocketFloat", "z")
        elif self.vector_type == "VEC4":
            self.outputs.new("SocketVector4", "vector")
            self.inputs.new("SocketFloat", "x")
            self.inputs.new("SocketFloat", "y")
            self.inputs.new("SocketFloat", "z")
            self.inputs.new("SocketFloat", "w")

    vector_type = bpy.props.EnumProperty(name="Type",
                                         description="Data type of the vector",
                                         items=vector_types,
                                         default="VEC2",
                                         update=update_type)

    def init(self, context):
        self.outputs.new("SocketVector2", "vector")
        self.inputs.new("SocketFloat", "x")
        self.inputs.new("SocketFloat", "y")

    def draw_buttons(self, context, layout):
        layout.prop(self, "vector_type")

    def export(self):
        result = super().export()
        if self.vector_type == "VEC2":
            result["type"] = "compose_vec2"
        elif self.vector_type == "VEC3":
            result["type"] = "compose_vec3"
        elif self.vector_type == "VEC4":
            result["type"] = "compose_vec4"
        return result


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
            self.outputs.new("SocketInt", "output")
        elif self.uniform_type == "FLOAT":
            self.outputs.new("SocketFloat", "output")
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
        self.outputs.new("SocketFloat", "output")

    def draw_buttons(self, context, layout):
        layout.prop(self, "uniform_name")
        layout.prop(self, "uniform_type")

    def export(self):
        result = super().export()
        result["type"] = "uniform"
        result["uniform_name"] = self.uniform_name
        result["uniform_type"] = self.uniform_type.lower()
        return result


class VaryingVertexNode(Node, VertexNode):
    """
    A Node representing a Varying in the Vertex Shader
    """
    bl_idname = "VaryingVertexNode"
    bl_label = "Varying Node"
    bl_icon = "SOUND"

    def update_type(self, context):
        self.inputs.remove(self.inputs[0])
        if self.varying_type == "INT":
            self.inputs.new("SocketInt", "input")
        elif self.varying_type == "FLOAT":
            self.inputs.new("SocketFloat", "input")
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
        self.inputs.new("SocketFloat", "input")

    def draw_buttons(self, context, layout):
        layout.prop(self, "varying_name")
        layout.prop(self, "varying_type")

    def export(self):
        result = super().export()
        result["type"] = "varying_vertex"
        result["varying_name"] = self.varying_name
        result["varying_type"] = self.varying_type.lower()
        return result


class VaryingFragmentNode(Node, FragmentNode):
    """
    A Node representing a Varying in the Fragment Shader
    """
    bl_idname = "VaryingFragmentNode"
    bl_label = "Varying Node"
    bl_icon = "SOUND"

    def update_type(self, context):
        self.outputs.remove(self.outputs[0])
        if self.varying_type == "INT":
            self.outputs.new("SocketInt", "input")
        elif self.varying_type == "FLOAT":
            self.outputs.new("SocketFloat", "input")
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
        self.outputs.new("SocketFloat", "input")

    def draw_buttons(self, context, layout):
        layout.prop(self, "varying_name")
        layout.prop(self, "varying_type")

    def export(self):
        result = super().export()
        result["type"] = "varying_fragment"
        result["varying_name"] = self.varying_name
        result["varying_type"] = self.varying_type.lower()
        return result


class CustomNode(Node, VertexFragmentNode):
    """
    A custom node for vertex shader
    """
    bl_idname = "CustomNode"
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
                self.inputs.new("SocketFloat", input_sock["name"])
            elif input_sock["type"] == "FLOAT":
                self.inputs.new("SocketFloat", input_sock["name"])
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
                self.outputs.new("SocketFloat", output_sock["name"])
            elif output_sock["type"] == "FLOAT":
                self.outputs.new("SocketFloat", output_sock["name"])
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

    def export(self):
        result = super().export()
        result["type"] = self.node_type
        return result


def load_node_def(definition):
    print("Loading custom node definition for: ", definition["name"])
    if "type" in definition and definition["type"] == "BOTH":
        CustomNode.node_types.append((definition["name"], definition["name"], definition["name"]))
        CustomNode.input_sets[definition["name"]] = []
        CustomNode.output_sets[definition["name"]] = []
        if "in" in definition["params"]:
            for name in definition["params"]["in"]:
                CustomNode.input_sets[definition["name"]].append(
                    dict(name=name, type=definition["params"]["in"][name]["type"].upper()))
        if "out" in definition["params"]:
            for name in definition["params"]["out"]:
                CustomNode.output_sets[definition["name"]].append(
                    dict(name=name, type=definition["params"]["out"][name]["type"].upper()))


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
    bpy.utils.register_class(BackfaceNormalNode)
    bpy.utils.register_class(ComposeVectorNode)
    bpy.utils.register_class(DecomposeVectorNode)
    bpy.utils.register_class(FragInfoNode)
    bpy.utils.register_class(VertInfoNode)
    bpy.utils.register_class(VectorMathNode)
    bpy.utils.register_class(FragOutNode)
    bpy.utils.register_class(VertOutNode)


def unregister():
    print("Unregistering shader node types")
    bpy.utils.unregister_class(UniformNode)
    bpy.utils.unregister_class(VaryingVertexNode)
    bpy.utils.unregister_class(VaryingFragmentNode)
    bpy.utils.unregister_class(CustomNode)
    bpy.utils.unregister_class(BackfaceNormalNode)
    bpy.utils.unregister_class(ComposeVectorNode)
    bpy.utils.unregister_class(DecomposeVectorNode)
    bpy.utils.unregister_class(FragInfoNode)
    bpy.utils.unregister_class(VertInfoNode)
    bpy.utils.unregister_class(VectorMathNode)
    bpy.utils.unregister_class(FragOutNode)
    bpy.utils.unregister_class(VertOutNode)
