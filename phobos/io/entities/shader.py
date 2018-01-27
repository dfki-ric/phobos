import os
import yaml
import shutil
import phobos.utils.io as ioUtils

def export_shader(model, outpath):
    for shadername in model["shaders"]:
        shader = model["shaders"][shadername]
        shader_path = os.path.join(outpath, shader["name"] + ".yaml")
        with open(shader_path, 'w') as shaderfile:
            shaderfile.write(yaml.dump(shader, default_flow_style=False))
        # Add custom node files to export
        node_path = os.path.join(os.path.dirname(__file__), "../..", "shader_nodes")
        export_path = os.path.join(outpath, "custom")
        ioUtils.securepath(export_path)
        for custom_node in shader["custom"]:
            shutil.copy2(os.path.join(node_path, custom_node + ".yaml"), export_path)
            shutil.copy2(os.path.join(node_path, custom_node + ".frag"), export_path)


def import_shader(filepath):
    pass


# registering export functions of types with Phobos
entity_type_dict = {'shader': {'export': export_shader,
                             'import': import_shader,
                             'extensions': ('yml', 'yaml')}
                    }