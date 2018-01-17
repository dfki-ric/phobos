import os
import yaml

def export_shader(model, outpath):
    for shadername in model["shaders"]:
        shader = model["shaders"][shadername]
        shader_path = os.path.join(outpath, shader["name"] + ".yaml")
        with open(shader_path, 'w') as shaderfile:
            shaderfile.write(yaml.dump(shader, default_flow_style=False))


def import_shader(filepath):
    pass


# registering export functions of types with Phobos
entity_type_dict = {'shader': {'export': export_shader,
                             'import': import_shader,
                             'extensions': ('ps',)}
                    }