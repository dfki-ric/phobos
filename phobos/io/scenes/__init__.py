import os
import imp

scene_types = dict()
structure_export_folders = []
for filename in os.listdir(os.path.dirname(__file__)):
    mod_name, file_ext = os.path.splitext(os.path.split(filename)[-1])
    if (filename != os.path.split(__file__)[-1]) and (file_ext.lower() == '.py'):
        py_mod = imp.load_source(mod_name, os.path.join(os.path.dirname(__file__), filename))

        if hasattr(py_mod, 'scene_type_dict'):
            # scene_types = {**scene_types, **py_mod.scene_type_dict}  # unpacking (> Python 3.5)
            scene_types.update(py_mod.scene_type_dict.copy())
