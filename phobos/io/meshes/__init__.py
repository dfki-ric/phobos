import os
import imp

mesh_types = dict()
for filename in os.listdir(os.path.dirname(__file__)):
    mod_name, file_ext = os.path.splitext(os.path.split(filename)[-1])
    if (filename != os.path.split(__file__)[-1]) and (file_ext.lower() == '.py'):
        py_mod = imp.load_source(mod_name, os.path.join(os.path.dirname(__file__), filename))

        if hasattr(py_mod, 'mesh_type_dict'):
            # mesh_types = {**mesh_types, **py_mod.mesh_type_dict}  # unpacking (> Python 3.5)
            mesh_types.update(py_mod.mesh_type_dict.copy())
