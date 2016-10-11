import os
import imp

entity_types = dict()
structure_export_folders = []
for filename in os.listdir(os.path.dirname(__file__)):
    mod_name,file_ext = os.path.splitext(os.path.split(filename)[-1])
    if (filename != os.path.split(__file__)[-1]) and (file_ext.lower() == '.py'):
        py_mod = imp.load_source(mod_name, os.path.join(os.path.dirname(__file__),filename))

        if hasattr(py_mod, 'entity_type_name'):
            entity_types[py_mod.entity_type_name] = py_mod
        if hasattr(py_mod, 'structure_subfolder'):
            structure_export_folders.append(py_mod.structure_subfolder)