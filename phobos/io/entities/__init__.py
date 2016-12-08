import os
import imp

entity_types = dict()
for filename in os.listdir(os.path.dirname(__file__)):
    mod_name, file_ext = os.path.splitext(os.path.split(filename)[-1])
    if (filename != os.path.split(__file__)[-1]) and (file_ext.lower() == '.py'):
        py_mod = imp.load_source(mod_name, os.path.join(os.path.dirname(__file__), filename))
        try:
            entity_types.update(py_mod.entity_type_dict.copy())
        except AttributeError:
            print('ERROR in entities/__init__: "'+filename+'" has no valid entity plugin interface.')
