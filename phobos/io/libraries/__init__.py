import os
import imp

def register():
    for filename in os.listdir(os.path.dirname(__file__)):
        mod_name, file_ext = os.path.splitext(os.path.split(filename)[-1])
        if (filename != os.path.split(__file__)[-1]) and (file_ext.lower() == '.py'):
            py_mod = imp.load_source(mod_name, os.path.join(os.path.dirname(__file__), filename))
            #try:
            py_mod.register()
            print('Registering phobos lib plugin:', mod_name)
            #except AttributeError:
            #    print('ERROR in libraries/__init__: "'+mod_name+'" has no register function.')
