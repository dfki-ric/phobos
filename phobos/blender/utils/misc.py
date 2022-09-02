from phobos.blender.model.models import deriveModelDictionary


def print_model_dict(name='', objectlist=[], indent=0):
    """
    Prints the internal blender dictionary to the command line
    """
    import bpy
    import phobos.blender.utils.selection as sUtils
    root = sUtils.getRoot(bpy.context.selected_objects[0])
    blender_model = deriveModelDictionary(root, name, objectlist)
    print_dict(blender_model, indent)


def print_dict(dictionary, indent=0):
    for k, v in dictionary.items():
        print(" " * indent, k + ": ", end="")
        if type(v) == object:
            print("Type: " + type(v))
        elif type(v) == dict:
            print()
            print_dict(v, indent=indent + 2)
        else:
            print(v)
