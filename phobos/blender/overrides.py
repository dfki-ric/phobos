import bpy

from phobos.blender import reserved_keys

blender_default_properties_edit = bpy.ops.wm.properties_edit


def _new_properties_edit(data_path='', property_name='', property_type='FLOAT', is_overridable_library=False,
                        description='', use_soft_limits=False, array_length=3,
                        default_int=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0),
                        min_int=- 10000, max_int=10000, soft_min_int=- 10000, soft_max_int=10000, step_int=1,
                        default_float=(
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                        min_float=- 10000.0,
                        max_float=- 10000.0, soft_min_float=- 10000.0, soft_max_float=- 10000.0, precision=3,
                        step_float=0.1, subtype='NONE', default_string='', eval_string=''):
    ctxt_obj = bpy.context.active_object
    if data_path == 'object' and ctxt_obj is not None and hasattr(ctxt_obj, "phobostype"):
        prohibited_keys = reserved_keys.INTERNAL_KEYS
        if hasattr(reserved_keys, ctxt_obj.phobostype.upper()+"_KEYS"):
            prohibited_keys += getattr(reserved_keys, ctxt_obj.phobostype.upper()+"_KEYS")
        if property_name in prohibited_keys:
            raise ValueError(f"{property_name} is reserved for internal usage. You can't edit reserved properties "
                             f"via 'Custom Properties', please use the 'Phobos Property Information' panel")

    blender_default_properties_edit(
        data_path=data_path, property_name=property_name, property_type=property_type,
        is_overridable_library=is_overridable_library, description=description,
        use_soft_limits=use_soft_limits, array_length=array_length, default_int=default_int,
        min_int=min_int, max_int=max_int, soft_min_int=soft_min_int, soft_max_int=soft_max_int,
        step_int=step_int, default_float=default_float, min_float=min_float,
        max_float=max_float, soft_min_float=soft_min_float, soft_max_float=soft_max_float,
        precision=precision, step_float=step_float, subtype=subtype,
        default_string=default_string, eval_string=eval_string
    )


setattr(bpy.ops.wm, "properties_edit", _new_properties_edit)
del _new_properties_edit

blender_default_properties_edit_value = bpy.ops.wm.properties_edit_value


def _new_properties_edit_value(data_path='', property_name='', eval_string=''):
    ctxt_obj = bpy.context.active_object
    if data_path == 'object' and ctxt_obj is not None and hasattr(ctxt_obj, "phobostype"):
        prohibited_keys = reserved_keys.INTERNAL_KEYS
        if hasattr(reserved_keys, ctxt_obj.phobostype.upper() + "_KEYS"):
            prohibited_keys += getattr(reserved_keys, ctxt_obj.phobostype.upper() + "_KEYS")
        if property_name in prohibited_keys:
            raise ValueError(f"{property_name} is reserved for internal usage. You can't edit reserved properties "
                             f"via 'Custom Properties', please use the 'Phobos Property Information' panel")

    blender_default_properties_edit_value(data_path=data_path, property_name=property_name, eval_string=eval_string)


setattr(bpy.ops.wm, "properties_edit_value", _new_properties_edit_value)
del _new_properties_edit_value

blender_default_remove_property = bpy.ops.wm.remove_property


def _new_remove_property(data_path='', property_name=''):
    ctxt_obj = bpy.context.active_object
    if data_path == 'object' and ctxt_obj is not None and hasattr(ctxt_obj, "phobostype"):
        prohibited_keys = reserved_keys.INTERNAL_KEYS
        if hasattr(reserved_keys, ctxt_obj.phobostype.upper() + "_KEYS"):
            prohibited_keys += getattr(reserved_keys, ctxt_obj.phobostype.upper() + "_KEYS")
        if property_name in prohibited_keys:
            raise ValueError(f"{property_name} is reserved for internal usage. You can't remove reserved it.")

    blender_default_remove_property(data_path=data_path, property_name=property_name)


setattr(bpy.ops.wm, "remove_property", _new_remove_property)
del _new_remove_property