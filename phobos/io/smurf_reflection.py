from ..io.yaml_reflection import YamlReflection


# [Todo v2.1.0] we need to introduce a key dict for the return properties to store properties under different smurf keys
class SmurfBase(YamlReflection):
    """Base class for a smurf object related to a urdf.
    Wraps methods for joint and link as properties and additionally which variables get exported.
    """

    def __init__(self, returns=None, **kwargs):
        assert "robot" not in kwargs
        super(YamlReflection, self).__init__()
        # The object has to know which properties to export, this is done via
        self.returns = [] if returns is None else returns
        # Additionally, we must exclude some private attributes
        self.excludes = ['returns', 'excludes']
        # if the SmurfBase instance has a target property this will set the expected type
        self.add_annotations(overwrite=True, **kwargs)

    def __str__(self):
        if hasattr(self, "name"):
            return self.name
        raise NotImplementedError("Not implemented for "+str(type(self)))

    def set_unique_name(self, value):
        if hasattr(self, "name"):
            self.name = value
        else:
            raise NotImplementedError("Not implemented for "+str(type(self)))

    # Define the export variables, overwrite the standard get_refl_vars
    def get_refl_vars(self):
        out = []
        # Collect all variables (and properties) which are given in the object self.returns
        export_props = [v for v in vars(self).keys() if v not in self.excludes and not v.startswith("_")] + \
                       [v for v in set(self.returns) if v not in self.excludes]
        for var in export_props:
            value = getattr(self, var)
            if value is not None:  # this we have to do via redefining get_refl_vars and (type(value) not in [list, tuple, set, dict] or len(value) > 0):
                out += [var]

        return list(set(out))

    def add_annotation(self, key, value, overwrite=False):
        self.add_annotations(overwrite=overwrite, **{key: value})

    def add_annotations(self, overwrite=False, **kwargs):
        # Just Parse everything else
        for category, information in kwargs.items():
            if category.startswith("_"):
                continue
            # The object has to know which properties to export, this is done via
            self.returns.append(category)
            if overwrite or not hasattr(self, category) or getattr(self, category) is None:
                if category in self.type_dict.keys():
                    if type(information) == list:
                        information = [x if type(x) == str else str(x) for x in information]
                    elif type(information) != str and information is not None:
                        information = str(information)
                if information is not None or category in self.returns:
                    try:
                        setattr(self, category, information)
                    except AttributeError:
                        raise AttributeError(f"Can't set attribute {category} to {information} of {str(type(self))} {str(self)}")
