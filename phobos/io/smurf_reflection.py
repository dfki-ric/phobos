from ..io.yaml_reflection import YamlReflection


class SmurfBase(YamlReflection):
    """Base class for a smurf object related to a urdf.
    Wraps methods for joint and link as properties and additionally which variables get exported.
    """

    def __init__(self, returns=None, **kwargs):
        super(YamlReflection, self).__init__()
        # The object has to know which properties to export, this is done via
        self.returns = [] if returns is None else returns
        # Additionally, we must exclude some private attributes
        self.excludes = ['returns', 'excludes']
        # if the SmurfBase instance has a target property this will set the expected type
        self.add_annotations(overwrite=True, **kwargs)

    # Define the export variables, overwrite the standard get_refl_vars
    def get_refl_vars(self):
        out = []
        # Collect all variables (and properties) which are given in the object self.returns
        export_props = [v for v in vars(self).keys() if v not in self.excludes and not v.startswith("_")] + self.returns
        for var in export_props:
            if getattr(self, var) is not None:
                out += [var]

        return list(set(out))

    def add_annotations(self, overwrite=False, **kwargs):
        # Just Parse everything else
        for category, information in kwargs.items():
            if category.startswith("_"):
                continue
            # The object has to know which properties to export, this is done via
            self.returns.append(category)
            if overwrite or not hasattr(self, category):
                if category in self.type_dict.keys():
                    if type(information) == list:
                        information = [x if type(x) == str else x.name for x in information]
                    elif type(information) != str and information is not None:
                        information = information.name
                if information is not None or category in self.returns:
                    try:
                        setattr(self, category, information)
                    except AttributeError as e:
                        print(category, information)
                        raise e
