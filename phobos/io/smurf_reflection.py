from ..io.yaml_reflection import YamlReflection


class SmurfBase(YamlReflection):
    """Base class for a smurf object related to a urdf.
    Wraps methods for joint and link as properties and additionally which variables get exported.
    """

    def __init__(self, **kwargs):
        # The object has to know which properties to export, this is done via
        self.returns = []
        # Additionally, we must exclude some private attributes
        self.excludes = ['returns', 'excludes']

        self.add_annotations(**kwargs)

    # Define the export variables, overwrite the standard get_refl_vars
    def get_refl_vars(self):
        # Collect all variables (and properties) which are given in the object self.returns
        export_props = []
        for var in self.returns:
            if getattr(self, var) is not None:
                export_props += [var]

        # Collect all other vars
        export_props += [v for v in vars(self).keys() if v not in self.excludes]

        return list(set(export_props))

    def add_annotations(self, overwrite=False, **kwargs):
        # Just Parse everything else
        for category, information in kwargs.items():
            if overwrite or not hasattr(self, category):
                setattr(self, category, information)
        # The object has to know which properties to export, this is done via
        self.returns += list(kwargs.keys())
