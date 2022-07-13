from ..io.yaml_reflection import YamlReflection


class SmurfBase(YamlReflection):
    """Base class for a smurf object related to a urdf.
    Wraps methods for joint and link as properties and additionally which variables get exported.
    """

    def __init__(self, target_type: str = None, **kwargs):
        super(YamlReflection, self).__init__(
            robot=kwargs["robot"] if "robot" in kwargs else None, target_type=target_type)

        # The object has to know which properties to export, this is done via
        self.returns = []
        # Additionally, we must exclude some private attributes
        self.excludes = ['returns', 'excludes']
        # if the SmurfBase instance has a target property this will set the expected type

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
                if category in self._type_dict.keys():
                    if type(information) == list:
                        information = [x if type(information) == str else information.name for x in information]
                    elif type(information) != str and information is not None:
                        information = information.name
                if information is not None:
                    setattr(self, category, information if category not in self._type_dict or type(information) == str else information.name)
        # The object has to know which properties to export, this is done via
        self.returns += list(kwargs.keys())
