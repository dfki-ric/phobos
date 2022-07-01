from ..io import representation
from ..io.base import Representation
from ..io.yaml_reflection import YamlReflection


class SmurfBase(YamlReflection, Representation):
    """Base class for a smurf object related to a urdf.
    Wraps methods for joint and link as properties and additionally which variables get exported.
    """

    def __init__(self, **kwargs):
        super().__init__()
        # Just Parse everything else
        for category, information in kwargs.items():
            setattr(self, category, information)

        # The object has to know which properties to export, this is done via
        self.returns = []
        # Additionally, we must exclude some private attributes
        self.excludes = ['returns', 'excludes']

    # Define the export variables, overwrite the standard get_refl_vars
    def get_refl_vars(self):
        # Collect all variables (and properties) which are given in the object self.returns
        export_props = []
        for var in self.returns:
            if hasattr(self, var) and getattr(self, var) is not None:
                export_props += [var]

        # Collect all other vars
        export_props += [v for v in vars(self).keys() if v not in self.excludes]

        return list(set(export_props))


class SmurfAnnotation(SmurfBase):
    """Base class for a smurf object related to a urdf.
    Wraps methods for joint and link as properties and additionally which variables get exported.
    """

    def __init__(self, robot=None, name=None, joint=None, link=None, **kwargs):
        super().__init__()
        # Name of the object
        self.name = name
        # If a joint is given, add that
        if type(joint) is str:
            self._joint = robot.get_joint(joint)
            if self._joint is None:
                raise NameError("There is no joint with the name "+joint)
        else:
            self._joint = joint
        # If a link is given, add that
        if type(link) is str:
            self._link = robot.get_link(link)
            if self._link is None:
                raise NameError("There is no link with the name "+link)
        else:
            self._link = link

        # Just Parse everything else
        for category, information in kwargs.items():
            setattr(self, category, information)

        # The object has to know which properties to export, this is done via
        self.returns += []
        # Additionally, we must exclude some private attributes
        self.excludes += ['_joint', '_link', 'robot']

    # Define the joint as a property
    @property
    def joint(self):
        if type(self._joint) is str:
            return self._joint
        elif isinstance(self._joint, representation.Joint):
            return self._joint.name
        else:
            return None

    @joint.setter
    def joint(self, joint):
        if not isinstance(joint, representation.Joint):
            self._joint = None
        else:
            self._joint = joint

    # Define the link as a property
    @property
    def link(self):
        if type(self._link) is str:
            return self._link
        elif isinstance(self._link, representation.Link):
            return self._link.name
        else:
            return None

    @link.setter
    def link(self, link):
        if not isinstance(link, representation.Link):
            self._link = None
        else:
            self._link = link
