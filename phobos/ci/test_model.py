import os

from phobos.core import Robot
from phobos.defs import dump_json, load_json, dump_yaml

class TestModel(object):
    def __init__(self, root, tolerances, model_in_repo=None, submechanisms_in_repo=None, floating_base=False,
                 floatingbase_submechanisms_in_repo=None, swing_my_robot=False):
        self.root = root
        self.robot = Robot(inputfile=os.path.abspath(model_in_repo),
                                submechanisms_file=os.path.abspath(submechanisms_in_repo), )
        self.submechanisms_file_path = self.robot.submechanisms_file
        self.submechanisms_file = None
        if os.path.isfile(self.submechanisms_file_path):
            self.submechanisms_file = load_json(
                open(os.path.join(self.root, self.submechanisms_file_path), "r").read())
        self.tolerances = tolerances
        self.floatingbase = floating_base
        self.swing_my_robot = swing_my_robot
        self._floatingbase_submechanisms_file_path = floatingbase_submechanisms_in_repo

    @property
    def urdf(self):
        return self.robot.xmlfile

    @property
    def floatingbase_urdf(self):
        """Convenience getter related to class TestModel"""
        return self.urdf[:-5]+"_floatingbase.urdf"

    @property
    def modelname(self):
        return os.path.basename(self.root)

    @property
    def modeldir(self):
        return self.root

    @property
    def floatingbase_submechanisms_file_path(self):
        if self._floatingbase_submechanisms_file_path is None:
            try1 = self.submechanisms_file_path.replace("_submechanisms", "_floatingbase_submechanisms")
            if os.path.exists(try1):
                return try1
            return self.submechanisms_file_path[:-4]+"_floatingbase.yml"
        else:
            return self._floatingbase_submechanisms_file_path

