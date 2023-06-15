import os

from ..core import Robot
from ..defs import load_json


class TestModel(object):
    def __init__(self, root, test, model_in_repo=None, submechanisms_in_repo=None, floating_base=False,
                 floatingbase_submechanisms_in_repo=None, swing_my_robot=False, tempdir=None):
        self.model_in_repo = model_in_repo
        self.submechanisms_in_repo = submechanisms_in_repo
        self.tempdir = tempdir
        self.root = root
        self.robot = None
        self.test = test
        if "model_in_repo" not in self.test["compare_model"] or self.test["compare_model"] is None:
            self.test["compare_model"]["model_in_repo"] = self.model_in_repo
        if "submechanisms_in_repo" not in self.test["compare_model"] or self.test["compare_model"] is None:
            self.test["compare_model"]["submechanisms_in_repo"] = self.submechanisms_in_repo
        self.floatingbase = floating_base
        self.swing_my_robot = swing_my_robot
        self._floatingbase_submechanisms_file_path = floatingbase_submechanisms_in_repo

    def recreate_sym_links(self):
        pass

    def _load_robot(self):
        self.robot = Robot(
            inputfile=os.path.abspath(self.model_in_repo),
            submechanisms_file=os.path.abspath(self.submechanisms_in_repo)
            if self.submechanisms_in_repo is not None else None
        )
        if self.submechanisms_in_repo is None and self.submechanisms_file_path is not None:
            assert os.path.isabs(self.submechanisms_file_path)
            self.submechanisms_in_repo = self.submechanisms_file_path

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
    def robotname(self):
        return self.robot.name

    @property
    def modeldir(self):
        return self.root

    @property
    def submechanisms_file(self):
        if self.robot is not None:
            return self.robot.submechanisms_file
        return self.submechanisms_in_repo

    @property
    def floatingbase_submechanisms_file_path(self):
        if self._floatingbase_submechanisms_file_path is None:
            try1 = self.submechanisms_file.replace("_submechanisms", "_floatingbase_submechanisms")
            if os.path.exists(try1):
                return try1
            return self.submechanisms_file[:-4]+"_floatingbase.yml"
        else:
            return self._floatingbase_submechanisms_file_path

    @property
    def exportdir(self):
        return self.root