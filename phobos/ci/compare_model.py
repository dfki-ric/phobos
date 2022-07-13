from ..core.robot import Robot


class CompareModel(Robot):
    def __init__(self, name, robotfile, submechanisms_path=None, directory=None):
        super().__init__(name=name, inputfile=robotfile)
        self.submechanisms_path = submechanisms_path
        self.directory = directory
