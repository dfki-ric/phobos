from ..core.robot import Robot


class CompareModel(Robot):
    def __init__(self, name, robotfile, submechanisms_file=None, directory=None):
        super().__init__(name=name, inputfile=robotfile, submechanisms_file=submechanisms_file)
        self.directory = directory
