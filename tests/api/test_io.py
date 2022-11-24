import unittest
import phobos
import difflib


class TestURDFIO(unittest.TestCase):

    def test_import(self):
        robot = phobos.core.Robot(inputfile="test_model/example_mechanism/urdf/example_mechanism.urdf")
        self.assertIsNotNone(robot.get_root())

    def test_export(self):
        robot = phobos.core.Robot(inputfile="test_model/example_mechanism/smurf/example_mechanism.smurf")
        robot.export_urdf(outputfile="test_model/temp/test.urdf")

        with open("test_model/example_mechanism/urdf/example_mechanism.urdf", "r") as f:
            ground_truth = f.readlines()
        with open("test_model/temp/test.urdf", "r") as f:
            export = f.readlines()

        if ground_truth != export:
            differ = difflib.Differ()
            print("".join(differ.compare(ground_truth, export)))
        self.assertEqual(ground_truth, export)


class TestConversion(unittest.TestCase):
    def test_sdf2urdf(self):
        # Note currently we copy the model not the mehes so the pathes are adapted accordingly and thereby lead to a diff
        robot = phobos.core.Robot(inputfile="test_model/example_mechanism/urdf/example_mechanism.urdf")
        robot.export_sdf(outputfile="test_model/temp/example_mechanism.sdf")
        robot = phobos.core.Robot(inputfile="test_model/temp/example_mechanism.sdf", verify_meshes_on_import=False)
        robot.export_urdf(outputfile="test_model/temp/example_mechanism.urdf")

        with open("test_model/example_mechanism/urdf/example_mechanism.urdf", "r") as f:
            ground_truth = [l for l in f.readlines() if "meshes/" not in l]
        with open("test_model/temp/example_mechanism.urdf", "r") as f:
            export = [l for l in f.readlines() if "meshes/" not in l]

        if ground_truth != export:
            differ = difflib.Differ()
            print("".join(differ.compare(ground_truth, export)))
        self.assertEqual(ground_truth, export)


class TestSMURFIO(unittest.TestCase):
    def test_import(self):
        robot = phobos.core.Robot(inputfile="test_model/example_mechanism/smurf/example_mechanism.smurf")
        self.assertIsNotNone(robot.get_root())

    def test_export(self):
        robot = phobos.core.Robot(inputfile="test_model/example_mechanism/smurf/example_mechanism.smurf")
        robot.export_smurf(outputdir="test_model/temp/")

        # Note currently we copy the model not the mehes so the pathes are adapted accordingly and thereby lead to a diff
        with open("test_model/example_mechanism/urdf/example_mechanism.urdf", "r") as f:
            ground_truth = [l for l in f.readlines() if "meshes/" not in l]
        with open("test_model/temp/urdf/example_mechanism.urdf", "r") as f:
            export = [l for l in f.readlines() if "meshes/" not in l]

        if ground_truth != export:
            differ = difflib.Differ()
            print("".join(differ.compare(ground_truth, export)))
        self.assertEqual(ground_truth, export)