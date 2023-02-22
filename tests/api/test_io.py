import unittest
import difflib
import os

import phobos
# phobos.commandline_logging.setup_logger_level("DEBUG")


class TestURDFIO(unittest.TestCase):
    def test_import(self):
        robot = phobos.core.Robot(inputfile="test_data/example_mechanism/urdf/example_mechanism.urdf")
        self.assertIsNotNone(robot.get_root())

    def test_export(self):
        robot = phobos.core.Robot(inputfile="test_data/example_mechanism/smurf/example_mechanism.smurf")
        robot.export_urdf(outputfile="test_data/.temp/test.urdf")

        # Note: here we reuse the meshes therefore the path changes
        with open("test_data/example_mechanism/urdf/example_mechanism.urdf", "r") as f:
            ground_truth = [l for l in f.readlines() if "meshes/" not in l]
        with open("test_data/.temp/urdf/example_mechanism.urdf", "r") as f:
            export = [l for l in f.readlines() if "meshes/" not in l]

        if ground_truth != export:
            differ = difflib.Differ()
            print("".join(differ.compare(ground_truth, export)))
        self.assertEqual(ground_truth, export)


class TestSMURFIO(unittest.TestCase):
    def test_import(self):
        robot = phobos.core.Robot(inputfile="test_data/example_mechanism/smurf/example_mechanism.smurf")
        self.assertIsNotNone(robot.get_root())

    def test_export(self):
        robot = phobos.core.Robot(inputfile="test_data/example_mechanism/smurf/example_mechanism.smurf")
        robot.export(outputdir="test_data/.temp/", export_config=phobos.utils.resources.get_default_export_config("minimal"))

        with open("test_data/example_mechanism/urdf/example_mechanism.urdf", "r") as f:
            ground_truth = f.readlines()
        with open("test_data/.temp/urdf/example_mechanism.urdf", "r") as f:
            export = f.readlines()

        if ground_truth != export:
            differ = difflib.Differ()
            print("".join(differ.compare(ground_truth, export)))
        self.assertEqual(ground_truth, export)
        self.assertTrue(os.path.isfile("test_data/.temp/meshes/stl/Cone.stl"))


class TestConversion(unittest.TestCase):
    def test_sdf2urdf(self):
        # Note currently we copy the model not the mehes so the pathes are adapted accordingly and thereby lead to a diff
        robot = phobos.core.Robot(inputfile="test_data/example_mechanism/urdf/example_mechanism.urdf")
        robot.export_sdf(outputfile="test_data/.temp/example_mechanism.sdf")
        robot = phobos.core.Robot(inputfile="test_data/.temp/example_mechanism.sdf", verify_meshes_on_import=False)
        robot.export_urdf(outputfile="test_data/.temp/example_mechanism.urdf")

        # Note: here we reuse the meshes therefore the path changes
        with open("test_data/example_mechanism/urdf/example_mechanism.urdf", "r") as f:
            ground_truth = [l for l in f.readlines() if "meshes/" not in l]
        with open("test_data/.temp/example_mechanism.urdf", "r") as f:
            export = [l for l in f.readlines() if "meshes/" not in l]

        if ground_truth != export:
            differ = difflib.Differ()
            print("".join(differ.compare(ground_truth, export)))
        self.assertEqual(ground_truth, export)

