
import bpy
import unittest
import os
import sys

# Add the phobos root to the python path to allow imports
# The script is in phobos/blender/utils/, so we need to go up 3 levels
phobos_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if phobos_root not in sys.path:
    sys.path.append(phobos_root)

# Now we can import the function to be tested
try:
    from phobos.blender.utils import blender as bUtils
    from phobos.blender.utils import selection as sUtils
except ImportError as e:
    print(f"Failed to import Phobos modules: {e}")
    print(f"Current sys.path: {sys.path}")
    # Exit gracefully if imports fail, as tests can't run
    sys.exit(1)


class TestCreatePreview(unittest.TestCase):

    def setUp(self):
        """Create a clean scene for each test."""
        bpy.ops.scene.new(type='NEW')
        self.test_scene = bpy.context.scene
        # Ensure temp directory exists
        self.temp_dir = os.path.join(phobos_root, "temp")
        os.makedirs(self.temp_dir, exist_ok=True)

    def tearDown(self):
        """Remove the test scene."""
        # This is tricky as there is no direct 'delete scene' op that always works
        # For now, we just switch back to the default scene if it exists
        if "Scene" in bpy.data.scenes:
            bpy.context.window.scene = bpy.data.scenes["Scene"]
        
        # Clean up test scenes
        for scene in bpy.data.scenes:
            if scene.name.startswith("TestScene"):
                 bpy.data.scenes.remove(scene)
        for world in bpy.data.worlds:
            if world.name.startswith("TestWorld"):
                bpy.data.worlds.remove(world)


    def test_createPreview_no_background_node(self):
        """Test that createPreview runs without error when the world has no background node."""
        print("\nRunning test: test_createPreview_no_background_node...")
        
        # Ensure the world and node tree exist, but remove the Background node
        world = bpy.data.worlds.new("TestWorld_NoBG")
        self.test_scene.world = world
        
        world.use_nodes = True
        node_tree = world.node_tree
        
        # Remove the default 'Background' node if it exists
        if 'Background' in node_tree.nodes:
            node_tree.nodes.remove(node_tree.nodes['Background'])

        # We need a camera and an object for the preview function to work
        bpy.ops.object.camera_add()
        camera = bpy.context.active_object
        self.test_scene.camera = camera
        bpy.ops.mesh.primitive_cube_add()
        cube = bpy.context.active_object
        
        objects_to_preview = [cube]

        try:
            # The function is defined in blender.py in phobos/blender/utils
            bUtils.createPreview(objects_to_preview, self.temp_dir, "test_preview_no_bg")
            # If the function completes without an exception, the test passes.
            print("...Success: createPreview completed without error.")
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"createPreview() raised an exception unexpectedly with no background node: {e}")

    def test_createPreview_with_background_node(self):
        """Test that createPreview runs correctly with a standard background node."""
        print("\nRunning test: test_createPreview_with_background_node...")

        # We need a camera and an object
        bpy.ops.object.camera_add()
        camera = bpy.context.active_object
        self.test_scene.camera = camera
        bpy.ops.mesh.primitive_cube_add()
        cube = bpy.context.active_object
        
        objects_to_preview = [cube]

        try:
            model_name = "test_preview_with_bg"
            bUtils.createPreview(objects_to_preview, self.temp_dir, model_name)
            # Check if the output file was created
            target_file = os.path.join(self.temp_dir, model_name + ".png")
            self.assertTrue(os.path.exists(target_file), "Preview file was not created")
            print("...Success: Preview file was created.")
        except Exception as e:
            self.fail(f"createPreview() raised an exception unexpectedly with a background node: {e}")

def run():
    """Function to be called to run the tests."""
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(TestCreatePreview))
    runner = unittest.TextTestRunner()
    # run() returns a TestResult object
    result = runner.run(suite)
    # Exit with a non-zero status code if tests failed
    if not result.wasSuccessful():
        sys.exit(1)

# This allows running the test from the command line via Blender
if __name__ == '__main__':
    run()
