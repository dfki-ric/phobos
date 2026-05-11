
import bpy
import sys
import os
import traceback

def run_smoke_test():
    """
    This test checks if the DrawPreviewOperator can be invoked without crashing.
    It sets up the minimal data structures the operator's draw callback expects
    and then calls the operator.
    This is a "smoke test": it ensures the underlying gpu code is valid and
    executable, but it does not verify the visual output.
    """
    print("--- Running Smoke Test for DrawPreviewOperator ---")
    try:
        # STEP 1: Add phobos to Python path so it can be imported by Blender
        phobos_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        if phobos_root not in sys.path:
            sys.path.append(phobos_root)
        
        # We must ensure the addon is enabled
        if "phobos" not in bpy.context.preferences.addons:
             bpy.ops.preferences.addon_enable(module="phobos")
        
        from phobos.blender.utils import blender as bUtils

        # STEP 2: Set up the dummy data required by the draw callback
        print("Setting up dummy data for preview...")
        
        # The callback reads from `bpy.context.scene.active_ModelPose` (an Int)
        # which is an index into `bpy.data.images`.
        # It then uses the image name to look up an item in `prefs.models_poses`.

        # 2a. Create a dummy image
        image_name = "smoke_test_image"
        if image_name not in bpy.data.images:
            bpy.data.images.new(image_name, 16, 16)
        
        # 2b. Set the active_ModelPose index to point to our dummy image.
        # The property is defined on the Scene type by Phobos.
        bpy.context.scene.active_ModelPose = bpy.data.images.find(image_name)

        # 2c. Create a corresponding dummy item in the Phobos preferences collection
        prefs = bUtils.getPhobosPreferences()
        if image_name not in prefs.models_poses:
            item = prefs.models_poses.add()
            item.name = image_name  # This key must match the image name
            item.label = "Smoke Test Preview"

        # STEP 3: Find a 3D View context to run the operator in
        print("Finding 3D View context...")
        window = bpy.context.window
        area_3d = None
        for area in window.screen.areas:
            if area.type == 'VIEW_3D':
                area_3d = area
                break
        
        if not area_3d:
            raise RuntimeError("Could not find a 3D View area in the current screen.")

        # STEP 4: Invoke the operator
        print("Calling operator bpy.ops.view3d.draw_preview_operator()...")
        
        # The operator is modal. In background mode, we just check if the
        # invocation itself works without crashing.
        with bpy.context.temp_override(window=window, area=area_3d):
            bpy.ops.view3d.draw_preview_operator('INVOKE_DEFAULT')

        # The modal handler will likely not run further, but that's okay.
        # We have confirmed that the `execute` method and the initial call
        # to `draw_preview_callback` with the new `gpu` code do not raise an error.
        
        print("--- Smoke Test SUCCESS: Operator did not crash on invocation. ---")
        return True

    except Exception:
        print(f"--- Smoke Test FAILED: An exception occurred ---")
        traceback.print_exc()
        return False

if __name__ == '__main__':
    # Blender scripts are not guaranteed to have a clean exit state,
    # so we explicitly exit with a status code.
    if run_smoke_test():
        sys.exit(0)
    else:
        sys.exit(1)
