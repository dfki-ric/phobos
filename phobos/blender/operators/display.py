import bpy
from bpy.props import BoolProperty
from bpy.types import Operator

from ..display import draw_callback_2d, draw_callback_3d

class DisplayInformationOperator(Operator):
    """Draw additional information about Phobos objects"""

    bl_idname = "phobos.display_information"
    bl_label = "Draw Model Information"

    def get_drawing_status(self):
        """TODO Missing documentation"""
        return bpy.context.window_manager.drawing_status

    def set_drawing_status(self, value):
        """

        Args:
          value:

        Returns:

        """
        bpy.context.window_manager.drawing_status = value

    running : BoolProperty(
        name="running",
        description="Whether the drawing thread is running or not.",
        get=get_drawing_status,
        set=set_drawing_status,
    )

    def modal(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        wm = context.window_manager
        context.area.tag_redraw()

        if not self.running:
            from phobos.blender.phoboslog import log

            bpy.types.SpaceView3D.draw_handler_remove(self._handle2d, 'WINDOW')
            bpy.types.SpaceView3D.draw_handler_remove(self._handle3d, 'WINDOW')
            log("Stop drawing Phobos information.", 'DEBUG')
            return {'FINISHED'}

        if event.type == 'PAGE_UP' and event.value == 'PRESS':
            wm.phobos_msg_offset += 1
        if event.type == 'PAGE_DOWN' and event.value == 'PRESS':
            wm.phobos_msg_offset -= 1
        if event.shift and event.type == 'LEFTMOUSE' and event.value == 'CLICK':
            pass

        return {'PASS_THROUGH'}

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        from phobos.blender.phoboslog import log

        wm = context.window_manager

        if context.area.type != 'VIEW_3D':
            from phobos.blender.phoboslog import log

            log("View3D not found, cannot run " + self.bl_idname, 'WARNING')
            return {'CANCELLED'}

        log("Start drawing Phobos information.", 'DEBUG')
        self.running = True
        self._handle2d = bpy.types.SpaceView3D.draw_handler_add(
            draw_callback_2d, (self, context), 'WINDOW', 'POST_PIXEL'
        )
        self._handle3d = bpy.types.SpaceView3D.draw_handler_add(
            draw_callback_3d, (self, context), 'WINDOW', 'POST_VIEW'
        )
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

class WireFrameOperator(Operator):
    """Toggle selected objects between wire frame and textured"""

    bl_idname = "phobos.display_wire"
    bl_label = "Toggle wire frame"

    def invoke(self, context, event):
        currentlyWireFrame = False

        for ob in context.selected_objects:
            if ob.display_type == "WIRE":
                currentlyWireFrame = True
                break

        for ob in context.selected_objects:
            if currentlyWireFrame:
                ob.display_type = "TEXTURED"
            else:
                ob.display_type = "WIRE"

        print("Wired:", currentlyWireFrame)
        return {'FINISHED'}


classes = [
    DisplayInformationOperator,
    WireFrameOperator
]

def register():
    """TODO Missing documentation"""
    # add the drawing status boolean to the window manager
    bpy.types.WindowManager.drawing_status = BoolProperty(
        default=False,
        name='Hide Model Information',
        description="Draw additional data visualization for Phobos items in 3D View.",
    )

    for c in classes:
        bpy.utils.register_class(c)


def unregister():
    """TODO Missing documentation"""
    # remove the drawing status boolean to the window manager
    del bpy.types.WindowManager.drawing_status

    for c in classes:
        bpy.utils.unregister_class(c)
