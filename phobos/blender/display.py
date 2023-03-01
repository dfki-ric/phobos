#!/usr/bin/python

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import collections

import bgl
import blf
import bpy
from bpy.props import BoolProperty
from bpy.types import Operator
from bpy_extras import view3d_utils
from mathutils import Vector

from .utils import naming as nUtils

progressinfo = None
colors = {
    'debug': (1.0, 0.0, 1.0),
    'info': (0.0, 1.0, 0.0),
    'warning': (0.5, 0.25, 0.25),
    'error': (1.0, 0.0, 0.0),
    'none': (1.0, 1.0, 1.0),
    'submechanism': (0.9, 0.8, 0.3, 1.0),
    'background': (0.1, 0.1, 0.1, 0.8),
    'bright_background': (1.0, 1.0, 1.0, 0.7),
    'white': (1.0, 1.0, 1.0, 1.0),
    'black': (0.0, 0.0, 0.0, 1.0),
    'light_grey': (0.8, 0.8, 0.8, 1.0),
    'grey': (0.5, 0.5, 0.5, 1.0),
    'dark_grey': (0.2, 0.2, 0.2, 1.0),
    'red': (1.0, 0.0, 0.0, 1.0),
    'green': (0.0, 1.0, 0.0, 1.0),
    'blue': (0.0, 0.0, 0.1, 1.0),
    'transparent': (1.0, 1.0, 1.0, 0.0),
}
messages = collections.deque([], 50)
slotheight = 22
slotlower = [4 + slotheight * slot for slot in range(50)]


def push_message(text, msgtype='none'):
    """

    Args:
      text:
      msgtype: (Default value = 'none')

    Returns:

    """
    messages.appendleft({'text': text, 'type': msgtype})


def getRegionData():
    """TODO Missing documentation"""
    # region = [area.region for area in bpy.context.screen.areas if area.type == 'VIEW_3D'][0]
    return bpy.context.region, bpy.context.space_data.region_3d


def to3d(coords, distance=1.0):
    """

    Args:
      coords:
      distance: (Default value = 1.0)

    Returns:

    """
    # bgl.glUnProject()
    return view3d_utils.region_2d_to_origin_3d(*getRegionData(), (coords), distance)


def to2d(coords):
    """

    Args:
      coords:

    Returns:

    """
    return view3d_utils.location_3d_to_region_2d(*getRegionData(), coords)


def draw_2dpolygon(points, linecolor=None, fillcolor=None, distance=0.2, linewidth=1):
    """

    Args:
      points:
      linecolor: (Default value = None)
      fillcolor: (Default value = None)
      distance: (Default value = 0.2)
      linewidth: (Default value = 1)

    Returns:

    """
    # background
    bgl.glEnable(bgl.GL_BLEND)
    bgl.glLineWidth(linewidth)
    if fillcolor:
        #bgl.glColor4f(*fillcolor)
        bgl.glBegin(bgl.GL_POLYGON)
        for p in points:
            bgl.glVertex3f(*p, distance)
        bgl.glEnd()
    # frame
    if linecolor:
        #bgl.glColor4f(*linecolor)
        bgl.glBegin(bgl.GL_LINE_STRIP)
        for p in points:
            bgl.glVertex3f(*p, distance)
        bgl.glVertex3f(*points[0], distance)
        bgl.glEnd()
    bgl.glLineWidth(1)
    bgl.glDisable(bgl.GL_BLEND)


def draw_text(text, position, color=(1.0, 1.0, 1.0, 1.0), size=14, dpi=150, font_id=0):
    """

    Args:
      text:
      position:
      color: (Default value = (1.0)
      1.0:
      1.0):
      size: (Default value = 14)
      dpi: (Default value = 150)
      font_id: (Default value = 0)

    Returns:

    """
    #bgl.glColor4f(*color)
    blf.position(font_id, *position, 0.25)
    blf.size(font_id, size, dpi)
    blf.draw(font_id, text)


def draw_textbox(
    text,
    origin,
    textsize=6,
    textcolor=colors['white'],
    backgroundcolor=colors['background'],
    offset=Vector((0.0, 0.0)),
    linewidth=2,
    hborder=3,
    vborder=4,
    rightalign=False,
    indicator_line=True,
):
    """

    Args:
      text:
      origin:
      textsize: (Default value = 6)
      textcolor: (Default value = colors['white'])
      backgroundcolor: (Default value = colors['background'])
      offset: (Default value = Vector((0.0)
      0.0)):
      linewidth: (Default value = 2)
      hborder: (Default value = 3)
      vborder: (Default value = 4)
      rightalign: (Default value = False)
      indicator_line: (Default value = True)

    Returns:

    """
    blf.size(0, textsize, 150)
    width = blf.dimensions(0, text)[0]
    height = blf.dimensions(0, text)[1]
    if rightalign:
        origin = origin + Vector((-width - 2 * hborder, 0)) + offset
    else:
        origin = origin + offset
    points = (
        origin + Vector((-hborder, -vborder * 1.5)),
        origin + Vector((width + hborder, -vborder * 1.5)),
        origin + Vector((width + hborder, height + vborder)),
        origin + Vector((-hborder, height + vborder)),
    )
    draw_2dpolygon(points, fillcolor=backgroundcolor, linecolor=textcolor, linewidth=linewidth)
    draw_text(text, position=origin, size=textsize, color=textcolor)


def draw_message(text, msgtype, slot, opacity=1.0, offset=0):
    """

    Args:
      text:
      msgtype:
      slot:
      opacity: (Default value = 1.0)
      offset: (Default value = 0)

    Returns:

    """
    blf.size(0, 6, 150)
    width = bpy.context.region.width
    start = width - blf.dimensions(0, text)[0] - 6
    points = (
        (start, slotlower[slot]),
        (width - 2, slotlower[slot]),
        (width - 2, slotlower[slot] + slotheight - 4),
        (start, slotlower[slot] + slotheight - 4),
    )
    draw_2dpolygon(points, fillcolor=(*colors[msgtype], 0.2 * opacity))
    draw_text(text, (start + 2, slotlower[slot] + 4), size=6, color=(1, 1, 1, opacity))
    if slot == 0 and offset > 0:
        # draw_text(str(offset) + ' \u25bc', (start - 30, slotlower[0] + 4), size=6, color=(1, 1, 1, opacity))
        draw_text('+' + str(offset), (start - 30, slotlower[0] + 4), size=6, color=(1, 1, 1, 1))


def draw_progressbar(value):
    """

    Args:
      value:

    Returns:

    """
    region = bpy.context.region
    text = str(round(value * 100)) + '%'
    lc = (0.0, 1.0, 0.0, 1.0)
    fc = (0.0, 1.0, 0.0, 0.3)
    xstart = region.width * 0.2
    xend = region.width * 0.8
    span = xend - xstart
    points = (
        (xstart, region.height - 10),
        (xend, region.height - 10),
        (xend, region.height - 25),
        (xstart, region.height - 25),
    )
    progresspoints = (
        (xstart, region.height - 10),
        (xstart + value * span, region.height - 10),
        (xstart + value * span, region.height - 25),
        (xstart, region.height - 25),
    )
    draw_2dpolygon(points, linecolor=lc, fillcolor=fc)
    draw_2dpolygon(progresspoints, fillcolor=lc, distance=0.2)
    draw_text(text, position=(xend + 20, region.height - 25), size=9)
    #if progressinfo:
        #draw_text(progressinfo, position=(xstart + 10, region.height - 42), size=6)


def draw_joint(joint, length):
    """

    Args:
      joint:
      length:

    Returns:

    """
    origin = Vector(joint.matrix_world.to_translation())
    axis = joint.matrix_world @ (length * joint.data.bones[0].vector.normalized())
    endpoint = axis

    #bgl.glColor4f(0.0, 1.0, 0.0, 0.5)
    bgl.glLineWidth(2)
    bgl.glBegin(bgl.GL_LINE_STRIP)
    bgl.glVertex3f(*origin)
    bgl.glVertex3f(*endpoint)
    bgl.glEnd()


def draw_path(path, color=colors['white'], dim3=False, width=4):
    """

    Args:
      path:
      color: (Default value = colors['white'])
      dim3: (Default value = False)
      width: (Default value = 4)

    Returns:

    """
    origins = []
    for e in range(len(path)):
        origins.append(path[e].matrix_world.to_translation())

    bgl.glEnable(bgl.GL_BLEND)
    bgl.glLineWidth(width)

    bgl.glBegin(bgl.GL_LINE_STRIP)
    #bgl.glColor4f(*color)
    for o in origins:
        if dim3:
            bgl.glVertex3f(o)
        else:
            bgl.glVertex2f(*to2d(o))
    bgl.glEnd()
    bgl.glDisable(bgl.GL_BLEND)


def draw_callback_3d(self, context):
    """Callback function for 3d drawing.

    Args:
      context:

    Returns:

    """
    active = context.object
    selected = context.selected_objects
    wm = context.window_manager

    bgl.glEnable(bgl.GL_BLEND)

    # joint axes
    if len(selected) > 0:
        if wm.draw_jointaxes:
            for j in [o for o in selected if o.phobostype == 'link']:
                draw_joint(j, wm.jointaxes_length)

    # restore opengl defaults
    bgl.glLineWidth(1)
    bgl.glDisable(bgl.GL_BLEND)
    #bgl.glColor4f(0.0, 0.0, 0.0, 1.0)


def draw_callback_2d(self, context):
    """Callback function for 2d drawing.

    Args:
      context:

    Returns:

    """
    active = context.object
    selected = context.selected_objects
    if active and selected:
        objects = set(selected + [active])
    else:
        objects = []
    wm = context.window_manager

    # code that can be used to draw on 2d surface in 3d mode, not used any more
    # due to separate handlers for 2d and 3d
    # bgl.glMatrixMode(bgl.GL_PROJECTION)
    # bgl.glLoadIdentity()
    # bgl.gluOrtho2D(0, bpy.context.region.width, 0, bpy.context.region.height)
    # bgl.glMatrixMode(bgl.GL_MODELVIEW)
    # bgl.glLoadIdentity()

    bgl.glEnable(bgl.GL_BLEND)

    # submechanisms
    if objects and wm.draw_submechanisms:
        submechanism_roots = [obj for obj in bpy.data.objects if 'submechanism/name' in obj]
        # draw spanning trees
        for root in submechanism_roots:
            if 'submechanism/spanningtree' in root:
                if set(root['submechanism/spanningtree']).intersection(
                    set(bpy.context.selected_objects)
                ):
                    linecolor = colors['submechanism']
                    linewidth = 5
                else:
                    linecolor = (*colors['submechanism'][:3], 0.4)
                    linewidth = 3
                draw_path(root['submechanism/spanningtree'], color=linecolor, width=linewidth)
                # joint['submechanism/independent'],
                # joint['submechanism/active'])
                avgpos = Vector()
                for obj in root['submechanism/spanningtree']:
                    avgpos += obj.matrix_world.translation
                origin = to2d(avgpos / len(root['submechanism/spanningtree']))
                draw_textbox(
                    root['submechanism/name'],
                    origin,
                    textsize=8,
                    textcolor=linecolor,
                    backgroundcolor=colors['background'],
                    offset=Vector((-30, 0)),
                )

    # joints
    if objects and wm.draw_jointnames:
        for obj in [obj for obj in objects if obj.phobostype == 'link']:
            origin = to2d(obj.matrix_world.translation)
            draw_textbox(
                nUtils.getObjectName(obj, 'joint'),
                origin,
                textsize=6,
                textcolor=colors['dark_grey'],
                backgroundcolor=colors['bright_background'],
                rightalign=True,
                offset=Vector((-16, 0)),
            )
            if wm.draw_submechanisms and 'submechanism/jointname' in obj:
                draw_textbox(
                    obj['submechanism/jointname'],
                    origin,
                    textsize=6,
                    textcolor=colors['dark_grey'],
                    backgroundcolor=(*colors['submechanism'][:3], 0.7),
                    offset=Vector((16, 0)),
                )

    # interfaces
    if selected:
        for interface in [obj for obj in selected if obj.phobostype == 'interface']:
            color = interface.active_material.diffuse_color
            maxc = max(color)
            color = [0.6 + c / maxc * 0.4 for c in color]
            bgcolor = [c * 0.4 for c in interface.active_material.diffuse_color]
            draw_textbox(
                nUtils.getObjectName(interface),
                to2d(interface.matrix_world.translation),
                textsize=6,
                textcolor=(*color, 1.0 if interface.show_name else 0.4),
                backgroundcolor=(*bgcolor, 1.0) if interface.show_name else colors['background'],
                linewidth=3 if interface.show_name else 2,
            )

    # progress bar
    #if wm.draw_progress and context.window_manager.progress not in [0, 1]:
    #    draw_progressbar(wm.progress)

    # log messages
    if wm.draw_messages:
        for m in range(wm.phobos_msg_count):
            opacity = 1.0
            if 1 >= m <= wm.phobos_msg_offset - 1 or m >= wm.phobos_msg_count - 2:
                opacity = 0.5
            if wm.phobos_msg_offset > 1 > m or m >= wm.phobos_msg_count - 1:
                opacity = 0.1
            try:
                msg = messages[m + wm.phobos_msg_offset]
                draw_message(msg['text'], msg['type'], m, opacity, offset=wm.phobos_msg_offset)
            except IndexError:
                pass

    # restore opengl defaults
    bgl.glLineWidth(1)
    bgl.glDisable(bgl.GL_BLEND)
    #bgl.glColor4f(0.0, 0.0, 0.0, 1.0)


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


def setProgress(value, info=None):
    """

    Args:
      value:
      info: (Default value = None)

    Returns:

    """
    global progressinfo
    win = bpy.context.window_manager
    if not progressinfo:
        win.progress_begin(0, 100)
        win.progress_update(value*100)
        progressinfo = value
    elif progressinfo > value:
        win.progress_end()
        win.progress_begin(0, 100)
        win.progress_update(value*100)
        progressinfo = value
    #c.window_manager.progress = value
    #import sys
    #    if not "-b" in sys.argv:
    #bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    #progressinfo = info
    #
    # for area in c.screen.areas:
    #     if area.type == 'VIEW_3D':
    #         override = {'window': c.window, 'screen': c.screen, 'area': area}
    #         area.tag_redraw()
    #         break

def endProgress():
    global progressinfo
    win = bpy.context.window_manager
    if progressinfo:
        win.progress_end()
        progressinfo = None

def register():
    """TODO Missing documentation"""
    # add the drawing status boolean to the window manager
    bpy.types.WindowManager.drawing_status = BoolProperty(
        default=False,
        name='Hide Model Information',
        description="Draw additional data visualization for Phobos items in 3D View.",
    )

    bpy.utils.register_class(DisplayInformationOperator)


def unregister():
    """TODO Missing documentation"""
    # remove the drawing status boolean to the window manager
    del bpy.types.WindowManager.drawing_status

    bpy.utils.unregister_class(DisplayInformationOperator)
