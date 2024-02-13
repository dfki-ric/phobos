#!/usr/bin/python

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import collections

import gpu
from gpu_extras.batch import batch_for_shader
import blf
import bpy
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


def draw_2dpolygon(left, top, width, height, linecolor=None, fillcolor=None, linewidth=1):
    """

    Args:
      left:
      top:
      width:
      height:
      linecolor: (Default value = None)
      fillcolor: (Default value = None)
      linewidth: (Default value = 1)

    Returns:

    """

    points = (
        (left, top),
        (left + width, top),
        (left + width, top + height),
        (left, top + height)
    )

    # background
    gpu.state.blend_set("ALPHA")
    shader = gpu.shader.from_builtin('2D_SMOOTH_COLOR')
    color = len(points)*[fillcolor]
    if fillcolor:
        batch = batch_for_shader(shader, 'TRI_FAN', {"pos": points, "color": color})
        batch.draw(shader)
    # frame
    if linecolor:
        batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": points, "color": color})
        batch.draw(shader)


def draw_text(text, position, color=(1.0, 1.0, 1.0, 1.0), size=14, dpi=150, font_id=0):
    """

    Args:
      text:
      position:
      color: (Default value = (1.0, 1.0, 1.0, 1.0)):
      size: (Default value = 14)
      dpi: (Default value = 150)
      font_id: (Default value = 0)

    Returns:

    """
    blf.color(font_id, *color)
    blf.position(font_id, *position, 0.25)
    blf.size(font_id, size, dpi)
    blf.draw(font_id, text)

def get_text_width(text, size=14, dpi=150, font_id=0):
    """

    Args:
      text:
      size: (Default value = 14)
      dpi: (Default value = 150)
      font_id: (Default value = 0)

    Returns:

    """
    #bgl.glColor4f(*color)
    blf.size(font_id, size, dpi)
    return blf.dimensions(font_id, text)[0]


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
    l, t = origin[0]-hborder, origin[1]-vborder * 1.5
    w, h = width + 2*hborder, height + 2.5*vborder
    draw_2dpolygon(l, t, w, h, fillcolor=backgroundcolor, linecolor=textcolor, linewidth=linewidth)
    draw_text(text, position=origin, size=textsize, color=textcolor)


def draw_message(text, msgtype, slot, opacity=1.0, scroll=0):
    """

    Args:
      text:
      msgtype:
      slot:
      opacity: (Default value = 1.0)
      scroll: (Default value = 0)

    Returns:

    """
    blf.size(0, 6, 150)
    margin = 4
    start = 20
    if slot == 0 and scroll > 0:
        start = start + get_text_width('+' + str(scroll), size=6) + margin
    blocksize = 10
    left, top = start, slotlower[slot] + 4
    width, height = blocksize, blocksize
    draw_2dpolygon(left, top, width, height, fillcolor=(*colors[msgtype], 0.2 * opacity))
    draw_text(text, (start + margin + blocksize - 1, slotlower[slot] + 4 - 1), size=6, color=(0, 0, 0, opacity))
    draw_text(text, (start + margin + blocksize, slotlower[slot] + 4), size=6, color=(1, 1, 1, opacity))
    if slot == 0 and scroll > 0:
        draw_text('+' + str(scroll), (10, slotlower[0] + 4), size=6, color=(1, 1, 1, 1))


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

    points = (origin, endpoint)
    color = 2*[(0.0, 1.0, 0.0, 0.5)]
    shader = gpu.shader.from_builtin("3D_SMOOTH_COLOR")
    batch = batch_for_shader(shader, "LINE_STRIP", {"pos": points, "color": color})
    batch.draw(shader)


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

    gpu.state.blend_set("ALPHA")
    points = []
    for o in origins:
        if dim3:
            points.append(o)
        else:
            points.append(to2d(o))
    c = len(points)*[color]
    if dim3:
        shader = gpu.shader.from_builtin("3D_SMOOTH_COLOR")
    else:
        shader = gpu.shader.from_builtin("2D_SMOOTH_COLOR")
    batch = batch_for_shader(shader, "LINE_STRIP", {"pos": points, "color": c})
    batch.draw(shader)


def draw_callback_3d(self, context):
    """Callback function for 3d drawing.

    Args:
      context:

    Returns:

    """
    selected = context.selected_objects
    wm = context.window_manager

    gpu.state.blend_set("ALPHA")

    # joint axes
    if len(selected) > 0:
        if wm.draw_jointaxes:
            for j in [o for o in selected if o.phobostype == 'link']:
                draw_joint(j, wm.jointaxes_length)


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

    # submechanisms
    if objects and wm.draw_submechanisms:
        submechanism_roots = [obj for obj in bpy.data.objects if obj.phobostype == "submechanism"]
        # draw spanning trees
        for root in submechanism_roots:
            if 'jointnames_spanningtree' in root:
                spanningTree = root['jointnames_spanningtree']
                if set(spanningTree).intersection(
                    set(bpy.context.selected_objects)
                ):
                    linecolor = colors['submechanism']
                    linewidth = 5
                else:
                    linecolor = (*colors['submechanism'][:3], 0.4)
                    linewidth = 3
                draw_path(spanningTree, color=linecolor, width=linewidth)
                # joint['submechanism/independent'],
                # joint['submechanism/active'])
                avgpos = Vector()
                for obj in spanningTree:
                    avgpos += obj.matrix_world.translation
                origin = to2d(avgpos / len(spanningTree))
                draw_textbox(
                    root['name'],
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
                textcolor=(*color[0:3], 1.0 if interface.show_name else 0.4),
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
            if 1 >= m <= wm.phobos_msg_scroll - 1 or m >= wm.phobos_msg_count - 2:
                opacity = 0.5
            if wm.phobos_msg_scroll > 1 > m or m >= wm.phobos_msg_count - 1:
                opacity = 0.1
            try:
                msg = messages[m + wm.phobos_msg_scroll]
                draw_message(msg['text'], msg['type'], m, opacity, scroll=wm.phobos_msg_scroll)
            except IndexError:
                pass

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
