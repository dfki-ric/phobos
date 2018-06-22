import collections
import bpy
import bgl
import blf
from bpy_extras import view3d_utils
from mathutils import Vector
from phobos.utils import naming as nUtils


progressinfo = None
colors = {'debug': (1.0, 0.0, 1.0),
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
slotlower = [4 + slotheight*slot for slot in range(50)]


def push_message(text, msgtype='none'):
    messages.appendleft({'text': text, 'type': msgtype})


def start_draw_operator(self, context):
    if self.draw_phobos_infos:
        bpy.ops.phobos.draw_infos_operator('INVOKE_DEFAULT')
    return None


def getRegionData():
    # region = [area.region for area in bpy.context.screen.areas if area.type == 'VIEW_3D'][0]
    return bpy.context.region, bpy.context.space_data.region_3d


def to3d(coords, distance=1.0):
    # bgl.glUnProject()
    return view3d_utils.region_2d_to_origin_3d(*getRegionData(), (coords), distance)


def to2d(coords):
    return view3d_utils.location_3d_to_region_2d(*getRegionData(), coords)


def draw_2dpolygon(points, linecolor=None, fillcolor=None, distance=0.2, linewidth=1):
    # background
    bgl.glEnable(bgl.GL_BLEND)
    bgl.glLineWidth(linewidth)
    if fillcolor:
        bgl.glColor4f(*fillcolor)
        bgl.glBegin(bgl.GL_POLYGON)
        for p in points:
            bgl.glVertex3f(*p, distance)
        bgl.glEnd()
    # frame
    if linecolor:
        bgl.glColor4f(*linecolor)
        bgl.glBegin(bgl.GL_LINE_STRIP)
        for p in points:
            bgl.glVertex3f(*p, distance)
        bgl.glVertex3f(*points[0], distance)
        bgl.glEnd()
    bgl.glLineWidth(1)
    bgl.glDisable(bgl.GL_BLEND)


def draw_text(text, position, color=(1.0, 1.0, 1.0, 1.0), size=14, dpi=150, font_id=0):
    bgl.glColor4f(*color)
    blf.position(font_id, *position, 0.25)
    blf.size(font_id, size, dpi)
    blf.draw(font_id, text)


def draw_textbox(text, origin, textsize=6, textcolor=colors['white'],
                 backgroundcolor=colors['background'], offset=Vector((0.0, 0.0)),
                 linewidth=2, hborder=3, vborder=3, rightalign=False,
                 indicator_line=True):
    blf.size(0, textsize, 150)
    width = blf.dimensions(0, text)[0]
    height = blf.dimensions(0, text)[1]
    if rightalign:
        origin = origin + Vector((-width-2*hborder, 0)) + offset
    else:
        origin = origin + offset
    points = ((origin + Vector((-hborder, -vborder * 1.5)),
               origin + Vector((width + hborder, -vborder * 1.5)),
               origin + Vector((width + hborder, height + vborder)),
               origin + Vector((-hborder, height + vborder))))
    draw_2dpolygon(points, fillcolor=backgroundcolor,
                   linecolor=textcolor, linewidth=linewidth)
    draw_text(text, position=origin, size=textsize, color=textcolor)


def draw_message(text, msgtype, slot, opacity=1.0, offset=0):
    blf.size(0, 6, 150)
    width = bpy.context.region.width
    start = width - blf.dimensions(0, text)[0]-6
    points = ((start, slotlower[slot]), (width-2, slotlower[slot]),
              (width-2, slotlower[slot]+slotheight-4), (start, slotlower[slot]+slotheight-4))
    draw_2dpolygon(points, fillcolor=(*colors[msgtype], 0.2*opacity))
    draw_text(text, (start+2, slotlower[slot]+4), size=6, color=(1, 1, 1, opacity))
    if slot == 0 and offset > 0:
        #draw_text(str(offset) + ' \u25bc', (start - 30, slotlower[0] + 4), size=6, color=(1, 1, 1, opacity))
        draw_text('+'+str(offset), (start - 30, slotlower[0] + 4), size=6, color=(1, 1, 1, 1))


def draw_progressbar(value):
    region = bpy.context.region
    text = str(round(value*100))+'%'
    lc = (0.0, 1.0, 0.0, 1.0)
    fc = (0.0, 1.0, 0.0, 0.3)
    xstart = region.width*0.2
    xend = region.width*0.8
    span = xend-xstart
    points = ((xstart, region.height-10),
              (xend, region.height-10),
              (xend, region.height-25),
              (xstart, region.height-25))
    progresspoints = ((xstart, region.height-10),
                      (xstart+value*span, region.height-10),
                      (xstart+value*span, region.height-25),
                      (xstart, region.height-25))
    draw_2dpolygon(points, linecolor=lc, fillcolor=fc)
    draw_2dpolygon(progresspoints, fillcolor=lc, distance=0.2)
    draw_text(text, position=(xend+20, region.height-25), size=9)
    if progressinfo:
        draw_text(progressinfo, position=(xstart + 10, region.height - 42), size=6)


def draw_joint(joint, length):
    origin = Vector(joint.matrix_world.to_translation())
    axis = joint.matrix_world * (length * joint.data.bones[0].vector.normalized())
    endpoint = axis

    bgl.glColor4f(0.0, 1.0, 0.0, 0.5)
    bgl.glLineWidth(2)
    bgl.glBegin(bgl.GL_LINE_STRIP)
    bgl.glVertex3f(*origin)
    bgl.glVertex3f(*endpoint)
    bgl.glEnd()


def draw_path(path, color=colors['white'], dim3=False):
    origins = []
    for e in range(len(path)):
        origins.append(path[e].matrix_world.to_translation())

    bgl.glEnable(bgl.GL_BLEND)
    bgl.glLineWidth(4)

    bgl.glBegin(bgl.GL_LINE_STRIP)
    bgl.glColor4f(*color)
    for o in origins:
        if dim3:
            bgl.glVertex3f(o)
        else:
            bgl.glVertex2f(*to2d(o))
    bgl.glEnd()
    bgl.glDisable(bgl.GL_BLEND)


def draw_callback_3d(self, context):
    """Callback function for 3d drawing.
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
    bgl.glColor4f(0.0, 0.0, 0.0, 1.0)


def draw_callback_2d(self, context):
    """Callback function for 2d drawing.
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
        groupedobjects = [o for o in objects if hasattr(o, 'users_group')]
        # draw spanning tree
        submechanism_groups = set([group for obj in groupedobjects for group in obj.users_group
                                   if group.name.startswith('submechanism:')])
        for group in submechanism_groups:
            for joint in group.objects:
                if 'submechanism/spanningtree' in joint:
                    draw_path(joint['submechanism/spanningtree'], color=colors['submechanism'])
                                      #joint['submechanism/independent'],
                                      #joint['submechanism/active'])
        # draw labels
        for obj in [obj for obj in objects if obj.phobostype == 'link']:
            if 'submechanism/jointname' in obj:
                jointname = obj['submechanism/jointname']
                origin = to2d(obj.matrix_world.translation) + Vector((16, 0))
                draw_textbox(jointname, origin, textsize=8,
                             textcolor=colors['submechanism'])

    # interfaces
    if selected:
        for interface in [obj for obj in selected if obj.phobostype == 'interface']:
            color = interface.active_material.diffuse_color
            maxc = max(color)
            color = [0.6 + c/maxc * 0.4 for c in color]
            bgcolor = [c * 0.4 for c in interface.active_material.diffuse_color]
            draw_textbox(nUtils.getObjectName(interface), to2d(interface.matrix_world.translation),
                         textsize=6, textcolor=(*color, 1.0 if interface.show_name else 0.4),
                         backgroundcolor=(*bgcolor, 1.0) if interface.show_name else colors['background'],
                         linewidth=3 if interface.show_name else 2)

    # progress bar
    if wm.draw_progress and context.window_manager.progress not in [0, 1]:
        draw_progressbar(wm.progress)

    # log messages
    if wm.draw_messages:
        for m in range(wm.phobos_msg_count):
            opacity = 1.0
            if 1 >= m <= wm.phobos_msg_offset-1 or m >= wm.phobos_msg_count-2:
                opacity = 0.5
            if wm.phobos_msg_offset > 1 > m or m >= wm.phobos_msg_count-1:
                opacity = 0.1
            try:
                msg = messages[m+wm.phobos_msg_offset]
                draw_message(msg['text'], msg['type'], m, opacity, offset=wm.phobos_msg_offset)
            except IndexError:
                pass

    # restore opengl defaults
    bgl.glLineWidth(1)
    bgl.glDisable(bgl.GL_BLEND)
    bgl.glColor4f(0.0, 0.0, 0.0, 1.0)


class DrawInfosOperator(bpy.types.Operator):
    """Draw additional information about Phobos objects"""
    bl_idname = "phobos.draw_infos_operator"
    bl_label = "Draw Additional Phobos Information"

    def modal(self, context, event):
        context.area.tag_redraw()

        if event.type == 'PAGE_UP' and event.value == 'PRESS':
            context.window_manager.phobos_msg_offset += 1
        if event.type == 'PAGE_DOWN' and event.value == 'PRESS':
            context.window_manager.phobos_msg_offset -= 1
        if event.shift and event.type == 'LEFTMOUSE' and event.value == 'CLICK':
            pass
        if not context.window_manager.draw_phobos_infos:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
            return {'CANCELLED'}

        return {'PASS_THROUGH'}
        # return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        if context.area.type == 'VIEW_3D':
            self._handle = bpy.types.SpaceView3D.draw_handler_add(draw_callback_3d, (self, context),
                                                                  'WINDOW', 'POST_VIEW')
            self._handle = bpy.types.SpaceView3D.draw_handler_add(draw_callback_2d, (self, context),
                                                                  'WINDOW', 'POST_PIXEL')
            context.window_manager.modal_handler_add(self)
            return {'RUNNING_MODAL'}
        else:
            self.report({'WARNING'}, "View3D not found, cannot run " + self.bl_idname)
            return {'CANCELLED'}


def setProgress(value, info=None):
    global progressinfo
    c = bpy.context
    c.window_manager.progress = value
    bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    progressinfo = info
    #
    # for area in c.screen.areas:
    #     if area.type == 'VIEW_3D':
    #         override = {'window': c.window, 'screen': c.screen, 'area': area}
    #         area.tag_redraw()
    #         break


def register():
    bpy.utils.register_class(DrawInfosOperator)


def unregister():
    bpy.utils.unregister_class(DrawInfosOperator)


if __name__ == "__main__":
    register()
