import collections
import bpy
import bgl
import blf
from bpy_extras import view3d_utils
from mathutils import Vector


progressinfo = None
colors = {'debug': (1.0, 0.0, 1.0),
          'info': (0.0, 1.0, 0.0),
          'warning': (0.5, 0.25, 0.25),
          'error': (1.0, 0.0, 0.0),
          'none': (1.0, 1.0, 1.0),
          'submechanism': (1.0, 0.8, 0.1, 1.0),
          'background': (0.1, 0.1, 0.1, 0.8),
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


def draw_submechanism(spanningtree, independent=None, active=None):
    origins = []
    region, rv3d = getRegionData()
    offset = view3d_utils.region_2d_to_origin_3d(region, rv3d, (region.width/2.0,
                                                                region.height/2.0)).normalized()
    for e in range(len(spanningtree)):
        origins.append(Vector(spanningtree[e].matrix_world.to_translation()) + offset)

    bgl.glEnable(bgl.GL_BLEND)
    bgl.glLineWidth(4)

    bgl.glBegin(bgl.GL_LINE_STRIP)
    bgl.glColor4f(*colors['submechanism'])
    for o in origins:
        bgl.glVertex3f(*o)
    bgl.glEnd()
    bgl.glDisable(bgl.GL_BLEND)


def draw_callback_3d(self, context):
    """Callback function for 3d drawing.
    """
    # FIXME: does not work for perspective view, only for ortho
    active = context.object
    selected = context.selected_objects
    wm = context.window_manager

    bgl.glEnable(bgl.GL_BLEND)

    # submechanisms
    if active and wm.draw_submechanisms:
        if any('submechanism' in prop for prop in active.keys()):
            groups = active.users_group
            for group in groups:
                for obj in group.objects:
                    if 'submechanism/spanningtree' in obj:
                        draw_submechanism(obj['submechanism/spanningtree'],
                                          obj['submechanism/independent'],
                                          obj['submechanism/active'])

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
    if selected and wm.draw_submechanisms:
        for obj in [obj for obj in selected if obj.phobostype == 'link']:
            if 'submechanism/jointname' in obj:
                jointname = obj['submechanism/jointname']
                origin = to2d(obj.matrix_world.translation) + Vector((16, 0))
                blf.size(0, 8, 150)
                width = blf.dimensions(0, jointname)[0]
                height = blf.dimensions(0, jointname)[1]
                border = 6
                points = ((origin + Vector((-border, -border * 1.5)),
                           origin + Vector((width + border, -border * 1.5)),
                           origin + Vector((width + border, height + border)),
                           origin + Vector((-border, height + border))))
                draw_2dpolygon(points, fillcolor=colors['background'],
                               linecolor=colors['submechanism'], linewidth=2)
                draw_text(jointname, position=origin, size=8, color=colors['submechanism'])

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
