import bpy
import bgl
import blf
from bpy_extras import view3d_utils
from mathutils import Vector


progressinfo = None


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


def draw_2dpolgyon(points, linecolor=None, fillcolor=None, distance=0.2):
    # background
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


def draw_text(text, position, color=(1, 1, 1, 1), size=14, dpi=150, font_id=0):
    bgl.glColor4f(*color)
    blf.position(font_id, *position, 1.0)
    blf.size(font_id, size, dpi)
    blf.draw(font_id, text)


def draw_progressbar(value):
    region = bpy.context.region
    text = str(round(value*100))+'%'
    lc = (0.0, 1.0, 0.0, 1.0)
    fc = (0.0, 1.0, 0.0, 0.3)
    min = region.width*0.2
    max = region.width*0.8
    span = max-min
    points = ((min, region.height-10),
              (max, region.height-10),
              (max, region.height-25),
              (min, region.height-25))
    progresspoints = ((min, region.height-10),
                      (min+value*span, region.height-10),
                      (min+value*span, region.height-25),
                      (min, region.height-25))
    framepoints = ((0, 1), (2, 1), (2, 2), (0, 2))
    draw_2dpolgyon(points, linecolor=lc, fillcolor=fc)
    draw_2dpolgyon(progresspoints, fillcolor=lc, distance=0.2)
    draw_text(text, position=(max+20, region.height-25), size=9)
    if progressinfo:
        draw_text(progressinfo, position=(min + 10, region.height - 42), size=6)


def draw_joint(joint):
    bgl.glColor4f(0.0, 1.0, 0.0, 0.5)
    bgl.glLineWidth(2)

    origin = Vector(joint.matrix_world.to_translation())
    axis = joint.matrix_world * joint.data.bones[0].vector.normalized()
    endpoint = axis

    bgl.glBegin(bgl.GL_LINE_STRIP)
    bgl.glVertex3f(*origin)
    bgl.glVertex3f(*endpoint)
    bgl.glEnd()


def draw_submechanism(elements):
    linecolor = (0.0, 1.0, 0.0, 0.8)
    areacolor = (0.0, 1.0, 0.0, 0.5)

    bgl.glEnable(bgl.GL_BLEND)
    bgl.glColor4f(*linecolor)
    bgl.glLineWidth(5)

    vecs = []
    # offset = Vector((-sum(elements[0].dimensions), 0.0, 0.0))
    region, rv3d = getRegionData()
    offset = view3d_utils.region_2d_to_origin_3d(region, rv3d, (region.width/2.0, region.height/2.0)).normalized()
    vecs.append(Vector(elements[0].matrix_world.to_translation()) + offset)

    for e in range(len(elements)):
        vecs.append(Vector(elements[e].matrix_world.to_translation()) + offset)

    bgl.glBegin(bgl.GL_LINE_STRIP)
    for v in vecs:
        bgl.glVertex3f(*v)
    bgl.glVertex3f(*vecs[0])
    bgl.glEnd()

    bgl.glBegin(bgl.GL_POLYGON);
    bgl.glColor4f(*areacolor);
    for v in vecs:
        bgl.glVertex3f(*v)
    bgl.glEnd();


def draw_callback_px(self, context):
    # define common variables
    region, rv3d = getRegionData()
    active = context.object
    selected = context.selected_objects

    # ----- 3D Drawing -----
    bgl.glEnable(bgl.GL_BLEND)

    if active:
        if 'submechanism/spanningtree' in context.object:
            draw_submechanism(context.object['submechanism/spanningtree'])

    if len(selected) > 0:
        for j in [o for o in selected if o.phobostype == 'link']:
            draw_joint(j)

    # restore opengl defaults
    bgl.glLineWidth(1)
    bgl.glDisable(bgl.GL_BLEND)
    bgl.glColor4f(0.0, 0.0, 0.0, 1.0)

    # ----- 2D Drawing -----
    bgl.glMatrixMode(bgl.GL_PROJECTION)
    bgl.glLoadIdentity()
    bgl.gluOrtho2D(0, bpy.context.region.width, 0, bpy.context.region.height)
    bgl.glMatrixMode(bgl.GL_MODELVIEW)
    bgl.glLoadIdentity()
    bgl.glEnable(bgl.GL_BLEND)

    if context.window_manager.progress not in [0, 1]:
        draw_progressbar(context.window_manager.progress)

    bgl.glDisable(bgl.GL_BLEND)


class DrawInfosOperator(bpy.types.Operator):
    """Draw additional information about Phobos objects"""
    bl_idname = "phobos.draw_infos_operator"
    bl_label = "Draw Additional Phobos Information"

    def modal(self, context, event):
        #TODO: check this
        try:
            context.area.tag_redraw()
        except AttributeError:
            pass

        if not context.window_manager.draw_phobos_infos:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
            return {'CANCELLED'}

        return {'PASS_THROUGH'}

    def invoke(self, context, event):
        if context.area.type == 'VIEW_3D':
            self._handle = bpy.types.SpaceView3D.draw_handler_add(draw_callback_px,
                                                                  (self, context), 'WINDOW', 'POST_VIEW')
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
