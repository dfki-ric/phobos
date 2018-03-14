import bpy
import bgl
import blf
from bpy_extras import view3d_utils
from mathutils import Vector


def start_draw_operator(self, context):
    if self.draw_phobos_infos:
        bpy.ops.phobos.draw_infos_operator('INVOKE_DEFAULT')
    return None


def draw_polgyon(points, linecolor=None, fillcolor=None):
    # background
    bgl.glColor4f(*fillcolor)
    bgl.glBegin(bgl.GL_POLYGON)
    for p in points:
        bgl.glVertex2i(*p)
    bgl.glEnd()
    # frame
    bgl.glColor4f(*linecolor)
    bgl.glBegin(bgl.GL_LINE_STRIP)
    for p in points:
        bgl.glVertex2i(*p)
    bgl.glVertex2i(*(points[0]))
    bgl.glEnd()


def draw_text(text, position, color=(1, 1, 1, 1), size=16, dpi=72, font_id=0):
    bgl.glColor4f(*color)
    blf.position(font_id, *position)
    blf.size(font_id, size, dpi)
    blf.draw(font_id, text)


def draw_progressbar(value, region):
    text = str(round(value*100))+'%'
    lc = bgl.glColor4f(0.0, 1.0, 0.0, 1.0)
    fc = bgl.glColor4f(0.0, 1.0, 0.0, 0.3)
    points = ((100, region.height - 40),
              (region.width - 100, region.height - 40),
              (region.width - 100, region.height - 20),
              (100, region.height - 20))
    draw_polgyon(points, linecolor=lc, fillcolor=fc)
    draw_polgyon(points, fillcolor=lc)
    draw_text(text, (region.width - 60, region.height - 35, 0))


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


def draw_submechanism(elements, viewvec):
    linecolor = (0.0, 1.0, 0.0, 0.8)
    areacolor = (0.0, 1.0, 0.0, 0.5)

    bgl.glEnable(bgl.GL_BLEND)
    bgl.glColor4f(*linecolor)
    bgl.glLineWidth(5)

    vecs = []
    # offset = Vector((-sum(elements[0].dimensions), 0.0, 0.0))
    offset = viewvec
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
    region = context.region
    rv3d = context.region_data
    viewvec = view3d_utils.region_2d_to_origin_3d(region, rv3d, (region.width / 2.0, region.height / 2.0)).normalized()
    active = context.object
    selected = context.selected_objects

    # setup OpenGl
    bgl.glEnable(bgl.GL_BLEND)

    if context.window_manager.progress != 0:
        draw_progressbar(context.window_manager.progress, region)

    if 'submechanism/spanningtree' in context.object:
        draw_submechanism(context.object['submechanism/spanningtree'], viewvec)

    for j in [o for o in selected if o.phobostype == 'link']:
        draw_joint(j)

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


def setProgress(value):
    bpy.context.window_manager.progress = value


def register():
    bpy.utils.register_class(DrawInfosOperator)


def unregister():
    bpy.utils.unregister_class(DrawInfosOperator)


if __name__ == "__main__":
    register()
