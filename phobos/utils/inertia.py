import numpy
import trimesh
import trimesh.graph

from phobos.utils.transform import angle_between_vectors


def calculateBoxInertia(mass, size):
    """Returns upper diagonal of inertia tensor of a box as tuple.

    Args:
      mass(float): The box' mass.
      size(iterable): The box' size.

    Returns:
      : tuple(6)

    """
    i = mass / 12
    ixx = i * (size[1] ** 2 + size[2] ** 2)
    ixy = 0
    ixz = 0
    iyy = i * (size[0] ** 2 + size[2] ** 2)
    iyz = 0
    izz = i * (size[0] ** 2 + size[1] ** 2)
    return ixx, ixy, ixz, iyy, iyz, izz


def calculateCylinderInertia(mass, r, h):
    """Returns upper diagonal of inertia tensor of a cylinder as tuple.

    Args:
      mass(float): The cylinders mass.
      r(float): The cylinders radius.
      h(float): The cylinders height.

    Returns:
      : tuple(6)

    """
    i = mass / 12 * (3 * r ** 2 + h ** 2)
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = 0.5 * mass * r ** 2
    return ixx, ixy, ixz, iyy, iyz, izz


def calculateSphereInertia(mass, r):
    """Returns upper diagonal of inertia tensor of a sphere as tuple.

    Args:
      mass(float): The spheres mass.
      r(float): The spheres radius.

    Returns:
      : tuple(6)

    """
    i = 0.4 * mass * r ** 2
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = i
    return ixx, ixy, ixz, iyy, iyz, izz


def calculateEllipsoidInertia(mass, size):
    """Returns upper diagonal of inertia tensor of an ellipsoid as tuple.

    Args:
      mass(float): The ellipsoids mass.
      size: The ellipsoids size.

    Returns:
      : tuple(6)

    """
    i = mass / 5
    ixx = i * (size[1] ** 2 + size[2] ** 2)
    ixy = 0
    ixz = 0
    iyy = i * (size[0] ** 2 + size[2] ** 2)
    iyz = 0
    izz = i * (size[0] ** 2 + size[1] ** 2)
    return ixx, ixy, ixz, iyy, iyz, izz


def calculateMeshInertia(mass, data, scale=None):
    """Calculates and returns the inertia tensor of arbitrary mesh objects.

    Implemented after the general idea of 'Finding the Inertia Tensor of a 3D Solid Body,
    Simply and Quickly' (2004) by Jonathan Blow (1) with formulas for tetrahedron inertia
    from 'Explicit Exact Formulas for the 3-D Tetrahedron Inertia Tensor in Terms of its
    Vertex Coordinates' (2004) by F. Tonon. (2). The latter has an issue, according the
    element order of the inertia tensor: b' and c' are exchanged. According to 'Technische
    Mechanik 3 - Dynamik' (2012) by Russel C. Hibbeler this has been fixed.

    Links: (1) http://number-none.com/blow/inertia/body_i.html
           (2) http://docsdrive.com/pdfs/sciencepublications/jmssp/2005/8-11.pdf
           (3) https://elibrary.pearson.de/book/99.150005/9783863265151
    Args:
      data(bpy.types.BlendData): mesh data of the object
      mass(float): mass of the object
      scale(list): scale vector
    Returns:
      6: inertia tensor
    """
    if scale is None:
        scale = [1.0, 1.0, 1.0]
    scale = numpy.asarray(scale)
    tetrahedra = []
    mesh_volume = 0
    origin = numpy.array((0.0, 0.0, 0.0))
    vertices = None
    faces = None
    triangle_normals = None

    if not isinstance(data, trimesh.Trimesh):
        try:
            import bpy
            vertices = numpy.asarray([numpy.asarray(scale * v.co) for v in data.vertices])
            prev_mode = bpy.context.mode
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.quads_convert_to_tris(quad_method='FIXED')
            bpy.ops.object.mode_set(mode=prev_mode)
            faces = [[v for v in p.vertices] for p in data.polygons]
            triangle_normals = numpy.asarray([t.normal for t in data.polygons])
        except ImportError:
            raise TypeError("Invalid mesh type " + repr(type(data)))

    if vertices is None:
        vertices = numpy.asarray([numpy.asarray(scale * v) for v in data.vertices])
    if faces is None:
        faces = data.faces
    if triangle_normals is None:
        triangle_normals = data.face_normals

    quads = []
    triangles = []
    for face in faces:
        if len(face) == 4:
            tris = trimesh.geometry.triangulate_quads(quads)
            triangles += tris
        else:
            triangles += [face]

    com = numpy.mean(vertices, axis=0)
    vertices = vertices - com[numpy.newaxis]

    mesh_volume = 0.0
    signs = []
    abs_det_Js = []
    Js = []

    for triangle_idx in range(len(triangles)):
        verts = vertices[triangles[triangle_idx]]
        triangle_normal = triangle_normals[triangle_idx]
        triangle_center = numpy.mean(verts, axis=0)
        normal_angle = angle_between_vectors(triangle_center, triangle_normal)
        sign = -1.0 if normal_angle > numpy.pi / 2.0 else 1.0

        J = numpy.array([
            [verts[0, 0], verts[0, 1], verts[0, 2], 1.0],
            [verts[1, 0], verts[1, 1], verts[1, 2], 1.0],
            [verts[2, 0], verts[2, 1], verts[2, 2], 1.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        abs_det_J = numpy.linalg.det(J)

        signs.append(sign)
        abs_det_Js.append(abs_det_J)
        Js.append(J)

        volume = sign * abs_det_J / 6.0
        mesh_volume += volume

    signs = numpy.array(signs)
    abs_det_Js = numpy.array(abs_det_Js)
    Js = numpy.array(Js)

    density = mass / mesh_volume

    A = (signs
         * density
         * abs_det_Js
         * (Js[:, 0, 1] ** 2
            + Js[:, 0, 1] * Js[:, 1, 1]
            + Js[:, 1, 1] ** 2
            + Js[:, 0, 1] * Js[:, 2, 1]
            + Js[:, 1, 1] * Js[:, 2, 1]
            + Js[:, 2, 1] ** 2
            + Js[:, 0, 1] * Js[:, 3, 1]
            + Js[:, 1, 1] * Js[:, 3, 1]
            + Js[:, 2, 1] * Js[:, 3, 1]
            + Js[:, 3, 1] ** 2
            + Js[:, 0, 2] ** 2
            + Js[:, 0, 2] * Js[:, 1, 2]
            + Js[:, 1, 2] ** 2
            + Js[:, 0, 2] * Js[:, 2, 2]
            + Js[:, 1, 2] * Js[:, 2, 2]
            + Js[:, 2, 2] ** 2
            + Js[:, 0, 2] * Js[:, 3, 2]
            + Js[:, 1, 2] * Js[:, 3, 2]
            + Js[:, 2, 2] * Js[:, 3, 2]
            + Js[:, 3, 2] ** 2
            )
         / 60
         )

    B = (signs
         * density
         * abs_det_Js
         * (
                 Js[:, 0, 0] ** 2
                 + Js[:, 0, 0] * Js[:, 1, 0]
                 + Js[:, 1, 0] ** 2
                 + Js[:, 0, 0] * Js[:, 2, 0]
                 + Js[:, 1, 0] * Js[:, 2, 0]
                 + Js[:, 2, 0] ** 2
                 + Js[:, 0, 0] * Js[:, 3, 0]
                 + Js[:, 1, 0] * Js[:, 3, 0]
                 + Js[:, 2, 0] * Js[:, 3, 0]
                 + Js[:, 3, 0] ** 2
                 + Js[:, 0, 2] ** 2
                 + Js[:, 0, 2] * Js[:, 1, 2]
                 + Js[:, 1, 2] ** 2
                 + Js[:, 0, 2] * Js[:, 2, 2]
                 + Js[:, 1, 2] * Js[:, 2, 2]
                 + Js[:, 2, 2] ** 2
                 + Js[:, 0, 2] * Js[:, 3, 2]
                 + Js[:, 1, 2] * Js[:, 3, 2]
                 + Js[:, 2, 2] * Js[:, 3, 2]
                 + Js[:, 3, 2] ** 2
         )
         / 60
         )

    C = (signs
         * density
         * abs_det_Js
         * (
                 Js[:, 0, 0] ** 2
                 + Js[:, 0, 0] * Js[:, 1, 0]
                 + Js[:, 1, 0] ** 2
                 + Js[:, 0, 0] * Js[:, 2, 0]
                 + Js[:, 1, 0] * Js[:, 2, 0]
                 + Js[:, 2, 0] ** 2
                 + Js[:, 0, 0] * Js[:, 3, 0]
                 + Js[:, 1, 0] * Js[:, 3, 0]
                 + Js[:, 2, 0] * Js[:, 3, 0]
                 + Js[:, 3, 0] ** 2
                 + Js[:, 0, 1] ** 2
                 + Js[:, 0, 1] * Js[:, 1, 1]
                 + Js[:, 1, 1] ** 2
                 + Js[:, 0, 1] * Js[:, 2, 1]
                 + Js[:, 1, 1] * Js[:, 2, 1]
                 + Js[:, 2, 1] ** 2
                 + Js[:, 0, 1] * Js[:, 3, 1]
                 + Js[:, 1, 1] * Js[:, 3, 1]
                 + Js[:, 2, 1] * Js[:, 3, 1]
                 + Js[:, 3, 1] ** 2
         )
         / 60
         )

    A_bar = (signs
             * density
             * abs_det_Js
             * (2 * Js[:, 0, 1] * Js[:, 0, 2]
                + Js[:, 1, 1] * Js[:, 0, 2]
                + Js[:, 2, 1] * Js[:, 0, 2]
                + Js[:, 3, 1] * Js[:, 0, 2]
                + Js[:, 0, 1] * Js[:, 1, 2]
                + 2 * Js[:, 1, 1] * Js[:, 1, 2]
                + Js[:, 2, 1] * Js[:, 1, 2]
                + Js[:, 3, 1] * Js[:, 1, 2]
                + Js[:, 0, 1] * Js[:, 2, 2]
                + Js[:, 1, 1] * Js[:, 2, 2]
                + 2 * Js[:, 2, 1] * Js[:, 2, 2]
                + Js[:, 3, 1] * Js[:, 2, 2]
                + Js[:, 0, 1] * Js[:, 3, 2]
                + Js[:, 1, 1] * Js[:, 3, 2]
                + Js[:, 2, 1] * Js[:, 3, 2]
                + 2 * Js[:, 3, 1] * Js[:, 3, 2]
                )
             / 120
             )

    B_bar = (signs
             * density
             * abs_det_Js
             * (2 * Js[:, 0, 0] * Js[:, 0, 2]
                + Js[:, 1, 0] * Js[:, 0, 2]
                + Js[:, 2, 0] * Js[:, 0, 2]
                + Js[:, 3, 0] * Js[:, 0, 2]
                + Js[:, 0, 0] * Js[:, 1, 2]
                + 2 * Js[:, 1, 0] * Js[:, 1, 2]
                + Js[:, 2, 0] * Js[:, 1, 2]
                + Js[:, 3, 0] * Js[:, 1, 2]
                + Js[:, 0, 0] * Js[:, 2, 2]
                + Js[:, 1, 0] * Js[:, 2, 2]
                + 2 * Js[:, 2, 0] * Js[:, 2, 2]
                + Js[:, 3, 0] * Js[:, 2, 2]
                + Js[:, 0, 0] * Js[:, 3, 2]
                + Js[:, 1, 0] * Js[:, 3, 2]
                + Js[:, 2, 0] * Js[:, 3, 2]
                + 2 * Js[:, 3, 0] * Js[:, 3, 2]
                )
             / 120
             )

    C_bar = (signs
             * density
             * abs_det_Js
             * (2 * Js[:, 0, 0] * Js[:, 0, 1]
                + Js[:, 1, 0] * Js[:, 0, 1]
                + Js[:, 2, 0] * Js[:, 0, 1]
                + Js[:, 3, 0] * Js[:, 0, 1]
                + Js[:, 0, 0] * Js[:, 1, 1]
                + 2 * Js[:, 1, 0] * Js[:, 1, 1]
                + Js[:, 2, 0] * Js[:, 1, 1]
                + Js[:, 3, 0] * Js[:, 1, 1]
                + Js[:, 0, 0] * Js[:, 2, 1]
                + Js[:, 1, 0] * Js[:, 2, 1]
                + 2 * Js[:, 2, 0] * Js[:, 2, 1]
                + Js[:, 3, 0] * Js[:, 2, 1]
                + Js[:, 0, 0] * Js[:, 3, 1]
                + Js[:, 1, 0] * Js[:, 3, 1]
                + Js[:, 2, 0] * Js[:, 3, 1]
                + 2 * Js[:, 3, 0] * Js[:, 3, 1]
                )
             / 120
             )

    a_bar = -numpy.sum(A_bar)
    b_bar = -numpy.sum(B_bar)
    c_bar = -numpy.sum(C_bar)

    I = numpy.array([[numpy.sum(A), c_bar, b_bar],
                     [c_bar, numpy.sum(B), a_bar],
                     [b_bar, a_bar, numpy.sum(C)]])
    return I[0][0], I[0][1], I[0][2], I[1][1], I[1][2], I[2][2]