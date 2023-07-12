import bpy
import math
import numpy as np
import re

from ..phoboslog import log

from ..utils import blender


def __calculate_normal_vector(points):
    """Returns Normal-Vector for a Plane defined by 3 Points"""
    v1 = [points[1][r] - points[0][r] for r in [0, 1, 2]]
    v2 = [points[2][r] - points[0][r] for r in [0, 1, 2]]

    n = [v1[1] * v2[2] - v1[2] * v2[1],
         v1[2] * v2[0] - v1[0] * v2[2],
         v1[0] * v2[1] - v1[1] * v2[0]]

    return n


def __normalize_vector(vector):
    """Scales a Vector to the length of 1"""
    length = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2) + math.pow(vector[2], 2))
    return [i * (1 / length) for i in vector]


def __calculate_normal_vectors_of_plane(plane):
    """Calculates 4 Normal-Vectors for a plane defined by 4 Points"""
    vertices = plane.data.vertices
    vertices_coordinates = []
    for v in vertices:
        co_final = plane.matrix_world @ v.co
        vertices_coordinates.append(co_final)

    normal_vectors = []
    for i in range(0, 4):
        cp = vertices_coordinates.copy()
        cp.remove(cp[i])
        normal_vectors.append(__normalize_vector(__calculate_normal_vector(cp)))

    return normal_vectors


def __is_equal(a, b):
    """Checks if two floats are equal"""
    result = True
    for i in range(0, 3):
        if not math.isclose(abs(a[i]), abs(b[i])):
            result = False
    return result


def is_valid_plane(plane):
    """Checks if a plane defined by 4 points is even"""
    normal_vectors = __calculate_normal_vectors_of_plane(plane)
    if len(plane.data.vertices) != 4:
        log("Object is no Plane", 'DEBUG')
        return False
    for i in range(0, 4):
        for j in range(0, 4):
            if not __is_equal(normal_vectors[i], normal_vectors[j]):
                return False

    return True


def __get_edges_of_object(context, object_name):
    """Returns the edges of an object in world coordinates"""
    obj = context.scene.objects[object_name]
    edges_of_obj = []
    if obj.type == "MESH":
        for e in obj.data.edges:
            edge = []
            for i in e.vertices:
                edge.append(obj.matrix_world @ obj.data.vertices[i].co)
            edges_of_obj.append(edge)
        return edges_of_obj
    else:
        log("Object does not have edges. Choose a valid Object with type: MESH", 'DEBUG')
        return []


def __is_on_edge(point, edge):
    """Checks if point is on edge"""
    for i in range(0, 3):
        v_max = max(edge[0][i], edge[1][i])
        v_min = min(edge[0][i], edge[1][i])
        if not v_max >= point[i] >= v_min:
            return False
    return True


def __intersects_edge_plane(plane, edge, epsilon=1e-6):
    """Checks if edge of Object intersects Cutting-Plane"""
    plane_normal = np.array(__calculate_normal_vector(plane[0:3]))
    plane_point = np.array(plane[0])
    ray_direction = np.array(edge[1] - edge[0])
    ray_point = np.array(edge[0])

    ndotu = plane_normal.dot(ray_direction)
    if abs(ndotu) < epsilon:
        log("The edge is parallel to the plane or in the plane", 'DEBUG')
        return None

    w = ray_point - plane_point
    si = -plane_normal.dot(w) / ndotu
    psi = w + si * ray_direction + plane_point

    if __is_on_edge(psi, edge):
        return True


def __check_intersection(context, object_name: str):
    """Check Intersection of Object with Cutting-Plane"""
    plane_obj = context.active_object
    plane = [plane_obj.matrix_world @ v.co for v in plane_obj.data.vertices]
    intersecting_edges = []
    for edge in __get_edges_of_object(context, object_name):
        if __intersects_edge_plane(plane, edge):
            intersecting_edges.append(edge)
    if len(intersecting_edges) > 0:
        return True



def get_intersecting_link(context):
    """Gets Link which intersects Cutting-Plane"""
    intersecting_links = []
    for obj in context.scene.objects:
        if obj.name != context.active_object.name:
            log("Checking if {obj} is intersecting Cutting-Plane".format(obj=obj.name), 'DEBUG')
            if __check_intersection(context, obj.name):
                intersecting_links.append(obj.name)
                log("{obj} is intersecting Cutting Plane".format(obj=obj.name), 'DEBUG')
    if len(intersecting_links) > 1:
        s = "The Plane intersects more than one Link. It intersects: "
        for l in intersecting_links:
            s += "{link}, ".format(link=l)
        s = s[:-2]
        s += ". But only one intersecting link is allowed!"
        log(s, 'DEBUG')
        return None
    if len(intersecting_links) == 0:
        log("The Plane does not intersect a Link. But is has to intersect one", 'DEBUG')
        return None

    return intersecting_links[0]


def __calculate_mean_of_vectors(list_of_vectors):
    list_of_vectors = np.array(list_of_vectors)
    print(list_of_vectors)
    mean_along_axis = np.mean(list_of_vectors, axis=0)
    return mean_along_axis


def __calculate_skew_symmetric_cross_product_matrix(vector):
    return np.array([[0, -vector[2], vector[1]], [vector[2], 0, -vector[0]], [-vector[1], vector[0], 0]])


"""def align_orthogonal_to_link():
    edges = [v[1] - v[0] for v in self.intersecting_edges] #Intersecting edges are saved as two corresponding vertices
    normal_of_plane = __normalize_vector(__calculate_normal_vector(self.plane_vertices[:3]))
    for i in range(0, len(edges)):
        if np.dot(normal_of_plane, edges[i]) <= 0:
            edges[i] *= -1
    mean_direction_of_edges = __normalize_vector(__calculate_mean_of_vectors(edges))
    print(mean_direction_of_edges, normal_of_plane)
    v = np.cross(mean_direction_of_edges, normal_of_plane)
    c = np.dot(mean_direction_of_edges, normal_of_plane)

    print("v:", v, "c:", c)

    v_cross = __calculate_skew_symmetric_cross_product_matrix(v)

    print("v_cross:", v_cross)

    rotation_matrix = np.eye(3) + v_cross + v_cross**2 * (1 / (1 + c))
    print("rotation_matrix:", rotation_matrix)
    print(self.plane_vertices)
    for v in range(0, len(self.plane_vertices)):
        self.plane_vertices[v] = rotation_matrix @ self.plane_vertices[v]
    print(self.plane_vertices)"""
