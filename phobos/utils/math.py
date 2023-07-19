import math
import numpy as np


def _calculate_normal_vector(points):
    """Returns Normal-Vector for a Plane defined by 3 Points"""
    v1 = [points[1][r] - points[0][r] for r in [0, 1, 2]]
    v2 = [points[2][r] - points[0][r] for r in [0, 1, 2]]

    n = [v1[1] * v2[2] - v1[2] * v2[1],
         v1[2] * v2[0] - v1[0] * v2[2],
         v1[0] * v2[1] - v1[1] * v2[0]]

    return n


def _normalize_vector(vector):
    """Scales a Vector to the length of 1"""
    length = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2) + math.pow(vector[2], 2))
    return [i * (1 / length) for i in vector]


def _calculate_normal_vectors_of_plane(plane):
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
        normal_vectors.append(_normalize_vector(_calculate_normal_vector(cp)))

    return normal_vectors


def _is_equal(a, b):
    """Checks if two floats are equal"""
    result = True
    for i in range(0, 3):
        if not math.isclose(abs(a[i]), abs(b[i])):
            result = False
    return result

def _is_on_edge(point, edge):
    """Checks if point is on edge"""
    for i in range(0, 3):
        v_max = max(edge[0][i], edge[1][i])
        v_min = min(edge[0][i], edge[1][i])
        if not v_max >= point[i] >= v_min:
            return False
    return True


def _intersects_edge_plane(plane, edge, epsilon=1e-6):
    """Checks if edge of Object intersects Cutting-Plane"""
    plane_normal = np.array(_calculate_normal_vector(plane[0:3]))
    plane_point = np.array(plane[0])
    ray_direction = np.array(edge[1] - edge[0])
    ray_point = np.array(edge[0])

    ndotu = plane_normal.dot(ray_direction)
    if abs(ndotu) < epsilon:
        return None

    w = ray_point - plane_point
    si = -plane_normal.dot(w) / ndotu
    psi = w + si * ray_direction + plane_point

    if _is_on_edge(psi, edge):
        return True



def _calculate_mean_of_vectors(list_of_vectors):
    list_of_vectors = np.array(list_of_vectors)
    print(list_of_vectors)
    mean_along_axis = np.mean(list_of_vectors, axis=0)
    return mean_along_axis


def change_direction_of_vectors_according_to_reference_vector(list_of_vectors, ref_vector):
    for i in range(0, len(list_of_vectors)):
        if np.dot(ref_vector, list_of_vectors[i]) <= 0:
            list_of_vectors[i] *= -1
    return list_of_vectors


def get_rotation_matrix_from_two_vectors(v1, v2):
    """Calculates a 4x4-Rotation-Matrix which aligns v1 to v2"""
    a = np.cross(v1, v2)
    c = np.dot(v1, v2)
    k = 1.0 / (1.0 + c)

    rotation_matrix = np.zeros((4,4))
    rotation_matrix[3,3] = 1
    rotation_matrix[0:3, 0:3] = np.array(
        [
            [a[0] * a[0] * k + c, a[1] * a[0] * k - a[2], a[2] * a[0] * k + a[1]],
            [a[0] * a[1] * k + a[2], a[1] * a[1] * k + c, a[2] * a[1] * k - a[0]],
            [a[0] * a[2] * k - a[1], a[1] * a[2] * k + a[0], a[2] * a[2] * k + c]
        ]
    )
    return rotation_matrix


def convert_to_4dim_vector(vector):
    v = np.ones(4)
    v[0:3] = vector
    return v


def inverse_matrix(matrix):
    return np.linalg.inv(matrix)


def convert_to_np_array(array):
    return np.array(array)