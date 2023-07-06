import bpy
import math
import numpy as np
import re

from ..phoboslog import log


class CuttingPlane:
    """Creates a Cutting-Plane-Object which stores the vertices of the Plane,
     the name of the intersecting Link and the intersecting Edges of the Link"""
    plane_vertices = None
    intersecting_edges = None
    name_of_intersecting_link = None
    is_valid = False

    def __init__(self, context, link=None):
        if link:
            self.load_cutting_plane_from_link(link)
        else:
            cutting_plane_obj = context.active_object
            if self.__is_valid_plane(cutting_plane_obj):
                intersecting_link = self.__get_intersecting_link(context)
                if intersecting_link:
                    self.name_of_intersecting_link = intersecting_link
                    self.plane_vertices = [cutting_plane_obj.matrix_world @ v.co
                                                          for v in cutting_plane_obj.data.vertices]
                    self.is_valid = True


    def __calculate_normal_vector(self, points):
        """Returns Normal-Vector for a Plane defined by 3 Points"""
        v1 = [points[1][r] - points[0][r] for r in [0, 1, 2]]
        v2 = [points[2][r] - points[0][r] for r in [0, 1, 2]]

        n = [v1[1] * v2[2] - v1[2] * v2[1],
             v1[2] * v2[0] - v1[0] * v2[2],
             v1[0] * v2[1] - v1[1] * v2[0]]

        return n


    def __normalize_vector(self, vector):
        """Scales a Vector to the length of 1"""
        length = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2) + math.pow(vector[2], 2))
        return [i * (1 / length) for i in vector]


    def __calculate_normal_vectors_of_plane(self, plane):
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
            normal_vectors.append(self.__normalize_vector(self.__calculate_normal_vector(cp)))

        return normal_vectors


    def __is_equal(self, a, b):
        """Checks if two floats are equal"""
        result = True
        for i in range(0, 3):
            if not math.isclose(abs(a[i]), abs(b[i])):
                result = False
        return result


    def __is_valid_plane(self, plane):
        """Checks if a plane defined by 4 points is even"""
        normal_vectors = self.__calculate_normal_vectors_of_plane(plane)
        if len(plane.data.vertices) != 4:
            log("Object is no Plane", 'DEBUG')
            return False
        for i in range(0, 4):
            for j in range(0, 4):
                if not self.__is_equal(normal_vectors[i], normal_vectors[j]):
                    return False

        return True


    def __get_edges_of_object(self, context, object_name):
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


    def __is_on_edge(self, point, edge):
        """Checks if point is on edge"""
        for i in range(0, 3):
            v_max = max(edge[0][i], edge[1][i])
            v_min = min(edge[0][i], edge[1][i])
            if not v_max >= point[i] >= v_min:
                return False
        return True


    def __intersects_edge_plane(self, plane, edge, epsilon=1e-6):
        """Checks if edge of Object intersects Cutting-Plane"""
        plane_normal = np.array(self.__calculate_normal_vector(plane[0:3]))
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

        if self.__is_on_edge(psi, edge):
            return True


    def __check_intersection(self, context, object_name: str):
        """Check Intersection of Object with Cutting-Plane"""
        plane_obj = context.active_object
        plane = [plane_obj.matrix_world @ v.co for v in plane_obj.data.vertices]
        intersecting_edges = []
        for edge in self.__get_edges_of_object(context, object_name):
            if self.__intersects_edge_plane(plane, edge):
                intersecting_edges.append(edge)
        if len(intersecting_edges) > 0:
            self.intersecting_edges = intersecting_edges
            return True



    def __get_intersecting_link(self, context):
        """Gets Link which intersects Cutting-Plane"""
        intersecting_links = []
        for obj in context.scene.objects:
            if obj.name != context.active_object.name:
                log("Checking if {obj} is intersecting Cutting-Plane".format(obj=obj.name), 'DEBUG')
                if self.__check_intersection(context, obj.name):
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


    def __save_plane_to_file(self, plane):
        if self.__is_valid_plane(plane):
            normal_vector = self.__calculate_normal_vectors_of_plane(plane)[0]
            point = plane.matrix_world @ plane.data.vertices[0].co
            point = [i for i in point]
            f = open("cutting_planes.txt", "a")
            if [point, normal_vector] not in self.__load_cutting_plane("cutting_planes.txt"):
                plane_string = "Point:'{point}', Normal-Vector:'{normal_vector}'\n" \
                    .format(id=2, point=point, normal_vector=normal_vector)
                f.write(plane_string)
            else:
                print("Plane already exists")
            f.close()


    def __convert_string_to_list_of_vectors(self, string):
        """Converts the String representation of a list of Floats back"""
        list_of_vectors = []
        vector_strings = re.split("/", string)
        for v in vector_strings:
            coordinate_strings = re.split(",", v)
            vector = []
            for c in coordinate_strings:
                vector.append(float(c))
            list_of_vectors.append(vector)
        print(list_of_vectors)
        return list_of_vectors


    def load_cutting_plane_from_link(self, link=bpy.types.Object):
        cutting_plane_data = link.cutting_plane
        self.plane_vertices = self.__convert_string_to_list_of_vectors(cutting_plane_data.plane_vertices)
        self.intersecting_edges = self.__convert_string_to_list_of_vectors(cutting_plane_data.intersecting_edges)
        self.name_of_intersecting_link = link.name
        self.is_valid = True # TODO Check Validity


    def save_cutting_plane(self, context):
        """Saves the defined Cutting-Plane in the Properties of the intersecting Link"""
        if not self.is_valid:
            log("Can not safe invalid Cutting-Plane")
            return
        plane_vertices_str = ""
        for v in self.plane_vertices:
            for p in v:
                plane_vertices_str += "{p},".format(p=p)
            plane_vertices_str = plane_vertices_str[:-1]
            plane_vertices_str += "/"
        plane_vertices_str = plane_vertices_str[:-1]
        intersecting_edges_str = ""
        for e in self.intersecting_edges:
            for p in e:
                intersecting_edges_str += "{p}".format(p=p)
            intersecting_edges_str = intersecting_edges_str[:-1]
            intersecting_edges_str += "/"
        intersecting_edges_str = intersecting_edges_str[:-1]

        intersecting_link = context.scene.objects[self.name_of_intersecting_link]
        intersecting_link.cutting_plane.plane_vertices = plane_vertices_str
        intersecting_link.cutting_plane.intersecting_edges = intersecting_edges_str
        intersecting_link.cutting_plane.intersecting_link = self.name_of_intersecting_link


    def __calculate_mean_of_vectors(self, list_of_vectors):
        list_of_vectors = np.array(list_of_vectors)
        mean_along_axis = np.mean(list_of_vectors, axis=0)
        return mean_along_axis


    def calculate_skew_symmetric_cross_product_matrix(self, vector):
        return np.array([[0, -vector[2], vector[1]], [vector[2], 0, -vector[0]], [-vector[1], vector[0], 0]])

    def align_orthogonal_to_link(self):
        mean_direction_of_edges = self.__calculate_mean_of_vectors(self.intersecting_edges)
        normal_of_plane = self.__calculate_normal_vector(self.plane_vertices[:3])
        v = np.cross(mean_direction_of_edges, normal_of_plane)
        c = np.dot(mean_direction_of_edges, normal_of_plane)

        v_cross = self.calculate_skew_symmetric_cross_product_matrix(self, v)

        rotation_matrix = np.eye(3) + v_cross + v_cross**2 * (1 / (1 + c))

        self.plane_vertices = rotation_matrix @ self.plane_vertices




class CuttingPlaneData(bpy.types.PropertyGroup):
    """Stores all information of a Cutting-Plane"""
    plane_vertices: bpy.props.StringProperty()
    intersecting_edges: bpy.props.StringProperty()
    intersecting_link: bpy.props.StringProperty()


def register():
    bpy.utils.register_class(CuttingPlaneData)
    bpy.types.Object.cutting_plane = bpy.props.PointerProperty(
        type=CuttingPlaneData)  # TODO Look for another Place for this Statement
