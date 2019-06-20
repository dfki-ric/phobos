#!/usr/bin/python3

import bpy
import phobos
import unittest

class TestInertiaModel(unittest.TestCase):

    def test_calculateBoxInertia(self):
        mass = 12
        size = [1, 2, 3]
        result = [13, 0, 0, 10, 0, 5]

        self.assertEqual(phobos.model.inertia.calculateBoxInertia(mass, size), result)

    def test_calculateCylinderInertia(self):
        mass = 12
        radius = 2
        height = 5
        result = [37, 0, 0, 37, 0, 24]

        self.assertEqual(phobos.model.inertia.calculateCylinderInertia(mass, radius, height), result)

    def test_calculateSphereInertia(self):
        mass = 12
        radius = 2
        result = [19.2, 0, 0, 19.2, 0, 19.2]

        self.assertEqual(phobos.model.inertia.calculateSphereInertia(mass, radius), result)

    def test_calculateEllipsoidInertia(self):
        mass = 5
        size = [1, 2, 3]
        result = [13, 0, 0, 10, 0, 5]

        self.assertEqual(phobos.model.inertia.calculateEllipsoidInertia(mass, size), result)

    def test_inertiaListToMatrix(self):
        inertialist = [1, 2, 3, 4, 5, 6]
        result = [[1, 2, 3], [2, 4, 5], [3, 5, 6]]

        self.assertEqual(phobos.model.inertia.inertiaListToMatrix(inertialist), result)

    def test_inertiaMatriToList(self):
        matrix = [[1, 2, 3], [2, 4, 5], [3, 5, 6]]
        result = [1, 2, 3, 4, 5, 6]

        self.assertEqual(phobos.model.inertia.inertiaMatrixToList(matrix), result)

        # TODO continue with joints
